// powered_descent.ks — the powered descent, reduced to its invariants.
//
// This file assumes the plan is good: it checks feasibility once at PDI —
// declining to ignite on a stable ellipse is the abort — and carries no
// other envelope protection.
// Design and the full argument: notes/klumpp-descent-redesign.md.
//
// Assumes (plan_doi.ks's contract): the DOI burn is behind us, PDI is the
// periapsis of the ellipse we are on, the planner sized the ground lead so
// the guidance profile's peak thrust demand sits at its reserved throttle,
// and the orbital plane passes near the site.
//
// Five ideas, one per section:
//   1. One vector law flies braking: commanded acceleration from position
//      and velocity errors against the gate state, closed form, no
//      integration — a_cmd = 6*dR/t_go^2 - 2*dV/t_go carries both errors
//      to zero as t_go runs out (Klumpp's guidance, Apollo P63's law).
//   2. t_go is chosen once at ignition where the profile's two endpoint
//      thrust demands are equal — the minimax, the cheapest profile the
//      bracket holds — and decrements by clock.
//   3. The law aims at a virtual gate: the real gate state propagated a
//      floor time tau_f forward along the profile, position and velocity
//      both, so braking exits at t_go = tau_f occupying the real gate
//      state and the 1/t_go^2 divergence never enters.
//   4. High gate opens the vertical corridor: total speed small enough
//      that an f_max burn can rest the craft above the pad. Inside it at
//      the gate, the arrest schedule always fires above the pad.
//   5. Terminal is the flown chain kept whole: FALL — engine off,
//      retrograde hold, drift already nulled — then the arrest burn from
//      the schedule, plumb below walking speed, settle.

@lazyglobal off.

clearscreen.
print "=== POWERED DESCENT ===".

run "common".              // engine_isp
run "../core/kepler".      // geoposition_at, wrap_longitude; bisect rides along

parameter target_lat is 0.
parameter target_lng is 0.
parameter f_max is 0.85.
parameter plan_pe_lng is 999.  // planner's wanted periapsis longitude, deg;
                               // 999 = not supplied, witness logs delivered only

local tgt is body:geopositionlatlng(target_lat, target_lng).
local ipu_prior is config:ipu.
// Raised for the ignition solve, kept for the flight: the solve's bisection
// costs about a second of game time at the default rate, and a second at
// PDI is ~560 m of along-track — a third of the delivery window spent on
// arithmetic. At 2000 the solve fits in a tenth of a second.
set config:ipu to 2000.

// Descent geometry. g0 is surface gravity; h_pad the flare height; v_floor
// the touchdown descent rate; v_switch the speed below which the surface
// retrograde direction is mostly noise and the nose goes to plumb — a
// small tolerance, not a landing number.
local g0 is body:mu / body:radius ^ 2.
local h_pad is 5.
local v_floor is 2.
local v_switch is 5.

// h_gate: high gate's height above the site's terrain — the design's one
// terrain-clearance input. Sized to contain the arrest with a factor
// k_gate ~ 2 in hand: low gate falls near h_gate/2 (the corridor's
// mid-target entered at v_gate makes h_lg = h_pad +
// (v_gate^2 + 2*g0*(h_gate - h_pad)) / (2*(a_dec + g0)) ~ h_gate/2), so
// the schedule fires with as much altitude below it as the burn needs.
// Craft- and body-free as a ratio; the flown anchor is the fixbatch
// flight's 283 m gate arresting at radar 127.
local h_gate is 300.

// tau_f: the t_go floor and the virtual gate's propagation time, seconds.
// Family of a settle time — what a command reversal takes at steering
// rates — and underived (register: Open).
local tau_f is 3.

// The t_go bracket, in units of X/v0 (ground distance over ground speed).
// It straddles the demand curve's dip (~2.0*X/v0) and excludes the
// short-burn wall below and the hover branch beyond 3*X/v0. The dip
// structure is established numerically for this geometry, not proved, so
// the bracket is load-bearing (register: Open).
local tau_lo_frac is 1.6.
local tau_hi_frac is 2.6.

// Gravity as a vector at the ship: body:position points from ship to the
// body's center, so this is the downward pull, live each read.
function g_vec {
  return body:position:normalized * (body:mu / body:position:mag ^ 2).
}

// Great-circle ground distance to the site.
function dist_to_site {
  return body:radius * constant:degtorad
       * vang(ship:position - body:position, tgt:position - body:position).
}

// The guidance profile's two endpoint accelerations for boundary
// conditions (r0 -> r_tgt, v0 -> v_tgt) flown in time tau: the commanded
// acceleration is linear in time, so these two points carry the whole
// profile. dR is the position miss a pure coast would book at tau; dV the
// velocity still to be removed.
//   a0 =  6*dR/tau^2 - 2*dV/tau     (ignition end)
//   a1 = -6*dR/tau^2 + 4*dV/tau     (gate end)
function profile_ends {
  parameter r0.      // ship-relative vector to the gate
  parameter v0_.     // surface velocity
  parameter v_tgt.   // gate-state velocity
  parameter tau.
  local d_r is r0 - v0_ * tau.
  local d_v is v_tgt - v0_.
  return list(d_r * (6 / tau ^ 2) - d_v * (2 / tau),
              d_r * (-6 / tau ^ 2) + d_v * (4 / tau)).
}

// Thrust demand at both profile endpoints, as fractions of available
// thrust: |a - g| is what the engine must supply, and each endpoint is
// priced at its own mass — the craft is ~20 % lighter at the gate, and
// that difference is what decides whether the gate-end peak fits.
function demand_pair {
  parameter r0, v0_, v_tgt, tau, m0, m_gate, g_gate.
  local ends is profile_ends(r0, v0_, v_tgt, tau).
  return list(
      (ends[0] - g_vec()):mag * m0 / ship:availablethrust,
      (ends[1] - g_gate):mag * m_gate / ship:availablethrust).
}

// The ignition solve: t_go where the two endpoint demands cross — the
// minimax, since the commanded profile is linear in time and its demand
// peak therefore sits at an endpoint; equalizing the endpoints minimizes
// the peak, and the crossing is unique on the bracket (unimodal there,
// validated numerically). Wrapped in a two-pass gate-mass iteration: the
// gate mass sets both v_gate (through a_dec) and the gate-end demand, and
// each pass shrinks the mass error by roughly the propellant fraction
// (~0.2) — a contraction, two passes leave it below a tenth of a percent.
// Returns the solution, or "ok" false with both bracket-end demand gaps
// when the crossing left the bracket — the infeasibility witness.
function solve_ignition {
  local m0 is ship:mass.
  local r_gate is body:radius + tgt:terrainheight + h_gate.
  local up_site is (tgt:position - body:position):normalized.
  local g_gate is -up_site * (body:mu / r_gate ^ 2).
  local x_lead is dist_to_site().
  local v0_ is ship:velocity:surface.
  local t_lo is tau_lo_frac * x_lead / v0_:mag.
  local t_hi is tau_hi_frac * x_lead / v0_:mag.

  local m_gate is m0.
  local v_gate is 0.
  local tau is 0.
  local r0 is v(0, 0, 0).
  local v_tgt is v(0, 0, 0).
  local pass_ is 0.
  until pass_ >= 2 {
    local a_dec is f_max * ship:availablethrust / m_gate - g0.
    set v_gate to sqrt(2 * a_dec * (h_gate - h_pad)) / 2.
    set r0 to tgt:position + up_site * h_gate.
    set v_tgt to -up_site * v_gate.
    local gap is { parameter t_.
      local d is demand_pair(r0, v0_, v_tgt, t_, m0, m_gate, g_gate).
      return d[0] - d[1]. }.
    // Bracket check before bisect, so its four-line failure print never
    // tears the screen: the gate-end demand dominates at short tau, the
    // ignition end at long, and a same-signed pair means the crossing —
    // and the dip — sits outside the bracket this design certifies.
    local gap_lo is gap(t_lo).
    local gap_hi is gap(t_hi).
    if gap_lo * gap_hi > 0 {
      return lexicon("ok", false, "gap_lo", gap_lo, "gap_hi", gap_hi,
                     "x", x_lead).
    }
    set tau to bisect(gap, t_lo, t_hi, 0.05).
    local d is demand_pair(r0, v0_, v_tgt, tau, m0, m_gate, g_gate).
    // Propellant estimate: the thrust-acceleration trapezoid over the
    // profile, endpoints priced at their own masses via the demands.
    local dv_est is (d[0] + d[1]) / 2 * (ship:availablethrust / m0) * tau
                  * (m0 / ((m0 + m_gate) / 2)).
    set m_gate to m0 * constant:e ^ (-dv_est / (engine_isp() * constant:g0)).
    set pass_ to pass_ + 1.
  }
  local d is demand_pair(r0, v0_, v_tgt, tau, m0, m_gate, g_gate).
  local ends is profile_ends(r0, v0_, v_tgt, tau).
  return lexicon("ok", true, "tau", tau, "m_gate", m_gate,
                 "v_gate", v_gate, "d_dip", max(d[0], d[1]),
                 "a0", ends[0], "a1", ends[1], "x", x_lead).
}

// === FLIGHT RECORDER ===
// One CSV row per second from the powered phases. Lines beginning '#' are the
// planning numbers the flight is judged against.
local flightlog is "flight_log.csv".

// zem: the position miss a pure coast would book at the virtual gate —
// the law's own error measure, zero when the profile is converged. dem:
// commanded thrust demand as a fraction of available — above f_max is
// saturation, and the demand-vs-f_max trace is the feasibility witness.
// ach: achieved thrust acceleration over commanded, from the velocity
// difference since the last row — a persistent ratio off 1 is a wrong
// f_max or a stale mass, visible long before it is a saturated arrival.
// clear: running minimum radar altitude — instrumentation for the
// braking-arc clearance no rule yet covers (register: Open).
function log_state {
  parameter phase, t_go_, zem, dem, cmd_vec, ach, cross, clear.
  local to_site is vxcl(up:vector, tgt:position):normalized.
  log round(time:seconds, 1) + "," + phase + "," + round(t_go_, 1) + ","
      + round(altitude) + "," + round(alt:radar) + ","
      + round(vdot(ship:velocity:surface, to_site), 1) + ","
      + round(verticalspeed, 1) + ","
      + round(zem) + ","
      + round(dem, 3) + "," + round(throttle, 3) + ","
      + round(vang(cmd_vec, ship:facing:vector), 1) + ","
      + round(ship:mass, 3) + "," + round(ship:deltav:current, 1) + ","
      + round(90 - vang(up:vector, ship:facing:vector), 1) + ","
      + round(90 - vang(up:vector, cmd_vec), 1) + ","
      + round(ach, 3) + ","
      + round(cross) + ","
      + round(clear)
      to flightlog.
}

// === COAST TO PDI ===
print "Coasting to PDI: " + round(eta:periapsis) + " s.".
warpto(time:seconds + eta:periapsis - 60).
wait until eta:periapsis <= 60.
sas off.                   // kOS warns at run time that SAS fights lock steering
lock steering to srfretrograde.

// The delivery witness, logged the moment the coast ends: the body-fixed
// longitude periapsis will arrive at, against the planner's want when
// supplied — the one line that separates a plan that missed from a burn
// that delivered it wrong (notes/node-delivery-window.md).
local t_pdi is time + eta:periapsis.
if exists(flightlog) { deletepath(flightlog). }
log "# target " + round(target_lat, 4) + " " + round(target_lng, 4)
    + "  terrain " + round(tgt:terrainheight) + " m" to flightlog.
local pe_lng is geoposition_at(t_pdi, ship:orbit):lng.
log "# pe  lng " + round(pe_lng, 3) + "  site_lng " + round(tgt:lng, 3)
    + "  lead " + round(wrap_longitude(tgt:lng - pe_lng), 3) + " deg"
    + (choose "  want " + round(plan_pe_lng, 3) + "  err "
           + round(wrap_longitude(pe_lng - plan_pe_lng), 3)
       if plan_pe_lng < 999 else "") to flightlog.

// === IGNITION ===
// The t_go choice is made from the delivered state — delivery error and
// the ellipse's actual flight-path angle are absorbed into the schedule
// here, not carried as command margin. The reserve from the planned dip
// (at the planner's f_cap) up to f_max covers what arrives after
// ignition: model error, steering lag, the decrementing clock's drift.
wait until time:seconds >= t_pdi:seconds.
local sol is solve_ignition().

if not sol["ok"] {
  // Declining to ignite is the abort: the ship sits at the periapsis of a
  // stable, quicksave-able ellipse, and a profile whose demand crossing
  // left the bracket is not one this design certifies.
  print "PDI INFEASIBLE: demand crossing outside bracket "
      + "(gap_lo " + round(sol["gap_lo"], 3)
      + ", gap_hi " + round(sol["gap_hi"], 3) + "). Not igniting.".
  log "# infeasible  gap_lo " + round(sol["gap_lo"], 3)
      + "  gap_hi " + round(sol["gap_hi"], 3)
      + "  dist " + round(sol["x"]) to flightlog.
  unlock steering.
  sas on.
  set config:ipu to ipu_prior.
} else if sol["d_dip"] > f_max {
  print "PDI INFEASIBLE: dip demand " + round(sol["d_dip"], 3) + " > f_max "
      + round(f_max, 3) + ". Not igniting.".
  log "# infeasible  d_dip " + round(sol["d_dip"], 3)
      + "  f_max " + round(f_max, 3)
      + "  dist " + round(sol["x"]) to flightlog.
  unlock steering.
  sas on.
  set config:ipu to ipu_prior.
} else {

// === BRAKING ===
local tau is sol["tau"].
local v_gate is sol["v_gate"].
// The virtual gate's frozen pieces: the profile's gate-end acceleration
// and its jerk, raw-frame vectors captured at the solve. The body rotates
// under them ~0.4 deg over the burn — sub-metre on offsets this size —
// while the gate's position and vertical are rebuilt live each tick from
// the target, so the aim point rides the rotating ground exactly.
local k_jerk is (sol["a1"] - sol["a0"]) * (1 / tau).
local off_v is sol["a1"] * tau_f + k_jerk * (tau_f ^ 2 / 2).
local off_p is sol["a1"] * (tau_f ^ 2 / 2) + k_jerk * (tau_f ^ 3 / 6).
local t_ign is time:seconds.
local t_total is tau + tau_f.

// The live guidance command: the same closed form as the solve, evaluated
// each tick at the current state against the virtual gate, t_go running
// down by clock. Position and velocity targets rebuild the vertical from
// the live site direction; only the profile-extension offsets are frozen.
local t_go is t_total.
function brake_cmd {
  local up_site is (tgt:position - body:position):normalized.
  local r0 is tgt:position + up_site * (h_gate - v_gate * tau_f) + off_p.
  local v_tgt is -up_site * v_gate + off_v.
  local d_r is r0 - ship:velocity:surface * t_go.
  local d_v is v_tgt - ship:velocity:surface.
  local a_cmd is d_r * (6 / t_go ^ 2) - d_v * (2 / t_go).
  return a_cmd - g_vec().
}

print "BRAKE: t_go " + round(tau, 1) + " s, dip demand "
    + round(sol["d_dip"], 3) + ", " + round(sol["x"] / 1000, 1)
    + " km to the site.".
// Deploy the legs now, not at terminal entry: ship:bounds must be read
// after the craft has finished changing shape — a cached bounds goes
// stale on gear deployment — and the braking burn gives the animation
// two minutes where FALL may give seconds. In vacuum the deployed gear
// costs nothing.
gear on.

log "# h_pdi " + round(ship:altitude) + "  speed_pdi "
    + round(ship:velocity:surface:mag, 1)
    + "  t_go " + round(tau, 1) + "  d_dip " + round(sol["d_dip"], 3)
    + "  v_gate " + round(v_gate, 1) + "  m_gate " + round(sol["m_gate"], 3)
    + "  dist " + round(sol["x"])
    + "  dv_at_pdi " + round(ship:deltav:current, 1) to flightlog.
log "t,phase,t_go,alt,radar,v_to_site,v_vert,zem,dem,throttle,facing_err,mass,dv_rem,pitch,cmd_pitch,ach_ratio,cross,clear_min"
    to flightlog.

lock steering to lookdirup(brake_cmd(), ship:facing:topvector).
lock throttle to min(1, brake_cmd():mag * ship:mass
                        / max(0.001, ship:availablethrust)).

// The loop is witness-keeping: the locks fly the ship, the loop logs it.
// sat_s accumulates seconds of demand above f_max — saturation duration
// is the pre-gate observable that predicts a wall-side arrival. clear_min
// tracks the lowest radar reading of the arc. prev_* carry the last row's
// velocity and command for the achieved-over-commanded ratio.
local t_logged is 0.
local sat_s is 0.
local clear_min is alt:radar.
local prev_v is ship:velocity:surface.
local prev_t is time:seconds.
local prev_cmd is brake_cmd().
until t_go <= tau_f {
  set t_go to max(tau_f, t_total - (time:seconds - t_ign)).
  set clear_min to min(clear_min, alt:radar).
  if time:seconds - t_logged >= 1 {
    local cmd is brake_cmd().
    local dem is cmd:mag * ship:mass / max(0.001, ship:availablethrust).
    if dem > f_max { set sat_s to sat_s + (time:seconds - t_logged). }
    local dt_row is max(0.1, time:seconds - prev_t).
    local a_meas is (ship:velocity:surface - prev_v) * (1 / dt_row) - g_vec().
    local ach is a_meas:mag / max(0.001, prev_cmd:mag).
    local up_site is (tgt:position - body:position):normalized.
    local zem is (tgt:position + up_site * (h_gate - v_gate * tau_f) + off_p
                  - ship:velocity:surface * t_go):mag.
    log_state("BRAKE", t_go, zem, dem, cmd, ach,
        vdot(tgt:position, vcrs(ship:velocity:surface, up:vector):normalized),
        clear_min).
    print "BRK tgo=" + round(t_go) + " dem=" + round(dem, 3)
        + " zem=" + round(zem) + " v=" + round(ship:velocity:surface:mag, 1)
        + " d=" + round(dist_to_site()) + "        " at (0, 10).
    set prev_v to ship:velocity:surface.
    set prev_t to time:seconds.
    set prev_cmd to cmd.
    set t_logged to time:seconds.
  }
  wait 0.
}

// === HIGH GATE ===
// Braking's exit is the clock: t_go at the floor, the ship at the real
// gate state to the law's tracking accuracy. Two arrival checks are
// witnessed before FALL commits — the corridor fraction (must be <= 1,
// designed ~0.25) and the residuals the law promised — visible here with
// the engine still lit, not at the pad.
local a_dec_gate is f_max * ship:availablethrust / ship:mass - g0.
local corridor is ship:velocity:surface:mag ^ 2
    / (2 * a_dec_gate
         * max(1, ship:altitude - tgt:terrainheight - h_pad)).
log "# high gate  radar " + round(alt:radar)
    + "  speed " + round(ship:velocity:surface:mag, 1)
    + "  drift " + round(vxcl(up:vector, ship:velocity:surface):mag, 1)
    + "  offset " + round(vxcl(up:vector, tgt:position):mag)
    + "  corridor " + round(corridor, 2)
    + "  sat_s " + round(sat_s, 1)
    + "  lat " + round(ship:geoposition:lat, 4)
    + "  lng " + round(ship:geoposition:lng, 4) to flightlog.

// === TERMINAL DESCENT ===
// The flown chain, kept whole. FALL: engine off, holding surface
// retrograde — with drift nulled at the gate, retrograde and plumb
// coincide to within the residual, so the nose is already on the thrust
// direction when the arrest ignites and there is no slew to pay. The
// arrest burn holds the vertical deceleration a_req that carries the
// descent rate to v_floor at the pad; the v/|vv| factor is the retrograde
// lean's cosine, restoring the vertical share the lean sends sideways,
// gated with the steering — below v_switch the nose is plumb and there is
// no lean to pay for. Behind schedule the request rises past f_max into
// the reserve on its own; near the ground a_req falls below hover and the
// ship settles instead of bouncing.
print "FALL: from " + round(alt:radar) + " m.".
lock throttle to 0.
// The terminal phase's altimeter: the height of the craft's lowest point
// above the ground. alt:radar measures from the core, which sits metres
// above the legs' contact point, so heights against it plan the flare
// into the ground by the craft's own core height. ship:bounds is
// obtained once — the docs price obtaining it as expensive, and the box
// only goes stale if the ship changes shape, which it finished doing
// when the gear deployed at ignition; the bottomaltradar suffix off the
// stored box is the cheap per-tick read, and it tracks attitude, so
// while the craft still leans it reads the corner that would touch
// first. The one-time check guards an API this program has not flown
// through a flare: a box reading above the core, or a core height beyond
// any lander's geometry, falls back to the core radar — the flown,
// six-metres-wrong behavior — rather than flying a nonsense number.
local box is ship:bounds.
local dh_core is alt:radar - box:bottomaltradar.
local use_box is dh_core >= 0 and dh_core <= 20.
if not use_box {
  print "WARN: bounds datum " + round(dh_core, 1) + " m; using core radar.".
}
log "# bounds  dh_core " + round(dh_core, 2)
    + "  tilt " + round(vang(up:vector, ship:facing:vector), 1)
    + (choose "" if use_box else "  REJECTED") to flightlog.
local lock h_bot to choose box:bottomaltradar if use_box else alt:radar.
local lock a_dec to f_max * ship:availablethrust / ship:mass - g0.
local lock a_req to (verticalspeed ^ 2 - v_floor ^ 2)
                  / (2 * max(1, h_bot - h_pad)).
local burning is false.
lock steering to lookdirup(
    (choose srfretrograde:vector if ship:velocity:surface:mag > v_switch
            else up:vector),
    ship:facing:topvector).
lock throttle to choose 0 if not burning
    else (g0 + a_req)
       * (choose ship:velocity:surface:mag / max(1, -verticalspeed)
                 if ship:velocity:surface:mag > v_switch else 1)
       * ship:mass / max(0.001, ship:availablethrust).
set t_logged to 0.
local t_printed is 0.
until ship:status = "LANDED"
    or (h_bot < h_pad and verticalspeed > -0.1) {
  if not burning
      and ship:velocity:surface:mag ^ 2
          >= 2 * a_dec * max(0, h_bot - h_pad) {
    set burning to true.
    print "ARREST: from " + round(h_bot) + " m.".
  }
  // log_dt: seconds between CSV rows — 0.25 under 40 m radar altitude,
  // 1 s otherwise.
  local log_dt is choose 0.25 if alt:radar < 40 else 1.
  if time:seconds - t_logged >= log_dt {
    // The commanded vector for the row; max(0.001, ...) keeps it pointing
    // somewhere when the engine is off. The cross column carries the full
    // horizontal drift speed — v_to_site is blind to the tangential
    // component. zem is spent below the gate; ach rides on the throttle
    // request, meaningful only while burning.
    local cmd is max(0.001, throttle * ship:availablethrust / ship:mass)
        * (choose srfretrograde:vector
                  if ship:velocity:surface:mag > v_switch else up:vector).
    local dt_row is max(0.1, time:seconds - prev_t).
    local a_meas is (ship:velocity:surface - prev_v) * (1 / dt_row) - g_vec().
    local ach is choose a_meas:mag / max(0.001, prev_cmd:mag)
        if burning else 1.
    set clear_min to min(clear_min, alt:radar).
    log_state((choose "ARREST" if burning else "FALL"), 0, 0, throttle,
        cmd, ach, vxcl(up:vector, ship:velocity:surface):mag, clear_min).
    set prev_v to ship:velocity:surface.
    set prev_t to time:seconds.
    set prev_cmd to cmd.
    set t_logged to time:seconds.
  }
  if time:seconds - t_printed >= 1 {
    // Fixed-row readout: v the descent rate, sched the total speed that
    // ignites the arrest burn, f the throttle, miss the horizontal offset
    // that becomes the landing error, drift the horizontal speed the burn
    // is spending.
    print "TRM b=" + round(h_bot) + " v=" + round(verticalspeed, 1)
        + " sched=" + round(sqrt(2 * a_dec * max(0, h_bot - h_pad)))
        + " f=" + round(throttle, 3)
        + " miss=" + round(vxcl(up:vector, tgt:position):mag)
        + " drift=" + round(vxcl(up:vector, ship:velocity:surface):mag, 1)
        + "     " at (0, 10).
    set t_printed to time:seconds.
  }
  wait 0.
}
lock throttle to 0.
// Touchdown witness: state at ground contact, before the settle wait — vv
// the descent rate, drift the horizontal speed, tilt off plumb, radar the
// core altimeter reading, bot the height of the craft's lowest point
// above ground — the pair re-measures dh_core at contact.
log "# touchdown  vv " + round(verticalspeed, 1)
    + "  drift " + round(vxcl(up:vector, ship:velocity:surface):mag, 1)
    + "  tilt " + round(vang(up:vector, ship:facing:vector), 1)
    + "  radar " + round(alt:radar, 1)
    + "  bot " + round(h_bot, 1) to flightlog.
// Hold the ship plumb while the legs settle. lookdirup with the current
// topvector asks only for the nose: bare `up` is a full direction, roll
// included, and the steering manager would grind the landed ship around
// its legs to satisfy a roll nothing needs.
lock steering to lookdirup(up:vector, ship:facing:topvector).
// Hand off to SAS only once the craft has stopped moving: below ~1 deg/s of
// rotation the legs have stopped rocking. The clock is a hung-wait guard —
// rocking on a slope may never settle below the threshold.
local t_land is time:seconds.
// Settle trace: elapsed time since touchdown, tilt off plumb, and angular
// rate, once a second while the legs settle.
local t_settle_logged is 0.
until ship:angularvel:mag < 0.02 or time:seconds - t_land > 10 {
  if time:seconds - t_settle_logged >= 1 {
    log "# settle  t " + round(time:seconds - t_land, 1)
        + "  tilt " + round(vang(up:vector, ship:facing:vector), 1)
        + "  angularvel " + round(ship:angularvel:mag, 3) to flightlog.
    set t_settle_logged to time:seconds.
  }
  wait 0.
}
unlock steering.
unlock throttle.
set ship:control:pilotmainthrottle to 0.
sas on.
set config:ipu to ipu_prior.
local miss is vxcl(up:vector, tgt:position):mag.
// tilt: degrees off plumb after the legs settle. The landing target is a
// parked craft, so uprightness is a measured witness, not an assumption.
local tilt is round(vang(up:vector, ship:facing:vector), 1).
print "Landed. Miss: " + round(miss) + " m.  Tilt: " + tilt + " deg.".
log "# landed  miss " + round(miss) + " m  lat "
    + round(ship:geoposition:lat, 4) + "  lng "
    + round(ship:geoposition:lng, 4) + "  tilt " + tilt + "  dv_rem "
    + round(ship:deltav:current, 1) to flightlog.

}
