// powered_landing.ks — Apollo-style targeted powered descent (Mun/Minmus).
// Design: notes/apollo-powered-descent.md, with the quadratic t_go closure
// from notes/klumpp-guidance-derivation.md (§4b/§5).
//
// Deliberately bare-bones: this is the reference implementation of the
// guidance design, not an operational autopilot. Steps a more complex
// mission would need are marked "OMITTED:" at the point they would go.

@lazyglobal off.

clearscreen.
print "=== POWERED LANDING ===".

run "common".              // execute_node
run "../core/kepler".      // time_to_longitude, wrap_longitude (pulls in optimize.ks)

parameter target_lat is 0.
parameter target_lng is 0.
// h_pdi: PDI (periapsis) altitude, m. Mun 8-12 km, Minmus 4-6 km. The
// descent's cost trade lives here: braking-burn duration is pinned by the
// drop from h_pdi to the high gate, so on a high-TWR craft the Delta-v
// savings come from LOWERING h_pdi, not from burning harder. The floor
// under h_pdi is the landscape: the descent ellipse must clear every ridge
// on the approach corridor — which is mission planning (see Phase 2), not
// flight software.
parameter h_pdi is 10000.
parameter lead_deg is 0.          // PDI point this far up-range of the target; 0 = compute.
parameter brake_throttle is 0.75. // Design average throttle for the braking phase. The
                                  // reserve (1 - brake_throttle) is the only authority
                                  // margin in the design; 0.7-0.8 is the defensible band.

local tgt is body:geopositionlatlng(target_lat, target_lng).

// OMITTED: Phase 0 (plane alignment). We assume an equatorial parking orbit
// over a low-latitude site. An inclined target needs a normal-direction
// correction here — or patience, since the body rotates the site under the
// orbital plane twice per rotation.

// === DESCENT DESIGN CONSTANTS ===
// The high gate, in numbers. Read in three places — the planning block
// below (burn duration, feasibility, lead angle) and the gate
// constructors — so they live here, once. Provenance and reasoning for
// the values stay with the high_gate constructor's comment.
local hg_offset is 2000.        // aim point's up-range offset from the site, m
local hg_height is 2000.        // aim point's height above the site terrain, m
local hg_ground_speed is 60.    // arrival speed toward the site, m/s
local hg_descent_rate is 30.    // arrival descent rate (positive down), m/s
// Arrival-acceleration scalar for the quadratic closure, both gates:
// arrive gently slowing, 0.5 m/s^2 net upward.
local a_arrival is 0.5.

// Pre-flight authority check: local TWR at PDI altitude. Below ~1.5 the
// braking phase has no vertical authority left (design floor is 2).
// OMITTED: staging. a_max reads the current stage; a craft that stages
// during descent needs a per-phase mass and thrust model.
local g_pdi is body:mu / (body:radius + h_pdi) ^ 2.
local a_max is ship:availablethrust / ship:mass.
local twr_pdi is a_max / g_pdi.
if twr_pdi < 1.5 {
  print "ABORT: local TWR at PDI is " + round(twr_pdi, 2) + " (need >= 1.5).".
  print "Nothing has been committed. Add thrust or shed mass.".
  wait until false.
}

// Braking geometry, shared by the feasibility check and the lead angle.
local r_pe is body:radius + h_pdi.
local sma_park is (ship:orbit:semimajoraxis + r_pe) / 2.
// Surface-relative periapsis speed: ground-track speed is what both the
// lead and the burn have to cover.
local v_pe is sqrt(body:mu * (2 / r_pe - 1 / sma_park))
            - 2 * constant:pi * r_pe / body:rotationperiod.
// vertical distance from PDI down to the high gate
local vertical_distance_to_gate is
    max(0, h_pdi - (tgt:terrainheight + hg_height)).

// How long will the braking burn last? In flight, BRAKE's t_go comes from
// the quadratic closure solved on the vertical axis. At PDI the vertical
// state is known — vertical speed 0 at periapsis — but the vertical
// DISTANCE is more than vertical_distance_to_gate: an aim point d meters
// down-range sits d^2/(2*r_pe) below the ship's local horizontal plane,
// because the body curves away, and the law's local-vertical axis sees
// that sag as descent to be flown. On small bodies the sag can rival the
// nominal drop itself, stretching the flown t_go well past the flat-earth
// figure. Sag depends on distance, distance on duration, duration on sag
// — so iterate; the feedback is a correction of a correction, and three
// passes are plenty.
//
// Each pass solves, as in solve_t_go with vv = 0, vtv = -hg_descent_rate:
//   a_arrival*t^2 + (4*hg_descent_rate)*t - 6*(drop + sag) = 0
local brake_duration is 0.
local brake_distance is 0.
local sag is 0.
local pass is 0.
until pass >= 3 {
  local qa is a_arrival.
  local qb is 4 * hg_descent_rate.
  local qc is -6 * (vertical_distance_to_gate + sag).
  set brake_duration to (-qb + sqrt(qb ^ 2 - 4 * qa * qc)) / (2 * qa).
  // Ground covered while decelerating v_pe -> hg_ground_speed. PDI to the
  // AIM POINT is brake_distance; the site lies hg_offset farther on.
  set brake_distance to (v_pe + hg_ground_speed) / 2 * brake_duration.
  set sag to brake_distance ^ 2 / (2 * r_pe).
  set pass to pass + 1.
}

// Engine feasibility. The braking burn must shed (v_pe - hg_ground_speed)
// of ground speed, and the deceleration available for that is what remains
// of the design throttle after holding against gravity (a_h, horizontal
// budget). So the engine needs at least min_brake_duration; the burn's
// actual duration is brake_duration, fixed by the vertical geometry above.
// If the engine needs longer than the geometry gives, h_pdi is too low
// for this craft — the closure would demand more thrust than exists
// mid-burn. Authority is thus verified here, on the ground, and watched
// by the saturation guard in flight.
local a_h is sqrt(max(0.001, (brake_throttle * a_max) ^ 2 - g_pdi ^ 2)).
local min_brake_duration is (v_pe - hg_ground_speed) / a_h.
if min_brake_duration > brake_duration {
  print "ABORT: h_pdi too low. The engine needs " + round(min_brake_duration)
      + " s to brake; the descent to the gate lasts only "
      + round(brake_duration) + " s.".
  wait until false.
}

// Lead angle: the ground the ship covers during the braking burn, plus
// the gate's own up-range offset, converted to arc degrees. Lead, speed,
// and duration must all describe the SAME burn: hand the law a
// boundary-value problem that violates distance = speed x time and it
// will still solve it — by diving or by reversing, whichever contortion
// satisfies the constraints.
if lead_deg <= 0 {
  set lead_deg to (brake_distance + hg_offset) / r_pe * constant:radtodeg.
}

print "Target: " + round(target_lat, 2) + ", " + round(target_lng, 2)
    + "  terrain " + round(tgt:terrainheight) + " m.".
print "Local TWR at PDI: " + round(twr_pdi, 2)
    + "  lead angle: " + round(lead_deg, 1) + " deg"
    + "  (braking ~" + round(brake_duration) + " s).".

// === FLIGHT RECORDER ===
// Test-ladder instrument: one CSV row per second from the powered phases, so
// flights are analyzed from telemetry instead of remembered impressions.
// Lands in this directory (the kOS archive); overwritten on each launch.
// Lines beginning '#' are metadata — the planning numbers each flight is
// judged against, recorded so the CSV is self-contained and the console is
// never the only witness. v_to_site is signed horizontal speed toward the
// site — the reversal detector; facing_err vs throttle exposes
// wrong-direction burning; dv_rem turns phase costs into ledger entries.
local flightlog is "flight_log.csv".
if exists(flightlog) { deletepath(flightlog). }
log "# target " + round(target_lat, 4) + " " + round(target_lng, 4)
    + "  terrain " + round(tgt:terrainheight) + " m" to flightlog.
log "# h_pdi " + h_pdi + "  brake_throttle " + brake_throttle
    + "  twr_pdi " + round(twr_pdi, 2) to flightlog.
log "# lead_deg " + round(lead_deg, 2)
    + "  brake_duration " + round(brake_duration, 1)
    + "  brake_distance " + round(brake_distance)
    + "  sag " + round(sag) to flightlog.
log "# v_pe " + round(v_pe, 1)
    + "  min_brake_duration " + round(min_brake_duration, 1)
    + "  desired_pdi_lng " + round(wrap_longitude(tgt:lng - lead_deg), 2)
    to flightlog.
log "# dv_at_load " + round(ship:deltav:current, 1) to flightlog.
log "t,phase,t_go,alt,radar,v_to_site,v_vert,aim_dist,a_cmd,throttle,facing_err,mass,dv_rem,pitch,cmd_pitch"
    to flightlog.

// pitch/cmd_pitch are degrees above the horizon, of the nose and of the
// commanded thrust vector: cmd_pitch shows what guidance is asking for
// (e.g. a legitimately below-horizon command while building descent rate),
// pitch shows what the ship is doing about it.
function log_state {
  parameter phase, t_go, aim_geo, aim_alt, a_thrust.
  local to_site is vxcl(up:vector, tgt:position):normalized.
  log round(time:seconds, 1) + "," + phase + "," + round(t_go, 1) + ","
      + round(altitude) + "," + round(alt:radar) + ","
      + round(vdot(ship:velocity:surface, to_site), 1) + ","
      + round(verticalspeed, 1) + ","
      + round(aim_geo:altitudeposition(aim_alt):mag) + ","
      + round(a_thrust:mag, 2) + "," + round(throttle, 3) + ","
      + round(vang(a_thrust, ship:facing:vector), 1) + ","
      + round(ship:mass, 3) + "," + round(ship:deltav:current, 1) + ","
      + round(90 - vang(up:vector, ship:facing:vector), 1) + ","
      + round(90 - vang(up:vector, a_thrust), 1)
      to flightlog.
}

// === PHASE 1: DOI ===

// Plan the DOI burn: a retrograde node that drops the periapsis to h_pdi,
// placed so PDI falls lead degrees up-range (west) of the target longitude.
// Assumes a prograde, near-circular, equatorial parking orbit.
function plan_doi {
  parameter tgt_lng, lead.
  parameter lng_bias is 0.   // placement correction, degrees east; fed
                             // back by perform_doi's periapsis check

  // The site moves east while we coast half an orbit down to PDI, so aim
  // at where it will be. Coast duration comes from the descent ellipse;
  // the parking orbit is near-circular, so its sma stands in for the burn
  // radius.
  local sma_desc is (ship:orbit:semimajoraxis + body:radius + h_pdi) / 2.
  local t_coast is constant:pi * sqrt(sma_desc ^ 3 / body:mu).
  local site_advance is t_coast * 360 / body:rotationperiod.
  local aim_lng is tgt_lng + site_advance.

  // The burn point becomes the descent ellipse's apoapsis, half an orbit
  // (180 degrees inertial) before periapsis.
  local burn_lng is wrap_longitude(aim_lng - lead - 180 + lng_bias).
  local t_burn is time_to_longitude(burn_lng).   // absolute TimeStamp

  local r_burn is (positionat(ship, t_burn) - body:position):mag.
  local r_pe is body:radius + h_pdi.
  local sma is (r_burn + r_pe) / 2.
  local v_new is sqrt(body:mu * (2 / r_burn - 1 / sma)).
  local v_old is velocityat(ship, t_burn):orbit:mag.

  // OMITTED: fine placement. For sub-degree PDI placement, wrap this plan
  // in a minimize() over lead, scoring the resulting periapsis position
  // against the desired one (the landing_v2/calculate_deorbit_burn.ks
  // structure). Guidance absorbs degree-level error, so we don't bother.
  print "DOI plan: coast " + round(t_coast / 60, 1) + " min; site advances "
      + round(site_advance, 2) + " deg.".
  return node(t_burn:seconds, 0, 0, v_new - v_old).
}

function perform_doi {
  // Where the ship should be, in body-frame longitude, at PDI.
  local desired_pdi_lng is wrap_longitude(tgt:lng - lead_deg).
  local bias is 0.
  local nd is 0.
  local attempts is 0.

  until false {
    set nd to plan_doi(tgt:lng, lead_deg, bias).
    add nd.
    // A non-positive ETA means the planner failed (e.g. time_to_longitude
    // found no root and returned its time-in-the-past sentinel). Catch
    // the whole garbage-plan class here, before anything burns.
    if nd:eta <= 0 {
      remove nd.
      guidance_abort("DOI plan puts the burn in the past.").
    }

    // Check the plan against itself: nd:orbit is the post-burn orbit KSP
    // predicts, so ask where ITS periapsis actually falls and feed the
    // miss back into the burn longitude. This closes placement against
    // every modeling error at once — e.g. parking-orbit eccentricity
    // rotating the descent ellipse's periapsis away from the burn point,
    // which matters because a few m/s of radial velocity is large
    // against a small DOI burn.
    local t_pdi is time_of_periapsis(timestamp(nd:time), nd:orbit).
    local predicted_lng is geoposition_at(t_pdi, nd:orbit):lng.
    local error is wrap_longitude(predicted_lng - desired_pdi_lng).
    set attempts to attempts + 1.
    print "DOI plan " + attempts + ": periapsis lng "
        + round(predicted_lng, 2) + ", want " + round(desired_pdi_lng, 2)
        + " (err " + round(error, 2) + " deg).".
    log "# doi_plan " + attempts + ": pe_lng " + round(predicted_lng, 2)
        + "  want " + round(desired_pdi_lng, 2)
        + "  err " + round(error, 2) to flightlog.

    if abs(error) < 0.2 or attempts >= 4 { break. }
    remove nd.
    set bias to bias - error.
  }

  print "DOI: " + round(nd:prograde, 1) + " m/s in " + round(nd:eta) + " s.".
  log "# doi_burn " + round(nd:prograde, 1) + " m/s" to flightlog.
  execute_node(nd).
}

// === PHASE 2: COAST TO PDI ===

// No terrain guard here by design. The descent ellipse clears terrain
// because h_pdi was chosen to clear it, and the approach corridor is
// assumed no rougher than the site. Verifying both is mission planning,
// not guidance — the same division of labor Apollo used.
function coast_to_pdi {
  local coast_margin is 60.      // stop warp this far before PDI (s): time
                                 // for the steering lock to swing the ship
                                 // retrograde, with margin

  // OMITTED: coast monitoring. On longer missions an on-screen abort
  // instrument belongs here — "if the engine never lights, where do I
  // hit?" (core/impact.ks predict_impact) — plus the option of a
  // meter-per-second trim, cheap now and expensive later.

  print "Coasting to PDI: " + round(eta:periapsis) + " s.".
  warpto(time:seconds + eta:periapsis - coast_margin).
  wait until eta:periapsis <= coast_margin.

  // Pre-orient for the braking burn while the last minute runs out.
  // 1 s is "at periapsis" for our purposes: guidance absorbs tens of
  // seconds of ignition-timing slop, so the threshold only needs to be
  // small against that.
  lock steering to srfretrograde.
  wait until eta:periapsis <= 1.
}

// === GUIDANCE CORE (P63/P64) ===

// One tick of quadratic guidance: returns the thrust-acceleration vector that
// puts the ship at the aim point with velocity v_tgt after t_go seconds,
// assuming total acceleration varies linearly in time (constant jerk):
//   a_cmd = 6·(r_tgt − r)/t_go² − (4·v + 2·v_tgt)/t_go
// a_cmd is TOTAL acceleration — the thrust demand is a_cmd minus gravity.
function guidance_step {
  parameter aim_geo.   // geoposition of the aim point
  parameter aim_alt.   // altitude of the aim point above the datum
  parameter v_tgt.     // desired velocity at the aim point (surface frame)
  parameter t_go.      // time remaining to reach the aim point

  // r_tgt − r: the aim point, ship-relative. Re-queried every tick; body
  // rotation is absorbed by re-measurement, never transformed explicitly.
  local r_aim is aim_geo:altitudeposition(aim_alt).
  local vel is ship:velocity:surface.   // "v" would shadow the builtin V()
  local a_cmd is 6 * r_aim / t_go ^ 2 - (4 * vel + 2 * v_tgt) / t_go.

  local g_vec is body:position:normalized * (body:mu / body:position:mag ^ 2).
  return a_cmd - g_vec.
}

// Solve for t_go: the target-acceleration closure (Klumpp note §4b/§5).
// Evaluating the constant-jerk profile at arrival gives the acceleration
// the ship will have when it reaches the aim point:
//   a(T) = −6·R/T² + (2·v + 4·v_tgt)/T        (R = r_tgt − r, T = t_go)
// Demanding that this equal a chosen a_v_tgt and clearing denominators
// yields a quadratic in T:
//   a_v_tgt·T² − (2·v + 4·v_tgt)·T + 6·R = 0
// whose coefficients are qa, qb, qc below, all projected onto the local
// vertical — one scalar equation for one scalar unknown; the horizontal
// axes take whatever arrival acceleration the profile then gives.
// Returns the time-to-go, or -1 if no feasible positive root exists — an
// abort signal, not a nuisance. Where two roots are positive, the smallest
// is the direct trajectory (the first time the arrival condition can be
// met); the larger is a loitering profile we never want.
function solve_t_go {
  parameter aim_geo, aim_alt, v_tgt.
  parameter a_v_tgt.   // desired NET vertical accel at arrival (up-positive)

  local u is up:vector.
  local r_aim is aim_geo:altitudeposition(aim_alt).
  local vel is ship:velocity:surface.   // "v" would shadow the builtin V()

  // project onto the vertical axis -> scalars
  local rv is vdot(r_aim, u).
  local vv is vdot(vel, u).
  local vtv is vdot(v_tgt, u).

  local qa is a_v_tgt.
  local qb is -(2 * vv + 4 * vtv).
  local qc is 6 * rv.

  // 1e-6 / 1e-9 are "is this coefficient effectively zero" thresholds,
  // not physical tolerances.
  if abs(qa) < 1e-6 {               // degenerate: a_v_tgt = 0 -> linear
    if abs(qb) < 1e-9 { return -1. }
    local t is -qc / qb.
    if t > 0 { return t. }
    return -1.
  }

  local disc is qb * qb - 4 * qa * qc.
  if disc < 0 { return -1. }        // no real arrival time -> abort
  local s is sqrt(disc).

  // smallest strictly-positive root
  local best is -1.
  for t in list((-qb - s) / (2 * qa), (-qb + s) / (2 * qa)) {
    if t > 0 and (best < 0 or t < best) { set best to t. }
  }
  return best.
}

// === GATE FLYER (P63 = braking, P64 = approach) ===

// Halt-and-hand-over, the same convention as the pre-flight check, but with
// the engine cut first so the pilot inherits a ship, not a fight.
// OMITTED: active aborts. An operational script would fly abort-to-orbit
// (thrust up and up-range back to a stable ellipse) from any powered phase
// rather than handing a falling ship to the pilot.
function guidance_abort {
  parameter msg.
  lock throttle to 0.
  unlock steering.
  print "ABORT: " + msg.
  print "Automation halted; you have the ship.".
  wait until false.
}

// Guidance has diverged with no altitude left to argue about it: stop
// flying the law, kill velocity at full thrust, and land where we are.
// The target is abandoned — a wrong-place landing beats a right-place
// crater. Ends parked: the mission sequence must not continue past this.
function emergency_land {
  parameter why.
  print "EMERGENCY: " + why.
  print "Abandoning target. Killing velocity; landing here.".
  lock steering to srfretrograde.
  lock throttle to 1.
  wait until ship:velocity:surface:mag < 10 or ship:status = "LANDED".
  terminal_descent().
  print "Down, off target. You have the ship.".
  wait until false.
}

// === GATES ===
// A gate is the target state for one phase of the guidance law, as a
// lexicon:
//   name       printed phase tag
//   aim_geo    geoposition of the aim point (body-fixed)
//   aim_alt    aim point altitude above the datum, m
//   v_horiz    arrival speed toward the site, m/s
//   v_vert     arrival vertical speed, m/s (up-positive)
//   t_handoff  hand off to the next phase when t_go falls below this, s
//              (the law's 1/t_go^2 gains diverge at zero)
//   alt_floor  radar-altitude floor, m — a flight-software invariant, not
//              mission planning: there is no legitimate reason to be near
//              the ground with the gate still ahead, so crossing the floor
//              means guidance has diverged and emergency_land takes over
//   closure    the function that pins t_go — the seventh scalar condition
//              the six endpoint equations leave free: v_tgt -> t_go, or -1
//              if infeasible. A kOS closure implementing a guidance
//              closure: it captures its own aim point and design scalars.
//              Which condition to use is a per-gate choice (Klumpp §6).

// High gate — the P63→P64 handoff. Apollo's sat ~2,200 m up with the LM
// arriving near 150 m/s; ours is 2,000 m above the SITE's surveyed terrain
// (not the terrain under the gate — within 2 km of a surveyed site the
// difference is noise), arriving at 60 m/s forward and 30 m/s down, the
// Apollo state scaled to KSP's smaller worlds. The 2 km up-range offset
// sets the approach chord at ~43 deg — steeper than Apollo's 15–25 deg,
// which existed so the crew could see the site out the window; steep buys
// us terrain clearance and a near-vertical final phase. The 60:30 arrival
// ratio is a ~27 deg flight path, shallower than the chord, so the leg
// arcs in and steepens as horizontal velocity bleeds off.
function high_gate {
  local u_site is (tgt:position - body:position):normalized.
  local up_range is vxcl(u_site, -tgt:position):normalized.
  local aim_geo is body:geopositionof(tgt:position + hg_offset * up_range).
  local aim_alt is tgt:terrainheight + hg_height.

  // Quadratic closure, arriving gently slowing (a_arrival net up). The
  // engine does not appear in the equation: authority is checked before
  // DOI (min_brake_duration) and in flight (the saturation guard).
  local closure is {
    parameter v_tgt.
    return solve_t_go(aim_geo, aim_alt, v_tgt, a_arrival).
  }.

  return lexicon(
    "name",      "BRAKE",
    "aim_geo",   aim_geo,               // hg_offset up-range ("short") of the site
    "aim_alt",   aim_alt,               // hg_height above the site's terrain
    "v_horiz",   hg_ground_speed,       // arrive moving toward the site...
    "v_vert",    -hg_descent_rate,      // ...and down: ~27 deg arrival path
    "t_handoff", 5,
    "alt_floor", 1000,                  // braking has no business below 1 km
    "closure",   closure).
}

// Low gate — the P64→P66 handoff, Apollo's 500 ft kept unscaled: 150 m is
// set by "close enough that alt:radar is ground truth and the gear can come
// out," not by body size. All horizontal velocity dies here; arrive in a
// slow vertical descent for the terminal controller to inherit.
function low_gate {
  local aim_alt is tgt:terrainheight + 150.

  // Approach is geometry-limited: pick t_go so the ship arrives gently
  // slowing (0.5 m/s^2 net upward), handing the terminal controller a
  // tame descent.
  local closure is {
    parameter v_tgt.
    return solve_t_go(tgt, aim_alt, v_tgt, a_arrival).
  }.

  return lexicon(
    "name",      "APPROACH",
    "aim_geo",   tgt,       // directly over the site: the ground below IS the site
    "aim_alt",   aim_alt,   // Apollo's 500 ft low gate, kept unscaled
    "v_horiz",   0,         // all horizontal motion dead here
    "v_vert",    -5,        // slow vertical descent for terminal to inherit
    "t_handoff", 5,
    "alt_floor", 50,
    "closure",   closure).
}

// Fly the guidance law to one gate. Runs until t_go reaches the gate's
// handoff time, then returns with the engine still burning — the next
// phase picks up the throttle without a gap.
function fly_gate {
  parameter gate.

  // Desired arrival velocity, recomputed each call so the direction stays
  // body-fixed as the body rotates: v_horiz toward the site, v_vert up.
  local v_tgt_now is {
    local vt is gate:v_vert * up:vector.
    if gate:v_horiz <> 0 {
      local to_site is vxcl(up:vector,
        tgt:position - gate:aim_geo:altitudeposition(gate:aim_alt)).
      set vt to vt + gate:v_horiz * to_site:normalized.
    }
    return vt.
  }.

  // The gate's closure pins t_go. Extracted to a local once: kOS will not
  // call a delegate directly off a lexicon suffix.
  local gate_closure is gate:closure.
  local t_go is gate_closure(v_tgt_now()).
  if t_go < 0 {
    guidance_abort(gate:name + ": closure found no feasible t_go.").
  }
  // Down-range distance at gate entry: at BRAKE, this is the achieved DOI
  // placement, to be compared against the planned lead angle.
  print gate:name + ": t_go " + round(t_go) + " s; site "
      + round(vxcl(up:vector, tgt:position):mag / 1000, 1) + " km down-range.".

  local a_thrust is guidance_step(gate:aim_geo, gate:aim_alt, v_tgt_now(), t_go).
  lock steering to lookdirup(a_thrust, ship:facing:topvector).
  // max() guards the division against flameout. The vang gate holds the
  // throttle closed while the ship swings toward a newly commanded
  // direction: thrust at angle theta delivers cos(theta) of the command
  // and injects sin(theta) of fresh error — past 90 deg it fights itself.
  // Nominally this gate never fires; it exists to deny fuel to divergence,
  // since mis-pointed thrust is the energy source that sustains a
  // guidance limit cycle.
  lock throttle to choose
      min(1, a_thrust:mag * ship:mass / max(0.001, ship:availablethrust))
      if vang(a_thrust, ship:facing:vector) < 30
      else 0.

  local t_last is time:seconds.     // decrement anchor
  local t_solved is time:seconds.   // last re-solve
  local sat_start is -1.            // when the throttle demand hit the stop
  local t_logged is 0.              // last flight-recorder row

  until t_go < gate:t_handoff {
    set a_thrust to guidance_step(gate:aim_geo, gate:aim_alt, v_tgt_now(), t_go).

    // Ground-proximity invariant; see the gates glossary. Too low for
    // hand-over — use whatever authority is left to land, not to halt.
    if alt:radar < gate:alt_floor {
      emergency_land(gate:name + ": radar altitude below "
          + gate:alt_floor + " m floor.").
    }

    if time:seconds - t_logged >= 1 {
      log_state(gate:name, t_go, gate:aim_geo, gate:aim_alt, a_thrust).
      set t_logged to time:seconds.
    }

    // Saturation cross-guard: this closure never looks at a_max, so watch
    // the demand ourselves. Brief spikes are guidance absorbing an error;
    // sustained saturation means the gate is not reachable at full thrust.
    // The 5 s window is long against any transient spike and short against
    // the damage a genuinely unreachable gate can do.
    if a_thrust:mag * ship:mass >= ship:availablethrust {
      if sat_start < 0 { set sat_start to time:seconds. }
      else if time:seconds - sat_start > 5 {
        guidance_abort(gate:name + ": thrust saturated for 5 s.").
      }
    } else {
      set sat_start to -1.
    }

    wait 0.

    // t_go: decrement by wall clock each tick, re-solve every ~10 s to
    // shed accumulated model error. The cadence only needs to be short
    // against the multi-minute phase; ~10 s of decrement drift is noise.
    set t_go to t_go - (time:seconds - t_last).
    set t_last to time:seconds.
    if time:seconds - t_solved > 10 {
      set t_go to gate_closure(v_tgt_now()).
      if t_go < 0 {
        guidance_abort(gate:name + ": t_go re-solve found no feasible root.").
      }
      set t_solved to time:seconds.
    }
  }
}

// === PHASE 5: TERMINAL DESCENT (P66) ===

// Rate-of-descent control, which is what P66 actually was: the reference
// descent rate is a function of radar altitude, and the throttle servos
// the actual rate onto it around a gravity-cancelling feedforward.
//
// The reference profile is -min(5, max(2, alt:radar / 10)):
//   - capped at 5 m/s so it is continuous with the low gate's arrival
//     velocity (an uncapped alt/10 would command -15 m/s at 150 m,
//     making P66 begin by speeding the descent back up);
//   - proportional to height (tau = 10 s) through the middle;
//   - floored at 2 m/s so touchdown happens rather than being approached
//     asymptotically.
function terminal_descent {
  print "TERMINAL: rate-of-descent control from " + round(alt:radar) + " m.".

  // OMITTED: site redesignation and slope/quality checks. The ground below
  // is trusted because it was surveyed before the mission — that trust is
  // the core design trade, not an oversight.

  local g0 is body:mu / body:radius ^ 2.
  local lock v_ref to -min(5, max(2, alt:radar / 10)).

  // Gravity feedforward plus a proportional correction. The 0.3 gain is
  // in units of 1/s: an error of 1 m/s adds 0.3 m/s^2 of commanded accel,
  // a ~3.3 s closed-loop time constant — brisk against the 10 s reference
  // profile, yet 5 m/s of error demands only 1.5 m/s^2 over the
  // feedforward, well inside TWR-2 authority. max() guards flameout.
  lock throttle to (g0 + 0.3 * (v_ref - verticalspeed)) * ship:mass
                   / max(0.001, ship:availablethrust).

  // Nose up, tipped slightly against any residual horizontal drift
  // (vxcl projects it out of the vertical). The 0.1 gain tips ~6 deg per
  // m/s of drift, capping how far the ship will lean this close to the
  // ground.
  lock steering to lookdirup(
    up:vector - 0.1 * vxcl(up:vector, ship:velocity:surface),
    ship:facing:topvector).

  gear on.

  // LANDED is the real signal; the fallback catches a hover balanced just
  // above contact — 5 m is landing-leg scale, -0.1 m/s is "effectively
  // stopped." The recorder runs here too: a_cmd is the ROD servo's demand
  // reconstructed along the commanded steering direction; t_go is not a
  // quantity this phase has, so it logs 0; aim_dist is live distance to
  // the site — the touchdown drift.
  local t_logged is 0.
  until ship:status = "LANDED"
      or (alt:radar < 5 and verticalspeed > -0.1) {
    if time:seconds - t_logged >= 1 {
      local dir is (up:vector - 0.1 * vxcl(up:vector, ship:velocity:surface)):normalized.
      log_state("TERMINAL", 0, tgt, tgt:terrainheight,
          (g0 + 0.3 * (v_ref - verticalspeed)) * dir).
      set t_logged to time:seconds.
    }
    wait 0.
  }
  lock throttle to 0.
  print "Contact. Settling.".
  wait 3.                          // settle on the legs before releasing control
  unlock steering.
  unlock throttle.
  set ship:control:pilotmainthrottle to 0.
}

// === MISSION SEQUENCE ===
// Each phase runs until its exit condition and returns; the gates are just
// parameter sets handed to the same guidance law.

perform_doi().             // DOI      (plan + burn)
coast_to_pdi().            // COAST
fly_gate(high_gate()).     // BRAKE    (P63)
fly_gate(low_gate()).      // APPROACH (P64)
terminal_descent().        // TERMINAL (P66)

// The headline number (test ladder, step 5): horizontal distance from the
// touchdown point to the target site.
local miss is vxcl(up:vector, tgt:position):mag.
print "Landed. Miss distance: " + round(miss) + " m.".
log "# landed  miss " + round(miss) + " m  dv_rem " + round(ship:deltav:current, 1)
    to flightlog.
