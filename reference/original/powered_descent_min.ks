// powered_descent_min.ks — the powered descent, reduced to its invariants.
//
// This file assumes the plan is good; everything powered_descent_live.ks
// carries beyond it is envelope protection or coping with a plan that missed.
// Design and the full argument: notes/powered-descent-invariants.md.
//
// Assumes (plan_doi.ks's contract): the DOI burn is behind us, PDI is the
// periapsis of the ellipse we are on, the corridor under the arc is
// certified, and the orbital plane passes near the site. landing_height
// appears nowhere below: the planner already spent it into the ellipse, so
// the arc reaches the seam near the site without this program ever knowing
// the number.
//
// Five ideas, one per section:
//   1. Hold thrust retrograde and the trajectory is a one-parameter
//      family: current state plus throttle determines the whole arc.
//   2. Euler's method draws the arc: rates times a small dt, summed.
//   3. The endpoint's reach falls as throttle rises, so bisection finds
//      the one throttle whose arc ends a handoff's stopping distance up-range
//      of the site. Re-solving every few seconds from live state replaces
//      plan, table, and trim.
//   4. A small lateral bias on the retrograde hold closes the plane onto
//      the site while the ship is fast, where a degree costs least.
//   5. Terminal is a suicide burn in the vertical and Klumpp's guidance in
//      the horizontal, arriving at the low gate — burn ignition — with offset
//      and drift both zero.

@lazyglobal off.

clearscreen.
print "=== POWERED DESCENT (MIN) ===".

run "common".              // engine_isp
run "../core/optimize".    // bisect

parameter target_lat is 0.
parameter target_lng is 0.
parameter f_max is 0.85.

local tgt is body:geopositionlatlng(target_lat, target_lng).
local mdot_full is ship:availablethrust / (engine_isp() * constant:g0).
local ipu_prior is config:ipu.
set config:ipu to 2000.

// Integration tolerances for endpoint's Euler steps — accuracy bounds, not
// craft or body numbers. pitch_tol caps the flight-path rotation per step
// (degrees); v_frac caps the fractional speed change per step.
local pitch_tol is 1.
local v_frac is 0.02.

// Descent geometry, shared by the braking solve and the terminal descent.
// g0 is surface gravity; tilt_max the attitude margin the craft always keeps
// so it can swing back to brake; a_lat_max the horizontal acceleration that
// margin buys at hover-scale thrust — g0*tan(tilt_max), craft- and body-free;
// a_eff the fraction of that cap the planning budgets, the rest being
// feedback reserve for tracking; h_pad the flare height.
local g0 is body:mu / body:radius ^ 2.
local tilt_max is 30.
local a_lat_max is g0 * tan(tilt_max).
local a_eff is 0.8 * a_lat_max.
local h_pad is 5.

// Where the arc from the ship's current state at throttle f reaches the seam:
// the gravity turn integrated by Euler's method until the flight path tips
// there. Thrust, all of it retrograde, takes speed; gravity's along-path part
// adds speed back as the nose drops; its across-path part turns the path down
// at g*cos(pitch)/speed while the horizon rotates away under the ship at
// speed*cos(pitch)/r — the two rates whose difference is the turn.
//
// dt is the smaller of the time to rotate the flight path by pitch_tol and
// the time to change speed by v_frac of itself, so the step refines where the
// path bends over; the step cap is a non-convergence guard. The arc ends at
// the seam — the flight path steeper than 90 - tilt_max below horizontal,
// where retrograde comes within tilt_max of plumb and braking hands to
// terminal — or at the ground (h <= tgt:terrainheight — the site's surface,
// not the datum), so the reach of a throttle too weak to make the seam reads
// as a real undershoot. vh is the horizontal speed at the seam: what terminal
// must null, and what the aim's stopping distance is built from.
function endpoint {
  parameter f.
  local speed is ship:velocity:surface:mag.
  local pitch is arcsin(min(1, max(-1, verticalspeed / speed))).
  local h is ship:altitude.
  local m is ship:mass.
  local theta is 0.                // ground angle swept, radians
  local t is 0.
  local steps is 0.
  local thrust is f * ship:availablethrust.
  until pitch <= tilt_max - 90 or h <= tgt:terrainheight or steps >= 4000 {
    local r_ is body:radius + h.
    local g is body:mu / r_ ^ 2.
    local a_thr is thrust / m.
    local turn_rate is speed / r_ - g / speed.
    local dt_angle is pitch_tol
                    / (max(1e-6, abs(turn_rate)) * constant:radtodeg).
    local dt_speed is v_frac * speed / (a_thr + g).
    local dt is min(dt_angle, dt_speed).
    local d_speed is (-a_thr - g * sin(pitch)) * dt.
    local d_pitch is turn_rate * cos(pitch) * constant:radtodeg * dt.
    set h     to h     + speed * sin(pitch) * dt.
    set theta to theta + speed * cos(pitch) / r_ * dt.
    set speed to speed + d_speed.
    set pitch to pitch + d_pitch.
    set m     to m     - f * mdot_full * dt.
    set t     to t     + dt.
    set steps to steps + 1.
  }
  return lexicon("x", theta * body:radius, "t", t, "vh", speed * cos(pitch)).
}

// Great-circle ground distance to the site — the measure endpoint's x is in.
function dist_to_site {
  return body:radius * constant:degtorad
       * vang(ship:position - body:position, tgt:position - body:position).
}

// The one throttle whose arc ends the handoff's stopping distance up-range of
// the site, so the residual horizontal speed coasts the craft in while
// terminal brakes it to rest over the pad — braking owes terminal a workable
// state, not a landing. The aim is reach + vh^2/(2*a_eff) == dist: the seam's
// ground reach plus where that seam speed would stop, decelerated at the
// budgeted a_eff, equals the distance to the site; priced at a_eff, not the
// cap, so terminal flies the stopping parabola with reserve. Bracketed at
// zero: a no-thrust arc runs into the terrain and reads as undershoot, so the
// ceiling is the only throttle bound the descent has.
function solve_f {
  local miss is { parameter f. local e is endpoint(f).
                  return e["x"] + e["vh"] ^ 2 / (2 * a_eff) - dist_to_site(). }.
  return bisect(miss, 0, f_max, 0.001).
}

// === FLIGHT RECORDER ===
// One CSV row per second from the powered phases, same columns as the
// sibling renditions so one analysis reads all three. Lines beginning '#'
// are the planning numbers the flight is judged against.
local flightlog is "flight_log.csv".

function log_state {
  parameter phase, t_go, a_thrust, cross.
  local to_site is vxcl(up:vector, tgt:position):normalized.
  log round(time:seconds, 1) + "," + phase + "," + round(t_go, 1) + ","
      + round(altitude) + "," + round(alt:radar) + ","
      + round(vdot(ship:velocity:surface, to_site), 1) + ","
      + round(verticalspeed, 1) + ","
      + round(tgt:position:mag) + ","
      + round(a_thrust:mag, 2) + "," + round(throttle, 3) + ","
      + round(vang(a_thrust, ship:facing:vector), 1) + ","
      + round(ship:mass, 3) + "," + round(ship:deltav:current, 1) + ","
      + round(90 - vang(up:vector, ship:facing:vector), 1) + ","
      + round(90 - vang(up:vector, a_thrust), 1) + ","
      + round(cross)
      to flightlog.
}

// === COAST TO PDI ===
print "Coasting to PDI: " + round(eta:periapsis) + " s.".
warpto(time:seconds + eta:periapsis - 60).
wait until eta:periapsis <= 60.
lock steering to srfretrograde.

// Seed the ignition throttle well before periapsis: the solve costs a few
// seconds, and run early it cannot straddle periapsis and leave the ignition
// wait below to miss its exit condition. The seed is stale — solved from the
// live state a coast early — but it only gives the engine a throttle to
// light at, t_go, and the readout its first solution; braking re-solves from
// the true periapsis state on its first pass.
local f_cmd is solve_f().
if f_cmd < 0 { set f_cmd to f_max. }   // unreachable site: brake hard, land short
local seam is endpoint(f_cmd).
local t_go is seam["t"].
// The plane-closing time constant: a third of the burn, frozen at ignition,
// so the plane closes early — while the ship is fast, where a degree of yaw
// costs least — leaving e^-3 (five percent) of the PDI offset at handoff.
// Frozen rather than tracking t_go so the shrinking horizon never demands a
// growing bias for whatever remains; t_go is stable across the fall, so the
// seed's value serves.
local tau_yaw is t_go / 3.

// === BRAKING ===
wait until eta:periapsis <= 1.
print "BRAKE: f " + round(f_cmd, 3) + ", "
    + round(dist_to_site() / 1000, 1) + " km to the site.".

if exists(flightlog) { deletepath(flightlog). }
log "# target " + round(target_lat, 4) + " " + round(target_lng, 4)
    + "  terrain " + round(tgt:terrainheight) + " m" to flightlog.
log "# h_pdi " + round(ship:altitude) + "  speed_pdi "
    + round(ship:velocity:surface:mag, 1) + "  f_ignition " + round(f_cmd, 4)
    + "  t_go " + round(t_go, 1) + "  dist " + round(dist_to_site())
    + "  dv_at_pdi " + round(ship:deltav:current, 1) to flightlog.
log "t,phase,t_go,alt,radar,v_to_site,v_vert,aim_dist,a_cmd,throttle,facing_err,mass,dv_rem,pitch,cmd_pitch,cross"
    to flightlog.

// Retrograde, biased: pretend the ship owes a sideways speed of
// y/tau_yaw toward the site's plane (y its offset) and null that too.
// The bias fades as y closes; a few degrees of yaw spent while fast
// replaces a hover-and-translate at the bottom.
function braking_dir {
  local n is vcrs(ship:velocity:surface, up:vector):normalized.
  return -(ship:velocity:surface - n * vdot(tgt:position, n) / tau_yaw).
}
lock steering to lookdirup(braking_dir(), ship:facing:topvector).
lock throttle to f_cmd.

// The exit is the attitude seam: hand off when retrograde has come within
// tilt_max of plumb. Past that the retrograde hold is a near-hover —
// terminal's deliberate job, not braking's incidental one — and the handoff
// is attitude-continuous, since at the seam the retrograde hold and
// terminal's max lean are the same vector. The residual horizontal speed at
// the seam is what solve_f aimed up-range for; terminal brakes it to rest
// over the fall.
//
// Re-solve every 5 s until within ~10 deg of the seam: that last stretch is
// seconds long and metres-of-reach per unit of throttle have collapsed, so
// the last solution rides to the handoff. The 10 deg is a soft cutoff on a
// spent control, not a tuned landing number.
//
// t_solved and t_logged start at 0 so the first pass re-solves and logs at
// once. x_solved and miss_solved are the readout's numbers — the last
// solution's reach and its gap against the site as the distances stood at
// that solve — seeded from the warp-out arc until the first pass replaces
// them.
local t_solved is 0.
local t_logged is 0.
local x_solved is seam["x"].
local miss_solved is x_solved - dist_to_site().
until false {
  local retro_ang is vang(up:vector, srfretrograde:vector).
  if retro_ang <= tilt_max { break. }
  if retro_ang > tilt_max + 10 and time:seconds - t_solved >= 5 {
    local f is solve_f().
    if f > 0 {
      set f_cmd to f.
      local e is endpoint(f).
      set t_go to e["t"].
      set x_solved to e["x"].
      set miss_solved to x_solved - dist_to_site().
    }
    set t_solved to time:seconds.
  }
  if time:seconds - t_logged >= 1 {
    local n is vcrs(ship:velocity:surface, up:vector):normalized.
    log_state("BRAKE", max(0, t_go - (time:seconds - t_solved)),
        f_cmd * (ship:availablethrust / ship:mass) * braking_dir():normalized,
        vdot(tgt:position, n)).
    // x is the solved arc's reach and miss its gap against the site, both
    // priced at the last re-solve — a march is worth one look per solution,
    // not one per second — while d is the live ground distance, closing
    // between solves. Printed in place at a fixed row (trailing spaces
    // overwrite a prior, longer line) so the readout updates rather than
    // scrolling the screen.
    local d is dist_to_site().
    print "BRK f=" + round(f_cmd, 3) + " x=" + round(x_solved)
        + " d=" + round(d) + " miss=" + round(miss_solved)
        + " v=" + round(ship:velocity:surface:mag, 1) + "        "
        at (0, 10).
    set t_logged to time:seconds.
  }
  wait 0.
}

// === TERMINAL DESCENT ===
// A suicide burn in the vertical, Klumpp's guidance in the horizontal. Below
// v_sched — the speed a brake at a_dec could still arrest before the pad, a_dec
// being f_max's deceleration net of gravity — the ship falls; the horizontal
// flies the Apollo two-boundary law to the low gate, suicide-burn ignition,
// arriving there with offset and drift both zero, so the burn ignites plumb
// over the pad and owns the vertical alone. At the crossing the throttle
// commands exactly a_req, the deceleration that carries the current speed to
// v_floor at the pad: (v^2 - v_floor^2)/2h. a_req equals a_dec at the crossing
// and rises into the f_max..1 reserve if the ship is behind; near the ground
// a_req falls below zero and the throttle drops under hover, settling the ship
// instead of bouncing it. The schedule fixes ignition, the kinematics fix the
// brake, and the guidance spends the fall arriving.
print "TERMINAL: from " + round(alt:radar) + " m.".
local v_floor is 2.                // touchdown descent rate
local lock a_dec to f_max * ship:availablethrust / ship:mass - g0.
local lock v_sched to sqrt(2 * a_dec * max(0, alt:radar - h_pad)).
local lock a_req to (verticalspeed ^ 2 - v_floor ^ 2) / (2 * max(1, alt:radar - h_pad)).

// Time to the low gate — suicide-burn ignition. The gate altitude x is where
// free fall meets the ignition schedule, vv^2 + 2 g0 (h - x) = v_sched(x)^2,
// and the fall time to x follows from the same kinematics. Floored at t_settle:
// inside the fence the gate is effectively now, and the guidance must not
// divide by a vanishing horizon.
function t_gate {
  local vv is verticalspeed.
  local h is alt:radar.
  local x is (vv ^ 2 + 2 * (g0 * h + a_dec * h_pad)) / (2 * (g0 + a_dec)).
  local v_x is sqrt(vv ^ 2 + 2 * g0 * max(0, h - x)).
  return max(t_settle, (v_x + vv) / g0).
}
// The commanded horizontal acceleration: Klumpp's guidance (Apollo P63/P64,
// two-boundary form), horizontal components only. ZEM is the offset a pure
// coast would show at the gate; ZEV the drift to shed (the target velocity is
// zero — straight down). a = 6 ZEM/t^2 - 2 ZEV/t is the fuel-optimal
// linear-acceleration profile carrying both to zero at the gate: no gains, no
// schedule, t_gate scales everything. It saturates the cap only when the
// handoff was marginal; the cap is the attitude margin, as everywhere.
function a_lat_cmd {
  local t is t_gate().
  local off is vxcl(up:vector, tgt:position).
  local vh is vxcl(up:vector, ship:velocity:surface).
  local a is (off - vh * t) * (6 / t ^ 2) + vh * (2 / t).
  if a:mag > a_lat_max { return a:normalized * a_lat_max. }
  return a.
}
local lock in_ff to abs(verticalspeed) < v_sched.
// The commanded thrust vector. In free-fall the lean scales with the command —
// tan(lean) = sqrt(a_lat/a_lat_max) * tan(tilt_max), the geometric-mean
// shaping — so tilt_max is a true maximum, reached only at a saturated
// command, and a whisper costs a nod instead of a 30-degree slew. The
// vertical component, sqrt(a_lat * a_lat_max)/tan(tilt_max), is <= g0 with
// equality only at the cap, so the ship keeps falling and never climbs while
// the horizontal corrects. At the crossing it is the suicide brake g0+a_req
// up plus the same horizontal command, within tilt_max on its own since
// a_lat <= a_lat_max. A function rather than a lock: the command is drawn
// once and reused, where a lock naming a_lat twice would run the guidance
// law twice per look.
function thrust_vec {
  local a_lat is a_lat_cmd().
  if in_ff {
    return a_lat + up:vector * (sqrt(a_lat:mag * a_lat_max) / tan(tilt_max)).
  }
  return a_lat + up:vector * (g0 + a_req).
}
local lock thr_raw to thrust_vec():mag * ship:mass
                    / max(0.001, ship:availablethrust).
// The plumb fence, the one latch on the lateral law. The burn must ignite
// plumb — retrograde of a near-vertical fall — because a lean at ignition
// converts full throttle into sideways drift, and the law delivers zero
// offset and drift at the gate only while its command is under the cap: a
// saturated handoff arrives at the gate still leaning. t_ign underestimates
// the time to the schedule crossing by taking the gap at its fastest
// possible closure, g0 + a_dec; when it is inside t_settle — the time a
// tilt_max swing takes at steering-manager rates — burn_near latches, the
// lateral game is over for good, and the ship swings plumb and waits. The
// last metres belong to the burn's own trim, delivered aligned. The latch
// lives in the loop.
local t_settle is 3.
local burn_near is false.
// The commanded attitude is thrust_vec until the fence latches, plumb from
// the fence to ignition; ignition ends in_ff, and the attitude follows
// thrust_vec again — now the burn's own command. The magnitude floor is a
// degeneracy guard, not a deadband: below 0.005 m/s^2 the command's
// direction is within a tenth of a degree of up, and at exactly zero it has
// none, so plumb is the direction the vanishing vector was already naming.
// The same vanishing vector reads as unaligned to the gate below, so the
// engine is off wherever this guard is steering.
local lock plumb to (in_ff and burn_near) or thrust_vec():mag < 0.005.
// Thrust only when the nose is on the command: fired misaligned by an angle
// e, a correction delivers sin(e) of itself sideways — new drift manufactured
// from old — and an engine that burns through its own slews pumps the very
// error it is nulling. Gated at face_tol the engine waits out each slew: 15
// degrees keeps the sideways injection under a quarter of the correction, so
// corrections strictly shrink the error. The suicide burn is never gated —
// misaligned vertical braking still brakes, and cutting it near the ground
// costs more than its lean error.
local face_tol is 15.
local lock aligned to vang(ship:facing:vector, thrust_vec()) <= face_tol.
// The handoff assertion: guidance that ignites already saturated cannot
// promise the gate. A warning, not an abort.
if a_lat_cmd():mag >= a_lat_max {
  print "WARN: handoff marginal, lateral guidance saturated.".
}
lock throttle to choose 0 if in_ff and (burn_near or not aligned) else thr_raw.
lock steering to lookdirup((choose up:vector if plumb else thrust_vec()),
                           ship:facing:topvector).
gear on.
set t_logged to 0.
until ship:status = "LANDED"
    or (alt:radar < h_pad and verticalspeed > -0.1) {
  if not burn_near
      and (v_sched - abs(verticalspeed)) / (g0 + a_dec) < t_settle {
    set burn_near to true.
  }
  if time:seconds - t_logged >= 1 {
    // Log the thrust actually commanded (throttle, so the gated engine shows
    // as zero); max(0.001, ...) keeps the vector pointing up when it is off.
    // The cross column carries the full horizontal drift speed — v_to_site
    // is blind to the tangential component.
    log_state("TERMINAL", 0,
        max(0.001, throttle * ship:availablethrust / ship:mass)
        * (choose up:vector if plumb else thrust_vec():normalized),
        vxcl(up:vector, ship:velocity:surface):mag).
    // Fixed-row readout in BRK's idiom: f the throttle, v the descent rate,
    // miss the horizontal offset that becomes the landing error. sched is the
    // speed that gates ignition (the burn lights when v reaches it); drift is
    // the horizontal speed the burn cannot correct until it is lit.
    print "TRM r=" + round(alt:radar) + " v=" + round(verticalspeed, 1)
        + " sched=" + round(v_sched) + " f=" + round(throttle, 3)
        + " miss=" + round(vxcl(up:vector, tgt:position):mag)
        + " drift=" + round(vxcl(up:vector, ship:velocity:surface):mag, 1)
        + "     " at (0, 10).
    set t_logged to time:seconds.
  }
  wait 0.
}
lock throttle to 0.
// Hold the ship plumb while the legs settle. lookdirup with the current
// topvector asks only for the nose: bare `up` is a full direction, roll
// included, and the steering manager would grind the landed ship around
// its legs to satisfy a roll nothing needs.
lock steering to lookdirup(up:vector, ship:facing:topvector).
// Hand off to SAS only once the craft has stopped moving: below ~1 deg/s of
// rotation the legs have stopped rocking. The clock is a hung-wait guard —
// rocking on a slope may never settle below the threshold.
local t_land is time:seconds.
wait until ship:angularvel:mag < 0.02 or time:seconds - t_land > 10.
unlock steering.
unlock throttle.
set ship:control:pilotmainthrottle to 0.
sas on.
set config:ipu to ipu_prior.
local miss is vxcl(up:vector, tgt:position):mag.
print "Landed. Miss: " + round(miss) + " m.".
log "# landed  miss " + round(miss) + " m  dv_rem "
    + round(ship:deltav:current, 1) to flightlog.
