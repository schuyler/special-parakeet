// powered_descent_min.ks — the powered descent, reduced to its invariants.
//
// The answer to a question: how short can this program be and remain
// precise, efficient in the regime a well-placed PDI permits, and readable
// by someone with a modest grasp of calculus? About eighty lines of code
// that fly, plus the flight recorder — kept because the working agreement
// (no claim about flight behavior without telemetry) is part of the
// program, not part of the scaffolding: a spike that cannot be debugged
// is not shorter, only blinder. Everything powered_descent_live.ks
// carries beyond this file is envelope protection or coping with a plan
// that missed; this file assumes the plan is good. Design and the full
// argument: notes/powered-descent-invariants.md.
//
// Assumes (plan_doi.ks's contract): the DOI burn is behind us, PDI is the
// periapsis of the ellipse we are on, the corridor under the arc is
// certified, and the orbital plane passes near the site. Note what is
// absent because of that contract: landing_height appears nowhere below —
// the planner already spent it into the ellipse, so the arc that reaches
// the site bottoms out at the handoff height without this program ever
// knowing the number.
//
// Five ideas, one per section:
//   1. Hold thrust retrograde and the trajectory is a one-parameter
//      family: current state plus throttle determines the whole arc.
//   2. Euler's method draws the arc: rates times a small dt, summed.
//   3. The endpoint's reach falls as throttle rises, so bisection finds
//      the one throttle whose arc ends over the site. Re-solving every
//      few seconds from live state replaces plan, table, and trim.
//   4. A small lateral bias on the retrograde hold closes the plane onto
//      the site while the ship is fast, where a degree costs least.
//   5. Terminal descent is a rate servo around a gravity feedforward.

@lazyglobal off.

clearscreen.
print "=== POWERED DESCENT (MIN) ===".

run "common".              // engine_isp
run "../core/optimize".    // bisect

parameter target_lat is 0.
parameter target_lng is 0.
parameter speed_handoff is 5.    // the arc contract: must match plan_doi
parameter f_min is 0.05.
parameter f_max is 0.85.

local tgt is body:geopositionlatlng(target_lat, target_lng).
local mdot_full is ship:availablethrust / (engine_isp() * constant:g0).
local ipu_prior is config:ipu.
set config:ipu to 2000.

// Integration tolerances for endpoint's Euler steps — accuracy bounds, not
// craft or body numbers. pitch_tol caps the flight-path rotation per step
// (degrees); v_frac caps the fractional speed change per step. Either falsifies
// against a logged arc: too loose and the integrated reach drifts from the flown one.
local pitch_tol is 1.
local v_frac is 0.02.

// Where the arc from the ship's current state at throttle f bottoms out:
// the gravity turn integrated by Euler's method until the speed is spent.
// Thrust, all of it retrograde, takes speed; gravity's along-path part
// adds speed back as the nose drops; its across-path part turns the path
// down at g*cos(pitch)/speed while the horizon rotates away under the
// ship at speed*cos(pitch)/r — the two rates whose difference is the turn.
//
// The step is chosen inside the loop, not fixed: dt is the smaller of the
// time to rotate the flight path by pitch_tol and the time to change speed by
// v_frac of itself. Both track the dynamics, so the step refines where the
// path bends over and never depends on thrust — a weak engine no longer
// stretches the step until Euler's method diverges. The arc ends when speed
// is spent or when it reaches the ground (h <= tgt:terrainheight — the site's
// surface, not the datum); a throttle too weak to stop runs into the surface,
// and its reach there is a real undershoot, not garbage from integrating on
// below the ground. The step cap is a non-convergence guard.
function endpoint {
  parameter f.
  local speed is ship:velocity:surface:mag.
  local pitch is arcsin(min(1, max(-1, verticalspeed / speed))).
  local h is ship:altitude.
  local m is ship:mass.
  local theta is 0.                // ground angle swept, radians
  local t is 0.
  local steps is 0.
  until speed <= speed_handoff or h <= tgt:terrainheight or steps >= 4000 {
    local r_ is body:radius + h.
    local g is body:mu / r_ ^ 2.
    local turn is abs(speed / r_ - g / speed).
    local dt_angle is pitch_tol / (max(1e-6, turn) * constant:radtodeg).
    local dt_speed is v_frac * speed / (f * ship:availablethrust / m + g).
    local dt is min(dt_angle, dt_speed).
    local d_speed is (-(f * ship:availablethrust / m) - g * sin(pitch)) * dt.
    local d_pitch is (speed / r_ - g / speed) * cos(pitch)
                     * constant:radtodeg * dt.
    set h     to h     + speed * sin(pitch) * dt.
    set theta to theta + speed * cos(pitch) / r_ * dt.
    set speed to speed + d_speed.
    set pitch to pitch + d_pitch.
    set m     to m     - f * mdot_full * dt.
    set t     to t     + dt.
    set steps to steps + 1.
  }
  return lexicon("h", h, "x", theta * body:radius, "t", t).
}

// Great-circle ground distance to the site — the measure endpoint's x is in.
function dist_to_site {
  return body:radius * constant:degtorad
       * vang(ship:position - body:position, tgt:position - body:position).
}

// The one throttle whose arc ends over the site. Reach and distance are
// sampled in the same breath, so the miss is a property of the state and
// the throttle, steady while the ship flies on beneath the solve.
function solve_f {
  local miss is { parameter f. return endpoint(f)["x"] - dist_to_site(). }.
  return bisect(miss, f_min, f_max, 0.001).
}

// === FLIGHT RECORDER ===
// One CSV row per second from the powered phases, same columns as the
// sibling renditions so one analysis reads all three. Lines beginning '#'
// are the planning numbers the flight is judged against.
local flightlog is "flight_log.csv".

function log_state {
  parameter phase, t_go, aim_geo, aim_alt, a_thrust.
  parameter cross is 0.
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
      + round(90 - vang(up:vector, a_thrust), 1) + ","
      + round(cross)
      to flightlog.
}

// === COAST TO PDI ===
print "Coasting to PDI: " + round(eta:periapsis) + " s.".
warpto(time:seconds + eta:periapsis - 60).
wait until eta:periapsis <= 60.
lock steering to srfretrograde.
wait until eta:periapsis <= 1.

// === BRAKING ===
local f_cmd is solve_f().
if f_cmd < 0 { set f_cmd to f_max. }   // unreachable site: brake hard, land short
local t_go is endpoint(f_cmd)["t"].
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

// Retrograde, biased: pretend the ship owes a sideways speed of y/20
// toward the site's plane (y its offset, 20 s the closing time constant)
// and null that too. The bias fades as y closes; a few degrees of yaw
// spent while fast replaces a hover-and-translate at the bottom.
function braking_dir {
  local n is vcrs(ship:velocity:surface, up:vector):normalized.
  return -(ship:velocity:surface - n * vdot(tgt:position, n) / 20).
}
lock steering to lookdirup(braking_dir(), ship:facing:topvector).
lock throttle to f_cmd.

// Re-solve every 5 s while speed lasts. Below 6x the handoff speed the
// remaining arc is seconds long and metres-of-reach per unit of throttle
// have collapsed, so the last solution rides to the handoff; terminal
// owns the final metres.
local t_solved is time:seconds.
local t_logged is 0.
until ship:velocity:surface:mag <= speed_handoff {
  if ship:velocity:surface:mag > 6 * speed_handoff
      and time:seconds - t_solved >= 5 {
    local f is solve_f().
    if f > 0 { set f_cmd to f. set t_go to endpoint(f)["t"]. }
    set t_solved to time:seconds.
  }
  if time:seconds - t_logged >= 1 {
    local n is vcrs(ship:velocity:surface, up:vector):normalized.
    log_state("BRAKE", max(0, t_go - (time:seconds - t_solved)),
        tgt, tgt:terrainheight,
        f_cmd * (ship:availablethrust / ship:mass) * braking_dir():normalized,
        vdot(tgt:position, n)).
    // Terminal feedback on the solve: x is the held throttle's reach, d the
    // remaining ground distance to the site; their difference is the miss the
    // re-solve is nulling, watchable as the arc shortens and the ship closes.
    // Printed in place at a fixed row (trailing spaces overwrite a prior,
    // longer line) so the readout updates rather than scrolling the screen.
    local x is endpoint(f_cmd)["x"].
    local d is dist_to_site().
    print "BRK f=" + round(f_cmd, 3) + " x=" + round(x)
        + " d=" + round(d) + " miss=" + round(x - d)
        + " v=" + round(ship:velocity:surface:mag, 1) + "        "
        at (0, 10).
    set t_logged to time:seconds.
  }
  wait 0.
}

// === TERMINAL DESCENT ===
// Suicide burn with no servo gain in the vertical, plus a whisper of lateral
// steering through the fall. Below v_sched — the speed a brake at a_dec could
// still arrest before the pad, a_dec being f_max's deceleration net of gravity —
// the ship falls, but not dead-stick: a tipped thrust capped at a_lat_max — an
// acceleration, so authority is the same on any craft — nulls the horizontal
// drift the old free-fall let ride, cheap because it acts over the whole fall,
// bounded by tilt_max so the craft can always swing back to brake, and deadzoned
// below f_min so a centred ship idles the engine rather than burning a whisper it
// can't feel. At the crossing the throttle commands exactly a_req, the deceleration
// that carries the current speed to v_floor at the pad: (v^2 - v_floor^2)/2h.
// a_req equals a_dec at the crossing and rises into the f_max..1 reserve if the
// ship is behind; near the ground a_req falls below zero and the throttle drops
// under hover, settling the ship instead of bouncing it. The schedule fixes
// ignition, the kinematics fix the brake, tilt walks in the horizontal.
print "TERMINAL: from " + round(alt:radar) + " m.".
local g0 is body:mu / body:radius ^ 2.
local v_floor is 2.
local h_pad is 5.              // the burn spends its speed to v_floor by here; the last h_pad is a gentle coast
local a_lat_max is 0.3.        // cap on the free-fall lateral correction — an acceleration (m/s^2), so craft-free
local tilt_max is 30.          // cap on tilt from plumb, degrees — the attitude margin kept to still stick the burn
local lock a_dec to f_max * ship:availablethrust / ship:mass - g0.
local lock v_sched to sqrt(2 * a_dec * max(0, alt:radar - h_pad)).
local lock a_req to (verticalspeed ^ 2 - v_floor ^ 2) / (2 * max(1, alt:radar - h_pad)).

// Plumb, tipped against horizontal drift toward the site, but no further than
// tilt_max so a swing back to brake is always in reach.
function tilt {
  local off is vxcl(up:vector, tgt:position).
  local v_err is vxcl(up:vector, ship:velocity:surface)
               - off * min(0.2, 3 / max(0.001, off:mag)).
  local horiz is -0.1 * v_err.
  if horiz:mag > tan(tilt_max) { set horiz to horiz:normalized * tan(tilt_max). }
  return up:vector + horiz.
}
// How far the hold is tipped, 0 (plumb) to 1 (at tilt_max): the fraction of the
// lateral correction cap to spend, so a centred ship in free-fall asks for nothing.
local lock tilt_frac to min(1, tan(vang(up:vector, tilt())) / tan(tilt_max)).
// a_cmd is the commanded thrust acceleration: in free-fall the lateral correction,
// capped at a_lat_max; below the schedule the kinematic suicide brake. thr_raw is
// the throttle that delivers it — it rises to hold a_lat_max as TWR falls — and
// the free-fall throttle deadzones below f_min so a whisper too small to feel lets
// the ship truly fall instead of running the engine.
local lock in_ff to abs(verticalspeed) < v_sched.
local lock a_cmd to choose a_lat_max * tilt_frac if in_ff else g0 + a_req.
local lock thr_raw to a_cmd * ship:mass / max(0.001, ship:availablethrust).
lock throttle to choose 0 if in_ff and thr_raw < f_min else thr_raw.
lock steering to lookdirup(tilt(), ship:facing:topvector).
gear on.
set t_logged to 0.
until ship:status = "LANDED"
    or (alt:radar < 5 and verticalspeed > -0.1) {
  if time:seconds - t_logged >= 1 {
    // Log the thrust actually commanded (throttle, so the deadzone shows as
    // zero); max(0.001, ...) keeps the vector pointing up when it is off.
    log_state("TERMINAL", 0, tgt, tgt:terrainheight,
        max(0.001, throttle * ship:availablethrust / ship:mass) * tilt():normalized).
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
lock steering to up.        // hold the ship plumb while the legs settle
wait 3.
unlock steering.
unlock throttle.
set ship:control:pilotmainthrottle to 0.
sas on.
set config:ipu to ipu_prior.
local miss is vxcl(up:vector, tgt:position):mag.
print "Landed. Miss: " + round(miss) + " m.".
log "# landed  miss " + round(miss) + " m  dv_rem "
    + round(ship:deltav:current, 1) to flightlog.
