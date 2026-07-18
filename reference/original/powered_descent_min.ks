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

// Where the arc from the ship's current state at throttle f bottoms out:
// the gravity turn integrated by Euler's method until the speed is spent.
// Thrust, all of it retrograde, takes speed; gravity's along-path part
// adds speed back as the nose drops; its across-path part turns the path
// down at g*cos(pitch)/speed while the horizon rotates away under the
// ship at speed*cos(pitch)/r — the two rates whose difference is the turn.
function endpoint {
  parameter f.
  local speed is ship:velocity:surface:mag.
  local pitch is arcsin(min(1, max(-1, verticalspeed / speed))).
  local h is ship:altitude.
  local m is ship:mass.
  local theta is 0.                // ground angle swept, radians
  local t is 0.
  local steps is 0.
  local dt is (speed - speed_handoff) * m / (f * ship:availablethrust) / 150.
  until speed <= speed_handoff or steps >= 600 {
    local r_ is body:radius + h.
    local g is body:mu / r_ ^ 2.
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
    set t_logged to time:seconds.
  }
  wait 0.
}

// === TERMINAL DESCENT ===
// Reference descent rate from radar altitude (fall at alt/10, capped for
// continuity with the handoff, floored so touchdown happens); throttle
// servos onto it around the thrust that exactly cancels gravity. Steering
// hangs plumb, tipped against drift, walking the last metres to the site.
print "TERMINAL: from " + round(alt:radar) + " m.".
local g0 is body:mu / body:radius ^ 2.
local lock v_ref to -min(speed_handoff, max(2, alt:radar / 10)).
lock throttle to (g0 + 0.3 * (v_ref - verticalspeed)) * ship:mass
                 / max(0.001, ship:availablethrust).
function tilt {
  local off is vxcl(up:vector, tgt:position).
  local v_err is vxcl(up:vector, ship:velocity:surface)
               - off * min(0.2, 3 / max(0.001, off:mag)).
  return up:vector - 0.1 * v_err.
}
lock steering to lookdirup(tilt(), ship:facing:topvector).
gear on.
set t_logged to 0.
until ship:status = "LANDED"
    or (alt:radar < 5 and verticalspeed > -0.1) {
  if time:seconds - t_logged >= 1 {
    log_state("TERMINAL", 0, tgt, tgt:terrainheight,
        (g0 + 0.3 * (v_ref - verticalspeed)) * tilt():normalized).
    set t_logged to time:seconds.
  }
  wait 0.
}
lock throttle to 0.
wait 3.
unlock steering.
unlock throttle.
set ship:control:pilotmainthrottle to 0.
set config:ipu to ipu_prior.
local miss is vxcl(up:vector, tgt:position):mag.
print "Landed. Miss: " + round(miss) + " m.".
log "# landed  miss " + round(miss) + " m  dv_rem "
    + round(ship:deltav:current, 1) to flightlog.
