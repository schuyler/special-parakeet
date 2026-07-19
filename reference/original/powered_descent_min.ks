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
// the planner already spent it into the ellipse, so the arc reaches the seam near
// the site without this program ever knowing the number.
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
// (degrees); v_frac caps the fractional speed change per step. Either falsifies
// against a logged arc: too loose and the integrated reach drifts from the flown one.
local pitch_tol is 1.
local v_frac is 0.02.

// Descent geometry, hoisted here because the braking solve now needs it: the arc
// ends at an attitude (endpoint, below) and aims up-range by the handoff's stopping
// distance (solve_f, below). g0 is surface gravity; tilt_max the attitude margin the
// craft always keeps so it can swing back to brake; a_lat_max the horizontal
// correction that margin buys at hover-scale thrust — g0*tan(tilt_max), craft- and
// body-free; a_eff the fraction of that cap the planning budgets, so the rest of
// the cap is feedback reserve for tracking — the argument that holds f_max under 1,
// derating lean geometry rather than thrust; h_pad the flare height. The terminal
// section spends the last four.
local g0 is body:mu / body:radius ^ 2.
local tilt_max is 30.
local a_lat_max is g0 * tan(tilt_max).
local a_eff is 0.8 * a_lat_max.
local h_pad is 5.

// Where the arc from the ship's current state at throttle f reaches the seam:
// the gravity turn integrated by Euler's method until the flight path tips there.
// Thrust, all of it retrograde, takes speed; gravity's along-path part
// adds speed back as the nose drops; its across-path part turns the path
// down at g*cos(pitch)/speed while the horizon rotates away under the
// ship at speed*cos(pitch)/r — the two rates whose difference is the turn.
//
// The step is chosen inside the loop, not fixed: dt is the smaller of the
// time to rotate the flight path by pitch_tol and the time to change speed by
// v_frac of itself. Both track the dynamics, so the step refines where the
// path bends over and never depends on thrust — a weak engine no longer
// stretches the step until Euler's method diverges. The arc ends at the seam —
// the flight path steeper than 90 - tilt_max below horizontal, where retrograde
// comes within tilt_max of plumb and braking hands to terminal — or at the ground
// (h <= tgt:terrainheight — the site's surface, not the datum); a throttle too
// weak to reach the seam runs into the surface, and its reach there is a real
// undershoot, not garbage from integrating on below the ground. The step cap is a non-convergence guard. vh is the horizontal speed at
// the seam: what terminal must null, and what the aim's stopping distance is built from.
function endpoint {
  parameter f.
  local speed is ship:velocity:surface:mag.
  local pitch is arcsin(min(1, max(-1, verticalspeed / speed))).
  local h is ship:altitude.
  local m is ship:mass.
  local theta is 0.                // ground angle swept, radians
  local t is 0.
  local steps is 0.
  until pitch <= tilt_max - 90 or h <= tgt:terrainheight or steps >= 4000 {
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
  return lexicon("h", h, "x", theta * body:radius, "t", t, "vh", speed * cos(pitch)).
}

// Great-circle ground distance to the site — the measure endpoint's x is in.
function dist_to_site {
  return body:radius * constant:degtorad
       * vang(ship:position - body:position, tgt:position - body:position).
}

// The one throttle whose arc ends the handoff's stopping distance up-range of the
// site, so the residual horizontal speed coasts the craft in while terminal brakes
// it to rest over the pad — braking owes terminal a workable state, not a landing.
// The aim is reach + vh^2/(2*a_eff) == dist: the seam's ground reach plus where
// that seam speed would stop, decelerated at the budgeted a_eff, equals the
// distance to the site. Priced at a_eff, not the cap, so terminal can fly the
// stopping parabola with reserve — the same number gates both ends of the seam.
// Bracketed at zero: a no-thrust arc runs into the terrain floor and its reach
// there is a real undershoot, so the bottom end needs no tuned floor — the ceiling
// is the only throttle bound the descent has.
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

// Seed the ignition throttle now, well before periapsis. The solve costs a few
// seconds; run this early it finishes with time to spare, so it can't straddle
// periapsis and leave the ignition wait below to miss its exit condition. The
// seed is stale — solve_f reads the live state, well before actual periapsis —
// but it only has to give the engine a throttle to light at: braking re-solves
// from the true periapsis state on its first pass.
local f_cmd is solve_f().
if f_cmd < 0 { set f_cmd to f_max. }   // unreachable site: brake hard, land short
local seam is endpoint(f_cmd).
local t_go is seam["t"].
// The seed is spent: it seeded the ignition throttle and t_go, nothing else. The
// terminal workability check and trim gain once lived here, derived from this
// seam estimate — and the estimate poisoned them: integrated 60 s early, the arc
// burns through what the craft actually coasts, descends low enough to take
// endpoint's ground exit, and hands the gain a zero fall. Both now read the live
// state at the handoff itself, where no estimate is needed.
// The plane-closing time constant: a third of the burn, frozen at
// ignition, so the plane closes early — while the ship is fast, where a
// degree of yaw costs least — on any craft, leaving e^-3 (five percent)
// of the PDI offset at handoff. Frozen rather than tracking t_go so the
// shrinking horizon never demands a growing bias for whatever remains.
// Frozen from the seed, a coast early, is safe: t_go is stable across the fall.
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

// The exit is the attitude seam: hand off when retrograde has come within tilt_max
// of plumb. Past that the retrograde hold is a near-hover — terminal's deliberate
// job, not braking's incidental one — and the handoff is attitude-continuous, since
// at the seam the retrograde hold and terminal's max lean are the same vector. The
// residual horizontal speed at the seam is what solve_f aimed up-range for;
// terminal brakes it to rest over the fall.
//
// Re-solve every 5 s until within ~10 deg of the seam. That last stretch is seconds
// long and metres-of-reach per unit of throttle have collapsed, so the solve can no
// longer move the aim and the last solution rides to the handoff. The 10 deg is a
// soft cutoff on a spent control, not a tuned landing number.
//
// t_solved starts at 0 so the first pass re-solves at once, replacing the stale
// warp-out seed with the true periapsis solution — the engine is already lit on
// the seed, so this refresh costs nothing on the way in. (t_logged does the same
// to log the first row immediately.)
local t_solved is 0.
local t_logged is 0.
until vang(up:vector, srfretrograde:vector) <= tilt_max {
  if vang(up:vector, srfretrograde:vector) > tilt_max + 10
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
// g0, tilt_max, a_lat_max, h_pad are hoisted above the solver (a_eff is the
// braking aim's budget and terminal no longer touches it); v_floor is the
// touchdown floor.
local v_floor is 2.
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
  local v_x is sqrt(max(0, vv ^ 2 + 2 * g0 * max(0, h - x))).
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
local lock a_lat to a_lat_cmd().
local lock in_ff to abs(verticalspeed) < v_sched.
// The commanded thrust vector. In free-fall the lean scales with the command —
// tan(lean) = sqrt(a_lat/a_lat_max) * tan(tilt_max), the geometric-mean shaping
// — so tilt_max is a true maximum, reached only at a saturated command, and a
// whisper costs a nod instead of a 30-degree slew. The vertical component,
// sqrt(a_lat * a_lat_max)/tan(tilt_max), is <= g0 with equality only at the
// cap, so the ship keeps falling and never climbs while the horizontal
// corrects — the same invariant the old fixed lean enforced, now smooth to
// plumb. At the crossing it is the suicide brake g0+a_req up plus the same
// horizontal command, within tilt_max on its own since a_lat <= a_lat_max.
local lock thrust_vec to choose
     a_lat + up:vector * (sqrt(a_lat:mag * a_lat_max) / tan(tilt_max)) if in_ff
     else a_lat + up:vector * (g0 + a_req).
local lock thr_raw to thrust_vec:mag * ship:mass / max(0.001, ship:availablethrust).
// The predicted gate offset — Klumpp's ZEM, as a magnitude: where a pure coast
// puts the ship, horizontally, relative to the site at burn ignition. The burn
// is vertical, so this is also the touchdown miss a coast accepts.
function miss_pred {
  return (vxcl(up:vector, tgt:position)
        - vxcl(up:vector, ship:velocity:surface) * t_gate()):mag.
}
// The lateral law engages and disengages on consequence, never on error:
// corr_on latches when miss_pred exceeds h_pad — coasting would miss by more
// than the flare radius — and unlatches the moment miss_pred is back inside it.
// Parked, miss_pred is frozen (nothing accelerates the ship), so neither edge
// chatters. The unlatch matters as much as the latch: a correction that rode
// its lean into the ungated suicide burn threw 5 m/s of drift in the burn's
// first second, twice — so the lateral game must end, plumb, well above the
// burn, accepting the same h_pad of remainder the vertical flare accepts.
// The burn must ignite plumb — retrograde of a near-vertical fall — because a
// lean at ignition converts full throttle into sideways drift (two flights,
// 17 m each, all of it thrown in the burn's first second). t_ign underestimates
// the time to the schedule crossing by taking the gap at its fastest possible
// closure, g0 + a_dec; when it is inside t_settle, burn_near latches, the
// lateral game is over for good, and the ship swings plumb and waits. The last
// metres belong to the burn's own trim, delivered aligned. t_settle is the time
// a tilt_max swing takes at steering-manager rates. The state machine lives in
// the loop.
local t_settle is 3.
local burn_near is false.
local corr_on is false.
local lock centred to in_ff and not corr_on.
// Thrust only when the nose is on the command: fired misaligned by an angle e, a
// correction delivers sin(e) of itself sideways — new drift manufactured from old.
// Ungated, the engine burns through the handoff slew and every swing after it,
// pumping the very error it is nulling; the flights showed the closed loop, a
// rotating limit cycle circling the pad at saturated command. Gated at face_tol
// the engine waits out each slew: 15 degrees keeps the sideways injection under a
// quarter of the correction, so corrections strictly shrink the error. The
// suicide burn is never gated — misaligned vertical braking still brakes, and
// cutting it near the ground costs more than its lean error.
local face_tol is 15.
local lock aligned to vang(ship:facing:vector, thrust_vec) <= face_tol.
// The handoff assertion, measured where it is measurable: guidance that
// ignites already saturated cannot promise the gate. Warns like plan_doi's
// checks; it does not abort.
if a_lat_cmd():mag >= a_lat_max {
  print "WARN: handoff marginal, lateral guidance saturated.".
}
lock throttle to choose 0 if centred or (in_ff and not aligned) else thr_raw.
lock steering to lookdirup((choose up:vector if centred else thrust_vec),
                           ship:facing:topvector).
gear on.
set t_logged to 0.
until ship:status = "LANDED"
    or (alt:radar < h_pad and verticalspeed > -0.1) {
  if not burn_near and in_ff
      and (v_sched - abs(verticalspeed)) / (g0 + a_dec) < t_settle {
    set burn_near to true.
    set corr_on to false.
  }
  if corr_on and miss_pred() <= h_pad / 2 { set corr_on to false. }
  else if not corr_on and not burn_near and in_ff and miss_pred() > h_pad {
    set corr_on to true.
  }
  if time:seconds - t_logged >= 1 {
    // Log the thrust actually commanded (throttle, so the deadzone shows as
    // zero); max(0.001, ...) keeps the vector pointing up when it is off. The
    // cross column carries the full horizontal drift speed here — v_to_site is
    // blind to the tangential component, and the tangential component is what
    // the circling failure lived in.
    log_state("TERMINAL", 0, tgt, tgt:terrainheight,
        max(0.001, throttle * ship:availablethrust / ship:mass)
        * (choose up:vector if centred else thrust_vec:normalized),
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
local t_settle is time:seconds.
wait until ship:angularvel:mag < 0.02 or time:seconds - t_settle > 10.
unlock steering.
unlock throttle.
set ship:control:pilotmainthrottle to 0.
sas on.
set config:ipu to ipu_prior.
local miss is vxcl(up:vector, tgt:position):mag.
print "Landed. Miss: " + round(miss) + " m.".
log "# landed  miss " + round(miss) + " m  dv_rem "
    + round(ship:deltav:current, 1) to flightlog.
