// powered_descent.ks — the powered descent, from PDI to touchdown.
// Design: notes/capability-driven-descent.md (piece 1), with the quadratic
// t_go closure from notes/klumpp-guidance-derivation.md (§4b/§5).
//
// Begins work on a descent ellipse someone else planned: the DOI burn is
// already behind us, and periapsis is PDI. So the PDI altitude is not a
// parameter here, it is read off the orbit. Nothing in this file plans a
// burn, places a node, or looks at terrain up-range; that is the planner's
// job (pieces 2 and 3), and the node it left is the whole interface.
//
// Deliberately bare-bones: the reference implementation of the guidance
// design, not an operational autopilot. Steps a more complex mission would
// need are marked "OMITTED:" at the point they would go.

@lazyglobal off.

clearscreen.
print "=== POWERED DESCENT ===".

run "../core/optimize".    // bisect

parameter target_lat is 0.
parameter target_lng is 0.
// The arc ends this far above the site's terrain and this slow. Below and
// after, the terminal rate controller flies. Every metre of it is spent in a
// near-hover, so it is gravity loss; the floor under it is terminal's room to
// flare, null drift, and stay clear of the guidance law's divergent gains.
parameter landing_height is 50.
parameter speed_handoff is 5.
// Degrees of flight-path rotation per leg. The law flies each leg as a chord
// across the arc, and the chord's error is the sagitta of a circle of radius
// a_thrust: a_thrust * (1 - cos(turn_budget/2)). That bracket is also the
// fraction of thrust not pointing retrograde, which is what the chord costs:
// 0.9% at 15 degrees, 3.4% at 30. Every gravity turn sweeps the same 90
// degrees from level at PDI to vertical, so this fixes the leg count at six
// for any craft on any body.
parameter turn_budget is 15.
// Ceiling on the design throttle, as a fraction of full. The reserve (1 - f)
// is the authority the closed-loop law spends absorbing error in flight, so
// this is a constraint on the solve below rather than a value it aims for:
// solving under the ceiling only leaves more reserve, never less.
parameter f_max is 0.85.
parameter f_min is 0.05.
parameter dt_arc is 0.5.        // arc integration step, s
// How close the throttle solve has to get. Tighter than it looks like it needs
// to be, because the handoff height is violently sensitive to it: measured,
// 0.001 of throttle moves where the arc bottoms out by about 120 m. At a few
// percent throttle the burn runs minutes, and a small change compounds the
// whole way down. So this is worth ~12 m of handoff height, which is the
// resolution actually wanted. The solve also runs at dt_arc, the step the
// flown arc uses: a coarser step is four times faster but answers a question
// about a different trajectory, and that difference lands on the same number.
parameter f_epsilon is 0.0001.
parameter max_steps is 2000.    // arc step budget

local tgt is body:geopositionlatlng(target_lat, target_lng).
local a_max is ship:availablethrust / ship:mass.

// OMITTED: Phase 0 (plane alignment). We assume the descent ellipse already
// passes over the site; the planner owns that.

// === THE ARC ===
// The fuel-optimal airless descent is a gravity turn flown in reverse: hold
// thrust surface-retrograde and let gravity rotate the velocity vector from
// horizontal to straight down as the burn bleeds off speed, arriving vertical
// over the target. Retrograde is the minimum-Delta-v direction to null a
// velocity vector; it cancels the vertical component while the craft is still
// fast and centrifugal support makes vertical cheap; and it spends the least
// time slow, where gravity loss accrues fastest.
//
// Euler's method: hold the state in four numbers, compute how much each
// changes over a short interval dt, then add the changes on and repeat. Small
// enough steps and the sum of the straight hops traces the curve. Returns one
// sample per step, which gives the drop, the downrange, the duration, and the
// state at every point on the arc -- which is what the legs are read from.
function integrate_arc {
  parameter h_start.      // PDI altitude above the datum, m
  parameter speed_pdi.    // surface speed at PDI, m/s
  parameter a_thrust.     // retrograde thrust accel, m/s^2 = f * a_max
  parameter dt.           // integration step, s
  // Speed at which the arc ends, m/s. d_pitch carries speed in its
  // denominator, so the turn rate climbs steeply as speed falls; stopping
  // above zero keeps it finite and hands the rest of the descent to TERMINAL.
  parameter speed_low.
  // Cap on the number of steps. An arc that spends them all with speed still
  // above speed_low needs more thrust than this craft has.
  parameter max_steps_.
  // The throttle solve wants one number -- where the arc bottoms out -- and
  // runs this hundreds of times. Recording every step for it would build and
  // discard thousands of lexicons, so it asks for the endpoint alone. The
  // physics is the same either way, which is the point of the flag: two
  // steppers would be free to drift apart, and the solve would then optimise
  // an arc the ship never flies.
  parameter record is true.

  local speed is speed_pdi.
  local pitch is 0.       // degrees above the horizon; PDI is a periapsis
  local h is h_start.
  // Angle swept around the body's centre since PDI, radians. Radians because
  // theta * body:radius is then the distance travelled over the ground, which
  // is what the legs are placed along.
  local theta is 0.
  local t is 0.
  local steps is 0.
  local arc is list().

  until speed <= speed_low or steps >= max_steps_ {
    // r_ trails an underscore because kOS reserves R() for rotations, the same
    // way solve_t_go spells its velocity "vel" to stay clear of V().
    local r_ is body:radius + h.
    local g is body:mu / r_ ^ 2.

    // Recorded before anything steps, so the first sample is PDI's own state.
    if record {
      arc:add(lexicon("t", t, "speed", speed, "pitch", pitch, "h", h,
                      "x", theta * body:radius)).
    }

    // Every increment reads the state as it stands; they all apply together
    // at the bottom.
    //
    // Thrust points straight back along the velocity, so all of it changes
    // speed. Gravity splits: the part along the path adds speed as the nose
    // drops below the horizon, and the part across the path steers, on the
    // next line.
    local d_speed is (-a_thrust - g * sin(pitch)) * dt.
    // Two rates turn pitch. The local horizon rotates under the ship at
    // speed*cos(pitch)/r as it flies around the body, tipping the nose up.
    // Gravity pulls across the velocity at g*cos(pitch)/speed, tipping it
    // down. The two match at orbital speed and pitch holds. Below it gravity
    // wins and the path steepens toward vertical; above it the horizon wins
    // and the arc climbs. radtodeg converts the rate to degrees because kOS
    // trigonometry works in degrees.
    local d_pitch is (speed / r_ - g / speed) * cos(pitch)
                     * constant:radtodeg * dt.
    // The vertical and horizontal halves of the speed.
    local d_h is speed * sin(pitch) * dt.
    local d_theta is speed * cos(pitch) / r_ * dt.

    set speed to speed + d_speed.
    set pitch to pitch + d_pitch.
    set h     to h     + d_h.
    set theta to theta + d_theta.
    set t     to t     + dt.
    set steps to steps + 1.
  }

  // The loop samples at the top and steps at the bottom, so it always exits
  // holding a state it never recorded. Append it: that state is the handoff
  // the last leg aims at, and it is the only sample whose speed is at or below
  // speed_low -- which is what tells a closed arc from one that ran out of
  // steps with speed still to burn.
  arc:add(lexicon("t", t, "speed", speed, "pitch", pitch, "h", h,
                  "x", theta * body:radius)).
  return arc.
}

// The arc's total fall, from PDI to wherever the speed ran out.
function arc_drop {
  parameter arc.
  parameter h_start.
  return h_start - arc[arc:length - 1]["h"].
}

// === SOLVING THE THROTTLE ===
// The arc has to end at the handoff height. PDI's altitude is already fixed --
// it is the periapsis of the ellipse we are on, and nothing here can move it --
// so the only thing left to vary is how hard the engine pushes. Less throttle
// means a longer burn, which gives gravity longer to pull the path down, which
// deepens the fall; so the fall shrinks as the throttle grows, and there is one
// throttle that makes it come out exactly right. Bisection finds it.
//
// This is why the throttle is not a parameter here. The planner chooses the
// approach it wants and expresses that choice as PDI's altitude; by the time
// this script runs, that choice has already been made, and the throttle is
// whatever flies the arc it implies.
function solve_throttle {
  parameter h_pdi.
  parameter speed_pdi.
  parameter drop_wanted.   // h_pdi down to the handoff height, m

  local miss is {
    parameter f.
    local arc is integrate_arc(h_pdi, speed_pdi, f * a_max, dt_arc,
                               speed_handoff, max_steps, false).
    return arc_drop(arc, h_pdi) - drop_wanted.
  }.
  // Bisection needs the answer bracketed, and it is: the fall shrinks as the
  // throttle grows, so f_min overshoots the ground and f_max cannot reach it.
  // Returns -1 if that bracket does not hold, which is the abort.
  return bisect(miss, f_min, f_max, f_epsilon).
}

// === LEGS ===
// A leg is the target state for one stretch of the arc, handed to the guidance
// law as a lexicon. The law flies a chord between the endpoints, so a leg ends
// once the flight path has rotated turn_budget degrees -- see the parameter's
// comment for where that number comes from.
//
// The aim points are anchored to the SITE, not to PDI: a leg's aim sits
// (X - x) metres up-range of the target, where X is the arc's whole ground
// track. So the last leg's aim is the site itself, whatever the DOI burn
// actually achieved. Any error in PDI's placement shows up as the ship not
// being quite where the arc's first sample says it is, which is error for the
// law to absorb rather than a target to miss.
function tessellate {
  parameter arc.

  local legs is list().
  local x_total is arc[arc:length - 1]["x"].
  // Direction from the site back along the ground track, toward the ship.
  local u_site is (tgt:position - body:position):normalized.
  local up_range is vxcl(u_site, -tgt:position):normalized.

  local pitch_leg is arc[0]["pitch"].
  local i is 1.
  until i >= arc:length {
    local s is arc[i].
    local last is i = arc:length - 1.
    if abs(s["pitch"] - pitch_leg) >= turn_budget or last {
      // The vertical speed the arc has at this sample and at the next one give
      // the vertical acceleration it is under. That is what the closure below
      // needs: the acceleration the ship should ALREADY have when it arrives,
      // read off the ideal path rather than picked. Mid-arc it is negative --
      // the ship is legitimately gaining descent rate there.
      local v_vert is s["speed"] * sin(s["pitch"]).
      local a_vert is 0.
      if i > 0 {
        local p is arc[i - 1].
        set a_vert to (v_vert - p["speed"] * sin(p["pitch"])) / dt_arc.
      }

      local aim_geo is body:geopositionof(
          tgt:position + (x_total - s["x"]) * up_range).
      local aim_alt is s["h"].

      local closure is {
        parameter v_tgt.
        return solve_t_go(aim_geo, aim_alt, v_tgt, a_vert).
      }.

      legs:add(lexicon(
        "name",      "LEG" + (legs:length + 1),
        "aim_geo",   aim_geo,
        "aim_alt",   aim_alt,
        "v_horiz",   s["speed"] * cos(s["pitch"]),   // toward the site
        "v_vert",    v_vert,                          // up-positive, so negative here
        "t_plan",    s["t"],                          // the arc's own clock, for comparison
        "t_handoff", 5,
        // Ground-proximity invariant: half the leg's own aim height above the
        // site. There is no legitimate reason to be that low with the leg's
        // aim point still ahead, so crossing it means guidance has diverged.
        // It scales with the leg because the last one legitimately arrives
        // near the ground -- a floor fixed to the descent as a whole would sit
        // above the very target it was meant to protect.
        "alt_floor", (aim_alt - tgt:terrainheight) / 2,
        "closure",   closure)).
      set pitch_leg to s["pitch"].
    }
    set i to i + 1.
  }
  return legs.
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
  local vel is ship:velocity:surface.

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

// === ABORTS ===

// Halt-and-hand-over: engine cut first, so the pilot inherits a ship, not a
// fight.
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

// === PHASE 1: COAST TO PDI ===

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

// === FLIGHT RECORDER ===
// One CSV row per second from the powered phases, so flights are analyzed from
// telemetry instead of remembered impressions. Lands in this directory (the kOS
// archive); overwritten on each run. Lines beginning '#' are metadata — the
// planning numbers each flight is judged against.
//
// v_to_site is signed horizontal speed toward the site — the reversal detector;
// facing_err vs throttle exposes wrong-direction burning; dv_rem turns phase
// costs into ledger entries. pitch/cmd_pitch are the nose's and the commanded
// thrust vector's angle above the horizon: together they are the chord-vs-arc
// error the turn_budget is chosen against.
local flightlog is "flight_log.csv".

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

// === THE LEG FLYER (P63/P64) ===
// Fly the guidance law to one leg's target state. Runs until t_go reaches the
// leg's handoff time, then returns with the engine still burning — the next leg
// picks up the throttle without a gap.
function fly_leg {
  parameter leg.

  // Desired arrival velocity, recomputed each call so the direction stays
  // body-fixed as the body rotates: v_horiz toward the site, v_vert up.
  local v_tgt_now is {
    local vt is leg:v_vert * up:vector.
    if leg:v_horiz <> 0 {
      local to_site is vxcl(up:vector,
        tgt:position - leg:aim_geo:altitudeposition(leg:aim_alt)).
      set vt to vt + leg:v_horiz * to_site:normalized.
    }
    return vt.
  }.

  // The leg's closure pins t_go. Extracted to a local once: kOS will not
  // call a delegate directly off a lexicon suffix.
  local leg_closure is leg:closure.
  local t_go is leg_closure(v_tgt_now()).
  if t_go < 0 {
    guidance_abort(leg:name + ": closure found no feasible t_go.").
  }
  print leg:name + ": t_go " + round(t_go) + " s (arc says "
      + round(leg:t_plan) + "); site "
      + round(vxcl(up:vector, tgt:position):mag / 1000, 1) + " km down-range.".

  local a_thrust is guidance_step(leg:aim_geo, leg:aim_alt, v_tgt_now(), t_go).
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
  local t_logged is 0.              // last flight-recorder row

  until t_go < leg:t_handoff {
    set a_thrust to guidance_step(leg:aim_geo, leg:aim_alt, v_tgt_now(), t_go).

    // Ground-proximity invariant; see tessellate. Too low for hand-over —
    // use whatever authority is left to land, not to halt.
    if alt:radar < leg:alt_floor {
      emergency_land(leg:name + ": radar altitude below "
          + round(leg:alt_floor) + " m floor.").
    }

    if time:seconds - t_logged >= 1 {
      log_state(leg:name, t_go, leg:aim_geo, leg:aim_alt, a_thrust).
      set t_logged to time:seconds.
    }

    wait 0.

    // t_go: decrement by wall clock each tick, re-solve every ~10 s to
    // shed accumulated model error. The cadence only needs to be short
    // against the phase and long against the physics tick.
    set t_go to t_go - (time:seconds - t_last).
    set t_last to time:seconds.
    if time:seconds - t_solved > 10 {
      set t_go to leg_closure(v_tgt_now()).
      if t_go < 0 {
        guidance_abort(leg:name + ": t_go re-solve found no feasible root.").
      }
      set t_solved to time:seconds.
    }
  }
}

// === TERMINAL DESCENT (P66) ===

// Rate-of-descent control, which is what P66 actually was: the reference
// descent rate is a function of radar altitude, and the throttle servos
// the actual rate onto it around a gravity-cancelling feedforward.
//
// The reference profile is -min(5, max(2, alt:radar / 10)):
//   - capped at 5 m/s so it is continuous with the arc's arrival speed
//     (an uncapped alt/10 would command -15 m/s at 150 m, making P66 begin
//     by speeding the descent back up);
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

if hasnode {
  print "ABORT: a maneuver node is still pending. Burn it first;".
  print "this script begins on the descent ellipse, not before it.".
  wait until false.
}

// Plan on the prediction, fly on the measurement. The coast is unpowered, so
// the state at PDI is already determined by the ellipse we are on -- there is
// nothing to learn by waiting, and everything to lose: at PDI the ship is
// falling, and the solve takes seconds it does not have. Solving here, before
// the warp, also means an infeasible descent is caught while the whole coast
// is still ahead and the orbit can still be raised.
local h_pdi is ship:orbit:periapsis.
local r_pe is body:radius + h_pdi.
// Vis-viva gives the orbital speed at a radius: v^2 = mu*(2/r - 1/a). Subtract
// the speed of a point turning with the body underneath, because the arc is
// flown against the ground, not against the stars. Equatorial, per the Phase 0
// assumption above.
local speed_pdi is sqrt(body:mu * (2 / r_pe - 1 / ship:orbit:semimajoraxis))
                 - 2 * constant:pi * r_pe / body:rotationperiod.
local drop_wanted is h_pdi - (tgt:terrainheight + landing_height).
print "PDI predicted: " + round(h_pdi) + " m at " + round(speed_pdi, 1)
    + " m/s; drop to handoff " + round(drop_wanted) + " m.".

local f is solve_throttle(h_pdi, speed_pdi, drop_wanted).
if f < 0 {
  // Nothing is committed yet -- no lock is held and the coast has not begun --
  // so this hands back a ship in a stable orbit rather than a falling one.
  print "ABORT: no throttle between " + f_min + " and " + f_max + " flies this".
  print "ellipse down to the handoff. PDI is " + round(h_pdi) + " m over a site".
  print "at " + round(tgt:terrainheight) + " m: the arc either cannot reach the".
  print "site or would fly through the ground to get there.".
  print "Nothing has been committed. Re-plan the descent ellipse.".
  wait until false.
}
print "Throttle solved: " + round(f, 3) + " of full ("
    + round(f * a_max, 2) + " m/s^2).".

coast_to_pdi().

// At PDI the prediction stops mattering: the ship's own altitude IS the PDI
// altitude and its surface speed IS the arc's seed, so the arc the legs come
// from is integrated from what the ship actually has, not what the ellipse
// promised.
local h_actual is ship:altitude.
local speed_actual is ship:velocity:surface:mag.
print "PDI: " + round(h_actual) + " m at " + round(speed_actual, 1) + " m/s ("
    + round(h_actual - h_pdi) + " m, " + round(speed_actual - speed_pdi, 1)
    + " m/s off plan).".

local arc is integrate_arc(h_actual, speed_actual, f * a_max, dt_arc,
                           speed_handoff, max_steps).
local last is arc[arc:length - 1].
// The final sample is the only one at or below speed_handoff. An arc that
// spent its whole step budget never got there, which means this craft cannot
// fly this ellipse down -- and by here the coast has the steering, so the exit
// goes through guidance_abort.
if last["speed"] > speed_handoff {
  guidance_abort("the arc spent all " + max_steps + " steps with speed to burn.").
}

local legs is tessellate(arc).

if exists(flightlog) { deletepath(flightlog). }
log "# target " + round(target_lat, 4) + " " + round(target_lng, 4)
    + "  terrain " + round(tgt:terrainheight) + " m" to flightlog.
log "# h_pdi " + round(h_actual) + "  speed_pdi " + round(speed_actual, 1)
    + "  planned " + round(h_pdi) + " / " + round(speed_pdi, 1)
    + "  drop_wanted " + round(drop_wanted) to flightlog.
log "# f_solved " + round(f, 3) + "  a_thrust " + round(f * a_max, 2)
    + "  twr_pdi " + round(a_max / (body:mu / (body:radius + h_actual) ^ 2), 1)
    to flightlog.
log "# arc  steps " + arc:length + "  duration " + round(last["t"], 1)
    + " s  downrange " + round(last["x"]) + " m  end_h " + round(last["h"])
    + "  gamma " + round(arctan2(h_actual - last["h"], last["x"]), 2) + " deg"
    to flightlog.
log "# legs " + legs:length + "  turn_budget " + turn_budget
    + "  landing_height " + landing_height + "  dt_arc " + dt_arc to flightlog.
log "# dv_at_pdi " + round(ship:deltav:current, 1) to flightlog.
log "t,phase,t_go,alt,radar,v_to_site,v_vert,aim_dist,a_cmd,throttle,facing_err,mass,dv_rem,pitch,cmd_pitch"
    to flightlog.

print "Arc: " + round(last["t"], 1) + " s, "
    + round(last["x"] / 1000, 1) + " km downrange, ending "
    + round(last["h"]) + " m at " + round(last["speed"], 1) + " m/s; "
    + legs:length + " legs.".

for leg in legs {
  fly_leg(leg).
}
terminal_descent().

// The headline number: horizontal distance from the touchdown point to the
// target site.
local miss is vxcl(up:vector, tgt:position):mag.
print "Landed. Miss distance: " + round(miss) + " m.".
log "# landed  miss " + round(miss) + " m  dv_rem " + round(ship:deltav:current, 1)
    to flightlog.
