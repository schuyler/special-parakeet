// powered_descent.ks — the powered descent, from PDI to touchdown.
// Design: notes/capability-driven-descent.md (piece 1).
//
// Begins work on a descent ellipse someone else planned: the DOI burn is
// already behind us, and periapsis is PDI. So the PDI altitude is not a
// parameter here, it is read off the orbit. Nothing in this file plans a
// burn, places a node, or looks at terrain up-range; that is the planner's
// job (pieces 2 and 3), and the node it left is the whole interface.
//
// The braking burn is a gravity turn flown in reverse, and a gravity turn
// flies itself: hold thrust surface-retrograde and gravity rotates the path
// from level to vertical while the burn takes the speed. Before the coast,
// a bisection finds the one throttle whose arc bottoms out at the handoff
// height. At PDI the arc is integrated once from live state and kept as a
// table keyed by speed: the plan, written down. In flight two loops close
// against it at the recorder's cadence: a one-sided throttle trim —
// throttle up when the endpoint drifts long of the site, never down — and
// a few degrees of yaw that null the site's offset from the flown plane,
// buying cross-track correction while the ship is fast, where a degree
// costs least. Pulling the endpoint
// up-range costs a little extra height at the end; pushing it down-range
// would mean planning the arc into the ground, or arresting to translate
// at hover, the most expensive maneuver there is. So the plan arrives a
// little long on purpose, and the trim spends that overshoot allowance
// down to zero at the handoff. Terminal descent owns the last few metres
// of error with a drift cascade aimed at the site.
//
// Deliberately bare-bones: the reference implementation of the guidance
// design, not an operational autopilot. Steps a more complex mission would
// need are marked "OMITTED:" at the point they would go.

@lazyglobal off.

clearscreen.
print "=== POWERED DESCENT ===".

// common for engine_isp; kepler for orbital_speed and, through its own
// runoncepath, bisect. Both files define orbital_speed with different
// signatures; kepler runs last so its (altitude, orbit) form — the one
// called below — is the survivor.
run "common".
run "../core/kepler".

parameter target_lat is 0.
parameter target_lng is 0.
// The arc ends this far above the site's terrain and this slow. Below and
// after, the terminal rate controller flies. Every metre of it is spent in a
// near-hover, so it is gravity loss; the floor under it is terminal's room to
// flare, null drift, and stay clear of the ground.
parameter landing_height is 50.
// Speed at which the arc ends and terminal takes over. The gravity turn's
// turn rate carries speed in its denominator, so the arc must stop above
// zero; terminal's reference descent rate is capped at this same speed so
// the handoff is continuous.
parameter speed_handoff is 5.
// Ceiling on the design throttle, as a fraction of full. The reserve (1 - f)
// is the one-sided margin the trim spends pulling the endpoint back up-range,
// so this is a constraint on the solve rather than a value it aims for:
// solving under the ceiling only leaves more margin, never less.
parameter f_max is 0.85.
// Floor of the solve's bracket: a throttle this low burns long enough that
// the arc falls below the handoff on any ellipse worth flying, which is
// what bisection needs from that end.
parameter f_min is 0.05.
// How close the throttle solve has to get. The handoff height is violently
// sensitive to throttle — a small change compounds over the whole burn — so
// the tolerance is far tighter than any engine's throttle resolution.
parameter f_epsilon is 0.0001.
// Steps the march spends resolving an arc, whatever its duration: the step
// size comes from the burn's own estimated length, so a twelve-second burn
// and a four-minute one are drawn with the same fidelity.
parameter arc_steps is 500.
// The overshoot allowance, as a multiple of the model's self-measured
// endpoint error (see the mission sequence). Dimensionless: how far to
// distrust the model beyond what the model can see about itself.
parameter model_error_margin is 2.
// Fraction of the braking thrust the cross-track correction may spend
// steering off retrograde. The loss is 1 - cos(yaw), quadratic in the
// angle, so one percent buys about eight degrees of yaw. Demand past the
// cap saturates and the script warns: a plane that far off is the
// planner's error to fix, not the braking phase's to absorb.
parameter steering_loss_budget is 0.01.

local tgt is body:geopositionlatlng(target_lat, target_lng).
// The altitude, above the datum, where the arc ends and terminal begins:
// landing_height above the site's terrain. The solve aims the arc at it; the
// braking guard measures margin against it.
local h_handoff is tgt:terrainheight + landing_height.
// Locked, not sampled: mass falls through the burn, so readers during it
// (the recorder) see the live acceleration. Before ignition it is constant,
// so the solve and the table see the PDI value either way.
local lock a_max to ship:availablethrust / ship:mass.

// A dead stage cannot be planned around: every quantity below divides by
// the engine's thrust or its flow rate.
if ship:availablethrust <= 0 {
  print "ABORT: no live engine. Stage or activate the descent engine,".
  print "then rerun. Nothing has been committed.".
  wait until false.
}

// Mass leaves through the engine at thrust / (Isp * g0) at full throttle;
// the stepper scales it by the throttle. Depletion has to be modelled
// because it is systematic and one-sided: an arc planned at constant mass
// brakes harder than predicted as the ship lightens, and stops short — the
// side the throttle trim cannot reach. engine_isp reads the first live
// engine, so one engine type burns at a time.
local mdot_full is ship:availablethrust / (engine_isp() * constant:g0).

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
// Euler's method: hold the state in a handful of numbers, compute how much
// each changes over a short interval dt, then add the changes on and repeat.
// Small enough steps and the sum of the straight hops traces the curve.
function integrate_arc {
  parameter f.            // throttle, as a fraction of full thrust
  // Speed at which the arc ends, m/s. d_pitch carries speed in its
  // denominator, so the turn rate climbs steeply as speed falls; stopping
  // above zero keeps it finite and hands the rest of the descent to TERMINAL.
  parameter speed_low.
  // Rows recorded along the arc beyond the seed and the endpoint: 0 for the
  // throttle solve, which runs this march hundreds of times and reads only
  // where the arc bottoms out; table_rows for the run that produces the
  // descent table. One stepper serves both so they cannot drift apart, and
  // the solve never optimises an arc the ship doesn't fly.
  parameter rows_ is 0.
  // Step budget for the march. The default draws every arc at the same
  // fidelity; the halved budget in the mission sequence re-draws one arc
  // coarsely to price the discretization.
  parameter steps_ is arc_steps.
  // The descent ellipse the arc begins on.
  parameter orbit_ is ship:orbit.

  // The seed: periapsis is PDI's altitude, and the speed there is vis-viva
  // less the motion of the ground underneath, because the arc is flown
  // against the ground, not against the stars. Equatorial, per the Phase 0
  // assumption above.
  local h is orbit_:periapsis.
  local r_pe is orbit_:body:radius + h.
  local speed is orbital_speed(h, orbit_)
               - 2 * constant:pi * r_pe / orbit_:body:rotationperiod.

  local thrust is f * ship:availablethrust.   // constant: the throttle holds
  local m is ship:mass.
  local mdot is f * mdot_full.
  // The step: the burn's estimated duration over the step budget. The
  // estimate ignores the speed gravity feeds back along the path, so the
  // real burn runs longer; the loop cap below gives it four times the room.
  local dt is (speed - speed_low) * m / thrust / steps_.

  local pitch is 0.       // degrees above the horizon; PDI is a periapsis
  // Angle swept around the body's centre since PDI, radians. Radians because
  // theta * body:radius is then the distance travelled over the ground, which
  // is what the trim measures against.
  local theta is 0.
  local t is 0.
  local steps is 0.
  local record_dspeed is 0.
  if rows_ > 0 { set record_dspeed to (speed - speed_low) / rows_. }
  local speed_rec is speed - record_dspeed.
  local arc is list().
  // The seed row: the state the arc begins from — the plan's PDI.
  arc:add(lexicon("t", 0, "speed", speed, "h", h, "x", 0)).

  until speed <= speed_low or steps >= 4 * steps_ {
    // r_ trails an underscore because kOS reserves R() for rotations.
    local r_ is body:radius + h.
    local g is body:mu / r_ ^ 2.
    // The ship lightens as it burns, so the same thrust buys more braking
    // late in the arc.
    local a_thrust is thrust / m.

    if rows_ > 0 and speed <= speed_rec {
      arc:add(lexicon("t", t, "speed", speed, "h", h,
                      "x", theta * body:radius)).
      set speed_rec to speed_rec - record_dspeed.
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
    set m     to m     - mdot * dt.
    set t     to t     + dt.
    set steps to steps + 1.
  }

  // The loop samples at the top and steps at the bottom, so it always exits
  // holding a state it never recorded. Append it: that state is the handoff,
  // and it is the only sample whose speed is at or below speed_low — which is
  // what tells a closed arc from one that ran out of steps with speed still
  // to burn.
  arc:add(lexicon("t", t, "speed", speed, "h", h,
                  "x", theta * body:radius)).
  return arc.
}

// === SOLVING THE THROTTLE ===
// The arc has to end at the handoff height. PDI's altitude is already fixed —
// it is the periapsis of the ellipse we are on, and nothing here can move it —
// so the only thing left to vary is how hard the engine pushes. Less throttle
// means a longer burn, which gives gravity longer to pull the path down; the
// ending height rises with the throttle, and there is one throttle that makes
// it come out exactly right. Bisection finds it.
//
// This is why the throttle is not a parameter here. The planner chooses the
// approach it wants and expresses that choice as PDI's altitude; by the time
// this script runs, that choice has already been made, and the throttle is
// whatever flies the arc it implies.
function solve_throttle {
  parameter orbit_ is ship:orbit.

  // Where the arc bottoms out, relative to where it should: negative when
  // the burn ran long and fell below the handoff, positive when it stopped
  // above it.
  local miss is {
    parameter f.
    local arc is integrate_arc(f, speed_handoff, 0, arc_steps, orbit_).
    // A march that spent its whole step budget exited with speed still to
    // burn, and with speed left the arc is still falling: wherever it
    // stopped, its true bottom is lower. Report it far below the handoff,
    // which steers the search toward more throttle.
    if arc[arc:length - 1]["speed"] > speed_handoff { return -1e9. }
    return arc[arc:length - 1]["h"] - h_handoff.
  }.
  // Bisection needs the answer bracketed, and it is: the ending height rises
  // with the throttle, so f_min ends below the handoff and f_max above it.
  // Returns -1 if that bracket does not hold, which is the abort.
  return bisect(miss, f_min, f_max, f_epsilon).
}

// === THE DESCENT TABLE ===
// The arc, recorded and kept. A row is a statement: "when the surface speed
// has fallen to this value, the plan has the ship at altitude h, x metres
// along the ground from PDI, t seconds after ignition." Speed is the key
// because removing speed is the burn's whole job — how much remains IS how
// far along the job is — and because it falls strictly monotonically, which
// pitch does not: the arc rises just after PDI, so shallow pitches repeat.
// The middle rows are instruments, not targets. A trim deliberately flies
// off them, so that the flown x meets the last row's at the bottom.

// Rows in the table, whatever the descent's speed span. Enough that linear
// interpolation between neighbours is exact for practical purposes; memory
// is nothing.
local table_rows is 80.

// The table's row at this speed, interpolated linearly between the two
// recorded rows that bracket it. The scan is linear from the top: eighty-odd
// rows at a lookup a second is a rounding error against the IPU budget, and
// it keeps the lookup free of state.
function table_at {
  parameter tbl.
  parameter spd.

  local i is 0.
  until i >= tbl:length - 2 or tbl[i + 1]["speed"] <= spd {
    set i to i + 1.
  }
  local a is tbl[i].
  local b is tbl[i + 1].
  local span is a["speed"] - b["speed"].
  local frac is 0.
  if span > 0 { set frac to min(1, max(0, (a["speed"] - spd) / span)). }
  return lexicon(
    "t", a["t"] + frac * (b["t"] - a["t"]),
    "h", a["h"] + frac * (b["h"] - a["h"]),
    "x", a["x"] + frac * (b["x"] - a["x"])).
}

// Ground distance from the ship to the site: the angle between their radial
// directions, times the radius — a great-circle length, which is what the
// table's x column is measured in. A chord would understate it by tens of
// metres over a long descent.
function dist_to_site {
  return body:radius * constant:degtorad
       * vang(ship:position - body:position, tgt:position - body:position).
}

// === ABORTS ===

// OMITTED: active aborts. An operational script would fly abort-to-orbit
// (thrust up and up-range back to a stable ellipse) from any powered phase
// rather than handing a falling ship to the pilot.

// The descent has diverged with no altitude left to argue about it: stop
// following the plan, kill velocity at full thrust, and land where we are.
// The target is abandoned — a wrong-place landing beats a right-place
// crater. Ends parked: the mission sequence must not continue past this.
function emergency_land {
  parameter why.
  print "EMERGENCY: " + why.
  print "Abandoning target. Killing velocity; landing here.".
  lock steering to srfretrograde.
  lock throttle to 1.
  wait until ship:velocity:surface:mag < 10 or ship:status = "LANDED".
  terminal_descent(false).
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
  // 1 s is "at periapsis" for our purposes: the trim absorbs tens of
  // seconds of ignition-timing slop, so the threshold only needs to be
  // small against that.
  lock steering to srfretrograde.
  wait until eta:periapsis <= 1.
}

// === FLIGHT RECORDER ===
// One CSV row per second from the powered phases, so flights are analyzed
// from telemetry instead of remembered impressions. Lands in this directory
// (the kOS archive); overwritten on each run. Lines beginning '#' are
// metadata — the planning numbers each flight is judged against.
//
// v_to_site is signed horizontal speed toward the site — the reversal
// detector; facing_err vs throttle exposes wrong-direction burning; dv_rem
// turns phase costs into ledger entries. pitch/cmd_pitch are the nose's and
// the commanded thrust vector's angle above the horizon; their gap is the
// steering lock's tracking error. cross is the site's signed offset from
// the flown plane — the cross-track law's error signal; TERMINAL logs 0.
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

// === THE BRAKING PHASE ===
// Hold thrust along the commanded braking direction and let the arc fly
// itself. Two loops close at the recorder's cadence, one per axis, and both
// project the same way — a deviation measured now rides through to the
// endpoint:
//   - In-plane, the throttle. Measured ground remaining to the site,
//     differenced against the table's, is the projected overshoot; overshoot
//     beyond the tapering allowance raises the throttle, never lowers it.
//     Corrections end the arc a little higher as well as shorter, and
//     terminal pays that in a few seconds of descent — the price of keeping
//     every error on the side the throttle can reach.
//   - Cross-track, the steering. The site's offset from the plane of the
//     velocity is the projected lateral miss; a few degrees of yaw off
//     retrograde null it while the ship is fast, at a thrust cost quadratic
//     in the angle.
function fly_braking {
  parameter tbl.
  parameter f0.               // the solved throttle; f_cmd only rises from it
  parameter x_shrink_per_f.   // m of down-range removed per unit of throttle
  parameter allowance_pdi.    // overshoot allowance at PDI, m

  local x_total is tbl[tbl:length - 1]["x"].
  local t_total is tbl[tbl:length - 1]["t"].
  local speed_top is tbl[0]["speed"].

  local f_cmd is f0.
  local a_cross is v(0, 0, 0).   // lateral acceleration demand, m/s^2
  local cross_warned is false.

  // The commanded acceleration is retrograde braking plus the lateral
  // demand. Steering follows it, and the vang gate holds the throttle
  // closed unless the ship points near it. Nominally the gate never fires;
  // it exists to deny fuel to divergence, since mis-pointed thrust is the
  // energy source that sustains one.
  lock steering to lookdirup(
      f_cmd * a_max * srfretrograde:vector + a_cross,
      ship:facing:topvector).
  lock throttle to choose f_cmd
      if vang(f_cmd * a_max * srfretrograde:vector + a_cross,
              ship:facing:vector) < 30 else 0.

  print "BRAKE: retrograde hold at f " + round(f0, 4) + ".".

  local t_logged is 0.
  until ship:velocity:surface:mag <= speed_handoff {
    if time:seconds - t_logged >= 1 {
      local spd is ship:velocity:surface:mag.
      local plan is table_at(tbl, spd).
      local t_go is t_total - plan["t"].

      // Ground-proximity invariant: the plan's height above the handoff is
      // the margin the descent is entitled to spend, and being below the
      // plan by more than half of it means the burn is not delivering.
      // Trims only push the ship HIGH of the table, so this fires on
      // under-performance, never on a correction. Altitude against
      // altitude, both from the datum, so terrain under the track cannot
      // false-trigger it. The tolerance floors at landing_height: near the
      // bottom the plan converges on the handoff and half the margin
      // shrinks toward zero, where ordinary dispersion would read as
      // divergence.
      local margin is plan["h"] - h_handoff.
      if margin > 0 and plan["h"] - ship:altitude
                        > max(margin / 2, landing_height) {
        emergency_land("BRAKE: " + round(plan["h"] - ship:altitude)
            + " m below the planned arc.").
      }

      // The throttle trim, as a ratchet. Each look computes, from fixed
      // references, the total throttle whose arc ends at the allowance —
      // and only ever raises f_cmd to meet it. Repeating the computation
      // against an unchanged measurement changes nothing, so the look rate
      // cannot stack corrections while one is still taking effect. The
      // gain is scaled by the arc remaining, because a throttle change
      // applied now acts only on the burn still ahead.
      local overshoot is (x_total - plan["x"]) - dist_to_site().
      local allowance is allowance_pdi * (spd - speed_handoff)
                       / (speed_top - speed_handoff).
      local remaining is (x_total - plan["x"]) / x_total.
      if overshoot > allowance and remaining > 0 {
        set f_cmd to min(f_max, max(f_cmd,
            f0 + (overshoot - allowance) / (x_shrink_per_f * remaining))).
      }

      // Cross-track. The site's signed offset from the plane the ship is
      // flying in is the lateral miss the arc will book if nothing steers,
      // because an unforced gravity turn stays in its plane. The
      // constant-jerk profile that arrives centred with no drift asks for
      // 6 y / t_go^2 of lateral acceleration now; it is delivered by
      // yawing the thrust, and the cap keeps that yaw inside the steering
      // budget. Re-measuring against the live velocity each look absorbs
      // the correction's own progress and the body's rotation alike.
      local n_cross is vcrs(ship:velocity:surface, up:vector):normalized.
      local y_pred is vdot(tgt:position, n_cross).
      set a_cross to n_cross * (6 * y_pred / max(t_go, 1) ^ 2).
      // The cap fades with the fraction of the speed still horizontal: the
      // law steers by rotating the ground track, and as the path goes
      // vertical near handoff that leverage and n_cross's definition vanish
      // together — the same geometry, so no separate constant.
      local a_cross_max is f_cmd * a_max * sqrt(2 * steering_loss_budget)
                        * vxcl(up:vector, ship:velocity:surface):mag / spd.
      if a_cross:mag > a_cross_max {
        set a_cross to a_cross:normalized * a_cross_max.
        if not cross_warned {
          print "WARNING: cross-track demand saturated. The plane misses the".
          print "site by more than the steering budget corrects.".
          set cross_warned to true.
        }
      }

      log_state("BRAKE", t_go, tgt, h_handoff,
          f_cmd * a_max * srfretrograde:vector + a_cross, y_pred).
      set t_logged to time:seconds.
    }
    wait 0.
  }
}

// === TERMINAL DESCENT (P66) ===

// Rate-of-descent control, which is what Apollo's P66 was: the reference
// descent rate is a function of radar altitude, and the throttle servos the
// actual rate onto it around a gravity-cancelling feedforward.
//
// The reference profile is -min(speed_handoff, max(2, alt:radar / 10)):
//   - capped at speed_handoff so it is continuous with the arc's arrival
//     speed (uncapped, alt/10 would command -15 m/s at 150 m, beginning
//     the phase by speeding the descent back up);
//   - proportional to height (tau = 10 s) through the middle;
//   - floored at 2 m/s so touchdown happens rather than being approached
//     asymptotically.
function terminal_descent {
  // False when called from emergency_land: the target is abandoned, so the
  // position loop is zeroed and only the drift damper flies.
  parameter chase_site is true.

  print "TERMINAL: rate-of-descent control from " + round(alt:radar) + " m.".

  // OMITTED: site redesignation and slope/quality checks. The ground below
  // is trusted because it was surveyed before the mission — that trust is
  // the core design trade, not an oversight.

  local g0 is body:mu / body:radius ^ 2.
  local lock v_ref to -min(speed_handoff, max(2, alt:radar / 10)).
  local v_cap is choose 3 if chase_site else 0.

  // Gravity feedforward plus a proportional correction. The 0.3 gain is
  // in units of 1/s: an error of 1 m/s adds 0.3 m/s^2 of commanded accel,
  // a ~3.3 s closed-loop time constant — brisk against the 10 s reference
  // profile, yet 5 m/s of error demands only 1.5 m/s^2 over the
  // feedforward, well inside TWR-2 authority. max() guards flameout.
  lock throttle to (g0 + 0.3 * (v_ref - verticalspeed)) * ship:mass
                   / max(0.001, ship:availablethrust).

  // Two loops in cascade. The outer turns position error into a commanded
  // closing drift: 0.2 m/s per metre of horizontal offset to the site,
  // capped at v_cap, so beyond 15 m the ship closes at a constant 3 m/s and
  // inside that it eases off linearly. The inner is the drift damper: 0.1
  // of tilt per m/s of velocity error, ~6 degrees per m/s, so the cap also
  // bounds the lean this close to the ground at ~17 degrees. Over the site
  // the command is zero and this reduces to plain drift-nulling — which is
  // why the damper owns the inner loop: a lander that tips hard at 10 m to
  // chase 30 m of miss trades a wrong-place landing for a tipped-over one.
  function tilt {
    local off is vxcl(up:vector, tgt:position).
    local v_err is vxcl(up:vector, ship:velocity:surface)
                 - off * min(0.2, v_cap / max(0.001, off:mag)).
    return up:vector - 0.1 * v_err.
  }

  lock steering to lookdirup(tilt(), ship:facing:topvector).

  gear on.

  // LANDED is the real signal; the fallback catches a hover balanced just
  // above contact — 5 m is landing-leg scale, -0.1 m/s is "effectively
  // stopped." The recorder runs here too: a_cmd is the ROD servo's demand
  // along the commanded steering direction; t_go is not a quantity this
  // phase has, so it logs 0; aim_dist is live distance to the site — the
  // touchdown drift.
  local t_logged is 0.
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

// Everything the descent needs is computable before the coast: the coast is
// unpowered, so the ellipse the ship is on now is the ellipse it will reach
// PDI on, and the table built from it here is the same table PDI would
// yield. Planning before the warp also means an infeasible descent is
// caught while the whole coast is still ahead and the orbit can still be
// raised.
local f is solve_throttle().
if f < 0 {
  // Nothing is committed yet — no lock is held and the coast has not begun —
  // so this hands back a ship in a stable orbit rather than a falling one.
  print "ABORT: no throttle between " + f_min + " and " + f_max + " flies this".
  print "ellipse down to the handoff. PDI is " + round(ship:orbit:periapsis)
      + " m over a site".
  print "at " + round(tgt:terrainheight) + " m: the arc either cannot reach the".
  print "site or would fly through the ground to get there.".
  print "Nothing has been committed. Re-plan the descent ellipse.".
  wait until false.
}
print "Throttle solved: " + round(f, 4) + " of full ("
    + round(f * a_max, 2) + " m/s^2).".

// The descent table, at full resolution: seed row PDI, last row the handoff.
local tbl is integrate_arc(f, speed_handoff, table_rows).
local last is tbl[tbl:length - 1].
// The last row is the only one at or below speed_handoff. An arc that spent
// its whole step budget never got there, which means this craft cannot fly
// this ellipse down.
if last["speed"] > speed_handoff {
  print "ABORT: the arc spent its whole step budget with speed to burn.".
  print "Nothing has been committed. Re-plan the descent ellipse.".
  wait until false.
}
print "PDI predicted: " + round(tbl[0]["h"]) + " m at "
    + round(tbl[0]["speed"], 1) + " m/s; handoff at "
    + round(h_handoff) + " m.".
print "Arc: " + round(last["t"], 1) + " s, "
    + round(last["x"] / 1000, 1) + " km downrange, ending "
    + round(last["h"]) + " m at " + round(last["speed"], 1) + " m/s; "
    + tbl:length + " table rows.".

// The trim gain, by finite difference: the table's arc against one probed a
// little above it. The probe step is large against f_epsilon, so the
// difference reads the slope of X(f) rather than the solver's noise, and
// small against its curvature.
local df_probe is 0.005.
local probe_arc is integrate_arc(f + df_probe, speed_handoff).
local x_shrink_per_f is (last["x"]
                       - probe_arc[probe_arc:length - 1]["x"]) / df_probe.
print "Trim gain: " + round(x_shrink_per_f * 0.001)
    + " m shorter per 0.001 of throttle.".

// The overshoot allowance, from the model's self-measured error: the same
// arc marched again on half the step budget, and the difference between the
// two endpoints is the size of what this integrator, at this step, cannot
// say about its own answer. The trim protects a multiple of it. Derived
// rather than chosen, so it scales itself to the arc, the craft, and the
// body.
local coarse_arc is integrate_arc(f, speed_handoff, 0, arc_steps / 2).
local overshoot_allowance is model_error_margin
    * abs(last["x"] - coarse_arc[coarse_arc:length - 1]["x"]).
print "Overshoot allowance: " + round(overshoot_allowance) + " m at PDI.".

coast_to_pdi().

// Arriving at PDI only checks the table. The gap between the ship's
// measured state and the seed row is the whole prediction error of the
// coast, and the braking loops absorb it.
print "PDI arrival: " + round(ship:altitude - tbl[0]["h"]) + " m, "
    + round(ship:velocity:surface:mag - tbl[0]["speed"], 1)
    + " m/s off the table's seed.".

// Where the plan puts the endpoint relative to the site, before anything is
// flown: the planner's placement error plus the coast's, measured. The trim
// can only shorten, so short of the site is a miss already booked.
local overshoot_pdi is last["x"] - dist_to_site().
print "Endpoint projected " + round(abs(overshoot_pdi)) + " m "
    + (choose "long" if overshoot_pdi >= 0 else "short") + " of the site.".
if overshoot_pdi < 0 {
  print "WARNING: the trim only pulls the endpoint up-range. Expect to land".
  print "short unless terminal walks the difference.".
}

// The plane the coast delivered, measured the same way the braking law will
// measure it: the site's signed offset from the plane of the surface
// velocity.
local n_pdi is vcrs(ship:velocity:surface, up:vector):normalized.
local cross_pdi is vdot(tgt:position, n_pdi).
print "Plane misses the site laterally by " + round(abs(cross_pdi)) + " m.".

if exists(flightlog) { deletepath(flightlog). }
log "# target " + round(target_lat, 4) + " " + round(target_lng, 4)
    + "  terrain " + round(tgt:terrainheight) + " m" to flightlog.
log "# h_pdi " + round(ship:altitude) + "  speed_pdi "
    + round(ship:velocity:surface:mag, 1)
    + "  planned " + round(tbl[0]["h"]) + " / " + round(tbl[0]["speed"], 1)
    + "  h_handoff " + round(h_handoff) to flightlog.
log "# f_solved " + round(f, 4) + "  a_thrust " + round(f * a_max, 2)
    + "  twr_pdi " + round(a_max / (body:mu / (body:radius + tbl[0]["h"]) ^ 2), 1)
    to flightlog.
log "# arc  rows " + tbl:length + "  duration " + round(last["t"], 1)
    + " s  downrange " + round(last["x"]) + " m  end_h " + round(last["h"])
    + "  gamma " + round(arctan2(tbl[0]["h"] - last["h"], last["x"]), 2) + " deg"
    to flightlog.
log "# trim  gain " + round(x_shrink_per_f) + " m/f  allowance "
    + round(overshoot_allowance) + " m  overshoot_pdi " + round(overshoot_pdi)
    + "  cross_pdi " + round(cross_pdi) to flightlog.
log "# dv_at_pdi " + round(ship:deltav:current, 1) to flightlog.
log "t,phase,t_go,alt,radar,v_to_site,v_vert,aim_dist,a_cmd,throttle,facing_err,mass,dv_rem,pitch,cmd_pitch,cross"
    to flightlog.

fly_braking(tbl, f, x_shrink_per_f, overshoot_allowance).
terminal_descent().

// The headline number: horizontal distance from the touchdown point to the
// target site.
local miss is vxcl(up:vector, tgt:position):mag.
print "Landed. Miss distance: " + round(miss) + " m.".
log "# landed  miss " + round(miss) + " m  dv_rem " + round(ship:deltav:current, 1)
    to flightlog.
