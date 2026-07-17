// plan_doi.ks — mission planning for the powered descent: place the DOI node.
// Design: notes/capability-driven-descent.md (piece 2).
//
// Given gamma, the descent angle — the human's judgment, standing in for
// the terrain survey the smart planner will someday do — solve the PDI
// altitude that angle implies for this craft on this orbit, and leave the
// maneuver node whose burn delivers it. The node is the whole output. Burn
// it, then run powered_descent.ks, which reads the resulting ellipse and
// flies it down. Nothing here steers, burns, or warps.
//
// gamma is the slope, degrees above horizontal, of the straight line from
// the handoff point up to PDI. The flown arc leaves PDI level and steepens
// monotonically, so it is concave and lies above that line everywhere:
// terrain the line clears, the flight clears. Shallow gamma spends less
// delta-v; steep gamma clears more terrain.

@lazyglobal off.

clearscreen.
print "=== PLAN DOI ===".

// common for engine_isp and burn_duration; kepler for orbital_speed,
// time_to_longitude, time_of_periapsis, geoposition_at and, through its
// own runoncepath, bisect. Both files define orbital_speed; kepler runs
// last so its (altitude, orbit) form — the one integrate_arc calls —
// survives.
run "common".
run "../core/kepler".

// The descent angle, degrees. No default: it is the one judgment this
// script cannot supply.
parameter gamma.
parameter target_lat is 0.
parameter target_lng is 0.
// The arc contract: everything from here through steering_loss_budget must
// match what powered_descent.ks is run with, or the descent priced here is
// not the descent flown. The reasoning behind each value lives with its
// twin there.
parameter landing_height is 50.
parameter speed_handoff is 5.
parameter f_max is 0.85.
parameter f_min is 0.05.
parameter f_epsilon is 0.0001.
parameter arc_steps is 500.
parameter model_error_margin is 2.
parameter steering_loss_budget is 0.01.

local tgt is body:geopositionlatlng(target_lat, target_lng).
// Altitude, above the datum, where the arc ends: landing_height above the
// site's terrain. The fixed point builds h_pdi on top of it.
local h_handoff is tgt:terrainheight + landing_height.
local a_max is ship:availablethrust / ship:mass.

// Planning is a few hundred integrations of the arc; run them at the
// processor's ceiling and put the setting back on the way out.
local ipu_prior is config:ipu.
set config:ipu to 2000.

// Every abort path: drop whatever node this script added, restore the
// processor setting, stop. The entry guard below ensures any standing node
// is ours to remove, so the ship is left exactly as found.
function plan_abort {
  parameter why.
  until not hasnode { remove nextnode. }
  set config:ipu to ipu_prior.
  print "ABORT: " + why.
  print "Nothing has been committed: no node remains, nothing has burned.".
  wait until false.
}

// A pending node is not ours to reason about — or to delete.
if hasnode {
  set config:ipu to ipu_prior.
  print "ABORT: a maneuver node is already pending. Burn or remove it first.".
  wait until false.
}
if ship:availablethrust <= 0 {
  plan_abort("no live engine. Stage or activate the descent engine; every"
      + " quantity below divides by its thrust or flow rate.").
}
if gamma <= 0 or gamma >= 90 {
  plan_abort("gamma is " + gamma + "; it is a descent slope in degrees and"
      + " must lie strictly between 0 and 90.").
}

print "gamma " + round(gamma, 2) + " deg; target " + round(target_lat, 4)
    + " " + round(target_lng, 4) + ", terrain "
    + round(tgt:terrainheight) + " m.".

// Mass leaves through the engine at thrust / (Isp * g0) at full throttle;
// the stepper scales it by the throttle.
local mdot_full is ship:availablethrust / (engine_isp() * constant:g0).

// === THE ARC, DUPLICATED ===
// integrate_arc is copied verbatim from powered_descent.ks, and
// solve_throttle nearly so: the plan is only as good as its price, and the
// price is only right if the planner marches exactly the arc the flight
// controller will fly. Until the two share a library, a change to either
// copy must be made in both.

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

// The one throttle whose arc bottoms out at the handoff altitude, found by
// bisection: the candidate ellipse fixes PDI, so how hard the engine
// pushes is the only free variable, and the ending height rises
// monotonically with it. The step budget and tolerance are parameters —
// the one departure from powered_descent.ks's copy — so the coarse tier
// below can run the same solve cheaply.
function solve_throttle {
  parameter orbit_ is ship:orbit.
  parameter steps_ is arc_steps.
  parameter eps_ is f_epsilon.

  // Where the arc bottoms out, relative to where it should: negative when
  // the burn ran long and fell below the handoff, positive when it stopped
  // above it.
  local miss is {
    parameter f.
    local arc is integrate_arc(f, speed_handoff, 0, steps_, orbit_).
    // A march that spent its whole step budget exited with speed still to
    // burn, and with speed left the arc is still falling: wherever it
    // stopped, its true bottom is lower. Report it far below the handoff,
    // which steers the search toward more throttle.
    if arc[arc:length - 1]["speed"] > speed_handoff { return -1e9. }
    return arc[arc:length - 1]["h"] - h_handoff.
  }.
  // Bisection needs the answer bracketed, and it is: f_min ends below the
  // handoff and f_max above it. Returns -1 if that bracket does not hold,
  // which is the caller's abort.
  return bisect(miss, f_min, f_max, eps_).
}

// === THE NODE ===

// One candidate DOI burn: a retrograde node that drops the periapsis to
// h_pdi_, placed so periapsis falls lead_deg_ up-range (west) of the site.
// Assumes a prograde, near-circular, equatorial parking orbit; the errors
// that assumption books are measured and fed back by place_node.
function plan_node {
  parameter h_pdi_.
  parameter lead_deg_.
  parameter lng_bias is 0.   // placement correction, degrees east, from
                             // place_node's periapsis check

  // The site moves east while the ship coasts half an orbit down to
  // periapsis, so aim at where it will be.
  local sma_desc is (ship:orbit:semimajoraxis + body:radius + h_pdi_) / 2.
  local t_coast is constant:pi * sqrt(sma_desc ^ 3 / body:mu).
  local site_advance is t_coast * 360 / body:rotationperiod.
  local aim_lng is tgt:lng + site_advance.

  // The burn point becomes the descent ellipse's apoapsis, half an orbit
  // (180 degrees inertial) before periapsis.
  local burn_lng is wrap_longitude(aim_lng - lead_deg_ - 180 + lng_bias).
  local t_burn is time_to_longitude(burn_lng).   // absolute TimeStamp

  // Vis-viva at the burn radius, twice: the descent ellipse's speed there,
  // less the speed the ship will arrive with. The difference is the burn.
  local r_burn is (positionat(ship, t_burn) - body:position):mag.
  local r_pe is body:radius + h_pdi_.
  local sma is (r_burn + r_pe) / 2.
  local v_new is sqrt(body:mu * (2 / r_burn - 1 / sma)).
  local v_old is velocityat(ship, t_burn):orbit:mag.

  return node(t_burn:seconds, 0, 0, v_new - v_old).
}

// Place the node for real: plan it, ask the predicted orbit where its
// periapsis actually falls, and feed the longitude miss back into the burn
// point until it converges. Needed because the parking orbit's radial
// velocity is not small against a DOI-sized burn: the burn point is not
// the new apoapsis and periapsis is not 180 degrees away. Returns the node
// ADDED to the flight plan, with the placement it settled on.
function place_node {
  parameter h_pdi_.
  parameter lead_deg_.

  // Body-frame longitude where periapsis belongs. The site is fixed in
  // this frame, so no rotation term appears.
  local desired_lng is wrap_longitude(tgt:lng - lead_deg_).
  local bias is 0.
  local nd is 0.
  local attempts is 0.
  local predicted_lng is 0.
  local error is 0.
  local t_pdi is 0.

  until false {
    set nd to plan_node(h_pdi_, lead_deg_, bias).
    add nd.
    // A non-positive ETA is time_to_longitude's failure sentinel.
    if nd:eta <= 0 { plan_abort("the DOI plan puts the burn in the past."). }

    set t_pdi to time_of_periapsis(timestamp(nd:time), nd:orbit).
    set predicted_lng to geoposition_at(t_pdi, nd:orbit):lng.
    set error to wrap_longitude(predicted_lng - desired_lng).
    set attempts to attempts + 1.
    print "  place " + attempts + ": periapsis lng "
        + round(predicted_lng, 2) + ", want " + round(desired_lng, 2)
        + " (err " + round(error, 2) + " deg).".

    if abs(error) < 0.2 or attempts >= 4 { break. }
    remove nd.
    set bias to bias - error.
  }
  return lexicon("node", nd, "t_pdi", t_pdi, "pe_lng", predicted_lng,
                 "want", desired_lng, "err", error, "attempts", attempts).
}

// === THE FIXED POINT ===
// h_pdi = h_handoff + X tan(gamma), where X — the ground the arc covers —
// itself depends on the ellipse h_pdi implies: a one-dimensional fixed
// point, solved by iteration. Each pass prices the arc on a candidate
// ellipse and rebuilds h_pdi from the X it reads. The update is contracted
// by tan(gamma) times dX/dh_pdi, small for any shallow approach, so it
// settles in a few passes. Settling means h_pdi stopped moving at the
// metre scale: executing the burn will move the realized periapsis by
// metres anyway, so a plan converged tighter than the burn can deliver
// buys nothing.
//
// Two tiers. The coarse passes run the solve at a fifth of the steps and
// fifty times the tolerance; their errors are priced away by the fine
// passes that follow, so their only job is to hand the fine tier a nearby
// starting point. The fine passes run at flight fidelity and carry the
// full placement feedback, the allowance, and the trim gain.

// The seed. X = 0 would pose the first solve a degenerate ellipse —
// periapsis at the handoff, nothing to descend through — whose bracket can
// fail on a high-thrust craft. The shortest ground any braking arc can
// cover is the stop distance at the throttle ceiling, so seed X there:
// every pass then prices a real descent, approaching the answer from
// below.
local r_seed is body:radius + h_handoff.
local sma_seed is (ship:orbit:semimajoraxis + r_seed) / 2.
local v_seed is sqrt(body:mu * (2 / r_seed - 1 / sma_seed))
              - 2 * constant:pi * r_seed / body:rotationperiod.
local x_seed is v_seed ^ 2 / (2 * f_max * a_max).
local h_pdi is h_handoff + x_seed * tan(gamma).
local lead_deg is x_seed / body:radius * constant:radtodeg.
local x_arc is 0.
local t_arc is 0.

local coarse_steps is arc_steps / 5.
local coarse_eps is f_epsilon * 50.
local coarse_iters is 0.
local d_h is 1e9.

until abs(d_h) < 1 {
  // Eight passes without settling means the map is not contracting here,
  // and more passes will not help.
  if coarse_iters >= 8 {
    plan_abort("h_pdi moved " + round(abs(d_h)) + " m on coarse pass 8;"
        + " the fixed point is not settling. Try a shallower gamma.").
  }
  local nd is plan_node(h_pdi, lead_deg).
  add nd.
  if nd:eta <= 0 { plan_abort("the DOI plan puts the burn in the past."). }
  local f_c is solve_throttle(nd:orbit, coarse_steps, coarse_eps).
  if f_c < 0 {
    plan_abort("no throttle between " + f_min + " and " + f_max + " flies"
        + " the gamma " + gamma + " ellipse (PDI " + round(h_pdi)
        + " m) down to the handoff. Re-think gamma or the parking orbit.").
  }
  local arc is integrate_arc(f_c, speed_handoff, 0, coarse_steps, nd:orbit).
  remove nd.

  set x_arc to arc[arc:length - 1]["x"].
  local h_new is h_handoff + x_arc * tan(gamma).
  set d_h to h_new - h_pdi.
  set h_pdi to h_new.
  set lead_deg to x_arc / body:radius * constant:radtodeg.
  set coarse_iters to coarse_iters + 1.
  print "coarse " + coarse_iters + ": h_pdi " + round(h_pdi) + " m  X "
      + round(x_arc / 1000, 1) + " km  f " + round(f_c, 3) + ".".
}

// The probe step for the trim gain dX/df: large against f_epsilon, so the
// difference reads the slope of X(f) rather than the solver's noise, and
// small against its curvature.
local df_probe is 0.005.

local f is 0.
local allowance is 0.
local x_shrink_per_f is 0.
local fine is 0.
local fine_passes is 0.
local converged is false.

until converged {
  // Pass 1 moves the lead once, when the allowance goes from zero to
  // measured; pass 2 confirms it. Three passes without settling means
  // something is inconsistent, not merely unconverged.
  if fine_passes >= 3 {
    plan_abort("the flight-fidelity solve did not settle in 3 passes.").
  }
  set fine to place_node(h_pdi, lead_deg).
  local nd is fine["node"].

  set f to solve_throttle(nd:orbit).
  if f < 0 {
    plan_abort("at flight fidelity, no throttle between " + f_min + " and "
        + f_max + " flies the ellipse down to the handoff.").
  }
  local arc is integrate_arc(f, speed_handoff, 0, arc_steps, nd:orbit).
  local last is arc[arc:length - 1].
  if last["speed"] > speed_handoff {
    plan_abort("the arc spent its whole step budget with speed still to"
        + " burn; this craft cannot fly this ellipse down.").
  }

  // The overshoot allowance, from the model's self-measured error: the
  // same arc on half the step budget, differenced at the endpoint. The
  // flight controller derives the same number from the same march.
  local coarse_arc is integrate_arc(f, speed_handoff, 0, arc_steps / 2,
                                    nd:orbit).
  set allowance to model_error_margin
      * abs(last["x"] - coarse_arc[coarse_arc:length - 1]["x"]).
  // The trim gain, by finite difference, for the headroom check below.
  local probe_arc is integrate_arc(f + df_probe, speed_handoff, 0,
                                   arc_steps, nd:orbit).
  set x_shrink_per_f to (last["x"]
      - probe_arc[probe_arc:length - 1]["x"]) / df_probe.

  set x_arc to last["x"].
  set t_arc to last["t"].
  local h_new is h_handoff + x_arc * tan(gamma).
  // The lead places the endpoint one allowance LONG of the site: the
  // throttle trim can only pull the endpoint up-range, so every error the
  // plan books must sit on the far side.
  local lead_new is (x_arc - allowance) / body:radius * constant:radtodeg.
  set fine_passes to fine_passes + 1.
  print "fine " + fine_passes + ": h_pdi " + round(h_new) + " m  X "
      + round(x_arc / 1000, 1) + " km  f " + round(f, 4) + "  allowance "
      + round(allowance) + " m.".

  // Settled when the update moves the plan by less than the burn's own
  // slop (1 m of periapsis) and the placement loop's own tolerance
  // (0.2 deg of lead). The standing node was placed from the pre-update
  // values, which the criterion just certified as interchangeable.
  if abs(h_new - h_pdi) < 1 and abs(lead_new - lead_deg) < 0.2 {
    set converged to true.
  } else {
    set h_pdi to h_new.
    set lead_deg to lead_new.
    remove nd.
  }
}

local nd is fine["node"].

// Ignition leads the node by half the burn, and the ship needs time to
// swing onto the burn vector; a node closer than that will be burned late,
// which silently moves periapsis east. Failing is self-correcting: by the
// re-run this crossing has passed, and the next is most of an orbit out.
if nd:eta < burn_duration(nd:deltav:mag) / 2 + 60 {
  plan_abort("the burn is only " + round(nd:eta) + " s away — too close to"
      + " orient and ignite on time. Re-run for the next crossing.").
}

// === THE VERDICT ===

// The plane the node delivers, measured as the flight controller will
// measure it: the site's signed offset from the plane of the ground track.
// Two footprints bracketing PDI give the track's direction in the body
// frame — geoposition_at already carries the body's rotation — and the
// site, fixed in that frame, is dotted against the plane normal. 10 s of
// track is long enough to separate the footprints cleanly and short
// enough to be straight.
local t_pdi is fine["t_pdi"].
local u_pdi is (geoposition_at(t_pdi, nd:orbit):position
              - body:position):normalized.
local u_next is (geoposition_at(t_pdi + 10, nd:orbit):position
               - body:position):normalized.
local n_track is vcrs(u_next - u_pdi, u_pdi):normalized.
local cross_pdi is vdot(tgt:position - body:position, n_track).

// What the braking phase's yaw can null, measured at PDI: the lateral
// demand for a miss y is 6 y / t_go^2, the cap on lateral acceleration is
// the steering budget's, and t_go is the whole arc. Capacity falls as the
// burn shortens, so this is the best case.
local y_capacity is f * a_max * sqrt(2 * steering_loss_budget)
                 * t_arc ^ 2 / 6.
if abs(cross_pdi) > y_capacity {
  print "WARNING: the plane misses the site by more than the yaw budget"
      + " corrects. Fix the parking orbit's plane before burning this.".
}

// The band between the solved throttle and the ceiling is the trim's
// whole authority, in metres of endpoint. It must cover at least the
// allowance, because the plan deliberately arrives that far long and the
// trim must be able to pull it back.
local headroom is (f_max - f) * x_shrink_per_f.
if headroom < allowance {
  print "WARNING: trim headroom " + round(headroom) + " m is less than"
      + " the overshoot allowance " + round(allowance) + " m.".
}

// The price of gamma: the node's burn plus the braking arc by the rocket
// equation, at today's mass — the few kg the DOI burn spends first are
// inside the allowance. Terminal descent is extra and roughly constant.
local m0 is ship:mass.
local dv_doi is nd:deltav:mag.
local dv_arc is engine_isp() * constant:g0
              * ln(m0 / (m0 - f * mdot_full * t_arc)).

// The plan, printed and kept: doi_plan.log is the witness the flight is
// judged against.
local planlog is "doi_plan.log".
if exists(planlog) { deletepath(planlog). }
function report {
  parameter line.
  print line.
  log line to planlog.
}

report("# gamma " + round(gamma, 2) + " deg  target "
    + round(target_lat, 4) + " " + round(target_lng, 4) + "  terrain "
    + round(tgt:terrainheight) + " m").
report("# parking " + round(ship:orbit:periapsis) + " x "
    + round(ship:orbit:apoapsis) + " m  ecc "
    + round(ship:orbit:eccentricity, 4)).
report("# h_pdi " + round(h_pdi) + " m (node delivers "
    + round(nd:orbit:periapsis) + ")  X " + round(x_arc) + " m  lead "
    + round(lead_deg, 2) + " deg  passes " + coarse_iters + " coarse / "
    + fine_passes + " fine").
report("# f_solved " + round(f, 4) + "  headroom " + round(headroom)
    + " m  allowance " + round(allowance) + " m  arc " + round(t_arc, 1)
    + " s").
report("# dv  doi " + round(dv_doi, 1) + "  arc " + round(dv_arc, 1)
    + "  total " + round(dv_doi + dv_arc, 1) + " m/s (terminal excluded)").
report("# cross_pdi " + round(cross_pdi) + " m  yaw_capacity "
    + round(y_capacity) + " m").
report("# node  dv " + round(nd:deltav:mag, 1) + " m/s  eta "
    + round(nd:eta) + " s  pe_lng_err " + round(fine["err"], 2)
    + " deg in " + fine["attempts"] + " attempts").

set config:ipu to ipu_prior.
print "Node placed. Burn it, then run powered_descent.".
