// plan_doi.ks — mission planning for the powered descent: place the DOI node.
// Design: notes/capability-driven-descent.md (piece 2), revised to plan for
// the live re-solve flight controller (notes/powered-descent-invariants.md,
// flown as powered_descent_min.ks).
//
// Given gamma, the descent angle — the human's judgment, standing in for
// the terrain survey the smart planner will someday do — solve the PDI
// altitude that angle implies for this craft on this orbit, and leave the
// maneuver node whose burn delivers it. The node is the whole output. Burn
// it, then run powered_descent_min.ks, which reads the resulting ellipse
// and flies it down. Nothing here steers, burns, or warps.
//
// gamma is the slope, degrees above horizontal, of the straight line from
// the handoff point up to PDI. The flown arc leaves PDI level and steepens
// monotonically, so it is concave and lies above that line everywhere:
// terrain the line clears, the flight clears. Shallow gamma spends less
// delta-v; steep gamma clears more terrain. gamma is also the plan's whole
// fuel lever: the flight controller solves its throttle from live state
// and holds no margins of its own, so every metre per second the descent
// can save or waste is decided here — which is why the sweep below prices
// the judgment instead of leaving the trade a slogan.
//
// What is gone since the table-flying controller, and why: that controller
// froze a plan at PDI and trimmed toward it, so this file booked an
// overshoot allowance sized to the model's self-measured error, placed the
// endpoint deliberately long of the site, and checked the trim gain's
// headroom against the allowance. The live re-solve corrects both signs of
// error from fresh state every few seconds, so a plan that arrives long is
// no longer protection — it is just a burn flown above the solved throttle
// for its whole length. The endpoint is placed AT the site, and what
// margin-keeping remains is reported as what it is: the solved throttle's
// distance from its own bounds.

@lazyglobal off.

clearscreen.
print "=== PLAN DOI ===".

// common for engine_isp and burn_duration; kepler for orbital_speed,
// time_to_longitude, time_of_periapsis, geoposition_at and, through its
// own runoncepath, bisect. Both files define orbital_speed; kepler runs
// last so its (altitude, orbit) form — the one the arc march calls —
// survives.
run "common".
run "../core/kepler".

// The descent angle, degrees. No default: it is the one judgment this
// script cannot supply.
parameter gamma.
parameter target_lat is 0.
parameter target_lng is 0.
// The arc contract: everything from here down must match what
// powered_descent_min.ks is run with, or the descent priced here is not
// the descent flown. The reasoning behind each value lives with its twin
// there.
parameter landing_height is 50.
parameter speed_handoff is 5.
parameter f_max is 0.85.
parameter f_min is 0.05.

// The march's accuracy bounds — twins of the locals in
// powered_descent_min.ks, and locals here for the same reason they are
// there: accuracy bounds, not craft or body numbers. pitch_tol caps the
// flight-path rotation per Euler step (degrees); v_frac caps the
// fractional speed change per step.
local pitch_tol is 1.
local v_frac is 0.02.
// The throttle bisection's tolerance, matching the flight controller's
// solve_f; and how far the coarse tier loosens the march (both accuracy
// bounds scaled up, the tolerance by ten).
local f_eps is 0.001.
local coarse_scale is 5.

local tgt is body:geopositionlatlng(target_lat, target_lng).
// Altitude, above the datum, where the arc ends: landing_height above the
// site's terrain. The fixed point builds h_pdi on top of it. The flight
// controller never sees landing_height — it is spent into the ellipse
// here, which is the point.
local h_handoff is tgt:terrainheight + landing_height.
local a_max is ship:availablethrust / ship:mass.

// Planning is a few hundred marches of the arc; run them at the
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
// integrate_arc is powered_descent_min.ks's endpoint, nearly verbatim: the
// plan is only as good as its price, and the price is only right if the
// planner marches exactly the arc the flight controller will fly. Two
// departures, both at the seams: the seed is the candidate ellipse's
// periapsis instead of the live ship, and the accuracy bounds arrive as
// parameters so the coarse tier can loosen them. Until the two share a
// library, a change to either copy must be made in both.

function integrate_arc {
  parameter f.                    // throttle, as a fraction of full thrust
  parameter orbit_ is ship:orbit. // the descent ellipse the arc begins on
  parameter ptol_ is pitch_tol.
  parameter vfrac_ is v_frac.

  // The seed: periapsis is PDI's altitude, and the speed there is vis-viva
  // less the motion of the ground underneath, because the arc is flown
  // against the ground, not against the stars. Equatorial and prograde,
  // per the parking-orbit assumption.
  local h is orbit_:periapsis.
  local r_pe is orbit_:body:radius + h.
  local v0 is orbital_speed(h, orbit_)
            - 2 * constant:pi * r_pe / orbit_:body:rotationperiod.
  local speed is v0.
  local pitch is 0.        // degrees above the horizon; PDI is a periapsis
  local m is ship:mass.    // planning mass; the few kg the DOI burn spends
                           // first are absorbed by the live re-solve
  local theta is 0.        // ground angle swept, radians
  local t is 0.
  local steps is 0.
  until speed <= speed_handoff or h <= 0 or steps >= 4000 {
    local r_ is body:radius + h.
    local g is body:mu / r_ ^ 2.
    local turn is abs(speed / r_ - g / speed).
    local dt_angle is ptol_ / (max(1e-6, turn) * constant:radtodeg).
    local dt_speed is vfrac_ * speed / (f * ship:availablethrust / m + g).
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
  // speed rides along in the result because it is what tells a closed arc
  // from one that hit the step cap or the ground with speed still to burn.
  return lexicon("h", h, "x", theta * body:radius, "t", t,
                 "speed", speed, "v0", v0).
}

// The one throttle whose arc bottoms out at the handoff altitude, found by
// bisection: the candidate ellipse fixes PDI, so how hard the engine
// pushes is the only free variable, and the ending height rises
// monotonically with it. The same march the flight controller bisects on
// down-range, bisected here on altitude, because the planner's question is
// where the arc bottoms, not where it lands.
function solve_throttle {
  parameter orbit_ is ship:orbit.
  parameter ptol_ is pitch_tol.
  parameter vfrac_ is v_frac.
  parameter eps_ is f_eps.

  // Where the arc bottoms out, relative to where it should: negative when
  // the burn ran long and fell below the handoff, positive when it stopped
  // above it.
  local miss is {
    parameter f.
    local arc is integrate_arc(f, orbit_, ptol_, vfrac_).
    // Speed left with altitude left means the march hit its step cap while
    // still falling: wherever it stopped, its true bottom is lower. Report
    // it far below the handoff, which steers the search toward more
    // throttle. (Speed left at h <= 0 is a real impact below the handoff,
    // and the plain difference already says so.)
    if arc["speed"] > speed_handoff and arc["h"] > 0 { return -1e9. }
    return arc["h"] - h_handoff.
  }.
  // Bisection needs the answer bracketed, and it is: f_min ends below the
  // handoff and f_max above it. Returns -1 if that bracket does not hold,
  // which is the caller's abort.
  return bisect(miss, f_min, f_max, eps_).
}

// The price of an arc by the rocket equation, at today's mass — the few
// kg the DOI burn spends first shift it by less than the coarse tier's
// own error.
function arc_dv {
  parameter f_, t_.
  return engine_isp() * constant:g0
       * ln(ship:mass / (ship:mass - f_ * mdot_full * t_)).
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
// Two tiers. The coarse tier runs the march with its accuracy bounds
// loosened by coarse_scale and the throttle solve at ten times the
// tolerance; its errors are priced away by the fine passes that follow, so
// its only job is to hand the fine tier a nearby starting point — and, run
// at slopes the human did not ask for, to price the judgment (the sweep
// below). That second use is why it is a function that reports failure
// instead of aborting: a slope that fails to settle is a fact for the
// sweep to print, not a reason to stop.
function coarse_fixed_point {
  parameter gamma_.
  parameter verbose_ is false.

  // The seed. X = 0 would pose the first solve a degenerate ellipse —
  // periapsis at the handoff, nothing to descend through — whose bracket
  // can fail on a high-thrust craft. The shortest ground any braking arc
  // can cover is the stop distance at the throttle ceiling, so seed X
  // there: every pass then prices a real descent, approaching the answer
  // from below.
  local r_seed is body:radius + h_handoff.
  local sma_seed is (ship:orbit:semimajoraxis + r_seed) / 2.
  local v_seed is sqrt(body:mu * (2 / r_seed - 1 / sma_seed))
                - 2 * constant:pi * r_seed / body:rotationperiod.
  local x_seed is v_seed ^ 2 / (2 * f_max * a_max).
  local h_pdi_ is h_handoff + x_seed * tan(gamma_).
  local lead_ is x_seed / body:radius * constant:radtodeg.

  local iters is 0.
  local d_h is 1e9.
  local f_c is 0.
  local x_ is 0.
  local t_ is 0.
  local dv_doi_ is 0.

  until abs(d_h) < 1 {
    // Eight passes without settling means the map is not contracting here,
    // and more passes will not help.
    if iters >= 8 {
      return lexicon("ok", false, "why", "h_pdi moved " + round(abs(d_h))
          + " m on coarse pass 8; the fixed point is not settling").
    }
    local nd is plan_node(h_pdi_, lead_).
    add nd.
    if nd:eta <= 0 {
      remove nd.
      return lexicon("ok", false,
                     "why", "the DOI plan puts the burn in the past").
    }
    set f_c to solve_throttle(nd:orbit, pitch_tol * coarse_scale,
                              v_frac * coarse_scale, f_eps * 10).
    if f_c < 0 {
      remove nd.
      return lexicon("ok", false, "why", "no throttle between " + f_min
          + " and " + f_max + " flies the gamma " + round(gamma_, 2)
          + " ellipse (PDI " + round(h_pdi_) + " m) down to the handoff").
    }
    local arc is integrate_arc(f_c, nd:orbit, pitch_tol * coarse_scale,
                               v_frac * coarse_scale).
    set dv_doi_ to nd:deltav:mag.
    remove nd.

    set x_ to arc["x"].
    set t_ to arc["t"].
    local h_new is h_handoff + x_ * tan(gamma_).
    set d_h to h_new - h_pdi_.
    set h_pdi_ to h_new.
    set lead_ to x_ / body:radius * constant:radtodeg.
    set iters to iters + 1.
    if verbose_ {
      print "coarse " + iters + ": h_pdi " + round(h_pdi_) + " m  X "
          + round(x_ / 1000, 1) + " km  f " + round(f_c, 3) + ".".
    }
  }
  return lexicon("ok", true, "h_pdi", h_pdi_, "lead", lead_, "f", f_c,
                 "x", x_, "t", t_, "dv_doi", dv_doi_, "iters", iters).
}

local coarse is coarse_fixed_point(gamma, true).
if not coarse["ok"] {
  plan_abort(coarse["why"] + ". Re-think gamma or the parking orbit.").
}
local h_pdi is coarse["h_pdi"].
local lead_deg is coarse["lead"].
local coarse_iters is coarse["iters"].

// === THE PRICE OF GAMMA ===
// gamma is the plan's one fuel lever, so price the judgment: the same
// coarse fixed point at a slope shallower and one steeper than asked, each
// with its DOI burn and its braking arc through the rocket equation.
// Advisory only — nothing below reads these numbers — and coarse, so read
// the differences, not the digits.
local sweep_lines is list().
for mult in list(0.75, 1, 1.5) {
  local g_try is gamma * mult.
  if g_try > 0 and g_try < 90 {
    local r is coarse.
    if mult <> 1 { set r to coarse_fixed_point(g_try). }
    local line is "".
    if r["ok"] {
      set line to "# gamma " + round(g_try, 2) + " deg: dv "
          + round(r["dv_doi"] + arc_dv(r["f"], r["t"]), 1) + " m/s (doi "
          + round(r["dv_doi"], 1) + " + arc "
          + round(arc_dv(r["f"], r["t"]), 1) + ")  f " + round(r["f"], 3)
          + "  X " + round(r["x"] / 1000, 1) + " km"
          + (choose "  <- planned" if mult = 1 else "").
    } else {
      set line to "# gamma " + round(g_try, 2) + " deg: no plan ("
          + r["why"] + ")".
    }
    print line.
    sweep_lines:add(line).
  }
}

// The fine tier: flight fidelity, with the full placement feedback. Pass 1
// refines the coarse tier's X; pass 2 confirms it. Three passes without
// settling means something is inconsistent, not merely unconverged.
local f is 0.
local x_arc is 0.
local t_arc is 0.
local v_pdi is 0.
local fine is 0.
local fine_passes is 0.
local converged is false.

until converged {
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
  local arc is integrate_arc(f, nd:orbit).
  if arc["speed"] > speed_handoff {
    plan_abort("the arc hit its step cap or the ground with speed still to"
        + " burn; this craft cannot fly this ellipse down.").
  }

  set x_arc to arc["x"].
  set t_arc to arc["t"].
  set v_pdi to arc["v0"].
  local h_new is h_handoff + x_arc * tan(gamma).
  // The lead places the endpoint AT the site. The flight controller's
  // re-solve corrects both signs of error, so nothing is bought by
  // arriving long — the old allowance would only hold the flown throttle
  // above the solved one for the whole burn.
  local lead_new is x_arc / body:radius * constant:radtodeg.
  set fine_passes to fine_passes + 1.
  print "fine " + fine_passes + ": h_pdi " + round(h_new) + " m  X "
      + round(x_arc / 1000, 1) + " km  f " + round(f, 4) + ".".

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

// What the braking phase's yaw does with that offset. Its law: pretend the
// ship owes a sideways speed of cross/tau toward the site's plane and null
// it into the retrograde hold. Priced two ways, both against the law
// actually flown. The bias angle at PDI is the demanded sideways speed
// over the forward speed — past a few degrees the yaw stops being the
// cheap correction the design assumes, and the parking orbit's plane is
// the thing to fix. The residual is the offset a tau-constant closure
// leaves after the whole burn — if the burn is short against tau, the
// plane never closes and terminal's tilt walk inherits the rest.
local tau_yaw is 20.               // braking_dir's closing time constant
local bias_deg is arctan(abs(cross_pdi) / tau_yaw / v_pdi).
local cross_res is abs(cross_pdi) * constant:e ^ (-t_arc / tau_yaw).
if bias_deg > 5 {
  print "WARNING: the plane demands a " + round(bias_deg, 1) + " deg yaw"
      + " bias at PDI. Fix the parking orbit's plane before burning this.".
}
if cross_res > 5 {
  print "WARNING: the burn is short against the yaw's " + tau_yaw + " s"
      + " closure; about " + round(cross_res) + " m of cross-track will"
      + " remain at handoff.".
}

// The solved throttle's distance from its bounds is the trim's whole
// authority: room above absorbs overshoot (raising f shortens the arc),
// room below absorbs undershoot. Thin on either side means dispersions
// this size go uncorrected.
local f_band is f_max - f_min.
if f_max - f < 0.1 * f_band {
  print "WARNING: f_solved " + round(f, 3) + " sits within 10% of f_max;"
      + " little authority remains to shorten the arc.".
}
if f - f_min < 0.1 * f_band {
  print "WARNING: f_solved " + round(f, 3) + " sits within 10% of f_min;"
      + " little authority remains to stretch the arc.".
}

// The price of gamma: the node's burn plus the braking arc by the rocket
// equation, at today's mass. Terminal descent is extra and roughly
// constant.
local dv_doi is nd:deltav:mag.
local dv_arc is arc_dv(f, t_arc).

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
report("# f_solved " + round(f, 4) + "  margin " + round(f_max - f, 3)
    + " above / " + round(f - f_min, 3) + " below  arc " + round(t_arc, 1)
    + " s").
report("# dv  doi " + round(dv_doi, 1) + "  arc " + round(dv_arc, 1)
    + "  total " + round(dv_doi + dv_arc, 1) + " m/s (terminal excluded)").
report("# cross_pdi " + round(cross_pdi) + " m  bias_pdi "
    + round(bias_deg, 2) + " deg  residual " + round(cross_res, 1) + " m").
report("# node  dv " + round(nd:deltav:mag, 1) + " m/s  eta "
    + round(nd:eta) + " s  pe_lng_err " + round(fine["err"], 2)
    + " deg in " + fine["attempts"] + " attempts").
// The sweep, into the log as well: the prices the judgment was made
// against belong with the plan they produced.
for l in sweep_lines { log l to planlog. }

set config:ipu to ipu_prior.
print "Node placed. Burn it, then run powered_descent_min.".
