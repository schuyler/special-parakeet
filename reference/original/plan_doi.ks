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
// distance below the ceiling.

@lazyglobal off.

clearscreen.
print "=== PLAN DOI ===".

// common for engine_isp and burn_duration; kepler for orbital_speed,
// time_to_longitude, time_of_periapsis, geoposition_at. Both files define
// orbital_speed; kepler runs first so its (altitude, orbit) form — the
// one the arc march calls — survives.
run "../core/kepler".
run "common".

// The descent angle, degrees. No default: it is the one judgment this
// script cannot supply.
parameter gamma.
parameter target_lat is 0.
parameter target_lng is 0.
// The arc contract: speed_handoff and f_max must match what
// powered_descent_min.ks is run with, or the descent priced here is not
// the descent flown. landing_height is the planner's own — the handoff
// clearance it spends into the ellipse; the flight controller never sees
// the number, which is the design.
parameter landing_height is 50.
parameter speed_handoff is 5.
parameter f_max is 0.85.
// The coast's clearance floor, metres: the walk below the placement
// passes refuses the plan if the ellipse ever comes closer than this to
// the terrain between the DOI burn and PDI. Any negative (the default)
// means landing_height — the same benefit of the doubt the descent
// grants terrain everywhere else — resolved in code below rather than
// by a cross-parameter default expression.
parameter coast_clearance is -1.
// The gamma sweep prices the judgment at two slopes the human did not ask
// for — two more complete fixed points, which roughly triples the
// planning time. Advisory output only, so it is bought explicitly.
parameter do_sweep is false.
if coast_clearance < 0 { set coast_clearance to landing_height. }

// The march's accuracy bounds — twins of the locals in
// powered_descent_min.ks, and locals here for the same reason they are
// there: accuracy bounds, not craft or body numbers. pitch_tol caps the
// flight-path rotation per Euler step (degrees); v_frac caps the
// fractional speed change per step.
local pitch_tol is 1.
local v_frac is 0.02.

// The search's step scale. Throwaway marches inside the fixed-point and
// sweep loops run pitch_tol and v_frac this many times coarser than
// flight fidelity: their answers only steer the next pass, and the loops
// settle to x_tol, far above what the coarse march smears. The committed
// plan — the placement loop and everything reported — marches at scale 1,
// so nothing the flight inherits is priced coarse.
local search_scale is 4.

// Under-relaxation for the search. A coarse march steepens the fixed-point
// map — at scale 4 its slope near the answer runs about -1, so the raw
// iteration overshoots and oscillates instead of settling. Damping the update
// toward the marched answer, x <- x + step*(M(x) - x), pulls the effective
// slope to (1-step) + step*M', near zero at step 0.5 and M' ~ -1. But a full
// step is right while the move keeps its sign — that is healthy contraction,
// and damping it only adds passes — so the loop takes full steps until the
// move reverses and applies relax only then, on the overshoot the coarse map
// creates. The search converges to the coarse map's own fixed point; the
// scale-1 placement loop refines that to flight fidelity.
local relax is 0.5.
local search_budget is 12.

// How close in down-range reach the fixed-point and placement loops must settle
// before they stop. Not a precision of the plan but the slack in braking's
// reachable corridor: braking re-solves its throttle from live state and
// discards this plan's seed on its first pass, so a plan that lands PDI within
// x_tol of its own fixed point lands it well inside what the flight absorbs. Set
// above the Euler march's reach resolution (its last-step ground advance) so the
// loops stop at the descent's scale rather than banging against the integrator's
// floor. Provisional: a flight refines it.
local x_tol is 100.

local tgt is body:geopositionlatlng(target_lat, target_lng).
// Altitude, above the datum, where the arc ends: landing_height above the
// site's terrain. The fixed point builds h_pdi on top of it. The flight
// controller never sees landing_height — it is spent into the ellipse
// here, which is the point.
local h_handoff is tgt:terrainheight + landing_height.
local a_max is ship:availablethrust / ship:mass.

// Planning is a few dozen marches of the arc (a few hundred with the
// sweep); run them at the processor's ceiling and put the setting back
// on the way out.
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
// departures: the seed is the candidate ellipse's periapsis instead of
// the live ship, and the step tolerances take a scale so the search
// loops can march coarse — at scale 1 the physics is the flight's,
// verbatim. Until the two share a library, a change to either copy must
// be made in both.

function integrate_arc {
  parameter f.                    // throttle, as a fraction of full thrust
  parameter orbit_ is ship:orbit. // the descent ellipse the arc begins on
  parameter scale_ is 1.          // multiplies the step tolerances; 1 is
                                  // flight fidelity, search runs coarser

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
  // Every suffix chain below re-resolves each step in kOS, and the march
  // is the plan's unit of cost, so the loop's invariants live in locals.
  local thrust_f is f * ship:availablethrust.
  local mdot_f is f * mdot_full.
  local mu_ is body:mu.
  local r_body is body:radius.
  local h_floor is tgt:terrainheight.
  local r2d is constant:radtodeg.
  local ptol is pitch_tol * scale_.
  local vtol is v_frac * scale_.
  // The floor is the site's terrain, not the datum, matching the flight
  // controller: a throttle too weak to stop runs into ground that exists.
  until speed <= speed_handoff or h <= h_floor or steps >= 4000 {
    local r_ is r_body + h.
    local g is mu_ / r_ ^ 2.
    local turn is abs(speed / r_ - g / speed).
    local dt_angle is ptol / (max(1e-6, turn) * r2d).
    local dt_speed is vtol * speed / (thrust_f / m + g).
    local dt is min(dt_angle, dt_speed).
    local d_speed is (-(thrust_f / m) - g * sin(pitch)) * dt.
    local d_pitch is (speed / r_ - g / speed) * cos(pitch) * r2d * dt.
    set h     to h     + speed * sin(pitch) * dt.
    set theta to theta + speed * cos(pitch) / r_ * dt.
    set speed to speed + d_speed.
    set pitch to pitch + d_pitch.
    set m     to m     - mdot_f * dt.
    set t     to t     + dt.
    set steps to steps + 1.
  }
  // speed rides along in the result because it is what tells a closed arc
  // from one that hit the step cap or the ground with speed still to burn.
  return lexicon("h", h, "x", theta * r_body, "t", t,
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
  parameter f_seed is -1.   // the previous pass's answer, if there was one:
                            // the solve barely moves between passes, so a
                            // narrow bracket around it saves most halvings
  parameter scale_ is 1.    // step scale, handed through to the march

  // Where the arc bottoms out, relative to where it should: negative when
  // the burn ran long and fell below the handoff, positive when it stopped
  // above it.
  local miss is {
    parameter f.
    local arc is integrate_arc(f, orbit_, scale_).
    // Speed left with altitude left means the march hit its step cap while
    // still falling: wherever it stopped, its true bottom is lower. Report
    // it far below the handoff, which steers the search toward more
    // throttle. (Speed left at the terrain floor is a real impact below
    // the handoff, and the plain difference already says so.)
    if arc["speed"] > speed_handoff and arc["h"] > tgt:terrainheight {
      return -1e9.
    }
    return arc["h"] - h_handoff.
  }.
  // The bisection, priced in marches — each endpoint probe and each
  // halving costs a full arc march, which is why this does not call the
  // library bisect (it marches both endpoints to check a bracket this
  // function already knows). miss(0) is negative by construction: no
  // thrust runs the march into the terrain or its step cap, both reported
  // short. So the low end is never marched, and the ceiling is marched
  // only when no seed bracket holds — where miss(f_max) <= 0 is the real
  // failure, no throttle up to f_max flying the ellipse down, returned as
  // -1 for the caller. A seed shrinks the bracket to the 40% window
  // around the old answer when the window's signs hold; when they do not,
  // the two probes still tighten one end. The tolerance is f_max/128,
  // ~7e-3 of full scale: the flight's own re-solve bisects to 0.001 from
  // true state, so the planner has no reason to out-resolve it.
  local eps is f_max / 1024.
  local lo is 0.
  local hi is -1.
  if f_seed > 0 {
    local lo_try is 0.8 * f_seed.
    local hi_try is min(f_max, 1.2 * f_seed).
    if miss(lo_try) < 0 {
      if miss(hi_try) > 0 {
        set lo to lo_try.
        set hi to hi_try.
      } else if hi_try >= f_max {
        return -1.      // the window's top was already the ceiling
      } else {
        set lo to hi_try.
      }
    } else {
      set hi to lo_try.
    }
  }
  if hi < 0 {
    if miss(f_max) <= 0 { return -1. }
    set hi to f_max.
  }
  until hi - lo < eps {
    local c is (lo + hi) / 2.
    if miss(c) > 0 { set hi to c. } else { set lo to c. }
  }
  return (lo + hi) / 2.
}

// The price of an arc by the rocket equation, at today's mass — the few
// kg the DOI burn spends first shift it by less than the burn's own slop.
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
// One tier, at flight fidelity: the adaptive march is cheap enough that
// every pass prices exactly the arc the ship will fly, so the coarse
// tier that used to hand this loop a starting point priced nothing worth
// keeping. The fixed point places its candidate nodes with plan_node's
// equatorial shortcut; the placement passes that follow correct what the
// shortcut books. It is a function that reports failure instead of
// aborting because the sweep below runs it at slopes the human did not
// ask for, and a slope that fails to settle is a fact for the sweep to
// print, not a reason to stop.
function fixed_point {
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
  local x_ is x_seed.
  local dx is 1e9.
  local f_c is 0.
  local t_ is 0.
  local dv_doi_ is 0.

  // Settle the endpoint's down-range move, not the PDI altitude: the two differ
  // by tan(gamma) (d_h = dx * tan(gamma)), so an altitude tolerance quietly
  // tightens as gamma shallows, while what the descent cares about is where the
  // arc ends. x_tol is the corridor slack, argued where it is defined.
  until abs(dx) < x_tol {
    // The march has a reach floor; below it dx stops shrinking. Reaching the
    // budget within the floor (dx small) means x_tol sits below the floor for
    // this gamma: accept the floored point — it is inside the corridor — rather
    // than banging on. Reaching it still moving in large steps means the map is
    // not settling here, which is a real failure.
    if iters >= search_budget {
      if abs(dx) >= 4 * x_tol {
        return lexicon("ok", false, "why", "the endpoint moved "
            + round(abs(dx)) + " m in " + search_budget
            + " passes; the fixed point is not settling").
      }
      break.
    }
    local nd is plan_node(h_pdi_, lead_).
    add nd.
    if nd:eta <= 0 {
      remove nd.
      return lexicon("ok", false,
                     "why", "the DOI plan puts the burn in the past").
    }
    // Coarse marches and a warm seed: pass 1 solves cold (f_c is 0), the
    // rest bracket around the last answer.
    set f_c to solve_throttle(nd:orbit, f_c, search_scale).
    if f_c < 0 {
      remove nd.
      return lexicon("ok", false, "why", "no throttle up to " + f_max
          + " flies the gamma " + round(gamma_, 2) + " ellipse (PDI "
          + round(h_pdi_) + " m) down to the handoff").
    }
    local arc is integrate_arc(f_c, nd:orbit, search_scale).
    set dv_doi_ to nd:deltav:mag.
    remove nd.

    local dx_new is arc["x"] - x_.
    // Full step while the move keeps its sign; damp when it reverses, which is
    // the coarse map overshooting its fixed point. dx carries the raw move for
    // the convergence test either way.
    local step is choose relax if dx_new * dx < 0 else 1.
    set dx to dx_new.
    set x_ to x_ + step * dx_new.
    set t_ to arc["t"].
    set h_pdi_ to h_handoff + x_ * tan(gamma_).
    set lead_ to x_ / body:radius * constant:radtodeg.
    set iters to iters + 1.
    if verbose_ {
      print "pass " + iters + ": h_pdi " + round(h_pdi_) + " m  X "
          + round(x_ / 1000, 1) + " km  f " + round(f_c, 3) + ".".
    }
  }
  return lexicon("ok", true, "h_pdi", h_pdi_, "lead", lead_, "f", f_c,
                 "x", x_, "t", t_, "dv_doi", dv_doi_, "iters", iters).
}

local fp is fixed_point(gamma, true).
if not fp["ok"] {
  plan_abort(fp["why"] + ". Re-think gamma or the parking orbit.").
}
local h_pdi is fp["h_pdi"].
local lead_deg is fp["lead"].
local fp_iters is fp["iters"].

// === THE PRICE OF GAMMA ===
// gamma is the plan's one fuel lever, so price the judgment: the same
// fixed point at a slope shallower and one steeper than asked, each with
// its DOI burn and its braking arc through the rocket equation. Advisory
// only — nothing below reads these numbers — and two more fixed points
// is most of another planning run, so it waits for do_sweep.
local sweep_lines is list().
local sweep_mults is choose list(0.75, 1, 1.5) if do_sweep else list().
for mult in sweep_mults {
  local g_try is gamma * mult.
  if g_try > 0 and g_try < 90 {
    local r_ is fp.
    if mult <> 1 { set r_ to fixed_point(g_try). }
    local line is "".
    if r_["ok"] {
      set line to "# gamma " + round(g_try, 2) + " deg: dv "
          + round(r_["dv_doi"] + arc_dv(r_["f"], r_["t"]), 1) + " m/s (doi "
          + round(r_["dv_doi"], 1) + " + arc "
          + round(arc_dv(r_["f"], r_["t"]), 1) + ")  f " + round(r_["f"], 3)
          + "  X " + round(r_["x"] / 1000, 1) + " km"
          + (choose "  <- planned" if mult = 1 else "").
    } else {
      set line to "# gamma " + round(g_try, 2) + " deg: no plan ("
          + r_["why"] + ")".
    }
    print line.
    sweep_lines:add(line).
  }
}

// The placement passes: the fixed point placed its candidate nodes with
// plan_node's equatorial shortcut; place_node now measures where
// periapsis really falls and feeds the miss back. Pass 1 corrects what
// the shortcut booked; pass 2 confirms it. Three passes without settling
// means something is inconsistent, not merely unconverged.
// Both seeds come from the fixed point: the throttle warm-starts each
// placement solve, and the reach measures pass 1's move off the shortcut.
// The placement's marches run at scale 1 — flight fidelity — because this
// loop's answer is the plan.
local f is fp["f"].
// Seed the reach from the fixed point's answer (h_pdi = h_handoff + x*tan gamma),
// so pass 1's move measures how far the real placement shifts the reach off the
// shortcut's estimate.
local x_arc is (h_pdi - h_handoff) / tan(gamma).
local t_arc is 0.
local v_pdi is 0.
local placed is 0.
local place_passes is 0.
local place_dx is 1e9.
local converged is false.

until converged {
  set placed to place_node(h_pdi, lead_deg).
  local nd is placed["node"].

  set f to solve_throttle(nd:orbit, f).
  if f < 0 {
    plan_abort("on the placed ellipse, no throttle up to " + f_max
        + " flies the arc down to the handoff.").
  }
  local arc is integrate_arc(f, nd:orbit).
  if arc["speed"] > speed_handoff {
    plan_abort("the arc hit its step cap or the ground with speed still to"
        + " burn; this craft cannot fly this ellipse down.").
  }

  set place_dx to arc["x"] - x_arc.
  set x_arc to arc["x"].
  set t_arc to arc["t"].
  set v_pdi to arc["v0"].
  local h_new is h_handoff + x_arc * tan(gamma).
  // The lead places the endpoint AT the site. The flight controller's
  // re-solve corrects both signs of error, so nothing is bought by
  // arriving long — the old allowance would only hold the flown throttle
  // above the solved one for the whole burn.
  local lead_new is x_arc / body:radius * constant:radtodeg.
  set place_passes to place_passes + 1.
  print "placement " + place_passes + ": h_pdi " + round(h_new) + " m  X "
      + round(x_arc / 1000, 1) + " km  f " + round(f, 4) + ".".

  // Settled when the placed endpoint stops moving by more than the corridor
  // slack (x_tol, subsuming the old lead tolerance: 100 m of reach is under
  // 0.01 deg of lead). The standing node was placed from the pre-update values,
  // which the criterion just certified as interchangeable. Same reach floor as
  // the fixed point, so the same budget rule: after three passes accept a floored
  // point rather than aborting; abort only if it is still moving in large steps.
  if abs(place_dx) < x_tol {
    set converged to true.
  } else if place_passes >= 3 {
    if abs(place_dx) >= 4 * x_tol {
      remove nd.
      plan_abort("the placement endpoint still moved " + round(abs(place_dx))
          + " m on pass 3; the placement is not settling.").
    }
    set converged to true.
  } else {
    set h_pdi to h_new.
    set lead_deg to lead_new.
    remove nd.
  }
}

local nd is placed["node"].

// Ignition leads the node by half the burn, and the ship needs time to
// swing onto the burn vector; a node closer than that will be burned late,
// which silently moves periapsis east. Failing is self-correcting: by the
// re-run this crossing has passed, and the next is most of an orbit out.
if nd:eta < burn_duration(nd:deltav:mag) / 2 + 60 {
  plan_abort("the burn is only " + round(nd:eta) + " s away — too close to"
      + " orient and ignite on time. Re-run for the next crossing.").
}

// === THE COAST ===
// The one stretch the gamma ray never certifies: between the DOI burn
// and PDI the ship rides the ellipse over terrain the plan has so far
// only assumed away — and it binds for real: measured over the Great
// Flats, the coast's clearance beat PDI's by ten metres, on the
// flattest ground the body owns. The check is nearly exact, because the
// coast is on rails and kOS terrain is the game's own ground: walk the
// placed ellipse from the burn to PDI, keep the minimum of altitude
// over terrain, refuse the plan if it comes under coast_clearance. No
// early-out cleverness — half an orbit of samples is cheap at this
// IPU. The step is anchored to ground metres at periapsis speed, where
// the coast is lowest and fastest, so it is an accuracy bound like
// pitch_tol, not a craft or body number.
local coast_dx is 200.
local t_node is timestamp(nd:time).
local t_pdi is placed["t_pdi"].
local dt_c is coast_dx / orbital_speed(nd:orbit:periapsis, nd:orbit).
local cc_samples is floor((t_pdi:seconds - t_node:seconds) / dt_c).
print "Walking the coast: " + cc_samples + " samples.".

local cc_min is 1e12.
local cc_dt is 0.      // seconds before PDI — open item 1's own coordinate
local cc_alt is 0.
local cc_terr is 0.
local cc_lng is 0.
local i is 0.
until t_pdi:seconds - i * dt_c < t_node:seconds {
  local t_i is timestamp(t_pdi:seconds - i * dt_c).
  local st is orbit_at(t_i, nd:orbit).
  local alt_i is st["position"]:mag - body:radius.
  local geo is geoposition_at(t_i, nd:orbit, st["position"]).
  if alt_i - geo:terrainheight < cc_min {
    set cc_min to alt_i - geo:terrainheight.
    set cc_dt to i * dt_c.
    set cc_alt to alt_i.
    set cc_terr to geo:terrainheight.
    set cc_lng to geo:lng.
  }
  set i to i + 1.
}
if cc_min < coast_clearance {
  plan_abort("the coast dips to " + round(cc_min) + " m over the terrain "
      + round(cc_dt) + " s before PDI (ellipse " + round(cc_alt)
      + " m, terrain " + round(cc_terr) + " m, lng " + round(cc_lng, 2)
      + "); the floor is " + coast_clearance + " m. If the dip is near"
      + " PDI, steepen gamma or move the site; if it sits well up-range"
      + " toward the burn, the parking orbit itself is too low for this"
      + " terrain.").
}

// === THE VERDICT ===

// The plane the node delivers, measured as the flight controller will
// measure it: the site's signed offset from the plane of the ground track.
// Two footprints bracketing PDI give the track's direction in the body
// frame — geoposition_at already carries the body's rotation — and the
// site, fixed in that frame, is dotted against the plane normal. 10 s of
// track is long enough to separate the footprints cleanly and short
// enough to be straight.
local u_pdi is (geoposition_at(t_pdi, nd:orbit):position
              - body:position):normalized.
local u_next is (geoposition_at(t_pdi + 10, nd:orbit):position
               - body:position):normalized.
local n_track is vcrs(u_next - u_pdi, u_pdi):normalized.
local cross_pdi is vdot(tgt:position - body:position, n_track).

// What the braking phase's yaw does with that offset. Its law: pretend
// the ship owes a sideways speed of cross/tau toward the site's plane and
// null it into the retrograde hold, tau being a third of the burn frozen
// at ignition. That construction fixes the closure — e^-3, five percent,
// of the PDI offset survives to handoff, where terminal's tilt walk
// inherits it — but not the price: the bias angle at PDI is the demanded
// sideways speed over the forward speed, and past a few degrees the yaw
// stops being the cheap correction the design assumes. Either warning
// means fix the parking orbit's plane, not the bias.
local tau_yaw is t_arc / 3.        // braking_dir's twin: frozen at ignition
local bias_deg is arctan(abs(cross_pdi) / tau_yaw / v_pdi).
local cross_res is abs(cross_pdi) * constant:e ^ (-t_arc / tau_yaw).
if bias_deg > 5 {
  print "WARNING: the plane demands a " + round(bias_deg, 1) + " deg yaw"
      + " bias at PDI. Fix the parking orbit's plane before burning this.".
}
if cross_res > 5 {
  print "WARNING: the five-percent residual of this plane offset is "
      + round(cross_res) + " m at handoff — more than terminal's tilt"
      + " walk should inherit.".
}

// The gap between the solved throttle and the ceiling is the trim's
// authority to shorten the arc — the side that absorbs an overshoot.
// Room below needs no twin warning: it is f itself, and a plan that
// solved absurdly low announces itself in the delta-v report.
if f_max - f < 0.1 * f_max {
  print "WARNING: f_solved " + round(f, 3) + " sits within 10% of f_max;"
      + " little authority remains to shorten the arc.".
}

// Terminal's contract at the seam: its schedule must be able to arrest
// the handoff speed inside the clearance, or the burn ignites already
// behind schedule and spends the f_max..1 reserve at the worst moment.
// The arrestable speed is sqrt(2 a_dec landing_height), a_dec being
// f_max's deceleration net of surface gravity — at today's mass, so
// slightly understated, and ignoring terminal's short h_pad coast, so
// slightly overstated; the planner is the only program that holds both
// numbers, which is why the check lives here and not in flight.
local a_dec_plan is f_max * a_max - body:mu / body:radius ^ 2.
local v_arrest is sqrt(2 * max(0, a_dec_plan) * landing_height).
if speed_handoff > v_arrest {
  print "WARNING: terminal ignites behind schedule at handoff — "
      + round(speed_handoff, 1) + " m/s arrives, only "
      + round(v_arrest, 1) + " m/s is arrestable in " + landing_height
      + " m of clearance. Raise landing_height or lower speed_handoff.".
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
    + round(lead_deg, 2) + " deg  passes " + fp_iters + " fixed-point / "
    + place_passes + " placement").
report("# f_solved " + round(f, 4) + "  margin " + round(f_max - f, 3)
    + " below f_max  arc " + round(t_arc, 1) + " s").
report("# coast  min clearance " + round(cc_min) + " m at " + round(cc_dt)
    + " s before PDI (ellipse " + round(cc_alt) + " m, terrain "
    + round(cc_terr) + " m, lng " + round(cc_lng, 2) + ")  floor "
    + coast_clearance + " m  " + cc_samples + " samples").
report("# dv  doi " + round(dv_doi, 1) + "  arc " + round(dv_arc, 1)
    + "  total " + round(dv_doi + dv_arc, 1) + " m/s (terminal excluded)").
report("# cross_pdi " + round(cross_pdi) + " m  bias_pdi "
    + round(bias_deg, 2) + " deg  residual " + round(cross_res, 1) + " m").
report("# node  dv " + round(nd:deltav:mag, 1) + " m/s  eta "
    + round(nd:eta) + " s  pe_lng_err " + round(placed["err"], 2)
    + " deg in " + placed["attempts"] + " attempts").
// The sweep, into the log as well: the prices the judgment was made
// against belong with the plan they produced.
for l in sweep_lines { log l to planlog. }

set config:ipu to ipu_prior.
print "Node placed. Burn it, then run powered_descent_min.".
