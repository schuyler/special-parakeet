// plan_doi.ks — mission planning for the powered descent: place the DOI node.
// Design: notes/gamma-free-planner.md, replacing the human-judged descent
// angle with a derived one. Plans for the live re-solve flight controller
// (notes/powered-descent-invariants.md, flown as powered_descent_min.ks).
//
// h_pdi, the PDI altitude, is not asked of the pilot. It is the largest of
// three demands, each an altitude: the coast demand (the ellipse must clear
// the terrain between the DOI burn and PDI by coast_clearance), the chord
// demand (the straight line from the handoff up to PDI must clear every
// obstacle under the arc's own footprint by terrain_margin), and the
// capability demand (the throttle the descent solves for must leave
// f_headroom of authority below f_max for the flight controller's live
// re-solve). Whichever is largest sets h_pdi; the plan's verdict names it,
// the way a terrain survey names its forcing obstacle. The node is the
// whole output. Burn it, then run powered_descent_min.ks, which reads the
// resulting ellipse and flies it down. Nothing here steers, burns, or
// warps.
//
// gamma, the chord's slope, is reported but never asked for:
// arctan((h_pdi - h_handoff) / X), X the arc's down-range reach. It is the
// floor under everything the flown arc does above it, certified by a
// two-span argument. While the arc sits at or above h_pdi it clears the
// chord trivially, h_pdi being the chord's own greatest height. Once the
// arc has descended back through h_pdi it is sub-circular and stays
// sub-circular, so its turn rate is negative at every throttle, the path
// is concave, and it rides above the straight segment whose ends sit on
// the chord — the re-crossing at h_pdi, the endpoint at the handoff.
//
// coast_clearance is the plan's one fuel lever: the flight controller
// solves its throttle from live state and holds no margins of its own, so
// every metre per second the descent can save or waste is decided by how
// much clearance the pilot demands — which is why the sweep below prices
// the judgment instead of leaving the trade a slogan.
//
// The endpoint is placed at the site: the flight controller re-solves its
// throttle from live state every few seconds and corrects both signs of
// error, so the plan holds no overshoot allowance. What margin remains is
// reported as what it is: the solved throttle's distance below the
// ceiling.

@lazyglobal off.

clearscreen.
print "=== PLAN DOI ===".

// common for engine_isp and burn_duration; kepler for orbital_speed,
// time_to_longitude, time_of_periapsis, geoposition_at, true_anomaly,
// body_rotation, wrap_longitude. Both files define orbital_speed; kepler
// runs first so its (altitude, orbit) form — the one the arc march calls —
// survives.
run "../core/kepler".
run "common".

parameter target_lat is 0.
parameter target_lng is 0.
// The arc contract: f_max must match the authority ceiling
// powered_descent_min.ks is run with, or the throttle priced here is not
// the throttle flown. speed_handoff is where the planner stops pricing
// the arc; the flight controller brakes to its attitude seam and re-solves
// from live state, so the two ends need only describe the same descent.
// landing_height is the planner's own — the handoff clearance it spends
// into the ellipse; the flight controller never sees the number, which is
// the design.
parameter landing_height is 50.
parameter speed_handoff is 5.
parameter f_max is 0.85.
// The coast's clearance floor, metres: the walk below the placement
// passes refuses the plan if the ellipse ever comes closer than this to
// the terrain between the DOI burn and PDI. It is also the seed and, most
// of the time, the binding demand on h_pdi — the plan's one dial. Any
// negative (the default) means landing_height — the same benefit of the
// doubt the descent grants terrain everywhere else — resolved in code
// below rather than by a cross-parameter default expression.
parameter coast_clearance is -1.
// How far the terrain model is trusted under the chord, metres: every
// sample the chord walk reads is treated as this much taller than kOS
// reports. Any negative (the default) means landing_height — up-range
// terrain gets the same benefit of the doubt as the clearance granted at
// the site, one judgment, not two — until the model earns a number of its
// own.
parameter terrain_margin is -1.
// The share of the throttle ceiling the solve refuses to spend, so the
// flight controller's live re-solve keeps room to shorten the arc if it
// needs to. Dimensionless: 0.1 means the solve accepts no plan whose
// throttle solves above 90% of f_max, raising h_pdi and spending delta-v
// to buy the reserve back when the craft's capability is what binds.
parameter f_headroom is 0.1.
// The sweep prices the judgment at floors the pilot did not ask for —
// three more complete solves, which roughly quadruples the planning time.
// Advisory output only, so it is bought explicitly.
parameter do_sweep is false.
if coast_clearance < 0 { set coast_clearance to landing_height. }
if terrain_margin < 0 { set terrain_margin to landing_height. }

// The march's accuracy bounds — twins of the locals in
// powered_descent_min.ks, and locals here for the same reason they are
// there: accuracy bounds, not craft or body numbers. pitch_tol caps the
// flight-path rotation per Euler step (degrees); v_frac caps the
// fractional speed change per step.
local pitch_tol is 1.
local v_frac is 0.02.

// The search's step scale. Throwaway marches inside the solve and sweep
// loops run pitch_tol, v_frac, and the coast walk's step this many times
// coarser than flight fidelity: their answers only steer the next pass,
// and the loops settle to h_tol and the endpoint tolerances, far above what the coarse
// march smears. The committed plan — the placement pass and everything
// reported — marches at scale 1, so nothing the flight inherits is priced
// coarse.
local search_scale is 4.

// Under-relaxation for the solve. Each pass's marched reach and coast
// clearance carry the throttle solve's tolerance as noise, which rules
// out slope-estimating accelerators. The iteration is the map's own step,
// damped only when the move reverses sign — the overshoot the coarse map
// creates — which contracts through the noise instead of amplifying it.
local relax is 0.5.
local search_budget is 12.

// How close in down-range reach the placement pass must settle before it
// stops. Not a precision of the plan but the slack in braking's reachable
// corridor: braking re-solves its throttle from live state and discards
// this plan's seed on its first pass, so a plan that lands PDI within
// x_tol of its own solve lands it well inside what the flight absorbs. Set
// above the Euler march's reach resolution (its last-step ground advance)
// so the pass stops at the descent's scale rather than banging against the
// integrator's floor. Provisional: a flight refines it.
local x_tol is 100.
// The coarse solve's endpoint tolerance, ground metres — looser than
// x_tol because the solve only has to seed the flight-fidelity placement
// pass, which lands the endpoint at scale 1, and the live braking
// re-solve absorbs the down-range residual on top of that. It sits above
// the endpoint scatter the coarse march's throttle solve admits when the
// chord and capability demands fall within a few metres and trade the
// binding pass to pass, so the loop settles here rather than chasing the
// placement pass's tighter x_tol through that noise.
local x_settle is 1000.
// The solve's altitude tolerance, metres: an accuracy bound in the family
// of coast_dx and pitch_tol, not a craft or body number. It sits above
// the metre-scale slop the node's delivered periapsis carries and the
// reach noise the throttle solve's eps admits, and below the scale at
// which a clearance judgment changes meaning. It does not track
// coast_clearance: the tolerance answers to the solve's own noise floor,
// which does not move when the pilot's caution does.
local h_tol is 5.
// The coast walk's sample step, ground metres at periapsis speed — an
// accuracy bound like pitch_tol, argued where the walk is defined below.
local coast_dx is 200.
// The chord walk's sample step, ground metres — the same kind of
// accuracy bound, argued where the walk is defined below.
local chord_dx is 100.

local tgt is body:geopositionlatlng(target_lat, target_lng).
// Altitude, above the datum, where the arc ends: landing_height above the
// site's terrain. The solve builds h_pdi on top of it. The flight
// controller never sees landing_height — it is spent into the ellipse
// here, which is the point.
local h_handoff is tgt:terrainheight + landing_height.
local a_max is ship:availablethrust / ship:mass.
// Metres of ground per degree of longitude along the site's parallel: the
// chord walk's ground scale, under the same equatorial-track assumption
// place_node and the coast walk share.
local m_per_deg is body:radius * cos(target_lat) * constant:degtorad.

// Planning is a few dozen marches of the arc (more with the sweep); run
// them at the processor's ceiling and put the setting back on the way
// out.
local ipu_prior is config:ipu.
set config:ipu to 2000.

// Every abort path: drop whatever node this script added, restore the
// processor setting, stop. The entry guard below ensures any standing node
// is ours to remove, so the ship is left exactly as found.
function plan_abort {
  parameter why.
  // wait 0 lets hasnode see the removal: it is stale within the physics
  // tick, and a second remove on an empty list throws, crashing the abort.
  until not hasnode { remove nextnode. wait 0. }
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

print "target " + round(target_lat, 4) + " " + round(target_lng, 4)
    + ", terrain " + round(tgt:terrainheight) + " m; handoff "
    + round(h_handoff) + " m.".

// Mass leaves through the engine at thrust / (Isp * g0) at full throttle;
// the stepper scales it by the throttle.
local mdot_full is ship:availablethrust / (engine_isp() * constant:g0).

// === THE ARC, DUPLICATED ===
// integrate_arc is the same Euler march as powered_descent_min.ks's
// endpoint — the same rates, the same step rule — because the plan is
// only as good as its price, and the price is only right if the planner
// marches the arc the flight controller flies. The departures: the seed
// is the candidate ellipse's periapsis instead of the live ship; the
// march ends at speed_handoff instead of the attitude seam, because the
// planner prices the whole braking descent where the flight controller
// hands the seam's residual to terminal; and the step tolerances take a
// scale so the search loops can march coarse. Until the two share a
// library, a change to the step physics must be made in both.

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

// The one throttle whose arc bottoms out at the handoff altitude: the
// candidate ellipse fixes PDI, so how hard the engine pushes is the only
// free variable, and the ending height rises monotonically with it. The
// same march the flight controller solves on down-range, solved here on
// altitude, because the planner's question is where the arc bottoms, not
// where it lands.
function solve_throttle {
  parameter orbit_ is ship:orbit.
  parameter f_seed is -1.   // the previous pass's answer, if there was one:
                            // the solve barely moves between passes, so a
                            // bracket beside it saves most marches
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
  // The search, priced in marches — every probe costs a full arc march.
  // The returned throttle is always an end of a sign-checked bracket
  // whose miss is positive: its arc bottoms at or above the handoff, so
  // the closure check downstream holds by construction. The miss is too
  // lopsided for an unbracketed root-follower — below the root it
  // saturates near -landing_height while above it climbs steeply, so a
  // secant settles on the low side and hands back an arc that grounds
  // out.
  //
  // Seeded — every pass after the first — the root barely moves, so a
  // 10% window beside the old answer brackets it in two marches, and
  // false position closes the bracket in two or three more, the
  // interpolant cut in half each time the saturated low side repeats so
  // it cannot stall there. A window that misses the root still tightens
  // one end of the bracket below. Unseeded, bisection from the full
  // bracket: miss(0) is negative by construction — no thrust runs the
  // march into the terrain or its step cap, both reported short — so
  // only the ceiling is probed, and miss(f_max) <= 0 is the real
  // failure, no throttle up to f_max flying the ellipse down, returned
  // as -1 for the caller. The
  // tolerance is f_max/256, ~4e-3 of full scale: the flight's own
  // re-solve bisects to 0.001 from true state, so the planner has no
  // reason to out-resolve it.
  local eps is f_max / 256.
  local lo is 0.
  local hi is -1.
  if f_seed > 0 {
    local lo_s is 0.95 * f_seed.
    local hi_s is min(f_max, 1.05 * f_seed).
    local m_lo is miss(lo_s).
    if m_lo >= 0 {
      set hi to lo_s.        // the root sits below the window
    } else {
      local m_hi is miss(hi_s).
      if m_hi > 0 {
        local iters is 0.
        until hi_s - lo_s < eps or iters >= 8 {
          // The interpolant is trusted only between two real misses: a
          // sentinel at the low end, or a point pinned to an end of the
          // bracket, halves the bracket instead.
          local c is hi_s - m_hi * (hi_s - lo_s) / (m_hi - m_lo).
          if c <= lo_s or c >= hi_s or m_lo < -1e8 {
            set c to (lo_s + hi_s) / 2.
          }
          local m_c is miss(c).
          if m_c > 0 {
            set hi_s to c.
            set m_hi to m_c.
          } else {
            set lo_s to c.
            set m_lo to m_c.
            set m_hi to m_hi / 2.
          }
          set iters to iters + 1.
        }
        return hi_s.
      }
      if hi_s >= f_max { return -1. }   // the window's top was the ceiling
      set lo to hi_s.      // the root sits above the window
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
  return hi.
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

// The lead angle a down-range reach implies: X as an angle at the body's
// centre, the placement that puts the arc's endpoint at the site.
function lead_from_reach {
  parameter x_.
  return x_ / body:radius * constant:radtodeg.
}

// === THE COAST ===
// The one stretch the chord never certifies: between the DOI burn and PDI
// the ship rides the ellipse over terrain the chord has no opinion about
// — and it binds for real: over the flattest ground on the body, the
// coast's clearance has measured within metres of PDI's. The walk is
// nearly exact, because the coast is on rails and kOS terrain is the
// game's own ground: step the placed ellipse from the burn to PDI in true
// anomaly, where every quantity is closed-form — radius from the conic
// equation, time from Kepler's equation run forward (anomaly to time
// needs no iteration; only the reverse does), footprint from the orbit's
// elements and the body's rotation — and keep the minimum of altitude
// over terrain. coast_dx, scaled by step_scale, is anchored to ground
// metres, so it is an accuracy bound like pitch_tol, not a craft or body
// number: in-solve callers march it search_scale coarser, the verdict
// walks it at scale 1.
//
// Alongside the minimum, the walk banks the terrain readings immediately
// before and after it: their difference over the ground they bracket is
// the pinch's local slope, which the verdict turns into a placement-error
// sensitivity — how many metres of clearance a few tenths of a degree of
// burn error would cost.
function coast_walk {
  parameter nd_orbit.
  parameter t_node.
  parameter t_pdi.
  parameter step_scale.

  local ecc_c is nd_orbit:eccentricity.
  local p_c is nd_orbit:semimajoraxis * (1 - ecc_c ^ 2).
  local lan_c is nd_orbit:longitudeofascendingnode.
  local aop_c is nd_orbit:argumentofperiapsis.
  local inc_c is nd_orbit:inclination.
  local period_c is nd_orbit:period.
  local rot_pdi is body_rotation(t_pdi, nd_orbit).
  local rot_rate is 360 / body:rotationperiod.
  local sq_lo is sqrt(1 - ecc_c).
  local sq_hi is sqrt(1 + ecc_c).
  local r_body_c is body:radius.
  local r2d_c is constant:radtodeg.
  local step_ is coast_dx * step_scale.

  local cc_min is 1e12.
  local cc_dt is 0.      // seconds before PDI, where the minimum sits
  local cc_alt is 0.
  local cc_terr is 0.
  local cc_lng is 0.
  local cc_lat is 0.
  local cc_samples is 0.
  local terr_prev is 0.
  local terr_before is 0.  // terrain one sample up-range of the minimum
  local terr_after is 0.   // terrain one sample down-range of the minimum
  local just_updated is false.
  local nu is true_anomaly(t_node, nd_orbit).
  until nu >= 360 {
    local r_ is p_c / (1 + ecc_c * cos(nu)).
    // True anomaly to eccentric to mean; the mean anomaly left to 360 is
    // the fraction of the period left to coast before PDI.
    local ea is 2 * arctan2(sq_lo * sin(nu / 2), sq_hi * cos(nu / 2)).
    if ea < 0 { set ea to ea + 360. }
    local ma is ea - ecc_c * sin(ea) * r2d_c.
    local dt_ is (360 - ma) / 360 * period_c.
    // The footprint: latitude is the inclination's projection at this
    // argument of latitude; longitude follows kepler's body_longitude
    // convention, the body having rotated dt_ less than it will have at
    // PDI.
    local lat_i is arcsin(sin(inc_c) * sin(aop_c + nu)).
    local lng_i is wrap_longitude(lan_c + aop_c + nu
                                  - (rot_pdi - rot_rate * dt_)).
    local terr is body:geopositionlatlng(lat_i, lng_i):terrainheight.
    local alt_i is r_ - r_body_c.
    if just_updated {
      set terr_after to terr.
      set just_updated to false.
    }
    if alt_i - terr < cc_min {
      set cc_min to alt_i - terr.
      set cc_dt to dt_.
      set cc_alt to alt_i.
      set cc_terr to terr.
      set cc_lng to lng_i.
      set cc_lat to lat_i.
      set terr_before to terr_prev.
      set just_updated to true.
    }
    set terr_prev to terr.
    set cc_samples to cc_samples + 1.
    set nu to nu + step_ / r_ * r2d_c.
  }
  // The minimum found on the walk's last sample has no down-range
  // neighbour; fall back to the up-range difference alone rather than
  // reading an uninitialized reading as flat ground.
  if just_updated { set terr_after to cc_terr. }
  local slope is (terr_after - terr_before) / (2 * step_).
  return lexicon("min", cc_min, "dt", cc_dt, "alt", cc_alt, "terr", cc_terr,
                 "lng", cc_lng, "lat", cc_lat, "samples", cc_samples,
                 "slope", slope).
}

// === THE CHORD ===
// The certificate's ground truth: the straight line from the handoff up
// to h_pdi at X, the arc's own down-range reach, must clear the terrain
// under it by terrain_margin. Requiring
// h_pdi >= h_handoff + X * max over x of ((terrain(x) + terrain_margin -
// h_handoff) / x) for every x in (0, X] gives the altitude any h_pdi must
// reach — independent of h_pdi itself, since the chord's own height only
// enters the profile log kept for the verdict. Walked along the site's
// parallel like the retired survey, under the same equatorial-track
// assumption place_node and the coast walk share; chord_dx is an accuracy
// bound, not a craft or body number. The walk starts at x = chord_dx, not
// 0, because the ratio blows up at the site if terrain there sits above
// h_handoff.
function chord_walk {
  parameter x_reach.
  parameter h_pdi_.

  local demand is 0.        // the steepest per-metre ratio seen, scaled by X
  local force_x is 0.       // where it was seen; 0 means nothing demanded
  local force_h is 0.
  local force_lng is 0.
  local profile is list().  // decimated (x, terrain, chord) triples
  local prof_step is max(chord_dx, x_reach / 50).
  local next_prof is 0.
  local x is 0.
  until x + chord_dx > x_reach {
    set x to x + chord_dx.
    local lng_i is wrap_longitude(target_lng - x / m_per_deg).
    local terr is body:geopositionlatlng(target_lat, lng_i):terrainheight.
    local ratio is (terr + terrain_margin - h_handoff) / x.
    if ratio * x_reach > demand {
      set demand to ratio * x_reach.
      set force_x to x.
      set force_h to terr.
      set force_lng to lng_i.
    }
    if x >= next_prof {
      local chord is h_handoff + x / x_reach * (h_pdi_ - h_handoff).
      profile:add(list(x, terr, chord)).
      set next_prof to next_prof + prof_step.
    }
  }
  return lexicon("h_demand", h_handoff + demand, "force_x", force_x,
                 "force_h", force_h, "force_lng", force_lng,
                 "profile", profile).
}

// === THE SOLVE ===
// h_pdi is the largest of three demands, each an altitude, evaluated at a
// candidate placement: the coast demand (coast_walk's minimum clearance
// at or above coast_clearance), the chord demand (chord_walk's h_demand),
// and the capability demand (solve_throttle's f at or below
// (1 - f_headroom) * f_max). None of the three has a closed form in
// h_pdi, so each pass corrects toward its own demand by one step and the
// max of the three becomes the next h_pdi:
//
// - The coast demand is a root — raising h_pdi with the burn radius fixed
//   raises the ellipse everywhere between, so the minimum clearance rises
//   monotonically with h_pdi — approached by adding the clearance deficit
//   straight to h_pdi. That is a one-sided, under-correcting step (the
//   clearance gain per metre of h_pdi is generally under 1 away from
//   periapsis), left for the outer iteration to finish rather than solved
//   to convergence on every pass.
// - The chord demand needs no correction: chord_walk hands back the
//   altitude directly, from terrain and X alone.
// - The capability demand has no formula at all — no march here says how
//   many metres of h_pdi buy back a given slice of throttle — so it is a
//   plain proportional step, scaled by both the fractional overrun and
//   how much altitude the descent already has above the handoff, moving
//   toward relief and never away from it.
//
// The iteration is the map's own step on h_pdi, damped only when the move
// reverses sign, the same discipline the old fixed point used on X: two
// noisy passes cannot be trusted to extrapolate, but a full step that
// overshot and comes back is trustworthy evidence of where the answer
// sits. Converged when both the h_pdi move and the down-range move settle
// — h_tol and x_settle are different quantities and neither implies the
// other now that h_pdi no longer tracks X through a fixed slope. A
// solve that fails to settle in search_budget passes is reported, not
// hidden: the map's contraction is observed on this body's telemetry, not
// proved.
//
// Candidate nodes are placed with plan_node's equatorial shortcut, coarse
// and cheap; the placement pass that follows corrects what the shortcut
// books, at flight fidelity.
function solve_hpdi {
  parameter verbose_ is false.
  parameter coast_floor_ is coast_clearance.

  // The seed. h_handoff + coast_floor_ is the lowest h_pdi any demand
  // could return. The lead comes from the stop-distance reach at the
  // throttle ceiling — the shortest ground any braking arc can cover —
  // so the first candidate prices a real descent rather than a
  // degenerate one.
  local r_seed is body:radius + h_handoff.
  local sma_seed is (ship:orbit:semimajoraxis + r_seed) / 2.
  local v_seed is sqrt(body:mu * (2 / r_seed - 1 / sma_seed))
                - 2 * constant:pi * r_seed / body:rotationperiod.
  local x_seed is v_seed ^ 2 / (2 * f_max * a_max).
  local h_pdi_ is h_handoff + coast_floor_.
  local lead_ is lead_from_reach(x_seed).
  local f_cap is (1 - f_headroom) * f_max.

  local iters is 0.
  local d_h is 1e9.
  local x_ is x_seed.
  local d_x is 1e9.
  local f_c is 0.
  local t_ is 0.
  local dv_doi_ is 0.
  local binding_ is "".

  until abs(d_h) < h_tol and abs(d_x) < x_settle {
    if iters >= search_budget {
      return lexicon("ok", false, "why", "h_pdi moved " + round(abs(d_h))
          + " m and the endpoint " + round(abs(d_x)) + " m on pass "
          + search_budget + "; the solve is not settling").
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
          + " flies the candidate ellipse (PDI " + round(h_pdi_)
          + " m) down to the handoff").
    }
    local arc is integrate_arc(f_c, nd:orbit, search_scale).
    set dv_doi_ to nd:deltav:mag.
    local t_node_ is timestamp(nd:time).
    local t_pdi_ is time_of_periapsis(t_node_, nd:orbit).
    local cc is coast_walk(nd:orbit, t_node_, t_pdi_, search_scale).
    remove nd.

    local x_new is arc["x"].
    set t_ to arc["t"].
    local chd is chord_walk(x_new, h_pdi_).

    // The three demands, each an altitude, at this placement.
    local demand_coast is h_pdi_ + (coast_floor_ - cc["min"]).
    local demand_chord is chd["h_demand"].
    local demand_cap is h_pdi_.
    if f_c > f_cap {
      set demand_cap to h_pdi_
          + (f_c - f_cap) / f_max * (h_pdi_ - h_handoff).
    }
    local h_new is max(demand_coast, max(demand_chord, demand_cap)).
    if h_new = demand_coast {
      set binding_ to "coast".
    } else if h_new = demand_chord {
      set binding_ to "chord".
    } else {
      set binding_ to "capability".
    }

    // Full step while the move keeps its sign; damp when it reverses,
    // which is the coarse map overshooting its fixed point.
    local d_h_new is h_new - h_pdi_.
    local step is choose relax if d_h_new * d_h < 0 else 1.
    set d_h to d_h_new.
    set h_pdi_ to h_pdi_ + step * d_h_new.
    set d_x to x_new - x_.
    set x_ to x_new.
    set lead_ to lead_from_reach(x_).
    set iters to iters + 1.
    if verbose_ {
      print "pass " + iters + ": h_pdi " + round(h_pdi_) + " m  X "
          + round(x_ / 1000, 1) + " km  f " + round(f_c, 3) + "  ("
          + binding_ + ").".
    }
  }
  return lexicon("ok", true, "h_pdi", h_pdi_, "lead", lead_, "f", f_c,
                 "x", x_, "t", t_, "dv_doi", dv_doi_, "iters", iters,
                 "binding", binding_).
}

local hp is solve_hpdi(true).
if not hp["ok"] {
  plan_abort(hp["why"] + ". Re-think coast_clearance, terrain_margin, or"
      + " the parking orbit.").
}
local h_pdi is hp["h_pdi"].
local lead_deg is hp["lead"].
local binding is hp["binding"].
local hp_iters is hp["iters"].

// === THE PRICE OF COAST_CLEARANCE ===
// coast_clearance is the plan's one fuel lever, so price the judgment:
// the same solve at a floor half as generous, twice, and four times, each
// with its DOI burn and its braking arc through the rocket equation.
// Advisory only — nothing below reads these numbers — and three more
// solves is several times the planning run, so it waits for do_sweep.
local sweep_lines is list().
local sweep_mults is choose list(0.5, 1, 2, 4) if do_sweep else list().
for mult in sweep_mults {
  local cc_try is coast_clearance * mult.
  local r_ is hp.
  if mult <> 1 { set r_ to solve_hpdi(false, cc_try). }
  local line is "".
  if r_["ok"] {
    local dv_a is arc_dv(r_["f"], r_["t"]).
    set line to "# coast_clearance " + round(cc_try) + " m: dv "
        + round(r_["dv_doi"] + dv_a, 1) + " m/s (doi "
        + round(r_["dv_doi"], 1) + " + arc " + round(dv_a, 1) + ")  f "
        + round(r_["f"], 3) + "  X " + round(r_["x"] / 1000, 1) + " km  "
        + r_["binding"]
        + (choose "  <- planned" if mult = 1 else "")
        + (choose "  (advisory: a floor the pilot declined)"
           if mult = 0.5 else "").
  } else {
    set line to "# coast_clearance " + round(cc_try) + " m: no plan ("
        + r_["why"] + ")".
  }
  print line.
  sweep_lines:add(line).
}

// === THE PLACEMENT ===
// The solve above placed its candidate nodes with plan_node's equatorial
// shortcut; place_node now measures where periapsis really falls and
// feeds the miss back. h_pdi is fixed on entry — the solve already pinned
// it — so only the lead moves, converging on the down-range reach the
// placed ellipse actually flies rather than the shortcut's estimate.
// Three passes without settling means something is inconsistent, not
// merely unconverged. The throttle solve is seeded from the caller's
// last answer, and it runs at flight fidelity: a coarse-solved f is
// biased low by more than landing_height's altitude slop, and its
// flight-fidelity arc flies into the ground with speed remaining.
function place_at_fidelity {
  parameter h_pdi_.
  parameter lead_.
  parameter f_seed.

  local f_ is f_seed.
  local x_arc_ is 0.
  local t_arc_ is 0.
  local v_pdi_ is 0.
  local placed_ is 0.
  local place_passes_ is 0.
  local place_dx is 1e9.
  local converged is false.

  until converged {
    set placed_ to place_node(h_pdi_, lead_).
    local nd is placed_["node"].

    set f_ to solve_throttle(nd:orbit, f_).
    if f_ < 0 {
      plan_abort("on the placed ellipse, no throttle up to " + f_max
          + " flies the arc down to the handoff.").
    }
    local arc is integrate_arc(f_, nd:orbit).
    if arc["speed"] > speed_handoff {
      plan_abort("the arc hit its step cap or the ground with speed still"
          + " to burn; this craft cannot fly this ellipse down.").
    }

    set place_dx to arc["x"] - x_arc_.
    set x_arc_ to arc["x"].
    set t_arc_ to arc["t"].
    set v_pdi_ to arc["v0"].
    local lead_new is lead_from_reach(x_arc_).
    set place_passes_ to place_passes_ + 1.
    print "placement " + place_passes_ + ": h_pdi " + round(h_pdi_) + " m  X "
        + round(x_arc_ / 1000, 1) + " km  f " + round(f_, 4) + ".".

    // Settled when the placed endpoint stops moving by more than the
    // corridor slack (x_tol). The standing node was placed from the
    // pre-update lead, which the criterion just certified as
    // interchangeable with the measured one.
    if abs(place_dx) < x_tol {
      set converged to true.
    } else if place_passes_ >= 3 {
      if abs(place_dx) >= 4 * x_tol {
        remove nd.
        plan_abort("the placement endpoint still moved "
            + round(abs(place_dx)) + " m on pass 3; the placement is not"
            + " settling.").
      }
      set converged to true.
    } else {
      set lead_ to lead_new.
      remove nd.
    }
  }
  return lexicon("placed", placed_, "f", f_, "x_arc", x_arc_,
                 "t_arc", t_arc_, "v_pdi", v_pdi_,
                 "place_passes", place_passes_).
}

local pr is place_at_fidelity(h_pdi, lead_deg, hp["f"]).
local placed is pr["placed"].
local f is pr["f"].
local x_arc is pr["x_arc"].
local t_arc is pr["t_arc"].
local v_pdi is pr["v_pdi"].
local place_passes is pr["place_passes"].
local nd is placed["node"].

// Ignition leads the node by half the burn, and the ship needs time to
// swing onto the burn vector; a node closer than that will be burned late,
// which silently moves periapsis east. Failing is self-correcting: by the
// re-run this crossing has passed, and the next is most of an orbit out.
if nd:eta < burn_duration(nd:deltav:mag) / 2 + 60 {
  plan_abort("the burn is only " + round(nd:eta) + " s away — too close to"
      + " orient and ignite on time. Re-run for the next crossing.").
}

// === THE CERTIFICATE ===
// The solve's coast and chord walks ran coarse, to keep the search cheap;
// certify the settled geometry against both at full fidelity before
// trusting it. A deficit here is what the coarse walks were built to
// approximate, not to guarantee, so it gets one recovery pass — raise
// h_pdi to what the full-fidelity walks now demand, re-place, and check
// again. A deficit that survives that pass means the coarse solve's
// approximation, not just its precision, missed something, and the plan
// is refused rather than patched further.
local t_node is timestamp(nd:time).
local t_pdi is placed["t_pdi"].
local cc is coast_walk(nd:orbit, t_node, t_pdi, 1).
local chd is chord_walk(x_arc, h_pdi).

if cc["min"] < coast_clearance or chd["h_demand"] > h_pdi {
  local demand_coast is h_pdi + (coast_clearance - cc["min"]).
  local demand_chord is chd["h_demand"].
  set h_pdi to max(demand_coast, demand_chord).
  set binding to choose "coast" if h_pdi = demand_coast else "chord".
  remove nd.
  set pr to place_at_fidelity(h_pdi, lead_from_reach(x_arc), f).
  set placed to pr["placed"].
  set f to pr["f"].
  set x_arc to pr["x_arc"].
  set t_arc to pr["t_arc"].
  set v_pdi to pr["v_pdi"].
  set place_passes to place_passes + pr["place_passes"].
  set nd to placed["node"].
  if nd:eta < burn_duration(nd:deltav:mag) / 2 + 60 {
    plan_abort("the burn is only " + round(nd:eta) + " s away — too close"
        + " to orient and ignite on time. Re-run for the next crossing.").
  }
  set t_node to timestamp(nd:time).
  set t_pdi to placed["t_pdi"].
  set cc to coast_walk(nd:orbit, t_node, t_pdi, 1).
  set chd to chord_walk(x_arc, h_pdi).
  if cc["min"] < coast_clearance {
    plan_abort("the coast still dips to " + round(cc["min"]) + " m over the"
        + " terrain " + round(cc["dt"]) + " s before PDI (ellipse "
        + round(cc["alt"]) + " m, terrain " + round(cc["terr"]) + " m, lng "
        + round(cc["lng"], 2) + ") after one recovery pass; the floor is "
        + coast_clearance + " m. If the dip is near PDI, raise"
        + " coast_clearance or move the site; if it sits well up-range"
        + " toward the burn, the parking orbit itself is too low for this"
        + " terrain.").
  }
  if chd["h_demand"] > h_pdi {
    plan_abort("the chord still demands " + round(chd["h_demand"])
        + " m of PDI altitude after one recovery pass; h_pdi settled at "
        + round(h_pdi) + " m over terrain " + round(chd["force_h"])
        + " m at " + round(chd["force_x"] / 1000, 1) + " km up-range."
        + " Raise terrain_margin's tolerance for that ground, or move the"
        + " site.").
  }
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
// authority to shorten the arc — the side that absorbs an overshoot. The
// solve above already refuses a plan that spends past this reserve on
// this ellipse; this warning is what catches the flight-fidelity
// placement drifting back into it. Room below needs no twin warning: it
// is f itself, and a plan that solved absurdly low announces itself in
// the delta-v report.
if f_max - f < f_headroom * f_max {
  print "WARNING: f_solved " + round(f, 3) + " sits within "
      + round(f_headroom * 100) + "% of f_max; little authority remains"
      + " to shorten the arc.".
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

// The price of the plan: the node's burn plus the braking arc by the
// rocket equation, at today's mass. Terminal descent is extra and roughly
// constant.
local dv_doi is nd:deltav:mag.
local dv_arc is arc_dv(f, t_arc).

// The chord's slope, degrees above horizontal: the honest one-line
// summary of how steep the approach is, derived rather than asked for.
local gamma_derived is arctan((h_pdi - h_handoff) / x_arc).

// The pinch's sensitivity: metres of coast clearance a placement error of
// a tenth of a degree would cost, the pinch's own terrain slope times the
// ground a tenth of a degree of longitude covers at the pinch's latitude
// — the walk is equatorial in practice, so this stays close to the
// site's own m_per_deg.
local m_per_deg_pinch is body:radius * cos(cc["lat"]) * constant:degtorad.
local sensitivity is abs(cc["slope"]) * m_per_deg_pinch * 0.1.

// The plan, printed and kept: doi_plan.log is the witness the flight is
// judged against.
local planlog is "doi_plan.log".
if exists(planlog) { deletepath(planlog). }
function report {
  parameter line.
  print line.
  log line to planlog.
}

report("# gamma " + round(gamma_derived, 2) + " deg (chord slope)  target "
    + round(target_lat, 4) + " " + round(target_lng, 4) + "  terrain "
    + round(tgt:terrainheight) + " m").
if binding = "coast" {
  report("# bound: coast — pinch terrain " + round(cc["terr"]) + " m at lng "
      + round(cc["lng"], 2) + ", " + round(cc["dt"]) + " s up-range of PDI").
} else if binding = "chord" {
  report("# bound: chord — " + round(chd["force_h"]) + " m terrain at lng "
      + round(chd["force_lng"], 4) + ", " + round(chd["force_x"] / 1000, 1)
      + " km up-range").
} else {
  local f_cap_ is (1 - f_headroom) * f_max.
  report("# bound: capability — f_solved " + round(f, 3) + " against the "
      + round(f_cap_, 3) + " cap (f_headroom " + f_headroom + ")").
}
report("# parking " + round(ship:orbit:periapsis) + " x "
    + round(ship:orbit:apoapsis) + " m  ecc "
    + round(ship:orbit:eccentricity, 4)).
report("# h_pdi " + round(h_pdi) + " m (node delivers "
    + round(nd:orbit:periapsis) + ")  X " + round(x_arc) + " m  lead "
    + round(lead_deg, 2) + " deg  passes " + hp_iters + " solve / "
    + place_passes + " placement").
report("# f_solved " + round(f, 4) + "  margin " + round(f_max - f, 3)
    + " below f_max  arc " + round(t_arc, 1) + " s").
report("# coast  min clearance " + round(cc["min"]) + " m at "
    + round(cc["dt"]) + " s before PDI (ellipse " + round(cc["alt"])
    + " m, terrain " + round(cc["terr"]) + " m, lng " + round(cc["lng"], 2)
    + ")  floor " + coast_clearance + " m  sensitivity "
    + round(sensitivity, 1) + " m per 0.1 deg placement error  "
    + cc["samples"] + " samples").
report("# dv  doi " + round(dv_doi, 1) + "  arc " + round(dv_arc, 1)
    + "  total " + round(dv_doi + dv_arc, 1) + " m/s (terminal excluded)").
report("# cross_pdi " + round(cross_pdi) + " m  bias_pdi "
    + round(bias_deg, 2) + " deg  residual " + round(cross_res, 1) + " m").
report("# node  dv " + round(nd:deltav:mag, 1) + " m/s  eta "
    + round(nd:eta) + " s  pe_lng_err " + round(placed["err"], 2)
    + " deg in " + placed["attempts"] + " attempts").
// The corridor itself, log only: enough of the profile to plot the chord
// against the ground it clears.
log "# profile: x_m,terrain_m,chord_m" to planlog.
for p in chd["profile"] {
  log round(p[0]) + "," + round(p[1]) + "," + round(p[2]) to planlog.
}
// The sweep, into the log as well: the prices the judgment was made
// against belong with the plan they produced.
for l in sweep_lines { log l to planlog. }

set config:ipu to ipu_prior.
print "Node placed. Burn it, then run powered_descent_min.".
