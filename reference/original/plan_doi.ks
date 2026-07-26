// plan_doi.ks — the DOI planner: one node that drops a parking orbit onto a
// descent ellipse whose periapsis is PDI, over the landing site.
//
// Terrain clearance is the pilot's to certify, not this program's: check on
// the map that the descent ellipse down to PDI and the braking arc up from it
// both clear the ground. h_pdi is a dial rather than a search, so the braking
// arc is marched once.
//
// The DOI burn is tangential, so periapsis lands roughly half an orbit ahead
// of it. Roughly, because the burn point is an apsis only on a circular
// parking orbit; anywhere else the burn leaves radial speed in play, and
// periapsis comes down past the antipode and lower than vis-viva sizing off an
// apsis predicts. Neither offset is modelled here. The placement asks the game
// where the node it just made puts periapsis, and moves the burn by the miss.
//
// The output is the node. Burn it, then run powered_descent.ks, which reads
// the resulting ellipse and flies it down. f_max here must match the f_max
// that program is run with, or the reach priced here is not the reach flown.

@lazyglobal off.

clearscreen.
print "=== PLAN DOI ===".

// kepler for time_to_longitude, time_of_periapsis, geoposition_at,
// wrap_longitude; common for engine_isp and burn_duration.
run "../core/kepler".
run "common".

parameter target_lat is 0.
parameter target_lng is 0.
// The periapsis altitude, metres above the target's terrain. PDI is placed
// here; the pilot certifies by eye that the ellipse down to it and the arc up
// from it both clear the ground. plan_doi derives this from terrain demands;
// here it is a dial.
parameter pdi_height is 2000.
// The throttle ceiling the descent is flown at. Must equal
// powered_descent.ks's f_max, or the arc priced here is not the arc flown.
parameter f_max is 0.85.
// The share of the throttle ceiling left unspent, so the flight controller's
// live re-solve keeps authority to shorten the arc. PDI is placed at the reach
// of a brake at (1 - f_headroom) * f_max rather than at f_max: the extra ground
// that lower throttle covers is the reserve, self-scaled to the craft's thrust
// and the body's gravity instead of a fixed distance factor. Dimensionless,
// provisional until a flight falsifies it.
parameter f_headroom is 0.1.

// A pending node is not ours to reason about — or to delete.
if hasnode {
  print "ABORT: a maneuver node is already pending. Burn or remove it first.".
  wait until false.
}
if ship:availablethrust <= 0 {
  print "ABORT: no live engine. Stage or activate the descent engine.".
  wait until false.
}

local tgt is body:geopositionlatlng(target_lat, target_lng).
// PDI altitude above the datum: pdi_height above the site's terrain.
local h_pdi is tgt:terrainheight + pdi_height.
local f_cap is (1 - f_headroom) * f_max.

// Surface gravity and the descent geometry powered_descent.ks shares:
// tilt_max is the attitude margin the craft keeps so it can swing back to
// brake; a_lat_max the horizontal acceleration that margin buys at hover-scale
// thrust, g_surf * tan(tilt_max); a_eff the fraction of it the planning
// budgets, the rest being feedback reserve. These fix the terminal stopping
// distance the aim below adds to the arc's reach — the same numbers, so the
// same aim.
local g_surf is body:mu / body:radius ^ 2.
local tilt_max is 30.
local a_lat_max is g_surf * tan(tilt_max).
local a_eff is 0.8 * a_lat_max.

// The march's accuracy bounds — twins of powered_descent.ks's, and locals
// here for the same reason: accuracy bounds, not craft or body numbers.
// pitch_tol caps the flight-path rotation per Euler step (degrees); v_frac
// caps the fractional speed change per step.
local pitch_tol is 1.
local v_frac is 0.02.
// How many times the placement below puts the node down and reads it back.
// Every pass after the first flies the previous pass's miss into the
// placement. A pass removes about nine tenths of its miss rather than all of
// it, because sliding the burn also slides it to a different true anomaly on
// the parking ellipse, which changes how far past the antipode periapsis
// falls. So the miss decays by roughly a decimal digit a pass — measured over
// the Mun at 15.7 degrees, 1.9, 0.18, 0.015 — and levels off near a
// thousandth of a degree from the sixth. An accuracy bound like the two
// above, not a craft or body number.
local passes is 8.

// Mass leaves through the engine at thrust / (Isp * g0) at full throttle; the
// stepper scales it by the throttle.
local mdot_full is ship:availablethrust / (engine_isp() * constant:g0).

// One march is cheap, but run it at the processor's ceiling anyway and put the
// setting back on the way out.
local ipu_prior is config:ipu.
set config:ipu to 2000.

print "target " + round(target_lat, 4) + " " + round(target_lng, 4)
    + ", terrain " + round(tgt:terrainheight) + " m; PDI " + round(h_pdi)
    + " m (" + round(pdi_height) + " over terrain).".

// === THE ARC ===
// powered_descent.ks's endpoint march, reseeded from the descent ellipse's
// periapsis instead of the live ship. The same Euler step and the same seam
// exit (flight path steeper than 90 - tilt_max below horizontal, where braking
// hands to terminal), so the reach it returns is the reach the flight
// controller's solve_f will price. The seed: periapsis altitude is h_pdi, the
// speed there is vis-viva on the descent ellipse less the ground's motion,
// because the arc is flown against the ground; pitch is zero, periapsis being
// horizontal. Equatorial and prograde, per the parking-orbit assumption. The
// descent ellipse is the one plan_node places: apoapsis at the parking radius,
// periapsis at h_pdi.
function brake_reach {
  parameter f.

  local r_pe is body:radius + h_pdi.
  local sma_desc is (r_pe + ship:orbit:semimajoraxis) / 2.
  local speed is sqrt(body:mu * (2 / r_pe - 1 / sma_desc))
              - 2 * constant:pi * r_pe / body:rotationperiod.
  local h is h_pdi.
  local pitch is 0.        // degrees above the horizon; PDI is a periapsis
  local m is ship:mass.
  local theta is 0.        // ground angle swept, radians
  local steps is 0.
  local thrust is f * ship:availablethrust.
  // The floor is the site's terrain, matching the flight controller: a brake
  // too weak to make the seam runs into ground that exists, and reads as a
  // failed arc below.
  local h_floor is tgt:terrainheight.
  until pitch <= tilt_max - 90 or h <= h_floor or steps >= 4000 {
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
    set steps to steps + 1.
  }
  // pitch rides along so the caller can tell a closed arc (reached the seam)
  // from one that hit the floor or the step cap with the path still shallow.
  return lexicon("x", theta * body:radius, "vh", speed * cos(pitch),
                 "h", h, "pitch", pitch, "steps", steps).
}

// === THE NODE ===
// One retrograde node that drops periapsis to h_pdi over desired_lng. Each
// pass places the node, asks the game what that node actually does, and
// corrects the two numbers it was asked for: the burn slides west or east by
// the longitude miss, and the radius the sizing formula aims at absorbs the
// periapsis miss. Both corrections are applied at the top of a pass, so the
// last one computed is also flown into a placement.
function plan_node {
  parameter desired_lng.   // body-fixed longitude periapsis is wanted at
  parameter t_burn.        // seed burn time, roughly half an orbit before it

  // Degrees of longitude the ground track covers per second: 360 over the
  // synodic period, the body's own rotation already netted out. It turns a
  // longitude miss into the burn-time change that removes it.
  local lng_rate is 360 / synodic_period(ship:orbit).
  // The periapsis radius the vis-viva sizing aims at. It starts at the one
  // wanted and absorbs the difference between what a tangential burn off an
  // apsis delivers and what one here does.
  local r_aim is body:radius + h_pdi.
  // The previous pass's misses: metres of periapsis below h_pdi, and degrees
  // of longitude east of where periapsis was wanted. Zero on the first pass,
  // which places the node from the seed alone.
  local pe_miss is 0.
  local lng_miss is 0.

  local nd is node(t_burn:seconds, 0, 0, 0).
  add nd.
  from { local pass is 0. } until pass >= passes step { set pass to pass + 1. } do {
    set r_aim to r_aim + pe_miss.
    set t_burn to t_burn - lng_miss / lng_rate.

    // The ship's own radius and speed at the burn, read off the parking orbit.
    // positionat and velocityat follow the predicted trajectory through a
    // pending node, and at t_burn that is this node's own discontinuity.
    local st is orbit_at(t_burn, ship:orbit).
    local r_burn is st["position"]:mag.
    set nd:time to t_burn:seconds.
    // Vis-viva for the ellipse with apoapsis at the burn and periapsis at
    // r_aim, less the speed the ship already has there.
    set nd:prograde to sqrt(body:mu * (2 / r_burn - 2 / (r_burn + r_aim)))
                     - st["velocity"]:mag.

    local t_pe is time_of_periapsis(timestamp(nd:time), nd:orbit).
    set pe_miss to h_pdi - nd:orbit:periapsis.
    set lng_miss to wrap_longitude(
        geoposition_at(t_pe, nd:orbit):lng - desired_lng).
  }
  return nd.
}

// === THE PLAN ===

// The reach of a brake at the reserved throttle, plus where its residual
// horizontal speed would stop decelerated at a_eff — solve_f's aim, evaluated
// at f_cap so the flight controller solves near f_cap and holds f_headroom in
// hand.
local arc is brake_reach(f_cap).
if arc["pitch"] > tilt_max - 90 {
  set config:ipu to ipu_prior.
  print "ABORT: at " + round(f_cap, 3) + " throttle the arc "
      + (choose "ran into the terrain" if arc["h"] <= tgt:terrainheight
         else "hit its step cap")
      + " with the flight path still " + round(arc["pitch"], 1)
      + " deg above the seam. This craft cannot fly a " + round(h_pdi)
      + " m periapsis down here; raise pdi_height, add thrust, or lower"
      + " f_headroom.".
  wait until false.
}
local braking is arc["x"] + arc["vh"] ^ 2 / (2 * a_eff).
local lead_deg is braking / body:radius * constant:radtodeg.

// Where periapsis is wanted, and the burn that seeds the search for it: half
// an orbit before that longitude, and further west by the rotation the body
// turns through while the ship coasts down.
local desired_lng is wrap_longitude(tgt:lng - lead_deg).
local sma_seed is (ship:orbit:semimajoraxis + body:radius + h_pdi) / 2.
local t_coast is constant:pi * sqrt(sma_seed ^ 3 / body:mu).
local burn_lng is wrap_longitude(desired_lng
    + t_coast * 360 / body:rotationperiod - 180).
local t_seed is time_to_longitude(burn_lng).
// A time in the past is time_to_longitude's sentinel for a root it could not
// bracket.
if (t_seed - time):seconds <= 0 {
  set config:ipu to ipu_prior.
  print "ABORT: found no crossing of the burn longitude "
      + round(burn_lng, 2) + " deg ahead.".
  wait until false.
}

local nd is plan_node(desired_lng, t_seed).
// The corrections move the burn by as much as tens of seconds, which from a
// seed already close to now can walk it into the past.
if nd:eta <= 0 {
  remove nd.
  set config:ipu to ipu_prior.
  print "ABORT: the DOI plan puts the burn in the past.".
  wait until false.
}
// Ignition leads the node by half the burn, and the ship needs time to swing
// onto the burn vector; a node closer than that burns late, which silently
// moves periapsis east. Failing is self-correcting: by the re-run this crossing
// has passed and the next is most of an orbit out.
if nd:eta < burn_duration(nd:deltav:mag) / 2 + 60 {
  remove nd.
  set config:ipu to ipu_prior.
  print "ABORT: the burn is only " + round(nd:eta) + " s away — too close to"
      + " orient and ignite. Re-run for the next crossing.".
  wait until false.
}

// === THE VERDICT ===
// Where periapsis lands and how high, against what was asked of it. These are
// the placement's residuals, not a drift it declined to correct: the loop
// aimed at both, so a search that did not converge shows up here rather than
// going unsaid.
local t_pdi is time_of_periapsis(timestamp(nd:time), nd:orbit).
local pe_lng is geoposition_at(t_pdi, nd:orbit):lng.
local pe_err is wrap_longitude(pe_lng - desired_lng).
local pe_miss is nd:orbit:periapsis - h_pdi.

// The chord's slope, degrees below horizontal: how steep the approach is, from
// the PDI altitude down to where the arc reached the seam, over the ground it
// covered getting there.
local gamma is arctan((h_pdi - arc["h"]) / arc["x"]).
local dv_doi is nd:deltav:mag.

// The plan, printed and kept: doi_plan.log is the witness the flight is judged
// against.
local planlog is "doi_plan.log".
if exists(planlog) { deletepath(planlog). }
function report {
  parameter line.
  print line.
  log line to planlog.
}
report("# PLAN DOI (MIN)  target " + round(target_lat, 4) + " "
    + round(target_lng, 4) + "  terrain " + round(tgt:terrainheight) + " m").
report("# parking " + round(ship:orbit:periapsis) + " x "
    + round(ship:orbit:apoapsis) + " m  ecc "
    + round(ship:orbit:eccentricity, 4)).
report("# h_pdi " + round(h_pdi) + " m (node delivers "
    + round(nd:orbit:periapsis) + ", miss " + round(pe_miss, 1)
    + " m)  gamma " + round(gamma, 2) + " deg").
report("# arc  reach " + round(arc["x"]) + " m  vh " + round(arc["vh"], 1)
    + " m/s  stop " + round(arc["vh"] ^ 2 / (2 * a_eff)) + " m  lead "
    + round(lead_deg, 2) + " deg at f_cap " + round(f_cap, 3)).
report("# node  dv " + round(dv_doi, 1) + " m/s  eta " + round(nd:eta)
    + " s  pe_lng " + round(pe_lng, 2) + " want " + round(desired_lng, 2)
    + " (err " + round(pe_err, 3) + " deg)").

set config:ipu to ipu_prior.
print "Node placed. Eyeball the ellipse for terrain clearance, then burn and"
    + " run powered_descent.".
