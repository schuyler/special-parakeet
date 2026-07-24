// plan_entry.ks — the entry planner for a lifting craft: one retrograde node,
// its Δv solved so the impact point Trajectories predicts sits at a chosen
// longitude.
//
// A capsule's aim point is the landing site; a spaceplane's cannot be. Flown
// at a pitch where it keeps authority through the atmosphere, the plane makes
// lift, and it glides past wherever the drag-only prediction says it falls —
// by a distance that belongs to the craft, not to this program. So the aim is
// the parameter: put the predicted impact at target_lng, fly the entry, and
// see where the craft stops. The site itself is the zero-lift aim; each
// flight of a given plane walks its aim west by the overshoot that flight
// showed. That is a per-craft dial on purpose — the lift lives in the craft.
//
// The solve leans on Trajectories (addons:tr), which prices the atmosphere
// but not the lift this plane will fly — which is exactly why the aim is a
// dial. The prediction must track the pending node, as deorbit_node.ks
// already relies on; if the impact readout ignores nodes, fix that in the
// Trajectories settings before trusting this plan.
//
// Equatorial prograde orbit assumed, like the rest of this library. The
// output is the node: burn it, then fly reentry.ks.

@lazyglobal off.

clearscreen.
print "=== PLAN ENTRY ===".

// kepler for time_to_longitude and wrap_longitude; common for bisect,
// burn_duration, and engine_isp.
run "../core/kepler".
run "common".

// The longitude the drag-only impact point is aimed at, degrees east. The
// craft's dial: the default is KSC, the zero-lift aim, and flights move it
// west per craft by the glide they demonstrate.
parameter target_lng is -74.5.
// Degrees of longitude the burn point sits west of the target. Half a turn
// is the Δv optimum — tangential, lowering the far side of the orbit — but
// it also buys the shallowest entry that still comes down, and shallow is
// the expensive direction here: downrange sensitivity to burn error and to
// the unmodeled lift both peak there, so the craft's aim dial wanders
// flight to flight, and the plane rides thin air for minutes of physics
// warp. 135 trades a few tens of m/s for a steeper, more repeatable entry.
// The plan log prices the trade at any lead, so the choice stays
// falsifiable without a flight.
parameter burn_lead is 135.
// The shallow bound of the Δv search: the first trial periapsis, as a
// fraction of the atmosphere's height. A bracket bound, not an aim — it only
// needs to sit shallow enough that its impact, if Trajectories finds one at
// all, falls east of the target, and it scales with the body's atmosphere
// instead of naming a craft or a body.
parameter pe_frac is 0.75.

// A pending node is not ours to reason about — or to delete.
if hasnode {
  print "ABORT: a maneuver node is already pending. Burn or remove it first.".
  wait until false.
}
if not addons:tr:available {
  print "ABORT: the Trajectories addon is not answering (addons:tr).".
  wait until false.
}
if ship:availablethrust <= 0 {
  print "ABORT: no live engine. Activate the rocket engine first.".
  wait until false.
}
// The solve varies a periapsis between the surface and the shallow bound;
// a ship already dipping below the atmosphere's top has no orbit to plan
// from.
if ship:orbit:periapsis < body:atm:height {
  print "ABORT: periapsis is already inside the atmosphere. This plans an"
      + " entry from orbit; the entry has begun.".
  wait until false.
}

// The Δv search runs a bisection inside a bisection; run it at the
// processor's ceiling and put the setting back on the way out.
local ipu_prior is config:ipu.
set config:ipu to 2000.

local burn_lng is wrap_longitude(target_lng - burn_lead).
local t_burn is time_to_longitude(burn_lng).

local nd is node(t_burn:seconds, 0, 0, 0).
add nd.
// A non-positive ETA is time_to_longitude's past-time sentinel arriving as a
// node in the past.
if nd:eta <= 0 {
  remove nd.
  set config:ipu to ipu_prior.
  print "ABORT: the plan puts the burn in the past. Re-run for the next"
      + " crossing.".
  wait until false.
}

local speed_burn is velocityat(ship, t_burn):orbit:mag.

// The prograde Δv (negative, so retrograde) that puts the node's post-burn
// periapsis at pe_target. nd:orbit answers instantly, so this inner solve
// costs nothing next to the Trajectories waits outside it. The floor,
// -0.9 of the burn-point speed, keeps the trial orbit a real ellipse
// (killing all of the speed degenerates the conic) while burying periapsis
// far below any atmosphere, so the bracket always straddles.
function dv_for_periapsis {
  parameter pe_target.
  local pe_err is {
    parameter dv.
    set nd:prograde to dv.
    return nd:orbit:periapsis - pe_target.
  }.
  return bisect(pe_err, -0.9 * speed_burn, 0, 0.1).
}

// Give Trajectories time to redo its prediction after the node moves, then
// poll the impact flag briefly — it can lag the change by a frame or two.
// The settle time is a guess in the deorbit_node.ks tradition; the verdict
// below re-reads the final answer, so a stale read mid-search costs
// iterations, not correctness.
function settled_impact {
  local tries is 0.
  wait 0.1.
  until addons:tr:hasimpact or tries >= 10 {
    wait 0.05.
    set tries to tries + 1.
  }
  return addons:tr:hasimpact.
}

// Signed longitude error of the predicted impact for a trial Δv, degrees
// east of the target. A pass too shallow for any impact reads as far east —
// the craft carries on around the planet — which keeps the search pushing
// steeper.
function impact_error {
  parameter dv.
  set nd:prograde to dv.
  if not settled_impact() { return 179. }
  return wrap_longitude(addons:tr:impactpos:lng - target_lng).
}

// === THE BRACKET ===
// Steep end: periapsis at the datum, which always impacts. Shallow end: the
// pe_frac periapsis, walked deeper until Trajectories finds an impact — a
// skimming pass may not come down inside the prediction's patience. The
// target must fall between the two ends' impact points or there is no root
// to find, and the reachable window is reported instead of guessed past.
local dv_steep is dv_for_periapsis(0).
local err_steep is impact_error(dv_steep).

local frac is pe_frac.
local dv_shallow is dv_for_periapsis(frac * body:atm:height).
set nd:prograde to dv_shallow.
until settled_impact() or frac = 0 {
  set frac to max(0, frac - 0.15).
  set dv_shallow to dv_for_periapsis(frac * body:atm:height).
  set nd:prograde to dv_shallow.
}
// The walk bottoms out at the steep end, which impacts by construction, so
// arriving here without an impact means the prediction itself is absent.
if not addons:tr:hasimpact {
  remove nd.
  set config:ipu to ipu_prior.
  print "ABORT: Trajectories reports no impact even at zero periapsis."
      + " Check that its prediction follows maneuver nodes.".
  wait until false.
}
local err_shallow is wrap_longitude(addons:tr:impactpos:lng - target_lng).

if err_steep * err_shallow > 0 {
  local lng_steep is wrap_longitude(target_lng + err_steep).
  local lng_shallow is wrap_longitude(target_lng + err_shallow).
  remove nd.
  set config:ipu to ipu_prior.
  print "ABORT: from a burn at " + round(burn_lng, 1) + " deg the impact"
      + " only reaches [" + round(lng_steep, 1) + " .. "
      + round(lng_shallow, 1) + "] deg; the target " + round(target_lng, 1)
      + " deg is outside it. Move burn_lead or the aim.".
  wait until false.
}

// === THE SOLVE ===
// Impact longitude moves monotonically west as the burn deepens, so the
// error crosses zero once between the bracket's ends. A quarter m/s of Δv
// moves the impact by a few kilometres — inside the walk-there precision
// this library settles for.
local dv_entry is bisect(impact_error@, dv_steep, dv_shallow, 0.25).
// bisect's lost-bracket sentinel: the ends re-evaluated with the same sign,
// which a prediction flickering mid-solve can produce.
if dv_entry = -1 {
  remove nd.
  set config:ipu to ipu_prior.
  print "ABORT: the impact prediction flickered and the search lost its"
      + " bracket. Re-run.".
  wait until false.
}
set nd:prograde to dv_entry.

// Ignition leads the node by half the burn, and the ship needs time to swing
// onto the burn vector; a node closer than that burns late, which silently
// moves the impact east. Failing is self-correcting: by the re-run this
// crossing has passed and the next is most of an orbit out.
if nd:eta < burn_duration(nd:deltav:mag) / 2 + 60 {
  remove nd.
  set config:ipu to ipu_prior.
  print "ABORT: the burn is only " + round(nd:eta) + " s away — too close to"
      + " orient and ignite. Re-run for the next crossing.".
  wait until false.
}

// === THE VERDICT ===
// The achieved impact point against the aim. The flight's falsifier is
// different and better: where the plane actually stops, against this aim —
// that difference, in degrees, is the craft's glide, and it feeds the next
// run's target_lng.
local err_final is impact_error(dv_entry).
local impact_km is err_final * constant:degtorad * body:radius / 1000.

local planlog is "entry_plan.log".
if exists(planlog) { deletepath(planlog). }
function report {
  parameter line.
  print line.
  log line to planlog.
}
report("# PLAN ENTRY  target lng " + round(target_lng, 2)
    + "  burn lng " + round(burn_lng, 2) + " (lead " + round(burn_lead, 1)
    + " deg)").
report("# orbit  " + round(ship:orbit:periapsis) + " x "
    + round(ship:orbit:apoapsis) + " m  ecc "
    + round(ship:orbit:eccentricity, 4)).
report("# window  steep " + round(wrap_longitude(target_lng + err_steep), 2)
    + "  shallow " + round(wrap_longitude(target_lng + err_shallow), 2)
    + " deg  (shallow pe frac " + round(frac, 2) + ")").
report("# node  dv " + round(nd:prograde, 1) + " m/s  eta " + round(nd:eta)
    + " s  pe " + round(nd:orbit:periapsis) + " m").
if addons:tr:hasimpact {
  local site is addons:tr:impactpos.
  report("# impact  " + round(site:lat, 3) + " " + round(site:lng, 3)
      + "  err " + round(err_final, 2) + " deg (" + round(impact_km, 1)
      + " km)").
}

set config:ipu to ipu_prior.
print "Node placed. Burn it, then fly reentry. Log the touchdown longitude:"
    + " touchdown minus " + round(target_lng, 1) + " is this craft's glide.".
