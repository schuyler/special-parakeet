clearscreen.

// === TRANSFER PLANNER (step 3 of 4) ===
//
// The rendezvous procedure, one script per step, each allowed to collapse
// to "nothing to do":
//   1. detune.ks     change our period so the clock will line up
//   2. loiter.ks     coast, counting laps, until it does (no burn)
//   3. transfer.ks   this: the half-ellipse over to the target's radius
//   4. rendezvous.ks match velocities at the encounter
//
// The geometry burn, and only the geometry burn. From a (near-)circular
// orbit coplanar with the target (match_planes first), plan the tangent
// half-ellipse from our radius to one of the target's apsides, departing
// at our next crossing of the departure point (the apsis antipode). We aim
// at an apsis because there the target's velocity is purely horizontal, so
// the final match burn is a pure magnitude change.
//
// This script does not hunt for a well-timed window. Once the two radii
// are chosen, Kepler freezes the transfer's shape, departure point, and
// flight time; the only timing freedom in the whole procedure is when the
// departure crossing happens, and that is steps 1-2's business. Run this
// after loiter says the window is open (or detune has made one). The
// departure state is read from the actual predicted orbit — radius and
// speed at the crossing — so arriving on a detuned loiter orbit is fine.
//
// Collapse, two ways. If the flight plan already predicts a closest
// approach inside approach_tol, the transfer is flown or already planned
// and there is nothing to fix — this matters because the co-radial test
// below cannot see that state (mid-transfer, our semimajor axis matches
// neither radius) and a blind rerun would wipe good nodes to plan a burn
// from the middle of the ellipse. And if we already fly at the target's
// apsis radius there is no half-ellipse to build — the "transfer" is half
// a lap of our own orbit — so there is nothing to burn and no node to add.
//
// After adding the node we predict the closest approach it actually
// produces; a big miss here means the clock was never fixed, not that the
// geometry is wrong.
//
// Next: run refine (tune the node against the real predicted miss), run
// next (fly it), run rendezvous (plan the match burn at closest approach).

parameter aps_pick is "auto".   // "pe", "ap", or auto = cheaper of the two
parameter approach_tol is -1.   // skip if plan already meets this; warn if the new node misses it, m; -1 = policy
parameter tol_coradial is 0.01. // |r1-r2|/r1 at/below this: collapsed

run common.
run orbital.

if approach_tol < 0 {
  set approach_tol to plan_approach_tol.
}

local mu is body:mu.
local r1 is ship:orbit:semimajoraxis.
local v1 is sqrt(mu / r1).
local t_ship is ship:orbit:period.
local t_tgt is target:orbit:period.
local a_tgt is target:orbit:semimajoraxis.

// visviva, angle_ahead, time_to_apsis, closest_approach come from orbital.ks.

function main {
  print "=== TRANSFER PLAN (step 3: fix the geometry) ===".

  if ship:orbit:eccentricity > plan_e_circular {
    print "WARNING: orbit e=" + round(ship:orbit:eccentricity, 3)
      + "; departure timing assumes near-circular.".
  }
  local rel_inc is relative_inclination().
  if rel_inc > plan_inc_warn {
    print "WARNING: planes off by " + round(rel_inc, 2) + " deg; run match_planes first.".
  }

  // Encounter gate: closest_approach honors pending nodes, so this asks the
  // question directly — does the flight plan as it stands already meet the
  // target? Mid-transfer the encounter sits half our (ellipse) period out,
  // always inside this window. A pending departure node's encounter usually
  // is too; when an extreme geometry pushes it past the window the gate just
  // falls through to a re-plan — the pre-gate behavior, never a wrong burn.
  local ca0 is closest_approach(0, t_ship + t_tgt, 96).
  if ca0["dist"] <= approach_tol {
    print "Transfer collapses: the flight plan already meets the target.".
    print "  Predicted closest approach: " + round(ca0["dist"]) + " m at +"
      + round(ca0["t"] / 60, 1) + "m.".
    print "Nothing to plan (a rerun here would only wipe it).".
    print "Next: run refine to tighten it. run next. run rendezvous.".
    return.
  }

  // Size up both apsides; the transfer to each is frozen by geometry, so
  // "auto" just compares their total costs. An apsis below the body's
  // safe radius (core/safety.ks) is refused outright: the transfer
  // ellipse bottoms out at r2, and the alarm-clock workflow means a plan
  // must be safe to coast unflown.
  local chosen is 0.
  local found is false.
  local any_unsafe is false.
  for aps in list(list("pe", 0, body:radius + target:orbit:periapsis),
                  list("ap", 180, body:radius + target:orbit:apoapsis)) {
    local aps_name is aps[0].
    local r2 is aps[2].
    if r2 < safe_radius(body) {
      set any_unsafe to true.
      print aps_name + " (" + round((r2 - body:radius) / 1000, 1)
        + " km) is below " + body:name + "'s safe altitude ("
        + round(safe_alt(body) / 1000, 1) + " km); not aiming there.".
    } else if abs(r1 - r2) / r1 <= tol_coradial {
      print aps_name + " is within " + round(tol_coradial * 100, 1)
        + "% of our radius; no transfer there (collapsed).".
    } else if aps_pick = "auto" or aps_pick = aps_name {
      local t_aps0 is time:seconds + time_to_apsis(target:orbit, aps[1]).
      local a_t is (r1 + r2) / 2.
      local est is abs(visviva(r1, a_t) - v1)
        + abs(visviva(r2, a_tgt) - visviva(r2, a_t)).
      if not found or est < chosen["est"] {
        set found to true.
        set chosen to lexicon(
          "aps", aps_name, "r2", r2, "est", est,
          "dir", positionat(target, t_aps0) - body:position).
      }
    }
  }

  if not found {
    print "Transfer collapses: nothing to fly at the requested apsis.".
    if any_unsafe {
      print "An apsis was refused as unsafe (above) and the rest collapsed.".
      print "Meeting this target takes its other apsis (aps_pick) or a".
      print "hand-planned approach.".
    } else {
      print "We are already at the target's radius; if the encounter is still".
      print "missing, the clock is wrong, not the geometry: run loiter, and".
      print "detune if loiter says the window won't come.".
    }
    return.
  }

  // Depart at our next crossing of the departure point. The departure
  // state comes from the predicted orbit there, not a circular assumption,
  // so a slightly eccentric loiter orbit hands over cleanly.
  local ang is mod(angle_ahead(chosen["dir"]) - 180 + 360, 360).
  local t_dep is time:seconds + ang / 360 * t_ship.
  if t_dep < time:seconds + plan_min_lead {
    set t_dep to t_dep + t_ship.
  }

  local r2 is chosen["r2"].
  local r_dep is (positionat(ship, t_dep) - body:position):mag.
  local v_dep is velocityat(ship, t_dep):orbit:mag.
  local a_t is (r_dep + r2) / 2.
  local t_h is constant:pi * sqrt(a_t ^ 3 / mu).
  local dv_dep is visviva(r_dep, a_t) - v_dep.
  local dv_arr is abs(visviva(r2, a_tgt) - visviva(r2, a_t)).

  until not hasnode {
    remove nextnode.
  }
  add node(t_dep, 0, 0, dv_dep).
  print "Transfer to " + chosen["aps"] + ": " + round(dv_dep, 1)
    + " m/s prograde in " + round(t_dep - time:seconds) + "s"
    + "; arrival in " + round((t_dep + t_h - time:seconds) / 60, 1) + "m.".
  print "Estimated match burn at arrival: " + round(dv_arr, 1) + " m/s.".

  // Approach gate: with the node in the flight plan, ask what gap the
  // transfer actually leaves at the encounter. positionat honors the node,
  // so this is the real predicted miss. The dip sits within half a target
  // period of the predicted arrival, so scan a window that wide around it.
  local arr is t_dep + t_h - time:seconds.
  local ca is closest_approach(arr - t_tgt / 2, arr + t_tgt / 2, 48).
  print "Predicted closest approach: " + round(ca["dist"]) + " m at +"
    + round(ca["t"] / 60, 1) + "m.".
  if ca["dist"] > approach_tol {
    print "WARNING: that exceeds the " + round(approach_tol) + " m tolerance.".
    print "  Geometry is all this script fixes; a miss this size means the".
    print "  clock was never lined up. Run detune + loiter first, or let".
    print "  refine try to close a modest miss.".
  }

  print "Next: run refine. run next. run rendezvous.".
}

main().
