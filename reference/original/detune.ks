clearscreen.

// === DETUNE PLANNER (step 1 of 4) ===
//
// The rendezvous procedure, one script per step, each allowed to collapse
// to "nothing to do":
//   1. detune.ks     this: change our period so the clock will line up
//   2. loiter.ks     coast, counting laps, until it does (no burn)
//   3. transfer.ks   the half-ellipse over to the target's radius
//   4. rendezvous.ks match velocities at the encounter
//
// The clock-fixing burn. When loiter.ks reports that no transfer window
// lines up within max_wait — our position drifts toward the window at the
// synodic rate, which vanishes as the two periods converge — this script
// makes a window instead of waiting for one: burn at our next crossing of
// the transfer departure point (the target-apsis antipode) onto an orbit
// whose period is detuned so that after n whole laps we return to that
// point exactly at a chosen window's departure time. The detuned orbit is
// tangent at the burn point, so we return with purely horizontal velocity
// and transfer.ks can take over on the spot.
//
// For each passage k of the target through each apsis, the loiter time dt
// is fixed; the integer n nearest dt / (our period) detunes least and so
// costs least — one candidate per (apsis, k). Candidates are scored by
// total dv including the transfer and match burns that follow, so pe and
// ap arrivals compare fairly. The far side of the detuned orbit sits at
// 2*a_p - r1; any candidate that dips below the safety floor or climbs
// past the SOI is rejected.
//
// Collapse: if some window inside max_wait already lines up to within
// fix_tol on its own, there is nothing to fix — this prints "just loiter"
// and adds no node.
//
// Next: run next (fly the burn), run loiter (watch the window line up),
// run transfer at the return.

parameter max_wait is 0.        // latest arrival, s from now; 0 = default
parameter safe_margin is 10000. // clearance over atmosphere/surface for dips
parameter fix_tol is 25.        // natural miss at/below this: no burn, m/s

run common.
run orbital.

local mu is body:mu.
local r1 is ship:orbit:semimajoraxis.
local v1 is sqrt(mu / r1).
local t1 is ship:orbit:period.
local t_tgt is target:orbit:period.
local a_tgt is target:orbit:semimajoraxis.

local floor_r is body:radius + safe_margin.
if body:atm:exists {
  set floor_r to body:radius + body:atm:height + safe_margin.
}

// visviva, angle_ahead, and time_to_apsis come from orbital.ks.

function describe {
  parameter c.
  return c["aps"] + " n=" + c["n"]
    + ": burn +" + round((c["t_burn"] - time:seconds) / 60, 1) + "m"
    + ", window +" + round((c["t_dep"] - time:seconds) / 60, 1) + "m"
    + ", arr +" + round((c["t_arr"] - time:seconds) / 60, 1) + "m"
    + ", dv " + round(c["total"], 1) + " m/s".
}

print "=== DETUNE PLAN (step 1: fix the clock) ===".

if ship:orbit:eccentricity > 0.02 {
  print "WARNING: orbit e=" + round(ship:orbit:eccentricity, 3)
    + "; the seed assumes circular. Expect refine to work harder.".
}
local rel_inc is relative_inclination().
if rel_inc > 0.5 {
  print "WARNING: planes off by " + round(rel_inc, 2) + " deg; run match_planes first.".
}

if max_wait = 0 {
  local dperiod is abs(t1 - t_tgt).
  if dperiod < t1 * 0.01 {
    set max_wait to 30 * t_tgt.  // co-orbital: no synodic beat to lean on
  } else {
    set max_wait to min(30 * t_tgt, max(3 * t_tgt, 2 * t1 * t_tgt / dperiod)).
  }
  print "No max_wait given; defaulting to " + round(max_wait / 60) + " min.".
}

local horizon is time:seconds + max_wait.
// Also look past the bound, to tell the user what patience would buy.
local hint_horizon is time:seconds + 3 * max_wait.

local candidates is list().
local best_natural is 0.
local has_natural is false.

for aps in list(list("pe", 0, body:radius + target:orbit:periapsis),
                list("ap", 180, body:radius + target:orbit:apoapsis)) {
  local aps_name is aps[0].
  local r2 is aps[2].
  local t_aps0 is time:seconds + time_to_apsis(target:orbit, aps[1]).
  local aps_dir is positionat(target, t_aps0) - body:position.

  // The transfer this detune serves is frozen by Kepler; co-radial
  // degenerates cleanly (t_h = half our own period, zero departure burn).
  local a_t is (r1 + r2) / 2.
  local t_h is constant:pi * sqrt(a_t ^ 3 / mu).
  local dv_arr is abs(visviva(r2, a_tgt) - visviva(r2, a_t)).

  // Our next crossing of this apsis' departure point (its antipode) —
  // the burn happens there so the detuned orbit returns there.
  local ang is mod(angle_ahead(aps_dir) - 180 + 360, 360).
  local t_cross is time:seconds + ang / 360 * t1.
  if t_cross < time:seconds + 120 {
    set t_cross to t_cross + t1.
  }

  local k is 0.
  until t_aps0 + k * t_tgt > hint_horizon or k > 100 {
    local t_arr is t_aps0 + k * t_tgt.
    local t_dep is t_arr - t_h.

    // The no-burn miss at this window, for the collapse check: if some
    // in-bound window already lines up, detuning is pointless.
    if t_dep > time:seconds + 60 and t_arr <= horizon {
      local delta is abs(angle_ahead(aps_dir, t_dep) - 180).
      local dv_fix is r2 * delta * constant:degtorad / max(1, t_h / 2).
      if not has_natural or dv_fix < best_natural["dv_fix"] {
        set has_natural to true.
        set best_natural to lexicon(
          "aps", aps_name, "t_dep", t_dep, "t_arr", t_arr, "dv_fix", dv_fix).
      }
    }

    local dt is t_dep - t_cross.
    if dt > 0.5 * t1 {
      local n is max(1, round(dt / t1)).
      local t_p is dt / n.
      local a_p is (mu * (t_p / (2 * constant:pi)) ^ 2) ^ (1 / 3).
      local r_other is 2 * a_p - r1.
      if r_other > floor_r and r_other < body:soiradius * 0.9 {
        local dv_p is visviva(r1, a_p) - v1.
        // The transfer will depart from the detuned orbit's speed, not v1,
        // so part of the detune burn is money down on the transfer.
        local dv_xfer is abs(visviva(r1, a_t) - visviva(r1, a_p)).
        candidates:add(lexicon(
          "aps", aps_name, "n", n,
          "t_burn", t_cross, "t_dep", t_dep, "t_arr", t_arr,
          "dv_p", dv_p, "t_p", t_p,
          "total", abs(dv_p) + dv_xfer + dv_arr)).
      }
    }
    set k to k + 1.
  }
}

if has_natural and best_natural["dv_fix"] <= fix_tol {
  print "Step 1 collapses: the " + best_natural["aps"] + " window at +"
    + round((best_natural["t_dep"] - time:seconds) / 60, 1)
    + "m already lines up (~" + round(best_natural["dv_fix"], 1) + " m/s).".
  print "No burn needed. Run loiter, then transfer.".
} else {
  // Split candidates by the bound; track the best beyond it for the hint.
  local in_bound is list().
  local has_out is false.
  local best_out is 0.
  for c in candidates {
    if c["t_arr"] <= horizon {
      in_bound:add(c).
    } else if not has_out or c["total"] < best_out["total"] {
      set has_out to true.
      set best_out to c.
    }
  }

  if in_bound:length = 0 {
    print "No detune reaches a window within " + round(max_wait / 60) + " min.".
    if has_out {
      print "Best beyond the bound: " + describe(best_out).
    }
    print "Rerun with a larger max_wait.".
  } else {
    print "Candidates arriving in time (" + in_bound:length + " found, best 3):".
    local winner is in_bound[0].
    local remaining is in_bound:copy().
    local shown is 0.
    until shown = 3 or remaining:length = 0 {
      local best_i is 0.
      local i is 1.
      until i >= remaining:length {
        if remaining[i]["total"] < remaining[best_i]["total"] {
          set best_i to i.
        }
        set i to i + 1.
      }
      print "  " + describe(remaining[best_i]).
      if shown = 0 {
        set winner to remaining[best_i].
      }
      remaining:remove(best_i).
      set shown to shown + 1.
    }

    if has_out and best_out["total"] < 0.8 * winner["total"] {
      print "A larger bound would help: " + describe(best_out).
    }

    until not hasnode {
      remove nextnode.
    }
    add node(winner["t_burn"], 0, 0, winner["dv_p"]).
    print "Node added: " + round(winner["dv_p"], 1) + " m/s prograde in "
      + round(winner["t_burn"] - time:seconds) + "s.".
    print "Loiter " + winner["n"] + " laps of "
      + round(winner["t_p"] / 60, 1) + " min; you re-cross the departure"
      + " point at +" + round((winner["t_dep"] - time:seconds) / 60, 1)
      + "m, on time for the window.".
    print "Next: run next. run loiter to verify. run transfer at the window.".
  }
}
