clearscreen.

// === LOITER REPORT (step 2 of 4) ===
//
// The rendezvous procedure, one script per step, each allowed to collapse
// to "nothing to do":
//   1. detune.ks     change our period so the clock will line up
//   2. loiter.ks     this: coast, counting laps, until it does (no burn)
//   3. transfer.ks   the half-ellipse over to the target's radius
//   4. rendezvous.ks match velocities at the encounter
//
// Loitering is the step that costs nothing: stay on the current orbit and
// count laps until a transfer window opens. This script plans no burn and
// adds no node — it only does the counting. A transfer window is a moment
// when departing on the frozen half-ellipse would arrive at the target's
// apsis just as the target does; we are in the window when we cross the
// departure point (the apsis antipode) at that moment. For each upcoming
// passage of the target through each apsis, this prints when the ideal
// departure falls and how far off our position will be then — as an angle,
// and as the rough mid-course correction that miss would cost to fix.
//
// If the best miss inside max_wait is cheap, the answer is "just wait":
// warp there, then run transfer. If it is not — our position drifts toward
// the window at the synodic rate, which goes to zero as the two periods
// converge — then no amount of waiting inside the bound helps, and step 1
// exists for exactly that: run detune. Run this again afterward to watch
// the manufactured window line up.

parameter max_wait is 0. // latest acceptable arrival, s from now; 0 = default
parameter fix_tol is 25. // est. correction at/below this: just wait, m/s

run common.
run orbital.

local mu is body:mu.
local r1 is ship:orbit:semimajoraxis.
local t1 is ship:orbit:period.
local t_tgt is target:orbit:period.

// visviva, angle_ahead, and time_to_apsis come from orbital.ks.

function describe {
  parameter c.
  return c["aps"] + ": dep +" + round((c["t_dep"] - time:seconds) / 60, 1)
    + "m, arr +" + round((c["t_arr"] - time:seconds) / 60, 1) + "m"
    + ", miss " + round(c["delta"], 1) + " deg"
    + " (~" + round(c["dv_fix"], 1) + " m/s to fix)".
}

print "=== LOITER REPORT (step 2: count laps) ===".

if ship:orbit:eccentricity > 0.02 {
  print "WARNING: orbit e=" + round(ship:orbit:eccentricity, 3)
    + "; window timing assumes near-circular.".
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

for aps in list(list("pe", 0, body:radius + target:orbit:periapsis),
                list("ap", 180, body:radius + target:orbit:apoapsis)) {
  local aps_name is aps[0].
  local r2 is aps[2].
  local t_aps0 is time:seconds + time_to_apsis(target:orbit, aps[1]).
  local aps_dir is positionat(target, t_aps0) - body:position.

  // The transfer's geometry is frozen by Kepler: semimajor axis from the
  // two radii, flight time half its period. Co-radial degenerates cleanly
  // (t_h = half our own period) so both apsides get the same treatment.
  local a_t is (r1 + r2) / 2.
  local t_h is constant:pi * sqrt(a_t ^ 3 / mu).

  local k is 0.
  until t_aps0 + k * t_tgt > hint_horizon or k > 100 {
    local t_arr is t_aps0 + k * t_tgt.
    local t_dep is t_arr - t_h.
    if t_dep > time:seconds + 60 {
      // Our angular distance from the departure point at the ideal moment
      // is the window's miss; it becomes an along-track miss of about
      // r2 * delta at arrival, priced as an impulsive fix applied with
      // half the transfer still to fly.
      local delta is abs(angle_ahead(aps_dir, t_dep) - 180).
      local dv_fix is r2 * delta * constant:degtorad / max(1, t_h / 2).
      candidates:add(lexicon(
        "aps", aps_name, "t_dep", t_dep, "t_arr", t_arr,
        "delta", delta, "dv_fix", dv_fix)).
    }
    set k to k + 1.
  }
}

// Split candidates by the bound; track the best beyond it for the hint.
local in_bound is list().
local has_out is false.
local best_out is 0.
for c in candidates {
  if c["t_arr"] <= horizon {
    in_bound:add(c).
  } else if not has_out or c["dv_fix"] < best_out["dv_fix"] {
    set has_out to true.
    set best_out to c.
  }
}

if in_bound:length = 0 {
  print "No window opens within " + round(max_wait / 60) + " min.".
  if has_out {
    print "Best beyond the bound: " + describe(best_out).
  }
  print "Run detune to make one, or rerun with a larger max_wait.".
} else {
  print "Windows inside the bound (" + in_bound:length + " found, best 3):".
  local winner is in_bound[0].
  local remaining is in_bound:copy().
  local shown is 0.
  until shown = 3 or remaining:length = 0 {
    local best_i is 0.
    local i is 1.
    until i >= remaining:length {
      if remaining[i]["dv_fix"] < remaining[best_i]["dv_fix"] {
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

  if winner["dv_fix"] <= fix_tol {
    print "The clock lines up on its own — step 1 collapses.".
    print "Loiter " + round((winner["t_dep"] - time:seconds) / t1, 1)
      + " laps (" + round((winner["t_dep"] - time:seconds) / 60, 1)
      + " min), then run transfer.".
  } else {
    print "No natural window inside the bound comes closer than "
      + round(winner["dv_fix"], 1) + " m/s.".
    print "Waiting longer cannot beat the synodic drift: run detune.".
  }

  if has_out and best_out["dv_fix"] < 0.8 * winner["dv_fix"] {
    print "A larger bound would help: " + describe(best_out).
  }
}
