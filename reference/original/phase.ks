clearscreen.

// === PHASING PLANNER ===
//
// When we already fly at the target's apsis radius, there is no transfer to
// build — only a clock to fix. This script sizes a phasing orbit: burn at
// our next crossing of the apsis direction onto an orbit with a slightly
// detuned period, loiter n laps, and return to the burn point exactly as the
// target's k-th apsis passage brings it there. Matching velocities once we
// arrive is rendezvous.ks's job; this script only creates the departure node.
//
// We aim at an apsis (periapsis or apoapsis) because there the target's
// velocity is purely horizontal, so the later match burn is a pure magnitude
// change. For each arrival passage k, the loiter time dt = t_arr - t_cross is
// fixed; the integer n nearest dt/t1 detunes our period least and so costs
// least, giving one candidate per k. The other apsis of the phasing orbit
// sits at 2*a_p - r1; we reject any that dips below the floor or climbs past
// the SOI.
//
// The knob is max_wait: the latest acceptable arrival, seconds from now.
// Every candidate arriving inside it is scored by total delta-v and the
// cheapest wins; a larger bound can only find cheaper plans. If our radius is
// NOT the target's apsis radius (differs by more than tol_coradial) there is
// a real transfer to fly — run intercept.ks instead.
//
// After adding the node we predict the closest approach it actually produces
// and warn if that gap exceeds approach_tol.
//
// Next: run refine (tune the node against the real predicted miss), run next
// (fly it), run rendezvous (plan the match burn at closest approach).

parameter max_wait is 0.        // latest arrival, s from now; 0 = default
parameter safe_margin is 10000. // clearance over atmosphere/surface for dips
parameter approach_tol is 2000. // warn if predicted miss exceeds this, m
parameter tol_coradial is 0.01. // |r1-r2|/r1 must be at/below this to phase

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

// visviva, angle_ahead, time_to_apsis, closest_approach come from orbital.ks.

function describe {
  parameter c.
  return c["aps"] + " n=" + c["n"]
    + ": dep +" + round((c["t_dep"] - time:seconds) / 60, 1) + "m"
    + ", arr +" + round((c["t_arr"] - time:seconds) / 60, 1) + "m"
    + ", dv " + round(c["total"], 1) + " m/s".
}

print "=== PHASING PLAN ===".

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
local offradius_count is 0.

for aps in list(list("pe", 0, body:radius + target:orbit:periapsis),
                list("ap", 180, body:radius + target:orbit:apoapsis)) {
  local aps_name is aps[0].
  local aps_m is aps[1].
  local r2 is aps[2].
  local t_aps0 is time:seconds + time_to_apsis(target:orbit, aps_m).

  if abs(r1 - r2) / r1 > tol_coradial {
    // Our radius and this apsis differ enough that a real half-ellipse links
    // them: that is intercept.ks's transfer, not a phasing problem. Skip it.
    set offradius_count to offradius_count + 1.
    print aps_name + " is " + round(abs(r1 - r2) / r1 * 100, 1)
      + "% off our radius; that is a transfer (intercept territory).".
  } else {
    // Burn at our next crossing of the apsis direction, loiter n laps on an
    // orbit of period (t_arr - t_cross) / n, and return exactly as the
    // target's k-th apsis passage brings it to the burn point. For each k the
    // integer n nearest dt/t1 detunes our period least and so costs least.
    local aps_dir is positionat(target, t_aps0) - body:position.
    local t_cross is time:seconds + angle_ahead(aps_dir) / 360 * t1.
    if t_cross < time:seconds + 120 {
      set t_cross to t_cross + t1.
    }
    local k is 0.
    until t_aps0 + k * t_tgt > hint_horizon or k > 100 {
      local t_arr is t_aps0 + k * t_tgt.
      local dt is t_arr - t_cross.
      if dt > 0.5 * t1 {
        local n is max(1, round(dt / t1)).
        local t_p is dt / n.
        local a_p is (mu * (t_p / (2 * constant:pi)) ^ 2) ^ (1 / 3).
        local r_other is 2 * a_p - r1.
        if r_other > floor_r and r_other < body:soiradius * 0.9 {
          local dv_p is visviva(r1, a_p) - v1.
          local dv_arr is abs(visviva(r1, a_tgt) - visviva(r1, a_p)).
          candidates:add(lexicon(
            "aps", aps_name, "n", n,
            "t_dep", t_cross, "t_arr", t_arr,
            "dv_dep", dv_p, "dv_arr", dv_arr,
            "total", abs(dv_p) + dv_arr)).
        }
      }
      set k to k + 1.
    }
  }
}

if offradius_count = 2 {
  print "Neither apsis is at our radius: this is a transfer, not a phasing".
  print "problem. Run intercept.ks instead.".
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
    print "No plan arrives within " + round(max_wait / 60) + " min.".
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
    add node(winner["t_dep"], 0, 0, winner["dv_dep"]).
    print "Node added: " + round(winner["dv_dep"], 1) + " m/s prograde in "
      + round(winner["t_dep"] - time:seconds) + "s.".
    print "Estimated match burn at arrival: " + round(winner["dv_arr"], 1) + " m/s.".

    // Approach gate: with the departure node in the flight plan, ask what gap
    // the phasing orbit actually leaves at the encounter. positionat honors
    // the node, so this is the real predicted miss, not the seed's estimate.
    // The dip sits within half a target period of the predicted arrival, so
    // scan a window that wide around it.
    local arr is winner["t_arr"] - time:seconds.
    local ca is closest_approach(arr - t_tgt / 2, arr + t_tgt / 2, 48).
    print "Predicted closest approach: " + round(ca["dist"]) + " m at +"
      + round(ca["t"] / 60, 1) + "m.".
    if ca["dist"] > approach_tol {
      print "WARNING: that exceeds the " + round(approach_tol) + " m tolerance.".
      print "  The geometry may want intercept.ks, or the bound is too tight".
      print "  to reach a well-phased window. Node kept; refine may close it.".
    }

    print "Next: run refine. run next. run rendezvous.".
  }
}
