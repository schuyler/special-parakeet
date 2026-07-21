clearscreen.

// === INTERCEPT PLANNER ===
//
// Put us on a transfer orbit that intercepts the current target — crossing
// its path; matching velocities there is rendezvous.ks's job. Plans from a
// (near-)circular orbit,
// after match_planes has put us in the target's plane. The one real knob
// is max_wait: the latest acceptable *arrival* time, in seconds from now.
// Every strategy arriving inside that bound is scored by total delta-v and
// the cheapest wins. Don't like the answer? Delete the node and rerun with
// a larger bound — more time can only find cheaper plans.
//
// Strategies considered, all arriving at one of the target's apsides
// (the only points on an eccentric orbit where its velocity is purely
// horizontal, so the match burn is a pure magnitude change):
//  - xfer:  tangent half-ellipse from our radius to the apsis radius,
//           timed against each of the target's apsis passages in the
//           window. Timing misfit shows up as an along-track miss, priced
//           into the score as a mid-course correction estimate.
//  - phase: when our radius is already the apsis radius (co-tangent),
//           loiter n laps on a detuned-period orbit tangent at the burn
//           point, sized so we return exactly when the target arrives.
//
// This script only creates the departure node. Then: run refine (tune
// the node against the real predicted miss), run next (fly it), run
// rendezvous (plan the match burn at closest approach).

parameter max_wait is 0.        // latest arrival, s from now; 0 = default
parameter safe_margin is 10000. // clearance over atmosphere/surface for dips

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

function visviva {
  parameter r_, a_.
  return sqrt(mu * (2 / r_ - 1 / a_)).
}

// Angle from our position at time t to a body-centered direction, measured
// forward along our direction of motion, [0, 360). Handedness-free: built
// from dot products against our own radial and prograde directions.
function angle_ahead {
  parameter dir_.
  parameter t is time:seconds.
  local rdir is (positionat(ship, t) - body:position):normalized.
  local vdir is velocityat(ship, t):orbit:normalized.
  local ang is arctan2(vdot(dir_:normalized, vdir), vdot(dir_:normalized, rdir)).
  if ang < 0 {
    set ang to ang + 360.
  }
  return ang.
}

function time_to_apsis {
  parameter ob.
  parameter aps_m. // mean anomaly of the apsis: 0 = pe, 180 = ap
  local m is mean_anomaly_at_t(ob).
  return mod(aps_m - m + 360, 360) / 360 * ob:period.
}

function describe {
  parameter c.
  return c["kind"] + "->" + c["aps"]
    + (choose " n=" + c["n"] if c["kind"] = "phase" else "")
    + ": dep +" + round((c["t_dep"] - time:seconds) / 60, 1) + "m"
    + ", arr +" + round((c["t_arr"] - time:seconds) / 60, 1) + "m"
    + ", dv " + round(c["total"], 1) + " m/s".
}

print "=== INTERCEPT PLAN ===".

if ship:orbit:eccentricity > 0.02 {
  print "WARNING: orbit e=" + round(ship:orbit:eccentricity, 3)
    + "; the seed assumes circular. Expect refine to work harder.".
}
local rel_inc is vang(orbit_normal(ship), orbit_normal(target)).
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
  local aps_m is aps[1].
  local r2 is aps[2].
  local t_aps0 is time:seconds + time_to_apsis(target:orbit, aps_m).

  if abs(r1 - r2) / r1 > 0.05 {
    // Tangent transfer: the departure point is fixed (the apsis antipode)
    // and so is the flight time, so each target apsis passage k defines
    // one candidate. Our angular offset from the antipode at the ideal
    // departure time is this window's phasing error: it becomes an
    // along-track miss of about r2 * delta at arrival, priced here as an
    // impulsive fix applied with half the transfer still to fly.
    local a_t is (r1 + r2) / 2.
    local t_h is constant:pi * sqrt(a_t ^ 3 / mu).
    local dv_dep is visviva(r1, a_t) - v1.
    local dv_arr is abs(visviva(r2, a_tgt) - visviva(r2, a_t)).
    local k is 0.
    until t_aps0 + k * t_tgt > hint_horizon or k > 100 {
      local t_arr is t_aps0 + k * t_tgt.
      local t_dep is t_arr - t_h.
      if t_dep > time:seconds + 60 {
        local aps_dir is positionat(target, t_arr) - body:position.
        local delta is abs(angle_ahead(aps_dir, t_dep) - 180).
        local dv_fix is r2 * delta * constant:degtorad / max(1, t_h / 2).
        candidates:add(lexicon(
          "kind", "xfer", "aps", aps_name, "n", 0,
          "t_dep", t_dep, "t_arr", t_arr,
          "dv_dep", dv_dep, "dv_arr", dv_arr,
          "total", abs(dv_dep) + dv_arr + dv_fix)).
      }
      set k to k + 1.
    }
  } else {
    // Co-tangent: we already fly at the apsis radius, so the "transfer"
    // degenerates and only the clock is wrong. Burn at our next crossing
    // of the apsis direction, loiter n laps on an orbit with period
    // (t_arr - t_cross) / n, and return exactly as the target's k-th
    // apsis passage brings it to the burn point. For each k, the integer
    // n nearest dt/t1 detunes our period least and so costs least.
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
            "kind", "phase", "aps", aps_name, "n", n,
            "t_dep", t_cross, "t_arr", t_arr,
            "dv_dep", dv_p, "dv_arr", dv_arr,
            "total", abs(dv_p) + dv_arr)).
        }
      }
      set k to k + 1.
    }
  }
}

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
  print "Next: run refine. run next. run rendezvous.".
}
