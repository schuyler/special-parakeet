clearscreen.

// === MEET (the whole rendezvous, in sittable pieces) ===
//
// Master script for the four-step procedure: run every planner in order,
// fly each burn as it comes, and stop the moment the next thing to do is
// further away than we are willing to sit. On stopping it prints the time
// to set an alarm for (stock alarm clock or KAC; see
// notes/kos-stock-alarm-addon.md for making kOS set it itself someday).
// When the alarm rings, come back and just `run meet.` again: nothing is
// saved between runs — the orbit is the state — and every completed step
// collapses to a no-op, so the pipeline resumes exactly where it stopped.
// Running `run next.` by hand first works too; meet sweeps the spent node
// it leaves behind and carries on.
//
//   match_planes -> detune -> [loiter] -> transfer -> refine -> rendezvous
//
// The loiter is the only wait that gets a hard cap (max_loiter): the
// synodic clock can be arbitrarily slow, so detune must be told how long
// a wait is worth enduring before it manufactures a window instead. Every
// other wait takes as long as it takes — patience only decides whether we
// sit through it now (warping) or come back for it on an alarm.
//
// The dv budget covers the burns still ahead of this run. Because done
// steps collapse, a rerun after an alarm re-checks the remaining cost
// against the same figure — no bookkeeping across runs needed.

parameter patience is 15.    // min: longest wait we sit through here and now
parameter max_loiter is 0.   // s: cap on the window search; 0 = detune's default
parameter dv_budget is 1000. // m/s: for the burns still ahead

run common.
run orbital.

local approach_tol is plan_approach_tol. // same gate transfer.ks uses
local spent is 0.

// Fly the next node now, unless it busts the budget or the wait outruns
// patience. Returns false when this run should end (reason printed).
function fly_gate {
  parameter why.
  local nd is nextnode.
  if spent + nd:deltav:mag > dv_budget {
    print "STOP: " + why + " needs " + round(nd:deltav:mag, 1) + " m/s;"
      + " only " + round(dv_budget - spent, 1) + " m/s of budget remains.".
    print "Node kept for inspection. Rerun with a bigger dv_budget, or replan.".
    return false.
  }
  if nd:eta > patience * 60 {
    print "PAUSE: " + why + " is " + round(nd:eta / 60, 1)
      + " min out — past patience (" + patience + " min).".
    print "Set a maneuver alarm (node at UT " + round(nd:time)
      + "), then `run meet.` to resume.".
    return false.
  }
  print "Flying " + why + ": " + round(nd:deltav:mag, 1) + " m/s.".
  set spent to spent + nd:deltav:mag.
  execute_node(nd).
  wait 1.
  return true.
}

function main {
  if not hastarget {
    print "No target set. Pick one in map view and rerun.".
    return.
  }

  // The whole procedure assumes laps are free; on an orbit that dips
  // below the body's safe altitude (core/safety.ks) they are anything
  // but. Refuse up front rather than plan a loiter we won't survive.
  if ship:orbit:periapsis < safe_alt(body) {
    print "STOP: periapsis " + round(ship:orbit:periapsis / 1000, 1)
      + " km is below " + body:name + "'s safe altitude ("
      + round(safe_alt(body) / 1000, 1) + " km).".
    print "Raise it first; a rendezvous starts from a stable orbit.".
    return.
  }

  print "=== MEET: " + target:name + " ===".

  // Resume bookkeeping. A previous run (or a manual `run next`, which
  // keeps the executed node) may have left nodes behind: sweep any that
  // are spent or whose time has passed, then fly whatever real one is
  // still pending before planning anything new.
  local swept is 0.
  until not hasnode or (nextnode:deltav:mag >= plan_spent_dv and nextnode:eta > 0) {
    remove nextnode.
    wait 0.
    set swept to swept + 1.
  }
  if swept > 0 {
    print "Swept " + swept + " spent/stale node(s).".
  }
  if hasnode {
    print "Resuming: a planned burn is pending.".
    if not fly_gate("the pending burn") { return. }
  }

  // If the flight plan already meets the target, the first three steps
  // have nothing left to say — skip straight to the match burn. This is
  // also what makes rerunning meet mid-transfer safe: detune must not be
  // asked to plan from the middle of the ellipse.
  local ca is closest_approach(0, ship:orbit:period + target:orbit:period, 96).
  if ca["dist"] > approach_tol {

    // Step 0: planes. Collapses below plan_inc_matched.
    run match_planes.
    if hasnode {
      if not fly_gate("the plane match") { return. }
    }

    // Steps 1-2: the clock. detune collapses to "just loiter" when a
    // natural window lines up; either way it exports the window time.
    run detune(max_loiter).
    if detune_status = "none" {
      print "STOP: no transfer window inside the loiter cap.".
      print "Rerun with a bigger max_loiter (or 0 for detune's default).".
      return.
    }
    if detune_status = "burn" {
      if not fly_gate("the detune burn") { return. }
    }
    local dt_win is detune_t_dep - time:seconds.
    if dt_win > patience * 60 {
      print "PAUSE: the transfer window opens in " + round(dt_win / 60, 1)
        + " min — past patience (" + patience + " min).".
      print "Set an alarm for UT " + round(detune_t_dep - plan_burn_lead)
        + " (window minus " + round(plan_burn_lead / 60, 1)
        + " min), then `run meet.` to resume.".
      return.
    }
    if dt_win > plan_burn_lead + plan_min_lead {
      print "Warping to the transfer window.".
      warpto(detune_t_dep - plan_burn_lead).
      wait until time:seconds >= detune_t_dep - plan_burn_lead.
    }

    // Step 3: the geometry, plus polish. When transfer collapses
    // co-radial the encounter comes ballistically; nothing to fly.
    run transfer.
    if hasnode {
      run refine.
      local ca_x is closest_approach(0, ship:orbit:period + target:orbit:period, 96).
      if ca_x["dist"] > approach_tol {
        print "STOP: even refined, the transfer misses by "
          + round(ca_x["dist"]) + " m.".
        print "Geometry cannot fix a broken clock; that window was not".
        print "really open. Node kept: try loiter and detune by hand.".
        return.
      }
      if not fly_gate("the transfer burn") { return. }
    }
  } else {
    print "Flight plan already meets the target ("
      + round(ca["dist"]) + " m); skipping to the match burn.".
  }

  // Step 4: match velocities. Collapses once we're already matched.
  run rendezvous.
  if hasnode {
    if not fly_gate("the match burn") { return. }
  }

  print "=== MEET: done ===".
  print "Separation " + round(target:position:mag) + " m, relative speed "
    + round((target:velocity:orbit - ship:velocity:orbit):mag, 1) + " m/s.".
  print "Next: run dock2.".
}

main().
