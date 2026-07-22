// === PLANNING POLICY ===
//
// The targeting pipeline's shared constants, in one home. Each of these
// appeared verbatim in two or more of match_planes / detune / loiter /
// transfer / rendezvous / meet before moving here, and a threshold
// edited in one script but not another makes the steps quietly disagree
// about what counts as matched, met, or worth waiting for. One-off
// numerics (scan densities, solver tolerances, brackets) stay in the
// scripts, next to the comments that justify them.
//
// Scripts can't read these in parameter defaults — kOS binds parameters
// before `run common.` executes — so tunable ones use a -1 sentinel:
// `parameter fix_tol is -1.` then `if fix_tol < 0 { set fix_tol to
// plan_fix_tol. }` after the runs.
//
// Three ladders live here whose ORDER is the real invariant:
//
//   dv:     execute_node burns a node down below ~0.1 m/s
//         < plan_spent_dv   (meet sweeps the leftover as spent)
//         < plan_matched_dv (rendezvous declares victory).
//           Tighten the sweep or loosen the match and resume breaks:
//           spent nodes survive, or live correction nodes get eaten.
//
//   time:   plan_min_lead  (youngest departure transfer will plan)
//         < plan_burn_lead (meet's window-warp buffer, detune's
//           youngest burn). Meet must arrive at the window early
//           enough that transfer still sees the crossing ahead.
//
//   angle:  plan_inc_matched (match_planes stops burning)
//         < plan_inc_warn    (the planners start nagging).
//           Hysteresis: a just-matched plane must not re-trip the nag.

global plan_e_circular is 0.02.   // e above this: the circular seeds warn
global plan_inc_warn is 0.5.      // deg planes-off: nag "run match_planes first"
global plan_inc_matched is 0.05.  // deg below which match_planes is a no-op
global plan_fix_tol is 25.        // m/s: a natural window this cheap = just wait
global plan_approach_tol is 2000. // m: "the flight plan meets the target"
global plan_matched_dv is 0.5.    // m/s: velocities count as matched
global plan_spent_dv is 0.2.      // m/s: a node this small is a spent one
global plan_min_lead is 60.       // s: youngest flyable departure
global plan_burn_lead is 120.     // s: youngest plannable burn; window buffer
global plan_hint_factor is 3.     // look this far past max_wait for hints
global plan_hint_better is 0.8.   // hint only if beyond-bound beats this fraction

// The default bound on the window search when the caller gives none.
// Two synodic beats reaches any phase alignment; floored at three
// target laps so near-synchronous chases still see a few windows;
// capped at thirty. Within 1% of co-orbital there is no synodic beat
// to lean on, and the cap is all there is.
function default_max_wait {
  parameter t1, t_tgt.
  local dperiod is abs(t1 - t_tgt).
  if dperiod < t1 * 0.01 {
    return 30 * t_tgt.
  }
  return min(30 * t_tgt, max(3 * t_tgt, 2 * t1 * t_tgt / dperiod)).
}
