clearscreen.

// How many revolutions of our orbit to search for the closest approach.
// One is right after a transfer burn; more lets the encounter fall on a
// later revolution, e.g. while a phasing orbit is still closing the gap.
parameter orbits is 1.
parameter min_dv is -1. // relative speed at/below this: matched, no node; -1 = policy

run common.
run orbital.  // closest_approach, relative_velocity_at

if min_dv < 0 {
  set min_dv to plan_matched_dv.
}

// === RENDEZVOUS (step 4 of 4) ===
//
// The rendezvous procedure, one script per step, each allowed to collapse
// to "nothing to do":
//   1. detune.ks     change our period so the clock will line up
//   2. loiter.ks     coast, counting laps, until it does (no burn)
//   3. transfer.ks   the half-ellipse over to the target's radius
//   4. rendezvous.ks this: match velocities at the encounter
//
// An intercept crosses the target's path; a rendezvous also matches its
// velocity. This assumes the detune/loiter/transfer pipeline (or luck)
// already produced a close approach, and plans the burn that nulls the
// relative velocity there.
//
// Collapse: if the relative speed at the encounter is already inside
// min_dv there is nothing left to null — dock2 owns the last few m/s —
// so no node is added. The prediction honors planned nodes, so rerunning
// with the match node still pending collapses the same way instead of
// stacking a second near-zero node.

print "=== RENDEZVOUS (step 4: match velocities) ===".

if not hastarget {
  print "No target set.".
} else {
  // Full periods, not period/2: after a transfer burn the encounter sits at
  // half the period, right on the old window's boundary. The scan handles
  // the multiple local dips a wider window contains; scale the sample count
  // with the window so the grid stays dense enough to catch one dip per rev.
  local ca is closest_approach(0, ship:orbit:period * orbits, 24 * orbits).
  local dv is relative_velocity_at(ca["t"]).
  print "Closest approach: " + round(ca["dist"]) + " m at +"
    + round(ca["t"] / 60, 1) + "m.".
  print "Relative velocity: " + round(dv:mag, 2) + " m/s.".

  if dv:mag <= min_dv {
    print "Rendezvous collapses: already matched to within "
      + min_dv + " m/s. No node.".
    print "Next: run dock2.".
  } else {
    until not hasnode {
      remove nextnode.
    }
    local nd is node_from_velocity(dv, time:seconds + ca["t"]).
    add nd.
    print "Match node added: " + round(dv:mag, 1) + " m/s in "
      + round(ca["t"]) + "s.".
    print "Next: run next. Then dock2 for the final approach.".
  }
}
