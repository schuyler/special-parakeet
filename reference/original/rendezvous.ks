clearscreen.

// How many revolutions of our orbit to search for the closest approach.
// One is right after a transfer burn; more lets the encounter fall on a
// later revolution, e.g. while a phasing orbit is still closing the gap.
parameter orbits is 1.

run common.

// === RENDEZVOUS CALCULATION ===
//
// An intercept crosses the target's path; a rendezvous also matches its
// velocity. This assumes intercept.ks (or luck) already produced a close
// approach, and plans the burn that nulls the relative velocity there.

function separation_at {
  parameter t.
  local s1 is positionat(ship, time:seconds + t).
  local s2 is positionat(target, time:seconds + t).
  local d_pos is s2 - s1.
  return d_pos:mag.
}

function closest_approach {
  // Full periods, not period/2: after a transfer burn the encounter sits at
  // half the period, right on the old window's boundary. The scan handles
  // the multiple local dips a wider window contains; scale the sample count
  // with the window so the grid stays dense enough to catch one dip per rev.
  return minimize_scan(separation_at@, 0, ship:orbit:period * orbits, 0.1, 24 * orbits).
}

function relative_velocity_at {
  parameter t.
  local v1 is velocityat(ship, time:seconds + t):orbit.
  local v2 is velocityat(target, time:seconds + t):orbit.
  return v2 - v1.
}

function rendezvous {
  local dt to closest_approach().
  local dv to relative_velocity_at(dt).
  print "Closest approach: " + round(dt) + "s.".
  print "Velocity: " + dv + " = " + dv:mag + " m/s.".

  local t is time:seconds + dt.
  local nd is node_from_velocity(dv, t).
  add(nd).
}

print "=== RENDEZVOUS ===".

rendezvous().
