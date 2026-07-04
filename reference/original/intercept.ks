clearscreen.

run common.

// === INTERCEPT CALCULATION ===

function separation_at {
  parameter t.
  local s1 is positionat(ship, time:seconds + t).
  local s2 is positionat(target, time:seconds + t).
  local d_pos is s2 - s1.
  return d_pos:mag.
}

function closest_approach {
  return minimize(separation_at@, 0, ship:orbit:period / 2, 0.1).
}

function relative_velocity_at {
  parameter t.
  local v1 is velocityat(ship, time:seconds + t):orbit.
  local v2 is velocityat(target, time:seconds + t):orbit.
  return v2 - v1.
}

function intercept {
  local dt to closest_approach().
  local dv to relative_velocity_at(dt).
  print "Closest approach: " + round(dt) + "s.".
  print "Velocity: " + dv + " = " + dv:mag + " m/s.".

  local t is time:seconds + dt.
  local nd is node_from_velocity(dv, t).
  add(nd).
}

print "=== APPROACH ===".

intercept().
