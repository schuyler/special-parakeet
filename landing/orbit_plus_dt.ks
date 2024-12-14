@lazyglobal off.

function orbit_plus_dt {
  parameter delta_t.   // Time delta in seconds to advance orbit by
  parameter orbit_ is ship:orbit.    // Orbit object to analyze

  // Create new orbit with same elements but shifted epoch. This causes KSP to 
  // evaluate the orbit as if delta_t seconds have passed.
  return createorbit(
    orbit_:inclination,
    orbit_:eccentricity, 
    orbit_:semimajoraxis,
    orbit_:longitudeofascendingnode,
    orbit_:argumentofperiapsis,
    orbit_:meananomalyatepoch,
    orbit_:epoch - delta_t, // Shift epoch back by delta_t seconds
    orbit_:body
  ).
}

function test_orbit_plus_dt {
  local delta_t is 10.
  local position_a is positionat(ship, time:seconds+delta_t).
  local position_b is orbit_plus_dt(delta_t):position.
  local delta_s is (position_a - position_b):mag.
  print "Orbit A: " + position_a.
  print "Orbit B: " + position_b.
  print "Difference: " + delta_s.

  local velocity_a is velocityat(ship, time:seconds+delta_t):orbit.
  local velocity_b is orbit_plus_dt(delta_t):velocity:orbit.
  print "Velocity A: " + velocity_a.
  print "Velocity B: " + velocity_b.
  print "Difference: " + (velocity_a - velocity_b):mag.
}

test_orbit_plus_dt.