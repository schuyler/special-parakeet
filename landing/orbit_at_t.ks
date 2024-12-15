@lazyglobal off.

function orbit_at_t {
  parameter orbit_ is ship:orbit.    // Orbit object to analyze
  parameter ut is time.   // Universal Time to evaluate orbit at

  // Create new orbit with same elements but shifted epoch. This causes KSP to 
  // evaluate the orbit as if delta_t seconds have passed.
  return createorbit(
    orbit_:inclination,
    orbit_:eccentricity, 
    orbit_:semimajoraxis,
    orbit_:longitudeofascendingnode,
    orbit_:argumentofperiapsis,
    orbit_:meananomalyatepoch,
    (orbit_:epoch + (time - ut)):seconds, // Shift epoch back by delta_t seconds
    orbit_:body
  ).
}

function test_orbit_at_t {
  local ut is time:seconds+1000.
  local position_a is positionat(ship, ut).
  local position_b is orbit_at_t(ship:orbit, ut):position.
  local delta_s is (position_a - position_b):mag.
  print "Orbit A: " + position_a.
  print "Orbit B: " + position_b.
  print "Difference: " + delta_s.

  local velocity_a is velocityat(ship, ut):orbit.
  local velocity_b is orbit_at_t(ship:orbit, ut):velocity:orbit.
  print "Velocity A: " + velocity_a.
  print "Velocity B: " + velocity_b.
  print "Difference: " + (velocity_a - velocity_b):mag.
}

//test_orbit_at_t.