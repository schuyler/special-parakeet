@lazyglobal off.

run "orbit_plus_dt".

// Create a new orbit object representing the result of a maneuver
function orbit_after_maneuver {
  parameter burn_vector.  // The delta-v vector to apply
  parameter burn_time.    // When to apply the burn (in universal time)
  parameter orbit_ is ship:orbit.  // The current orbit
  
  local dt is burn_time - time:seconds.
 
  // Get the state vectors at burn time
  local new_orbit is orbit_plus_dt(dt, orbit_).
  local pos is new_orbit:position - new_orbit:body:position.
  local vel is new_orbit:velocity:orbit + burn_vector.
  
  // The arguments of CREATEORBIT(pos, vel, body, ut) are swizzled 
  // https://github.com/KSP-KOS/KOS/issues/3009#issuecomment-1405377308
  function swizzle { parameter vec. return V(+VEC:X,+VEC:Z,+VEC:Y). }

  // Create and return the new orbit as if burn_time was right now
  return createorbit(swizzle(pos), swizzle(vel), new_orbit:body, time:seconds).
}

function test_orbit_after_maneuver {
  clearscreen.

  local v_ is ship:velocity:orbit.
  local burn_time is time:seconds + 60.
  local burn_vector is v_:normalized * 100.

  print "Current velocity: " + v_.
  print "Current position: " + ship:position.
  print "".
  print "Predicted position: " + positionat(ship, burn_time).
  print "Predicted velocity: " + velocityat(ship, burn_time):orbit.
  print "".
  print "Burn vector: " + burn_vector.
  print "".
  local new_orbit is orbit_after_maneuver(burn_vector, burn_time, ship:orbit).
  print "New orbit position: " + new_orbit:position.
  print "New orbit velocity: " + new_orbit:velocity:orbit.

  print "Difference: " + (new_orbit:velocity:orbit - velocityat(ship, burn_time):orbit):mag.
}

// test_orbit_after_maneuver.