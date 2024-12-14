// Create a new orbit object representing the result of a maneuver
function orbit_after_maneuver {
  parameter burn_vector.  // The delta-v vector to apply
  parameter burn_time.    // When to apply the burn (in universal time)
  
  local current_orbit is ship:orbit.
  local body_ is current_orbit:body.
  local ut is time:seconds + burn_time.
  
  // Get the state vectors at burn time
  local pos is positionat(ship, ut) - body_:position.
  local vel is velocityat(ship, ut):orbit + burn_vector.
  
  // Create and return the new orbit
  return createorbit(pos, vel, body_, ut).
}