@lazyglobal off.

run "anomaly".
run "orbit_at_t".

// Calculate the universal time when an orbit will reach a given true anomaly
function time_to_true_anomaly {
  parameter orbit_.           // Orbit to analyze
  parameter target_theta.     // Desired true anomaly in degrees
  
  // Convert both current and target positions to mean anomaly
  local ecc is orbit_:eccentricity.
  local current_M is true_to_mean_anomaly(orbit_:trueanomaly, ecc).
  local target_M is true_to_mean_anomaly(target_theta, ecc).
  
  // Calculate change in mean anomaly, ensuring we get the next occurrence
  local delta_M is target_M - current_M.
  if delta_M < 0 {
    set delta_M to delta_M + 360.
  }
  
  // Convert to time using orbital period
  local time_delta is delta_M / 360 * orbit_:period.
  return time:seconds + time_delta.
}

// Test function
function test_time_to_true_anomaly {
  local initial_theta is ship:orbit:trueanomaly.
  local test_theta is 180.  // Test reaching apoapsis
  local result_ut is time_to_true_anomaly(ship:orbit, test_theta).
  
  print "Initial true anomaly: " + initial_theta.
  print "Target true anomaly: " + test_theta.
  print "Time until position: " + (result_ut - time:seconds).
  print "Ship ETA:apoapsis: " + ship:orbit:eta:apoapsis.
  print "Difference: " + abs((result_ut - time:seconds) - ship:orbit:eta:apoapsis).
}

// test_time_to_true_anomaly.