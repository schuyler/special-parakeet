@lazyglobal off.

// Convert true anomaly to mean anomaly for an orbit
function true_to_mean_anomaly {
  parameter theta.     // True anomaly in degrees
  parameter ecc.      // Orbit eccentricity
  
  // First convert to eccentric anomaly
  local E is arctan2(sqrt(1-ecc^2) * sin(theta), ecc + cos(theta)).
  
  // Then to mean anomaly
  local M is E - ecc * sin(E) * constant:radtodeg.
  
  // Return normalized to 0-360
  if M < 0 { 
    return M + 360. 
  }
  return M.
}

// Convert mean anomaly to true anomaly for an orbit
function mean_to_true_anomaly {
  parameter M.        // Mean anomaly in degrees
  parameter ecc.      // Orbit eccentricity
  parameter epsilon is 0.0001.  // Desired precision in degrees
  
  // Solve Kepler's equation iteratively to find E (eccentric anomaly)
  local E is M.  // Initial guess
  local delta is epsilon + 1.
  local count is 0.
  until delta < epsilon and count < 100 {
    local E_next is M + ecc * sin(E) * constant:radtodeg.
    set delta to abs(E_next - E).
    set E to E_next.
    set count to count + 1.
  }
  
  // Convert to true anomaly
  local theta is arctan2(sqrt(1-ecc^2) * sin(E), cos(E) - ecc).
  
  // Return normalized to 0-360
  if theta < 0 {
    return theta + 360.
  }
  return theta.
}