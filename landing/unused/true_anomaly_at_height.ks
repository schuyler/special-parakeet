@lazyGlobal off.

// Calculate true anomaly where orbit reaches a given height above datum
// Returns -1 if orbit never reaches that height
function true_anomaly_at_height {
  parameter orbit_ is ship:orbit.  // Orbit to analyze
  parameter height_above_datum is 0.

  local body_ is orbit_:body.
  local body_radius is body_:radius + height_above_datum.
  
  // First check if we ever reach this height. The periapsis is the lowest point
  // in the orbit, so if it's above our target height, we'll never get there.
  if orbit_:periapsis > height_above_datum {
    return -1.
  }
  
  // The orbit's shape is defined by these elements:
  local ecc is orbit_:eccentricity.     // How stretched the orbit is (0=circle, 1=parabola)
  local sma is orbit_:semimajoraxis.    // Average of periapsis and apoapsis distances
  local true_anomaly is orbit_:trueanomaly.  // Current angle from periapsis
  
  // The semi-latus rectum (p) is the distance from focus to orbit at ±90° true anomaly
  // It's related to angular momentum and remains constant throughout the orbit
  // For elliptical orbits: p = a(1-e²) where a=semi-major axis, e=eccentricity
  local p is sma * (1 - ecc^2).
  
  // The orbit's radius as a function of true anomaly (θ) is given by:
  // r = p/(1 + e*cos(θ))
  // We want to solve this for θ when r = body_radius + height
  // Rearranging: cos(θ) = (p/r - 1)/e
  local cos_theta is (p/body_radius - 1) / ecc.
  
  // arccos gives us two possible angles where the orbit intersects this height
  // One is on the ascending portion of the orbit, one on the descending
  local theta1 is arccos(cos_theta).  // 0° to 180°
  local theta2 is 360 - theta1.       // 180° to 360°
  
  // We want the next intersection point in our direction of motion
  // This is the theta that's:
  // 1. Ahead of our current true anomaly
  // 2. Less than 180° ahead (the closer intersection point)
  if (theta1 - true_anomaly > 0) and (theta1 - true_anomaly < 180) {
    return theta1.
  } else {
    return theta2.
  }
}