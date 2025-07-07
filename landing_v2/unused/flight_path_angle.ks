@lazyglobal off.
run "true_anomaly_at_height".

// Calculate flight path angle (γ) at a given height in an orbit
// Returns angle between velocity vector and local horizontal plane
// Negative angles indicate descent below horizontal
// Positive angles indicate ascent above horizontal 
function flight_path_angle {
  parameter orbit_ is ship:orbit.  // Orbit to analyze
  parameter height_above_datum is 0.

  local impact_theta is true_anomaly_at_height(orbit_, height_above_datum).
  if impact_theta < 0 {
    return lexicon(
      "angle", 0,
      "theta", -1
    ).
  }
  
  local ecc is orbit_:eccentricity.
  // Flight path angle formula from orbital mechanics:
  // γ = -arcsin(-e*sin(θ) / sqrt(1 + 2e*cos(θ) + e²))
  // Negative sign ensures descent is negative per convention
  local angle is -arcsin(
    -ecc * sin(impact_theta) /
    sqrt(1 + 2*ecc*cos(impact_theta) + ecc^2)
  ).
  
  return lexicon(
    "angle", angle,
    "true_anomaly", impact_theta
  ).
}

//print flight_path_angle().