@lazyglobal off.

run "true_anomaly_at_height".
run "orbit_at_t".

// Calculate predicted impact point for current vessel using orbital elements
// height_above_datum: meters above the reference sphere to check for intersection
function predict_datum_impact {
  parameter orbit_ is ship:orbit.
  parameter height_above_datum is 0.
  
  local body_ is orbit_:body.
  
  // If periapsis is above intersection height, no impact possible
  if orbit_:periapsis > height_above_datum {
    return lexicon(
      "lat", 0,
      "lng", 0,
      "time", -1
    ).
  }
  
  // Get orbital elements for impact calculation
  local ecc is orbit_:eccentricity.
  local true_anomaly is orbit_:trueanomaly.  
  local impact_true_anomaly to true_anomaly_at_height(orbit_, height_above_datum).

  // Calculate time to impact using mean anomaly difference
  local current_E is arctan2(sqrt(1-ecc^2) * sin(true_anomaly), ecc + cos(true_anomaly)).
  local impact_E is arctan2(sqrt(1-ecc^2) * sin(impact_true_anomaly), ecc + cos(impact_true_anomaly)).
  
  local current_M is current_E - ecc * sin(current_E).
  local impact_M is impact_E - ecc * sin(impact_E).
  local delta_M is impact_M - current_M.
  if delta_M < 0 { set delta_M to delta_M + 360. }
  local time_to_impact is delta_M / 360 * orbit_:period.
  
  // Get position at impact 
  local impact_ut is time:seconds + time_to_impact.
  local impact_orbit is orbit_at_t(orbit_, impact_ut).
  local impact_pos is impact_orbit:position. 
  local impact_lat is body_:geopositionof(impact_pos):lat.
  local impact_lng is body_:geopositionof(impact_pos):lng.
  
  // Account for body rotation during fall
  local rotation_deg is time_to_impact * (360 / body_:rotationperiod).
  local final_lng is impact_lng - rotation_deg.

  // Normalize longitude to -180 to 180
  if final_lng > 180 {
    set final_lng to final_lng - 360.
  } else if final_lng < -180 {
    set final_lng to final_lng + 360.
  }
  
  return lexicon(
    "geo", body_:geopositionlatlng(impact_lat, final_lng),
    "eta", time_to_impact,
    "ut", impact_ut
  ).
}