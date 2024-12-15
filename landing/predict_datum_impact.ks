@lazyglobal off.

run "orbital".

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
  
  // Find time to impact
  local time_to_impact is time_to_altitude(orbit_, height_above_datum, false).
  
  // Get position at impact 
  local impact_ut is time + time_to_impact.
  local impact_orbit is orbit_at_t(orbit_, impact_ut).
  local impact_pos is impact_orbit:position. 
  local impact_lat is body_:geopositionof(impact_pos):lat.
  local impact_lng is body_:geopositionof(impact_pos):lng.
  
  // Account for body rotation during fall
  local rotation_deg is time_to_impact:seconds * (360 / body_:rotationperiod).
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