@lazyglobal off.

run "orbital".
run "geoposition_at_t".

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
  local impact_geo is geoposition_at_t(body_:geopositionof(impact_pos), impact_ut).

  return lexicon(
    "geo", impact_geo,
    "position", impact_pos,
    "eta", time_to_impact,
    "time", impact_ut
  ).
}