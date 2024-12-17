@lazyglobal off.

// Calculate great circle distance between two points on a spherical body
// geo1, geo2: Geocoordinate objects
// Returns: Distance in meters
function surface_distance {
  parameter geo1, geo2.
  
  // Convert positions to radians
  local rlat1 is geo1:lat * constant:degtorad.
  local rlat2 is geo2:lat * constant:degtorad.
  local rlng1 is geo1:lng * constant:degtorad.
  local rlng2 is geo2:lng * constant:degtorad.
  
  // Haversine formula
  local dlat is rlat2 - rlat1.
  // Handle anti-meridian case by ensuring longitude difference is ≤ 180°
  local dlng is abs(geo2:lng - geo1:lng).
  if dlng > 180 {
    set dlng to 360 - dlng.
  }
  set dlng to dlng * constant:degtorad.
  
  local a is sin(dlat/2)^2 + 
           cos(rlat1) * cos(rlat2) * sin(dlng/2)^2.
  local c is 2 * arctan2(sqrt(a), sqrt(1-a)).
  
  // Use the body radius at datum
  return geo1:body:radius * c.
}