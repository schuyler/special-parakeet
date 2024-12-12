run "common".

// Calculate true anomaly where orbit reaches a given height above datum
// Returns -1 if orbit never reaches that height
function true_anomaly_at_height {
  parameter orbit_.  // Orbit to analyze
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

// Calculate flight path angle (γ) at a given height in an orbit
// Returns angle between velocity vector and local horizontal plane
// Negative angles indicate descent below horizontal
// Positive angles indicate ascent above horizontal 
function flight_path_angle {
  parameter orbit_.  // Orbit to analyze
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
    "theta", impact_theta
  ).
}

// Calculate predicted impact point for current vessel using orbital elements
// height_above_datum: meters above the reference sphere to check for intersection
function predict_impact {
  parameter height_above_datum is 0.
  
  local orbit_ is ship:orbit.
  local body_ is ship:body.
  
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
  local impact_pos is positionat(ship, time:seconds + time_to_impact).
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
    "lat", impact_lat,
    "lng", final_lng,
    "time", time_to_impact
  ).
}

// Predict impact point considering terrain height
function predict_terrain_impact {
  // Get datum intersection as starting point
  local sphere_impact is predict_impact(0).
  
  // If no intersection with datum, no terrain impact possible
  if sphere_impact:time < 0 {
    return lexicon(
      "geo", 0,
      "time", -1
    ).
  }
  
  // Get impact velocity for time bound calculations
  local impact_time is time:seconds + sphere_impact:time.
  local impact_velocity to velocityat(ship, impact_time):orbit:mag.
  
  // Set search bounds based on ±15km terrain deviation
  local max_time_delta is 15000 / impact_velocity.
  local t_start is time:seconds + sphere_impact:time - max_time_delta.
  local t_end is time:seconds + sphere_impact:time + max_time_delta.
  
  print "Impact velocity: " + round(impact_velocity,3) + 
        " max_time_delta: " + round(max_time_delta,3) + 
        " t_start: " + round(t_start,3) + 
        " t_end: " + round(t_end,3).
  
  function altitude_difference {
    parameter t.
    local pos is positionat(ship, t).
    local geo is body:geopositionof(pos).
    local terrain_radius is body:radius + geo:terrainheight.
    local orbit_radius is (pos - body:position):mag.
    local diff is orbit_radius - terrain_radius.
    
    if diff < 5000 {
      local orb_v is velocityat(ship,t):orbit.
      local srf_v is velocityat(ship,t):surface.
      print "t=" + round(t,1) + 
            " diff=" + round(diff,1) + 
            " th=" + round(geo:terrainheight,1) +
            " orad=" + round(orbit_radius,1) +
            " lat=" + round(geo:lat,4) + 
            " lng=" + round(geo:lng,4) +
            " ov=" + round(orb_v:mag,1) +
            " sv=" + round(srf_v:mag,1).
    }
    return abs(diff).
  }
  
  // Find actual impact time using minimize
  local terrain_impact_time is minimize(altitude_difference@, t_start, t_end, 0.01).
  local fall_time is terrain_impact_time - time:seconds.
  local impact_pos is positionat(ship, terrain_impact_time).
  local geo is body:geopositionof(impact_pos).
  local impact_lat is geo:lat.
  local impact_lng is geo:lng.
  
  // Account for body rotation during fall
  local rotation_deg is fall_time * (360 / body:rotationperiod).
  local final_lng is impact_lng - rotation_deg.
  
  // Normalize longitude to -180 to 180
  if final_lng > 180 {
    set final_lng to final_lng - 360.
  } else if final_lng < -180 {
    set final_lng to final_lng + 360.
  }
  
  // Create and return geoposition with rotated coordinates
  return lexicon(
    "geo", body:geopositionlatlng(impact_lat, final_lng),
    "time", fall_time
  ).
}

// Predict landing coordinates if we perform a suicide burn
function predict_landing_coordinates {
  // First get terrain impact data if we don't burn
  local impact is predict_terrain_impact().
  if impact:time < 0 { 
    return lexicon(
      "lat", 0,
      "lng", 0,
      "time", -1
    ).
  }
  
  // Calculate gravity at impact altitude
  local impact_pos is positionat(ship, time:seconds + impact:time).
  local g is body:mu / ((impact_pos - body:position):mag ^ 2).
  
  // Get impact velocity and required burn duration
  local impact_time is time:seconds + impact:time.
  local v_impact is velocityat(ship, impact_time):orbit:mag.
  
  // Required delta-v is impact velocity plus gravity losses
  local burn_time is burn_duration(v_impact).  // Initial estimate
  local delta_v is v_impact + (g * burn_time).
  // Recalculate burn time with gravity losses included
  set burn_time to burn_duration(delta_v).
  
  // Calculate distance covered during burn
  // s = v0*t - (1/2)a*t^2
  local acceleration is ship:availablethrust / ship:mass.
  local burn_distance is v_impact * burn_time - (0.5 * acceleration * burn_time^2).
  
  // Start burn this far before impact point
  local start_time is impact_time - burn_time.
  local start_pos is positionat(ship, start_time).
  
  // Project landing point along surface velocity vector
  local start_vel is velocityat(ship, start_time):surface.
  local landing_pos is start_pos + burn_distance * start_vel:normalized.
  local geo is body:geopositionof(landing_pos).
  
  return lexicon(
    "geo", geo,
    "lat", geo:lat,
    "lng", geo:lng,
    "time", start_time - time:seconds,
    "terrain_height", geo:terrainheight,
    "burn_time", burn_time,
    "delta_v", delta_v
  ).
}

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
  local dlng is rlng2 - rlng1.
  
  local a is sin(dlat/2)^2 + 
           cos(rlat1) * cos(rlat2) * sin(dlng/2)^2.
  local c is 2 * arctan2(sqrt(a), sqrt(1-a)).
  
  // Use the body radius at datum
  return geo1:body:radius * c.
}


local prediction to predict_terrain_impact().
print prediction:geo + " at " + prediction:time + "s".
until false {
    print "Ground error: "+surface_distance(prediction:geo, ship:geoposition) + " Surface velocity: " + ship:velocity:surface + " Descent angle: " + descent_angle(ship:orbit):angle.
    wait alt:radar/1000.
}