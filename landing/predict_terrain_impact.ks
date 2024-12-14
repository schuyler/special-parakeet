@lazyglobal off.

run "predict_datum_impact".
run "minimize".
run "surface_distance".
run "flight_path_angle".

// Predict impact point considering terrain height
function predict_terrain_impact {
  // Get datum intersection as starting point
  local sphere_impact is predict_datum_impact(0, ship:orbit).
  
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

function go_nuts {
  local prediction to predict_terrain_impact().
  print prediction:geo + " at " + prediction:time + "s".
  until false {
      print "Ground error: "+surface_distance(prediction:geo, ship:geoposition) + 
            " Surface velocity: " + ship:velocity:surface +
            " Descent angle: " + flight_path_angle(prediction:geo:terrainheight):angle.
      wait alt:radar/1000.
  }
}

go_nuts.