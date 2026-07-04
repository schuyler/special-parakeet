@lazyglobal off.

run "predict_datum_impact".
run "minimize".

// Predict impact point considering terrain height
function predict_terrain_impact {
  // Get datum intersection as starting point
  local sphere_impact is predict_datum_impact().
  
  // Get impact velocity for time bound calculations
  local impact_time is sphere_impact:time.
  local impact_velocity to velocityat(ship, impact_time):orbit:mag.
  
  // Set search bounds based on ±15km terrain deviation
  local max_time_delta is 15000 / impact_velocity.
  local t_start is impact_time:seconds - max_time_delta.
  local t_end is impact_time:seconds + max_time_delta.
  
  function altitude_difference {
    parameter t.
    local pos is positionat(ship, t).
    local geo_ is body:geopositionof(pos).
    local terrain_radius is body:radius + geo_:terrainheight.
    local orbit_radius is (pos - body:position):mag.
    local diff is orbit_radius - terrain_radius.
    return abs(diff).
  }
  
  // Find actual impact time using minimize
  local terrain_impact_time is minimize(altitude_difference@, t_start, t_end, 0.1).
  local fall_time is terrain_impact_time - time.
  local impact_pos is positionat(ship, terrain_impact_time).
  local geo is body:geopositionof(impact_pos).
  local impact_lat is geo:lat.
  local impact_lng is geo:lng.
  
  // Account for body rotation during fall
  local rotation_deg is fall_time:seconds * (360 / body:rotationperiod).
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
    "eta", fall_time,
    "time", terrain_impact_time
  ).
}

run "surface_distance".
//run "flight_path_angle".

function go_nuts {
  local prediction to predict_terrain_impact().
  print prediction:geo + " at " + prediction:eta + "s".
  until false {
      print "Ground error: "+surface_distance(prediction:geo, ship:geoposition) + 
            " Surface velocity: " + ship:velocity:surface. 
            //+ " Descent angle: " + flight_path_angle(ship:orbit, prediction:geo:terrainheight):angle.
      wait alt:radar/1000.
  }
}

//go_nuts.