@lazyglobal off.

run "time_to_closest_approach".
run "predict_datum_impact".
run "orbit_at_t".
run "minimize".
run "orbit_after_maneuver".
run "surface_distance".

// Calculate a retrograde deorbit burn to target a specific landing site
function calculate_deorbit_burn {
  parameter target_geo.      // Geocoordinates of desired landing site
  parameter lead_angle is 45.  // Angle before closest approach to burn
  parameter search_orbits is 1.  // How many orbits ahead to search
  
  // Find time of closest approach to target
  local lead_time is lead_angle / 360 * ship:orbit:period.
  local approach is time_to_closest_approach(
    ship:orbit,
    target_geo:position,
    ship:orbit:period * search_orbits
  ).

    // Calculate how much time represents the desired angle
  if time > approach:time - lead_time {
    set approach to time_to_closest_approach(
      orbit_at_t(ship:orbit, approach:time + 1),
      target_geo:position,
      ship:orbit:period * search_orbits
    ).
  }
  local burn_time to approach:time - lead_time.

  // Function to evaluate quality of a retrograde burn
  function evaluate_burn {
    parameter dv_mag.
    
    // Get velocity at burn time
    local burn_orbit_ is orbit_at_t(ship:orbit, burn_time).
    local burn_vel_ is burn_orbit_:velocity:orbit.
    local burn_vector_ is -burn_vel_:normalized * dv_mag.
    
    // Create new orbit after applying burn
    local new_orbit is orbit_after_maneuver(
      ship:orbit,
      burn_vector_,
      burn_time
    ).

    // Find where this orbit intersects the datum
    local impact is predict_datum_impact(new_orbit, target_geo:terrainheight).
    if impact:time < 0 {
      // print "dv: " + dv_mag + " no impact".
      return 100000000.  // Arbitrary large number if no impact
    }
    
    // Return distance to target
    local dist to surface_distance(impact:geo, target_geo).
    // print "dv: " + dv_mag + " dist: " + dist.
    return dist.
  }
  
  // Calculate velocity needed at datum using vis-viva
  // local body_ is ship:orbit:body.
  // local burn_orbit is orbit_at_t(ship:orbit, burn_time).
  // local r_burn is (burn_orbit:position - body_:position):mag.
  // local r_datum is body_:radius.
  // local sma is ship:orbit:semimajoraxis.
  // local v_burn is sqrt(body_:mu * (2/r_burn - 1/sma)).
  // local v_datum is sqrt(body_:mu * (2/r_datum - 1/sma)).
  
  // Velocity difference needed is the current orbital velocity minus datum velocity
  // local max_dv is v_burn - v_datum.
  local orbit_vel is velocityat(ship, burn_time):orbit:mag.
  local best_dv is minimize(evaluate_burn@, 0, orbit_vel, 1).
  
  // Calculate final burn vector
  local burn_orbit to orbit_at_t(ship:orbit, burn_time).
  local burn_vel is burn_orbit:velocity:orbit.
  local burn_vector is -burn_vel:normalized * best_dv.
  
  return lexicon(
    "vector", burn_vector,
    "time", burn_time,
    "distance", evaluate_burn(best_dv)
  ).
}

// Test function
function test_deorbit {
  clearscreen.
  local tgt is body:geopositionlatlng(0,0).
  local result is calculate_deorbit_burn(tgt).
  
  print "Burn vector: " + round(result:vector:mag, 1) + "m/s".
  print "Burn time: " + round((result:time - time):seconds, 1) + "s".
  print "Expected miss distance: " + round(result:distance, 1) + "m".
}

test_deorbit.