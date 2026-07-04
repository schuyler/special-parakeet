@lazyglobal off.

run "minimize".
run "orbit_at_t".
run "geoposition_at_t".

// Find time until closest approach between an orbit and a fixed position
function time_to_closest_approach {
  parameter orbit_.           // Orbit to analyze
  parameter target_geo.  // Raw position vector to approach
  parameter t_start is time.
  parameter t_end is t_start + orbit_:period.  // How far ahead to search
  parameter epsilon is 1.     // Desired precision in seconds
  
  // Function to calculate distance at a given time
  function distance_at_time {
    parameter t.
    local orbit_t is orbit_at_t(orbit_, t).
    local target_t is target_geo. //geoposition_at_t(target_geo, t).
    return (orbit_t:position - target_t:position):mag.
  }
  
  // Find time of minimum distance using ternary search
  local best_time is minimize(
    distance_at_time@,
    t_start:seconds,
    t_end:seconds,
    epsilon
  ).
  set best_time to timestamp(best_time).
  local closest to orbit_at_t(orbit_, best_time).
  local closest_geo to body:geopositionof(closest:position).

  return lexicon(
    "time", best_time,
    "eta", best_time - time,
    "closest", closest_geo,
    "distance", distance_at_time(best_time:seconds)
  ).
}

// Test function
function test_closest_approach {
  // Get current ship orbit and some future target position
  local test_orbit is ship:orbit.
  local test_position is body:geopositionof(orbitat(ship, time:seconds + 600):position).
  
  // Find closest approach
  local result is time_to_closest_approach(
    test_orbit,
    test_position
  ).

  print "Closest approach to orbit location + 600s:". 
  print "Time to closest approach: " + round(result:eta:seconds, 1) + "s".
  print "Distance at approach: " + round(result:distance, 1) + "m".

  print "Closest approach to (0,0):".
  set test_position to body:geopositionlatlng(0,0).
  set result to time_to_closest_approach(
    test_orbit,
    test_position
  ).
  print "Time to closest approach: " + round(result:eta:seconds, 1) + "s".
  print "Distance at approach: " + round(result:distance, 1) + "m".
}

// test_closest_approach.