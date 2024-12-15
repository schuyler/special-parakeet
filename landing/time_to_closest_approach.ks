@lazyglobal off.

run "minimize".
run "orbit_at_t".

// Find time until closest approach between an orbit and a fixed position
function time_to_closest_approach {
  parameter orbit_.           // Orbit to analyze
  parameter target_position.  // Raw position vector to approach
  parameter search_time is orbit_:period.  // How far ahead to search
  parameter epsilon is 1.     // Desired precision in seconds
  
  // Function to calculate distance at a given time
  function distance_at_time {
    parameter t.
    local future_orbit is orbit_at_t(orbit_, t).
    return (future_orbit:position - target_position):mag.
  }
  
  // Search window from now until search_time
  local t_start is time:seconds.
  local t_end is t_start + search_time.
  
  // Find time of minimum distance using ternary search
  local best_time is minimize(
    distance_at_time@,
    t_start,
    t_end,
    epsilon
  ).
  
  return lexicon(
    "ut", best_time,
    "eta", best_time - time:seconds,
    "distance", distance_at_time(best_time)
  ).
}

// Test function
function test_closest_approach {
  // Get current ship orbit and some future target position
  local test_orbit is ship:orbit.
  local test_position is positionat(ship, time:seconds + 600).
  
  // Find closest approach
  local result is time_to_closest_approach(
    test_orbit,
    test_position
  ).

  print "Closest approach to orbit location + 600s:". 
  print "Time to closest approach: " + round(result:eta, 1) + "s".
  print "Distance at approach: " + round(result:distance, 1) + "m".

  print "Closest approach to (0,0):".
  set test_position to body:geopositionlatlng(0,0):position.
  set result to time_to_closest_approach(
    test_orbit,
    test_position
  ).
  print "Time to closest approach: " + round(result:eta, 1) + "s".
  print "Distance at approach: " + round(result:distance, 1) + "m".
}

//test_closest_approach.