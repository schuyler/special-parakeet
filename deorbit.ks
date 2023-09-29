// KSC is at 74.5ºW and we need to overshoot.
parameter initial_lng is -65.
parameter target_periapsis is -4000.

runpath("common").
runpath("orbital").

clearscreen.
print "=== PLOT DEORBIT NODE ===".

// from the current orbit, if you deorbit from deorbit_lng, what dv burn do you need for periapsis to be target_periapsis at target_lng
//
// 1. given deorbit_lng, target_lng, target_periapsis, dv
// 2. what is utc at deorbit_lng on current orbit
// 3. compute new orbit starting at utc using target_periapsis
// 4. what is time_to_altitude(0) on that orbit
// 5. what is the position on that orbit at that time
// 6. what is the lng of the geoposition of that orbit at that time
// 7. what is the difference between the landing lng and the target_lng

function deorbit_speed {
  parameter ob0 is ship:orbit.
  parameter t is timestamp().
  parameter target is target_periapsis.
  local alt_ is altitude_at(ob).

  // what if the apoapsis was the same but the periapsis was lower?
  return orbital_speed(ob, alt_, ob:periapsis, target).
}

function deorbit_trajectory {
  parameter ob0 is ship:orbit.
  parameter t is timestamp().
  parameter target is target_periapsis.

  local ob to orbit_at(ob0, t).
  local v to deorbit_speed(ob, t, target).

  // create an orbit that's heading in the same direction but at a different speed
  return createorbit(ob:position, ob:velocity:orbit:normalize * v, ob:body, t).
}

function landing_site {
  parameter ob.

  // what is the timestamp when that trajectory crosses sea level?
  local t1 is time_to_altitude(ob, 0).

  // what is the geoposition at that time?
  local site is geoposition_at(ob, t1).
  return site.
}

function find_deorbit_time {
  parameter ob0.         // the original orbit
  parameter target_alt.  // the desired periapsis
  parameter target_lng.  // the desired landing longitude
  parameter t.           // deorbit start time

  // determine the deorbit trajectory at time 0 that results in the desired
  // (low) periapsis.
  local ob to deorbit_trajectory(ob0, t, target_alt).

  // Figure out where we crash land if no atmosphere and no braking
  local site to landing_site(ob).

  // minimize the difference between that and our target longitude.
  local d_lng is abs(site:lng - target_lng).
  return d_lng.
}


// Replace existing nodes
until not hasnode {
  remove nextnode.
}

// Set up the evaluation function
local eval_timestamp to find_deorbit_time@bind(ship:orbit, target_periapsis, target_lng).

// Minmize the evaluation function to find the best time
local deorbit_t to minimize(eval_timestamp, time:seconds, time:seconds + ship:orbit:period - 1).

// Take the best trajectory and set up the node
local trajectory to deorbit_trajectory(ship:orbit, deorbit_t, target_periapsis).
local v1 to orbital_speed(trajectory, deorbit_t).
local v0 to orbital_speed(ship:orbit, deorbit_t).

local deorbit_node to node(deorbit_t, 0, 0, v1 - v0).
add deorbit_node.

print "Deorbiting in " + round(deorbit_t - time:seconds) + "s at " + round(deorbit_node:prograde) + "m/s dV.".

local site to landing_site(trajectory).
print "Expected landing site will be near " + round(site:lat, 3) + "ºN, " + round(site:lng, 3) + "ªE.".
