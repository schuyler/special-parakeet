@lazyglobal off.

// KSC is at 74.5ºW and we need to overshoot.
parameter initial_lng is -65.
parameter target_periapsis is -20000.

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
  parameter ob is ship:orbit.
  parameter t is timestamp().
  parameter target is target_periapsis.
  local alt_ is altitude_at(ob, t).

  // print "alt_ = " + round(altitude) + " at " + t.
  // what if this really is the apoapsis?!?!
  return orbital_speed(ob, alt_, alt_, target).
}

function deorbit_trajectory {
  parameter ob0 is ship:orbit.
  parameter t is timestamp().
  parameter target is target_periapsis.

  local ob to orbit_at(ob0, t).
  local s to deorbit_speed(ob, t, target).
  // print "orbital speed: " + orbital_speed(ob).
  // print "deorbit speed from " + round(altitude_at(ob, t)) + "m = " + round(s, 1) + " m/s.".

  print "deorbit_trajectory ob:velocity = " + ob:velocity:orbit.
  local vel to ob:velocity:orbit:normalized * s.
  // print "trajectory velocity: " + vel + " = " + vel:mag.
  // print round(orbital_speed(ob) - vel:mag, 1) + " m/s deorbit burn in " + round(t:seconds - time:seconds) + "s".

  // create an orbit that's heading in the same direction but at a different speed
  // https://github.com/KSP-KOS/KOS/issues/2862#issue-794415466
  local ob1 to createorbit(
    -V(ob:body:position:x, ob:body:position:z, ob:body:position:y),
    V(vel:x, vel:z, vel:y),
    ob:body,
    t:seconds).


  print "traj apo = " + round(ob1:apoapsis) + " peri = " + round(ob1:periapsis).
  return ob1.
}

function landing_site {
  parameter ob.

  // what is the timestamp when that trajectory crosses sea level?
  local t1 is time_to_altitude(ob, 0, false).
  // print "time to sea level: " + t1.

  // what is the geoposition at that time?
  local site is geoposition_at(ob, time:seconds + t1).
  return site.
}

function find_deorbit_time {
  parameter ob0.         // the original orbit
  parameter target_alt.  // the desired periapsis
  parameter target_lng.  // the desired landing longitude
  parameter t.           // deorbit start time

  // determine the deorbit trajectory at time 0 that results in the desired
  // (low) periapsis.
  local ob to deorbit_trajectory(ob0, timestamp(t), target_alt).
 
  // Figure out where we crash land if no atmosphere and no braking
  local site to landing_site(ob).

  // minimize the difference between that and our target longitude.
  local d_lng is abs(site:lng - target_lng).

  // print "apo = " + round(ob:apoapsis) + " peri = " + round(ob:periapsis) + " "
  print round((timestamp(t)-timestamp()):seconds, 1) + "s " +  round(site:lng, 3) + "ºE".
  //wait 1.
  return d_lng.
}


function program_deorbit_burn {
  local ob to ship:orbit.

  // Set up the evaluation function
  local eval_timestamp to find_deorbit_time@:bind(ship:orbit, target_periapsis, initial_lng).

  // Minmize the evaluation function to find the best time
  local start_t to time:seconds.
  local end_t to time:seconds + ship:orbit:period - 1.
  //local deorbit_t to minimize(eval_timestamp, start_t, end_t, 1).

  local ground_lng to -1000.
  local deorbit_t to start_t.

  until deorbit_t >= end_t or abs(ground_lng - initial_lng) <= 1 {
    set deorbit_t to deorbit_t + 10.
    local trajectory to deorbit_trajectory(ob, timestamp(deorbit_t), target_periapsis).
    set ground_lng to landing_site(trajectory):lng.
    print round(deorbit_t, 1) + " > " + round(ground_lng, 3) + " @ " + round(trajectory:periapsis) + "m.".
  }
  set deorbit_t to timestamp(deorbit_t).

  print "Deorbit time is " + deorbit_t.

  // Take the best trajectory and set up the node
  local trajectory to deorbit_trajectory(ob, deorbit_t, target_periapsis).
  local v1 to deorbit_speed(ob, deorbit_t, target_periapsis).
  local v0 to orbital_speed(orbit_at(ob, deorbit_t)).

  // print v1 + " > " + v0.

  local deorbit_node to node(deorbit_t, 0, 0, v1 - v0).
  add deorbit_node.

  print "Deorbiting in " + round(deorbit_t:seconds - time:seconds) + "s at " + round(deorbit_node:prograde) + "m/s dV.".
  print "Estimated periapsis will be " + round(trajectory:periapsis) + "m.".

  local site to landing_site(trajectory).
  print "Expected landing site will be near " + round(site:lat, 3) + "ºN, " + round(site:lng, 3) + "ªE.".
}

function test_deorbit {
  until false {
    clearscreen.
    local ob to deorbit_trajectory(). 
    print ob:apoapsis.
    print ob:periapsis.
    print landing_site(ob).
    wait 1.
  } 
}

program_deorbit_burn().
// test_deorbit().
