parameter burn_lng to -180.
parameter target_periapsis to -5000.

runpath("orbital").

print "=== SIMPLE DEORBIT PLAN ===".

function deorbit_speed {
  parameter ob is ship:orbit.
  parameter t is timestamp().
  parameter target is target_periapsis.
  local alt_ is altitude_at(ob, t).

  // print "alt_ = " + round(altitude) + " at " + t.
  // what if this really is the apoapsis?!?!
  return orbital_speed_v1(ob, alt_, ob:periapsis, target).
}

local burn_t to timestamp() + time_to_meridian(ship:orbit, burn_lng).

print "Starting deorbit burn at " + burn_lng + "º in " + round(burn_t:seconds) + "s.".

local s0 to orbital_speed_v1(ship:orbit, altitude_at(ship:orbit, burn_t)).
local s1 to deorbit_speed(ship:orbit, burn_t, target_periapsis).
local nd to node(burn_t, 0, 0, s1 - s0).
add(nd).
