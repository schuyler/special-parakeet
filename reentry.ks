clearscreen.
print "== REENTRY ==".

if periapsis > 70000 {
  print "Re-entry is not expected on this orbit.".
  exit.
}

run common.

lock steering to heading(90, 20).
set warp to 3.
print "Warping to atmospheric re-entry.".

wait until altitude < 71000.
set warp to 0.
print "Orienting to prograde.".

wait until altitude < 70000.
set warp to 3.

print "Waiting for aerodynamic control.".
until airspeed < 1200 {
   print "Landing in " + round(landing_time(), 1) + "s.".
   wait 5.
}

print "Flight controls unlocked.".
set warp to 0.
unlock steering.
sas on.

