clearscreen.
print "=== AEROBRAKING ===".

sas off. 
set start_apoapsis to orbit:apoapsis.
set start_periapsis to orbit:periapsis.

print "Periapsis is currently " + round(orbit:periapsis) + "m.".

// Position the spacecraft perpendicular to the prograde vector.
lock steering to prograde * r(0, 90, -90).
if altitude > 70000 {
  print "Warping to atmosphere.".
  set warpmode to "rails".
  set warp to 3.
}

wait until altitude < 71000.
set warp to 0.

wait until altitude < 70000.
print "Aerobraking start.".

set start_time to time:seconds.
set warpmode to "physics".
set warp to 2.
wait until altitude > 70000 or altitude < 20000.

set elapsed to time:seconds - start_time.
print "Aerobraking took " + round(elapsed) + "s.".
print "Apoapsis was lowered by " + round(start_apoapsis - orbit:apoapsis) + "m.".
print "Periapsis was lowered by " + round(start_periapsis - orbit:periapsis) + "m.".

set warp to 0.
unlock steering.
