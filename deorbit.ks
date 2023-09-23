set deorbit_lng to 175.
set final_periapsis to 0.

run "0://common".

clearscreen.
print "=== DEORBIT ===".

set period to ship:orbit:period.
set lng0 to ship:geoposition:lng.
set d_lng to deorbit_lng - lng0.
if lng0 > deorbit_lng {
  set d_lng to d_lng + 360.
}
// account for the rotation of Kerbin during the time to node
// NB: this doesn't seem to be enough, really
set time_to_node to (period + period / 60) * (d_lng / 360).
set nd_time to time:seconds + time_to_node.

set s to orbitat(ship, nd_time).
set v0 to orbital_speed(ship, s:altitude, s:apoapsis, s:periapsis).
set v1 to orbital_speed(ship, s:altitude, s:periapsis, s:final_periapsis).

print "v0: " + round(v0 + 3) + " v1: " + round(v1 + 3) + " dV: " + round(v1-v0 + 3).

set deorbit_node to node(nd_time, 0, 0, v1 - v0).
add deorbit_node.
