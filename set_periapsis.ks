runpath("orbital").

parameter new_periapsis is orbit:apoapsis.

local apo to orbit:apoapsis.

local v0 to orbital_speed(orbit, apo).
local v1 to orbital_speed(orbit, apo, apo, new_periapsis).

local t to time_to_altitude(orbit, apo).
// print "Apoapsis will be in " + t:minute + ":" + t:second.

local nd to node(t, 0, 0, v1 - v0).
add nd.
