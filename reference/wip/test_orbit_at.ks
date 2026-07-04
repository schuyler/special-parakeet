
cd(scriptPath():parent).
runpath("../core/kepler.ks").

parameter dt is 1000.

local t to time + dt.
local p1 to orbit_at(t):position.
local p2 to positionAt(ship, t) - body:position.

// This number is consistently correct
print "distance from p1 to p2: " +  round((p1 - p2):mag) + " m.".

local g1 to geoposition_at(t).
local g2 to body:geopositionof(positionAt(ship, t)).

// These two values differ as dt increases
print "geoposition_at: " + g1.
print "positionAt: " + g2.

// This value is off by ~9m for every 10s of dt in a circular, equatorial orbit around the Mun. 
// The Mun rotates at ~9.18 m/s at the equator, so the observed difference is consistent with the
// Mun's rotation speed and the notion that body:geopositionof() is not accounting for it.

print "distance from g1 to g2: " +  round((g1:position - g2:position):mag) + " m.".

// When dt is 0, these two values are very close.
print "kepler.ks terrain height: " + round(g1:terrainheight) + " m.".
print "built-in terrain height: " + round(g2:terrainheight) + " m.".
