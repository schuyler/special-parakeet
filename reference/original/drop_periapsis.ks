@lazyGlobal off.

// Drop the periapsis to a given height over a given longitude.
parameter new_periapsis to 5000.
parameter target_lng to 0.

// Load orbital functions.
runpath("orbital").

// FIXME: Should probably check that the periapsis at the desired point is higher than new_periapsis...

// Obviously the burn should be 180º away from the desired periapsis longitude.
local burn_lng to mod(target_lng + 180, 360).
if (burn_lng > 180) {
    set burn_lng to burn_lng - 360.
}

// How many degrees is that away from where the ship currently is?
local current_lng to body:geopositionof(ship:position):lng.
local delta_lng to burn_lng - current_lng.
if (delta_lng < 0) {
    set delta_lng to delta_lng + 360.
}

// How long will it take to get there?
local delta_t to ship:orbit:period * delta_lng / 360.

// Great. Find the altitude at that point, then use the vis-viva equation to compute the instantaneous delta-V to get into the orbit we want.
local alt_burn to altitude_at(ship:orbit, time + delta_t).
local v0 to orbital_speed_v1(ship:orbit, alt_burn).
local v1 to orbital_speed_v1(ship:orbit, alt_burn, ship:orbit:apoapsis, new_periapsis).

// Create the maneuver node at that time.
local nd to node(time:seconds + delta_t, 0, 0, v1 - v0).
add nd.