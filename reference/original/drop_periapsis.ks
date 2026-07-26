@lazyGlobal off.

// KNOWN WRONG, deliberately. This is the worked-wrong version of the placement
// plan_doi.ks does correctly; the faults below are marked where they sit. All
// three were measured over the Mun from a parking orbit at e = 0.012, and all
// three push periapsis east of where it was asked for. See notes/doi-planner.md.
//
//   1. delta_t converts a longitude gap to a wait at the ORBITAL rate. A
//      body-fixed longitude closes at the SYNODIC rate, the body having turned
//      under the ship meanwhile: ~6.7 degrees of error per orbit waited at the
//      Mun, ~20 at Minmus. kepler.ks's time_to_longitude chases the real
//      ground track instead of assuming its rate.
//   2. Nothing here accounts for the body turning during the coast from the
//      burn down to periapsis — 3.2 degrees, 11 km, at the Mun.
//   3. The burn is placed at the target's antipode and sized as though the
//      burn point were an apsis. Off an apsis the burn leaves radial speed in
//      play, and periapsis lands ~15 degrees (53 km) past the antipode and
//      ~1 km low.

// Drop the periapsis to a given height over a given longitude.
parameter new_periapsis to 5000.
parameter target_lng to 0.

// Load orbital functions.
runpath("orbital").

// FIXME: Should probably check that the periapsis at the desired point is higher than new_periapsis...

// Obviously the burn should be 180º away from the desired periapsis longitude.
// FAULT 3 (placement) and FAULT 2: the antipode is where periapsis lands only
// from an apsis, and this longitude carries no term for the rotation the body
// turns through during the coast.
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
// FAULT 1: ship:orbit:period is the rate the ship closes an INERTIAL angle.
// The gap being closed is a body-fixed one, and it closes at the synodic rate.
local delta_t to ship:orbit:period * delta_lng / 360.

// Great. Find the altitude at that point, then use the vis-viva equation to compute the instantaneous delta-V to get into the orbit we want.
local alt_burn to altitude_at(ship:orbit, time + delta_t).
local v0 to orbital_speed_v1(ship:orbit, alt_burn).
// FAULT 3 (sizing): this prices the speed on an ellipse that keeps the current
// apoapsis, which the orbit after the burn does only if the burn is at
// apoapsis. Anywhere else the delivered periapsis comes in low.
local v1 to orbital_speed_v1(ship:orbit, alt_burn, ship:orbit:apoapsis, new_periapsis).

// Create the maneuver node at that time.
local nd to node(time:seconds + delta_t, 0, 0, v1 - v0).
add nd.