// Functions related to Keplerian orbits
// https://ksp-kos.github.io/KOS/structures/orbits/orbit.html

cd(scriptPath:parent).
runoncepath("optimize.ks").

// == Helper functions ==

function wrap_degrees {
    // Normalize the angular value.
    // Double modulo operation ensures that we always get a positive angle in the range [0, 360),
    // even if the input is negative.
    parameter deg.
    return mod(mod(deg, 360) + 360, 360).
}

// == Keplerian orbit functions ==

function mean_anomaly {
    parameter t is time.
    parameter orbit_ is ship:orbit.
    // Calculate the mean anomaly M at time t for the given orbit.
    // This could also be calculated using mean motion, which is: sqrt(mu / a^3) * (t - epoch).
    local ma is orbit_:meanAnomalyAtEpoch + 360 * (t:seconds - orbit_:epoch) / orbit:period.
    return mod(ma, 360).
}

function eccentric_anomaly {
    parameter t is time.
    parameter orbit_ is ship:orbit.
    parameter epsilon is 0.001.

    // Calculate the eccentric anomaly E from the mean anomaly M using Kepler's equation.
    local M is mean_anomaly(t, orbit_).
    local ecc is orbit_:eccentricity.

    // Solve Kepler's equation: M = E - ecc * sin(E)
    // By looking for the root of f(E) = E - ecc * sin(E) - M
    // However, the product `ecc * sin(E)` is in radians, even though E is in degrees.
    // Since E and M are in degrees, we need to convert that term from radians to degrees, so that the units match.
    local f is {
        parameter E.
        return E - ecc * sin(E) * (180/constant:pi) - M.
    }.

    // The derivative of f(E) is df(E) = 1 - ecc * cos(E)
    local df is {
        parameter E.
        return 1 - ecc * cos(E).
    }.

    // Initial guess for E is M, which is a good approximation.
    return find_zero(f, df, M, epsilon).
}

function true_anomaly {
    parameter t is time.
    parameter orbit_ is ship:orbit.
    // Calculate the true anomaly ν at time t for the given orbit.
    local E is eccentric_anomaly(t, orbit_).
    local ecc is orbit_:eccentricity.
    // True anomaly is given by: ν = 2 * arctan2(sqrt(1 + ecc) * sin(E / 2), sqrt(1 - ecc) * cos(E / 2))
    return 2 * arctan2(sqrt(1 + ecc) * sin(E / 2), sqrt(1 - ecc) * cos(E / 2)).
}

function synodic_period {
    parameter orbit_ is ship:orbit.
    // Calculate the synodic period of the orbit in seconds: T_synodic = 1 / (1/T_orbital - 1/T_rotation)
    // This is the time it takes for the craft to return to the same longitude relative to the body.
    local t_orbital is orbit_:period.
    local t_rotation is orbit_:body:rotationPeriod.
    return 1 / (1 / t_orbital - 1 / t_rotation).
}

// Vis-viva equation relates orbital height and speed. 
function orbital_speed {
    parameter alt_ is alt:radar.
    parameter orbit_ is ship:orbit.

    local r_ to orbit_:body:radius + alt_.
    return sqrt(orbit_:body:mu * ((2 / r_) - (1 / orbit_:semimajoraxis))).
}

// == Body position functions ==

function orbital_axis {
    parameter orbit_.
    local pos to orbit_:position - orbit_:body:position.  // Position of the ship in SOI-RAW coordinates.
    local vel to orbit_:velocity:orbit.                   // Velocity of the ship in SOI-RAW coordinates.
    return vcrs(pos, vel):normalized.
}

function orbit_at {
    parameter t is time.
    parameter orbit_ is ship:orbit.

    local nu_0 is orbit_:trueanomaly.
    local pos_0 to orbit_:position - orbit_:body:position.  // Position of the ship in SOI-RAW coordinates.

    // True anomaly at time t.
    local nu_t to true_anomaly(t, orbit_).

    // Radius of the orbit at time t.
    local e to orbit_:eccentricity.
    local a to orbit_:semimajoraxis.
    local r_t to a * (1 - e^2) / (1 + e * cos(nu_t)).

    // Find the angle between the prime meridian and the longitude of the true anomaly.
    // local lng_offset to orbit_:longitudeofascendingnode + orbit_:argumentofperiapsis + nu_0.
    local lng_offset to nu_0.

    // Create a rotation matrix around the orbital axis by the angle between the future true anomaly and the prime meridian.
    local axis to orbital_axis(orbit_).
    local orbital_rotation to angleaxis(nu_t - lng_offset, axis).

    // Rotate the initial position vector pos_0 around the orbital axis by the angle of the true anomaly difference.
    // This gives us the position of the ship in SOI-RAW coordinates at time t
    local pos_angle_t to orbital_rotation * pos_0:normalized.

    // Scale the direction vector by the radius of the orbit at time t.
    local pos_t to pos_angle_t * r_t.

    // Magnitude of velocity at time t give by vis-viva equation
    local v_t_mag to sqrt(orbit_:body:mu * (2 / r_t - 1 / a)).

    // Flight path angle at time t (explain)
    local flight_path_angle to arctan2(e * sin(nu_t), (1 + e * cos(nu_t))).

    // Velocity direction versus flight path angle (explain)
    local vel_angle_t to 90 - flight_path_angle.

    // Rotate the future positiion vector by the corresponding angle and scale to magnitude
    local vel_t to angleaxis(vel_angle_t, axis) * pos_t:normalized * v_t_mag.

    return lexicon(
        "position", pos_t,
        "velocity", vel_t,
        "time", t
    ).
}

// == Body sphere functions ==

function wrap_longitude {
    parameter lng.
    // Shift the longitude to the range [0, 360) by 180, then shift it back to the range [-180, 180) by subtracting whole multiples of 360.
    return lng - 360 * floor((lng + 180) / 360).
}

function body_rotation {
    parameter t is time.
    parameter orbit_ is ship:orbit.
    // Calculate the rotation angle of the body at time t.
    //
    // Per https://ksp-kos.github.io/KOS/structures/celestial_bodies/body.html#attribute:BODY:ROTATIONANGLE
    //
    // "The rotation angle is the number of degrees between the Solar Prime Vector and the current position of the body’s prime meridian
    //   (body longitude of zero). The value is in constant motion, and once per body’s rotation period (“sidereal day”), its :rotationangle
    //   will wrap around through a full 360 degrees."
    return orbit_:body:rotationangle + (360 / orbit_:body:rotationPeriod) * (t - orbit_:epoch):seconds.
}

function body_longitude {
    // Longitude = (Ω + ω + ν) - ω_body × (t - t₀)
    parameter t is time.
    parameter orbit_ is ship:orbit.
    local lng is (
        orbit_:longitudeOfAscendingNode +
        orbit_:argumentOfPeriapsis + 
        true_anomaly(t, orbit_) - 
        body_rotation(t, orbit_)
    ).
    return wrap_longitude(lng).
}

function geoposition_at {
    parameter t is time.
    parameter orbit_ is orbit.
    parameter pos is 0.

    // Project the orbit to time t
    if pos = 0 {
        local future is orbit_at(t, orbit_).
        set pos to future:position.
    }

    // Latitude is 90º minus the angle between the Y-axis and the SOI-RAW position vector 
    local lat is 90 - vang(v(0, 1, 0), pos:normalized).

    // Longitude is LAN + AoP + True Anomaly - body rotation
    local lng is body_longitude(t, orbit_).

    return orbit_:body:geoPositionLatLng(lat, lng).
}

function time_to_longitude {
    // Convert time to longitude for a given orbit.
    parameter target_longitude.
    parameter orbit_ is ship:orbit.
    // Minimum accuracy for time estimate, default is 1 second.
    parameter epsilon is 2.
    // Time window for the search, default is 5% of the orbital period.
    parameter t_window is orbit_:period / 20. 

    // Estimated true anomaly at the target longitude is ν = target_longitude - (Ω + ω - ω_body).
    // We're using the current time to get a rough estimate of the body rotation.
    local nu_estimate is wrap_degrees(
        target_longitude -
        orbit_:longitudeOfAscendingNode -
        orbit_:argumentOfPeriapsis +
        body_rotation(time, orbit_)
    ).

    // Time estimate is based on the difference between the estimated true anomaly and the current true anomaly,
    local nu_0 is orbit_:trueanomaly.

    // If the estimated true anomaly is less than the current one, we need to look one orbit ahead.
    if nu_estimate < nu_0 {
        set nu_estimate to nu_estimate + 360.
    }

    // The time estimate is the difference in true anomaly, divided by the mean motion.
    local t_estimate is
        (nu_estimate - nu_0) * orbit_:period / 360.

    // Estimate the time window around the target longitude.
    local t0 is time.
    local t_start is max(t_estimate - t_window, 0).
    local t_end is min(t_estimate + t_window, orbit_:period).

    // Function to find the difference between the body's longitude at time t and the target longitude.
    // This is the function we want to find the root of.
    local longitude_diff is {
        parameter t.
        local diff to target_longitude - body_longitude(t0 + t, orbit_).
        if diff > 180 { set diff to diff - 360. }
        else if diff < -180 { set diff to diff + 360. }
        return diff.
    }.

    return t0 + bisect(longitude_diff, t_start, t_end, epsilon).
}
