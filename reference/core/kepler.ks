// Functions related to Keplerian orbits
// https://ksp-kos.github.io/KOS/structures/orbits/orbit.html

cd(scriptPath():parent).
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

function _mean_to_eccentric_anomaly {
    parameter M.
    parameter orbit_ is ship:orbit.
    // Calculate the eccentric anomaly E from the mean anomaly M using Kepler's equation.
    // E = M + ecc * sin(E) * (180/constant:pi)
    // This is an iterative solution, as Kepler's equation cannot be solved algebraically.
    local ecc is orbit_:eccentricity.

    // Function to find the root of f(E) = E - ecc * sin(E) - M
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
    return find_zero(f, df, M, 0.01).
}

function _eccentric_to_true_anomaly {
    parameter E.
    parameter orbit_ is ship:orbit.
    // Calculate the true anomaly ν for a given eccentric anomaly E on the current orbit.
    // True anomaly is given by: ν = 2 * arctan2(sqrt(1 + ecc) * sin(E / 2), sqrt(1 - ecc) * cos(E / 2))
    local ecc is orbit_:eccentricity.
    return 2 * arctan2(sqrt(1 + ecc) * sin(E / 2), sqrt(1 - ecc) * cos(E / 2)).
}

function eccentric_anomaly {
    parameter t is time.
    parameter orbit_ is ship:orbit.

    // Calculate the eccentric anomaly E from the mean anomaly M using Kepler's equation.
    local M is mean_anomaly(t, orbit_).
    return _mean_to_eccentric_anomaly(M, orbit_).
}

function true_anomaly {
    parameter t is time.
    parameter orbit_ is ship:orbit.
    // Calculate the true anomaly ν at time t for the given orbit.
    local E is eccentric_anomaly(t, orbit_).
    return _eccentric_to_true_anomaly(E, orbit_).
}

// Vis-viva equation relates orbital height and speed. 
function orbital_speed {
    parameter alt_ is alt:radar.
    parameter orbit_ is ship:orbit.

    local r_ to orbit_:body:radius + alt_.
    return sqrt(orbit_:body:mu * ((2 / r_) - (1 / orbit_:semimajoraxis))).
}

// Calculate the radial and tangential components of a given velocity vector in the orbit's SOI-RAW coordinates.
function orbital_velocity {
    parameter vel is ship:velocity:orbit.
    parameter orbit_ is ship:orbit.
    local v_up to (orbit_:position - orbit_:body:position):normalized.
    local v_radial to vdot(vel, v_up) * v_up.
    local v_tangent to vel - v_radial.
    return lexicon(
        "radial", v_radial,
        "tangent", v_tangent
    ).
}

// Calculate the time to fall to a given altitude. This uses a kinematic equation for free fall,
// and is much cheaper to calculate than using orbital mechanics as in `time_to_altitude`.
// But they should give similar results for small altitudes.
function free_fall_time {
    parameter target_alt is 0.
    parameter orbit_ is ship:orbit.

    // Kinematic equation for free fall: d = v0 * t - 0.5 * g * t^2
    // Solving for t gives us: t = (sqrt(v0^2 + 2 * g * d) - v0) / g

    local current_alt to orbit_:body:altitudeof(orbit_:position).
    local d to current_alt - target_alt.

    // Get the vertical component of the velocity vector in the orbit's SOI-RAW coordinates.
    local v_up to (orbit_:position - orbit_:body:position):normalized.
    local v0 to vdot(orbit_:velocity:surface, v_up).

    // v0 is negative when descending, so we treat g as having a negative sign also.
    local g_ is -body:mu / ((body:radius + (target_alt + current_alt) / 2) ^ 2).

    // print "v0: " + round(v0, 1) + " m/s.".
    // print "g: " + round(g_, 1) + " m/s²".
    // print "d: " + round(d, 1) + " m.".

    return (-v0 - sqrt(v0^2 - 2 * g_ * d)) / g_.
}

// Estimate the time to a given altitude using the relation between orbital radius and eccentric anomaly.
function time_to_altitude {
    parameter alt_ is alt:radar.
    parameter orbit_ is ship:orbit.

    // Capture the true anomaly at the current time.
    local t0 is time.
    local nu_0 is orbit_:trueanomaly.

    local r_target to orbit_:body:radius + alt_.
    local a to orbit_:semimajoraxis.
    local ecc to orbit_:eccentricity.

    // According to Wikipedia, r = a(1 - e cos E), which means that E = acos((a - r_) / (e * a)).
    // https://en.wikipedia.org/wiki/Eccentric_anomaly#Radius_and_eccentric_anomaly
    local elliptic_ratio to (a - r_target) / (ecc * a).

    // Ratio clamping needed because floating point rounding at the boundary leads to arccos returning NaN
    set elliptic_ratio to min(max(elliptic_ratio, -1), 1).
    local E_r to arccos(elliptic_ratio). // return º

    // arccos() returns [0, 180º], so we have to check if we are currently in the first or second half of the orbit.
    // This makes sense because a spacecraft in an elliptical orbit will pass through the same altitude twice,
    // once ascending and once descending. The edge cases are when the spacecraft is at periapsis or apoapsis,
    // or when the orbit is circular.
    //
    // https://www.reddit.com/r/Kos/comments/4tm0wq/two_common_mistakes_people_make_when_calculating/

    if nu_0 > 180 {
        // If the current true anomaly is greater than 180, we are in the second half of the orbit.
        // We need to adjust E_r to be in the range [180, 360).
        set E_r to 360 - E_r.
    }

    // Convert E to true anomaly
    local nu_r to _eccentric_to_true_anomaly(E_r, orbit_).

    // It's possible that we've passed the target altitude already, so we need to check if the target true anomaly.
    // is less than the current one.
    if nu_r < orbit_:trueanomaly {
        // If it is, we need to add 360 to get the correct time to the target altitude.
        set nu_r to nu_r + 360.
    }
    
    // Now we can be certain that the difference in true anomaly is positive.
    local d_nu to nu_r - nu_0.
    
    // Calculate the time to the target altitude based on the mean motion.
    local dt_target to d_nu / 360 * orbit_:period.

    // Return the time to the target altitude.
    return t0 + dt_target.
}


// == Body position functions ==

// The axis of the orbit is the vector that is perpendicular to the orbital plane.
// This is also the angular momentum vector of the orbit, which is constant in KSP.
function orbital_axis {
    parameter orbit_.
    local pos to orbit_:position - orbit_:body:position.  // Position of the ship in SOI-RAW coordinates.
    local vel to orbit_:velocity:orbit.                   // Velocity of the ship in SOI-RAW coordinates.
    return vcrs(pos, vel):normalized.
}

// Compute the state vector of the ship at a given time t in the orbit's SOI-RAW coordinates.
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

function synodic_period {
    parameter orbit_ is ship:orbit.
    // Calculate the synodic period of the orbit in seconds: T_synodic = 1 / (1/T_orbital - 1/T_rotation)
    // This is the time it takes for the craft to return to the same longitude relative to the body.
    local t_orbital is orbit_:period.
    local t_rotation is orbit_:body:rotationPeriod.
    return 1 / (1 / t_orbital - 1 / t_rotation).
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
