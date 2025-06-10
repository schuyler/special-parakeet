// Functions related to Keplerian orbits
// https://ksp-kos.github.io/KOS/structures/orbits/orbit.html


function find_zero { // of a function using the Newton-Raphson method
    parameter f.
    parameter df. // Derivative of f.
    parameter x0.
    parameter epsilon is 0.0001.
    parameter max_iterations is 100.
    
    local x is x0.
    local deltaX is 1.0.
    local iteration is 0.

    // Newton-Raphson finds roots of f(x) = 0 using: x_{n+1} = x_n - f(x_n)/f'(x_n)
    // It stops when the change in x is less than epsilon or after max_iterations.

    until abs(deltaX) < epsilon or iteration > max_iterations {
        set iteration to iteration + 1.
        set df_x to df(x).
        if df_x = 0 { // Avoid division by zero
            print "Derivative is zero at x = " + round(x, 6) + ". Stopping iteration.".
            return x.
        }
        set deltaX to f(x) / df_x.
        set x to x - deltaX.
        //print "Iteration " + iteration + ": x = " + round(x, 6) + ", f(x) = " + round(f(x), 6) + ", df(x) = " + round(df(x), 6) + ".".
    }
    
    return x.
}

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
    parameter epsilon is 0.0001.

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

function wrap_longitude {
    parameter lng.
    // Shift the longitude to the range [0, 360) by 180, then shift it back to the range [-180, 180) by subtracting whole multiples of 360.
    return lng - 360 * floor((lng + 180) / 360).
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

function time_to_longitude {
    // Convert time to longitude for a given orbit.
    parameter target_longitude.
    parameter orbit_ is ship:orbit.
    parameter dt is 0.1.

    local f is {
        parameter t.
        // Function to find the difference between the body's longitude at time t and the target longitude.
        // This is the function we want to find the root of.
        local x is body_longitude(t, orbit_) - target_longitude.
        if x > 360 {
            // If the longitude is greater than 360, wrap it around.
            set x to x - 360.
        } else if x < -360 {
            // If the longitude is less than -360, wrap it around.
            set x to x + 360.
        }
        return x.
    }.

    local df is {
        parameter t.
        // Derivative of the longitude over time using finite difference.
        // This is a numerical approximation of the derivative.
        return (body_longitude(t + dt, orbit_) - body_longitude(t, orbit_)) / dt.
    }.

    // Starting estimate = (target_longitude - current_longitude) / (orbital_rate - body_rotation_rate)
    local estimate is time + (target_longitude - body_longitude(time, orbit_)) / (360 / orbit_:period - 360 / orbit_:body:rotationPeriod).

    // Use the Newton-Raphson method to find the time when the body's longitude matches the target longitude.
    return find_zero(f, df, estimate, 0.001).
}
