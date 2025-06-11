// Functions related to Keplerian orbits
// https://ksp-kos.github.io/KOS/structures/orbits/orbit.html

// == Helper functions ==

function wrap_degrees {
    // Normalize the angular value.
    // Double modulo operation ensures that we always get a positive angle in the range [0, 360),
    // even if the input is negative.
    parameter deg.
    return mod(mod(deg, 360) + 360, 360).
}

// === Optimization functions ===

function find_zero { // of a function using the Newton-Raphson method
    parameter f.
    parameter df. // Derivative of f.
    parameter x0.
    parameter epsilon is 0.0001.
    parameter max_iterations is 100.
    parameter debug is false.

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
        if debug {
            print "Iteration " + iteration + ": x = " + x + ", f(x) = " + f(x) + ", df(x) = " + df(x) + ".".
        }
    }
    
    //print "Iterations: " + iteration + ", final x = " + x + ", f(x) = " + f(x) +".".
    return x.
}

function bisect {
    parameter f.
    parameter start.
    parameter end.
    parameter epsilon is 0.0001.
    parameter max_iterations is 100.
    parameter debug is false.

    local a is start.
    local b is end.
    local c is (a + b) / 2.
    local iteration is 0.
    // Bisection method finds roots of f(x) = 0 by repeatedly halving the interval [a, b] where f(a) and f(b) have opposite signs.
    if f(a) * f(b) > 0 {
        print "Bisection bracketing failed:".
        print "  f(" + round(a,2) + ") = " + round(f(a),4).
        print "  f(" + round(b,2) + ") = " + round(f(b),4).
        print "  Both have same sign - no root in interval".
        return -1.
    }

    local f_a is f(a).
    local f_c is f(c).
    until abs(b - a) < epsilon or iteration > max_iterations {
        set iteration to iteration + 1.
        if f_c = 0 {
            return c. // Found exact root
        }
        if f_a * f_c < 0 {
            set b to c. // Root is in [a, c]
        } else {
            set a to c. // Root is in [c, b]
            set f_a to f_c. // Update f(a) to the new value
        }
        set c to (a + b) / 2.
        set f_c to f(c).
        if debug {
            // Print the current state of the bisection method.
            print "Iteration " + iteration + ": a = " + a + ", b = " + b + ", c = " + c + ", f(c) = " + f_c + ".".
        }
    }
    return c.
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
    local nu_0 is true_anomaly(time, orbit_).

    // If the estimated true anomaly is less than the current one, we need to look one orbit ahead.
    if nu_estimate < nu_0 {
        set nu_estimate to nu_estimate + 360.
    }

    // The time estimate is the difference in true anomaly, divided by the mean motion.
    local t_estimate is
        (nu_estimate - true_anomaly(time, orbit_)) * orbit_:period / 360.

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
