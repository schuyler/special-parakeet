
runpath("kepler.ks").

function test_mean_anomaly_at_epoch {
    print "Built-in mean anomaly: " + orbit:meanAnomalyAtEpoch.
    print "Calculated mean anomaly: " + mean_anomaly(time(orbit:epoch)).
}

function test_mean_anomaly {
    // Test the mean_anomaly function with a specific time and orbit.
    local orbit_ is ship:orbit.
    local t is time.

    // Calculate the mean anomaly at time t for the given orbit.
    local M is mean_anomaly(t, orbit_).
    
    // Print the result.
    print "Mean anomaly: " + round(M, 6) + "º.".
}

function test_eccentric_anomaly {
    // Test the eccentric_anomaly function with a specific time and orbit.
    local orbit_ is ship:orbit.
    local t is time.

    // Calculate the eccentric anomaly at time t for the given orbit.
    local E is eccentric_anomaly(t, orbit_).
    
    // Print the result.
    print "Eccentric anomaly: " + round(E, 6) + "º".
}

function test_true_anomaly {
    // Test the true_anomaly function with a specific time and orbit.
    local orbit_ is ship:orbit.
    local t is time.

    // Calculate the true anomaly at time t for the given orbit.
    local ν is true_anomaly(t, orbit_).
    
    // Print the result.
    print "== True Anomaly Test ==".
    print "True anomaly: " + round(ν, 6) + "º.".
    print "kOS built-in: " + ship:orbit:trueanomaly.
    print "Difference: " + round(ν - ship:orbit:trueanomaly, 6) + "º.".
    print "".
}

function test_orbit_at {
    parameter dt is 0.
    local t is time.

    local orbit_t is orbit_at(t + dt, ship:orbit).
    local pos_t is positionAt(ship, time + dt) - body:position. // orbit_t:position is SOI-RAW
    local vel_t is velocityAt(ship, time + dt).

    // print "body:position " + body:position + " (" + round(body:position:mag, 1) + ")".
    print "== Orbital Position Prediction ==".
    print "positionat(t) " + pos_t + " (" + round(pos_t:mag, 1) + ")".
    print "orbit_t:position " + orbit_t:position + " (" + round(orbit_t:position:mag, 1) + ")".
    print "delta: " + round((pos_t - orbit_t:position):mag, 3) + " m".

    print "== Orbital Velocity Prediction ==".
    print "velocityat(t) " + vel_t:orbit + " (" + round(vel_t:orbit:mag, 1) + ")".
    print "orbit_t:velocity " + orbit_t:velocity + " (" + round(orbit_t:velocity:mag, 1) + ")".
    print "delta: " + round((vel_t:orbit - orbit_t:velocity):mag, 3) + " m/s".

}

function test_body_longitude {
    // Calculate the longitude of the body at the current time.
    local lon is body_longitude().
    
    // Print the result.
    print "== Body Longitude Test ==".
    print "Estimated longitude: " + round(lon, 6) + "º.".
    print "Given longitude: " + ship:longitude + "º.".
    print "Difference: " + round(lon - ship:longitude, 6) + "º.".
    print "".
}

function test_geoposition_at {
    local t is time.
    print "== Body Geoposition Test ==".
    print "geoposition_at(): " + geoposition_at(t).
    print "body:geopositionof(ship): " + body:geopositionof(ship:position).
}

function test_time_to_longitude {
    // Test the time_to_longitude function with a specific target longitude.
    local orbit_ is ship:orbit.

    print "== Time to Longitude Test ==".

    // Determine the current longitude of periapsis
    local periapsis_longitude is body_longitude(time + orbit_:eta:periapsis, orbit_).
    local t is time_to_longitude(periapsis_longitude, orbit_).
    
    // Print the result.
    print "Current time: " + time + " seconds.".
    print "Orbital period: " + orbit_:period + " seconds.".
    print "Current longitude: " + ship:longitude + "º.".
    print "Time to reach longitude " + periapsis_longitude + " is: " + t + " seconds.".
    print "Time to reach periapsis is: " + (time + orbit_:eta:periapsis) + " seconds.".
    print "Difference: " + (t - (time + orbit_:eta:periapsis)) + " seconds.".
    print "".
}

clearscreen.
test_mean_anomaly_at_epoch().
test_mean_anomaly().
test_eccentric_anomaly().
test_true_anomaly().
test_orbit_at(600).
test_body_longitude().
test_geoposition_at().
//test_time_to_longitude().
//print "Longitude: " + body_longitude(time) + ", longitude + 1 orbit: "  + body_longitude(time+synodic_period()).