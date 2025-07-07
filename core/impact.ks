run "kepler".

// Spacecraft position is r(t) = r(t₀) + v(t₀)·t + ½g·t²

function velocity_at_datum {
    parameter ob.
    local up_ is ob:position - ob:body:position.
    local g_ is body:mu / up_:mag ^ 2.
    local v_falling is sqrt(2 * g_ * up_:mag) * up_:normalized.
    return ob:velocity:surface - v_falling.
}

function time_to_datum {
    parameter ob.
    local alt_ is (ob:position - ob:body:position):mag - ob:body:radius.
    local v0 is ob:velocity:surface.
    local v_impact is velocity_at_datum(ob).
    return alt_ / ((v_impact:mag - v0:mag) / 2).
}

function predict_impact {
    parameter orbit_ is orbit.
    parameter epsilon is 0.1.
    parameter t_start is 0.
    parameter t_end is orbit_:eta:periapsis.
    
    local t0 is time.
    local i to 0.

    local height_above_terrain is {
        parameter t.
        local future to orbit_at(t0 + t, orbit_).
        local geo to geoposition_at(t0 + t, orbit_, future:position).
        local alt_ to future:position:mag - orbit_:body:radius.
        set i to i + 1.
        return alt_ - geo:terrainheight.
    }.

    local dt_impact is bisect(height_above_terrain, t_start, t_end, epsilon).
    local t_impact is t0 + dt_impact.
    local impact is orbit_at(t_impact, orbit_).
    return lexicon(
        "position", impact:position,
        "velocity", impact:velocity,
        "geo", geoposition_at(t_impact, orbit_, impact:position),
        "time", t_impact,
        "iterations", i
    ).
}

// function compute_deorbit_burn {
//     parameter target.
//     parameter orbit_.
//     parameter epsilon.

//     // Deorbit 90º before the target
//     local lng_burn to wrap_longitude(target:lng - 90).
//     local t_burn to time_to_longitude(lng_burn).
//     // Get the orbital state at that time
//     local initial to orbit_at(t_burn, orbit_).

//     local dv_min is -500.
//     local dv_max is 0.

//     local impact_distance to {
//         parameter dv.
//         local t_start to t_burn - time.
//         local impact to predict_impact(orbit_, 1, t_start, t_start + orbit_:period / 4).
//         return (target:position - impact:geo:position):mag.
//     }.

//     local dv to bisect(f, 
// }


function test_predict_impact {
    clearscreen.
    local oldPos is 0.
    //log "current_time,impact_time,altitude,current_lng,impact_lng,estimate_delta,iterations" to "impact_log.txt".
    local t_start is 0.
    local t_end is orbit:eta:periapsis.
    until false {
        local t_ is time.
        local impact is predict_impact(ship:orbit, 0.1, t_start, t_end).
        set t_start to (impact:time - time):seconds - 5.
        set t_end to t_start + 10.

        print "Impact lng: " + round(impact:geo:lng, 3) at (1, 19).
        //print "Velocity: " + round(impact:velocity:mag, 2) at (1, 20).
        print "ETA: " + round((impact:time - time):seconds, 3) at (1, 21).
        //print "Altitude: " + round(impact:altitude, 1) + "   " at (1, 22).
        //print "Distance to impact: " + round(alt:radar - impact:altitude, 1) at (1, 23).
        print "Iterations: " + impact:iterations + "   " at (1, 24).
        if oldPos <> 0 {
            print "Estimate delta: " + round((oldPos - impact:position):mag, 3) + "   " at (1, 25).
            set current_lng to body:geopositionof(ship:orbit:position):lng.
            //log round(t_:seconds, 3) + "," + round(impact:time:seconds, 3) + "," + round(alt:radar, 3) + "," + round(current_lng, 3) + "," + round(impact:geo:lng, 3) + "," + round((oldPos - impact:position):mag, 3) + "," + impact:iterations to "impact_log.txt".
        }
        set oldPos to impact:position.
        wait 0.
    }
}

test_predict_impact().