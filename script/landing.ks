@lazyGlobal off.

cd(scriptPath():parent).
runOncePath("../core/kepler.ks").
runOncePath("../core/rocket.ks").

parameter landing_speed_ is 5.
parameter burn_margin_ is 0.9.
parameter warp_margin_ is 30.

function perform_landing {
    parameter landing_speed.
    parameter burn_margin.
    parameter warp_margin.

    // Parameterize this.
    local landing_altitude is 5.

    // Ensure that the ship has active engines.
    if ship:availablethrust = 0 {
        print "No engines available.".
        return.
    }

    // Remove any existing maneuver nodes to avoid conflicts.
    until not hasnode {
        remove nextnode.
        wait 1.
    }

    // Disable SAS to allow automatic control.
    sas off.

    // Wait until the vertical speed is negative, indicating free fall.
    wait until verticalspeed < 0.

    // Lock the steering to the surface retrograde vector.
    lock steering to ship:srfretrograde * r(0,0,0).

    // Initialize variables for the landing process.
    local state to "Initial Free Fall".
    local cached_impact to "".
    local lock terrain_height to ship:altitude - alt:radar.

    local lock fall_time to free_fall_time(terrain_height).
    local lock predicted_impact to geoposition_at(time + fall_time).
    local lock landing_burn_duration to burn_duration(free_fall_velocity(terrain_height)).
    local lock burn_start to fall_time - (landing_burn_duration * (1 / burn_margin)).
    local lock throttle_needed to min(1.0, landing_burn_duration / max(fall_time, 0.1)).

    print "TERRAIN HEIGHT: " + round(terrain_height, 1) + " m ".
    if burn_start > warp_margin {
        set warp to 3.
    }

    when burn_start < warp_margin then {
        set warp to 0.
        gear on.
    }

    when burn_start <= 0 and verticalspeed < 0 then {
        set state to "Braking Burn".
        set cached_impact to predicted_impact.
        lock terrain_height to cached_impact:terrainheight.
        lock throttle to throttle_needed.
        return alt:radar > landing_altitude.
    }

    when airspeed < landing_speed then {
        set state to "Free Fall".
        lock terrain_height to ship:altitude - alt:radar.
        lock throttle to 0.
        return alt:radar > landing_altitude.
    }

    when alt:radar <= landing_altitude then {
        lock steering to ship:up * r(0,0,0).
        lock throttle to 0.
    }

    until alt:radar <= 2 {
        print "State          : " + state + "            " at (1,22).
        print "Suicide burn   : T-" + round(burn_start, 1) + " s " at (1,23).
        print "Burn duration  : " + round(landing_burn_duration, 1) + " s " at (1,24).
        print "Throttle needed: " + round(throttle_needed, 2) + " " at (1,25).
        print "Vspd           : " + round(verticalspeed, 1) + " m/s " at (1,26).
        print "Speed          : " + round(airspeed, 1) + " m/s " at (1,27).
        print "Radar          : " + round(alt:radar) + " m " at (1,28).

        print "Fall time      : " + round(fall_time, 1) + " s " at (1,30).
        print "Location       : " + round(body:geopositionof(ship:position):lng, 4) + "º " at (1,31).
        if cached_impact <> "" {
            print "Cached impact  : " + round(cached_impact:lng, 4) + "º " at (1,32).
            print "Terrain height : " + round(cached_impact:terrainheight, 1) + " m " at (1,33).
        }
        print "Est. terrain h : " + round(terrain_height, 1) + " m " at (1,34).
        print "Free fall vel  : " + round(free_fall_velocity(terrain_height), 1) + " m/s " at (1,35).
        wait 0.25.
    }

    lock throttle to 0.
    wait 5.

    unlock throttle.
    unlock steering.
    sas on.
}

clearscreen.
print "== POWERED LANDING ==".
perform_landing(landing_speed_, burn_margin_, warp_margin_).
