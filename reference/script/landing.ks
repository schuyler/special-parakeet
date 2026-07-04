@lazyGlobal off.

cd(scriptPath():parent).
runOncePath("../core/kepler.ks").
runOncePath("../core/rocket.ks").

parameter landing_speed_ is 5.
parameter throttle_target_ is 1.
parameter warp_margin_ is 30.

function perform_landing {
    parameter landing_speed.
    parameter throttle_target.
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

    // Set up some basic parameters for the landing.
    local lock g_ to body:mu / (body:distance ^ 2).
    local lock twr to ship:availablethrust / ship:mass.

    // Initialize variables for the landing process.
    local state to "Initial Free Fall".

    // terrain_height estimates the elevation ASL of the terrain at the landing site.
    // Initially, we have to assume that it's sea level.
    local lock terrain_height to 0.

    // fall_time estimates the time it takes to fall to the surface.
    local lock fall_time to free_fall_time(terrain_height).

    // predicted_impact is the predicted impact point on the surface.
    local lock predicted_impact to geoposition_at(time + fall_time).
    
    // cached_impact is used to store the last known impact point.
    // It is updated during the landing process to avoid recalculating the impact point too often
    local cached_impact to predicted_impact.
    local last_impact_update to time.

    // Now we can start to update the terrain height based on the predicted impact point.
    local lock terrain_height to cached_impact:terrainheight.

    // landing_burn_duration is the duration of the burn needed to slow down to the landing speed.
    local lock landing_burn_duration to burn_duration(orbital_speed(terrain_height) - landing_speed) / 2.

    // burn_start is the difference between how long the craft has to fall and how long the burn will take.
    // throttle_target is the target throttle level for the landing burn. Using a value less than 1 provides a margin of safety.
    local lock burn_start to fall_time - landing_burn_duration * (1 / throttle_target).

    // throttle_needed is the throttle level needed to achieve the landing speed.
    // If landing_burn_duration and fall_time are equal, then the throttle_needed is 1.
    // If fall_time is greater than landing_burn_duration, then the throttle_needed is less than 1.
    // If fall_time is less than landing_burn_duration, then the craft is going to crash.
    local lock throttle_needed to 1. //landing_burn_duration / max(fall_time, landing_burn_duration).

    // Warp until warp_margin seconds before the burn starts.
    if burn_start > warp_margin {
        set warp to 3.
    }

    // When the burn is ready to start, turn off time warp and drop the gear.
    when burn_start < warp_margin then {
        set warp to 0.
        gear on.
    }

    // Update the predicted impact point every ~5 seconds. Stop when horizontal speed is low enough,
    // because then we switch over to radar.
    when time > last_impact_update + max(log10(alt:radar), 2) then {
        set cached_impact to predicted_impact.
        set last_impact_update to time.
        return groundspeed > landing_speed.
    }

    // Start the braking burn when landing_burn_duration is less than the time to impact.
    when burn_start <= 1 and verticalspeed < 0 then {
        set state to "Braking Burn".
        lock throttle to throttle_needed.

        // When the horizontal speed is low enough, we can drop into free fall until it's time to start the landing burn.
        when groundspeed < landing_speed then {
            set state to "Final Descent".
            lock throttle to 0.

            // Start using the radar altitude to estimate the terrain height.
            lock fall_time to free_fall_time(terrain_height).
            lock terrain_height to ship:altitude - alt:radar.

            // At this point, the orbit is degenerate, so use a kinematic estimate for the impact speed.
            lock landing_burn_duration to burn_duration(ship:velocity:surface:mag + g_ * fall_time).

            // The landing burn will start when the fall time is equal to the landing burn duration \.
            when burn_start <= 1 and verticalspeed < 0 then {
                set state to "Landing Burn".
                // Ensure that we don't throttle below the minimum needed to achieve the landing speed.
                lock throttle to 1.

                when airspeed < landing_speed then {
                    lock throttle to 0.99 / twr.
                }
            }
        }
    }

    // When we make contact, kill the throttle and lock the steering to the ship's up vector.
    when alt:radar <= landing_altitude then {
        lock steering to ship:up * r(0,0,0).
        lock throttle to 0.
    }

    until alt:radar <= 2 {
        print "State          : " + state + "            " at (1,22).
        print "Suicide burn   : T-" + round(burn_start, 1) + " s " at (1,23).
        print "Burn duration  : " + round(landing_burn_duration, 1) + " s " at (1,24).
        print "Throttle needed: " + round(throttle_needed, 2) + " " at (1,25).
        //print "Vspd           : " + round(verticalspeed, 1) + " m/s " at (1,26).
        //print "Speed          : " + round(airspeed, 1) + " m/s " at (1,27).
        //print "Radar          : " + round(alt:radar) + " m " at (1,28).

        print "Fall time      : " + round(fall_time, 1) + " s " at (1,30).
        //print "Location       : " + round(body:geopositionof(ship:position):lng, 4) + "º " at (1,31).
        // if cached_impact <> "" {
        //     print "Cached impact  : " + round(cached_impact:lng, 4) + "º " at (1,32).
        //     print "Terrain height : " + round(cached_impact:terrainheight, 1) + " m " at (1,33).
        // }
        print "Last update    : " + round((time - last_impact_update):seconds, 1) + " s ago " at (1,33).
        print "Est. terrain h : " + round(terrain_height, 1) + " m " at (1,34).
        // print "Free fall vel  : " + round(orbital_speed(terrain_height), 1) + " m/s " at (1,35).
        wait 0.25.
    }

    // Wait for the ship to settle on the ground, then unlock the controls and turn SAS back on.
    wait 5.
    unlock throttle.
    unlock steering.
    sas on.
}

clearscreen.
print "== POWERED LANDING ==".
perform_landing(landing_speed_, throttle_target_, warp_margin_).
