cd(scriptPath():parent).
runOncePath("../core/kepler.ks").

// ship:altitude is measured from the reference spheroid, i.e. "sea level".
// alt:radar is measured from the terrain, i.e. "ground level".

local lock surface_elevation to ship:altitude - alt:radar.
warpTo(time:seconds + free_fall_time(surface_elevation) - 60).


until false {
    clearScreen.

    print "Current altitude: " + round(ship:altitude) + " m ASL.".
    print "Surface elevation: " + round(surface_elevation) + " m ASL.".
    //print "Current velocity: " + round(v_:radial:mag) + " m/s radial, " + round(v_:tangent:mag) + " m/s tangential.".
    print "Vertical speed " + round(ship:verticalspeed) + " m/s.".

    print "Free fall time to surface: " + round(free_fall_time(surface_elevation), 1) + " s.".
    print "Free fall velocity at surface: " + round(orbital_speed(surface_elevation)) + " m/s.".
    wait 0.1.
}