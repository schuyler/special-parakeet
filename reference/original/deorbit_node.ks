run "common".

// Adjusts a maneuver node until the resulting orbit's periapsis is at the surface
// Returns true if successful, false if it fails to converge
function zero_periapsis {
    parameter
        mnv_node is nextnode,  // The node to adjust (defaults to next node)
        min_dv is -250,        // Minimum DV to try
        max_dv is -50,           // Maximum DV to try
        tolerance is 5.        // How close to zero we need to get (in dv)
    
    // Create objective function for this node
    local pe_func is {
        parameter dv.
        set mnv_node:prograde to dv.
        return abs(mnv_node:orbit:periapsis).
    }.
    
    // Find DV that minimizes absolute periapsis
    local best_dv is minimize(pe_func, min_dv, max_dv, tolerance).
    
    // Apply the best DV and check result
    set mnv_node:prograde to best_dv.
    return abs(mnv_node:orbit:periapsis).
}

// Computes the surface distance between a deorbit burn's predicted landing
// point and the target landing site
function compute_landing_error {
    parameter 
        burn_start,      // When to place the maneuver node
        target_coord,    // Target landing site (latlng)
        min_dv,
        max_dv.
        
    local not_found to 1e9.

    // Return a very large error for out-of-bounds times
    // if burn_start < min_time or burn_start > max_time {
    //     return not_found.
    // }

    // Create a test node at the specified time
    local test_node is node(burn_start, 0, 0, 0).
    add test_node.
    
    // Optimize it for zero periapsis
    zero_periapsis(test_node, min_dv, max_dv).
    
    // Wait a bit for Trajectories to update
    wait 0.05.
    
    local distance is 0.
    
    // If we have an impact prediction, compute distance to target
    if addons:tr:hasimpact {
        local impact_pos is addons:tr:impactpos.
        set distance to (target_coord:position - impact_pos:position):mag.
    } else {
        // No impact predicted, return a very large number
        set distance to not_found.
    }
    
    // Clean up the test node
    remove test_node.
    print burn_start + " " + distance.
    return distance.
}

// Finds the optimal deorbit burn time to land at a target location
function compute_landing {
    parameter target_coord.   // Target landing site (latlng)
    
    // Create an error function that minimize() can use
    // It takes a time value and returns the landing error
    local start_time is time:seconds.

    // Search over next 2 full orbits
    local min_time is start_time + 60.  // Start 1 minute from now to give some setup time
    local max_time is start_time + orbit:period - 61.
    
    // Create the final node at the optimal time
    local nd is node(min_time, 0, 0, 0).
    add nd.

    // Optimize it for zero periapsis
    zero_periapsis(nd).
    remove nd.
    set start_dv to nd:prograde.

    local error_func is {
        parameter t.
        return compute_landing_error(t, target_coord, start_dv*0.75, start_dv*1.25).
    }.

    // Find the time that minimizes landing error
    print "min_time: " + min_time.
    print "max_time: " + max_time.
    local best_time is minimize(error_func, min_time, max_time, 5).
    print "best_time: " + best_time.
    
    // Create the final node at the optimal time
    local final_node is node(best_time, 0, 0, 0).
    add final_node.

    // Optimize it for zero periapsis
    zero_periapsis(final_node).
    return final_node.
}

// Returns the angle in degrees between a ship-relative position vector
// and a geographic coordinate on the current body's surface
function angle_to_surface_position {
    parameter 
        source_pos,    // Vector relative to ship
        surface_pos.    // Geographic coordinates (latlng object)
        
    // Get the surface position vector relative to body center
    local p1 is source_pos - body:position.
    local p2 is surface_pos:position - body:position.
      
    // Return angle between vectors in degrees
    return vang(p1, p2).
}

//add node(time:seconds+300,0,0,0).
// adjust_for_impact().

local ksc is latlng(0, -74.5).
compute_landing(ksc).