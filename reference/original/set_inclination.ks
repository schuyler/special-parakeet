@lazyglobal off.

// Returns angle between current position and ascending node in degrees
// Returns angle in range [0, 360)
function angle_to_node_line {
    // Angle from AN = true anomaly + argument of periapsis
    local true_anom is ship:orbit:trueanomaly.
    local arg_pe is ship:orbit:argumentofperiapsis.
    return mod(true_anom + arg_pe, 360).
}

// Returns time in seconds until next ascending node crossing
function time_to_an {
    local angle is angle_to_node_line().
    local period is ship:orbit:period.
    return (360 - angle) * period / 360.
}

// Returns time in seconds until next descending node crossing
function time_to_dn {
    local angle is angle_to_node_line().
    local period is ship:orbit:period.
    local angle_past_dn is mod(angle - 180, 360).
    return (360 - angle_past_dn) * period / 360.
}

// Returns required delta-v for a plane change maneuver
// Parameters:
//   target_inc - target inclination in degrees (default 0)
//   node_time - absolute time of burn in seconds (default now)
// Returns: delta-v magnitude in m/s (always positive)
function plane_change_dv {
    parameter 
        target_inc is 0,
        node_time is time:seconds.
    
    local inc_diff is target_inc - ship:orbit:inclination.
    local vel is velocityat(ship, node_time):orbit:mag.
    return 2 * vel * sin(abs(inc_diff/2)).
}

// Creates and returns a maneuver node for changing orbital inclination
// Parameter: target_inc - target inclination in degrees (default 0)
// Returns: maneuver node object
function change_inclination {
    parameter target_inc is 0.
    
    // Find nearest node
    local t_an is time_to_an().
    local t_dn is time_to_dn().
    local use_an is t_an < t_dn.
    local node_time is time:seconds + (choose t_an if use_an else t_dn).
    
    // Calculate delta-v magnitude
    local dv is plane_change_dv(target_inc, node_time).
    
    // At AN: positive inc change = normal, negative = antinormal
    // At DN: opposite - positive inc change = antinormal, negative = normal
    local inc_diff is target_inc - ship:orbit:inclination.
    local sign is (choose 1 if use_an else -1) * (choose 1 if inc_diff > 0 else -1).
    local normal_dv is dv * sign.
    
    // Create the node (prograde, normal, radial, time)
    return node(node_time, 0, normal_dv, 0).
}

// Test function
function test_change_inclination {
    local target_inc is 0.
    local current_inc is ship:orbit:inclination.
    local nd is change_inclination(target_inc).
    local t_an is time_to_an().
    local t_dn is time_to_dn().
    
    // Print details for inspection
    print "Current inclination: " + current_inc.
    print "Target inclination: " + target_inc.
    print "Time to AN: " + t_an.
    print "Time to DN: " + t_dn.
    print "Node time from now: " + (nd:time - time:seconds).
    print "Node components:".
    print "  Prograde: " + nd:prograde.
    print "  Normal: " + nd:normal.
    print "  Radial: " + nd:radial.
    
    // Validate node parameters
    local valid is true.
    
    // Check if node is at either AN or DN
    local at_node is abs(nd:time - (time:seconds + t_an)) < 0.1 or
                  abs(nd:time - (time:seconds + t_dn)) < 0.1.
    if not at_node {
        print "ERROR: Node not at AN or DN".
        set valid to false.
    }
    
    // Check if prograde and radial components are zero
    if abs(nd:prograde) > 0.01 or abs(nd:radial) > 0.01 {
        print "ERROR: Non-zero prograde or radial component".
        set valid to false.
    }
    
    // Check if normal component has reasonable magnitude
    local vel is velocityat(ship, nd:time):orbit:mag.
    if abs(nd:normal) > 2 * vel {
        print "ERROR: Normal component exceeds maximum possible delta-v".
        set valid to false.
    }
    
    // Check if sign of normal component matches node choice
    local using_an is abs(nd:time - (time:seconds + t_an)) < 0.1.
    local inc_diff is target_inc - current_inc.
    local correct_sign is (using_an and inc_diff > 0 and nd:normal > 0) or
                        (using_an and inc_diff < 0 and nd:normal < 0) or
                        (not using_an and inc_diff > 0 and nd:normal < 0) or
                        (not using_an and inc_diff < 0 and nd:normal > 0).
    if not correct_sign {
        print "ERROR: Incorrect sign of normal component".
        set valid to false.
    }
    
    return valid.
}

parameter target_incl is 0.
if hasNode {
    remove nextnode.
}
print "Before inclination: " + round(ship:orbit:inclination, 3).
local nd is change_inclination(target_incl).
add nd.
print "Normal dV: " + round(nd:normal, 3).
run next.
print "After inclination: " + round(ship:orbit:inclination, 3).