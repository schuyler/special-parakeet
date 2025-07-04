@lazyGlobal off.

run "kepler".

// Compute maneuver node from desired delta-V vector
function maneuver_from_delta_v {
    parameter t.
    parameter delta_v.
    parameter orbit_ is orbit.

    local orbit_t is orbit_at(t, orbit_).
    local pos_t to orbit_t:position.   // SOI-RAW position
    local vel_t to orbit_t:velocity.   // SOI-RAW velocity

    // The prograde direction is simply the instantaneous velocity direction.
    local v_prograde to vel_t.
    // The normal direction is perpendicular to current velocity and position.
    local v_normal to vcrs(vel_t, pos_t).
    // The radial direction is at right angles to the prograde and normal directions.
    local v_radial to vcrs(v_normal, v_prograde).

    // Now compute the components in each axis of the desired delta-v
    local dv_prograde is vdot(delta_v, v_prograde:normalized).
    local dv_normal is vdot(delta_v, v_normal:normalized).
    local dv_radial is vdot(delta_v, v_radial:normalized).
        
    // Construct the maneuver node
    return node(t, dv_radial, dv_normal, dv_prograde).
}  

// -- NOT FUNCTIONING YET --
function plane_change_maneuver {
    parameter i_1.
    parameter orbit_ is orbit.

    // Time mark
    local t_0 is time.

    // The orbit normal for the current orbit is the cross product of position and velocity vectors.
    local pos_0 is orbit_:position.
    local vel_0 is orbit_:velocity:orbit.
    local normal_0 to vcrs(pos_0, vel_0):normalized.

    // Now construct the orbit normal for the target orbit.
    // Rotate the X axis around the Y axis to point to the longitude of the ascending node
    local x_axis to V(1, 0, 0).
    local y_axis to V(0, 1, 0).
    local node_direction to angleaxis(orbit_:longitudeofascendingnode, y_axis) * x_axis.

    // Rotate the Y axis of this reference frame to the target inclination.
    local normal_1 to angleaxis(i_1, node_direction) * y_axis.

    // Find the intersection line between the initial and target orbital planes
    local intersection_line to vcrs(normal_0, normal_1):normalized.

    // The angle between the two orbital planes equals the angle between their normal vectors.
    local plane_angle to vang(normal_0, normal_1).

    // Rotate the current velocity vector by the angle needed to bring it into the new orbital plane.
    // The velocity magnitude is unchanaged because the orbital potential energy is unchanged.
    local vel_1 to angleaxis(plane_angle, intersection_line) * vel_0.
    
    // Now find future node crossing
    local delta_nu is vang(pos_0:normalized, intersection_line).
    local node_crossing is t_0 + delta_nu * (orbit_:period / 360).

    // TODO: Make sure the sign of the delta_v vector matches the N/S direction of the node crossing

    // Compose the maneuver node
    local delta_v to vel_1 - vel_0.
    return maneuver_from_delta_v(node_crossing, delta_v).
}

// === test code === //

function test_maneuver_from_delta_dv {
    add maneuver_from_delta_v(time + 60, v(0, 50, 0)).
}

// test_maneuver_from_delta_dv().

function test_plane_change_maneuver {
    parameter i_1 is 0.
    add plane_change_maneuver(i_1).
}

test_plane_change_maneuver(0).