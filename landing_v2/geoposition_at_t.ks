@lazyGlobal off.

function geoposition_at_t {
    parameter geo.
    parameter ut.
    local body_ is geo:body.
    // Account for body rotation between now and ut. Thankfully no body in KSP has axial tilt.
    local dt is ut - time.
    local rotation_deg is dt:seconds * (360 / body_:rotationperiod).
    // We _subtract_ rotation_deg to find what position the target will _be_ at ut.
    local final_lng is geo:lng - rotation_deg.
    if final_lng < -180 {
        set final_lng to final_lng + 360.
    } else if final_lng > 180 {
        set final_lng to final_lng - 360.
    }
    return body_:geopositionlatlng(geo:lat, final_lng).
}

function test_geoposition_at_t {
    local geo is body:geopositionlatlng(0, 0).
    local ut is time + body:rotationperiod / 2.
    local future_geo is geoposition_at_t(geo, ut).
    print "Current position: " + geo.
    print "Future position at " + ut + ": " + future_geo. // should be 180 degrees away from current position
}

// test_geoposition_at_t.