runpath("./kepler").

function deorbit_for_ksc {
    parameter lead_angle is 90.
    parameter dv is -100.
    local t to time_to_longitude(wrap_longitude(-74.5 - lead_angle)).
    until not hasNode {
        remove nextNode.
    }
    add node(t:seconds, 0, 0, dv).
}

parameter theta is 115.
deorbit_for_ksc(theta).