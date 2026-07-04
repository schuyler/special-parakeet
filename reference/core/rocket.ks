@lazyGlobal off.

// Compute weighted average ISP of all engines on the vessel
function vessel_isp {
    local total_isp is 0.
    local total_thrust is ship:availablethrust.

    // Iterate over all engines and compute the weighted average Isp
    local eng_list to list().
    list engines in eng_list.
    for en_ in eng_list {
        set total_isp to total_isp + en_:isp * en_:availablethrust / total_thrust.
    }
    return total_isp.
}

function burn_duration {
    parameter dv_mag.

    // Rocket equation: delta_v = v_exhaust * ln(m_initial / m_final)
    //
    // v_exhaust is the effective exhaust velocity, which relates specific impulse (1/s)
    // in terms of how much velocity is produced per unit mass of the propellant.
    // g_0 is just the universal scaling factor for specific impulse (Isp).
    local isp is vessel_isp().
    local v_exhaust is isp * constant:g0.

    // Rearranging the Rocket Equation gives m_final = m_initial * e^(-delta_v / v_exhaust)
    local m_initial is ship:mass.
    local m_final is m_initial * constant:e ^ (-dv_mag / v_exhaust).
    
    // The thrust is the product of mass flow rate and effective exhaust velocity,
    // therefore the mass flow (rate of change of mass) can be expressed as:
    local mass_flow is ship:availablethrust / v_exhaust.

    // The burn duration, then, is how long it takes to expel the fuel mass:
    local dt_burn is (m_initial - m_final) / mass_flow.
    return dt_burn.
}