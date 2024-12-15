@lazyglobal off.

run "orbit_at_t".

// --- ORBITAL COMPUTATION

// How far have we come around the orbit in angular units at time t, since last
// periapsis?
//
function mean_anomaly_at_t { // return [0, 360)º
  parameter ob is ship:orbit.  
  parameter t is timestamp().

  // as Wikipedia says, M = M0 + n * (t - t0) where n = 360º / T.
  local m to ob:meananomalyatepoch + (360 / ob:period) * (t:seconds - ob:epoch).
  return mod(m, 360). 
}

// We going up or down?
function is_ascending {
  parameter ob is ship:orbit.
  parameter t is timestamp().
  return mean_anomaly_at_t(ob, t) < 180. // before apoapsis
}

// Estimate the time since periapsis.
//
function time_since_periapsis {
  parameter ob is ship:orbit.
  // mean anomaly is the orbital angle since periapsis
  parameter m is mean_anomaly_at_t(ob).
  // mean_anomaly is in range [0, 360º] so scale to fraction of an orbit
  return timespan(ob:period * m / 360).
}

// vis-viva equation
function orbital_speed {
  parameter orbit_ is ship:orbit.
  parameter altitude_ is altitude_at(orbit_).
  // could be an orbit with different parameters
  parameter apo is orbit_:apoapsis.
  parameter peri is orbit_:periapsis.

  local body_ to orbit_:body.
  local g to body_:mu.
  local r_ to body_:radius + altitude_.
  local a to body_:radius + (apo + peri) / 2.
  return sqrt(g * ((2 / r_) - (1 / a))).
}

// --- ALTITUDE COMPUTATION

// Eccentric anomaly at a given orbital radius (distance from body CoM)
//
function eccentric_anomaly_at_r {
  parameter ob is ship:orbit.
  parameter r_ is altitude_at(ob) + ob:body:radius. 
  local a to ob:semimajoraxis.
  local e to ob:eccentricity.
  // according to Wikipedia, E = acos((a - r_) / (e * a)).
  // ratio is not a good name for this relation
  local ratio to (a - r_) / (e * a).
  // ratio clamping needed because floating point rounding at the boundary leads to arccos returning NaN
  set ratio to min(max(ratio, -1), 1).
  local ecc to arccos(ratio). // return º
  // arccos() returns [0, 180º] but this is OK because E is the same on either side of periapsis
  // https://www.reddit.com/r/Kos/comments/4tm0wq/two_common_mistakes_people_make_when_calculating/
  return ecc.
}

// How far have we come around the orbit since last periapsis in order to find
// ourselves at orbital radius r_? Two answers, no guessing.
//
function mean_anomaly_at_r {
  parameter ob is ship:orbit.
  parameter r_ is altitude_at(ob) + ob:body:radius. 
  local e to ob:eccentricity.
  local e_r to eccentric_anomaly_at_r(ob, r_).

  // according to Wikipedia, M = E - e * sin(E) ... but all in radians.
  // the first term of m_r is in degrees, but the second is in radians, so convert
  local m_r to e_r - e * sin(e_r) * constant:radtodeg.

  // m_r is now always [0, 180º] - but there are actually two valid answers,
  // one is before apoapsis and the other is after apoapsis. Consider cos(x)
  // = cos(-x) = cos(360 - x) with periapsis at 0º exactly between xº and -xº.
  //
  return list(m_r, 360 - m_r).
}

// Estimate the time to reach a given orbital height from the current orbit.
function time_to_altitude {
  parameter ob is ship:orbit.
  parameter target_alt is ob:periapsis.
  parameter ascending is is_ascending(ob).

  // Get the mean anomaly at the specified time.
  local m0 to mean_anomaly_at_t(ob).
    
  // Estimate the mean anomaly at the future altitude. There will be two such
  // altitudes within the next orbit.
  //
  local r1 to target_alt + ob:body:radius.
  local ms1 to mean_anomaly_at_r(ob, r1).

  // We want the altitude given before or after apoapsis?
  local m1 to 0.
  if ascending {
    set m1 to ms1[0].
  } else {
    set m1 to ms1[1].
  }

  // Compute the time from periapsis to each mean anomaly, and then subtract
  // the future timestamp from the current one.
  //
  local t0 to time_since_periapsis(ob, m0).
  local t1 to time_since_periapsis(ob, m1).
  local dt to t1 - t0.
  if dt < 0 {
    set dt to dt + ob:period. // wrap around periapsis
  }
  return dt.
}

// --- kOS ORBIT HELPERS

// What's the altitude of that orbit at time t?
// This is defined on Orbitables but not Orbits, so we cheat.
//
function altitude_at { // return m above sea level
  parameter ob0 is ship:orbit.
  parameter t is timestamp().
  local ob to orbit_at_t(ob0, t).
  return ob:body:altitudeof(ob:position).
} 

// Get the body-surface coordinates of the orbit but at time t
//
function geoposition_at {
  parameter ob0 is ship:orbit.
  parameter t is timestamp().
  local ob to orbit_at_t(ob0, t).
  return ob:body:geopositionof(ob:position).
}


// -- TEST FUNCTION

function test_orbital {
  until false {
    clearscreen.
    local t to timestamp().

    local alt_ to altitude_at(orbit, t).
    print "Current altitude: " + round(alt_) + " m".
  
    local alt2 to altitude_at(orbit, t + 60).
    print "Altitude in 60s: " + round(alt2) + " m".

    local s to orbital_speed(orbit, alt_).
    print "Orbital speed: " + round(s, 1) + " m/s".

    local m to mean_anomaly_at_t(orbit, t).
    print "Mean anomaly based on time: " + round(m, 3) + "º".
    set m to mean_anomaly_at_r(orbit, alt_ + body:radius).
    print "Mean anomaly based on altitude: " + round(m[0], 3) + "ª " + round(m[1], 3) + "º".

    local dt to time_to_altitude(orbit, orbit:apoapsis).
    print "Time to apoapsis: " + dt:minute + ":" + dt:second.
    print "".
    set dt to time_to_altitude(orbit, orbit:periapsis).
    print "Time to periapsis: " + dt:minute + ":" + dt:second.

    wait 1.
  }
}