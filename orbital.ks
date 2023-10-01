@lazyglobal off.

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
  parameter target_alt is ob:apoapsis.

  // Get the mean anomaly at the current time.
  local m0 to mean_anomaly_at_t(ob).
    
  // Estimate the mean anomaly at the future altitude. There will be two such
  // altitudes within the next orbit.
  //
  local r1 to target_alt + ob:body:radius.
  local ms1 to mean_anomaly_at_r(ob, r1).

  // If the current mean anomaly is less than _or_ greater than _both_ of the
  // possible mean anomalies of the future altitude, pick the smaller one,
  // otherwise use the larger. It helps to visualize this on an ellipse and
  //
  local m1 to 0.
  if m0 > max(ms1[0], ms1[1]) or m0 < min(ms1[0], ms1[1])  {
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

// Estimate how long it will take to cross a particular meridian on the parent
// body... assuming the orbit is equatorial ... :D :D :D *sob*
//
function time_to_meridian {
  parameter orbit_ is ship:orbit.
  parameter given_lng is 0.

  local geo_pos to orbit_:body:geopositionof(orbit_:position).
  local d_lng to given_lng - geo_pos:lng.
  if d_lng < 0 {
    local d_lng to d_lng + 360.
  }
  // not sure this rotation accounting is correct
  local dt to orbit_:period * (1 + orbit_:period / orbit_:body:rotationperiod) * (d_lng / 360).
  return timespan(dt).  
}

// --- kOS ORBIT HELPERS

// Give us a kOS Orbit object as if orbiter were at a different point in its
// orbit right now. This feels like it shouldn't work.
//
function orbit_at { // return Orbit
  parameter ob is ship:orbit.
  parameter t is timestamp().
  return createorbit(
    ob:inclination,
    ob:eccentricity,
    ob:semimajoraxis,
    ob:lan,
    ob:argumentofperiapsis,
    // as if the orbiting body were at that point in the orbit
    mean_anomaly_at_t(ob, t),
    // ... but right now :D
    time:seconds,
    ob:body).
}

// What's the altitude of that orbit at time t?
// This is defined on Orbitables but not Orbits, so we cheat.
//
function altitude_at { // return m above sea level
  parameter ob0 is ship:orbit.
  parameter t is timestamp().
  local ob to orbit_at(ob0, t).
  return ob:body:altitudeof(ob:position).
} 

// Get the body-surface coordinates of the orbit but at time t
//
function geoposition_at {
  parameter ob0 is ship:orbit.
  parameter t is timestamp().
  local ob to orbit_at(ob0, t).
  return ob:body:geopositionof(ob:position).
}

// Get the current Orbit when it reaches the specified altitude.
// This new object will contain the body-centric position at that time.
function orbit_at_altitude {
  parameter orbit_ is ship:orbit.
  parameter alt_ is 0.
  local t to time_to_altitude(orbit_, alt_).
  return orbit_at(orbit_, timestamp() + t).
}
