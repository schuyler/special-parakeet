// --- ORBITAL COMPUTATION

// Eccentric anomaly at a given orbital radius (distance from body CoM)
function eccentric_anomaly_at_r {
  parameter ob is ship:orbit.
  parameter r_ is altitude_at(ob) + ob:body:radius. 
  local a to ob:semimajoraxis.
  local e to ob:eccentricity.
  // according to Wikipedia, E = acos((a - r_) / (e * a)).
  // ratio is not a good name for this relation
  local ratio to (a - r_) / (e * a).
  // ratio clamping needed because floating point rounding at the boundary leads to arccos returning NaN
  local ecc to arccos(min(max(ratio, -1), 1)). // return º
  // arccos() returns [0, 180º] but E is in range [0, 360º]
  // https://www.reddit.com/r/Kos/comments/4tm0wq/two_common_mistakes_people_make_when_calculating/
  if sin(ratio) > 0 {
    return ecc.
  } else {
    return -ecc.
  }
}

// How far have we come around the orbit in angular units at orbital radius r_,
// since last periapsis?
// TODO: explain how this happens twice in an orbit but we get the right result?
// Probably something to do with the sin(ratio) check above... apoapsis is always 180º
//
function mean_anomaly_at_r {
  parameter ob is ship:orbit.
  parameter r_ is altitude_at(ob) + ob:body:radius. 
  local e to ob:eccentricity.
  local e_r to eccentric_anomaly_at_r(ob, r_).
  // according to Wikipedia, M = E - e * sin(E) ... but all in radians.
  // the first term of m_r is in degrees, but the second is in radians, so convert
  local m_r to e_r - e * sin(e_r) * 180 / constant:pi.
  return 360 -  m_r. // returns º in the range [0, 360)
}

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

// vis-viva equation
function orbital_speed {
  parameter orbit_ is ship:orbit.
  parameter altitude_ is altitude_at(orbit_).
  // could be a different orbit
  parameter apo is orbit_:apoapsis.
  parameter peri is orbit_:periapsis.

  local body_ to orbit_:body.
  local g to body_:mu.
  local r_ to body_:radius + altitude_.
  local a to body:radius + (apo + peri) / 2.
  return sqrt(g * ((2 / r_) - (1 / a))).
}

// --- kOS ORBIT HELPERS

// Give us a kOS Orbit object for a given timestamp.
function orbit_at { // return Orbit
  parameter ob is ship:orbit.
  parameter t is timestamp().
  return createorbit(
    ob:inclination,
    ob:eccentricity,
    ob:semimajoraxis,
    ob:lan,
    ob:argumentofperiapsis,
    mean_anomaly_at_t(ob, t),
    t:seconds,
    ob:body).
}

// What's the altitude of that orbit at time t?
// This is defined on Orbitables but not Orbits
function altitude_at { // return m above sea level
  parameter ob0 is ship:orbit.
  parameter t is timestamp().
  local ob to orbit_at(ob0, t).
  return ob:body:altitudeof(ob:position).
} 

// Get the body-surface coordinates of the orbit but at time t
function geoposition_at {
  parameter ob0 is ship:orbit.
  parameter t is timestamp().
  local ob to orbit_at(ob0, t).
  return ob:body:geopositionof(ob:position).
}

// Estimate the time since periapsis, given an orbital height.
function time_since_periapsis {
  parameter ob is ship:orbit.
  parameter alt_ is altitude_at(ob).
  local m to mean_anomaly_at_r(ob, alt_ + ob:body:radius).
  // mean_anomaly is in range [0, 360º] so scale to fraction of an orbit
  return timespan(ob:period * m / 360).
}

// Estimate the time to reach a given orbital height from the current orbit.
function time_to_altitude {
  parameter ob is ship:orbit.
  parameter target_alt is ob:apoapsis.
  local t0 to time_since_periapsis(ob, altitude_at(ob)).
  local t1 to time_since_periapsis(ob, target_alt).
  local dt to t1 - t0.
  if dt < 0 {
    set dt to dt + ob:period.
  }
  return dt.
}

// Get the current Orbit when it reaches the specified altitude.
// This new object will contain the body-centric position at that time.
function orbit_at_altitude {
  parameter orbit_ is ship:orbit.
  parameter alt_ is 0.
  local t to time_to_altitude(orbit_, alt_).
  return orbit_at(orbit_, time:seconds + t).
}

function time_to_geo_lng {
  parameter orbit_ is ship:orbit.
  parameter given_lng is deorbit_lng.

  set geo_pos to orbit_:body:geopositionof(orbit_:position).
  set d_lng to given_lng - geo_pos:lng.
  if d_lng < 0 {
    set d_lng to d_lng + 360.
  }
  // not sure this rotation accounting is correct
  return orbit_:period * (1 + orbit_:period / orbit_:body:rotationperiod) * (d_lng / 360).
}
