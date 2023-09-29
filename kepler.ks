// Bunch of Keplerian math.
//
// Translated all this from https://physics.stackexchange.com/a/333897
//
// But you know what? Don't need it. See orbit.ks instead

function _eccentricity {
  parameter apo, peri.
  parameter r_b is ship:body:radius.
  local r_p to peri + r_b.
  local r_a to apo + r_b.
  return (r_a - r_p) / (r_a + r_p).
}

function _semi_major_axis {
  parameter apo, peri.
  parameter r_b is ship:body:radius.
  return r_b + (apo + peri) / 2.
}

function _ecc_anomaly {
  parameter e, a, r_. // r_ is radius from body CoM
  local ratio to (a - r_) / (e * a).
  // clamping ratio needed because floating point rounding at the boundary leads to arccos returning NaN
  local ecc to arccos(min(max(ratio, -1), 1)). // return º
  // arccos() returns [0, 180º] but E is in range [0, 360º]
  // https://www.reddit.com/r/Kos/comments/4tm0wq/two_common_mistakes_people_make_when_calculating/
  if sin(ratio) > 0 {
    return ecc.
  } else {
    return -ecc.
  }
}

function _mean_anomaly {
  parameter e, a, r_.
  local e_r to _ecc_anomaly(e, a, r_).
  // the first term of m_r is in degrees, but the second is in radians, so convert
  local m_r to e_r - e * sin(e_r) * 180 / constant:pi.
  return m_r. // returns º in the range [0, 360)
}

function _orbital_period {
  // Kepler's 3rd law
  parameter a.
  parameter mu is ship:body:mu.
  return 2 * constant:pi * sqrt(a ^ 3 / mu).
}

function _time_since_periapsis {
  parameter alt_.
  parameter apo is ship:orbit:apoapsis.
  parameter peri is ship:orbit:periapsis.
  parameter r_ is ship:body:radius + alt_.
  local a to semi_major_axis(apo, peri).
  local e to eccentricity(apo, peri).
  local m_r to mean_anomaly(e, a, r_). // returns º
  // mean_anomaly is in range [0, 360º] so scale to fraction of an orbit
  return orbital_period(a) * m_r / 360.
}

function _time_to_altitude {
  parameter target_alt.
  parameter start_alt is ship:altitude.
  parameter apo is ship:orbit:apoapsis.
  parameter peri is ship:orbit:periapsis.
  local t0 to time_since_periapsis(start_alt, apo, peri).
  local t1 to time_since_periapsis(target_alt, apo, peri).
  local dt to t1 - t0.
  if dt < 0 {
    set dt to dt + orbital_period(semi_major_axis(apo, peri)).
  }
  return dt.
}

