@lazyglobal off.

// --- ORBITAL COMPUTATION

// The orbit normal. In kOS's left-handed frame this points the opposite
// way from the right-handed textbook convention, but as long as every
// normal comes from this function, angles between them come out right.
function orbit_normal {
  parameter obj is ship.
  parameter t is time:seconds.
  local r_ is positionat(obj, t) - obj:body:position.
  local v_ is velocityat(obj, t):orbit.
  return vcrs(v_, r_):normalized.
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

// Convert true anomaly to mean anomaly for an orbit
function true_to_mean_anomaly {
  parameter theta.     // True anomaly in degrees
  parameter ecc.      // Orbit eccentricity
  
  // First convert to eccentric anomaly
  local E is arctan2(sqrt(1-ecc^2) * sin(theta), ecc + cos(theta)).
  
  // Then to mean anomaly (KEEP the radtodeg factor!)
  local M is E - ecc * sin(E) * constant:radtodeg.
  
  // Return normalized to 0-360
  if M < 0 { 
    return M + 360. 
  }
  return M.
}

// Convert mean anomaly to true anomaly for an orbit
function mean_to_true_anomaly {
  parameter M.        // Mean anomaly in degrees
  parameter ecc.      // Orbit eccentricity
  parameter epsilon is 0.0001.  // Desired precision in degrees
  
  // Solve Kepler's equation iteratively to find E (eccentric anomaly)
  local E is M.  // Initial guess
  local delta is epsilon + 1.
  local count is 0.
  until delta < epsilon and count < 100 {
    local E_next is M + ecc * sin(E) * constant:radtodeg.  // KEEP radtodeg!
    set delta to abs(E_next - E).
    set E to E_next.
    set count to count + 1.
  }
  
  // Convert to true anomaly
  local theta is arctan2(sqrt(1-ecc^2) * sin(E), cos(E) - ecc).
  
  // Return normalized to 0-360
  if theta < 0 {
    return theta + 360.
  }
  return theta.
}

// We going up or down?
function is_ascending {
  parameter ob is ship:orbit.
  parameter t is timestamp().
  return mean_anomaly_at_t(t) < 180. // before apoapsis
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

// vis-viva equation in its natural variables: speed at radius r_ on an
// orbit with semi-major axis a_. Everything else is a repackaging of this.
function visviva {
  parameter r_, a_.
  parameter mu is ship:body:mu.
  return sqrt(mu * ((2 / r_) - (1 / a_))).
}

// vis-viva, packaged as speed at an altitude on an orbit given by apo/peri
function orbital_speed_v1 {
  parameter orbit_ is ship:orbit.
  parameter altitude_ is altitude_at(orbit_).
  // could be an orbit with different parameters
  parameter apo is orbit_:apoapsis.
  parameter peri is orbit_:periapsis.

  local body_ to orbit_:body.
  return visviva(body_:radius + altitude_,
                 body_:radius + (apo + peri) / 2,
                 body_:mu).
}

// Time until an orbit next passes the given apsis: 0 = periapsis, 180 =
// apoapsis (those being the apsides' mean anomalies).
function time_to_apsis {
  parameter ob.
  parameter aps_m.
  local m is mean_anomaly_at_t(ob).
  return mod(aps_m - m + 360, 360) / 360 * ob:period.
}

// Angle from our position at time t to a body-centered direction, measured
// forward along our direction of motion, [0, 360). Handedness-free: built
// from dot products against our own radial and prograde directions.
function angle_ahead {
  parameter dir_.
  parameter t is time:seconds.
  local rdir is (positionat(ship, t) - body:position):normalized.
  local vdir is velocityat(ship, t):orbit:normalized.
  local ang is arctan2(vdot(dir_:normalized, vdir), vdot(dir_:normalized, rdir)).
  if ang < 0 {
    set ang to ang + 360.
  }
  return ang.
}

// --- TARGET-RELATIVE PREDICTION (these need a target set)

function relative_inclination {
  parameter t is time:seconds.
  return vang(orbit_normal(ship, t), orbit_normal(target, t)).
}

// Ship-to-target distance, t seconds from now. Honors planned maneuver
// nodes, so this predicts the flight plan, not just the current orbit.
function separation_at {
  parameter t.
  return (positionat(target, time:seconds + t) - positionat(ship, time:seconds + t)):mag.
}

// Predicted closest approach to the target over the window [t0, t1] seconds
// from now, honoring planned nodes (separation_at does). Returns a lexicon
// "t" = seconds from now at which the gap is smallest, "dist" = that gap in
// metres. samples is how many points minimize_scan lays across the window
// before refining the best bracket: a wide window hiding a brief, sharp
// encounter needs more of them, or the scan steps clean over the dip.
// Needs minimize_scan from optimize.ks (pulled in by common).
function closest_approach {
  parameter t0, t1.
  parameter samples is 24.
  local t_ca is minimize_scan(separation_at@, t0, t1, 2, samples).
  return lexicon("t", t_ca, "dist", separation_at(t_ca)).
}

// Target's velocity relative to ours, t seconds from now.
function relative_velocity_at {
  parameter t.
  return velocityat(target, time:seconds + t):orbit - velocityat(ship, time:seconds + t):orbit.
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
// Returns a RELATIVE dt in seconds. (core/kepler.ks's time_to_altitude
// returns an absolute timestamp — hence the distinct name, so both
// libraries can be loaded together.)
function time_until_altitude {
  parameter ob is ship:orbit.
  parameter target_alt is ob:periapsis.
  parameter ascending is false.

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

// Estimate how long it will take to cross a particular meridian on the parent
// body... assuming the orbit is equatorial ... :D :D :D *sob*
//
function time_to_meridian {
  parameter orbit_ is ship:orbit.
  parameter given_lng is 0.

  local geo_pos to orbit_:body:geopositionof(orbit_:position).
  local d_lng to given_lng - geo_pos:lng.
  if d_lng < 0 {
    set d_lng to d_lng + 360.
  }
  
  // Current true anomaly
  local current_true_anomaly to orbit_:trueanomaly.
  
  // Simple relationship: longitude advance = true anomaly advance
  local target_true_anomaly to mod(current_true_anomaly + d_lng, 360).
  
  // Convert to mean anomaly and calculate time
  local target_mean_anomaly to true_to_mean_anomaly(target_true_anomaly, orbit_:eccentricity).
  local current_ma to mean_anomaly_at_t(orbit_).
  
  local ma_diff to target_mean_anomaly - current_ma.
  if ma_diff < 0 {
    set ma_diff to ma_diff + 360.
  }
  
  local mean_motion to 360 / orbit_:period.
  local dt to (ma_diff / mean_motion).
  return timespan(dt).
}

// Altitude of an orbit at time t, straight from the conic equation:
// r = a(1 - e²) / (1 + e·cos ν). No orbit_at/createorbit trick needed.
// (This file used to define orbit_at and geoposition_at too; those names
// now belong to core/kepler.ks, whose versions the descent scripts use,
// and removing them here makes the two libraries safe to load together.)
function altitude_at { // return m above sea level
  parameter ob0 is ship:orbit.
  parameter t is timestamp().
  local e is ob0:eccentricity.
  local m is mean_anomaly_at_t(ob0, t).
  local nu is mean_to_true_anomaly(m, e).
  local r_ is ob0:semimajoraxis * (1 - e ^ 2) / (1 + e * cos(nu)).
  return r_ - ob0:body:radius.
}

// Test the conversion functions
// local current_ma to mean_anomaly_at_t(ship:orbit).
// local current_ta to ship:orbit:trueanomaly.
// local converted_ta to mean_to_true_anomaly(current_ma, ship:orbit:eccentricity).
// local converted_ma to true_to_mean_anomaly(current_ta, ship:orbit:eccentricity).

// print("Current MA: " + round(current_ma, 2)).
// print("Current TA: " + round(current_ta, 2)).
// print("MA->TA conversion: " + round(converted_ta, 2)).
// print("TA->MA conversion: " + round(converted_ma, 2)).