run orbital.

until false {
  clearscreen.
  set t to timestamp().

  set s to orbital_speed(orbit, alt_).
  print "Orbital speed: " + round(s, 1) + " m/s".

  set alt_ to altitude_at(orbit, t).
  print "Current altitude: " + round(alt_) + " m".
 
  set alt2 to altitude_at(orbit, t + 60).
  print "Altitude in 60s: " + round(alt2) + " m".

  set m to mean_anomaly_at_t(orbit, t).
  print "Mean anomaly based on time: " + round(m, 3) + "º".
  set m to mean_anomaly_at_r(orbit, alt_ + body:radius).
  print "Mean anomaly based on altitude: " + round(m[0], 3) + "ª " + round(m[1], 3) + "º".

  set dt to time_to_altitude(orbit, orbit:apoapsis).
  print "Time to apoapsis: " + dt:minute + ":" + dt:second.
  print "".
  set dt to time_to_altitude(orbit, orbit:periapsis).
  print "Time to periapsis: " + dt:minute + ":" + dt:second.

  wait 1.
}
