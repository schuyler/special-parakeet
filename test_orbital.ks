run orbital.

clearscreen.

set t to timestamp().
set alt_ to altitude_at(orbit, t).
print "Current altitude: " + round(alt_) + "m".

set m to mean_anomaly_at_t(orbit, t).
print "Mean anomaly based on time: " + round(m, 4).
set m to mean_anomaly_at_r(orbit, alt_ + body:radius).
print "Mean anomaly based on altitude: " + round(m, 4).

set dt to time_to_altitude(orbit, orbit:apoapsis).
print "Time to apoapsis: " + dt:minute + ":" + dt:second.
print "".
set dt to time_to_altitude(orbit, orbit:periapsis).
print "Time to periapsis: " + dt:minute + ":" + dt:second.



