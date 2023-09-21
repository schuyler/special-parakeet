clearscreen.
print "== ORIOLE LAUNCH ==".

///// set up flight controls

set hdg to heading(90, 0).
set sas to false.

lock throttle to 1.0.
lock steering to hdg.

set lights to true.

///// TAKE OFF SEQUENCE /////

print "Starting engines.".
stage.
wait until airspeed > 125.

print "Take off.".
set hdg to heading(90, 20).
wait 3.
set gear to false.

wait until altitude > 250.

set warpmode to "physics".
set warp to 2.

///// DRY MODE ASCENT /////

print "Accelerating to maximum speed.".
set hdg to heading(90, 3).

set maxspeed to airspeed.
wait 5.
until airspeed < maxspeed {
  set maxspeed to airspeed.
  wait 1.
}

print "Starting ascent in dry mode at " + round(airspeed) + " m/s.".
wait until airspeed < 250 or altitude > 5000.

///// WET MODE ASCENT /////

print "Switching engines to wet mode at " + round(altitude) + "m.".
set hdg to heading(90, 5). // KNOWN WORKING
set ag1 to true.

// Replace this next block with something that accelerates to 500 m/s and holds it there
// Otherwise the nose naturally pitches up, which is fine but not consistent between crafts
// Also going too fast too low is draggy.

set maxspeed to airspeed.
wait 5.
until airspeed < maxspeed {
  set maxspeed to airspeed.
  wait 1.
}

wait until airspeed < 325 or altitude > 12000.

///// ROCKET ASCENT /////

print "Activating rocket engines at " + round(altitude) + "m.".
stage.
wait 1.

if airspeed < 500 {
  set hdg to heading(90, 5).
}

wait until ship:airspeed > 500.

print "Starting rocket ascent.".

// 20º is too flat to start. OTOH it results in an orbital burn of 98 m/s.

set ascent to 22.
set hdg to heading(90, ascent).

wait until eta:apoapsis > 30.

print "Pitching down at " + round(altitude) + "m.".
set ag2 to true.

// Level off -- this is a TOTAL hack

lock pitch to ascent - (eta:apoapsis - 30) / 2.
lock hdg to heading(90, max(0, pitch)).

until eta:apoapsis > 60 {
  print round(pitch) + "º > " + round(prograde:pitch) + "º / " + round(eta:apoapsis) + "s @ " + round(altitude) + "m.".
  wait 5.
 }

wait until eta:apoapsis > 60 or altitude > 40000.

print "Setting heading to prograde at " + round(altitude) + "m.".
lock steering to prograde.

wait until apoapsis > 72000.
lock throttle to 0.
wait 1.

/// FINISH IN SUB-ORBIT

set warp to 0.
unlock throttle.
unlock steering.

wait until kuniverse:timewarp:issettled.
