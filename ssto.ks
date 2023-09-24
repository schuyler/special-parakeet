clearscreen.
print "== SSTO LAUNCH ==".

///// CONFIGURE /////

set dir to 90.

///// set up flight controls /////

set hdg to heading(dir, 0).
set sas to false.

lock throttle to 1.0.
lock steering to hdg.

set lights to true.

///// TAKE OFF SEQUENCE /////

print "Starting engines.".
stage.
wait until airspeed > 120.

print "Take off.".
set hdg to heading(dir, 20).
wait 3.
set gear to false.

wait until altitude > 500.

set warpmode to "physics".
set warp to 2.

///// DRY MODE ASCENT /////

print "Accelerating to maximum speed.".
set hdg to heading(dir, 3).

set max_speed to airspeed.
wait 5.
until airspeed < max_speed {
  set max_speed to airspeed.
  wait 1.
}

print "Starting ascent in dry mode at " + round(airspeed) + " m/s.".
wait until airspeed <= altitude / 16.

// NOTE: interestingly, slowing down to almost stall speed is a big waste of fuel.
//
// lock current_pitch to 90 - vang(ship:up:vector, srfprograde:vector).
// print "Current pitch: " + round(current_pitch, 1) + "º.".
// wait until current_pitch <= 1.
// unlock current_pitch.

///// WET MODE ASCENT /////

print "Switching engines to wet mode at " + round(altitude) + "m.".
set hdg to heading(dir, 5). // KNOWN WORKING
set ag1 to true.

// Todo: replace "pitch" with "AoA"

set min_pitch to 3.
set max_pitch to 30.
set pitch to min_pitch.
set d_pitch to 0.005.

set max_speed to airspeed.
lock hdg to heading(dir, pitch).
set start_speed to airspeed.
lock ideal_speed to max(start_speed, altitude / 16).

wait 5.

// Until we start to lose airspeed at or below min_pitch...
until max_speed > airspeed and pitch <= min_pitch {

  // Pitch up if we're going faster than we want
  if airspeed > ideal_speed and pitch < max_pitch {
    set pitch to pitch + d_pitch * (airspeed - ideal_speed).
  }

  // Pitch down if we're going slower than we want
  if airspeed < ideal_speed and pitch > min_pitch {
    set pitch to pitch + d_pitch * (airspeed - ideal_speed).
  }

  // Track our fastest speed so far.
  if max_speed < airspeed {
    set max_speed to airspeed.
  }
  wait 0.1.
}

unlock hdg.

///// ROCKET ASCENT /////

print "Activating rocket engines at " + round(altitude) + "m.".
stage.
wait 5.

if airspeed < 500 {
  set hdg to heading(dir, 5).
  wait until ship:airspeed > 500.
}

print "Starting rocket ascent.".

// 20º is too flat to start. OTOH it results in an orbital burn of 98 m/s.
// 22º is pretty good and doesn't result in much heating.

set ascent to 22.
set hdg to heading(dir, ascent).

wait 5. // settle down
wait until eta:apoapsis > 30.

print "Pitching down at " + round(altitude) + "m.".
set ag2 to true.

// Level off -- this is a TOTAL hack
// this probably wants some computation based on drag

lock pitch to ascent - (eta:apoapsis - 30) / 2.
lock hdg to heading(dir, max(0, pitch)).

until eta:apoapsis > 60 {
  print round(pitch) + "º / " + round(eta:apoapsis) + "s @ " + round(altitude) + "m.".
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
