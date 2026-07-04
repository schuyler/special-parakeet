parameter target_apoapsis is 72000.
parameter speed_factor is 28. // this should be based on TWR probably

local start_time to time:seconds.

// clearscreen.
print "".
print "== SSTO LAUNCH ==".

///// CONFIGURE /////

set dir to 90.

///// set up flight controls /////

set hdg to heading(dir, 0).
set sas to false.
set brakes to false.

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

set dir to 90.
set hdg to heading(dir, 3).
wait until vang(hdg:vector, ship:facing:vector) < 2.

set max_speed to airspeed.
wait 5.
until airspeed < max_speed {
  set max_speed to airspeed.
  wait 1.
}

print "Starting ascent in dry mode at " + round(airspeed) + " m/s.".
wait until airspeed <= altitude / speed_factor or airspeed < 120.

// NOTE: interestingly, slowing down to almost stall speed is a big waste of fuel.
//
// lock current_pitch to 90 - vang(ship:up:vector, srfprograde:vector).
// print "Current pitch: " + round(current_pitch, 1) + "º.".
// wait until current_pitch <= 1.
// unlock current_pitch.

///// WET MODE ASCENT /////

print "Switching engines to wet mode at " + round(altitude) + "m.".
//set hdg to heading(dir, 5). // KNOWN WORKING

// Todo: replace "pitch" with "AoA"

set min_pitch to 5.
set max_pitch to 30.
set pitch to min_pitch.
set d_pitch to 0.001.

lock hdg to heading(dir, pitch).
set ag1 to true.

set start_speed to airspeed.
set max_speed to start_speed.

wait 5.
lock ideal_speed to max(start_speed, altitude / speed_factor).

// Until we start to lose airspeed at or below min_pitch...
until max_speed > airspeed and verticalspeed <= 0 {

  // Pitch up if we're going faster than we want
  if airspeed > ideal_speed and pitch < max_pitch {
    set pitch to pitch + d_pitch * (airspeed - ideal_speed).
  }

  // Pitch down if we're going slower than we want
  if (airspeed < ideal_speed or verticalspeed <= 0) and pitch > min_pitch {
    set pitch to pitch + d_pitch * (airspeed - ideal_speed).
  }

  // Track our fastest speed so far.
  if max_speed < airspeed {
    set max_speed to airspeed.
  }

  // print "V: " + round(verticalspeed) + " m/s | P: " + round(pitch) + "º".
  wait 0.1.
}

///// ROCKET ASCENT /////

print "Activating rocket engines at " + round(altitude) + "m.".
stage.
wait 5.

if airspeed < 500 {
  set pitch to min_pitch.
  wait until ship:airspeed > 500.
}

print "Starting rocket ascent.".

// 20º is too flat to start. OTOH it results in an orbital burn of 98 m/s.
// 22º is pretty good and doesn't result in much heating.

set ascent to 21.
until pitch >= ascent or eta:apoapsis >= 30 {
  set pitch to pitch + 0.25.
  wait 0.5.
}
set final_pitch to pitch.

wait 5. // settle down
wait until eta:apoapsis > 30.

print "Pitching down at " + round(altitude) + "m.".
set ag2 to true.

// Level off -- this is a TOTAL hack
// this probably wants some computation based on drag

lock pitch to min(final_pitch, ascent - (eta:apoapsis - 30) / 2).
lock hdg to heading(dir, max(0, pitch)).

// until eta:apoapsis > 60 {
//   print round(pitch) + "º / " + round(eta:apoapsis) + "s @ " + round(altitude) + "m.".
//  wait 5.
// }

wait until eta:apoapsis > 60 or altitude > 40000.

print "Setting heading to prograde at " + round(altitude) + "m.".
lock steering to prograde.

wait until apoapsis > target_apoapsis.
lock throttle to 0.
wait 1.

/// Keep apoapsis suborbital.

until altitude >= 70000 {
  if apoapsis < 70000 {
    print "Burning to keep apoapsis above atmospheric boundary.".
    lock throttle to 1.
    wait until apoapsis > target_apoapsis.
    lock throttle to 0.
  }
  wait 1.
}


/// FINISH IN SUB-ORBIT

set warp to 0.
unlock throttle.
unlock steering.

wait until kuniverse:timewarp:issettled.

panels on.

run circularize.
run next.

local lng to body:geopositionof(ship:position):lng.
print "Longitude after circularization: " + round(lng, 3) + "º.".
print "Time to orbit: " + round(time:seconds - start_time) + "s.".
