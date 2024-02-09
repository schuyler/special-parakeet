parameter target_apoapsis is 72000.
parameter twr_factor is 13.33.

local start_time to time:seconds.

clearscreen.
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

set warpmode to "physics".
set warp to 2.

print "Starting engines.".
stage.
wait until airspeed > 120.

print "Take off.".
set hdg to heading(dir, 10).
wait 5.
set gear to false.

wait until altitude > 250.

///// DRY MODE ASCENT /////

print "Beginning dry mode ascent.".


lock twr to ship:availablethrust / (ship:mass * constant:g0).
lock pitch to twr_factor * twr.
lock hdg to heading(dir, pitch).

wait until vang(hdg:vector, ship:facing:vector) < 1.

//set pid to pidloop(0.002, 0, 0).

set max_speed to airspeed.
set max_vertical_speed to verticalspeed.

when airspeed < max_speed and verticalspeed < max_vertical_speed then {
  print "Switching engines to wet mode at " + round(altitude) + "m.".
  set ag1 to true.
  local wet_mode to time:seconds.

  when time:seconds > wet_mode + 10 then {
    when airspeed < max_speed and verticalspeed < max_vertical_speed then {
      print "Activating rocket engines at " + round(altitude) + "m.".
      stage.
      lock pitch to min(21, 21 - (eta:apoapsis - 30) / 2).
      when altitude > 20000 then {
	set ag2 to true.
      }
    }
  }
}

lock prograde_angle to 90 - vang(ship:prograde:vector, up:vector).

until apoapsis > target_apoapsis or pitch <= prograde_angle {
  //set pid:setpoint to ???
  //set pitch to pitch + pid:update(time:seconds, ship:verticalspeed).
  print "Max speed: " + round(max_speed, 1) + " m/s   " at (1, 20).
  print "V speed:   " + round(verticalspeed, 1) + " m/s   " at (1, 21).
  print "Pitch:     " + round(pitch, 1) + "º   " at (1, 23).
  print "TWR:       " + round(twr, 3) at (1, 24).
  //print "Kp:        " + round(pid:kp, 4) + "     " at (1, 26).
  //print "Ki:        " + round(pid:ki, 4) + "     " at (1, 27).
  //print "Kd:        " + round(pid:kd, 4) + "     " at (1, 28).
  if max_speed <= airspeed {
    set max_speed to airspeed.
  } 
  if max_vertical_speed < verticalspeed {
    set max_vertical_speed to verticalspeed.
  }
  wait 0.01.
}

///// ROCKET ASCENT /////

print "Setting heading to prograde at " + round(altitude) + "m.".
lock steering to prograde.

wait until apoapsis > target_apoapsis.
lock throttle to 0.
wait 1.

/// Keep apoapsis suborbital.

until altitude >= 69000 {
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

run circularize.

panels on.

local lng to body:geopositionof(ship:position):lng.
print "Longitude after circularization: " + round(lng, 3) + "º.".
print "Time to orbit: " + round(time:seconds - start_time) + "s.".
