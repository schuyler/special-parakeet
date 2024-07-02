parameter target_apoapsis is 72000.
parameter ascent_pitch is 15.

local start_time to time:seconds.
local eng_list is list().
list engines in eng_list.

run "aero".

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
//set warp to 2.

print "Starting engines.".
set brakes to false.
stage.
wait until airspeed > 120.

print "Take off.".
set hdg to heading(dir, 15).

wait until altitude > 100.
set gear to false.

///// DRY MODE ASCENT /////

set target_pitch to 15.
set pitch to 15.
lock hdg to heading(dir, pitch).

local g to body:mu / (body:radius ^ 2).
lock twr to ship:availablethrust / (ship:mass * g).
lock prograde_angle to 90 - vang(ship:prograde:vector, up:vector).
lock angle_of_ascent to 90 - vang(ship:velocity:surface, up:vector).

set max_speed to airspeed.
set max_vertical_speed to verticalspeed.
set max_thrust to ship:availablethrust.
set phase to 0.

print "Beginning dry mode ascent at " + round(pitch,1) + "º with TWR " + round(twr, 2) + ".".

when vang(hdg:vector, ship:facing:vector) < 1 then {
  set max_speed to airspeed.
  set max_vertical_speed to verticalspeed.

  //when abs(target_pitch - pitch) > 0.1 then {
  //  set pitch to pitch + (target_pitch - pitch) * 0.5.
  //  return true.
  //}

  //when phase = 0 and angle_of_ascent > 0.5 then {
  //  set target_pitch to target_pitch - 0.01.
  //  return phase = 0.
  //}

  //when phase = 0 and angle_of_ascent <= 0.5 then {
  //  set target_pitch to target_pitch + 0.01.
  //  return phase = 0.
  //}

  when phase = 0 and airspeed > 250 then { //mach_number() > 1 then {
    set pitch to ascent_pitch.
    set phase to 1.
    when angle_of_ascent > 5 then {
      set phase to 2.
      wait 2.
      when angle_of_ascent <= 5 or airspeed < max_speed then {
	print "Activating rocket engines at " + round(altitude) + "m.".
	stage.
	set phase to 3.
	when pitch > angle_of_ascent and angle_of_ascent > 0 then {
	  set pitch to pitch - min(altitude / target_apoapsis, 1) * 0.01.
	  return altitude < level_off.
	}
	when altitude > level_off then {
	  print "Setting heading to prograde at " + round(altitude) + "m.".
	  set phase to 4.
	  lock steering to prograde.
	}
	when throttle = 1 and apoapsis >= target_apoapsis * 1.01 then {
	  lock throttle to 0.
	  return altitude < 70000.
	}
	when throttle = 0 and apoapsis < target_apoapsis * 0.99 then {
	  print "Burning to keep apoapsis above atmospheric boundary.".
	  lock throttle to 1.
	  return altitude < 70000.
	}
      }
    }
  }
}

until altitude > 70000 {
  print "Phase:     " + phase:tostring:padleft(6)  at (1, 16).
  print "Ascent:    " + round(angle_of_ascent(), 3):tostring:padleft(6) + "º"  at (1, 17).
  print "Mach:      " + round(mach_number(), 3):tostring:padleft(6)  at (1, 19).
  print "Max speed: " + round(max_speed, 1):tostring:padleft(6) + " m/s   " at (1, 20).
  print "V speed:   " + round(verticalspeed, 1):tostring:padleft(6) + " m/s   " at (1, 21).
  print "Pitch:     " + round(pitch, 1):tostring:padleft(6) + "º   " at (1, 23).
  print "TWR:       " + round(twr, 3):tostring:padleft(6) at (1, 24).
  print "Apoapsis:  " + round(apoapsis):tostring:padleft(6) + " m" at (1,25).
  local p to body:atm:altitudepressure(altitude).
  local n to 27.
  for en in eng_list {
    print en:name:padright(18) + ": " + round(en:availablethrust):tostring:padleft(6) + " kN   ISP: " + round(en:ispat(p)):tostring:padleft(4) at (1,n).
    set n to n + 1.
    if en:flameout {
      en:shutdown().
    }
  }
  if max_speed <= airspeed {
    set max_speed to airspeed.
  } 
  if max_vertical_speed < verticalspeed {
    set max_vertical_speed to verticalspeed.
  }
  if max_thrust < ship:availablethrust {
    set max_thrust to ship:availablethrust.
  }
  wait 0.1.
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
