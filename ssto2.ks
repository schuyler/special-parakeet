parameter target_apoapsis is 72000.
parameter twr_factor is 13.33.
parameter rocket_ascent is 23.

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
set warp to 2.

print "Starting engines.".
set brakes to false.
stage.
wait until airspeed > 120.

print "Take off.".
set hdg to heading(dir, 15).

wait until altitude > 250.
set gear to false.

///// DRY MODE ASCENT /////

lock pitch to twr_factor * twr.
lock hdg to heading(dir, pitch).

local g to body:mu / (body:radius ^ 2).
lock twr to ship:availablethrust / (ship:mass * g).
lock prograde_angle to 90 - vang(ship:prograde:vector, up:vector).
lock angle_of_ascent to 90 - vang(ship:velocity:surface, up:vector).

set max_speed to airspeed.
set max_vertical_speed to verticalspeed.
set max_thrust to ship:availablethrust.

print "Beginning dry mode ascent at " + round(pitch,1) + "º with TWR " + round(twr, 2) + ".".

when altitude > 2000 and vang(hdg:vector, ship:facing:vector) < 1 and angle_of_ascent > 5 then {
  set max_speed to airspeed.
  set max_vertical_speed to verticalspeed.

  when (airspeed < max_speed and verticalspeed < max_vertical_speed) or angle_of_ascent < 5 then {
    print "Switching engines to wet mode at " + round(altitude) + "m.".
    // TODO: Change this to iterate through the engines and flip any with modes
    set ag1 to true.
    local wet_mode to time:seconds.

    when time:seconds > wet_mode + 1 and angle_of_ascent < 5 then {
      print "Activating rocket engines at " + round(altitude) + "m.".
      stage.
      set rocket_start to time:seconds.
      if mach_number() < 3 {
	lock pitch to 10. // + arcsin(1/twr).
      }
      when mach_number() > 3 and ship:availablethrust <= max_thrust then {
	// Robin: 1340m/s on orbit (25º, 1 - altitude / 40000)
	//        1336m/s (21º, 1 - altitude / 40000)
	//lock pitch to max(prograde_angle, rocket_ascent * (1 - altitude / 40000)).
	lock pitch to max(prograde_angle, rocket_ascent * (1 - eta:apoapsis / 90)).
      }
    }
  }
}

until apoapsis > target_apoapsis {
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

///// ROCKET ASCENT /////

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
