clearscreen.
parameter target_apoapsis is 72000.
parameter level_off is 40000.
parameter roll is 270.

set min_pitch to 5.
set atm_exp to 1/2.

//parameter orbital_speed is 2150.
print("= LAUNCH = ").

//set g to body:mu / (body:radius ^ 2).
//local twr is ship:availablethrust / (ship:mass * g).

sas off.
lock prograde_angle to 90 - vang(ship:srfprograde:vector, up:vector).
lock air_pressure to body:atm:altitudepressure(ship:altitude).
lock pitch to 90.
lock steering to heading(90, pitch, roll).
lock throttle to 1.
stage.

set warpmode to "physics".
//set warp to 2.

//local tick to time:seconds.
when ship:altitude > 1500 then {
  lock pitch to max(90 * air_pressure ^ atm_exp, min_pitch).
  when ship:altitude > level_off then {
    lock steering to ship:prograde.
  }
}


when ship:availablethrust = 0 then {
  stage.
  return true.
}

until ship:altitude > 70000 {
  if ship:apoapsis > target_apoapsis {
    lock throttle to 0.
  } else {
    lock throttle to 1.
  }
  print "Pitch: " + round(pitch, 1) at (1,5).
  print "Prograde: " + round(prograde_angle, 1) at (1,6).
  print "Apoapsis: " + round(apoapsis, 1) at (1,7).
  print "Pressure: " + round(air_pressure, 3) at (1, 8).
  wait 0.25.
}

set warp to 0.
wait until kuniverse:timewarp:issettled.

unlock throttle.
unlock steering.

panels on.
antenna on.

run circularize.
run next.
