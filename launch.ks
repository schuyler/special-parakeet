clearscreen.
parameter target_apoapsis is 72000.
parameter level_off is 45000.
parameter pitch_factor is 1/500.
print("= LAUNCH = ").

set g to body:mu / (body:radius ^ 2).
local twr is ship:availablethrust / (ship:mass * g).

sas off.
lock prograde_angle to 90 - vang(ship:srfprograde:vector, up:vector).
lock pitch to 90.
lock steering to heading(90, pitch).
lock throttle to 1.
stage.

set warpmode to "physics".
set warp to 2.

local tick to time:seconds.
when ship:altitude > 1500 then {
  //local time_to_level to (level_off - altitude) / max(verticalspeed, 1).
  //local deg_per_sec to prograde_angle / max(time_to_level, 1).
  //local delta_pitch to (time:seconds - tick) * deg_per_sec.
  //print "time_to_level: " + round(time_to_level, 3) at (1,12).
  //print "deg_per_sec: " + round(deg_per_sec, 3) at (1,13).
  //print "delta_pitch: " + round(delta_pitch, 3) at (1,14).
  lock pitch to max(90 * (1 - sqrt(apoapsis / target_apoapsis)), 10).
  return altitude < level_off.
}

when ship:altitude > level_off then {
  lock steering to ship:prograde.
}

when ship:availablethrust = 0 then {
  stage.
  return true.
}

until ship:altitude > 70000 {
  if ship:apoapsis > 75000 {
    lock throttle to 0.
  } else {
    lock throttle to 1.
  }
  print "Pitch: " + round(pitch, 1) at (1,5).
  print "Prograde: " + round(prograde_angle, 1) at (1,6).
  print "Apoapsis: " + round(apoapsis, 1) at (1,7).
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
