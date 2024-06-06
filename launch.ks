clearscreen.
parameter level_off is 45000.
print("= LAUNCH = ").

sas off.
lock pitch to 90 * max((level_off - ship:altitude) / level_off, 0).
lock steering to heading(90, pitch).
lock throttle to 1.
stage.

set warpmode to "physics".
set warp to 2.

when ship:altitude > 45000 then {
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
  print "Apoapsis: " + round(apoapsis, 1) at (1,6).
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
