clearscreen.
parameter level_off is 40000.
print("= LAUNCH = ").

sas off.
lock pitch to round((level_off - ship:altitude) / (level_off / 90), 1).
lock steering to heading(90, pitch).
lock throttle to 1.
stage.

set warpmode to "physics".
set warp to 2.

when ship:altitude > 45000 then {
  lock steering to ship:prograde.
}

when ship:apoapsis > 75000 then {
  lock throttle to 0.
  lock steering to ship:prograde.
}

until ship:altitude > 70000 {
  if ship:apoapsis < 75000 {
    lock throttle to 1.
  } else {
    lock throttle to 0.
  }
  wait 0.1.
}

set warp to 0.
wait until kuniverse:timewarp:issettled.

unlock throttle.
unlock steering.

panels on.
antenna on.

run circularize.
run next.
