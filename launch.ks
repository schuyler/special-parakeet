clearscreen.
parameter level_off is 25000.
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
}

wait until ship:altitude > 70000.
unlock throttle.
unlock steering.
panels on.
antenna on.

set warp to 0.
run circularize.
