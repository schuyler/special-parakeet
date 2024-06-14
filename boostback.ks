clearscreen.
print "=== BOOST BACK ===".


// there's a better way to compute this, I think
lock vel to ship:velocity:surface.
lock upVector to (ship:body:position - ship:position):normalized.
lock verticalVelocity to vdot(vel, upVector) * upVector.
lock horizontalVelocity to vel - verticalVelocity.

sas off.
lock steering to -horizontalVelocity:normalized * r(0, 0, 0).

wait until vang(steering, ship:facing:vector) < 0.25.
lock throttle to 1 - 1 / max(horizontalVelocity:mag, 1)).

when vang(steering, ship:facing:vector) > 5 {
  lock throttle to 0.
  lock steering to horizontalVelocity:normalized * r(0,0,0).
  when vang(steering, ship:facing:vector) < 0.25 {
    lock throttle to 1.
  }
}

when ship:maxthrust <= 0 then {
  lock throttle to 0.
}

until throttle = 0 {
  print "Ground speed: " + round(groundspeed, 1):tostring:padleft(6) + " m/s." at (1, 5).
}

// wait until parachutes are deployed, actually
wait until ship:altitude < 15000.
unlock steering.
