clearscreen.
print "=== BOOST BACK ===".


// there's a better way to compute this, I think
lock vel to ship:velocity:surface.
lock upVector to ship:up. //(ship:body:position - ship:position):normalized.
lock verticalVelocity to vdot(vel, upVector) * upVector.
lock horizontalVelocity to vel - verticalVelocity.

sas off.

if ship:maxthrust > 0 {
  lock steering to heading(270,0). //-horizontalVelocity:normalized * r(0, 0, 0).
  wait until vang(steering:vector, ship:facing:vector) < 0.25.
  lock throttle to 1.
}

when ship:maxthrust <= 0 then {
  lock throttle to 0.
  lock hdg to vang(horizontalVelocity, ship:north).
  lock steering to heading(hdg, 30).
  when altitude < 12000 {
    chutes on.
  }
}

when chutes then {
  unlock steering.
}

until altitude < 100 {
  print "Ground speed: " + round(groundspeed, 1):tostring:padleft(6) + " m/s." at (1, 5).
}

