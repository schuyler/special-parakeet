clearscreen.
print "=== BOOST BACK ===".


// there's a better way to compute this, I think
lock vel to ship:velocity:surface.
lock upVector to ship:up:vector. //(ship:body:position - ship:position):normalized.
lock verticalVelocity to vdot(vel, upVector) * upVector.
lock horizontalVelocity to vel - verticalVelocity.

when altitude > 60000 then { //or verticalspeed < 0 then {
  sas off.
   //stage.

  if ship:maxthrust > 0 {
    print "Reversing direction.".
    lock steering to heading(270,0). //-horizontalVelocity:normalized * r(0, 0, 0).
    when vang(steering:vector, ship:facing:vector) < 1 then {
      print "Starting boostback.".
      lock throttle to 1.
    }
  }

  when ship:maxthrust <= 0 then {
    print "Pitching up for re-entry.".
    lock throttle to 0.
    lock hdg to vang(-horizontalVelocity, ship:north:vector).
    lock steering to heading(hdg, 30).
    when altitude < 12000 then {
      chutes on.
    }
  }

  when chutes then {
    print "Chutes engaged.".
    unlock steering.
  }
}

until altitude < 100 {
  print "Ground speed: " + round(groundspeed, 1):tostring:padleft(6) + " m/s." at (1, 5).
}

