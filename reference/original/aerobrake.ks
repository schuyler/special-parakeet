run common.

clearscreen.
print "=== AEROBRAKING ===".

sas off.
parameter pitch is 90.
parameter peri_floor is 31000.
set start_apoapsis to orbit:apoapsis.
set start_periapsis to orbit:periapsis.

print "Periapsis is currently " + round(orbit:periapsis) + "m, floor is "
  + round(peri_floor) + "m.".
if orbit:periapsis < peri_floor {
  print "Floor is above the periapsis; the craft will not brace.".
}

// Attitude built from two vectors we name, so the axis the pitch turns about is
// one we chose: nose along the velocity vector, top toward local vertical. pitch
// is then the angle between the nose and the airflow, turning in the vertical
// plane -- 0 is nose-first, 90 is broadside with the belly into the wind. The
// argument is negated because r()'s first angle turns the nose down.
lock steering to lookdirup(prograde:vector, up:vector) * r(-pitch, 0, 0).

// A craft on rails holds whatever attitude it had when the warp began, so the
// turn has to both start and finish out of warp.
set warp to 0.
wait until kuniverse:timewarp:issettled.

// Five degrees rather than the 0.25 common.ks defaults to for a burn: drag does
// not care about five degrees, and a craft with little torque may never hold
// tighter than that broadside. The deadline is a stall guard, not a tuning
// knob -- a ship that cannot make the turn in two minutes will not make it.
local align_deadline is time:seconds + 120.
wait until steering_aligned_to(steering:vector, 5) or time:seconds > align_deadline.

// Angle from the nose to the airflow, and from the nose to straight up. With
// the nose pitched up out of the flow they read `pitch` and 90 - pitch.
print "Nose is " + round(vang(ship:facing:vector, prograde:vector)) + " deg off prograde, "
  + round(vang(ship:facing:vector, up:vector)) + " deg off vertical.".

if altitude > 70000 {
  print "Warping to atmosphere.".
  set warpmode to "rails".
  set warp to 3.
}

wait until altitude < 71000.
set warp to 0.

wait until altitude < 70000.
print "Aerobraking start.".

panels off.

set start_time to time:seconds.
set warpmode to "physics".
set warp to 2.

// Braking is done when the apoapsis is low enough; from there the craft rides
// nose-first.
when apoapsis < 150000 then {
  print "Apoapsis target reached; going nose-first.".
  lock steering to prograde.
}

// A periapsis still falling this deep means the pass is biting harder than
// planned, so shed drag the same way. The floor has to sit below the periapsis
// the pass starts from, or it is met on entry and the craft never braces at all.
when periapsis < peri_floor then {
  print "Periapsis floor crossed; going nose-first.".
  lock steering to prograde.
}

wait until altitude > 70000 or altitude < 20000.

set elapsed to time:seconds - start_time.
print "Aerobraking took " + round(elapsed) + "s.".
print "Apoapsis was lowered by " + round(start_apoapsis - orbit:apoapsis) + "m.".
print "Periapsis was lowered by " + round(start_periapsis - orbit:periapsis) + "m.".

set warp to 0.
panels on.
unlock steering.
