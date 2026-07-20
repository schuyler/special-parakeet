clearscreen.
print "== REENTRY ==".

if periapsis > 70000 {
  print "Re-entry is not expected on this orbit.".
  exit.
}

run common.

print "Orienting to prograde.".
set warp to 0.
sas off.
set pitch to 20.

// Compass azimuth, degrees clockwise from north, of a vector v_ projected
// onto the local horizontal plane. north and up are the local tangent-north
// and radial-out unit vectors; their cross product points east. arctan2 of
// the east and north components of v_ is its heading.
function compass_for {
  parameter v_.
  local east is vcrs(ship:up:vector, ship:north:vector).
  local az is arctan2(vdot(east, v_), vdot(ship:north:vector, v_)).
  if az < 0 { return az + 360. }
  return az.
}

// Hold the nose `pitch` degrees above the horizon along the ground track,
// wings level (roll 0, the wing axis parallel to the ground). heading()
// builds this from the surface-velocity azimuth and the horizon, so it is
// independent of orbit inclination. The old srfprograde + r(...) rode
// srfprograde's own roll reference, which tumbles with inclination and
// gimbal-locks near polar orbits.
lock hdg to heading(compass_for(srfprograde:vector), pitch, 0).
lock steering to hdg.
wait until steering_aligned_to(hdg:vector).

print "Disabling rocket engines and re-enabling jet engines.".
local en_list is list().
list engines in en_list.
for en in en_list {
  if not en:ignition {
    en:activate().
  }
  if en:ignition and en:availablethrust > 0 {
    en:shutdown().
  }
}


set warp to 2.
print "Warping to atmospheric re-entry.".

wait until altitude < 72000.
panels off.

set warpmode to "physics".
set warp to 0.

rcs on.
wait until steering_aligned_to(hdg:vector).

set warp to 2.

print "Waiting for aerodynamic control.".
when airspeed < 800 then {
  print "Flight controls unlocked.".
  set warp to 0.
  unlock steering.
  sas on.
}

until airspeed < 100 {
   if periapsis <= 0 {
     local t_land to landing_time().
     local pos to positionat(ship, time:seconds + t_land).
     local site to ship:body:geopositionof(pos).
     print "Landing in " + floor(t_land / 60) + ":" + floor(mod(t_land, 60)) + " at (" + round(site:lat,3) + "º, " + round(site:lng, 3) + "º)." at (1,20).
   }
   //if site:lng > -72 {
   //  set pitch to min(pitch + 1, 20).
   //} else if site:lng < -77 {
   //  set pitch to max(pitch - 1, 0).
   //}
   print "Air pressure: " + round(ship:body:atm:altitudepressure(ship:altitude),4) at (1,19).
}


