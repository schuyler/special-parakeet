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

lock hdg to srfprograde + r(0, pitch, 0).
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

wait until altitude < 70000.
set warpmode to "physics".
set warp to 2.

print "Waiting for aerodynamic control.".
until airspeed < 1200 {
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
   wait 5.
}

print "Flight controls unlocked.".
set warp to 0.
unlock steering.
sas on.

