clearscreen.
print "== CIRCULARIZE ==".

///// CIRCULARIZE /////

// Start by reaching orbit so we know where apoapsis is.

if (altitude < 70000) {
  print "Ascending to 70 km.".
  set warp to 2.
  wait until altitude > 70000.
}
set warp to 0.
wait until kuniverse:timewarp:issettled.

// Compute dV

set grav_param to kerbin:mu.
set r_apo to ship:apoapsis + kerbin:radius. // 600k = radius of Kerbin?

//Vis-viva equation to give speed we'll have at apoapsis.
set v_apo to SQRT(grav_param * ((2 / r_apo) - (1 / ship:orbit:semimajoraxis))).

//Vis-viva equation to calculate speed we want at apoapsis for a circular orbit. 
//For a circular orbit, desired SMA = radius of apoapsis.
set v_apo_wanted to SQRT(grav_param * ((2 / r_apo) - (1 / r_apo))). 
set dv to v_apo_wanted - v_apo.

print round(dv, 1) + " m/s needed.".

// determine engine ISP

list engines in eng_list.

for en_ in eng_list {
  if en_:vacuumisp > 0 {
	set en to en_.
  }
}

// determine burn time

set thrust to ship:maxthrustat(0).
set wMass to ship:mass.
set dMass to wMass / (constant:E ^ (dv / (en:isp * constant:g0))).
set flowRate to thrust / (en:isp * constant:g0).
set burn_time to (wMass - dMass) / flowRate.

print "Burn will take " + round(burn_time, 3) + "s.".

// Leave enough time to point to prograde.
// FIXME: This code doesn't leave enough time to reach prograde if warp is too high.
set delay to eta:apoapsis - burn_time / 2 - 60.
if delay > 0 {
  warpto(time:seconds + delay).
}
set warp to 0.
wait until kuniverse:timewarp:issettled.

// TODO: create maneuver node and execute it
lock steering to prograde.
set delay to (eta:apoapsis - burn_time / 2).
if delay > 0 {
  print "Performing burn in " + delay + "s.".
  wait delay.
}

lock throttle to 1.
wait burn_time. 
lock throttle to 0.

print "Burn complete.".

///// FINISH /////

unlock steering.
unlock throttle.
wait 1.
