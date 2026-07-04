// clearscreen.
print "".
print "== CIRCULARIZE ==".

///// CIRCULARIZE /////

// Compute dV
set r_apo to max(ship:apoapsis, ship:periapsis) + body:radius. // 600k = radius of Kerbin?

//Vis-viva equation to give speed we'll have at apoapsis.
set v_apo to sqrt(body:mu * ((2 / r_apo) - (1 / ship:orbit:semimajoraxis))).

//Vis-viva equation to calculate speed we want at apoapsis for a circular orbit. 
//For a circular orbit, desired SMA = radius of apoapsis.
set v_apo_wanted to sqrt(body:mu * ((2 / r_apo) - (1 / r_apo))). 
set dv to v_apo_wanted - v_apo.

print round(dv, 1) + " m/s needed.".

// determine engine ISP
set apsis to min(eta:apoapsis, eta:periapsis).
set nd to node(time:seconds + apsis, 0, 0, dv).
add nd.
