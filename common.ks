// === ORBITAL PREDICTION ===

// Minimize a function value
function minimize {
  // this is basically ternary search straight off Wikipedia
  parameter func, a, b.
  parameter epsilon is 0.2.
  parameter nmax is 1000.

  local n is 0.
  until n > nmax or abs(b - a) < epsilon {
    //print "A: " + round(a, 1) + " F(a): " + round(func(a), 1) + " B: " + round(b, 1) + " F(b): " + round(func(b),1).
    set m1 to a + (b - a) / 3.
    set m2 to b - (b - a) / 3.
    if func(m1) > func(m2) {
	set a to m1.
    } else {
	set b to m2.
    }
    set n to n + 1.
  }
  return (a + b) / 2.
}

// Rocket equation
function burn_duration {
  parameter delta_v.

  // determine engine ISP
  local eng_list to list().
  list engines in eng_list.

  for en_ in eng_list {
    // TODO: use en_:ignition instead
    if en_:vacuumisp > 0 {
	  set en to en_.
    }
  }

  // TBD: work through the Rocket Equation and confirm this math
  local thrust to ship:maxthrustat(0).
  local wMass to ship:mass.
  local dMass to wMass / (constant:E ^ (delta_v / (en:isp * constant:g0))).
  local flowRate to thrust / (en:isp * constant:g0).
  local burn_time to (wMass - dMass) / flowRate.
  return burn_time.
}

// vis-viva equation
function orbital_speed {
  parameter orbiter is ship.
  parameter altitude_ is orbiter:altitude.
  parameter apo is orbiter:apoapsis.
  parameter peri is orbiter:periapsis.

  local body_ to orbiter:body.
  local g to body_:mu.
  local r_ to body_:radius + altitude_.
  local a to (2 * body_:radius + apo + peri) / 2.
  return sqrt(g * ((2 / r_) - (1 / a))).
}

// Compute maneuver node from desired delta-V vector
function node_from_velocity {
  parameter dv.
  parameter t.

  // https://www.reddit.com/r/Kos/comments/701k7w/creating_maneuver_node_from_a_burn_vector/
  // Determine the prograde, normal, and radial components of the ship's velocity at time t.
  // As near as I can tell, this rotates the body-centered delta-v into the ship-centered axes
  // of the maneuver node.
  //
  local s_pro is velocityat(ship, t):orbit.
  // The normal axis is perpendicular to prograde and points away from the orbital body's center.
  local s_pos is positionat(ship, t) - body:position.
  local s_nrm is vcrs(s_pro,s_pos).
  // The radial axis is perpendicular to the prograde and normal axes.
  local s_rad is vcrs(s_nrm,s_pro).

  // Scale each burn axis by the desired amount in each direction
  local pro is vdot(dv,s_pro:normalized).
  local nrm is vdot(dv,s_nrm:normalized).
  local rad is vdot(dv,s_rad:normalized).

  return node(t, rad, nrm, pro).
}  

// Translated from https://physics.stackexchange.com/a/333897

// KNOWN WORKING
function eccentricity {
  parameter apo, peri.
  parameter r_b is ship:body:radius.
  local r_p to peri + r_b.
  local r_a to apo + r_b.
  return (r_a - r_p) / (r_a + r_p).
}

// KNOWN WORKING
function semi_major_axis {
  parameter apo, peri.
  parameter r_b is ship:body:radius.
  return r_b + (apo + peri) / 2.
}

function ecc_anomaly {
  parameter e, a, r_. // r_ is radius from body CoM
  local ratio to (a - r_) / (e * a).
  // clamping ratio needed because floating point rounding at the boundary leads to arccos returning NaN
  local ecc to arccos(min(max(ratio, -1), 1)). // return º
  // arccos() returns [0, 180º] but E is in range [0, 360º]
  // https://www.reddit.com/r/Kos/comments/4tm0wq/two_common_mistakes_people_make_when_calculating/
  if sin(ratio) > 0 {
    return ecc.
  } else {
    return -ecc.
  }
}

function mean_anomaly {
  parameter e, a, r_.
  local e_r to ecc_anomaly(e, a, r_).
  // the first term of m_r is in degrees, but the second is in radians, so convert
  local m_r to e_r - e * sin(e_r) * 180 / constant:pi.
  return m_r.
}

// KNOWN WORKING
function orbital_period {
  // Kepler's 3rd law
  parameter a.
  parameter mu is ship:body:mu.
  return 2 * constant:pi * sqrt(a ^ 3 / mu).
}
  
function time_from_periapsis {
  parameter alt_.
  parameter apo is ship:orbit:apoapsis.
  parameter peri is ship:orbit:periapsis.
  parameter r_ is ship:body:radius + alt_.
  local a to semi_major_axis(apo, peri).
  local e to eccentricity(apo, peri).
  local m_r to mean_anomaly(e, a, r_). // returns º
  // mean_anomaly is in range [0, 360º] so scale to orbital fraction
  return orbital_period(a) * m_r / 360.
}

function time_to_altitude {
  parameter target_alt.
  parameter start_alt is ship:altitude.
  parameter apo is ship:orbit:apoapsis.
  parameter peri is ship:orbit:periapsis.
  local t0 to time_from_periapsis(start_alt, apo, peri).
  local t1 to time_from_periapsis(target_alt, apo, peri).
  local dt to t1 - t0.
  if dt < 0 {
    set dt to dt + orbital_period(semi_major_axis(apo, peri)).
  }
  return dt.
}

// === OPERATIONS ===

function steering_aligned_to {
  parameter dv.
  return vang(dv:vector, ship:facing:vector) < 0.25.
}

function execute_node {
  parameter nd is nextnode.
  set dv to nd:deltav:mag.
  set initial_sas to sas.

  sas off.

  //print out node's basic parameters - ETA and deltaV
  print "Node in: " + round(nd:eta) + ", DeltaV: " + round(dv, 1).

  set burn_time to burn_duration(dv).

  print "Burn will take " + round(burn_time) + "s.".

  // FIXME: This code should track orientation to the node before/through warp
  set prepare_time to nd:time - burn_time / 2 - 60.
  if prepare_time > time:seconds {
   warpto(prepare_time).
  }
  wait until time:seconds > prepare_time.
  set warp to 0.

  print "Preparing to burn.".

  lock np to nd:deltav. //points to node, don't care about the roll direction.
  lock steering to np.

  //now we need to wait until the burn vector and ship's facing are aligned
  wait until steering_aligned_to(np).

  //the ship is facing the right direction, let's wait for our burn time
  wait until nd:eta <= (burn_time/2).

  //we only need to lock throttle once to a certain variable in the beginning of the loop, and adjust only the variable itself inside it
  set tset to 0.
  lock throttle to tset.

  print "Start burn.".

  set done to False.
  //initial deltav
  set dv0 to nd:deltav.
  until done
  {
      // recalculate current max_acceleration, as it changes while we burn through fuel
      set max_acc to ship:maxthrustat(0)/ship:mass.

      //throttle is 100% until there is less than 1 second of time left to burn
      //when there is less than 1 second - decrease the throttle linearly
      set tset to min(nd:deltav:mag/max_acc, 1).

      //here's the tricky part, we need to cut the throttle as soon as our nd:deltav and initial deltav start facing opposite directions
      //this check is done via checking the dot product of those 2 vectors
      if vdot(dv0, nd:deltav) < 0
      {
	  print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
	  lock throttle to 0.
	  break.
      }

      //we have very little left to burn, less then 0.1m/s
      if nd:deltav:mag < 0.1
      {
	  print "Finalizing burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
	  //we burn slowly until our node vector starts to drift significantly from initial vector
	  //this usually means we are on point
	  wait until vdot(dv0, nd:deltav) < 0.5.

	  lock throttle to 0.
	  print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
	  set done to True.
      }
  }
  unlock steering.
  unlock throttle.
  wait 1.

  // we no longer need the maneuver node, but leave it in case manual correction is needed
  remove nd.

  //set throttle to 0 just in case.
  SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
  set sas to initial_sas.
}

// === LANDING CALCULATION ===

function above_terrain {
  parameter t.
  local pos is positionat(ship, time:seconds + t).
  local b is ship:body.
  return b:altitudeof(pos) - b:geopositionof(pos):terrainheight.
}

function landing_time {
  local t is 0.
  local h is above_terrain(t).
  local dt is ship:orbit:eta:periapsis / 2.
  // print "DT: " + round(dt, 1) + " T:" + round(t, 1) + " H:" + round(h,1).
  until abs(dt) <= 0.1 {
    local t1 is t + dt.
    local h1 is above_terrain(t1).
    local slope is (h - h1) / dt.
    // print "DT: " + round(dt, 1) + " T:" + round(t1, 1) + " H:" + round(h1,1) + " M:" + round(slope, 3).
    set dt to h1 / slope.
    set h to h1.
    set t to t1.
  }
  // print "DT: " + round(dt, 1) + " T:" + round(t, 1) + " H:" + round(h,1).
  return t.
}

function landing_site {
   parameter t_land to landing_time().
   local pos to positionat(ship, time:seconds + t_land).
   return ship:body:geopositionof(pos).
}
