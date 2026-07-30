// minimize, minimize_scan, find_zero, and bisect come from
// core/optimize.ks — one home, no twin copies to keep in sync. Located
// relative to this file, kepler.ks's idiom, so the caller's current
// directory is not disturbed.
runoncepath(scriptPath():parent:parent:combine("core", "optimize.ks")).

// safe_alt and safe_radius — the per-body minimum-altitude policy —
// come from core/safety.ks, same idiom.
runoncepath(scriptPath():parent:parent:combine("core", "safety.ks")).

// The targeting pipeline's shared thresholds (plan_*) and
// default_max_wait come from core/planning.ks, same idiom.
runoncepath(scriptPath():parent:parent:combine("core", "planning.ks")).

// === ORBITAL PREDICTION ===

// Find the current engine ISP
function engine_isp {
  local eng_list to list().
  list engines in eng_list.

  for en_ in eng_list {
    if en_:availablethrust > 0 {
      return en_:isp.
    }
  }
  return 0.
}

// Find a root of func on [a, b] by bisection. Makes no assumptions about
// which way the function crosses; it only requires that func(a) and
// func(b) have opposite signs.
function find_root {
  parameter func, a, b.
  parameter epsilon is 0.2.
  parameter nmax is 1000.

  local fa is func(a).
  local n is 0.
  until n > nmax or abs(b - a) < epsilon {
    local mid is (a + b) / 2.
    local fmid is func(mid).
    if (fa < 0) = (fmid < 0) {
      // same sign: the root is in the right half
      set a to mid.
      set fa to fmid.
    } else {
      set b to mid.
    }
    set n to n + 1.
  }
  return (a + b) / 2.
}

// Rocket equation
function burn_duration {
  parameter delta_v.

  local isp is engine_isp().
  // TBD: work through the Rocket Equation and confirm this math
  local thrust to ship:availablethrust.
  local wMass to ship:mass.
  local dMass to wMass / (constant:E ^ (delta_v / (isp * constant:g0))).
  local flowRate to thrust / (isp * constant:g0).
  local burn_time to (wMass - dMass) / flowRate.
  return burn_time.
}

function orbital_height {
  parameter orbit_ is ship:orbit.
  return orbit_:body:altitudeof(orbit_:position).
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

// === OPERATIONS ===

function steering_aligned_to {
  parameter dv.
  parameter angle is 0.25.
  return vang(dv, ship:facing:vector) < angle.
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

function time_to_surface {
  parameter t is time:seconds.
  local pos is positionat(ship, t).
  local alt_ is (pos - body:position):mag - body:radius.
  local h is max(0, alt_ - body:geopositionof(pos):terrainheight).
  local surface_v is velocityat(ship, t):surface.
  local up_v to (pos - body:position):normalized.
  local v_ to vdot(surface_v, up_v).
  //print "v_: " + v_ + "  verticalSpeed: " + verticalSpeed + "   " at (1,32).
  local g_ is body:mu / ((body:radius + alt_) ^ 2).
  return (-v_ + sqrt(2 * g_ * h)) / g_.
}

function above_terrain {
  parameter t.
  local pos is positionat(ship, time:seconds + t).
  local b is ship:body.
  return b:altitudeof(pos) - b:geopositionof(pos):terrainheight.
}

// Seconds from now until the drag-free trajectory meets the terrain: 0 if
// the ship is already at or below it, -1 if it does not meet it before
// periapsis. above_terrain falls monotonically from now to periapsis
// whenever periapsis is underground, so those two times bracket exactly one
// crossing and bisection converges on it from both sides. A secant or
// Newton step cannot be used here: the height profile is symmetric about
// periapsis, so two samples straddling it have equal heights and a zero
// slope to divide by. The two ends are tested here rather than left to
// bisect, whose bracketing-failure branch prints four lines every call.
function landing_time {
  local t_pe is ship:orbit:eta:periapsis.
  if above_terrain(0) <= 0 { return 0. }
  if above_terrain(t_pe) >= 0 { return -1. }
  return bisect(above_terrain@, 0, t_pe, 0.1).
}

function landing_site {
   parameter t_land to landing_time().
   local pos to positionat(ship, time:seconds + t_land).
   return ship:body:geopositionof(pos).
}
