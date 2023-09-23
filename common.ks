function speed_of_sound {
  parameter alt_ is altitude.
  parameter body_ is body.

  set atm to body_:atm.
  set p to atm:altitudepressure(alt_) * constant:atmtokpa.
  set t to atm:altitudetemperature(alt_).
  set rho to p * atm:molarmass / (constant:idealgas * t).
  return sqrt(atm:adiabaticindex * p / rho).
}

function mach_number {
  parameter speed is airspeed.
  parameter alt_ is altitude.
  parameter body_ is body.
  return speed / speed_of_sound(alt_, body_).
}

function burn_time {
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

  // determine burn time
  // TBD: work through the Rocket Equation and confirm this math
  local thrust to ship:maxthrustat(0).
  local wMass to ship:mass.
  local dMass to wMass / (constant:E ^ (delta_v / (en:isp * constant:g0))).
  local flowRate to thrust / (en:isp * constant:g0).
  local burn_time to (wMass - dMass) / flowRate.
  return burn_time.
}

function orbital_speed {
  // it's the good old vis-viva equation
  parameter orbiter.
  parameter altitude_ is orbiter:altitude.
  parameter apo is orbiter:apoapsis.
  parameter peri is orbiter:periapsis.

  local body_ to orbiter:body.
  local g to body_:mu.
  local r_ to body_:radius + altitude_.
  local a to (2 * body_:radius + apo + peri) / 2.
  return sqrt(g * ((2 / r_) - (1 / a))).
}

function execute_node {
  parameter nd is nextnode.
  set dv to nd:deltav:mag.
  set initial_sas to sas.

  sas off.

  //print out node's basic parameters - ETA and deltaV
  print "Node in: " + round(nd:eta) + ", DeltaV: " + round(dv, 1).

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

  print "Burn will take " + round(burn_time) + "s.".

  set prepare_time to nd:time - burn_time / 2 - 60.
  if prepare_time > time:seconds {
    warpto(prepare_time).
  }

  print "Preparing to burn.".

  lock np to nd:deltav. //points to node, don't care about the roll direction.
  lock steering to np.

  //now we need to wait until the burn vector and ship's facing are aligned
  wait until vang(np, ship:facing:vector) < 0.25.

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
