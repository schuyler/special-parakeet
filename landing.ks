@lazyglobal off.
parameter landing_speed is 5.
parameter burn_margin is 1.01.
parameter warp_margin is 30.

clearscreen.
print "=== POWERED LANDING ===".
run "common".

function time_to_surface {
  parameter t.
  local pos is positionat(ship, t).
  local alt_ is (pos - body:position):mag - body:radius.
  local h is alt_ - body:geopositionof(pos):terrainheight.
  local surface_v is velocityat(ship, t):surface.
  local up_v to (pos - body:position):normalized.
  local v_ to vdot(surface_v, up_v) * up_v.
  local v_mag to v_:mag.
  local g_ is body:mu / (body:distance ^ 2).
  return (-v_mag + sqrt(v_mag ^ 2 + 2 * g_ * h)) / g_.
}

function perform_landing {
  parameter landing_speed.
  parameter burn_margin.
  parameter warp_margin.

  if ship:availablethrust = 0 {
    print "No engines available.".
    return.
  }

  until not hasnode {
    remove nextnode.
    wait 1.
  }

  sas off.
  wait until verticalspeed < 0.
  lock steering to ship:srfretrograde * r(0,0,0).

  local state to "Initial Free Fall".
  local landing_burn_duration to 0.
  local time_to_periapsis to 0.
  local burn_start to 0.
  local g to 0.

  lock g to body:mu / (body:distance ^ 2).
  lock landing_burn_duration to burn_duration(ship:velocity:surface:mag + sqrt(2 * alt:radar * g)).
  lock burn_start to time_to_surface(time:seconds + landing_burn_duration / 2 * burn_margin).
  lock time_to_periapsis to orbit:eta:periapsis - landing_burn_duration / 2.

  if burn_start > warp_margin {
    set warp to 3.
  }
  
  when burn_start < warp_margin or time_to_periapsis < warp_margin then {
    set warp to 0.
  }

  when burn_start <= 0 or time_to_periapsis <= 0 then {
    set state to "Braking Burn".
    lock throttle to 1.
    gear on.

    //when vang(ship:up:vector, ship:srfretrograde:vector) <= 89.0 then {
    when groundspeed <= landing_speed then {
      set state to "Free Fall".
      lock throttle to 0.
      
      when burn_start <= 0 then {
	set state to "Final Descent".
	lock throttle to 1.

	when airspeed <= landing_speed then {
	  set state to "Landing Burn".
	  local twr_ is ship:availablethrust / (ship:mass * g).
	  lock steering to ship:up * r(0,0,0).
	  lock throttle to (airspeed / landing_speed) * 0.99 / twr_.
	}
      }
    }

    when alt:radar <= 5 then {
      set state to "Landed".
      lock throttle to 0.
    }
  }

  until alt:radar <= 2 {
    print "State        : " + state + "            " at (1,22).
    print "Suicide burn : T-" + round(burn_start, 1) + " s " at (1,23).
    print "Periapsis    : T-" + round(time_to_periapsis, 1) + " s " at (1,24).
    print "Burn duration: " + round(landing_burn_duration, 1) + " s " at (1,25).
    print "Vspd         : " + round(verticalspeed, 1) + " m/s " at (1,26).
    print "Speed        : " + round(airspeed, 1) + " m/s " at (1,27).
    print "Radar        : " + round(alt:radar) + " m " at (1,28).
    wait 0.25.
  }

  lock steering to ship:up.
  lock throttle to 0.
  wait 5.

  unlock throttle.
  unlock steering.
  sas on.  
}

perform_landing(landing_speed, burn_margin, warp_margin).
