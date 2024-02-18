clearscreen.
print "=== LANDING ===".
run "common".

function above_surface {
  parameter t.
  local pos is positionat(ship, t).
  local msl_alt is (pos - body:position):mag - body:radius.
  local geo is body:geopositionof(pos).
  return msl_alt - geo:terrainheight.
}

function _time_to_surface {
  parameter v_ is ship:velocity:surface:mag.
  parameter h is alt:radar.
  local avg_r to (body:distance + (body:distance - h)) / 2.
  local g is body:mu / (avg_r ^ 2).
  return (-v_ + sqrt(v_ ^ 2 + 2 * g * h)) / g.
}

function time_to_altitude {
  parameter target is 0.
  local start is time:seconds.
  local t is 0.
  local h is ship:altitude.
  until h <= target {
    set h to above_surface(start + t).
    set t to t + h/5000.
  }
  return t - h/5000.
}

function simple_burn_time {
  parameter delta_v.

  local en to ship:engines[0].
  local thrust to ship:availablethrust.
  local wMass to ship:mass.
  local dMass to wMass / (constant:E ^ (delta_v / (en:isp * constant:g0))).
  local flowRate to thrust / (en:isp * constant:g0).
  local burn_time to (wMass - dMass) / flowRate.
  //local avgMass to (wMass + dMass) / 2.
  //local avgAcc to thrust / avgMass.
  //local burn_time to delta_v / avgAcc.
  return burn_time.
}

function burn_start_for {
  parameter target_alt is 0.
  parameter safety_margin is 1.05.
  local target_time to time:seconds + time_to_altitude(target_alt).
  local dv to velocityat(ship, target_time):surface:mag.
  local burn_time to simple_burn_time(dv).
  return target_time - (burn_time * safety_margin) / 2.
}

sas off.

// State 1: Initial free fall
// State 2: Braking burn
// State 3: Final free fall
// State 4: Landing burn
// State 5: Powered descent
// State 5: Landed

// Start in [Initial Free Fall].
//   - Set burn_time for est velocity at 100m above terrain.
//   - This leaves some margin for errors in landing estimate
// Switch to [Braking Burn] when t_land < (burn_time * safety_margin).
//   - Set throttle to 1.
//   - Bleed off most of the remaining orbital speed. Again with the safety margin.
// Switch to [Final Free Fall] when airspeed < 5 + (alt:radar * g).
//   - Set throttle to 0.
//   - Set burn_time for est velocity at 10m above terrain.
//   - Theoretially this sets us up to be moving at 5 m/s at 10m alt
// Switch to [Landing Burn] when t_land < burn_time.
//   - Set throttle to 1.
//   - Bleed off all remaining speed but the last 5 m/s
// Switch to [Final Descent] when airspeed < 5.
//   - Set throttle to (0.95 / twr).
//   - Reduce throttle to stay at ~5 m/s the rest of the way down.
// Switch to [Landed] when alt:radar stops decreasing.
//   - Set throttle to 0.
//   - Ideally use https://ksp-kos.github.io/KOS/structures/vessels/bounds.html to determine contact

set safety_margin to 1.05.
set braking_finish to 50. // meters
set vertical_descent to 15. // meters
set landing_speed to 7.

local state to "Initial Free Fall".

set burn_start to burn_start_for(braking_finish).

when time:seconds < burn_start - 60 then {
  set warp to 3.
}

when verticalspeed < -1 then {
  lock steering to ship:srfretrograde.
}

when time:seconds > burn_start - 45 then {
  set warp to 0.
}

when time:seconds > burn_start - 10 then {
  set burn_start to burn_start_for(braking_finish).
}

when time:seconds >= burn_start then {
  set state to "Braking Burn".
  lock throttle to 1.

  local g is body:mu / (body:distance ^ 2).
  local twr is ship:availablethrust / (ship:mass * g).
  local max_acc is ship:availablethrust / ship:mass. 

  lock target_speed to landing_speed + sqrt(2 * (ship:availablethrust - g) * alt:radar).
  when airspeed <= target_speed then {
    set state to "Final Free Fall".
    lock throttle to 0.

    set burn_start to burn_start_for(braking_finish).
  
    when time:seconds >= burn_start or alt:radar <= vertical_descent then {
      set state to "Landing Burn".
      lock throttle to 1.0.

      when airspeed <= landing_speed then {
	set state to "Final Descent".
        set twr to ship:availablethrust / (ship:mass * g).
	lock throttle to 1 / twr.
      }

      when verticalspeed > -0.1 or alt:radar <= 2 then {
	set state to "Landed".
	lock throttle to 0.
      }
    }
  }
}

until alt:radar <= 2 {
  print "State: " + state + "                   " at (1,22).
  print "Burn starts in " + round(burn_start - time:seconds, 1) + " s. " at (1,23).
  //print "Burn time: " + round(burn_time - max(time:seconds - burn_start, 0), 1) + " s." at (1,23).
  //print "Landing in " + round(t_land - time:seconds, 1) + " s." at (1,24).
  print "Vspd: " + round(verticalspeed, 1) + " m/s." at (1,25).
  print "Speed: " + round(airspeed, 1) + " m/s." at (1,26).
  //print "Off axis: " + round(vang(ship:facing:vector, ship:srfretrograde:vector), 1) at (1,27).
  print "Radar: " + round(alt:radar) + " m " at (1,27).
  print "Above terrain: " + round(above_surface(time:seconds)) + " m " at (1,28).
  //print "Terrain height ASL: " + round(ship:geoposition:terrainheight) + " m " at (1,29).
  wait 0.25.
}

lock throttle to 0.
wait 5.
unlock throttle.
unlock steering.
sas on.  
