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
  local h is ship:altitude.
  local t is 0.
  local step is 1. //(h / -ship:verticalspeed) / 100.
  until h <= target {
    set h to above_surface(start + t).
    set t to t + step.
  }
  return t - step.
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
  parameter target_alt.
  parameter margin.
  local target_time to time:seconds + time_to_altitude(target_alt).
  local dv to velocityat(ship, target_time):surface:mag.
  print("dv needed: " + round(dv, 1) + " m/s").
  local burn_time to simple_burn_time(dv).
  return target_time - (burn_time * margin) / 2.
}

function throttle_needed {
  parameter target_alt.
  local remaining_time is time_to_altitude(target_alt).
  local needed_time is simple_burn_time(ship:velocity:surface:mag).
  local ratio is needed_time / max(remaining_time, 1).
  return max(ratio, 0).
}

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

local g_asl is body:mu / (body:radius ^ 2).
local twr_asl is ship:availablethrust / (ship:mass * g_asl).

if twr_asl = 0 {
  print("==== WARNING: Engine thrust is zero. Do you need to stage? ====").
}

if hasnode {
  remove nextnode.
}

//local safety_margin to 1.01.
local throttle_start to 0.999.
local throttle_stop to 0.50.

set braking_finish to 250. // meters
set vertical_descent to 25. // meters
set landing_speed to 5.
set warp_margin to 60.

local state to "Initial Free Fall".
local target_altitude to vertical_descent.
local throttle_pc to 0.

sas off.
lock steering to ship:srfretrograde * r(0,0,1).

set burn_start to burn_start_for(target_altitude, 2).
if time:seconds < burn_start - warp_margin {
  set warp to 3.
  when time:seconds > burn_start - warp_margin then {
    set warp to 0.
  }
}

lock g to body:mu / (body:distance ^ 2).
lock twr to ship:availablethrust / (ship:mass * g).
lock throttle_pc to throttle_needed(target_altitude).
lock target_speed to landing_speed + sqrt(2 * (ship:availablethrust - g) * alt:radar).

when throttle_pc >= throttle_start then {
  set state to "Braking Burn".
  lock throttle to throttle_pc.

  when airspeed <= target_speed or throttle_pc < throttle_stop then {
    set state to "Free Fall".
    lock throttle to 0.
    when throttle_pc >= throttle_start then {
      set state to "Final Descent".
      lock throttle to max(throttle_pc, 1/twr).
    }
  }

  when verticalspeed > -0.1 then {
    set state to "Landed".
    lock throttle to 0.
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
  //set throttle_pc to throttle_needed(target_altitude).
  print "Throttle needed: " + round(throttle_pc, 3) + "       " at (1,29).
  print "Time to target altitude: " + round(time_to_altitude(target_altitude), 1) + "     " at (1,30).
  //set burn_start to burn_start_for(target_altitude, safety_margin).
  wait 0.25.
}

lock throttle to 0.
unlock throttle.
unlock steering.
sas on.  
// wait 5.
