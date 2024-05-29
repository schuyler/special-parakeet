@lazyglobal off.

clearscreen.
print "=== LANDING ===".
run "common".
run "orbital".

//function above_surface {
//  parameter t.
//  local pos is positionat(ship, t).
//  local msl_alt is (pos - body:position):mag - body:radius.
//  local geo is body:geopositionof(pos).
//  return msl_alt - geo:terrainheight.
//}

function throttle_needed {
  parameter target_alt.
  local remaining_time is alt:radar / -verticalspeed.
  local needed_time is burn_duration(ship:velocity:surface:mag).
  local ratio is needed_time / max(remaining_time, 1).
  return max(ratio, 0).
}

local g_asl is body:mu / (body:radius ^ 2).
local twr_asl is ship:availablethrust / (ship:mass * g_asl).

if twr_asl = 0 {
  print("==== WARNING: Engine thrust is zero. Do you need to stage? ====").
}

if hasnode {
  remove nextnode.
}

local throttle_start to 0.99.
local landing_speed to 5.
local warp_margin to 45.

local vel to 0.
local upVector to 0.
local verticalVelocity to 0.
local horizonalVelocity to 0.
local horizontalRetrograde to 0.
lock vel to ship:velocity:surface.
lock upVector to (ship:body:position - ship:position):normalized.
lock verticalVelocity to vdot(vel, upVector) * upVector.
lock horizontalVelocity to vel - verticalVelocity.

sas off.
lock steering to -horizontalVelocity:normalized * r(0, 0, 1).

local peri to orbit:periapsis.
local burn_start to time.

if peri > 0 {
  local burn_time to burn_duration(horizontalVelocity:mag).
  set burn_start to time + (time_to_altitude(orbit, peri) - burn_time / 2).

  if time < burn_start - warp_margin {
    set warp to 3.
    when time > burn_start - warp_margin then {
      set warp to 0.
    }
  }
}

local state to "Initial Free Fall".
local target_altitude to 15.
local g to 0.
local twr to 0.
local throttle_pc to 0.
local target_speed to 0.

when time > burn_start then {
  local state to "Reduce horizontal velocity".
  lock throttle to 1 - min(1 / horizontalVelocity:mag, 1).
  when horizontalVelocity:mag < 1 then {
    set state to "Free fall".
    lock throttle to 0.
    lock steering to ship:srfretrograde.
    gear on.

    lock g to body:mu / (body:distance ^ 2).
    lock twr to ship:availablethrust / (ship:mass * g).
    lock throttle_pc to throttle_needed(target_altitude).
    lock target_speed to landing_speed + sqrt(2 * (ship:availablethrust - g) * alt:radar).

    when throttle_pc >= throttle_start then {
      set state to "Braking burn".
      lock throttle to throttle_pc.

      when airspeed <= target_speed then {
	set state to "Final descent".
	lock throttle to 0.
	when throttle_pc >= throttle_start then {
	  set state to "Landing burn".
	  lock throttle to max(throttle_pc, 1/twr).
	}
      }

      when verticalspeed > -0.1 then {
	set state to "Landed".
	lock throttle to 0.
      }
    }
  }
}

until alt:radar <= 2 {
  print "State: " + state + "                   " at (1,22).
  print "Burn starts in " + round((burn_start - time):seconds, 1) + " s. " at (1,23).
  //print "Burn time: " + round(burn_time - max(time:seconds - burn_start, 0), 1) + " s." at (1,23).
  //print "Landing in " + round(t_land - time:seconds, 1) + " s." at (1,24).
  print "Vspd: " + round(verticalspeed, 1) + " m/s." at (1,25).
  print "Speed: " + round(airspeed, 1) + " m/s." at (1,26).
  //print "Off axis: " + round(vang(ship:facing:vector, ship:srfretrograde:vector), 1) at (1,27).
  print "Radar: " + round(alt:radar) + " m " at (1,27).
  //print "Above terrain: " + round(alt:radar, 1) + " m " at (1,28).
  //print "Terrain height ASL: " + round(ship:geoposition:terrainheight) + " m " at (1,29).
  //set throttle_pc to throttle_needed(target_altitude).
  //print "Throttle needed: " + round(throttle_pc, 3) + "       " at (1,29).
  print "Time to target altitude: " + round(time_to_altitude(orbit, target_altitude):seconds, 1) + "     " at (1,30).
  //set burn_start to burn_start_for(target_altitude, safety_margin).
  wait 0.25.
}

lock throttle to 0.
unlock throttle.
unlock steering.
sas on.  
// wait 5.
