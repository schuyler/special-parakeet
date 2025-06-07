@lazyglobal off.

clearscreen.
print "=== LANDING ===".
run "common".
run "orbital".

parameter target_longitude to 0.
parameter landing_speed to 2.

local warp_margin to 30.

local lock g to body:mu / (body:distance ^ 2).
local lock twr to ship:availablethrust / (ship:mass * g).

local lock upVector to (ship:body:position - ship:position):normalized.
local lock verticalVelocity to vdot(ship:velocity:surface, upVector) * upVector.
local lock horizontalVelocity to ship:velocity:surface - verticalVelocity.

function height_above_surface {
  parameter t is 0.
  local pos is positionat(ship, time:seconds + t).
  local alt_ is (pos - body:position):mag - body:radius.
  return alt_ - body:geopositionof(pos):terrainheight.
}

// Find the time when we'll impact the surface

local cached_impact_time to 0.
local last_impact_calculation to 0.

function impact_time {
  parameter look_ahead_time is cached_impact_time + min(cached_impact_time, 60).
  
  if last_impact_calculation > time:seconds + 1 {
    return cached_impact_time.
  }

  if look_ahead_time = 0 {
    set look_ahead_time to ship:orbit:eta:periapsis.
  }

  if ship:orbit:periapsis > 0 {
    // If periapsis is above the surface, we can't impact (sort of)
    return look_ahead_time.
  }

  local altitude_func is {
    parameter t.
    return abs(height_above_surface(t)).
  }.
  
  // Find the time when altitude above terrain is minimized
  local impact_t is minimize(altitude_func, 0, look_ahead_time, alt:radar / 100).
  
  // Verify we found an actual impact (not just closest approach)
  local final_altitude is altitude_func(impact_t).
  if final_altitude > 1000 {
    // No impact found, return the search limit
    return look_ahead_time.
  }
  
  set cached_impact_time to impact_t.
  set last_impact_calculation to time:seconds.
  return impact_t.
}

// Where will we impact the surface on the current trajectory?
function impact_geoposition {
  local t_impact is impact_time().
  local impact_pos is positionat(ship, time:seconds + t_impact).
  return ship:body:geopositionof(impact_pos).
}

function suicide_burn_duration {
  parameter t is 0.
  local h is height_above_surface(t).
  local v_ is velocityat(ship, time:seconds + t).
  
  return burn_duration(v_:surface:mag + sqrt(2 * g * max(h, 0)) - landing_speed).
}

function suicide_burn_start {
  parameter burn_margin is 0.
  local burn_duration is suicide_burn_duration().
  return impact_time(burn_duration) - (burn_duration / 2) - burn_margin.
}

function estimate_delta_x {
  local burn_start is suicide_burn_start().
  local v_start to velocityat(ship, time:seconds + burn_start).
  local t_burn is suicide_burn_duration().
  return v_start * t_burn - 0.35 * (ship:availableThrust / ship:mass) * t_burn ^ 2.
}

function perform_landing {
  if twr = 0 {
    print("==== WARNING: Engine thrust is zero. Do you need to stage? ====").
  }

  if hasnode {
    remove nextnode.
  }

  sas off.
  // lock steering to -horizontalVelocity:normalized * r(0, 0, 1).
  lock steering to ship:srfRetrograde.

  local lock burn_time to burn_duration(horizontalVelocity:mag).
  local lock burn_start to time + time_to_meridian(ship:orbit, target_longitude) - burn_time / 2.
  warpTo(burn_start:seconds - warp_margin).

  local state to "Initial free fall".

  when time > burn_start then {
    set state to "Deorbit burn".
    lock throttle to 1.

    when abs(impact_geoposition():lng - target_longitude) < 0.1 then {
      lock throttle to 0.
      set state to "Free fall".

      when suicide_burn_start(1) <= 0 then {
        lock throttle to 1.
        set state to "Braking burn".
        
        when horizontalVelocity:mag < landing_speed then {
          lock throttle to 0.
          set state to "Final descent".
          gear on.

          when suicide_burn_start(0) <= 0 then {
            lock throttle to 1.
            set state to "Landing burn".

            when airspeed < landing_speed then {
              lock throttle to 0.99 / twr.
              set state to "Powered landing".
            }
          }
        }
      }
    }

    when verticalspeed > -0.1 then {
      set state to "Landed".
      lock throttle to 0.
    }
  }

  until alt:radar <= 2 {
    local impact_geo is impact_geoposition().
    print "State: " + state + "                   " at (1,21).
    print "Time to impact: " + round(impact_time, 1) + " s.  " at (1,22).
    print "Suicide burn duration: " + round(suicide_burn_duration(), 1) + " s.  "  at (1,23).
    print "Suicide burn ETA: " + round(suicide_burn_start(), 1) + " s. " at (1,24).
    print "Horizontal: " + round(horizontalVelocity:mag, 1) at (1,25).
    print "Vertical: " + round(verticalVelocity:mag, 1)  at  (1,26).
    //print "Off axis: " + round(vang(ship:facing:vector, ship:srfretrograde:vector), 1) at (1,27).
    print "Radar: " + round(alt:radar) + " m " at (1,27).
    local pos to body:geopositionof(ship:position).
    print "Geoposition  : " + round(pos:lat, 4) + "º, " + round(pos:lng, 4) + "º  " at (1,28).
    print "Impact lng   : " + round(impact_geo:lng, 3) + "º  " at (1,29).
    //print "Periapsis    : " + round(orbit:periapsis) + "m  " at (1,29).

    wait 1.
  }

  lock throttle to 0.
  lock steering to -upVector.
  wait 5.

  unlock throttle.
  unlock steering.
  sas on.  
 }

perform_landing().