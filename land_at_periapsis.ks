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

// Global cache for expensive calculations
local cached_impact_time to 0.
local cached_impact_geo to latlng(0,0).
local cached_suicide_duration to 0.
local cached_suicide_start_5 to 0.
local cached_suicide_start_1 to 0.
local last_cache_update to time:seconds.
local cache_interval to 0.2.  // Update cache every 0.2 seconds

function update_cache {
  if time:seconds - last_cache_update > cache_interval {
    set cached_impact_time to impact_time().
    set cached_impact_geo to impact_geoposition().
    set cached_suicide_duration to suicide_burn_duration().
    set cached_suicide_start_5 to suicide_burn_start(5).
    set cached_suicide_start_1 to suicide_burn_start(1).
    set last_cache_update to time:seconds.
  }
}

// Cached versions of expensive functions
function cached_impact_geoposition {
  update_cache().
  return cached_impact_geo.
}

function cached_suicide_burn_start_5 {
  update_cache().
  return cached_suicide_start_5.
}

function cached_suicide_burn_start_1 {
  update_cache().
  return cached_suicide_start_1.
}

function height_above_surface {
  parameter t is 0.
  local pos is positionat(ship, time:seconds+t).
  local alt_ is (pos - body:position):mag - body:radius.
  return alt_ - body:geopositionof(pos):terrainheight.
}

// Find the time when we'll impact the surface
function impact_time {
  parameter look_ahead_time is ship:orbit:eta:periapsis.
  
  if ship:orbit:periapsis > 0 {
    // If periapsis is above the surface, we can't impact (sort of)
    return look_ahead_time.
  }

  local altitude_func is {
    parameter t.
    return abs(height_above_surface(t)).
  }.
  
  // Find the time when altitude above terrain is minimized
  local impact_t is minimize(altitude_func, 0, look_ahead_time, 1).
  
  // Verify we found an actual impact (not just closest approach)
  local final_altitude is altitude_func(impact_t).
  if final_altitude > 1000 {
    // No impact found, return the search limit
    return look_ahead_time.
  }
  
  return impact_t.
}

// Where will we impact the surface on the current trajectory?
function impact_geoposition {
  local t_impact is impact_time().
  local impact_pos is positionat(ship, time:seconds + t_impact).
  return ship:body:geopositionof(impact_pos).
}

function suicide_burn_duration {
  local h is height_above_surface().
  return burn_duration(max(0, ship:velocity:surface:mag + sqrt(2 * g * h) - landing_speed)).
}

function suicide_burn_start {
  parameter burn_margin is 5.
  //return time_to_surface(time:seconds + suicide_burn_duration() / 2 * burn_margin).
  local burn_duration is suicide_burn_duration().
  return impact_time(burn_duration) - (burn_duration / 2) - burn_margin.
}

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
local lock burn_start to time + time_to_meridian(ship:orbit, target_longitude) - burn_time.
warpTo(burn_start:seconds - warp_margin).

local state to "Initial free fall".

when time > burn_start then {
  set state to "Deorbit burn".
  lock throttle to 1.

  when abs(cached_impact_geoposition():lng - target_longitude) < 0.1 then {
    lock throttle to 0.
    set state to "Free fall".

    when cached_suicide_burn_start_5() <= 0 then {
      lock throttle to 1.
      set state to "Braking burn".
      
      when horizontalVelocity:mag < landing_speed then {
        lock throttle to 0.
        set state to "Final descent".
        gear on.

        when cached_suicide_burn_start_1() <= 0 then {
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
  update_cache().
  print "State: " + state + "                   " at (1,21).
  print "Time to impact: " + round(cached_impact_time, 1) + " s.  " at (1,22).
  print "Suicide burn duration: " + round(cached_suicide_duration, 1) + " s.  "  at (1,23).
  print "Next burn ETA: " + round(cached_suicide_start_5, 1) + " s. " at (1,24).
  print "Horizontal: " + round(horizontalVelocity:mag, 1) at (1,25).
  print "Vertical: " + round(verticalVelocity:mag, 1)  at  (1,26).
  //print "Off axis: " + round(vang(ship:facing:vector, ship:srfretrograde:vector), 1) at (1,27).
  print "Radar: " + round(alt:radar) + " m " at (1,27).
  local pos to body:geopositionof(ship:position).
  print "Geoposition  : " + round(pos:lat, 4) + "º, " + round(pos:lng, 4) + "º  " at (1,28).
  print "Impact lng   : " + round(cached_impact_geo:lng, 3) + "º  " at (1,29).
  //print "Periapsis    : " + round(orbit:periapsis) + "m  " at (1,29).

  wait 0.25.
}

lock throttle to 0.
unlock throttle.
unlock steering.
sas on.  
// wait 5.
