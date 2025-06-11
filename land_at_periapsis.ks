@lazyglobal off.

clearscreen.
print "=== LANDING ===".
run "common".
run "orbital".

parameter target_longitude to 0.
parameter landing_speed to 2.

local warp_margin to 15.

local lock g to body:mu / (body:distance ^ 2).
local lock twr to ship:availablethrust / (ship:mass * g).

local lock upVector to (ship:body:position - ship:position):normalized.
local lock verticalVelocity to vdot(ship:velocity:surface, upVector) * upVector.
local lock horizontalVelocity to ship:velocity:surface - verticalVelocity.

function wrap_longitude {
  parameter lng.
  local result is lng.
  if result > 180 {
    set result to result - 360.
  } else if result < -180 {
    set result to result + 360.
  }
  return result.
}

function body_rotation {
  parameter t.
  return t * 360 / body:rotationperiod.
}

function height_above_surface {
  parameter t is 0.
  local pos is positionat(ship, time:seconds + t).
  local alt_ is (pos - body:position):mag - body:radius.
  return alt_ - body:geopositionof(pos):terrainheight.
}

function time_to_meridian {
  parameter meridian.

  local longitude_difference is {
    parameter t.
    local pos is positionat(ship, time:seconds + t).
    local geo_pos is ship:body:geopositionof(pos).
    local d_lng is meridian - geo_pos:lng.
    if d_lng < 0 {
      set d_lng to d_lng + 360.
    }
    return d_lng.
  }.

  local dt is minimize(longitude_difference, 0, ship:orbit:period, 0.5).
  return timespan(dt).
}

function create_deorbit_node {
  parameter target_lng is target_longitude.
  parameter burn_angle is 90.
  local burn_lng is wrap_longitude(target_lng - burn_angle).
  local t_burn is time + time_to_meridian(burn_lng).
  local r_0 is ship:orbit:periapsis + ship:body:radius.
  local r_1 is ship:body:radius.
  local new_a is (r_0 + r_1) / 2.
  local v_1 is sqrt(ship:body:mu * (2 / r_0 - 1 / new_a)).
  local v_0 is velocityat(ship, t_burn):orbital.
  local delta_v is v_1:normalized * (v_1 - v_0:mag).

  // Optimize the delta_v to minimize the distance of the landing point to the target longitude
  local pos_0 is positionat(ship, t_burn).
  local test_delta_v is {
    parameter dv.
    local orbit_ is createorbit(pos_0 - body:position, v_0 + dv, ship:body, t_burn).
    local impact_geopos is ship:body:geopositionof(positionat(ship, t_burn + impact_time(orbit_))).
    return wrap_longitude(impact_geopos:lng - target_lng).
  }.
  set delta_v to find_zero_crossing(test_delta_v, delta_v * -0.75, delta_v * 1.25, 0.05).
  
  // Create a maneuver node at the burn time with the calculated delta_v
  return node(ship, t_burn, delta_v).
}


// Find the time when we'll impact the surface

local cached_impact_time to 0.
local last_impact_calculation to 0.
local impact_search_margin to 60.

function impact_time {
  parameter orbit_ is ship:orbit.
  parameter lower_bound is max(0, cached_impact_time - impact_search_margin).
  parameter upper_bound is cached_impact_time + min(cached_impact_time, 60).
  parameter search_depth is 8.
  
  local cache_duration is max(0.1, min(10, cached_impact_time / 100)).
  if time:seconds - last_impact_calculation < cache_duration {
    return cached_impact_time.
  }

  if upper_bound = 0 {
    set upper_bound to ship:orbit:eta:periapsis.
  }

  if ship:orbit:periapsis > 0 {
    // If periapsis is above the surface, we can't impact (sort of)
    return upper_bound.
  }

  local altitude_func is {
    parameter t.
    return height_above_surface(t).
  }.
  
  // Find the time when altitude above terrain is minimized
  local impact_t is find_zero_crossing(altitude_func, lower_bound, upper_bound / (2 ^ search_depth)).
  
  // Verify we found an actual impact (not just closest approach)
  local final_altitude is altitude_func(impact_t).
  if final_altitude > 1000 {
    // No impact found, return the search limit
    return upper_bound.
  }
  
  set cached_impact_time to impact_t.
  set last_impact_calculation to time:seconds.
  return impact_t.
}

// Where will we impact the surface on the current trajectory?
function impact_geoposition {
  parameter orbit_ is ship:orbit.
  local t_impact is impact_time(orbit_).
  local impact_pos is positionat(ship, time:seconds + t_impact).
  return ship:body:geopositionof(impact_pos).
}

function suicide_burn_duration {
  parameter t is impact_time().
  local h is height_above_surface(t).
  local v_ is velocityat(ship, time:seconds + t).
  return burn_duration(v_:surface:mag + sqrt(2 * g * max(h, 0)) - landing_speed).
}

function suicide_burn_start {
  parameter burn_margin is 0.
  parameter t is impact_time().
  local burn_duration is suicide_burn_duration(t).
  return t - (burn_duration / 2) - burn_margin.
}

function estimate_downrange_target {
  local t is time_to_meridian(target_longitude).
  local t_burn is suicide_burn_duration(time + t).
  print("t_burn = " + round(t_burn, 3)).
  local pseudo_target is positionat(ship, time + t + 0.5 * t_burn).
  return body:geopositionof(pseudo_target).
}

function perform_landing {
  if twr = 0 {
    print("==== WARNING: Engine thrust is zero. Do you need to stage? ====").
  }

  if hasnode {
    remove nextnode.
  }

  local pseudo_target is estimate_downrange_target().
  print "pseudo_target = " + pseudo_target.

  local prev_target_range is 0.
  function target_range_delta {
    local rng is pseudo_target:lng - impact_geoposition():lng.
    if rng < 0 {
      set rng to rng + 360.
    }
    local delta is rng - prev_target_range.
    if ship:orbit:periapsis < 0 {
      set prev_target_range to rng.
    }
    return delta.
  }

  sas off.
  // lock steering to -horizontalVelocity:normalized * r(0, 0, 1).
  lock steering to ship:srfRetrograde * r(0, 0, 1).

  local burn_start to time + time_to_meridian(pseudo_target:lng).
  local burn_start_vel to velocityat(ship, burn_start).
  local burn_time to burn_duration(burn_start_vel:surface:mag).

  warpTo(burn_start:seconds - burn_time - warp_margin).

  local state to "Initial free fall".

  when time > burn_start - burn_time then {
    set state to "Deorbit burn".
    lock throttle to 1.
    when ship:orbit:periapsis < 0 and target_range_delta() <= 0 then {
      lock throttle to 0.
      set state to "Free fall".

      when suicide_burn_start(2) <= 0 then {
        lock throttle to 1.
        set state to "Braking burn".
        
        when horizontalVelocity:mag < landing_speed then {
          lock throttle to 0.
          set state to "Final descent".
          gear on.

          when suicide_burn_start(0.1) <= 0 then {
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
    

    when verticalspeed > -0.1 and alt:radar < 5 then {
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
    print "Target range delta: " + round(target_range_delta, 3) at (1,30).

    wait 1.
  }

  lock throttle to 0.
  lock steering to -upVector.
  wait 5.

  unlock throttle.
  unlock steering.
  sas on.  
 }

function test_time_to_meridian {
  local t is orbit:eta:periapsis.
  local peri is body:geopositionof(positionat(ship, time+t)).
  local t_meridian is time_to_meridian(peri:lng).
  print("ETA to periapsis: " + round(t,1)).
  print("Periapsis = " + peri).
  print("Time to meridian: " + round(t_meridian:seconds, 1)).
}

perform_landing().