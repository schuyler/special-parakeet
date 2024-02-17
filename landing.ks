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
  local g is body:mu / (body:distance ^ 2).
  return (-v_ + sqrt(v_ ^ 2 + 2 * g * h)) / g.
}

function time_to_surface {
  local start is time:seconds.
  local t is 0.
  local h is ship:altitude.
  until t = 10000 or h <= 5 {
    set h to above_surface(start + t).
    set t to t + 1.
  }
  return t - 1.
}


function simple_burn_time {
  //parameter v_ is ship:velocity:surface:mag.
  //local accel is ship:availablethrust / ship:mass.
  //print "Accel: " + round(accel, 1) + " m/s^2" at (1,18).
  //return v_ / accel.

  parameter delta_v.

  local ens to list().
  list engines in ens.
  local en to ens[0].

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

until verticalspeed < -1 {
  wait 0.1.
}

// local g is body:mu / (body:distance ^ 2).
// lock t_land to time_to_surface(-verticalspeed).
//set dv to max(ship:velocity:surface:mag + g * t_land, 0).
set t_land to time:seconds + time_to_surface().
set dv to velocityat(ship, t_land):surface:mag.
set burn_time to simple_burn_time(dv).

print "Landing in ~" + round(t_land - time:seconds, 1) + "s.".
print "Surface velocity will be " + round(dv, 1) + " m/s.".
//print "Local g: " + round(g, 3).
//print "Starting component: " + round(ship:velocity:surface:mag, 1).
//print "Gravitational component: " + round(g * t_land, 1).

print "Beginning final descent.".

sas off.
lock steering to ship:srfretrograde.

until alt:radar < 5 {
  set t_land to time:seconds + time_to_surface().
  set dv to velocityat(ship, time:seconds + t_land):surface:mag.
  set burn_time to simple_burn_time(dv - 5).

  if t_land - time:seconds < burn_time {
    lock throttle to 1.
  } else if airspeed < 5 {
    lock throttle to 0.
    lock steering to ship:up.
  }

  print "Burn time: " + round(burn_time, 1) at (1,23).
  print "Landing in " + round(t_land - time:seconds, 1) + " s." at (1,24).
  print "Vspd: " + round(verticalspeed, 1) + " m/s." at (1,25).
  print "Speed: " + round(airspeed, 1) + " m/s." at (1,26).
  //print "Off axis: " + round(vang(ship:facing:vector, ship:srfretrograde:vector), 1) at (1,27).
  print "Radar: " + round(alt:radar, 1) at (1,27).
  print "Above terrain: " + round(above_surface(time:seconds), 1) at (1,28).
  print "Terrain height ASL: " + round(ship:geoposition:terrainheight) at (1,29).
  wait 0.1.
}

lock throttle to 0.
wait 5.
unlock throttle.
unlock steering.
//sas on.  
