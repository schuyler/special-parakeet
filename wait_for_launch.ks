clearscreen.
parameter time_to_orbit to 810.
parameter lng_at_apoapsis to -6.
parameter correction is 30. // so that we get to orbit a little behind the target

print "=== WAIT FOR LAUNCH ===".

local target_angular_speed to 360 / target:orbit:period.
local body_angular_speed to 360 / body:rotationperiod.
local target_motion_during_ascent to time_to_orbit * (target_angular_speed - body_angular_speed).

// local ship_lng to body:geopositionof(ship:position):lng.
local target_lng_at_launch to lng_at_apoapsis - target_motion_during_ascent.

if target_lng_at_launch < -180 {
  set target_lng_at_launch to target_lng_at_launch + 360.
}

local target_lng to body:geopositionof(target:position):lng.
local d_lng to target_lng_at_launch - target_lng.
if d_lng < 0 {
  set d_lng to d_lng + 360.
}
local dt to d_lng / (target_angular_speed - body_angular_speed).
set dt to dt + correction.

print "Target period: " + round(target:orbit:period, 1) + "s.".
print "Target angular speed: " + round(target_angular_speed, 3) + "ª/s.".
print "".
print "Time to orbit: " + round(time_to_orbit) + "s.".
print "Current target longitude: " + round(target_lng, 3) + "º.".
print "Target longitude at launch: " + round(target_lng_at_launch, 3) + "º.".
print "Time to launch: " + round(dt) + "s.".

set brakes to true.
warpto(time:seconds + dt).
wait until kuniverse:timewarp:issettled.
set brakes to false.

//run ssto.

