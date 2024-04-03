local ship_lng to body:geopositionof(ship:position):lng.
local target_lng to body:geopositionof(target:position):lng.

local target_angular_speed to 360 / target:orbit:period.
local error to (ship_lng - target_lng) / target_angular_speed.

print "Current longitude: " + round(ship_lng, 3) + ".".
print "Current longitude of target: " + round(target_lng, 3) + ".".
print "Time difference: " + round(error, 1) + "s".

