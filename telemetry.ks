@lazyglobal off.
 
parameter run_ is false.
parameter interval_ is 1.

run "aero".

local telemetry_path is "data".
local telemetry_running is false.

function telemetry_file {
  // local name to (ship:name + "-" + time:calendar + "-" + time:clock).replace(":", "-").
  local name to ship:name:replace(" ", "-").
  return name + ".csv".
}

function start_telemetry {
  parameter interval is 1.
  parameter filename is telemetry_file().
  //local vol is volume(0).
  //if not vol:exists(telemetry_path) {
  //  vol:createdir(telemetry_path).
 // }
  //local dir is vol:open(telemetry_path).
  //local file is dir:create(filename).
  deletepath(filename).
  local write_csv is {
    parameter vals.
    //file.writeln(vals.join(",")).
    log vals:join(",") to filename.
  }.
  local start is time:seconds.
  local tick is start.
  local accel is accelerometer().
  local accel_vector to V(0,0,0).
  lock accel_vector to accel().
  set telemetry_running to true.
  write_csv(list(
    "t",
    "altitude",
    "airspeed",
    "groundspeed",
    "verticalspeed",
    "acceleration",
    "availablethrust",
    "mass",
    "throttle",
    "pitch",
    "angle_of_attack",
    "dynamic_pressure",
    "air_pressure",
    "mach_number",
    "thrust_mag",
    "weight_mag",
    "drag_mag",
    "lift_mag"
  )).
  when telemetry_running and time:seconds > tick + interval then {
    set tick to time:seconds.
    write_csv(list(
      tick - start,
      altitude,
      airspeed,
      groundspeed,
      verticalspeed,
      accel_vector:mag,
      ship:availablethrust,
      ship:mass,
      throttle,
      pitch_angle(),
      angle_of_attack(),
      dynamic_pressure(),
      air_pressure(),
      mach_number(),
      thrust_vector():mag,
      weight_vector():mag,
      drag_vector(accel_vector):mag,
      lift_vector(accel_vector):mag
    )).
    return altitude < body:atm:height.
  }
}

function stop_telemetry {
  set telemetry_running to false.
}

if run_ {
  start_telemetry(interval_).
  wait until false.
} else {
  stop_telemetry().
}
