@lazyglobal off.

function air_pressure {
  parameter alt_ is altitude.
  return body:atm:altitudepressure(alt_).
}

function air_density {
  parameter alt_ is altitude.
  parameter body_ is body.
  local atm to body_:atm.
  local p to atm:altitudepressure(alt_) * constant:atmtokpa.
  local t to atm:altitudetemperature(alt_).
  return p * atm:molarmass / (constant:idealgas * t).
}

function speed_of_sound {
  parameter alt_ is altitude.
  parameter body_ is body.
  local atm to body_:atm.
  local p to atm:altitudepressure(alt_) * constant:atmtokpa.
  local rho to air_density(alt_, body_).
  if rho = 0 {
    return 0.
  }
  return sqrt(atm:adiabaticindex * p / rho).
}

function mach_number {
  parameter speed is airspeed.
  parameter alt_ is altitude.
  parameter body_ is body.
  return speed / speed_of_sound(alt_, body_).
}

function dynamic_pressure {
  parameter v_ is airspeed.
  parameter alt_ is altitude.
  return air_density(alt_) * (v_ ^ 2) / 2.
}

function pitch_angle {
  return 90 - vang(ship:facing:vector, ship:up:vector).
}

function angle_of_ascent {
  return 90 - vang(ship:velocity:surface, ship:up:vector).
}

function angle_of_attack {
  return vang(ship:facing:vector, ship:velocity:surface).
}

function accelerometer {
  local t is time:seconds.
  local v_ is ship:velocity:surface.
  local acc is {
    local t1 is time:seconds.
    local v1 is ship:velocity:surface.
    local a is (v1 - v_) / max(t1 - t, 0.001).
    set t to t1.
    set v_ to v1.
    return a.
  }.
  return acc.
}

function thrust_vector {
  parameter alt_ is altitude.
  parameter throttle_ is throttle.

  local p is body:atm:altitudepressure(alt_).
  local f is ship:availablethrustat(p) * throttle_.
  return ship:facing:vector:normalized * f.
}

function weight_vector {
  local m is ship:mass.
  local g is -body:mu / (body:radius ^ 2).
  return m * g * ship:up:vector.
}

function drag_vector {
  parameter accel.
  local m to ship:mass.
  local vel to ship:velocity:surface:normalized.
  local thrust to thrust_vector().
  local weight to weight_vector().
  local drag to -vdot(thrust, vel) + m * vdot(accel, vel) - vdot(weight, vel).
  // sign?
  return drag * vel.
}

function lift_vector {
  parameter accel.
  local m to ship:mass.
  local vel to ship:velocity:surface:normalized.
  local thrust to thrust_vector().
  local weight to weight_vector().
  local accel_ to accel - vdot(accel, vel) * vel.
  local thrust_ to thrust - vdot(thrust, vel) * vel.
  local weight_ to weight - vdot(weight, vel) * vel.
  // sign?
  return m * accel_ - thrust_ + weight_.
}

function lift_to_drag {
  parameter accel.
  local lift to lift_vector(accel).
  local drag to drag_vector(accel).
  return lift:mag / drag:mag.
}
