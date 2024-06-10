@lazyglobal off.

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
  return sqrt(atm:adiabaticindex * p / rho).
}

function mach_number {
  parameter speed is airspeed.
  parameter alt_ is altitude.
  parameter body_ is body.
  return speed / speed_of_sound(alt_, body_).
}

function relative_drag {
  parameter speed is airspeed.
  parameter alt_ is altitude.
  parameter body_ is body.
  return air_density(alt_, body_) * airspeed ^ 2.
}
