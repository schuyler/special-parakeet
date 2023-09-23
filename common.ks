
function burn_time {
  parameter delta_v.

  // determine engine ISP
  local eng_list to list().
  list engines in eng_list.

  for en_ in eng_list {
    if en_:vacuumisp > 0 {
	  set en to en_.
    }
  }

  // determine burn time
  // TBD: work through the Rocket Equation and confirm this math
  local thrust to ship:maxthrustat(0).
  local wMass to ship:mass.
  local dMass to wMass / (constant:E ^ (delta_v / (en:isp * constant:g0))).
  local flowRate to thrust / (en:isp * constant:g0).
  local burn_time to (wMass - dMass) / flowRate.
  return burn_time.
}

function orbital_speed {
  // it's the good old vis-viva equation
  parameter orbiter.
  parameter altitude_ is orbiter:altitude.
  parameter apo is orbiter:apoapsis.
  parameter peri is orbiter:periapsis.

  local body_ to orbiter:body.
  local g to body_:mu.
  local r_ to body_:radius + altitude_.
  local a to (2 * body_:radius + apo + peri) / 2.
  return sqrt(g * ((2 / r_) - (1 / a))).
}
