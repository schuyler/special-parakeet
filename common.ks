
function burn_time {
  parameter dv.

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
  local dMass to wMass / (constant:E ^ (dv / (en:isp * constant:g0))).
  local flowRate to thrust / (en:isp * constant:g0).
  local burn_time to (wMass - dMass) / flowRate.
  return burn_time.
}
