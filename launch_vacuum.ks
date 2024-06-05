@lazyglobal off.
parameter target_apoapsis is 20000.

clearscreen.
print "== AIRLESS LAUNCH ==".

function find_safe_ascent {
  parameter pos is ship:position.
  parameter hdg is 90.
  parameter limit is 5000.
  parameter margin is 0.
  parameter d_step is 200.

  local alt_ to ship:altitude.
  local min_angle to 0.
  local clearance to 0.
  from { local d to d_step. } until d > limit step { set d to d + d_step. } do {
    local pos to ship:position + heading(hdg, min_angle):vector:normalized * d.
    local alt_ to (pos - body:position):mag - body:radius.
    local geo to body:geopositionof(pos).
    local h to alt_ - (geo:terrainheight + margin).
    local angle to mod(vang(pos, geo:position), 90).
    if h < clearance {
      set clearance to h.
      if angle > min_angle {
	set min_angle to angle.
      }
    }
  }
  return min_angle.
}

function perform_launch {
  parameter target_apo.
  parameter hdg is 90.
  parameter min_pitch is 10.

  local g to body:mu / (body:radius ^ 2).
  local twr is ship:availablethrust / (ship:mass * g).
  local pitch to 0.
  local ascent to 0.

  sas off.
  lock ascent to find_safe_ascent(positionat(ship, time:seconds + orbit:eta:apoapsis), hdg).
  lock pitch to ascent.
  lock steering to body:up.
  lock throttle to 1.
  when pitch <= min_pitch and verticalspeed > 10 then {
    lock throttle to 0.
    lock ascent to find_safe_ascent(ship:position, hdg).
    lock pitch to min(ascent + min_pitch, 90).
    lock steering to heading(hdg, pitch).
    when pitch <= min_pitch and vang(ship:facing:vector, steering:vector) <= 5 then {
      lock throttle to 1.
      lock steering to heading(hdg, pitch) * r(0,0,0).
    }
  }
  until ship:apoapsis >= target_apo {
    print "Pitch: " + round(pitch, 1) + "º." at (1, 25).
    print "Ascent: " + round(ascent, 1) + "º." at (1, 26).
    print "TWR: " + round(twr, 1) + "." at (1, 27).
    wait 0.25.
  }
  
  lock throttle to 0.
  unlock steering.
  unlock throttle.
}

perform_launch(target_apoapsis).
wait 5.
run circularize.
wait until alt:radar > 5000.
set warp to 2.
wait until orbit:eta:apoapsis <= 60.
set warp to 0.
run next.
