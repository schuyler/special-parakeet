// Docking approach, flown on RCS.
//
// Monopropellant, not time, is the scarce resource here. A reposition of d
// metres is paid for twice — once to start the motion, once to stop it — and
// the bill falls in exact proportion to how long the approach is allowed to
// take. So the approach is sized from a delta-v budget rather than from fixed
// gains, attitude is held inside a deadband rather than continuously, and the
// run writes a CSV so the attitude spend and the translation spend can be told
// apart afterwards instead of guessed at.

parameter station is false.
parameter roll is 0.

// Translation delta-v the approach may spend, m/s. A light spaceplane's entire
// monopropellant load is often worth under ten m/s; two of them buys the
// approach and leaves the rest for attitude, a second attempt and undocking.
// Stated as delta-v, so it says nothing about this craft's thrusters or tanks.
parameter budget is 2.

// Attitude deadband, degrees. Stock ports capture from well outside ten
// degrees, so five is margin, not a limit. The ship is released again at a
// third of it, which makes the duty cycle a function of how fast the craft
// drifts rather than of this number.
parameter align is 5.

// Closing speed at contact, m/s. Fast enough that the magnets reach across
// their capture range before the corridor controller stalls inside its own
// deadband; slow enough that a missed latch is a nudge, not a collision.
parameter v_touch is 0.1.

// Ceiling on closing speed whatever the budget asks for, m/s. Above a couple
// of m/s a port bounces or breaks instead of latching. A collision limit is a
// design input, not something the budget gets to optimise away.
local v_max is 2.

local docklog is "dock_log.csv".

clearscreen.
print "=== DOCKING APPROACH ===".

if budget <= 0 {
  print "Budget must be a positive delta-v in m/s.".
  wait until false.
}

if not hastarget {
  print "No target. Pick a vessel or port in map view and rerun.".
  wait until false.
}

// A vessel target gets resolved to its closest free port.
if not target:istype("DockingPort") {
  local best is false.
  for port in target:dockingports {
    if port:state = "Ready" and (best:istype("Boolean") or port:position:mag < best:position:mag) {
      set best to port.
    }
  }
  if best:istype("Boolean") {
    print "No ready docking port on " + target:name + ".".
    wait until false.
  }
  set target to best.
  wait 0.1.
}

local myport is false.
for port in ship:dockingports {
  if port:state = "Ready" {
    set myport to port.
    break.
  }
}
if myport:istype("Boolean") {
  print "No free docking port on this ship.".
  wait until false.
}
myport:controlfrom().
sas off.
print "Target: " + target:ship:name + ", " + target:title.

// Monopropellant remaining, in units. The log's witness: what the flight
// actually cost, against what the budget said it would.
function monoprop {
  for res in ship:resources {
    if res:name = "MonoPropellant" { return res:amount. }
  }
  return 0.
}

// Where the ship should be pointing. Station mode faces the approacher; the
// approaching ship faces back down the target port's own axis.
global updir is ship:facing:topvector.

function aim_dir {
  if station { return lookdirup(target:position, updir) * r(0, 0, roll). }
  return lookdirup(target:portfacing:vector * -1, target:portfacing:topvector) * r(0, 0, roll).
}

lock aim to aim_dir().

// An attitude, once reached, costs nothing to keep in free space — only to
// change. A continuously held `lock steering` pays anyway, because the
// steering manager answers every disturbance the instant it appears, and on a
// craft whose only torque is RCS that answer is monopropellant. So the ship is
// steered only when it has drifted outside the deadband: engage at `band`
// degrees, release at a third of that once the error is already coming down,
// coast in between. `steering_on` tells the caller which of the two it is in.
global steering_on is false.
global att_prev is 180.

function hold_attitude {
  parameter band.   // deadband half-width, degrees

  // Pointing error only. Roll rides along whenever steering engages, and stock
  // ports snap to whatever roll they meet, so it is not worth a budget of its
  // own.
  local err is vang(ship:facing:vector, aim:vector).
  if steering_on {
    if err < band / 3 and err <= att_prev {
      unlock steering.
      set ship:control:rotation to v(0, 0, 0).
      set steering_on to false.
    }
  } else if err > band {
    lock steering to aim.
    set steering_on to true.
  }
  set att_prev to err.
  return err.
}

if station {
  // Point our port at the approacher, then hold that attitude in the deadband
  // until they dock. Nothing else to do, and nothing else worth burning.
  until not hastarget {
    hold_attitude(align).
    wait 0.1.
  }
} else {
  rcs on.

  local start is time:seconds.
  local mono0 is monoprop().
  local sep0 is target:position - myport:position.
  local axis0 is target:portfacing:vector.
  local axial0 is -vdot(sep0, axis0).
  local lat0 is vxcl(axis0, sep0):mag.

  // Closing a separation d on a proportional law v = d/tau costs d/tau to
  // start the motion and d/tau again to stop it, so a budget B spread over the
  // whole separation fixes the one time constant the approach needs:
  //
  //     tau = 2 d / B
  //
  // Halving the budget doubles the clock and changes nothing else. Every gain
  // below is this tau; there are no others to tune.
  // Floored at a second so a run started already touching the port cannot
  // divide by a zero separation; at that range there is nothing left to fly.
  local tau is max(1, 2 * (abs(axial0) + lat0) / budget).
  // Exponential run-down to where the contact floor takes over, plus the
  // constant-speed run-in, which costs one more time constant.
  local t_est is tau * (1 + max(0, ln(max(axial0, 1) / max(v_touch * tau, 0.1)))).

  print "Budget " + round(budget, 2) + " m/s -> tau " + round(tau) + " s, "
      + "about " + round(t_est) + " s to contact.".

  if exists(docklog) { deletepath(docklog). }
  log "# port " + target:ship:name + " " + target:title
      + "  axial " + round(axial0, 1) + " m  lateral " + round(lat0, 1) + " m" to docklog.
  log "# budget " + round(budget, 2) + "  tau " + round(tau, 1)
      + "  t_est " + round(t_est) + "  align " + round(align, 1)
      + "  v_touch " + round(v_touch, 2) + "  mono " + round(mono0, 2) to docklog.
  log "t,phase,axial,lateral,relv,v_cmd,dv_err,att_err,steer,mono" to docklog.

  local next_row is start.
  local handed_off is false.

  until not hastarget or target:state <> "Ready" {
    local sep is target:position - myport:position.  // our port to theirs
    local axis is target:portfacing:vector.
    local axial is -vdot(sep, axis).                 // how far in front of their port we sit
    local lat is vxcl(axis, sep).                    // corridor offset; points the way we should move
    local relv is ship:velocity:orbit - target:ship:velocity:orbit.
    local err is hold_attitude(align).

    // The approach corridor: a cone about the target port's axis widening one
    // metre for every five of range, with 20 cm of slack at the port itself.
    local corridor is 0.2 * axial + 0.2.

    local want is v(0, 0, 0).
    local dv is v(0, 0, 0).
    local phase is "SLEW".

    if axial < 0 {
      // Behind their port's plane. Hand translation back to the pilot once and
      // stay out of the way — holding zero relative velocity here would fight
      // whatever they do to fix it, and pay for the privilege.
      set phase to "BEHIND".
      if not handed_off {
        set ship:control:neutralize to true.
        set handed_off to true.
      }
      print "BEHIND TARGET PORT - reposition manually " at (1, 18).
    } else {
      set handed_off to false.
      print "                                         " at (1, 18).
      if steering_on {
        // Slewing. Translating now would push against a thruster set that is
        // still moving, so coast: velocity keeps itself.
        set ship:control:translation to v(0, 0, 0).
      } else {
        set want to lat * (1 / tau).                 // centre on the corridor
        if lat:mag < corridor {
          set want to want - axis * min(v_max, max(v_touch, axial / tau)).
        }
        set dv to want - relv.
        // Tolerate the velocity error that would carry the ship out of the
        // corridor no faster than the approach is closing it, floored where
        // the physics tick's own jitter lives. Nulling anything smaller is
        // monopropellant spent on noise, and the deadband widens with range
        // for the same reason: 5 cm/s is nothing at 100 m and everything at 1.
        local dv_band is max(0.02, corridor / tau).
        if dv:mag < dv_band {
          set ship:control:translation to v(0, 0, 0).
          set phase to "COAST".
        } else {
          set ship:control:translation to v(vdot(dv, ship:facing:starvector),
                                            vdot(dv, ship:facing:topvector),
                                            vdot(dv, ship:facing:vector)) * 1.5.
          set phase to "TRANS".
        }
      }
    }

    if time:seconds >= next_row {
      set next_row to time:seconds + 1.
      local steer is 0.
      if steering_on { set steer to 1. }
      log round(time:seconds - start, 1) + "," + phase + ","
          + round(axial, 2) + "," + round(lat:mag, 2) + ","
          + round(relv:mag, 3) + "," + round(want:mag, 3) + ","
          + round(dv:mag, 3) + "," + round(err, 2) + ","
          + steer + "," + round(monoprop(), 2) to docklog.
    }

    print "PHASE:   " + phase + "      " at (1, 19).
    print "AXIAL:   " + round(axial, 2) + " m      " at (1, 20).
    print "LATERAL: " + round(lat:mag, 2) + " m      " at (1, 21).
    print "RELVEL:  " + round(relv:mag, 2) + " m/s      " at (1, 22).
    print "ATT ERR: " + round(err, 2) + " deg      " at (1, 23).
    wait 0.1.
  }

  set ship:control:neutralize to true.
  rcs off.
  log "# done  elapsed " + round(time:seconds - start) + " s"
      + "  mono used " + round(mono0 - monoprop(), 2) + " units" to docklog.
  print "Approach over: magnets acquiring, or target lost." at (1, 25).
}

unlock steering.
