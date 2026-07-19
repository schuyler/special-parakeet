parameter station to false.
parameter roll to 0.

clearscreen.
print "=== DOCKING APPROACH ===".

// Closest port on a vessel that's free and undamaged, or false if it has none.
function resolve_port {
  parameter ves.
  local best is false.
  for port in ves:dockingports {
    if port:state = "Ready" and (best:istype("Boolean") or port:position:mag < best:position:mag) {
      set best to port.
    }
  }
  return best.
}

// Resolve whatever is targeted (or the nearest neighbor) to a docking port.
function pick_target {
  if hastarget {
    if target:istype("DockingPort") {
      return target.
    }
    return resolve_port(target).
  }
  local tgts is list().
  local best is false.
  list targets in tgts.
  for t in tgts {
    if t:position:mag < 2000 {
      local port is resolve_port(t).
      if not port:istype("Boolean") and (best:istype("Boolean") or port:position:mag < best:position:mag) {
        set best to port.
      }
    }
  }
  return best.
}

// Free port on this ship whose axis points closest to the target.
function pick_own_port {
  local aim is target:position:normalized.
  local best is false.
  for port in ship:dockingports {
    if port:state = "Ready" and (best:istype("Boolean") or vdot(port:portfacing:vector, aim) > vdot(best:portfacing:vector, aim)) {
      set best to port.
    }
  }
  return best.
}

function main {
  local tgt is pick_target().
  if tgt:istype("Boolean") {
    print "No ready docking port on target or within 2 km.".
    return.
  }
  set target to tgt.
  wait 0.1.

  local myport is pick_own_port().
  if myport:istype("Boolean") {
    print "No free docking port on this ship.".
    return.
  }
  myport:controlfrom().
  sas off.
  print "Target: " + tgt:ship:name + ", " + tgt:title.

  if station {
    // Point our port at the approacher, then hold attitude until they dock.
    local updir is ship:facing:topvector.
    lock steering to lookdirup(target:position, updir) * r(0, 0, roll).
    wait until vang(target:position, ship:facing:vector) < 1.
    lock steering to "kill".
    wait until not hastarget.
    return.
  }

  rcs on.
  lock steering to lookdirup(target:portfacing:vector * -1, target:portfacing:topvector) * r(0, 0, roll).

  until not hastarget or target:state <> "Ready" {
    local sep is target:position - myport:position.  // our port to theirs
    local axis is target:portfacing:vector.
    local axial is -vdot(sep, axis).                 // how far in front of their port we sit
    local lat is vxcl(axis, sep).                    // corridor offset; points the way we should move
    local relv is ship:velocity:orbit - target:ship:velocity:orbit.
    local aligned is vang(ship:facing:vector, axis * -1) < 5.

    local want is lat:normalized * min(1, 0.2 * lat:mag).
    if axial < 0 {
      set want to v(0, 0, 0).
      print "BEHIND TARGET PORT - reposition manually " at (1, 18).
    } else {
      print "                                         " at (1, 18).
      if aligned and lat:mag < 0.2 * axial + 0.2 {
        set want to want - axis * max(0.1, min(1, axial / 20)).
      }
    }

    local dv is want - relv.
    if dv:mag < 0.05 {
      set ship:control:translation to v(0, 0, 0).
    } else {
      set ship:control:translation to v(vdot(dv, ship:facing:starvector),
                                        vdot(dv, ship:facing:topvector),
                                        vdot(dv, ship:facing:vector)) * 1.5.
    }

    print "AXIAL:   " + round(axial, 2) + " m      " at (1, 20).
    print "LATERAL: " + round(lat:mag, 2) + " m      " at (1, 21).
    print "RELVEL:  " + round(relv:mag, 2) + " m/s      " at (1, 22).
    wait 0.1.
  }

  unlock steering.
  set ship:control:neutralize to true.
  rcs off.
  print "Approach over: magnets acquiring, or target lost." at (1, 24).
}

main().
unlock steering.
