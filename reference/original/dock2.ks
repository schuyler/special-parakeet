parameter station to false.
parameter roll to 0.

clearscreen.
print "=== DOCKING APPROACH ===".

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

if station {
  // Point our port at the approacher, then hold attitude until they dock.
  local updir is ship:facing:topvector.
  lock steering to lookdirup(target:position, updir) * r(0, 0, roll).
  wait until vang(target:position, ship:facing:vector) < 1.
  lock steering to "kill".
  wait until not hastarget.
} else {
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

  set ship:control:neutralize to true.
  rcs off.
  print "Approach over: magnets acquiring, or target lost." at (1, 24).
}

unlock steering.
