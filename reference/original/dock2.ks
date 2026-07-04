parameter station to false.
parameter roll to 0.

clearscreen.
print "=== DOCKING APPROACH ===".

if not hastarget {
  local tgts is list().
  local nearest is body.
  list targets in tgts.
  for t in tgts {
    if t:position:mag < 2000 and not t:dockingports:empty {
      for port in t:dockingports {
	if port:position:mag < nearest:position:mag {
	  set nearest to port.
	}
      }
    }
  }
  if nearest <> body {
    set target to nearest.
  }
}

for port in ship:dockingports {
  if not port:haspartner {
    port:controlfrom().
  }
}

sas off.
lock rng to target:position - ship:position.

if station {
  lock up_dir to ship:facing:topvector * r(0, 0, 0).
  lock steering to lookdirup(target:position, up_dir).
  wait until vang(target:position, ship:facing:vector) < 1.
  lock steering to "kill".
  wait until not hastarget.
} else {
  lock up_dir to ship:facing:topvector.
  lock orientation to target:portfacing * v(0, 0, -1).
  lock steering to lookdirup(orientation, target:facing:topvector).
  until target:state <> "Ready" { 
    set offset to vxcl(ship:position, target:position).
    print "X: " + round(offset:x, 3) + " m" at (1, 20). 
    print "Y: " + round(offset:y, 3) + " m" at (1, 21). 
    print "Z: " + round(offset:z, 3) + " m" at (1, 22). 
    wait 0.1.
  }
}

unlock steering.
