parameter station to 0.
parameter roll to 0.
parameter release to 2.

clearscreen.
print "=== DOCKING APPROACH ===".

sas off.
lock rng to target:position - ship:position.

lock up_dir to ship:facing:topvector. // * r(0, 0, 1).
if station = 1 {
  lock steering to lookdirup(target:position, up_dir).
  wait until vang(target:position, ship:facing:vector) < 1.
  lock steering to "kill".
  wait until rng:mag < release.
} else {
  lock orientation to target:facing * v(0, 0, -1).
  lock steering to lookdirup(orientation, up_dir).
  until rng:mag <= release {
    set offset to vxcl(ship:position, target:position).
    print "X: " + round(offset:x, 3) + " m" at (1, 20). 
    print "Y: " + round(offset:y, 3) + " m" at (1, 21). 
    print "Z: " + round(offset:z, 3) + " m" at (1, 22). 
    wait 0.1.
  }
}

unlock steering.
