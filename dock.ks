lazyglobal off.

parameter station to false.
parameter roll to 0.
parameter max_speed to 0.5.
parameter release to 2.

clearscreen.
print "=== DOCKING APPROACH ===".

function control_from_docking_port {
  local ctrl to ship:controlpart.
  print "current control part:" + ctrl + ctrl:typename().
  if not(ctrl:typename() = "DockingPort" and ctrl:state = "Ready") {
    for port in ship:partsdubbedpattern("docking") {
      print port:name + ": " + port:state.
      if port:state = "Ready" {
	port:controlfrom().
	break.
      }
    }
  }
  set ctrl to ship:controlpart.
  return ctrl:typename() = "DockingPort" and ctrl:state = "Ready".
}

function target_docking_port {
  local tgt to target.
  if tgt:typename() = "Vessel" and tgt:controlpart:typename = "DockingPort" {
    set tgt to target:controlpart.
  }
  return tgt.
}

function accept_docking {
  sas off.
  local tgt to target.
  local lock rng to ship:position - tgt:position.
  lock steering to lookdirup(tgt:position, ship:facing:topvector).
  wait until vang(tgt:position, ship:facing:vector) < 1.
  lock steering to "kill".
  wait until rng:mag < release.
  unlock steering.
}

function perform_docking {
  sas off.

  local tgt to target_docking_port().
  if tgt:typename() <> "DockingPort" {
    print "Can't determine target docking port.".
    return.
  }

  lock steering to lookdirup(-tgt:portfacing:forevector, tgt:portfacing:topvector).
  lock off_axis to vang(steering:vector, ship:facing:vector).
  until off_axis < 1 {
    print "Rotating: " + round(off_axis,1):tostring:padleft(5) + " deg" at (1, 9).
    wait 1.
  }

  rcs on.

  // Parameters
  local max_approach_speed is 0.5. // m/s
  local alignment_tolerance is 0.1. // m
  local final_approach_distance is 10. // m
  local docking_speed is 0.1. // m/s

  until (target:position - ship:position):mag <= release {
    // Update relative position and velocity
    local relative_position is ship:controlpart:position - tgt:position.
    local relative_velocity is ship:velocity:orbit - tgt:ship:velocity:orbit.
    
    // Transform to target's reference frame
    local tgt_frame is tgt:portfacing.
    local local_position is tgt_frame:inverse * relative_position.
    local local_velocity is tgt_frame:inverse * relative_velocity.
    
    // Determine desired velocity
    local desired_velocity is V(0,0,0).
    if abs(local_position:y) > alignment_tolerance or abs(local_position:z) > alignment_tolerance {
	// Align Y and Z first
	set desired_velocity to V(0, -local_position:y, -local_position:z):normalized * max_approach_speed.
    } else if local_position:x > final_approach_distance {
	// Approach along X axis
	set desired_velocity to V(-max_approach_speed, 0, 0).
    } else {
	// Final approach
	set desired_velocity to V(-docking_speed, 0, 0).
    }
    
    // Calculate required acceleration
    local required_acceleration is desired_velocity - local_velocity.
    
    // Convert back to ship's reference frame
    local ship_acceleration is tgt_frame * required_acceleration.
    
    // Apply acceleration
    set ship:control:translation to ship_acceleration:normalized * min(ship_acceleration:mag, 1).
    
    // Debug output
    print "Position: " + round(relative_position:x,3) + " " + round(relative_position:y,3) + " " + round(relative_position:z,3) at (0,1).
    print "Velocity: " + local_velocity at (0,2).
    print "Desired Velocity: " + desired_velocity at (0,3).
    print "Acceleration: " + ship_acceleration at (0,4).
    
    wait 0.1.
  }
  rcs off.
  unlock steering.
}

if defined target {
  print "Target: " + target:name.
  if control_from_docking_port() {
    if station {
      accept_docking().
    } else {
      perform_docking().
    }
   } else {
    print "Couldn't find an available docking port.".
   }
} else {
  print "No target found.".
}
