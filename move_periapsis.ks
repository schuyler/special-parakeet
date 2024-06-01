@lazyglobal off.
parameter lat is 1.18.
parameter lon is -6.9937.
parameter altitudeMargin is 3000.

run "common".

// Function to find the time when the ship is closest to the given ground point
function closestApproachTime {
  parameter targetLat, targetLon.
  // Convert targetLat and targetLon to a vector position
  local targetLocation to ship:body:geopositionlatlng(targetLat, targetLon).
  local distanceToGroundPoint to {
    parameter t.
    // Get the ship's position at time t
    local futurePosition to positionat(ship, t).
    // Calculate the distance between futurePosition and targetPos
    local d_pos to futurePosition - targetLocation:position.
    return d_pos:mag.
  }.

  // Set initial bounds for the minimization (one orbit period)
  local a to time:seconds.
  local b to a + ship:obt:period.

  // Use the minimize function to find the time of closest approach
  local tClosest to minimize(distanceToGroundPoint@, a, b).
  return tClosest.
}

function buildApproachNode {
  parameter targetLat, targetLon, heightAboveGround.

  local targetLocation to ship:body:geopositionlatlng(targetLat, targetLon).
  local targetAltitude to targetLocation:terrainheight + heightAboveGround.

  local distanceToGroundPoint to {
    parameter t.
    // Get the ship's position at time t
    local futureLon to targetLon + 360 * (t - time:seconds) / body:rotationperiod.
    if futureLon > 180 {
      set futureLon to futureLon - 360.
    }
    local futureTarget to ship:body:geopositionlatlng(targetLat, futureLon).
    local futurePosition to positionat(ship, t).
    // Calculate the distance between futurePosition and targetPos
    local d_pos to futurePosition - futureTarget:position.
    return d_pos:mag.
  }.
  
  local makeNode to {
    parameter t.
    local posAtT is positionat(ship, t).
    local initialAltitude is body:altitudeof(posAtT).
    local targetSpeed is orbital_speed(ship:orbit, initialAltitude, targetAltitude, initialAltitude).
    local initialSpeed is orbital_speed(ship:orbit, initialAltitude).
    return node(t, 0, 0, targetSpeed - initialSpeed).
  }.

  local evaluateNode to {
    parameter t.
    local nd to makeNode(t).
    add nd.
    local dist to distanceToGroundPoint(t + orbitat(ship, t+1):period / 2).
    remove nd.
    return dist.
  }.

  local a to time:seconds.
  local b to a + ship:obt:period.

  // Use the minimize function to find the time of closest approach
  local burnStart to minimize(evaluateNode@, a, b).    
  add makeNode(burnStart).
}

// Function to calculate the time to perform a burn to set the periapsis to a specific ground point
function timeToSetPeriapsisToGroundPoint {
  parameter targetLat, targetLon, desiredPeriapsis.

  // Get the body and radius
  local bodyRadius to body:radius.

  // Get current longitude
  local currentLon to body:geopositionof(ship:orbit:position):lng.
  print "Current lng: " + currentLon + "º".

  // Calculate the position of the target ground point
  local antipode is targetLon + 180.
  if antipode > 180 {
    set antipode to antipode - 360.
  }
  print "Antipode of " + targetLon + "º is " + antipode + "º".

  // Advance the antipode east for the time it will take to get there
  local orbitsToAntipode to mod(antipode - currentLon, 360) / 360.
  print "orbitsToAntipode: " + round(orbitsToAntipode, 3).
  print "seconds to antipode: " + round(orbitsToAntipode * orbit:period, 1) + "s.".
  set antipode to antipode + 360 * (orbit:period * orbitsToAntipode) / body:rotationperiod.
  print "Adjusted antipode: " + round(antipode, 3) + "º".

  // That's where we do the burn itself
  local orbitsToBurnStart to mod(antipode - currentLon, 360) / 360.
  local timeToBurn to ship:orbit:period * orbitsToBurnStart.
  print "timeToBurn: " + timeToBurn.

  // Still need to advance the antipode by half an orbit from the new apoapsis

  return time + timeToBurn.
}

// local t to closestApproachTime(lat, lon).
// local shipPos to body:geopositionof(positionat(ship, t)).
// local targetPos to body:geopositionlatlng(lat, lon).
// 
// print "Closest approach: " + (targetPos:position - shipPos:position):mag.
// print "Closest time: " + (timestamp(t) - time).
// 
// local burnStart to timeToSetPeriapsisToGroundPoint(shipPos:lat, shipPos:lng, 8000).
// print "Optimal burn time: " + burnStart.
// 
// local targetAltitude is targetPos:terrainheight + altitudeMargin.
// local initialAltitude is body:altitudeof(positionat(ship, burnStart)).
// local initialSpeed is orbital_speed(ship:orbit, initialAltitude).
// local targetSpeed is orbital_speed(ship:orbit, initialAltitude, targetAltitude, initialAltitude).
// 
// print "Burn altitude: " + round(initialAltitude).
// print "Current periapsis: " + round(orbit:periapsis).
// print "Target periapsis: " + round(targetAltitude).
// print "Initial speed: " + round(initialSpeed).
// print "Target speed: " + round(targetSpeed).
// print "Delta-V: " + round(initialSpeed - targetSpeed, 1).
// 
// local nd to node(burnStart, 0, 0, targetSpeed - initialSpeed).
// add nd.

buildApproachNode(lat, lon, 8000).
