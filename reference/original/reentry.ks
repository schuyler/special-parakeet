clearscreen.
print "== REENTRY ==".

if periapsis > 70000 {
  print "Re-entry is not expected on this orbit.".
  exit.
}

run common.

print "Orienting to prograde.".
set warp to 0.
sas off.
set pitch to 20.

// Compass azimuth, degrees clockwise from north, of a vector v_ projected
// onto the local horizontal plane. north and up are the local tangent-north
// and radial-out unit vectors; their cross product points east. arctan2 of
// the east and north components of v_ is its heading.
function compass_for {
  parameter v_.
  local east is vcrs(ship:up:vector, ship:north:vector).
  local az is arctan2(vdot(east, v_), vdot(ship:north:vector, v_)).
  if az < 0 { return az + 360. }
  return az.
}

// Hold the nose `pitch` degrees above the horizon along the ground track,
// wings level (roll 0, the wing axis parallel to the ground). heading()
// builds this from the surface-velocity azimuth and the horizon, so it is
// independent of orbit inclination. The old srfprograde + r(...) rode
// srfprograde's own roll reference, which tumbles with inclination and
// gimbal-locks near polar orbits.
lock hdg to heading(compass_for(srfprograde:vector), pitch, 0).
lock steering to hdg.

if altitude > 70000 {
  wait until steering_aligned_to(hdg:vector).
}

print "Disabling rocket engines and re-enabling jet engines.".
local en_list is list().
list engines in en_list.
for en in en_list {
  if not en:ignition {
    en:activate().
  }
  if en:ignition and en:availablethrust > 0 {
    en:shutdown().
  }
}

if altitude > 70000 {
  set warp to 2.
  print "Warping to atmospheric re-entry.".

  wait until altitude < 72000.
}

panels off.

set warpmode to "physics".
set warp to 0.

rcs on.
wait until steering_aligned_to(hdg:vector).

set warp to 2.

print "Waiting for aerodynamic control.".
when airspeed < 800 then {
  print "Flight controls unlocked.".
  set warp to 0.
  unlock steering.
  sas on.
}

// === FLIGHT RECORDER ===
// One CSV row per second, from atmospheric interface to touchdown. Lines
// beginning '#' are the numbers the flight is judged against. The columns
// serve three questions, all ahead of any pitch-feedback controller:
//  - tr_lng and kep_lng against the final lng row: which landing predictor
//    converges, how early, and how monotonically;
//  - steer_err and angvel against q: how firmly the craft holds the
//    commanded pitch across the envelope — the margin any future pitch
//    excursion would spend. steer_err means "held attitude" only while
//    steering is locked, above 800 m/s;
//  - q and airspeed: where the authority window for range control opens
//    and closes.
// q and angvel are logged as kOS reports them and compared only against
// themselves.
local flightlog is "entry_flight.csv".
if exists(flightlog) { deletepath(flightlog). }
log "# ENTRY FLIGHT  " + ship:name + "  mass " + round(ship:mass, 2)
    + " t  pitch " + pitch to flightlog.
log "# orbit  " + round(ship:orbit:periapsis) + " x "
    + round(ship:orbit:apoapsis) + " m  lng "
    + round(ship:geoposition:lng, 2) to flightlog.
log "t,alt,airspeed,v_vert,q,pitch_cmd,pitch_act,aoa,steer_err,angvel,tr_lng,kep_lng,lng"
    to flightlog.

// kep_lng arrives as a string: the drag-free impact point only exists once
// drag has pulled periapsis underground, and before that the column is
// empty. tr_lng goes empty the same way when Trajectories has no impact.
function log_state {
  parameter kep_lng is "".
  local tr_lng is "".
  if addons:tr:available and addons:tr:hasimpact {
    set tr_lng to round(addons:tr:impactpos:lng, 3).
  }
  log round(time:seconds, 1) + "," + round(altitude) + ","
      + round(airspeed, 1) + "," + round(verticalspeed, 1) + ","
      + round(ship:q, 5) + "," + round(pitch, 1) + ","
      + round(90 - vang(up:vector, ship:facing:vector), 1) + ","
      + round(vang(srfprograde:vector, ship:facing:vector), 1) + ","
      + round(vang(hdg:vector, ship:facing:vector), 1) + ","
      + round(ship:angularvel:mag, 4) + ","
      + tr_lng + "," + kep_lng + "," + round(ship:geoposition:lng, 3)
      to flightlog.
}

local t_logged is 0.
until ship:status = "LANDED" or ship:status = "SPLASHED" {
   local kep_lng to "".
   if periapsis <= 0 {
     local t_land to landing_time().
     if t_land >= 0 {
       local site to landing_site(t_land).
       set kep_lng to round(site:lng, 3).
       print "Landing in " + floor(t_land / 60) + ":" + floor(mod(t_land, 60)) + " at (" + round(site:lat,3) + "º, " + round(site:lng, 3) + "º)." at (1,20).
     } else {
       print "No terrain crossing before periapsis.              " at (1,20).
     }
   }
   print "Air pressure: " + round(ship:body:atm:altitudepressure(ship:altitude),4) at (1,19).
   if time:seconds - t_logged >= 1 {
     log_state(kep_lng).
     set t_logged to time:seconds.
   }
}
log "# down  lng " + round(ship:geoposition:lng, 3) + "  airspeed "
    + round(airspeed, 1) to flightlog.
print "Down at " + round(ship:geoposition:lng, 3) + " deg. The witness is entry_flight.csv.".


