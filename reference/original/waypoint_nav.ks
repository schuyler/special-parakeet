// waypoint_nav.ks -- fly a list of waypoints at the current altitude.
//
// The final step. heading_hold.ks held a FIXED bearing; here the bearing is
// live -- recomputed every tick as the compass heading to the current
// waypoint. That single substitution turns heading hold into navigation:
//
//   bearing-to-waypoint -> heading error -> bank command -> aileron
//   altitude error -> pitch command -> elevator
//
// Everything below the "target_bearing" line is byte-for-byte the same
// control law as heading_hold. The only additions are (1) a waypoint list,
// (2) reading the bearing from the current waypoint, and (3) an arrival
// test that advances to the next one. Guidance is a thin layer on top of
// control -- it decides WHERE to point; the cascades handle pointing.

@lazyglobal off.

runoncepath("aero").   // pitch/bank/heading helpers + ground_distance()

// --- the route ------------------------------------------------------------
// Replace with your own coordinates. latlng(latitude, longitude). These are
// illustrative points east of KSC on Kerbin.
local route is list(
  latlng(-0.05, -74.0),
  latlng( 0.50, -73.0),
  latlng( 1.20, -72.0)
).
local arrival_radius is 500.   // metres; count a waypoint reached inside this

// --- tunables -------------------------------------------------------------
local cruise_throttle is 0.5.

local hdg_pid   is pidloop(1.5,  0,     0,     -25, 25).  // hdg err -> bank cmd
local alt_pid   is pidloop(0.05, 0.001, 0,     -12, 12).  // alt err -> pitch cmd
local pitch_pid is pidloop(0.01, 0,     0.005, -1,  1).   // pitch   -> elevator
local roll_pid  is pidloop(0.01, 0,     0.005, -1,  1).   // bank    -> aileron

// --- setup ----------------------------------------------------------------
local target_alt is ship:altitude.        // capture present altitude (ASL)
set alt_pid:setpoint to target_alt.
lock throttle to cruise_throttle.

local idx is 0.                           // index of the waypoint we're chasing
print "waypoint_nav: " + route:length + " waypoints at "
    + round(target_alt) + " m ASL.".
print "  abort (backspace) to hand controls back.".

// --- control loop ---------------------------------------------------------
local done      is false.
local last_note is time:seconds.
until done {
  local now is time:seconds.
  local wp  is route[idx].

  // GUIDANCE: aim at the current waypoint. wp:heading is the built-in
  // compass heading from the ship to that point -- this is the only line
  // that differs from heading_hold's fixed bearing.
  local target_bearing is wp:heading.

  // outer loops produce inner setpoints
  set roll_pid:setpoint  to hdg_pid:update(now, heading_error(target_bearing)).
  set pitch_pid:setpoint to alt_pid:update(now, ship:altitude).

  // inner loops drive the surfaces
  set ship:control:roll  to roll_pid:update(now, bank_angle()).
  set ship:control:pitch to pitch_pid:update(now, pitch_angle()).

  // ARRIVAL: advance when inside the radius; finish after the last one.
  local dist is ground_distance(wp).
  if dist < arrival_radius {
    print "reached waypoint " + (idx + 1) + " of " + route:length + ".".
    set idx to idx + 1.
    if idx >= route:length {
      set done to true.
    }
  }

  // light status once a second
  if now - last_note >= 1 {
    print "wp " + (idx + 1) + "/" + route:length
        + "   dist " + round(dist) + " m"
        + "   hdg err " + round(heading_error(target_bearing), 1) + " deg"
        + "   bank cmd " + round(roll_pid:setpoint, 1) + " deg".
    set last_note to now.
  }

  wait 0.
  if abort { set done to true. }
}

// hand the controls back to the pilot
set ship:control:pitch to 0.
set ship:control:roll  to 0.
unlock throttle.
set ship:control:neutralize to true.
print "waypoint_nav: route complete, controls released.".
