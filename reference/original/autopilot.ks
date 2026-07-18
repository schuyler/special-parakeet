// autopilot.ks -- the whole thing: hold altitude and speed while following
// a list of waypoints.
//
// This is the culmination of the level_flight -> heading_hold ->
// waypoint_nav progression, with the speed loop from the original sketch
// restored so it is a complete autopilot rather than a teaching step. Five
// loops in the classic cascade:
//
//   bearing-to-waypoint -> heading err -> bank cmd -> roll -> aileron
//   altitude error      ------------------> pitch cmd -> pitch -> elevator
//   speed error         ------------------------------------> throttle
//
// Outer loops (slow, geometric: where am I) set the setpoints of inner
// loops (fast, attitude: which way am I pointed). Guidance -- choosing which
// waypoint to aim at -- is a thin layer on top of that control law.
//
// Altitude and speed are captured at start (hold what you've got); edit the
// route below before flying. Prerequisite: confirm the sign conventions
// with heading_hold.ks first (pitch-up polarity and compass handedness).

@lazyglobal off.

runoncepath("aero").   // pitch/bank/heading helpers + ground_distance()

// --- the route ------------------------------------------------------------
// Replace with your own coordinates: latlng(latitude, longitude).
local route is list(
  latlng(-0.05, -74.0),
  latlng( 0.50, -73.0),
  latlng( 1.20, -72.0)
).
local arrival_radius is 500.   // metres; count a waypoint reached inside this

// --- loops ----------------------------------------------------------------
local hdg_pid   is pidloop(1.5,  0,     0,     -25, 25).  // hdg err -> bank cmd
local alt_pid   is pidloop(0.05, 0.001, 0,     -12, 12).  // alt err -> pitch cmd
local spd_pid   is pidloop(0.1,  0.01,  0,      0,  1).   // spd err -> throttle
local pitch_pid is pidloop(0.01, 0,     0.005, -1,  1).   // pitch   -> elevator
local roll_pid  is pidloop(0.01, 0,     0.005, -1,  1).   // bank    -> aileron

// --- setup ----------------------------------------------------------------
local target_alt is ship:altitude.               // hold present altitude (ASL)
local target_spd is ship:velocity:surface:mag.   // hold present speed (m/s)
set alt_pid:setpoint to target_alt.
set spd_pid:setpoint to target_spd.

local idx     is 0.        // waypoint we're chasing
local arrived is false.    // true once the last waypoint is reached

print "autopilot: " + route:length + " waypoints, holding "
    + round(target_alt) + " m ASL at " + round(target_spd) + " m/s.".
print "  abort (backspace) to hand controls back.".

// --- control loop ---------------------------------------------------------
local done      is false.
local last_note is time:seconds.
until done {
  local now is time:seconds.

  // GUIDANCE: pick a bank command. While there are waypoints left, aim at
  // the current one (wp:heading = compass bearing to it). Once the route is
  // done, loiter: command wings level and just hold altitude and speed.
  local target_bearing is 0.
  local hdg_err is 0.
  if not arrived {
    local wp is route[idx].
    set target_bearing to wp:heading.
    set hdg_err to heading_error(target_bearing).

    // ARRIVAL: reached if inside the radius, OR the waypoint has slipped
    // more than 90 deg off the nose while we're already close (an abeam
    // pass we'd otherwise loop back for). The distance gate stops a bad
    // initial heading from skipping waypoints.
    local dist is ground_distance(wp).
    if dist < arrival_radius
       or (abs(hdg_err) > 90 and dist < arrival_radius * 4) {
      print "reached waypoint " + (idx + 1) + " of " + route:length + ".".
      set idx to idx + 1.
      if idx >= route:length {
        set arrived to true.
        print "route complete -- loitering (wings level).".
      }
    }
  }
  // after arrival, hdg_err stays 0 -> bank command 0 -> wings level

  // outer loops -> inner setpoints
  set roll_pid:setpoint  to hdg_pid:update(now, hdg_err).
  set pitch_pid:setpoint to alt_pid:update(now, ship:altitude).

  // inner loops -> surfaces and throttle
  set ship:control:roll  to roll_pid:update(now, bank_angle()).
  set ship:control:pitch to pitch_pid:update(now, pitch_angle()).
  set ship:control:mainthrottle to spd_pid:update(now, ship:velocity:surface:mag).

  // light status once a second
  if now - last_note >= 1 {
    local label is choose "loiter" if arrived else "wp " + (idx + 1) + "/" + route:length.
    print label
        + "   hdg err " + round(hdg_err, 1) + " deg"
        + "   alt err " + round(target_alt - ship:altitude, 1) + " m"
        + "   spd err " + round(target_spd - ship:velocity:surface:mag, 1) + " m/s".
    set last_note to now.
  }

  wait 0.
  if abort { set done to true. }
}

// hand the controls back to the pilot
set ship:control:pitch to 0.
set ship:control:roll  to 0.
set ship:control:mainthrottle to 0.
set ship:control:neutralize to true.
print "autopilot: released.".
