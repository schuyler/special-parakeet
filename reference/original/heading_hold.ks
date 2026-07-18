// heading_hold.ks -- hold a target bearing at the current altitude.
//
// One step up from level_flight.ks. Level flight held the wings level by
// commanding zero bank. Here we UNFREEZE that bank command: an outer loop
// turns heading error into a commanded bank angle, and the roll loop flies
// it. That is the whole idea of a coordinated turn -- to change heading,
// bank; to hold a heading, bank proportionally to how far off you are.
//
//   heading error -> bank command -> aileron         (lateral cascade, NEW)
//   altitude error -> pitch command -> elevator       (same as level_flight)
//
// The roll loop is unchanged from level_flight; only its setpoint moved
// from a constant 0 to the heading loop's output. Throttle stays fixed.
//
// This is also the seam where waypoint guidance later plugs in: replace
// the fixed target_bearing with a bearing-to-waypoint and nothing else
// changes.

@lazyglobal off.

runoncepath("aero").   // pitch_angle(), bank_angle(), compass_heading(), heading_error()

// --- tunables -------------------------------------------------------------
local target_bearing  is 90.     // compass heading to hold (deg): 90 = east
local cruise_throttle is 0.5.    // fixed throttle

// heading err (deg) -> bank command (deg), clamped to a civilised bank
local hdg_pid   is pidloop(1.5,  0,     0,     -25, 25).
// alt error (m)     -> pitch command (deg)
local alt_pid   is pidloop(0.05, 0.001, 0,     -12, 12).
// pitch (deg)       -> elevator (-1..1)
local pitch_pid is pidloop(0.01, 0,     0.005, -1,  1).
// bank (deg)        -> aileron  (-1..1)
local roll_pid  is pidloop(0.01, 0,     0.005, -1,  1).

// --- setup ----------------------------------------------------------------
local target_alt is ship:altitude.        // capture present altitude (ASL)
set alt_pid:setpoint to target_alt.
lock throttle to cruise_throttle.

print "heading_hold: holding " + round(target_bearing) + " deg at "
    + round(target_alt) + " m ASL.".
print "  abort (backspace) to hand controls back.".

// --- control loop ---------------------------------------------------------
// SIGN CHECKS on first flight: (1) positive control:pitch = nose-up, else
// negate pitch_pid; (2) command a heading 30 deg off and confirm it banks
// TOWARD the target, not away -- if away, flip the sign feeding roll_pid's
// setpoint (or heading_error's sign in aero.ks).
local done      is false.
local last_note is time:seconds.
until done {
  local now is time:seconds.

  // outer loops produce inner setpoints
  set roll_pid:setpoint  to hdg_pid:update(now, heading_error(target_bearing)).
  set pitch_pid:setpoint to alt_pid:update(now, ship:altitude).

  // inner loops drive the surfaces
  set ship:control:roll  to roll_pid:update(now, bank_angle()).
  set ship:control:pitch to pitch_pid:update(now, pitch_angle()).

  // light status once a second, for tuning feedback
  if now - last_note >= 1 {
    print "hdg err " + round(heading_error(target_bearing), 1) + " deg"
        + "   bank cmd " + round(roll_pid:setpoint, 1) + " deg"
        + "   alt err " + round(target_alt - ship:altitude, 1) + " m".
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
print "heading_hold: released.".
