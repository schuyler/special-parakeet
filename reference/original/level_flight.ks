// level_flight.ks -- hold level flight at the current altitude.
//
// The simplest useful aircraft autopilot. Everything that can be stripped
// away has been: no heading control, no waypoints, no speed loop. What's
// left is the irreducible core of "stay level where I am":
//
//   altitude error -> pitch command -> elevator     (two-loop cascade)
//   wings level    -> aileron                        (single loop, optional)
//
// Throttle is nailed to a constant. The present altitude is captured once
// at start and held.
//
// Why the pitch path is TWO loops and not one: elevator drives pitch RATE,
// which integrates to pitch, to flight-path angle, to vertical speed, to
// altitude -- three integrators of lag stacked up. A single PID from
// altitude straight to elevator cannot be tuned to sit still across all of
// that; it oscillates for any gains. The cascade splits it so each loop
// governs essentially one integrator, and each becomes tunable on its own.

@lazyglobal off.

runoncepath("aero").   // provides pitch_angle()

// --- tunables -------------------------------------------------------------
local cruise_throttle  is 0.5.    // fixed throttle; raise if the nose can't
                                  // hold altitude at this speed.
local hold_wings_level is true.   // set FALSE to test whether the airframe
                                  // holds its wings level on its own.

// alt error (m)  -> pitch command (deg), clamped to a gentle climb/descent
local alt_pid   is pidloop(0.05, 0.001, 0,     -12, 12).
// pitch (deg)    -> elevator (-1..1)
local pitch_pid is pidloop(0.01, 0,     0.005, -1,  1).
// bank (deg)     -> aileron  (-1..1)
local roll_pid  is pidloop(0.01, 0,     0.005, -1,  1).

// --- helpers --------------------------------------------------------------
// Bank angle in degrees; 0 = wings level. starvector points out the right
// wing. Banking right dips it below horizontal, so its component along "up"
// goes negative -- negate so that right-wing-down reads positive.
// SIGN CHECK: if the wings-level loop rolls AWAY from level instead of
// toward it, this sign (or the roll_pid gain sign) is inverted -- flip it.
function bank_angle {
  return -arcsin(vdot(ship:facing:starvector, ship:up:vector)).
}

// --- setup ----------------------------------------------------------------
local target_alt is ship:altitude.        // capture present altitude (ASL)
set alt_pid:setpoint  to target_alt.
set roll_pid:setpoint to 0.               // wings level
lock throttle to cruise_throttle.

print "level_flight: holding " + round(target_alt) + " m ASL.".
print "  abort (backspace) to hand controls back.".

// --- control loop ---------------------------------------------------------
// SIGN CHECK (pitch): this assumes positive ship:control:pitch commands
// nose-UP. If the plane dives into the ground on start, that convention is
// reversed for your build -- negate the pitch_pid output (or its gains).
local done      is false.
local last_note is time:seconds.
until done {
  local now is time:seconds.

  // outer loop: altitude error -> desired pitch attitude
  set pitch_pid:setpoint to alt_pid:update(now, ship:altitude).

  // inner loop: pitch attitude -> elevator
  set ship:control:pitch to pitch_pid:update(now, pitch_angle()).

  // wings level (optional -- see hold_wings_level above)
  if hold_wings_level {
    set ship:control:roll to roll_pid:update(now, bank_angle()).
  }

  // light status once a second, for tuning feedback
  if now - last_note >= 1 {
    print "alt err " + round(target_alt - ship:altitude, 1) + " m"
        + "   pitch cmd " + round(pitch_pid:setpoint, 1) + " deg"
        + "   bank " + round(bank_angle(), 1) + " deg".
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
print "level_flight: released.".
