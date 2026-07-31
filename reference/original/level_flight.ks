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

runoncepath("aero").   // pitch_angle(), bank_angle(), angle_of_ascent(),
                       // dynamic_pressure()
runoncepath("afbw").   // afbw_release(), afbw_restore()

// --- tunables -------------------------------------------------------------
local pitch_range is 30.
local cruise_throttle  is 0.5.    // fixed throttle; raise if the nose can't
                                  // hold altitude at this speed.
local hold_wings_level is true.   // set FALSE to test whether the airframe
                                  // holds its wings level on its own.

// Gains are named rather than written into the constructors so the log header
// can record them: a stack of tuning CSVs that does not say which gains
// produced each one cannot be compared against the next.
// alt error (m)  -> pitch deviation (deg), clamped to a gentle climb/descent
local alt_kp   is 0.05.
local alt_ki   is 0.001.
local alt_kd   is 0.
// pitch (deg)    -> elevator (-1..1)
// The integral term carries the deflection the airframe needs to hold its
// attitude, which a proportional term can only produce by standing off its
// setpoint. Its time constant is pitch_kp / pitch_ki, here 2 s: fast against
// the phugoid, the slow trade of altitude for speed that would carry the
// aircraft away while the term winds up, and slow against the short period,
// the roughly one-second pitch oscillation the derivative term damps. A
// deflection of 0.2 against an output clamped at 1 leaves room to wind.
local pitch_kp is 0.05.
local pitch_ki is 0.025.
local pitch_kd is 0.005.
// bank (deg)     -> aileron  (-1..1)
local roll_kp  is 0.01.
local roll_ki  is 0.
local roll_kd  is 0.005.

local alt_pid   is pidloop(alt_kp,   alt_ki,   alt_kd,   -pitch_range, pitch_range).
local pitch_pid is pidloop(pitch_kp, pitch_ki, pitch_kd, -1, 1).
local roll_pid  is pidloop(roll_kp,  roll_ki,  roll_kd,  -1, 1).

// --- state row ------------------------------------------------------------
// One row of state per second, rendered two ways from one list of values so
// the console and the CSV cannot drift apart. Column widths hold the widest
// value each column can produce; a value that outgrows its width loses its
// leading space and nothing else.
//
// The three angles the pitch cascade lives among, all in degrees:
//   pitch -- nose above the horizon, what the inner loop tracks
//   gamma -- velocity vector above the horizon, what sets climb rate
//            (v_vert = tas * sin(gamma))
//   aoa   -- pitch minus gamma, the nose's angle off the velocity vector in
//            the vertical plane. Signed, unlike aero.ks's angle_of_attack(),
//            because a stall limit needs a sign.
// cmd is the attitude the inner loop is being asked to hold: the angle of
// attack at engagement plus the deviation the outer loop asks for. cmd
// against pitch is
// the discriminating pair -- pitch tracking cmd means the inner loop is
// delivering what it was asked; pitch lagging cmd means it cannot.
// thr is the throttle the vessel is running; cthr is what this script wrote
// into the raw control structure a tick earlier. Both are read back rather
// than assumed from cruise_throttle, because all three have been seen to
// disagree. cthr equal to cruise_throttle while thr differs puts the loss
// downstream of the control structure; cthr differing too puts it upstream.
// q is dynamic pressure, air density times airspeed squared over two -- the
// pressure the air exerts on a surface held across the flow, and so what sets
// how much force a given elevator deflection produces. The deflection needed
// to hold an attitude goes as 1/q, which makes elev alone meaningless across
// conditions: two flights are only comparable through elev x q.
local col_names is list("t", "alt", "err", "cmd", "pitch", "gamma", "aoa",
                        "elev", "bank", "ail", "thr", "cthr", "tas", "q").
local col_width is list(8, 8, 7, 6, 6, 6, 6, 7, 7, 7, 6, 7, 6, 8).

function padded {
  parameter text, width.
  local out is "" + text.
  until out:length >= width { set out to " " + out. }
  return out.
}

function columns {
  parameter values, widths.
  local line is "".
  local i is 0.
  until i >= values:length {
    set line to line + padded(values[i], widths[i]).
    set i to i + 1.
  }
  return line.
}

function commas {
  parameter values.
  local line is "".
  local i is 0.
  until i >= values:length {
    if i > 0 { set line to line + ",". }
    set line to line + values[i].
    set i to i + 1.
  }
  return line.
}

// --- setup ----------------------------------------------------------------
// Take the axes off AFBW before anything else: it wins the arbitration against
// kOS, so a throttle lock set while it holds the axis is obeyed by nothing but
// the variable that reads it back.
local afbw_released is afbw_release().

local target_alt is ship:altitude.        // capture present altitude (ASL)
set alt_pid:setpoint  to target_alt.
set roll_pid:setpoint to 0.               // wings level

// The angle of attack the aircraft is flying as the script takes over: its
// nose attitude less its flight path angle. The outer loop commands pitch as a
// deviation from this rather than as an absolute attitude, because an aircraft
// in level flight sits nose-up -- it needs angle of attack to make lift -- so
// a command of zero degrees is a command to dive.
//
// Angle of attack rather than pitch is what makes the datum safe to sample at
// any moment. Pitch is angle of attack plus flight path angle, so sampling
// pitch imports whatever the aircraft was doing at that instant; and since
// pitch_range bounds the deviation from the datum, a datum inflated by a climb
// leaves the loop little authority in the other direction. Angle of attack
// stays within a few degrees whatever the flight path is doing.
local aoa_at_engage is pitch_angle() - angle_of_ascent().

// Lines beginning '#' are the settings the flight is judged against.
// cruise_throttle here is what the script commands; the thr column is what the
// vessel is running, so the two can be read against each other.
local settings is "# target_alt " + round(target_alt) + " m ASL"
    + "  aoa_at_engage " + round(aoa_at_engage, 2) + " deg"
    + "  pilotpitchtrim " + round(ship:control:pilotpitchtrim, 4)
    + "  pitch_range " + pitch_range
    + "  cruise_throttle " + cruise_throttle
    + "  hold_wings_level " + hold_wings_level
    + "  afbw_released " + afbw_released.
local gains is "# gains (kp ki kd)  alt " + alt_kp + " " + alt_ki + " " + alt_kd
    + "  pitch " + pitch_kp + " " + pitch_ki + " " + pitch_kd
    + "  roll " + roll_kp + " " + roll_ki + " " + roll_kd.
local flightlog is "level_flight_" + round(time:seconds) + ".csv".
log settings to flightlog.
log gains to flightlog.
log commas(col_names) to flightlog.

// Throttle goes through the raw control structure, not LOCK THROTTLE. The two
// are alternative control models rather than parts that combine, and this
// script is already raw on pitch and roll. It is re-asserted every tick in the
// loop below, so anything that takes the axis away is answered within a tick
// instead of leaving a lock that only looks like it is in force.

print "level_flight: holding " + round(target_alt) + " m ASL.".
print settings.
print gains.
print "  logging to " + flightlog + ".".
print "  abort (backspace) to hand controls back.".

// --- control loop ---------------------------------------------------------
// SIGN CHECK (pitch): this assumes positive ship:control:pitch commands
// nose-UP. If the plane dives into the ground on start, that convention is
// reversed for your build -- negate the pitch_pid output (or its gains).
local done      is false.
local last_note is time:seconds.
local rows      is 0.
until done {
  local now is time:seconds.

  // outer loop: altitude error -> pitch deviation from the engagement aoa
  set pitch_pid:setpoint to aoa_at_engage + alt_pid:update(now, ship:altitude).

  set ship:control:mainthrottle to cruise_throttle.

  // inner loop: pitch attitude -> elevator
  local pitch_ is pitch_angle().
  local elevator is pitch_pid:update(now, pitch_).
  set ship:control:pitch to elevator.

  // wings level (optional -- see hold_wings_level above)
  local aileron is 0.
  if hold_wings_level {
    set aileron to roll_pid:update(now, bank_angle()).
    set ship:control:roll to aileron.
  }

  // one state row a second, to the console and the log, for tuning feedback
  if now - last_note >= 1 {
    local gamma is angle_of_ascent().
    local row is list(round(now, 1),
                      round(ship:altitude, 1),
                      round(target_alt - ship:altitude, 1),
                      round(pitch_pid:setpoint, 2),
                      round(pitch_, 2),
                      round(gamma, 2),
                      round(pitch_ - gamma, 2),
                      round(elevator, 3),
                      round(bank_angle(), 2),
                      round(aileron, 3),
                      round(throttle, 3),
                      round(ship:control:mainthrottle, 3),
                      round(ship:airspeed, 1),
                      round(dynamic_pressure(), 2)).
    log commas(row) to flightlog.
    // reprint the header before it scrolls out of reach
    if mod(rows, 20) = 0 { print columns(col_names, col_width). }
    print columns(row, col_width).
    set rows to rows + 1.
    set last_note to now.
  }

  wait 0.
  if abort { set done to true. }
}

// hand the controls back to the pilot
set ship:control:pitch to 0.
set ship:control:roll  to 0.
set ship:control:neutralize to true.
afbw_restore(afbw_released).
print "level_flight: released.".
