// level_flight.ks -- hold level flight at the current altitude.
//
// The simplest useful aircraft autopilot. Everything that can be stripped
// away has been: no heading control, no waypoints, no speed loop. What's
// left is the irreducible core of "stay level where I am":
//
//   altitude error -> pitch command -> elevator     (two-loop cascade)
//   wings level    -> aileron                        (single loop, optional)
//
// Throttle stays the pilot's unless hold_throttle is set. The present
// altitude is captured once at start and held.
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
runoncepath("columns").  // columns()

// --- tunables -------------------------------------------------------------
// The pitch deviation from aoa_at_engage the outer loop may command, either
// way. The attitude the loop can reach is the datum plus or minus this, so the
// clamp bounds the loop's authority as much as its aggression. Chosen rather
// than derived: no flight has commanded more than about 7 degrees, so it has
// only ever bound a case where an inflated datum had already spent the
// allowance. The principled form is relative to prograde -- angle of attack is
// what a stall limits, not attitude -- which would make this a stall margin
// instead of a number.
local pitch_range is 30.
// Whether this script owns the throttle axis. Holding it fixed holds airspeed
// nearly constant, and the deflection an airframe needs to hold its attitude
// goes as 1/q, so a fixed throttle is what lets two flights be compared
// against each other. Leaving it to the pilot costs that comparability and
// nothing else: changing speed changes what the airframe needs, and the pitch
// loop's integral tracks it.
local hold_throttle is false.
local cruise_throttle  is 0.5.    // the setting hold_throttle pins the
                                  // throttle at; raise if the nose can't
                                  // hold altitude at this speed.
local hold_wings_level is true.   // set FALSE to test whether the airframe
                                  // holds its wings level on its own.

// Gains are named rather than written into the constructors so the log header
// can record them: a stack of tuning CSVs that does not say which gains
// produced each one cannot be compared against the next.
//
// Each is given below as the time constant it implies, checked against the
// aerodynamic mode it has to share the aircraft with. That states what a
// number means and makes a claim a flight can falsify. It is not a derivation
// of the number: these are values that have flown.

// alt error (m)  -> pitch deviation (deg), clamped to +/- pitch_range.
// alt_kp buys one degree of pitch deviation per twenty metres of error.
// alt_ki's time constant, alt_kp / alt_ki, is 50 s -- slow against the
// phugoid, the minutes-long trade of altitude for speed, so this term corrects
// drift without driving that mode. It is far too slow to supply the attitude
// level flight requires, which is why aoa_at_engage supplies that instead.
// alt_kd is zero. A derivative term here would answer a sink before it had
// become an altitude error, but the datum provides the command at engagement
// and the pitch integral holds it, so there is nothing left to reach sooner.
local alt_kp   is 0.05.
local alt_ki   is 0.001.
local alt_kd   is 0.
// pitch (deg)    -> elevator (-1..1)
// pitch_ki carries the deflection the airframe needs to hold its attitude,
// which a proportional term can only produce by standing off its setpoint.
// Its time constant, pitch_kp / pitch_ki, is 2 s: fast against the phugoid,
// which would carry the aircraft away while the term wound up, and slow
// against the short period, the roughly one-second pitch oscillation the
// derivative term damps. A deflection of 0.2 against an output clamped at 1
// leaves room to wind.
// pitch_kd's time constant, pitch_kd / pitch_kp, is 0.1 s -- well inside that
// short period, so the term damps the mode rather than lagging behind it.
// autotrim.ks reads pitch rate through this gain, so moving it also moves a
// threshold in that script.
local pitch_kp is 0.05.
local pitch_ki is 0.025.
local pitch_kd is 0.005.
// bank (deg)     -> aileron  (-1..1)
// roll_kd / roll_kp is 0.5 s, against roll subsidence -- the mode that damps a
// roll rate out unaided, faster still than the short period.
// roll_ki is zero because the standing error an integral would remove is
// negligible on a symmetric airframe: it needs almost no aileron to hold its
// wings level, so the error the proportional term carries to produce one stays
// small. Settled rows show a bank of 0.32 deg against an aileron of 0.003,
// which is roll_kp times that bank -- the same relation the pitch loop showed
// before it was given an integral, at a hundredth of the size.
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
// thr is the throttle the vessel is running; cthr is the throttle sitting in
// the raw control structure, which this script writes only while hold_throttle
// is set. Both are read back rather than assumed, because all three have been
// seen to disagree: cthr equal to cruise_throttle while thr differs puts the
// loss downstream of the control structure, and cthr differing too puts it
// upstream.
// q is dynamic pressure, air density times airspeed squared over two -- the
// pressure the air exerts on a surface held across the flow, and so what sets
// how much force a given elevator deflection produces. The deflection needed
// to hold an attitude goes as 1/q, which makes elev alone meaningless across
// conditions: two flights are only comparable through elev x q.
local col_names is list("t", "alt", "err", "cmd", "pitch", "gamma", "aoa",
                        "elev", "bank", "ail", "thr", "cthr", "tas", "q").
local col_width is list(8, 8, 7, 6, 6, 6, 6, 7, 7, 7, 6, 7, 6, 8).

// --- setup ----------------------------------------------------------------
// KSP's abort action group is a toggle, and the control loop below ends when
// it reads true. A run stopped with it therefore leaves it set, so the next
// run would end on its first tick, having written one tick of controls and no
// log rows. Clear it, and say so: a script that silently unsets an action
// group its pilot set is harder to reason about than one that announces it.
if abort {
  print "level_flight: abort was latched from an earlier run; clearing it.".
  set abort to false.
}

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
    + "  hold_throttle " + hold_throttle
    + "  cruise_throttle " + cruise_throttle
    + "  hold_wings_level " + hold_wings_level
    + "  afbw_released " + afbw_released.
local gains is "# gains (kp ki kd)  alt " + alt_kp + " " + alt_ki + " " + alt_kd
    + "  pitch " + pitch_kp + " " + pitch_ki + " " + pitch_kd
    + "  roll " + roll_kp + " " + roll_ki + " " + roll_kd.
local flightlog is "level_flight_" + round(time:seconds) + ".csv".
log settings to flightlog.
log gains to flightlog.
log col_names:join(",") to flightlog.

// While hold_throttle is set the throttle goes through the raw control
// structure rather than LOCK THROTTLE: the two are alternative control models
// rather than parts that combine, and this script is already raw on pitch and
// roll. It is re-asserted every tick in the loop below, so anything that takes
// the axis away is answered within a tick instead of leaving a lock that only
// looks like it is in force. Clear, the script never touches the axis and the
// throttle answers to the pilot alone.

print "level_flight: holding " + round(target_alt) + " m ASL.".
print settings.
print gains.
print "  logging to " + flightlog + ".".
print "  abort (backspace) to hand controls back.".

// --- control loop ---------------------------------------------------------
// Let go of every axis before taking the ones this script flies. The raw
// control structure belongs to the vessel, not to the program, so an axis
// keeps whatever value was last written into it -- including by a previous
// run -- and kOS goes on applying that value to any script holding raw
// control, whether or not this one writes the axis. Neutralize is what
// releases an axis; leaving it alone does not, and writing zero to it is a
// command, not a release.
set ship:control:neutralize to true.

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

  if hold_throttle { set ship:control:mainthrottle to cruise_throttle. }

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
    log row:join(",") to flightlog.
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
