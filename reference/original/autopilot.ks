// autopilot.ks -- hold altitude and speed while following a list of
// waypoints.
//
// Five loops in the classic cascade:
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
// route below before flying.
//
// If this script exits without printing "released" -- an uncaught
// exception -- AFBW stays switched off and the raw axes stay at whatever
// they last held; recover from AFBW's own toolbar button.

@lazyglobal off.

runoncepath("aero").   // pitch_angle(), bank_angle(), angle_of_ascent(),
                       // heading_error(), ground_distance(), dynamic_pressure()
runoncepath("afbw").   // afbw_release(), afbw_restore(), afbw_throttle_release_bound()
runoncepath("columns").  // columns()

// Select a subset of a list's elements by index, so a printed row can leave
// columns out without ever disagreeing with the CSV row it came from --
// same values, same names, same widths, fewer of them.
function subset {
  parameter lst, idx.
  local out is list().
  local i is 0.
  until i >= idx:length {
    out:add(lst[idx[i]]).
    set i to i + 1.
  }
  return out.
}

// --- the route ------------------------------------------------------------
// Replace with your own coordinates: latlng(latitude, longitude).
local route is list(
  latlng(-0.05, -74.0),
  latlng( 0.50, -73.0),
  latlng( 1.20, -72.0)
).
// metres; count a waypoint reached inside this. Unflown, and small against
// the turn radius hdg_pid's +/-25 deg bank clamp implies (on the order of
// 17 km at cruise speed) -- expect arrival by the off-nose branch below,
// not this radius, until flown.
local arrival_radius is 500.

// --- loops ----------------------------------------------------------------
// Gains below are unflown, chosen with no flight behind any of them.
local hdg_pid   is pidloop(1.5,  0,     0,     -25, 25).  // hdg err -> bank cmd
local alt_pid   is pidloop(0.05, 0.001, 0,     -12, 12).  // alt err -> pitch cmd
local spd_pid   is pidloop(0.1,  0.01,  0,      0,  1).   // spd err -> throttle
// Contradicts level_flight.ks's flown pitch gains (0.05/0.025/0.005) -- that
// flight is not evidence for these.
local pitch_pid is pidloop(0.01, 0,     0.005, -1,  1).   // pitch   -> elevator
local roll_pid  is pidloop(0.01, 0,     0.005, -1,  1).   // bank    -> aileron

// --- state row ------------------------------------------------------------
// One row of state per second, rendered two ways from one list of values so
// the console and the CSV cannot drift apart. Column widths hold the widest
// value each column can produce; a value that outgrows its width loses its
// leading space and nothing else.
//
// cmd is the pitch attitude alt_pid is asking pitch_pid to hold; pitch is
// what the airframe is actually holding -- cmd tracking pitch means the
// inner loop is delivering what it was asked, cmd lagging pitch means it
// cannot. gamma is the velocity vector's angle above the horizon (v_vert =
// tas * sin(gamma)); aoa is pitch minus gamma, the nose's angle off the
// velocity vector in the vertical plane, signed because a stall limit needs
// a sign.
// bankcmd is hdg_pid's output, the bank roll_pid is asked to hold; bank is
// bank_angle(), what the airframe is actually holding; ail is the aileron
// holding it. beta is sideslip, the angle between the nose and the relative
// wind in the horizontal plane -- positive means the wind is from the
// right. A settled turn with a standing bankcmd-bank gap and a matching ail
// answers whether that demand is adverse yaw (beta nonzero, same sign as
// ail -- the fix is a yaw channel) or a genuine standing bank error (beta
// near zero -- the fix is roll_ki). This script never writes
// ship:control:yaw.
// hdgerr is the signed error to the active waypoint's bearing
// (heading_error()). wp is the route index being sought, 0-based; it holds
// at route:length once the route is flown and nothing is being sought. dist
// is ground_distance() to that waypoint, 0 once the route is flown.
// sperr is the speed loop's error, target_spd less the surface speed
// spd_pid is holding to. thr is the throttle the vessel is running, the
// independent witness; cthr is ship:control:mainthrottle read back in the
// same tick it was written, so it echoes the command unless AFBW or some
// other axis-writer is also touching the throttle.
// q is dynamic pressure, air density times airspeed squared over two. The
// deflection an airframe needs to hold an attitude goes as 1/q, so elev and
// ail alone are not comparable across conditions without it.
local col_names is list("t", "alt", "err", "cmd", "pitch", "gamma", "aoa",
                        "elev", "bankcmd", "bank", "ail", "beta", "hdgerr",
                        "wp", "dist", "sperr", "thr", "cthr", "tas", "q").
local col_width is list(8, 8, 7, 6, 6, 6, 6, 7, 8, 7, 7, 7, 7, 3, 9, 7, 6, 7, 6, 8).

// The console is narrower than the full row -- fewer columns fit before the
// terminal wraps. This picks the columns that show whether the heading and
// throttle loops are behaving in the first few seconds; the CSV keeps every
// column regardless.
local console_idx is list(0, 1, 2, 8, 9, 10, 12, 13, 14, 15, 16, 17).

// --- setup ----------------------------------------------------------------
// KSP's abort action group is a toggle, and the control loop below ends when
// it reads true. A run stopped with it therefore leaves it set, so the next
// run would end on its first tick, having written one tick of controls and no
// log rows. Clear it, and say so: a script that silently unsets an action
// group its pilot set is harder to reason about than one that announces it.
if abort {
  print "autopilot: abort was latched from an earlier run; clearing it.".
  set abort to false.
}

// Take the axes off AFBW before anything else: it wins the arbitration
// against kOS, so a raw control command written while it holds an axis is
// obeyed by nothing but the variable that reads it back.
local afbw_released is afbw_release().

// SAS holds an attitude of its own by writing the same axes this script
// writes, so the two fight, and SAS wins. Switched off for the run, and
// handed back only if this script was what took it, so a pilot who had it
// off keeps it off.
local sas_was_on is sas.
if sas_was_on { set sas to false. }

local target_alt    is ship:altitude.               // hold present altitude (ASL)
local target_spd    is ship:velocity:surface:mag.   // hold present speed (m/s)
local thr_at_engage is throttle.                    // hold present throttle
set alt_pid:setpoint to target_alt.
set spd_pid:setpoint to target_spd.

// The correction below adds to thr_at_engage rather than commanding
// throttle outright, so its reachable range is what's left between closed
// and full throttle from that setting, not the (0, 1) spd_pid was
// constructed with. AFBW's own throttle axis re-ranges itself the same way,
// every tick, against whatever mainThrottle currently reads.
set spd_pid:minoutput to -thr_at_engage.
set spd_pid:maxoutput to 1 - thr_at_engage.

local idx     is 0.                    // waypoint we're chasing
local arrived is route:length = 0.     // true once the last waypoint is
                                       // reached, or immediately if there is
                                       // no route to fly

// Lines beginning '#' are the settings the flight is judged against.
// thr_at_engage is what the throttle loop's correction is added to, the
// same role aoa_at_engage plays for pitch in level_flight.ks: a loop
// captured at zero error commands zero, not "hold what you have", so the
// datum supplies what the loop cannot.
local settings is "# target_alt " + round(target_alt) + " m ASL"
    + "  target_spd " + round(target_spd, 1) + " m/s"
    + "  thr_at_engage " + round(thr_at_engage, 3)
    + "  pilotpitchtrim " + round(ship:control:pilotpitchtrim, 4)
    + "  arrival_radius " + arrival_radius + " m"
    + "  waypoints " + route:length
    + "  afbw_released " + afbw_released
    + "  throttle_release_bound " + afbw_throttle_release_bound()
    + "  sas_was_on " + sas_was_on.
local gains is "# gains (kp ki kd)  hdg " + hdg_pid:kp + " " + hdg_pid:ki + " " + hdg_pid:kd
    + "  alt " + alt_pid:kp + " " + alt_pid:ki + " " + alt_pid:kd
    + "  spd " + spd_pid:kp + " " + spd_pid:ki + " " + spd_pid:kd
    + "  pitch " + pitch_pid:kp + " " + pitch_pid:ki + " " + pitch_pid:kd
    + "  roll " + roll_pid:kp + " " + roll_pid:ki + " " + roll_pid:kd.
// alt_pid's clamp is the pitch cascade's known saturation defect; hdg_pid's
// sets the turn radius. Both bound what a flight can reach before anything
// else does.
local limits is "# limits (min max)  hdg " + hdg_pid:minoutput + " " + hdg_pid:maxoutput
    + "  alt " + alt_pid:minoutput + " " + alt_pid:maxoutput.
local flightlog is "autopilot_" + round(time:seconds) + ".csv".
log settings to flightlog.
log gains to flightlog.
log limits to flightlog.
log col_names:join(",") to flightlog.

print "autopilot: " + route:length + " waypoints, holding "
    + round(target_alt) + " m ASL at " + round(target_spd) + " m/s.".
if route:length = 0 {
  print "  route is empty -- loitering (wings level) from the start.".
}
print settings.
print gains.
print limits.
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

// SIGN CHECK (roll): this loop is already flown -- level_flight.ks runs the
// identical bank_angle() -> roll_pid -> ship:control:roll pipeline. A wrong
// sign here is positive feedback, not a wrong-way bank: the aircraft rolls
// off continuously to the aileron stop instead of settling. If that
// happens, negate roll_pid's output where it reaches ship:control:roll; not
// bank_angle(), which other scripts have flown.
// SIGN CHECK (heading): the aircraft should roll toward the waypoint and
// hdgerr, dist should both shrink within a second or two of engaging. If
// they grow instead, the loop is feeding back positively and the run is
// over -- negate hdg_pid's output where it sets roll_pid:setpoint, not
// heading_error().
local done      is false.
local last_note is time:seconds.
local rows      is 0.
until done {
  local now is time:seconds.

  // GUIDANCE: pick a bank command. While there are waypoints left, aim at
  // the current one (wp:heading = compass bearing to it). Once the route is
  // done, loiter: command wings level and just hold altitude and speed.
  local target_bearing is 0.
  local hdg_err        is 0.
  local dist           is 0.
  if not arrived {
    local wp is route[idx].
    set target_bearing to wp:heading.
    set hdg_err to heading_error(target_bearing).
    set dist to ground_distance(wp).

    // ARRIVAL: reached if inside the radius, OR the waypoint has slipped
    // more than 90 deg off the nose while we're already close (an abeam
    // pass we'd otherwise loop back for). The distance gate stops a bad
    // initial heading from skipping waypoints.
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

  // outer loops -> inner setpoints. kOS's PIDLoop computes setpoint - input,
  // so hdg_pid, fed hdg_err as its input against a setpoint of zero, returns
  // the negative of the bank wanted -- negate it back.
  set roll_pid:setpoint  to -hdg_pid:update(now, hdg_err).
  set pitch_pid:setpoint to alt_pid:update(now, ship:altitude).

  // inner loops -> surfaces and throttle
  local bank_    is bank_angle().
  local pitch_   is pitch_angle().
  local aileron  is roll_pid:update(now, bank_).
  local elevator is pitch_pid:update(now, pitch_).
  set ship:control:roll  to aileron.
  set ship:control:pitch to elevator.
  // spd_pid carries the correction from thr_at_engage rather than commanding
  // throttle outright, for the same reason alt_pid needed aoa_at_engage in
  // level_flight.ks: captured at zero error, an uncorrected loop would
  // command zero throttle for as long as it takes the integral to wind up.
  set ship:control:mainthrottle to thr_at_engage + spd_pid:update(now, ship:velocity:surface:mag).

  // one state row a second, to the console and the log, for tuning feedback
  if now - last_note >= 1 {
    local gamma is angle_of_ascent().
    local beta  is arcsin(vdot(ship:facing:starvector, ship:velocity:surface:normalized)).
    local row is list(round(now, 1),
                      round(ship:altitude, 1),
                      round(target_alt - ship:altitude, 1),
                      round(pitch_pid:setpoint, 2),
                      round(pitch_, 2),
                      round(gamma, 2),
                      round(pitch_ - gamma, 2),
                      round(elevator, 3),
                      round(roll_pid:setpoint, 2),
                      round(bank_, 2),
                      round(aileron, 3),
                      round(beta, 2),
                      round(hdg_err, 1),
                      idx,
                      round(dist, 1),
                      round(target_spd - ship:velocity:surface:mag, 1),
                      round(throttle, 3),
                      round(ship:control:mainthrottle, 3),
                      round(ship:airspeed, 1),
                      round(dynamic_pressure(), 2)).
    log row:join(",") to flightlog.
    // reprint the header before it scrolls out of reach
    if mod(rows, 20) = 0 { print columns(subset(col_names, console_idx), subset(col_width, console_idx)). }
    print columns(subset(row, console_idx), subset(col_width, console_idx)).
    set rows to rows + 1.
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
afbw_restore(afbw_released).
if sas_was_on { set sas to true. }
print "autopilot: released.".
