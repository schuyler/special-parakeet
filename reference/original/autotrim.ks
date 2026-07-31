// autotrim.ks -- set the craft's pitch trim so it holds level flight with the
// elevator near neutral, then hand the controls back.
//
// Trim is the control deflection an airframe needs to hold its attitude with
// the stick centred. It is not a property of the craft alone: the deflection
// required falls as dynamic pressure rises, so this trims for the condition
// the craft is in now, and wants re-running when speed or altitude move far.
//
// The method measures the fast mode and never waits for the slow one. An
// aircraft has two pitch modes: the short period (angle of attack against the
// tail's restoring moment, settling in about a second) and the phugoid
// (altitude traded against speed, which can run to minutes). Servoing trim
// against vertical speed fights the phugoid and takes minutes. Instead a pitch
// loop holds the nose at level flight, its elevator output settles on the
// short period, and that settled deflection is copied into trim.
//
// At rest the pitch rate is zero, so the airframe's total pitching moment is
// zero: elev + trim = deflection_required, and a pass adds
// d_trim = deflection_required - trim. With the trim axis correctly signed
// and scaled against the elevator, one pass measures the whole deflection and
// leaves nothing for a second pass to find -- max_pass defaults to 1 because
// that is the experiment. The normal end at max_pass 1 is therefore
// no_convergence, not converged: one pass measures and writes a correction,
// and that is what the pilot is handed unless the release window right after
// it shows the write was wrong (see below).
//
// d_trim_{n+1} = (1 - k) * d_trim_n, where k is how much of a pass's write
// actually reached the surfaces. 0 < k < 1 shrinks the sequence, same sign
// pass to pass -- a trim axis weaker than the elevator, converging slowly.
// 1 < k < 2 also shrinks, but alternates sign -- overshoot on a matched or
// overscaled axis. k <= 0 or k >= 2 grows the sequence and is a sign or scale
// error; k == 0, no authority at all, instead holds it flat, which a growth
// check alone cannot see and is classified separately below. Required
// deflection also falls as dynamic pressure rises, so a sequence that
// shrinks while q is rising has the same shape as a zero-authority axis
// would in that same rising-q run -- q, alt, and tas are all logged so the
// two are not told apart from tas alone.
//
// Trim reaches the control surfaces only through the PILOT's control
// structure; ship:control:pitchtrim is inert (notes/kos-facts.md). Whether
// trim acts at all while a script holds raw control is unverified, so the
// controls are fully released between passes -- setting an axis to zero is not
// the same as letting go of it.
//
// AFBW must be off for the whole run: a RELEASE window with a stick still on
// the axis is not a release, and AFBW writes the pilot control structure that
// pilotpitchtrim lives in, so it can clobber the trim this script is writing.
// The run refuses to start if AFBW is available and still reads enabled after
// afbw_release() (afbw.ks). That check runs once, at setup; every logged row
// also carries whether AFBW reads enabled at that instant, so a flight where
// it comes back on mid-run is visible after the fact even though the run
// itself does not watch for it live.
//
// Every outcome that leaves trim unvalidated -- abort, a settle timeout, a
// growing sequence, a flat (no-authority) sequence, the trim ceiling, a bank
// past release_bank_tol at release, gamma moving during release itself, or
// AFBW refusing to switch off -- restores pilotpitchtrim to the value it
// read on entry before handing the controls back. Running out of passes
// without hitting any of those is not on that list: every pass that reached
// a write had already cleared them all, so the trim accumulated so far is
// validated and is what the pilot is handed.

@lazyglobal off.

runoncepath("aero").   // pitch_angle(), bank_angle(), angle_of_ascent(), dynamic_pressure()
runoncepath("afbw").   // afbw_release(), afbw_restore()
runoncepath("columns").  // columns()

// --- tunables -------------------------------------------------------------
// A window counts as steady only if it spans several short-period cycles, so
// that a flat reading is settling rather than a momentary crossing. The short
// period is order 1 s for anything of aircraft scale.
local settle_window is 2.
// Width the bracketed window must close to before a pass counts as settled.
// The bracket tracks the pitch loop's P-term (see SETTLE below), which is
// free of the D-term's tick-to-tick noise, so this bound is on gamma's own
// jitter times pitch_kp, not on the elevator's raw output. Stated in the
// loop's -1..1 output range, so it applies unchanged to any airframe.
local settle_window_tol is 0.005.
// Pitch rate, degrees per second, below which the window's D-term peak
// counts as at rest (see SETTLE below). Kept apart from settle_window_tol:
// the D-term is pitch_kd times pitch rate, so a single shared tolerance
// would bound pitch rate at settle_window_tol / pitch_kd, moving that bound
// every time pitch_kd is retuned without anyone deciding to move it. Stated
// in degrees per second, so it applies unchanged to any airframe. Unflown:
// the one flight on record was flown before this condition existed, and
// settled on the bracket alone.
local settle_pitch_rate_tol is 1.
// Deflection step below which a pass's correction is not worth writing: it
// would move the elevator less than the loop's own output resolution. Same
// units and the same craft-free reasoning as settle_window_tol; kept as a
// separate tunable so widening one does not silently widen the other.
local trim_residual_tol is 0.005.
// Ceiling on abs(trim). 1.0 is full authority, where the axis is saturated
// and there is nothing left to converge toward. An airframe that needs more
// than half its trim throw just to hold level is either sign-flipped or out
// of the range this script can fix. Dimensionless, in trim's own -1..1
// control-authority units, so it applies unchanged to any airframe.
local trim_ceiling is 0.5.
// Tolerance band, as a fraction of the previous pass's deflection, on the
// pass-to-pass ratio used to classify the deflection sequence (see header):
// a ratio within this band of 1 reads as k ~= 0, no authority, rather than
// as growth from a sign or scale error. Dimensionless, so it applies
// unchanged to any airframe.
local shrink_tol is 0.1.
// Abandon a pass whose elevator never goes steady: turbulence, or an airframe
// that is not statically stable at this condition. A pass that hits this ends
// the run without writing trim -- the window it would write from is whatever
// was open when the clock ran out, not a settled one.
local settle_timeout is 15.
// How long the airframe gets to answer a trim step before the next
// measurement. The answer is short-period, so a few seconds is generous.
local release_window is 3.
// Sampling interval inside RELEASE. At 1 Hz the 3 s window gives 3 points,
// too few to tell a flat trace from one already sliding within a second or
// two -- exactly the shape release_gamma_tol below is checking for. Seconds,
// free of the craft and the body.
local release_log_dt is 0.25.
// Bank the roll loop must already be holding, degrees, before a RELEASE
// window is allowed to let go of it too. A tolerance on the roll loop's own
// zero setpoint, not a structural limit, so it is free of the craft and the
// body.
local release_bank_tol is 10.
// Flight path angle may move, degrees, between the SETTLE measurement and
// the end of the RELEASE window that follows it. SETTLE actively holds the
// nose, so a bad trim cannot show up there; RELEASE is the only window where
// the airframe flies on trim alone, and over release_window -- a few
// seconds, short against the phugoid's order-of-a-minute period -- a
// correctly trimmed aircraft drifts only a small fraction of that period's
// excursion. A short-period runaway under a bad trim is much larger than
// that inside the same few seconds, which is what this catches. Degrees,
// free of the craft and the body.
local release_gamma_tol is 2.
// One pass is the whole experiment this flight is asking: whether the craft
// is already close to converged on a single pass, per the equilibrium
// argument above, is itself the result -- and every further pass runs while
// dynamic pressure keeps moving the answer, so more passes would mostly
// measure that drift rather than trim authority. Raise this once a first
// flight's q/alt/thr columns show the drift is small enough that a second
// pass would still be measuring the same condition.
local max_pass is 1.
local hold_wings_level is true.

// pitch (deg) -> elevator (-1..1), and bank (deg) -> aileron: the same inner
// loops level_flight.ks flies. Rebuilt each pass so the derivative term does
// not carry history across the gap where the controls were released.
local pitch_kp is 0.05.
local pitch_kd is 0.005.
local roll_kp  is 0.01.
local roll_kd  is 0.005.
local pitch_pid is pidloop(pitch_kp, 0, pitch_kd, -1, 1).
local roll_pid  is pidloop(roll_kp,  0, roll_kd,  -1, 1).

// --- state row ------------------------------------------------------------
// Rendered two ways from one list of values so the console and the CSV cannot
// drift apart. elev is the loop's full PID output, the actual command sent to
// the surface; on a SETTLE/TIMEOUT row it instead holds the settled
// deflection that pass measured. trim is read back from the pilot control
// structure rather than assumed. q and alt are dynamic pressure and altitude,
// both of which the required deflection moves with (see header); thr is the
// throttle the vessel is actually running, since this script never commands
// it and afbw_release() changes who owns that axis at t=0. timeout is 1 on
// the row where a pass's settle window ran out before the elevator steadied.
// win is the current bracket width (win_max - win_min): live on settle rows,
// 0 on release rows -- what a timed-out pass was still closing on. dterm is
// its counterpart for the loop's derivative term: the window's peak
// abs(pitch_pid:dterm) so far, which is pitch_kd times the pitch rate, live
// on settle rows and 0 on release rows. afbw is whether AFBW reads enabled
// at that instant, logged every row because the setup-time refusal only
// guards against AFBW already being back on, not against it returning
// mid-run.
//
// Lowercase phase names (settle, release) are a periodic sample within a
// pass; uppercase ones (SETTLE, TIMEOUT) are the one row a pass's own
// measurement is decided on.
local col_names is list("t", "phase", "pass", "pitch", "gamma", "aoa",
                        "elev", "trim", "bank", "tas", "q", "alt", "thr",
                        "timeout", "win", "dterm", "afbw").
local col_width is list(8, 8, 5, 7, 7, 6, 7, 7, 7, 7, 9, 8, 6, 8, 7, 7, 6).

// Timestamped so a second run this session does not erase the first one's
// only record (notes/level-flight-tuning.md logs this as a known cost).
local flightlog is "autotrim_" + round(time:seconds) + ".csv".

function log_row {
  parameter now, phase, pass_no, elev, pass_timed_out, win_width, dterm_peak.
  local pitch_ is pitch_angle().
  local gamma  is angle_of_ascent().
  local timeout_flag is 0.
  if pass_timed_out { set timeout_flag to 1. }
  local afbw_on is 0.
  if addons:available("afbw") {
    if addons:afbw:enabled { set afbw_on to 1. }
  }
  local row is list(round(now, 1),
                    phase,
                    pass_no,
                    round(pitch_, 2),
                    round(gamma, 2),
                    round(pitch_ - gamma, 2),
                    round(elev, 3),
                    round(ship:control:pilotpitchtrim, 3),
                    round(bank_angle(), 2),
                    round(ship:airspeed, 1),
                    round(dynamic_pressure(), 1),
                    round(ship:altitude, 1),
                    round(throttle, 3),
                    timeout_flag,
                    round(win_width, 4),
                    round(dterm_peak, 4),
                    afbw_on).
  log row:join(",") to flightlog.
  print columns(row, col_width).
}

// --- setup ----------------------------------------------------------------
// KSP's abort action group is a toggle, and every loop below ends when it
// reads true. A run stopped with it therefore leaves it set, so the next run
// would end on its first tick with nothing measured. Clear it, and say so: a
// script that silently unsets an action group its pilot set is harder to
// reason about than one that announces it.
if abort {
  print "autotrim: abort was latched from an earlier run; clearing it.".
  set abort to false.
}

// AFBW has to be off for the whole run, not just the passes: during a RELEASE
// window the airframe is meant to be flown by trim alone, and a stick still on
// the axis would be answering instead. If it is still enabled after the
// release call -- present but refusing to switch off -- the run refuses to
// start rather than fly a RELEASE window with AFBW still on the axis.
local afbw_released is afbw_release().
local afbw_stuck is false.
if addons:available("afbw") {
  set afbw_stuck to addons:afbw:enabled.
}

// SAS has to be off for the same reason, and the consequence here is worse
// than contention. A RELEASE window measures whether trim alone holds the
// nose; SAS holds the nose too, by writing the elevator this script has just
// let go of. A run flown with SAS on would show gamma flat across every
// release window and report that trim has authority, whether or not it does --
// the one answer the run exists to produce, and no way to tell it from the
// real one afterward. Logged in the settings line so a CSV says which it was.
local sas_was_on is sas.
if sas_was_on { set sas to false. }

// Entry trim: restored if the run ends in an unsafe stop (see header). trim
// is the running accumulator written to pilotpitchtrim each pass.
local trim0 is ship:control:pilotpitchtrim.
local trim  is trim0.

local stop_reason is "".
if afbw_stuck { set stop_reason to "afbw". }

local settings is "# autotrim  settle_window " + settle_window
    + " s  settle_window_tol " + settle_window_tol
    + "  settle_pitch_rate_tol " + settle_pitch_rate_tol + " deg/s"
    + "  trim_residual_tol " + trim_residual_tol
    + "  trim_ceiling " + trim_ceiling
    + "  shrink_tol " + shrink_tol
    + "  settle_timeout " + settle_timeout + " s"
    + "  release_window " + release_window + " s"
    + "  release_log_dt " + release_log_dt + " s"
    + "  release_bank_tol " + release_bank_tol + " deg"
    + "  release_gamma_tol " + release_gamma_tol + " deg"
    + "  max_pass " + max_pass
    + "  hold_wings_level " + hold_wings_level
    + "  pitch_pid " + pitch_kp + " 0 " + pitch_kd
    + "  roll_pid " + roll_kp + " 0 " + roll_kd
    + "  trim0 " + round(trim0, 4)
    + "  afbw_released " + afbw_released
    + "  afbw_stuck " + afbw_stuck
    + "  sas_was_on " + sas_was_on.
log settings to flightlog.
log col_names:join(",") to flightlog.

print "autotrim: measuring the deflection this craft needs.".
print settings.
if afbw_stuck {
  print "  AFBW is enabled and refused to switch off -- refusing to run.".
} else {
  print "  abort (backspace) to hand controls back.".
  print columns(col_names, col_width).
}

// --- passes ---------------------------------------------------------------
local pass_no is 0.
local history is list().   // the settled deflection each pass measured

until stop_reason <> "" or pass_no >= max_pass {
  set pass_no to pass_no + 1.
  set pitch_pid to pidloop(pitch_kp, 0, pitch_kd, -1, 1).
  set roll_pid  to pidloop(roll_kp,  0, roll_kd,  -1, 1).
  set roll_pid:setpoint to 0.               // wings level

  // SETTLE: aim the nose where the velocity vector would sit on the horizon --
  // present attitude minus present flight path angle -- and wait for the
  // pitch loop's proportional term to stop moving.
  //
  // Taking the setpoint from the present attitude makes the loop's error
  // exactly minus the flight path angle, so this is a proportional loop on
  // gamma. At rest the pitch rate is zero, so the airframe's total pitching
  // moment is zero: elev + trim = deflection_required, and the loop's P-term
  // equals elev exactly at that point, because the D-term is itself
  // proportional to pitch rate and so is zero there too. Each pass hands the
  // deflection to trim, so the loop needs less of it and gamma settles
  // nearer zero on the pass after.
  //
  // win_min/win_max bracket the P-term (pitch_kp * error) rather than the
  // raw PID output: the raw output also carries the D-term, which reacts to
  // pitch-rate noise every physics tick and swamps a tolerance this tight
  // long before the airframe is actually settled. The P-term converges to
  // the same value at rest (see above) without that noise riding on it.
  //
  // A narrow P-term bracket alone is not sufficient: the setpoint is
  // recomputed from the same tick's pitch_, so pitch_ cancels out of the
  // error exactly, and P can sit flat while the airframe is still ringing in
  // angle of attack and pitch rate at a nearly constant gamma -- elev would
  // then be swinging on the D-term while P looks settled. kOS takes the PID
  // derivative on the measurement, not the error, so the D-term is exactly
  // -pitch_kd * pitch rate: a single tick of it can land on a zero crossing
  // of pitch rate while the airframe is still ringing, so the window tracks
  // its peak magnitude the same way it brackets the P-term, and closes only
  // once that peak has stayed under settle_pitch_rate_tol (converted to the
  // D-term's own units via pitch_kd) for the whole window -- the same claim
  // as "pitch rate is near zero throughout" that licenses reading d_trim as
  // elev's rest value. A window that closes narrow enough on both terms is
  // settled; one that does not starts a fresh window.
  local t0        is time:seconds.
  local win_start is t0.
  local win_min   is 1e9.
  local win_max   is -1e9.
  local win_dterm_max  is 0.
  local elev      is 0.
  local settled   is false.
  local pass_timed_out is false.
  local last_row  is 0.

  until settled or stop_reason <> "" {
    local now    is time:seconds.
    local pitch_ is pitch_angle().
    set pitch_pid:setpoint to pitch_ - angle_of_ascent().
    set elev to pitch_pid:update(now, pitch_).
    set ship:control:pitch to elev.
    if hold_wings_level {
      set ship:control:roll to roll_pid:update(now, bank_angle()).
    }

    local p_term is pitch_kp * (pitch_pid:setpoint - pitch_).
    set win_min to min(win_min, p_term).
    set win_max to max(win_max, p_term).
    set win_dterm_max to max(win_dterm_max, abs(pitch_pid:dterm)).
    if now - win_start >= settle_window {
      if win_max - win_min < settle_window_tol
          and win_dterm_max < settle_pitch_rate_tol * pitch_kd {
        set settled to true.
      } else {
        set win_start to now.
        set win_min to p_term.
        set win_max to p_term.
        set win_dterm_max to abs(pitch_pid:dterm).
      }
    }
    if not settled and now - t0 >= settle_timeout {
      set settled to true.
      set pass_timed_out to true.
    }

    if now - last_row >= 1 {
      log_row(now, "settle", pass_no, elev, false, win_max - win_min, win_dterm_max).
      set last_row to now.
    }
    wait 0.
    if abort { set stop_reason to "aborted". }
  }

  // A pass that ended on abort never reaches here, so it never writes trim.
  if stop_reason = "" {
    // The window's midpoint rather than the last tick: less swayed by noise.
    local d_trim is (win_min + win_max) / 2.
    local gamma_settle is angle_of_ascent().
    local settle_phase is "SETTLE".
    if pass_timed_out { set settle_phase to "TIMEOUT". }
    log_row(time:seconds, settle_phase, pass_no, d_trim, pass_timed_out, win_max - win_min, win_dterm_max).
    // A timed-out window can hold as little as one sample of a still-moving
    // elevator -- not a measurement, so it never enters the printed sequence.
    if not pass_timed_out {
      history:add(round(d_trim, 4)).
    }

    if pass_timed_out {
      set stop_reason to "timeout".
    } else if abs(d_trim) <= trim_residual_tol {
      set stop_reason to "converged".
    } else {
      if pass_no >= 2 {
        local prev_d_trim is history[history:length - 2].
        local ratio is abs(d_trim) / abs(prev_d_trim).
        if ratio > 1 + shrink_tol {
          // SIGN CHECK: a deflection sequence that grows means
          // trim_next = trim + (deflection_required - k*trim) has k <= 0 or
          // k >= 2 -- the trim axis is inverted or scaled more than double
          // the elevator's authority -- not that the axis lacks authority.
          // Flip pilotpitchtrim's sign (or its scale, if this axis is ever
          // regeared) and re-run before trusting a release window.
          set stop_reason to "diverged".
        } else if ratio > 1 - shrink_tol {
          // A ratio within shrink_tol of 1 is k ~= 0: the write is not
          // reaching the surfaces at all, not a sign or scale error.
          set stop_reason to "no_authority".
        }
      }

      if stop_reason = "" {
        local trim_next is trim + d_trim.
        if abs(trim_next) > trim_ceiling {
          set stop_reason to "trim_limit".
        } else if hold_wings_level and abs(bank_angle()) > release_bank_tol {
          set stop_reason to "bank".
        } else {
          // RELEASE: let go of every axis so the trim step can reach the
          // surfaces. Neutralize before writing trim -- whether NEUTRALIZE
          // clears the pilot control structure is undocumented
          // (notes/kos-facts.md), so writing trim first and neutralizing
          // after risks neutralize erasing the write this pass exists to
          // make. Trim is written once, not per tick, so the trim column
          // stays an independent witness rather than an echo of this
          // command. Roll goes with it -- an axis held at zero may still be
          // an axis kOS owns.
          set ship:control:neutralize to true.
          set trim to trim_next.
          set ship:control:pilotpitchtrim to trim.

          local t1 is time:seconds.
          set last_row to t1.
          until time:seconds - t1 >= release_window or stop_reason <> "" {
            local now is time:seconds.
            if now - last_row >= release_log_dt {
              log_row(now, "release", pass_no, 0, false, 0, 0).
              set last_row to now.
            }
            wait 0.
            if abort { set stop_reason to "aborted". }
          }
          log_row(time:seconds, "release", pass_no, 0, false, 0, 0).

          // RELEASE is the only window that can expose a bad trim -- SETTLE
          // was actively holding the nose, so a wrong write would not have
          // shown up there. If gamma moved more than release_gamma_tol while
          // the airframe flew on trim alone, the short-period is running
          // away under the trim just written, and that write is not kept.
          if stop_reason = "" {
            local gamma_release is angle_of_ascent().
            if abs(gamma_release - gamma_settle) > release_gamma_tol {
              set stop_reason to "release_diverged".
            }
          }
        }
      }
    }
  }
}

if stop_reason = "" {
  set stop_reason to "no_convergence".
}

// --- hand back --------------------------------------------------------------
// Neutralize before touching trim, for the same reason as inside the loop:
// NEUTRALIZE's effect on the pilot control structure is undocumented, so
// trim is restored only after it, never before.
set ship:control:pitch to 0.
set ship:control:roll  to 0.
set ship:control:neutralize to true.
local unsafe_stop is stop_reason = "aborted" or stop_reason = "timeout"
    or stop_reason = "diverged" or stop_reason = "no_authority"
    or stop_reason = "trim_limit" or stop_reason = "bank"
    or stop_reason = "release_diverged" or stop_reason = "afbw".
if unsafe_stop {
  set ship:control:pilotpitchtrim to trim0.
}
afbw_restore(afbw_released).
if sas_was_on { set sas to true. }

local outcome is "".
if stop_reason = "converged" {
  set outcome to "converged in " + pass_no + " pass(es).".
} else if stop_reason = "no_convergence" {
  set outcome to "measured " + pass_no + " pass(es); residual still above "
      + trim_residual_tol + " -- fly again to continue or confirm.".
} else if stop_reason = "aborted" {
  set outcome to "aborted -- pilotpitchtrim restored to " + round(trim0, 4) + ".".
} else if stop_reason = "timeout" {
  set outcome to "pass " + pass_no + " timed out waiting for the elevator to settle -- "
      + "trim not written, pilotpitchtrim restored to " + round(trim0, 4) + ".".
} else if stop_reason = "diverged" {
  set outcome to "pass " + pass_no + " deflection grew instead of shrinking -- "
      + "check the trim sign (SIGN CHECK above) -- pilotpitchtrim restored to "
      + round(trim0, 4) + ".".
} else if stop_reason = "no_authority" {
  set outcome to "pass " + pass_no + " deflection held flat instead of shrinking -- "
      + "the trim axis may have no authority here -- pilotpitchtrim restored to "
      + round(trim0, 4) + ".".
} else if stop_reason = "trim_limit" {
  set outcome to "pass " + pass_no + " would push trim past the "
      + trim_ceiling + " ceiling -- pilotpitchtrim restored to "
      + round(trim0, 4) + ".".
} else if stop_reason = "bank" {
  set outcome to "pass " + pass_no + " bank exceeded " + release_bank_tol
      + " deg at release -- pilotpitchtrim restored to " + round(trim0, 4) + ".".
} else if stop_reason = "release_diverged" {
  set outcome to "pass " + pass_no + " flight path angle moved more than "
      + release_gamma_tol + " deg during release -- pilotpitchtrim restored to "
      + round(trim0, 4) + ".".
} else if stop_reason = "afbw" {
  set outcome to "AFBW is enabled and refused to switch off -- refused to run.".
}

local result is "# result  stop_reason " + stop_reason
    + "  passes " + pass_no
    + "  trim " + round(ship:control:pilotpitchtrim, 4)
    + "  deflection_per_pass " + history:join(",").
log result to flightlog.

print "autotrim: " + outcome.
print "  pilotpitchtrim " + round(ship:control:pilotpitchtrim, 4).
print "  deflection per pass " + history:join(",").
print "  a sequence that shrinks while dynamic pressure is rising has the".
print "  same shape as a zero-authority axis would in that same rising-q".
print "  run (see the q and alt columns): required deflection falls with q".
print "  regardless of whether trim did anything. A flat sequence is caught".
print "  separately as no_authority; only a sequence that grows points to a".
print "  sign or scale error -- see the SIGN CHECK above.".
