// afbw.ks -- take the control axes away from Advanced Fly-By-Wire for the
// duration of a script, and give them back afterwards.
//
// AFBW writes the vessel's control axes every tick and wins the arbitration
// against kOS. The failure is silent in both directions: a script that locks
// throttle while AFBW holds the throttle axis reads its own commanded value
// back out of THROTTLE while the vessel runs whatever AFBW is sending, and a
// loop writing ship:control:pitch is answered by an airframe that is also
// being flown by the stick. kOS does not re-assert a lock when AFBW later
// lets go either, so toggling AFBW mid-flight leaves the lock stale until the
// script is restarted.
//
// Disabling AFBW does not by itself release the throttle. The working
// explanation is a throttle offset AFBW latches and keeps applying additively
// to whatever kOS commands -- mainThrottle = clamp(commanded + latch, 0, 1) --
// which ENABLED going false does not clear. One flight witnesses it: with the
// settings line recording afbw_released True, so AFBW was confirmed switched
// off, a commanded 0.5 ran the engines at 0.989. The clear happens inside the
// bridge's ENABLED setter, and only on a bridge DLL carrying that fix.
//
// The toggle is reached through the kOS-AFBW bridge addon, which exposes
// AFBW's global enable as ADDONS:AFBW:ENABLED. Reading ADDONS:AFBW at all
// requires that bridge to be installed.

@lazyglobal off.

// Switch AFBW off. Returns whether this call is the one that transitioned it
// from on to off -- only that caller should switch it back on, so a script
// does not re-enable a stick its pilot had deliberately switched off.
//
// The write happens even if ENABLED already reads false: the latch clear is
// triggered by the write itself, and ENABLED does not say whether a latch
// from earlier in the session is still live. The write is idempotent on
// AFBW's own flag.
//
// Availability is asked of the addon list, ADDONS:AVAILABLE("AFBW"), rather
// than of the addon, ADDONS:AFBW:AVAILABLE. Both suffixes exist -- kOS's base
// Addon class registers AVAILABLE on every addon. But ADDONS:AFBW is itself a
// suffix that exists only for addons kOS has registered, and the bridge ships
// its own addon class, so with the bridge absent ADDONS:AFBW raises before
// AVAILABLE can answer. The list method returns false instead. Either way it
// reports whether AFBW's assembly loaded, in any scene, so it is not a flight
// guard on its own.
//
// The setter writes a static field and so takes effect in any scene, but it
// returns early -- silently -- if the bridge could not reflect AFBW's internals
// at all. Reading the state back is what makes this safe: it distinguishes
// "switched off" from "asked, and nothing happened".
function afbw_release {
  if not addons:available("afbw") { return false. }
  local was_enabled is addons:afbw:enabled.
  set addons:afbw:enabled to false.
  if addons:afbw:enabled {
    print "afbw: could not switch AFBW off -- it will override the controls.".
    return false.
  }
  return was_enabled.
}

// Whether the bridge's reflection into AFBW's FlightManager -- the mechanism
// the ENABLED setter uses to clear the latched throttle offset -- is bound.
// This is a capability check, not a confirmation: it says the reflection
// handles are live, not that any particular afbw_release() call cleared the
// latch on this flight. HASSUFFIX guards a bridge DLL built before this
// suffix existed, where reading THROTTLE_RELEASE_BOUND directly would raise
// mid-flight; it reads false there, and false when AFBW is not installed.
//
// Ask after afbw_release(), not before: a bridge that binds its reflection
// handles inside the ENABLED setter has nothing to report until that setter
// has run.
function afbw_throttle_release_bound {
  if not addons:available("afbw") { return false. }
  if not addons:afbw:hassuffix("throttle_release_bound") { return false. }
  return addons:afbw:throttle_release_bound.
}

// Give the stick back, but only if afbw_release() was what took it away.
//
// A script that dies before reaching this leaves AFBW off and the stick dead
// until it is re-enabled from AFBW's own toolbar button.
function afbw_restore {
  parameter released.
  if released {
    set addons:afbw:enabled to true.
    if not addons:afbw:enabled {
      print "afbw: could not switch AFBW back on -- use its toolbar button.".
    }
  }
}
