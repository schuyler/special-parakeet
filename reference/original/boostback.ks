// boostback.ks -- fly a spent booster back toward the launch site on its
// remaining fuel and land it on parachutes.
//
// The stage separates downrange on a suborbital arc with fuel still in the
// tanks. Left alone it comes down where the arc says, tens or hundreds of
// kilometres out, and recovers a fraction of what it cost; the nearer the
// space centre it lands, the more of that cost comes back.
//
// The loop that flies it is targeting.ks, which this script shares with
// ballistic_hop.ks: burn along the horizontal miss until the predicted
// impact reaches the target. What this file owns is everything the loop
// deliberately does not -- the attitude the burn is flown at, which for a
// booster already lofted by its own ascent is flat; the vehicle's reasons
// for stopping; and the descent.
//
// Four phases:
//   1. SETTLE -- separation transient out, RCS on, engines lit if the
//      separation left them dark.
//   2. BOOST  -- flip, then burn until the loop says the miss is gone, the
//      tanks run dry, or the floor is met. Whichever comes first, the
//      descent still flies.
//   3. ENTRY  -- engine off, surface-retrograde hold, parachutes asked
//      for once a second from the atmospheric interface down.
//   4. DOWN   -- the distance the recovery is paid on.
//
// Pinpoint is not the bar and is not attempted: recovery value falls off
// over kilometres, so the burn stops where further burning stops buying
// anything the descent will not undo anyway.
//
// Design and the full argument: notes/booster-return-to-pad.md.

@lazyglobal off.

clearscreen.
print "=== BOOSTBACK ===".

runoncepath("common").     // landing_time, landing_site, burn_duration
runoncepath("aero").       // compass_for, ground_distance
runoncepath("afbw").       // afbw_release(), afbw_restore()
runoncepath("columns").    // columns(), subset()
runoncepath("targeting").  // targeting_new() and the loop

// The target. KSC's launchpad, because that is where a booster launched
// from and stock recovery pays on distance from the space centre. Pass a
// lat/lng to aim at the runway, or anywhere else worth landing near.
parameter target_lat is -0.0972.
parameter target_lng is -74.5577.

// use_tr: read the Trajectories addon's drag-aware impact prediction and
// aim by the difference between it and the drag-free one. False flies the
// drag-free predictor alone, which is the calibration case -- the flight's
// own touchdown miss is then a direct measurement of what drag is worth.
parameter use_tr is true.

// range_bias: metres to aim beyond the target along the approach, for the
// case where nothing models the drag. This is the one number in the file
// that is not free of the craft: it stands in for a ballistic coefficient
// the drag-free predictor cannot know. It should stay 0 whenever
// Trajectories is answering, and otherwise it is set from a flown miss.
parameter range_bias is 0.

// miss_bar: how near the target the predicted impact has to come before
// the burn is finished, metres. Recovery value moves over kilometres and
// the two predictors disagree by more than this, so below the bar the
// burn is spending fuel on precision the rest of the flight discards. An
// accuracy bound, free of the craft and the body.
parameter miss_bar is 250.

// align_bar: how far off the commanded burn direction the nose may be
// with the throttle open, degrees. cos(10 deg) = 0.985, so at most 1.5% of
// the burn goes somewhere other than commanded -- and that part is still
// steering the impact point, not lost. Wider than a node burn's 0.25 deg
// because a spent booster flips on whatever authority is left to it, and
// the loop re-aims every cycle regardless.
parameter align_bar is 10.

// t_taper: seconds of full-throttle closing left when the throttle starts
// coming down, so the cut lands on the bar instead of past it. A few
// control cycles -- long enough that the commanded throttle resolves
// before the miss does, short enough that the taper is not most of the
// burn. Unflown.
parameter t_taper is 2.

// tau_bias: seconds over which the drag correction is averaged. Unflown.
parameter tau_bias is 5.

// dv_reserve: delta-v held back from the boostback, m/s. Zero: the landing
// is on parachutes, so there is no landing burn to hold fuel for, and fuel
// carried home is recovered at its own value rather than the booster's.
parameter dv_reserve is 0.

// p_burn: the ambient pressure, in atmospheres, above which the boostback
// will not burn. Both reasons to stop are about air -- a booster held
// broadside to the airflow is a structural and control problem this script
// has no model for, and the drag-free impact predictor has stopped
// describing the flight -- so the floor is a pressure rather than an
// altitude, and means the same thing on any body with an atmosphere. An
// altitude here would be a Kerbin number wearing a general name.
// ballistic_hop.ks gates its own burn on the same quantity from the other
// side. Roughly a quarter of sea level; unflown, and the alt column against
// the cutoff row is what records where it actually bit.
parameter p_burn is 0.25.

// q_chute: the dynamic pressure, kPa, below which the backstop deploys
// every parachute unconditionally and hands the controls back.
//
// This replaces an altitude, and the reason is that an altitude cannot say
// what this needs to say. What destroys a canopy is dynamic pressure, so
// that is the quantity to test -- and testing it is exactly what
// CHUTESSAFE does, which makes it the right backstop for CHUTESSAFE
// failing to answer. It is also self-timing: q falls below the bar when
// drag has taken the booster to something near terminal velocity, which is
// both the earliest safe moment and, on the way down, shortly before the
// ground. Nothing has to estimate how much fall is left.
//
// Deliberately conservative and unverified. Being late costs altitude the
// working path would have used anyway; being early costs the canopy. The
// falsifier is the q column at the row the chutes come out, against
// whether they survived.
parameter q_chute is 10.

// t_settle: seconds between the script starting and the flip, for the
// separation transient to damp and the stage above to clear the plume.
// Unflown.
parameter t_settle is 3.

local tgt is body:geopositionlatlng(target_lat, target_lng).
local tk is targeting_new(tgt, miss_bar, t_taper, tau_bias, use_tr, range_bias).
local ipu_prior is config:ipu.
// Raised for the impact solve: landing_time bisects a terrain crossing
// thirteen levels deep every cycle, inside a burn where a second of
// arithmetic is a kilometre of along-track. At 2000 the solve costs a
// fraction of a tick.
set config:ipu to 2000.

function stand_down {
  parameter reason.
  print "ABORT: " + reason.
  set config:ipu to ipu_prior.
  wait until false.
}

// === REFUSALS ===
// Two, and both are cases where nothing this script does helps. Everything
// else it copes with: no engine skips the burn, a burn that cannot close
// the miss still flies the descent, because a booster that does not deploy
// its parachutes is not recovered at any distance.
if not body:atm:exists {
  stand_down(body:name + " has no atmosphere -- a parachute landing is "
      + "not available here.").
}
if ship:orbit:periapsis > body:atm:height {
  stand_down("periapsis " + round(ship:orbit:periapsis / 1000, 1)
      + " km is above the atmosphere -- this is an orbit, not a booster "
      + "arc, and it needs a deorbit burn first.").
}
if ship:modulesnamed("ModuleParachute"):length = 0 {
  print "WARNING: no stock parachute module aboard. The return will fly, "
      + "but nothing will slow it at the bottom.".
}

// KSP's abort action group is a toggle, and the loops below read it as the
// pilot's stop. A run ended with it therefore leaves it set, so the next
// run would stop on its first tick. Clear it, and say so.
if abort {
  print "boostback: abort was latched from an earlier run; clearing it.".
  set abort to false.
}

// AFBW writes the control axes every tick and wins the arbitration against
// kOS, throttle included: a script that locks throttle while AFBW holds it
// reads its own commanded value back while the vessel runs something else.
// That would not merely fly the burn wrong, it would corrupt the loop --
// `close` is the measured closing rate divided by the throttle believed to
// have produced it, so a throttle that is a fiction makes the taper a
// fiction (afbw.ks, notes/kos-facts.md).
local afbw_released is afbw_release().

// === FLIGHT RECORDER ===
// One row per second from settle to touchdown, rendered two ways from one
// list of values so the console and the CSV cannot drift apart. Lines
// beginning '#' are what the flight is judged against. The columns serve
// three questions:
//  - miss, close and thr against each other: did the closing rate the
//    taper is sized from hold steady, and did the cut land on the bar;
//  - d_kep against d_tr against the final dist: which predictor described
//    this booster's descent, and by how much drag moved the impact. This
//    is the pair that decides whether the drag correction is worth its
//    complexity, and it is the same comparison entry_flight.csv makes;
//  - serr and q: whether the airframe held the commanded attitude through
//    the burn and the entry, and where in the envelope it did not.
// dist is the number the recovery is paid on: the ship's own ground
// distance from the target, which at the last row is the result.
local col_names is list("t", "phase", "alt", "speed", "vspd", "q", "miss",
                        "d_kep", "d_tr", "bias", "close", "thr", "serr",
                        "mass", "dv", "dist", "lat", "lng").
local col_width is list(9, 7, 7, 8, 7, 9, 9, 9, 9, 7, 8, 6, 6, 7, 7, 9, 9, 9).
// The console is narrower than the full row. These are the columns that
// say whether the loop is converging and the craft is pointing where it
// was told; the CSV keeps every column regardless.
local console_idx is list(0, 1, 2, 3, 6, 10, 11, 12, 15).
local flightlog is "boostback_" + round(time:seconds) + ".csv".

local phase is "SETTLE".
local thr is 0.
local rows is 0.

// One steering lock for the whole flight, switched by phase. Locking once
// at file scope rather than per phase keeps every lock expression in the
// scope it was declared in, and it makes the settle wait productive:
// surface retrograde is within a few degrees of the boostback direction on
// a mostly-horizontal arc, so the flip is half done before BOOST opens the
// throttle.
//
// The boostback burn is flat -- heading(azimuth, 0). A booster arrives at
// separation already lofted by the ascent it just flew, so the arc it
// needs is bought entirely by reversing horizontal velocity, and at these
// speeds the impact point's sensitivity to horizontal delta-v dominates
// its sensitivity to vertical. A vertical component would buy range too,
// by lengthening the fall, but at the price of a higher, faster, hotter
// entry. This is exactly the choice ballistic_hop.ks makes differently.
//
// Retrograde points the *control point's* nose backward along the flight
// path, so a booster controlled from its upper end falls engine-first,
// mass ahead of drag, which is the stable way round. Controlled from the
// other end it flies the unstable way, and no steering command fixes that
// -- serr is the column that says which happened. The roll reference is
// the craft's own top vector, so the hold commands no roll to fight.
function steer_dir {
  if phase = "BOOST" { return heading(compass_for(tk["cmd"]), 0). }
  return lookdirup(-ship:velocity:surface, ship:facing:topvector).
}

// show false writes the CSV row without printing it. The console and the
// CSV still render the same list through the same widths -- the console
// just sees fewer of the rows, which is what lets the burn be sampled
// faster than a human can read.
function log_state {
  parameter show is true.
  // The attitude each phase is judged against: the burn direction while
  // there is a burn, surface retrograde after it.
  local ref is -ship:velocity:surface.
  if phase = "BOOST" { set ref to tk["cmd"]. }
  local row is list(round(time:seconds, 1),
                    phase,
                    round(ship:altitude),
                    round(ship:velocity:surface:mag, 1),
                    round(ship:verticalspeed, 1),
                    round(ship:q * constant:atmtokpa, 4),
                    targeting_miss_col(tk),
                    tk["d_kep"],
                    tk["d_tr"],
                    round(tk["bias"]:mag),
                    round(tk["close"], 1),
                    round(thr, 3),
                    round(vang(ref, ship:facing:vector), 1),
                    round(ship:mass, 3),
                    round(ship:deltav:current, 1),
                    round(ground_distance(tgt)),
                    round(ship:geoposition:lat, 4),
                    round(ship:geoposition:lng, 4)).
  log row:join(",") to flightlog.
  if not show { return. }
  // reprint the header before it scrolls out of reach
  if mod(rows, 20) = 0 {
    print columns(subset(col_names, console_idx), subset(col_width, console_idx)).
  }
  print columns(subset(row, console_idx), subset(col_width, console_idx)).
  set rows to rows + 1.
}

// === SETTLE ===
set warp to 0.
wait until kuniverse:timewarp:issettled.
sas off.                   // kOS warns at run time that SAS fights lock steering
rcs on.                    // the flip runs on whatever authority is left

// A stage that separated with its engines shut down has no thrust until
// they are lit again. Activating an engine that is already burning is a
// no-op, so this costs nothing when separation left them running, and the
// throttle is still zero either way.
if ship:availablethrust <= 0 {
  local en_list is list().
  list engines in en_list.
  for en_ in en_list {
    if not en_:ignition { en_:activate(). }
  }
}

log "# BOOSTBACK  " + ship:name + "  mass " + round(ship:mass, 2) + " t"
    + "  dv " + round(ship:deltav:current, 1) + " m/s"
    + "  thrust " + round(ship:availablethrust, 1) + " kN" to flightlog.
log "# target  " + round(target_lat, 4) + " " + round(target_lng, 4)
    + "  dist " + round(ground_distance(tgt))
    + "  alt " + round(ship:altitude)
    + "  speed " + round(ship:velocity:surface:mag, 1)
    + "  vspd " + round(ship:verticalspeed, 1) to flightlog.
log "# tunables  miss_bar " + miss_bar + "  align_bar " + align_bar
    + "  t_taper " + t_taper + "  tau_bias " + tau_bias
    + "  dv_reserve " + dv_reserve + "  p_burn " + p_burn
    + "  q_chute " + q_chute
    + "  use_tr " + use_tr + "  tr_available " + addons:available("TR")
    + "  range_bias " + range_bias
    + "  afbw_released " + afbw_released to flightlog.
log col_names:join(",") to flightlog.

print "Target is " + round(ground_distance(tgt) / 1000, 1) + " km away, "
    + "at " + round(ship:altitude / 1000, 1) + " km and "
    + round(ship:velocity:surface:mag) + " m/s.".
print "Logging to " + flightlog + ".  abort (backspace) ends the burn.".

lock steering to steer_dir().
lock throttle to thr.

// The settle rows are the baseline: what the untouched arc does. Surveying
// through them costs nothing the wait was not already spending, records
// where the booster would have come down had the engine stayed cold, and
// leaves cmd pointing the right way when BOOST opens the throttle.
local t_prev is time:seconds.
local t_logged is time:seconds - 1.
local t_printed is time:seconds - 1.
local t_settle_end is time:seconds + t_settle.
until time:seconds > t_settle_end or abort {
  if time:seconds - t_logged >= 1 {
    local dt is max(0.02, time:seconds - t_prev).
    set t_prev to time:seconds.
    targeting_survey(tk, dt, 0).
    log_state().
    set t_logged to time:seconds.
  }
  wait 0.
}

// === BOOST ===
// The flip is not a phase of its own: targeting_throttle holds the
// throttle shut while the nose is outside align_bar, so the burn starts
// the moment the turn finishes and stops again if the craft loses it.
local why is "no live engine".
local has_thrust is ship:availablethrust > 0.
// The burn cannot outlast its own propellant. burn_duration prices the
// whole tank at full throttle; twice that covers the taper and anything
// the alignment gate shuts off, and two minutes on top is aerobrake.ks's
// flip allowance -- a craft that cannot make the turn in two minutes will
// not make it.
local t_boost_end is time:seconds.
if has_thrust and not abort {
  set t_boost_end to time:seconds
      + 2 * burn_duration(ship:deltav:current) + 120.
  set why to "burn deadline".
  set phase to "BOOST".
  print "Flipping to " + round(compass_for(tk["cmd"])) + " deg and burning.".
} else if abort {
  set why to "pilot abort".
} else {
  print "No thrust available -- flying the descent only.".
}

local no_impact_s is 0.
if phase = "BOOST" {
  until false {
    local now is time:seconds.
    local dt is max(0.02, now - t_prev).
    set t_prev to now.

    if targeting_survey(tk, dt, thr) {
      set no_impact_s to 0.
      set thr to targeting_throttle(tk, steer_dir():vector, align_bar).
      local verdict is targeting_done(tk).
      if verdict <> "" { set why to verdict. break. }
    } else {
      set thr to 0.
      set no_impact_s to no_impact_s + dt.
      // A burn that lifted periapsis clear of the terrain has no impact
      // point to close on, and going on would be steering a stale number.
      if no_impact_s > 2 { set why to "no terrain crossing to aim at". break. }
    }

    if abort { set why to "pilot abort". break. }
    if ship:availablethrust <= 0 { set why to "engine dry". break. }
    if ship:deltav:current <= dv_reserve { set why to "reserve reached". break. }
    if air_pressure() > p_burn { set why to "into the air below the burn floor". break. }
    if now > t_boost_end { break. }

    // The burn is sampled four times a second and the console still reads
    // once. A second is the right row spacing for a coast and far too
    // coarse for the taper: the throttle comes down over t_taper, two
    // seconds, so at 1 Hz the whole of the manoeuvre this design is most
    // likely to have wrong would arrive as two rows. Two clocks rather
    // than one because a console printing four times a second is not a
    // console (powered_descent.ks logs its arrest burn the same way, for
    // the same reason).
    if now - t_logged >= 0.25 {
      local show is now - t_printed >= 1.
      log_state(show).
      if show { set t_printed to now. }
      set t_logged to now.
    }
    wait 0.
  }
}

set thr to 0.
unlock throttle.
set ship:control:pilotmainthrottle to 0.

local miss_text is "unknown".
if tk["miss"] >= 0 { set miss_text to round(tk["miss"]) + " m". }
log "# cutoff  " + why + "  miss " + miss_text
    + "  d_kep " + tk["d_kep"] + "  d_tr " + tk["d_tr"]
    + "  bias " + round(tk["bias"]:mag)
    + "  close " + round(tk["close"], 1)
    + "  alt " + round(ship:altitude)
    + "  speed " + round(ship:velocity:surface:mag, 1)
    + "  dv_rem " + round(ship:deltav:current, 1) to flightlog.
print "Cutoff: " + why + ", miss " + miss_text
    + ", " + round(ship:deltav:current) + " m/s left.".

// === ENTRY ===
// steer_dir now returns surface retrograde: the attitude that puts the
// most drag in the way -- the shortest, slowest, coolest entry -- and the
// one the canopies want when they open.
set phase to "ENTRY".
rcs off.

// The pilot's abort and the release altitude do the same thing, so they
// share a path: steering released, controls neutralised, SAS back on.
// Arming the parachutes is not part of what is handed back. A pilot who
// takes the controls still wants the canopies out, and asking CHUTESSAFE
// cannot open one early, so the loop goes on asking either way.
local released is false.
local hard_armed is false.
local warping is false.
until ship:status = "LANDED" or ship:status = "SPLASHED" {
  local now is time:seconds.

  // Above the atmosphere there is nothing to hold against and minutes to
  // wait, so ride rails. A craft on rails holds whatever attitude it had
  // when the warp began, which is aerobrake.ks's finding and why the warp
  // ends 5 km above the interface: that is the slew allowance, out of
  // warp, before the air starts to matter. Index 2 rather than higher
  // because KSP gates each rails rate on altitude, and a booster arc peaks
  // not far above the interface -- asking for a rate the game will not
  // grant there just leaves the warp where it was.
  if ship:altitude > body:atm:height + 5000 and not abort {
    if not warping {
      set warpmode to "rails".
      set warp to 2.
      set warping to true.
    }
  } else if warping {
    set warp to 0.
    wait until kuniverse:timewarp:issettled.
    set warping to false.
  }

  // Descending with the air already thick enough to be survivable for a
  // canopy: the backstop's own condition, and the moment nothing more is
  // gained by holding an attitude.
  local q_now is ship:q * constant:atmtokpa.
  local chute_safe is q_now < q_chute and ship:verticalspeed < 0
      and ship:altitude < body:atm:height.

  // The handback: the backstop's moment and the pilot's abort do the same
  // thing, so they share it.
  if (chute_safe or abort) and not released {
    unlock steering.
    set ship:control:neutralize to true.
    sas on.
    set released to true.
    set phase to "CHUTE".
    print "Controls handed back at " + round(ship:altitude) + " m, q "
        + round(q_now, 2) + " kPa"
        + (choose " (pilot abort)" if abort else "") + ".".
  }

  // The hard deploy is a separate flag from the handback, because an abort
  // high up hands the controls back long before the backstop is due and
  // must not consume it. This is the backstop for a parachute module kOS's
  // safe check does not cover: q under the bar is the same test CHUTESSAFE
  // makes, made here instead.
  if chute_safe and not hard_armed {
    chutes on.
    set hard_armed to true.
    log "# hard deploy  alt " + round(ship:altitude)
        + "  q " + round(q_now, 3)
        + "  vspd " + round(ship:verticalspeed, 1) to flightlog.
  }

  if now - t_logged >= 1 {
    // CHUTESSAFE deploys only what can safely deploy right now and is a
    // no-op otherwise, so asking every second from the interface down
    // cannot open a canopy early and cannot miss the first safe moment
    // (notes/kos-facts.md).
    if ship:altitude < body:atm:height { set chutessafe to true. }
    local dt is max(0.02, now - t_prev).
    set t_prev to now.
    targeting_survey(tk, dt, 0, false).
    log_state().
    set t_logged to now.
  }
  wait 0.
}

// === DOWN ===
local final_dist is ground_distance(tgt).
log "# down  dist " + round(final_dist)
    + "  lat " + round(ship:geoposition:lat, 4)
    + "  lng " + round(ship:geoposition:lng, 4)
    + "  status " + ship:status
    + "  speed " + round(ship:velocity:surface:mag, 1)
    + "  cutoff " + why to flightlog.

unlock steering.
unlock throttle.
set ship:control:neutralize to true.
sas on.
afbw_restore(afbw_released).
set config:ipu to ipu_prior.

print "Down " + round(final_dist / 1000, 2) + " km from the target ("
    + ship:status + ").".
print "The witness is " + flightlog + ".".
