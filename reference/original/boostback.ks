// boostback.ks -- fly a spent booster back toward the launch site on its
// remaining fuel and land it on parachutes.
//
// The stage separates downrange on a suborbital arc with fuel still in the
// tanks. Left alone it comes down where the arc says, tens or hundreds of
// kilometres out, and recovers a fraction of what it cost; the nearer the
// space centre it lands, the more of that cost comes back. So the whole
// program is a closed loop on one number -- where the current trajectory
// says the booster meets the ground, and how far that is from the target.
//
// The loop is clean because a coasting ballistic arc's impact point does
// not move. It is an invariant of the orbit, so every metre the miss
// closes was bought by thrust, and the measured closing rate is a direct
// reading of what the throttle is worth. That is what sizes the taper.
//
// Four phases:
//   1. SETTLE -- separation transient out, RCS on, engines lit if the
//      separation left them dark.
//   2. BOOST  -- flip, then burn along the horizontal miss until the
//      predicted impact reaches the target, the tanks run dry, or the
//      floor is met. Whichever comes first, the descent still flies.
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

run "common".   // landing_time, landing_site, burn_duration, steering_aligned_to
run "aero".     // compass_for, ground_distance

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

// tau_bias: seconds over which the drag correction is averaged. The
// correction belongs to the vehicle and the arc, which change over the
// whole burn, while Trajectories recomputes on its own clock and its
// answer steps. Filtering keeps the fast loop on the lag-free predictor
// and takes only the slow correction from the addon. Unflown.
parameter tau_bias is 5.

// dv_reserve: delta-v held back from the boostback, m/s. Zero: the landing
// is on parachutes, so there is no landing burn to hold fuel for, and fuel
// carried home is recovered at its own value rather than the booster's.
parameter dv_reserve is 0.

// alt_floor: the altitude, metres, below which the boostback will not
// burn. Down there a booster held broadside to the airflow is a structural
// and control problem this script has no model for, and the impact
// predictor's drag-free arc has stopped describing the flight. Unflown.
parameter alt_floor is 10000.

// alt_release: the altitude, metres, at which the retrograde hold is
// released, control handed back to SAS, and every parachute deployed
// unconditionally as a backstop against CHUTESSAFE never having fired.
// Below here the booster is at terminal velocity in dense air, which is
// the condition stock chutes are rated to open in, and what is left of the
// fall is tens of seconds -- enough for a canopy. Unflown.
parameter alt_release is 2500.

// t_settle: seconds between the script starting and the flip, for the
// separation transient to damp and the stage above to clear the plume.
// Unflown.
parameter t_settle is 3.

local tgt is body:geopositionlatlng(target_lat, target_lng).
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

// === FLIGHT RECORDER ===
// One CSV row per second, from settle to touchdown. Lines beginning '#'
// are what the flight is judged against. The columns serve three
// questions:
//  - miss, close and thr against each other: did the closing rate the
//    taper is sized from hold steady, and did the cut land on the bar;
//  - d_kep against d_tr against the final dist: which predictor described
//    this booster's descent, and by how much drag moved the impact. This
//    is the pair that decides whether the drag correction is worth its
//    complexity, and it is the same comparison entry_flight.csv makes;
//  - steer_err and q: whether the airframe held the commanded attitude
//    through the burn and the entry, and where in the envelope it did not.
// dist is the number the recovery is paid on: the ship's own ground
// distance from the target, which at the last row is the result.
local flightlog is "boostback_" + round(time:seconds) + ".csv".

// === PREDICTOR STATE ===
// cmd is the commanded burn direction, horizontal, unit. It seeds pointing
// straight at the target so the flip has something to turn to before the
// first survey lands.
local phase is "SETTLE".
local thr is 0.
local cmd is vxcl(up:vector, tgt:position):normalized.
local miss is -1.
local miss_rate is 0.
local closing_full is 0.
local bias is v(0, 0, 0).
local d_kep is "".
local d_tr is "".

// One cycle's reading of both predictors. Sets the state above and returns
// false when the arc has no terrain crossing to aim at.
//
// steer is false past the burn, where the miss is still worth logging but
// there is no longer a burn direction to command from it.
function survey {
  parameter dt.
  parameter steer is true.
  set d_kep to "".
  set d_tr to "".

  // Trajectories is asked through the addon list, not through ADDONS:TR,
  // which raises for an addon kOS has not registered (notes/kos-facts.md).
  local tr_ok is false.
  local tr_pos is v(0, 0, 0).
  if use_tr and addons:available("TR") and addons:tr:hasimpact {
    local tr_geo is addons:tr:impactpos.
    set tr_ok to true.
    set tr_pos to tr_geo:position.
    set d_tr to round(ground_distance(tgt, tr_geo)).
  }

  local t_land is landing_time().
  if t_land < 0 { return false. }
  local kep is landing_site(t_land).
  set d_kep to round(ground_distance(tgt, kep)).

  if tr_ok {
    // The drag correction: the displacement from the modelled impact to
    // the drag-free one -- how far past the real landing an arc with no
    // air in it carries.
    set bias to bias + ((kep:position - tr_pos) - bias) * min(1, dt / tau_bias).
  }

  // Aim at the target pushed out by that correction, so that driving the
  // drag-free impact onto the aim point puts the real one on the target.
  local aim is tgt:position + bias.
  if range_bias <> 0 {
    set aim to aim + vxcl(up:vector, tgt:position):normalized * range_bias.
  }

  // The miss, taken in the ship's own horizontal plane: its direction is
  // the burn direction wanted, and over the distances in play its length
  // and the great-circle distance agree to parts in ten thousand. d_kep
  // alongside is the honest great-circle number for the record.
  local m_vec is vxcl(up:vector, aim - kep:position).
  local m_new is m_vec:mag.
  if steer and m_new > 0 { set cmd to m_vec:normalized. }

  if miss >= 0 {
    // Filtered over a second: several control cycles, short against the
    // burn and long against the step the bisection's own tolerance puts
    // in the crossing time.
    set miss_rate to miss_rate + ((m_new - miss) / dt - miss_rate) * min(1, dt).
  }
  set miss to m_new.

  // What the miss closes at per unit throttle. Dividing by the throttle
  // that produced it is what stops the taper chasing its own tail: a rate
  // measured at half throttle predicts twice that at full, so the throttle
  // commanded below is a fixed point rather than a decaying one.
  if thr > 0.1 and miss_rate < 0 {
    set closing_full to -miss_rate / thr.
  }
  return true.
}

// Full throttle until the miss is within t_taper seconds of closing at
// full throttle, then linearly down, so the cut lands on the bar. Shut
// while the nose is outside align_bar, which makes the alignment wait a
// property of the loop instead of a phase of its own: the burn simply does
// not run until the flip is done, and stops again if the craft loses it.
function throttle_for {
  if vang(cmd, ship:facing:vector) > align_bar { return 0. }
  if closing_full <= 0 { return 1. }
  return max(0, min(1, miss / (closing_full * t_taper))).
}

// One steering lock for the whole flight, switched by phase. Locking once
// at file scope rather than per phase keeps every lock expression in the
// scope it was declared in, and it means the settle wait is productive:
// surface retrograde is within a few degrees of the boostback direction on
// a mostly-horizontal arc, so the flip is half done before BOOST opens the
// throttle. The roll reference is the craft's own top vector, so the hold
// commands no roll for the reaction wheels to fight.
function steer_dir {
  if phase = "BOOST" { return heading(compass_for(cmd), 0). }
  return lookdirup(-ship:velocity:surface, ship:facing:topvector).
}

function log_state {
  // The attitude each phase is judged against: the burn direction while
  // there is a burn, surface retrograde after it.
  local ref is -ship:velocity:surface.
  if phase = "BOOST" { set ref to cmd. }
  // Empty until the first survey lands, the same way d_kep and d_tr go
  // empty when their predictor has no answer.
  local miss_col is "".
  if miss >= 0 { set miss_col to round(miss). }
  log round(time:seconds, 1) + "," + phase + ","
      + round(ship:altitude) + ","
      + round(ship:velocity:surface:mag, 1) + ","
      + round(ship:verticalspeed, 1) + ","
      + round(ship:q * constant:atmtokpa, 4) + ","
      + miss_col + ","
      + d_kep + "," + d_tr + ","
      + round(bias:mag) + ","
      + round(closing_full, 1) + ","
      + round(thr, 3) + ","
      + round(vang(ref, ship:facing:vector), 1) + ","
      + round(ship:mass, 3) + ","
      + round(ship:deltav:current, 1) + ","
      + round(ground_distance(tgt)) + ","
      + round(ship:geoposition:lat, 4) + ","
      + round(ship:geoposition:lng, 4)
      to flightlog.
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
    + "  dv_reserve " + dv_reserve + "  alt_floor " + alt_floor
    + "  alt_release " + alt_release
    + "  use_tr " + use_tr + "  tr_available " + addons:available("TR")
    + "  range_bias " + range_bias to flightlog.
log "t,phase,alt,speed,vspd,q,miss,d_kep,d_tr,bias,close,thr,steer_err,mass,dv_rem,dist,lat,lng"
    to flightlog.

print "Target is " + round(ground_distance(tgt) / 1000, 1) + " km away, "
    + "at " + round(ship:altitude / 1000, 1) + " km and "
    + round(ship:velocity:surface:mag) + " m/s.".
print "Logging to " + flightlog + ".".
print "Settling for " + t_settle + " s.".

lock steering to steer_dir().
lock throttle to thr.

// The settle rows are the baseline: what the untouched arc does. Surveying
// through them costs nothing the wait was not already spending, records
// where the booster would have come down had the engine stayed cold, and
// leaves cmd pointing the right way when BOOST opens the throttle.
local t_prev is time:seconds.
local t_logged is time:seconds - 1.
local t_settle_end is time:seconds + t_settle.
until time:seconds > t_settle_end {
  if time:seconds - t_logged >= 1 {
    local dt is max(0.02, time:seconds - t_prev).
    set t_prev to time:seconds.
    survey(dt).
    log_state().
    set t_logged to time:seconds.
  }
  wait 0.
}

// === BOOST ===
// The flip is not a phase of its own: throttle_for holds the throttle shut
// while the nose is outside align_bar, so the burn starts the moment the
// turn finishes and stops again if the craft loses the attitude.
local why is "no live engine".
local has_thrust is ship:availablethrust > 0.
// The burn cannot outlast its own propellant. burn_duration prices the
// whole tank at full throttle; twice that covers the taper and anything
// the alignment gate shuts off, and two minutes on top is aerobrake.ks's
// flip allowance -- a craft that cannot make the turn in two minutes will
// not make it.
local t_boost_end is time:seconds.
if has_thrust {
  set t_boost_end to time:seconds
      + 2 * burn_duration(ship:deltav:current) + 120.
  set why to "burn deadline".
  set phase to "BOOST".
  print "Flipping to " + round(compass_for(cmd)) + " deg and burning.".
} else {
  print "No thrust available -- flying the descent only.".
}

local miss_min is 2 ^ 30.
local no_impact_s is 0.
if has_thrust {
  until false {
    local now is time:seconds.
    local dt is max(0.02, now - t_prev).
    set t_prev to now.

    if survey(dt) {
      set no_impact_s to 0.
      set thr to throttle_for().
      if miss < miss_min { set miss_min to miss. }
      if miss <= miss_bar { set why to "miss inside the bar". break. }
      // Past its own best by more than the bar: the burn is now making the
      // landing worse, whatever the predictor thinks it is doing.
      if miss > miss_min + miss_bar { set why to "miss growing past its best". break. }
    } else {
      set thr to 0.
      set no_impact_s to no_impact_s + dt.
      if no_impact_s > 2 { set why to "no terrain crossing to aim at". break. }
    }

    if ship:availablethrust <= 0 { set why to "engine dry". break. }
    if ship:deltav:current <= dv_reserve { set why to "reserve reached". break. }
    if ship:altitude < alt_floor { set why to "below the burn floor". break. }
    if now > t_boost_end { break. }

    if now - t_logged >= 1 {
      log_state().
      print "BOOST miss=" + round(miss) + " thr=" + round(thr, 2)
          + " dv=" + round(ship:deltav:current) + " err="
          + round(vang(cmd, ship:facing:vector), 1) + "      " at (0, 8).
      set t_logged to now.
    }
    wait 0.
  }
}

set thr to 0.
unlock throttle.
set ship:control:pilotmainthrottle to 0.

local miss_text is "unknown".
if miss >= 0 { set miss_text to round(miss) + " m". }
log "# cutoff  " + why + "  miss " + miss_text
    + "  d_kep " + d_kep + "  d_tr " + d_tr
    + "  bias " + round(bias:mag)
    + "  close " + round(closing_full, 1)
    + "  alt " + round(ship:altitude)
    + "  speed " + round(ship:velocity:surface:mag, 1)
    + "  dv_rem " + round(ship:deltav:current, 1) to flightlog.
print "Cutoff: " + why + ", miss " + miss_text
    + ", " + round(ship:deltav:current) + " m/s left.".

// === ENTRY ===
// steer_dir now returns surface retrograde: the attitude that puts the
// most drag in the way -- the shortest, slowest, coolest entry -- and the
// one the canopies want when they open. Retrograde points the *control
// point's* nose backward along the flight path, so a booster controlled
// from its upper end falls engine-first, mass ahead of drag, which is the
// stable way round. Controlled from the other end it flies the unstable
// way, and no steering command fixes that -- steer_err is the column that
// says which happened.
set phase to "ENTRY".
rcs off.

local released is false.
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
  if ship:altitude > body:atm:height + 5000 {
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

  if ship:altitude < alt_release and not released {
    // The backstop, for a parachute module kOS's safe check does not
    // cover: deploy unconditionally, and hand the airframe back to SAS so
    // nothing is steering against the canopies.
    chutes on.
    unlock steering.
    set ship:control:neutralize to true.
    sas on.
    set released to true.
    set phase to "CHUTE".
    print "Chutes hard-armed, controls handed back at "
        + round(ship:altitude) + " m.".
  }

  if now - t_logged >= 1 {
    // CHUTESSAFE deploys only what can safely deploy right now and is a
    // no-op otherwise, so asking every second from the interface down
    // cannot open a canopy early and cannot miss the first safe moment
    // (notes/kos-facts.md).
    if ship:altitude < body:atm:height { set chutessafe to true. }
    local dt is max(0.02, now - t_prev).
    set t_prev to now.
    survey(dt, false).
    log_state().
    print phase + " alt=" + round(ship:altitude) + " v="
        + round(ship:velocity:surface:mag) + " dist="
        + round(ground_distance(tgt)) + "        " at (0, 9).
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
set config:ipu to ipu_prior.

print "Down " + round(final_dist / 1000, 2) + " km from the target ("
    + ship:status + ").".
print "The witness is " + flightlog + ".".
