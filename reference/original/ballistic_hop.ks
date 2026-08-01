// ballistic_hop.ks -- put a vehicle on a ballistic arc that lands at a
// chosen place on the surface. KSC to an island, a polar anomaly, a
// contract site: anywhere the ground goes.
//
// This is boostback.ks's loop run in the other direction. There, thrust
// took a downrange impact point and dragged it back to the pad; here it
// takes an impact point a few tens of kilometres ahead and pushes it a
// thousand. Same invariant, same measured closing rate, same taper, same
// file -- targeting.ks. What differs is one thing, and it is the whole
// content of this script:
//
//   **A booster arrives already lofted. A vehicle leaving the ground has
//   to buy its own arc.**
//
// So boostback burns flat and this one burns at the minimum-energy loft
// angle for the range it has left, 45 - phi/4, correcting the flight-path
// angle rather than commanding the attitude open-loop. The rest of the
// file is the boundary: where this script takes over, and where it stops.
//
// It owns the boost and the coast, and nothing else:
//   WAIT  -- no controls taken. Fly the climb yourself or with
//            autopilot.ks; this engages when the air gets thin enough.
//   BOOST -- the loop, at the loft angle, until the miss is gone or the
//            tanks are.
//   COAST -- surface-retrograde hold down to the handoff altitude.
//   Then it lets go and says so. The entry and the landing belong to
//   reentry.ks and to autopilot.ks with the target as its last waypoint;
//   a script that tried to own those too would be reimplementing both.
//
// Design and the full argument: notes/ballistic-targeting.md.

@lazyglobal off.

clearscreen.
print "=== BALLISTIC HOP ===".

runoncepath("common").     // landing_time, landing_site, burn_duration
runoncepath("aero").       // compass_for, ground_distance, angle_of_ascent,
                           // air_pressure
runoncepath("afbw").       // afbw_release(), afbw_restore()
runoncepath("columns").    // columns(), subset()
runoncepath("targeting").  // the loop, range_angle, ballistic_loft/_speed

// Where to land. The default is the KSC island airfield, far enough east
// to be a real hop and near enough to fly on a first attempt.
parameter target_lat is -1.5175.
parameter target_lng is -71.9656.

parameter use_tr is true.
parameter range_bias is 0.

// miss_bar: how near the target the predicted impact must come before the
// boost is finished, metres. Larger than boostback's 250 for a reason
// worth stating: this arc has an entry between it and the ground, flown by
// a script that is not this one, and any range control that entry exercises
// is dispersion this bar cannot see. Aiming tighter than the handoff is
// precise is precision spent on nothing. Unflown.
parameter miss_bar is 2000.

// align_bar: how far off the commanded attitude the nose may be with the
// throttle open, degrees. Same argument as boostback's -- cos(10 deg) is
// 0.985 -- but a winged vehicle at boost has aerodynamic authority a spent
// booster does not, so if this one cannot hold 10 degrees the reason is
// worth finding rather than widening.
parameter align_bar is 10.

parameter t_taper is 2.
parameter tau_bias is 5.

// p_boost: the ambient pressure, in atmospheres, below which the boost may
// start. The loft correction points the nose off the velocity vector, so
// the burn buys an angle of attack as well as an arc; below a twentieth of
// sea level the drag and the airframe load that costs are small fractions
// of what the vehicle already survived climbing to here. Stated as a
// pressure rather than an altitude so it means the same thing on any body
// with air. Unflown -- q and serr in the log are what would falsify it.
parameter p_boost is 0.05.

// pitch_max, pitch_min: the band the commanded pitch is held inside,
// degrees above the horizon. The loft rule alone cannot run away, but the
// flight-path correction on top of it can, if gamma is far from the loft
// when the burn starts. The band is what stops a correction becoming a
// vertical climb or a dive. Unflown.
parameter pitch_max is 80.
parameter pitch_min is -20.

// alt_handoff: the altitude, metres, at which this script lets go. Above
// the air that matters, so whatever flies the entry gets the vehicle with
// its options open rather than already committed. Unflown.
parameter alt_handoff is 25000.

// dv_reserve: delta-v held back from the boost, m/s. Unlike a booster, a
// vehicle that has to get itself home may want some -- but the default is
// still 0, because the arc is what this script is for and the reserve is
// the caller's judgement, not this file's.
parameter dv_reserve is 0.

local tgt is body:geopositionlatlng(target_lat, target_lng).
local tk is targeting_new(tgt, miss_bar, t_taper, tau_bias, use_tr, range_bias).
local ipu_prior is config:ipu.
set config:ipu to 2000.

function stand_down {
  parameter reason.
  print "ABORT: " + reason.
  set config:ipu to ipu_prior.
  wait until false.
}

// === REFUSALS ===
if not body:atm:exists {
  stand_down(body:name + " has no atmosphere, so there is no climb to wait "
      + "on and no entry to hand off to. This is a launch, not a hop.").
}
if abort {
  print "hop: abort was latched from an earlier run; clearing it.".
  set abort to false.
}

local phi0 is range_angle(tgt).
if phi0 < 1 {
  stand_down("the target is " + round(ground_distance(tgt) / 1000, 1)
      + " km away, under a degree of arc. That is a landing problem, not a "
      + "ballistic one -- fly it with autopilot.ks.").
}

// === PHASE STATE ===
local phase is "WAIT".
local thr is 0.
local rows is 0.

// The commanded pitch: the minimum-energy loft for the range still to go,
// plus the same again of correction for wherever the velocity vector
// actually is.
//
// The rule prescribes the *velocity* angle at burnout; commanding the
// *thrust* at it would only reach it asymptotically, and a short burn is
// not asymptotic. So the thrust leads the wanted velocity direction by
// exactly as much as the current velocity lags it -- gamma low by five
// degrees points the nose five above the loft. Nothing is chosen in that:
// leading by the error is the symmetric statement, it converges for any
// positive gain, and one is the gain that costs the least cosine getting
// there. The band is the only thing here that is a number.
//
// phi is re-read every cycle, so as the ground closes the loft steepens
// toward 45 degrees on its own.
function boost_pitch {
  local loft is ballistic_loft(range_angle(tgt)).
  return max(pitch_min, min(pitch_max, loft + (loft - angle_of_ascent()))).
}

// Azimuth from the targeting loop, pitch from the loft rule. The azimuth
// carries the cross-range correction for free: the miss vector points
// wherever the impact is wrong, including sideways.
function steer_dir {
  if phase = "BOOST" {
    return heading(compass_for(tk["cmd"]), boost_pitch()).
  }
  return lookdirup(-ship:velocity:surface, ship:facing:topvector).
}

// === FLIGHT RECORDER ===
// gamma and loft against pitch answer the question this script adds to
// boostback's: is the flight-path angle going where the rule asked, and is
// the correction that drives it there bounded? gamma converging on loft
// while pitch sits between them is the signature. pitch pinned at a band
// edge is the correction running away.
local col_names is list("t", "phase", "alt", "speed", "vspd", "gamma",
                        "loft", "pitch", "q", "miss", "d_kep", "d_tr",
                        "close", "thr", "serr", "dv", "dist").
local col_width is list(9, 7, 7, 8, 7, 7, 7, 7, 9, 9, 9, 9, 8, 6, 6, 7, 10).
local console_idx is list(0, 1, 2, 3, 5, 6, 7, 9, 13, 16).
local flightlog is "hop_" + round(time:seconds) + ".csv".

function log_state {
  local ref is -ship:velocity:surface.
  if phase = "BOOST" { set ref to steer_dir():vector. }
  local loft is ballistic_loft(range_angle(tgt)).
  local row is list(round(time:seconds, 1),
                    phase,
                    round(ship:altitude),
                    round(ship:velocity:surface:mag, 1),
                    round(ship:verticalspeed, 1),
                    round(angle_of_ascent(), 1),
                    round(loft, 1),
                    round(boost_pitch(), 1),
                    round(ship:q * constant:atmtokpa, 4),
                    targeting_miss_col(tk),
                    tk["d_kep"],
                    tk["d_tr"],
                    round(tk["close"], 1),
                    round(thr, 3),
                    round(vang(ref, ship:facing:vector), 1),
                    round(ship:deltav:current, 1),
                    round(ground_distance(tgt))).
  log row:join(",") to flightlog.
  if mod(rows, 20) = 0 {
    print columns(subset(col_names, console_idx), subset(col_width, console_idx)).
  }
  print columns(subset(row, console_idx), subset(col_width, console_idx)).
  set rows to rows + 1.
}

// === THE PLAN ===
// Stated before anything is committed, and refused on if the vehicle
// plainly cannot do it. The speed test is a lower bound compared against
// an upper bound -- the least speed any ballistic arc to that range needs,
// against the most this vehicle could reach if every remaining metre per
// second went straight into its current velocity. Failing that comparison
// is a certainty, not an estimate; passing it proves nothing, because
// gravity losses, drag and the rotating ground are all on the other side.
local v_need is ballistic_speed(phi0).
local v_best is ship:velocity:surface:mag + ship:deltav:current.
print "Target " + round(ground_distance(tgt) / 1000, 1) + " km out, "
    + round(phi0, 1) + " deg of arc.".
print "Minimum-energy arc: " + round(v_need) + " m/s at "
    + round(ballistic_loft(phi0), 1) + " deg of loft.".
print "Vehicle could reach at most " + round(v_best) + " m/s.".
if v_best < v_need {
  stand_down("the arc needs " + round(v_need) + " m/s and the vehicle "
      + "cannot exceed " + round(v_best) + " m/s even spending every drop "
      + "along its current velocity. Closer target, or more stage.").
}

log "# BALLISTIC HOP  " + ship:name + "  mass " + round(ship:mass, 2) + " t"
    + "  dv " + round(ship:deltav:current, 1) + " m/s" to flightlog.
log "# target  " + round(target_lat, 4) + " " + round(target_lng, 4)
    + "  dist " + round(ground_distance(tgt))
    + "  phi " + round(phi0, 2) + " deg"
    + "  v_need " + round(v_need, 1)
    + "  v_best " + round(v_best, 1)
    + "  loft " + round(ballistic_loft(phi0), 2) to flightlog.
log "# tunables  miss_bar " + miss_bar + "  align_bar " + align_bar
    + "  t_taper " + t_taper + "  tau_bias " + tau_bias
    + "  p_boost " + p_boost + "  pitch band " + pitch_min + " " + pitch_max
    + "  alt_handoff " + alt_handoff + "  dv_reserve " + dv_reserve
    + "  use_tr " + use_tr + "  tr_available " + addons:available("TR")
    + "  range_bias " + range_bias to flightlog.
log col_names:join(",") to flightlog.
print "Logging to " + flightlog + ".  abort (backspace) ends the burn.".

// === WAIT ===
// No controls are taken here. The vehicle is climbing under someone else's
// hand -- a pilot's, or autopilot.ks's -- and this loop only watches the
// air thin out and records what the arc would do if nothing were done.
// Locking steering later rather than now is also what keeps every lock
// expression at file scope.
print "Waiting for ambient pressure below " + p_boost + " atm.".
local t_prev is time:seconds.
local t_logged is time:seconds - 1.
until air_pressure() <= p_boost or abort {
  if time:seconds - t_logged >= 1 {
    local dt is max(0.02, time:seconds - t_prev).
    set t_prev to time:seconds.
    targeting_survey(tk, dt, 0).
    log_state().
    set t_logged to time:seconds.
  }
  wait 0.
}

// An abort during WAIT is the cheapest exit there is: the controls were
// never taken, so there is nothing to hand back and no attitude to unwind.
if abort {
  print "hop: aborted during the climb -- the controls were never taken.".
  log "# aborted during WAIT  alt " + round(ship:altitude) to flightlog.
  set config:ipu to ipu_prior.
  print "The witness is " + flightlog + ".".
  exit.
}

// === BOOST ===
// AFBW is released here rather than at the top: the climb this script just
// watched may well have been flown on the stick, and taking it away during
// someone else's phase is not this file's business. From here the axes are
// ours, and a latched throttle would make `close` -- and so the whole
// taper -- a fiction (afbw.ks, notes/kos-facts.md).
local afbw_released is afbw_release().
local sas_was_on is sas.
if sas_was_on { set sas to false. }

set phase to "BOOST".
lock steering to steer_dir().
lock throttle to thr.
print "BOOST: " + round(compass_for(tk["cmd"])) + " deg at "
    + round(boost_pitch(), 1) + " deg of pitch.".

local why is "burn deadline".
local t_boost_end is time:seconds
    + 2 * burn_duration(ship:deltav:current) + 120.
local no_impact_s is 0.
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
    // Unlike a booster's, this burn is *trying* to push periapsis toward
    // the ground on the far side rather than clear of it. Losing the
    // terrain crossing altogether means the arc has gone orbital, which is
    // a miss of a different kind and worth its own name.
    set thr to 0.
    set no_impact_s to no_impact_s + dt.
    if no_impact_s > 2 { set why to "arc went orbital -- no impact". break. }
  }

  if abort { set why to "pilot abort". break. }
  if ship:availablethrust <= 0 { set why to "engine dry". break. }
  if ship:deltav:current <= dv_reserve { set why to "reserve reached". break. }
  if now > t_boost_end { break. }

  if now - t_logged >= 1 {
    log_state().
    set t_logged to now.
  }
  wait 0.
}

set thr to 0.
unlock throttle.
set ship:control:pilotmainthrottle to 0.

local miss_text is "unknown".
if tk["miss"] >= 0 { set miss_text to round(tk["miss"]) + " m". }
log "# cutoff  " + why + "  miss " + miss_text
    + "  d_kep " + tk["d_kep"] + "  d_tr " + tk["d_tr"]
    + "  close " + round(tk["close"], 1)
    + "  alt " + round(ship:altitude)
    + "  speed " + round(ship:velocity:surface:mag, 1)
    + "  gamma " + round(angle_of_ascent(), 2)
    + "  loft " + round(ballistic_loft(range_angle(tgt)), 2)
    + "  apoapsis " + round(ship:orbit:apoapsis)
    + "  dv_rem " + round(ship:deltav:current, 1) to flightlog.
print "Cutoff: " + why + ", miss " + miss_text + ".".

// === COAST ===
// Retrograde, down to the handoff. Holding an attitude through the coast
// costs nothing and means the vehicle arrives at the top of the air
// pointing somewhere deliberate rather than wherever the burn left it.
set phase to "COAST".
lock steering to lookdirup(-ship:velocity:surface, ship:facing:topvector).
until ship:altitude < alt_handoff or ship:status = "LANDED"
      or ship:status = "SPLASHED" or abort {
  if time:seconds - t_logged >= 1 {
    local dt is max(0.02, time:seconds - t_prev).
    set t_prev to time:seconds.
    targeting_survey(tk, dt, 0, false).
    log_state().
    set t_logged to time:seconds.
  }
  wait 0.
}

// === HANDOFF ===
log "# handoff  alt " + round(ship:altitude)
    + "  speed " + round(ship:velocity:surface:mag, 1)
    + "  dist " + round(ground_distance(tgt))
    + "  miss " + targeting_miss_col(tk)
    + "  d_kep " + tk["d_kep"] + "  d_tr " + tk["d_tr"]
    + "  lat " + round(ship:geoposition:lat, 4)
    + "  lng " + round(ship:geoposition:lng, 4) to flightlog.

unlock steering.
unlock throttle.
set ship:control:neutralize to true.
sas on.
afbw_restore(afbw_released).
set config:ipu to ipu_prior.

print "Handoff at " + round(ship:altitude) + " m, "
    + round(ground_distance(tgt) / 1000, 1) + " km to run.".
print "The arc is aimed; the entry is not this script's. Run reentry.ks, "
    + "then autopilot.ks with the target as its last waypoint.".
print "The witness is " + flightlog + ".".
