// powered_descent.ks — the powered descent, reduced to its invariants.
//
// This file assumes the plan is good: it carries no envelope protection and
// no coping for a plan that missed.
// Design and the full argument: notes/powered-descent-invariants.md.
//
// Assumes (plan_doi.ks's contract): the DOI burn is behind us, PDI is the
// periapsis of the ellipse we are on, the corridor under the arc is
// certified, and the orbital plane passes near the site. landing_height
// appears nowhere below: the planner already spent it into the ellipse, so
// the arc reaches the surface near the site without this program ever knowing
// the number.
//
// Five ideas, one per section:
//   1. Hold thrust retrograde and the trajectory is a one-parameter
//      family: current state plus throttle determines the whole arc.
//   2. Euler's method draws the arc: rates times a small dt, summed.
//   3. The predicted touchdown point falls back as throttle rises, so
//      bisection finds the one throttle whose arc — integrated through the
//      coast and the arrest burn — comes to rest at the site. Re-solving
//      every few seconds from live state replaces plan, table, and trim.
//   4. A small lateral bias on the retrograde hold closes the plane onto
//      the site while the ship is fast, where a degree costs least.
//   5. Terminal is the same retrograde hold flown to the ground: a coast
//      from high gate until f_max can just arrest the speed at the pad,
//      then the arrest burn. Thrust opposite the velocity takes drift and
//      descent rate together, and gravity uprights the velocity vector as
//      the horizontal dies, so touchdown is plumb without a lateral law.

@lazyglobal off.

clearscreen.
print "=== POWERED DESCENT ===".

run "common".              // engine_isp
run "../core/kepler".      // orbital_speed, ground_track_distance;
                           // bisect rides along

parameter target_lat is 0.
parameter target_lng is 0.
parameter f_max is 0.85.

local tgt is body:geopositionlatlng(target_lat, target_lng).
local mdot_full is ship:availablethrust / (engine_isp() * constant:g0).
local ipu_prior is config:ipu.
set config:ipu to 2000.

// Integration tolerances for endpoint's Euler steps — accuracy bounds, not
// craft or body numbers. pitch_tol caps the flight-path rotation per step
// (degrees); v_frac caps the fractional speed change per step.
local pitch_tol is 1.
local v_frac is 0.02.

// Descent geometry, shared by the planning integration and the flight.
// g0 is surface gravity; tilt_max the flight-path angle off vertical at
// which braking hands to terminal — past it the retrograde hold is a
// near-hover, terminal's deliberate job; h_pad the flare height; v_floor
// the touchdown descent rate.
local g0 is body:mu / body:radius ^ 2.
local tilt_max is 30.
local h_pad is 5.
local v_floor is 2.

// y_floor: the cross-range residual braking is asked to leave at high
// gate, metres. The floor is the downstream error budget, not a craft
// number: terminal contributes ~17 m of measured wander of its own, so
// driving braking's residual below ~20 m spends yaw — and yaw's
// unmodelled reach cost — on error the next phase re-creates. The <= 10 m
// campaign lowers this floor as terminal's wander is beaten down; k_yaw
// then rises through its own formula.
local y_floor is 20.

// f_bracket: half-width of the throttle bracket handed to each in-loop
// re-solve, seeded from the previous root. Sized from the root's per-look
// drift, not chosen: (per-look reach error ~100 m) / (dReach/df ≈ 47 km
// per unit f) ≈ 0.002 mid-burn, and 0.05 covers roughly 25 times that,
// because metres-per-throttle collapses approaching the tilt_max + 10
// cutoff, where a bracket this wide still has to widen to the hard limits
// — the nominal case there, not the exception. A too-narrow bracket costs
// one widening march per look (real time, displacing flown flight); it
// never costs accuracy, since the widening always reaches the true root.
local f_bracket is 0.05.

// Where the arc from state st at throttle f touches down: the whole descent
// under one set of Euler equations — braking, coast, and arrest burn — with
// only the thrust level switching, at the boundaries the flight itself
// uses. Braking thrusts retrograde at f until the flight path passes
// tilt_max short of vertical (high gate); the coast thrusts nothing; the
// arrest burn thrusts retrograde at f_max from the moment f_max could just
// bring the speed to rest a flare height above the site — the same
// schedule the flight ignites on, in flight referenced to the craft's
// lowest point, a metres-scale offset this model ignores. Thrust takes
// speed; gravity's along-path part adds speed back as the nose drops; its
// across-path part turns the path down at g*cos(pitch)/speed while the
// horizon rotates away under the ship at speed*cos(pitch)/r — the two
// rates whose difference is the turn.
//
// dt is the smaller of the time to rotate the flight path by pitch_tol and
// the time to change speed by v_frac of itself, so the step refines where the
// path bends over; the step cap is a non-convergence guard. The integration
// ends at the ground (h <= tgt:terrainheight — the site's surface, not the
// datum) or at walking speed, where the model's retrograde direction loses
// meaning and the remaining fall is vertical. x is the ground distance to
// that point, t the time to it, and t_brake the time spent braking — the
// horizon the yaw law's time constant is cut from.
function endpoint {
  parameter f.
  parameter st.   // start state: "speed" m/s, "pitch" deg above horizon,
                  // "h" m above the datum, "m" tonnes
  local speed is st["speed"].
  local pitch is st["pitch"].
  local h is st["h"].
  local m is st["m"].
  local theta is 0.                // ground angle swept, radians
  local t is 0.
  local t_brake is 0.
  local steps is 0.
  local phase_ is "BRAKE".
  until h <= tgt:terrainheight or speed < v_floor or steps >= 4000 {
    local r_ is body:radius + h.
    local g is body:mu / r_ ^ 2.
    if phase_ = "BRAKE" and pitch <= tilt_max - 90 {
      set phase_ to "COAST".
      set t_brake to t.
    }
    if phase_ = "COAST"
        and speed ^ 2 >= 2 * (f_max * ship:availablethrust / m - g)
                       * max(0, h - tgt:terrainheight - h_pad) {
      set phase_ to "ARREST".
    }
    local f_now is choose f if phase_ = "BRAKE"
                   else (choose 0 if phase_ = "COAST" else f_max).
    local a_thr is f_now * ship:availablethrust / m.
    local turn_rate is speed / r_ - g / speed.
    local dt_angle is pitch_tol
                    / (max(1e-6, abs(turn_rate)) * constant:radtodeg).
    local dt_speed is v_frac * speed / (a_thr + g).
    local dt is min(dt_angle, dt_speed).
    local d_speed is (-a_thr - g * sin(pitch)) * dt.
    local d_pitch is turn_rate * cos(pitch) * constant:radtodeg * dt.
    set h     to h     + speed * sin(pitch) * dt.
    set theta to theta + speed * cos(pitch) / r_ * dt.
    set speed to speed + d_speed.
    set pitch to pitch + d_pitch.
    set m     to m     - f_now * mdot_full * dt.
    set t     to t     + dt.
    set steps to steps + 1.
  }
  if phase_ = "BRAKE" { set t_brake to t. }
  return lexicon("x", theta * body:radius, "t", t, "t_brake", t_brake).
}

// The ship's state this instant, in endpoint's variables. Captured once per
// solve so every bisection evaluation prices the same problem: the marches
// take game seconds, the ship keeps flying, and a state read fresh
// mid-solve walks the root during the search.
function live_state {
  local speed is ship:velocity:surface:mag.
  return lexicon("speed", speed,
      "pitch", arcsin(min(1, max(-1, verticalspeed / speed))),
      "h", ship:altitude,
      "m", ship:mass).
}

// The state at the coming periapsis, where ignition happens: altitude off
// the ellipse; speed from vis-viva less the ground's eastward motion,
// because the arc is flown against the ground; flight path level, periapsis
// being horizontal; the mass now, the coast burning nothing.
function pdi_state {
  local h_pe is ship:orbit:periapsis.
  return lexicon(
      "speed", orbital_speed(h_pe, ship:orbit)
             - 2 * constant:pi * (body:radius + h_pe) / body:rotationperiod,
      "pitch", 0,
      "h", h_pe,
      "m", ship:mass).
}

// Great-circle ground distance to the site — the measure endpoint's x is in.
function dist_to_site {
  return body:radius * constant:degtorad
       * vang(ship:position - body:position, tgt:position - body:position).
}

// The in-plane ground distance to the site: the great-circle distance
// with the cross-plane part removed. The throttle spends reach only along
// the velocity's own ground track; the cross part is the yaw channel's to
// close, and aiming the solve at the full distance sends the throttle
// chasing ground it cannot cover — sideways.
function aim_distance {
  parameter cross_ is vdot(tgt:position,
      vcrs(ship:velocity:surface, up:vector):normalized).
  return sqrt(max(0, dist_to_site() ^ 2 - cross_ ^ 2)).
}

// The throttle whose arc from state st covers d_aim, with the invariants
// register's feasibility ordering folded in ("Two conditions, one knob",
// case 2). The problem is frozen — st and d_aim never change during the
// search — so the root is the root of one problem, not a truncation
// against a moving target. Reach falls strictly as f rises, so a seed
// endpoint with the wrong sign says which way the root left the bracket,
// and one widening to the hard limit settles it. Each endpoint state is
// marched once; bisect re-reads its endpoints, and the closure answers
// those from the values in hand. The f = 0 probe from a near-orbital
// state runs to the step cap, and its x is a sign, not a distance —
// reach beyond d_aim is all the branch reads from it.
// Returns "case" plus the arc of the returned throttle:
//   "OK"        — a root exists in [0, f_max]; "f" is the root.
//   "OVERSHOOT" — even the f_max arc ends past the site; "f" is f_max,
//                 the throttle that books the smallest overshoot
//                 (invariants case 2: fly it, eat it, say so).
//   "SHORT"     — even the f = 0 arc lands short; "f" is f_max — the
//                 safety choice, not the accuracy one (see the register
//                 debt note) — and "x_best" carries the f = 0 arc's
//                 reach, the smallest shortfall any throttle books.
function solve_arc {
  parameter st.
  parameter d_aim.
  parameter f_lo is 0.
  parameter f_hi is f_max.

  local e_hi is endpoint(f_hi, st).
  if e_hi["x"] > d_aim and f_hi < f_max {   // root above the seed's top
    set f_hi to f_max.
    set e_hi to endpoint(f_max, st).
  }
  if e_hi["x"] > d_aim {
    return lexicon("case", "OVERSHOOT", "f", f_max, "x", e_hi["x"],
                   "t", e_hi["t"], "t_brake", e_hi["t_brake"]).
  }
  local e_lo is endpoint(f_lo, st).
  if e_lo["x"] <= d_aim and f_lo > 0 {      // root below the seed's bottom
    set f_lo to 0.
    set e_lo to endpoint(0, st).
  }
  if e_lo["x"] <= d_aim {
    return lexicon("case", "SHORT", "f", f_max, "x", e_hi["x"],
                   "t", e_hi["t"], "t_brake", e_hi["t_brake"],
                   "x_best", e_lo["x"]).
  }
  local m_lo is e_lo["x"] - d_aim.
  local m_hi is e_hi["x"] - d_aim.
  local miss_ is { parameter f_.
    if f_ = f_lo { return m_lo. }
    if f_ = f_hi { return m_hi. }
    return endpoint(f_, st)["x"] - d_aim. }.
  local f_ is bisect(miss_, f_lo, f_hi, 0.001).
  local e is endpoint(f_, st).
  return lexicon("case", "OK", "f", f_, "x", e["x"], "t", e["t"],
                 "t_brake", e["t_brake"]).
}

// === FLIGHT RECORDER ===
// One CSV row per second from the powered phases. Lines beginning '#' are the
// planning numbers the flight is judged against.
local flightlog is "flight_log.csv".

function log_state {
  parameter phase, t_go, a_thrust, cross, d_aim.
  local to_site is vxcl(up:vector, tgt:position):normalized.
  // cmd_yaw's retrograde reference collapses to zero length once the ship
  // stops; substitute a_thrust's own direction then, the same guard the
  // a_thrust argument already carries at its call sites via max(0.001, ...).
  local retro_ is choose -ship:velocity:surface
      if ship:velocity:surface:mag > 0.001 else a_thrust.
  log round(time:seconds, 1) + "," + phase + "," + round(t_go, 1) + ","
      + round(altitude) + "," + round(alt:radar) + ","
      + round(vdot(ship:velocity:surface, to_site), 1) + ","
      + round(verticalspeed, 1) + ","
      + round(tgt:position:mag) + ","
      + round(a_thrust:mag, 2) + "," + round(throttle, 3) + ","
      + round(vang(a_thrust, ship:facing:vector), 1) + ","
      + round(ship:mass, 3) + "," + round(ship:deltav:current, 1) + ","
      + round(90 - vang(up:vector, ship:facing:vector), 1) + ","
      + round(90 - vang(up:vector, a_thrust), 1) + ","
      + round(vang(a_thrust, retro_), 1) + ","
      + round(cross) + ","
      + round(d_aim)
      to flightlog.
}

// === COAST TO PDI ===
print "Coasting to PDI: " + round(eta:periapsis) + " s.".
warpto(time:seconds + eta:periapsis - 60).
wait until eta:periapsis <= 60.
sas off.                   // kOS warns at run time that SAS fights lock steering
lock steering to srfretrograde.

// Solve the ignition throttle for the state the ship will have at
// periapsis, not the state it has now. Solved for a future state, the
// solve's own duration is free: the answer is applied at the instant it
// was solved for, however long the marches take. The aim distance is
// measured to where the site will be at ignition — the ground rotates
// east under the coast — while the cross part is read from the present
// geometry: the orbital plane is fixed in space, and the site's
// cross-plane motion over the coast is metres.
local t_pdi is time + eta:periapsis.
local n_seed is vcrs(ship:velocity:surface, up:vector):normalized.
local cross_seed is vdot(tgt:position, n_seed).
local d_aim is sqrt(max(0,
    ground_track_distance(t_pdi, tgt, ship:orbit) ^ 2 - cross_seed ^ 2)).
local sol is solve_arc(pdi_state(), d_aim).
local f_cmd is sol["f"].
if sol["case"] = "OVERSHOOT" {
  print "OVERSHOOT at ignition: flying f_max, smallest overshoot "
      + round(sol["x"] - d_aim) + " m.".
}
if sol["case"] = "SHORT" {
  print "SHORT at ignition: flying f_max, best reach "
      + round(d_aim - sol["x_best"]) + " m short.".
}
local t_go is sol["t"].                // time to predicted touchdown
// k_yaw: e-foldings of the ignition cross-range offset the yaw channel
// removes during braking; the high-gate residual is e^-k_yaw of the
// offset. Derived, not chosen: ln(offset / y_floor) is exactly the count
// that lands the residual on the floor — deeper spends yaw (and its
// unmodelled reach cost) on error the next phase re-creates, shallower
// leaves cross-range nothing downstream can fix. Clamped to [1, 5]: the
// clamp bounds k, not the yaw the command spends — above k = 5 the offset
// is outside any plan this planner certifies, and a failed plan should be
// re-planned, not answered with a yaw command left to grow unchecked.
local k_yaw is min(5, max(1, ln(max(0.001, abs(cross_seed)) / y_floor))).
// The plane-closing time constant: the braking horizon cut into k_yaw
// e-foldings, frozen at ignition from the periapsis-state arc so the
// shrinking horizon never demands a growing bias for whatever remains.
local tau_yaw is sol["t_brake"] / k_yaw.

// === BRAKING ===
wait until time:seconds >= t_pdi:seconds.
print "BRAKE: f " + round(f_cmd, 3) + ", "
    + round(dist_to_site() / 1000, 1) + " km to the site.".
// Deploy the legs now, not at terminal entry: ship:bounds must be read
// after the craft has finished changing shape — a cached bounds goes
// stale on gear deployment — and the braking burn gives the animation
// two minutes where the terminal coast may give seconds. In vacuum the
// deployed gear costs nothing.
gear on.

if exists(flightlog) { deletepath(flightlog). }
log "# target " + round(target_lat, 4) + " " + round(target_lng, 4)
    + "  terrain " + round(tgt:terrainheight) + " m" to flightlog.
log "# h_pdi " + round(ship:altitude) + "  speed_pdi "
    + round(ship:velocity:surface:mag, 1) + "  f_ignition " + round(f_cmd, 4)
    + "  t_go " + round(t_go, 1) + "  dist " + round(dist_to_site())
    + "  dv_at_pdi " + round(ship:deltav:current, 1)
    + "  t_brake " + round(sol["t_brake"], 1)
    + "  tau_yaw " + round(tau_yaw, 1) to flightlog.
log "t,phase,t_go,alt,radar,v_to_site,v_vert,aim_dist,a_cmd,throttle,facing_err,mass,dv_rem,pitch,cmd_pitch,cmd_yaw,cross,d_aim"
    to flightlog.
if not (sol["case"] = "OK") {
  log "# case " + sol["case"] + "  at_ignition  f " + round(f_cmd, 3)
      + "  d_aim " + round(d_aim) + "  x " + round(sol["x"])
      + (choose "  x_best " + round(sol["x_best"])
             if sol["case"] = "SHORT" else "") to flightlog.
}

// Retrograde, biased to close the plane. n is built from the velocity, so
// v . n = 0 identically: a yaw never pushes the ship sideways, it turns
// the ground track, and the offset y closes at (azimuth turn rate) * X,
// with X the in-plane ground distance remaining. A pretend cross-speed b
// turns the track at a_thr * b / (speed * v_h), so the closure delivered
// is b * gain with gain = a_thr * X / (speed * v_h) — measured 0.48-0.58,
// the half the old law silently left in. Dividing the command by the gain
// makes the closure y / tau_yaw by construction. The correction is capped
// at 2 — its exact value when X equals the arc's own stopping distance
// speed * v_h / (2 * a_thr), which is what X is whenever the solve has
// converged. X below that is off the solved family (overshoot, or the
// end-game where the channel is dead), and holding the on-family
// correction there is the freeze the design wants, bought with no new
// constant; the dying speed then fades the bias out on its own.
function braking_dir {
  local v_ is ship:velocity:surface.
  local n is vcrs(v_, up:vector):normalized.
  local y is vdot(tgt:position, n).
  local v_h is vxcl(up:vector, v_):mag.
  local a_thr is f_cmd * ship:availablethrust / ship:mass.
  local corr is min(2, v_:mag * v_h / max(1, a_thr * aim_distance(y))).
  return -(v_ - n * (y / tau_yaw) * corr).
}
lock steering to lookdirup(braking_dir(), ship:facing:topvector).
lock throttle to f_cmd.

// The exit is high gate: hand off when retrograde has come within tilt_max
// of plumb. The handoff changes nothing about the attitude — terminal keeps
// holding retrograde — it ends the re-solving, whose control is spent, and
// starts the coast toward the arrest burn.
//
// Re-solve every 5 s until within ~10 deg of high gate: that last stretch is
// seconds long and metres-of-reach per unit of throttle have collapsed, so
// the last solution rides to the handoff. The 10 deg is a soft cutoff on a
// spent control, not a tuned landing number.
//
// t_frozen is the instant the adopted solution's state was captured — the
// clock t_go ages against. t_solved is the instant the last solve
// returned — the cadence clock, so every look is followed by five
// seconds of flown, logged flight. Both start at ignition: the seed
// already describes the ignition state, so an immediate re-solve would
// only re-derive it, and the recorder gets the seed's first five seconds
// as flown witness instead of a gap.
local t_frozen is time:seconds.
local t_solved is time:seconds.
local t_logged is 0.
local x_solved is sol["x"].
local miss_solved is x_solved - d_aim.
local noticed is "".
until false {
  local retro_ang is vang(up:vector, srfretrograde:vector).
  if retro_ang <= tilt_max { break. }
  if retro_ang > tilt_max + 10 and time:seconds - t_solved >= 5 {
    // Freeze the problem at this instant, then solve it: the root must
    // belong to one state and one distance, not to a target re-read while
    // the ship closes on it at hundreds of m/s. The bracket rides the
    // previous root; a seed the root has left just widens to the hard
    // limits, priced at one march.
    local t_cap is time:seconds.
    set d_aim to aim_distance().
    set sol to solve_arc(live_state(), d_aim,
        max(0, f_cmd - f_bracket), min(f_max, f_cmd + f_bracket)).
    if sol["case"] = "OK" or sol["case"] = "OVERSHOOT" {
      set f_cmd to sol["f"].
      set t_go to sol["t"].
      set x_solved to sol["x"].
      set miss_solved to x_solved - d_aim.
      set t_frozen to t_cap.
    }
    if sol["case"] = "OK" { set noticed to "OK". }
    if not (sol["case"] = "OK") and not (sol["case"] = noticed) {
      local note_ is choose
          "OVERSHOOT: flying f_max, smallest overshoot "
              + round(sol["x"] - d_aim) + " m."
          if sol["case"] = "OVERSHOOT" else
          "SHORT: holding f=" + round(f_cmd, 3) + ", best reach "
              + round(d_aim - sol["x_best"]) + " m short.".
      print note_ + "        " at (0, 11).
      log "# case " + sol["case"] + "  t " + round(time:seconds, 1)
          + "  f " + round(f_cmd, 3) + "  d_aim " + round(d_aim)
          + "  x " + round(sol["x"])
          + (choose "  x_best " + round(sol["x_best"])
                 if sol["case"] = "SHORT" else "") to flightlog.
      set noticed to sol["case"].
    }
    set t_solved to time:seconds.
  }
  if time:seconds - t_logged >= 1 {
    local n is vcrs(ship:velocity:surface, up:vector):normalized.
    log_state("BRAKE", max(0, t_go - (time:seconds - t_frozen)),
        f_cmd * (ship:availablethrust / ship:mass) * braking_dir():normalized,
        vdot(tgt:position, n), d_aim).
    // x is the solved arc's reach and miss its gap against the site, both
    // priced at the last re-solve — a march is worth one look per solution,
    // not one per second — while d is the live ground distance, closing
    // between solves. Printed in place at a fixed row (trailing spaces
    // overwrite a prior, longer line) so the readout updates rather than
    // scrolling the screen.
    local d is dist_to_site().
    print "BRK f=" + round(f_cmd, 3) + " x=" + round(x_solved)
        + " d=" + round(d) + " miss=" + round(miss_solved)
        + " v=" + round(ship:velocity:surface:mag, 1) + "        "
        at (0, 10).
    set t_logged to time:seconds.
  }
  wait 0.
}

// === TERMINAL DESCENT ===
// The braking law continued to the ground. From high gate the ship keeps
// holding surface retrograde: first a coast, engine off, until f_max could
// just bring the speed to rest at the pad — the same schedule the planning
// integration ignites on — then the arrest burn. Thrust opposite the
// velocity takes descent rate and drift together, and gravity rotates the
// velocity vector toward plumb as the horizontal dies, so the ship reaches
// the last metres upright and drift-free with no lateral law. The throttle
// holds the vertical deceleration a_req that carries the descent rate to
// v_floor at the pad; the v/|vv| factor is the retrograde lean's cosine,
// restoring the vertical share the lean sends sideways, and it is gated
// with the steering — below v_switch the nose is plumb and there is no
// lean to pay for. If the ship runs
// behind schedule the request rises past f_max into the reserve on its own;
// near the ground a_req falls below hover and the ship settles instead of
// bouncing. Below v_switch the retrograde direction is mostly noise — a
// small tolerance like pitch_tol, not a landing number — and the nose goes
// to plumb for touchdown.
// Blank BRAKE's row-11 case notice so a stale one does not persist into
// the terminal readout. Printed before the scrolling print below so the
// blank always lands on row 11, regardless of whether that print scrolls
// the screen.
print "                                                            " at (0, 11).
print "TERMINAL: coasting from " + round(alt:radar) + " m.".
local v_switch is 5.
// The terminal phase's altimeter: the height of the craft's lowest point
// above the ground. alt:radar measures from the core, which sits metres
// above the legs' contact point, so heights against it plan the flare
// into the ground by the craft's own core height. ship:bounds is
// obtained once — the docs price obtaining it as expensive, and the box
// only goes stale if the ship changes shape, which it finished doing
// when the gear deployed at ignition; the bottomaltradar suffix off the
// stored box is the cheap per-tick read, and it tracks attitude, so
// while the craft still leans it reads the corner that would touch
// first. The one-time check guards an API this program has not flown: a
// box reading above the core, or a core height beyond any lander's
// geometry, falls back to the core radar — the flown, six-metres-wrong
// behavior — rather than flying a nonsense number.
local box is ship:bounds.
local dh_core is alt:radar - box:bottomaltradar.
local use_box is dh_core >= 0 and dh_core <= 20.
if not use_box {
  print "WARN: bounds datum " + round(dh_core, 1) + " m; using core radar.".
}
// The capture happens up to tilt_max off plumb (high gate just passed),
// and the box rotates with the craft, so dh_core carries an attitude
// term of a few tenths of a metre. Logging the tilt it was read at makes
// the number comparable with the touchdown pair, which is measured near
// plumb.
log "# bounds  dh_core " + round(dh_core, 2)
    + "  tilt " + round(vang(up:vector, ship:facing:vector), 1)
    + (choose "" if use_box else "  REJECTED") to flightlog.
local lock h_bot to choose box:bottomaltradar if use_box else alt:radar.
local lock a_dec to f_max * ship:availablethrust / ship:mass - g0.
local lock a_req to (verticalspeed ^ 2 - v_floor ^ 2)
                  / (2 * max(1, h_bot - h_pad)).
local burning is false.
// High gate in the witness: the drift the arrest burn inherits, the offset
// the flight can no longer correct, and the ground position — with the
// landed position and the target in the header, enough to split a miss
// into along-track and cross-track afterward.
log "# high gate  radar " + round(alt:radar)
    + "  drift " + round(vxcl(up:vector, ship:velocity:surface):mag, 1)
    + "  offset " + round(vxcl(up:vector, tgt:position):mag)
    + "  lat " + round(ship:geoposition:lat, 4)
    + "  lng " + round(ship:geoposition:lng, 4) to flightlog.
lock steering to lookdirup(
    (choose srfretrograde:vector if ship:velocity:surface:mag > v_switch
            else up:vector),
    ship:facing:topvector).
lock throttle to choose 0 if not burning
    else (g0 + a_req)
       * (choose ship:velocity:surface:mag / max(1, -verticalspeed)
                 if ship:velocity:surface:mag > v_switch else 1)
       * ship:mass / max(0.001, ship:availablethrust).
set t_logged to 0.
local t_printed is 0.
until ship:status = "LANDED"
    or (h_bot < h_pad and verticalspeed > -0.1) {
  if not burning
      and ship:velocity:surface:mag ^ 2
          >= 2 * a_dec * max(0, h_bot - h_pad) {
    set burning to true.
    print "ARREST: from " + round(h_bot) + " m.".
  }
  // log_dt: seconds between CSV rows — 0.25 under 40 m radar altitude,
  // 1 s otherwise.
  local log_dt is choose 0.25 if alt:radar < 40 else 1.
  if time:seconds - t_logged >= log_dt {
    // Log the thrust actually commanded; max(0.001, ...) keeps the vector
    // pointing somewhere when the engine is off. The cross column carries
    // the full horizontal drift speed — v_to_site is blind to the
    // tangential component.
    log_state((choose "ARREST" if burning else "COAST"), 0,
        max(0.001, throttle * ship:availablethrust / ship:mass)
        * (choose srfretrograde:vector
                  if ship:velocity:surface:mag > v_switch else up:vector),
        vxcl(up:vector, ship:velocity:surface):mag,
        vxcl(up:vector, tgt:position):mag).
    set t_logged to time:seconds.
  }
  if time:seconds - t_printed >= 1 {
    // Fixed-row readout in BRK's idiom: v the descent rate, sched the total
    // speed that ignites the arrest burn, f the throttle, miss the
    // horizontal offset that becomes the landing error, drift the
    // horizontal speed the burn is spending.
    print "TRM b=" + round(h_bot) + " v=" + round(verticalspeed, 1)
        + " sched=" + round(sqrt(2 * a_dec * max(0, h_bot - h_pad)))
        + " f=" + round(throttle, 3)
        + " miss=" + round(vxcl(up:vector, tgt:position):mag)
        + " drift=" + round(vxcl(up:vector, ship:velocity:surface):mag, 1)
        + "     " at (0, 10).
    set t_printed to time:seconds.
  }
  wait 0.
}
lock throttle to 0.
// Touchdown witness: state at ground contact, before the settle wait — vv
// the descent rate, drift the horizontal speed, tilt off plumb, radar the
// core altimeter reading, bot the height of the craft's lowest point
// above ground — the pair re-measures dh_core at contact.
log "# touchdown  vv " + round(verticalspeed, 1)
    + "  drift " + round(vxcl(up:vector, ship:velocity:surface):mag, 1)
    + "  tilt " + round(vang(up:vector, ship:facing:vector), 1)
    + "  radar " + round(alt:radar, 1)
    + "  bot " + round(h_bot, 1) to flightlog.
// Hold the ship plumb while the legs settle. lookdirup with the current
// topvector asks only for the nose: bare `up` is a full direction, roll
// included, and the steering manager would grind the landed ship around
// its legs to satisfy a roll nothing needs.
lock steering to lookdirup(up:vector, ship:facing:topvector).
// Hand off to SAS only once the craft has stopped moving: below ~1 deg/s of
// rotation the legs have stopped rocking. The clock is a hung-wait guard —
// rocking on a slope may never settle below the threshold.
local t_land is time:seconds.
// Settle trace: elapsed time since touchdown, tilt off plumb, and angular
// rate, once a second while the legs settle.
local t_settle_logged is 0.
until ship:angularvel:mag < 0.02 or time:seconds - t_land > 10 {
  if time:seconds - t_settle_logged >= 1 {
    log "# settle  t " + round(time:seconds - t_land, 1)
        + "  tilt " + round(vang(up:vector, ship:facing:vector), 1)
        + "  angularvel " + round(ship:angularvel:mag, 3) to flightlog.
    set t_settle_logged to time:seconds.
  }
  wait 0.
}
unlock steering.
unlock throttle.
set ship:control:pilotmainthrottle to 0.
sas on.
set config:ipu to ipu_prior.
local miss is vxcl(up:vector, tgt:position):mag.
// tilt: degrees off plumb after the legs settle. The landing target is a
// parked craft, so uprightness is a measured witness, not an assumption.
local tilt is round(vang(up:vector, ship:facing:vector), 1).
print "Landed. Miss: " + round(miss) + " m.  Tilt: " + tilt + " deg.".
log "# landed  miss " + round(miss) + " m  lat "
    + round(ship:geoposition:lat, 4) + "  lng "
    + round(ship:geoposition:lng, 4) + "  tilt " + tilt + "  dv_rem "
    + round(ship:deltav:current, 1) to flightlog.
