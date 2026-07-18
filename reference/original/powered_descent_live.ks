// powered_descent_live.ks — the powered descent, from PDI to touchdown,
// re-planned from live state instead of flown against a stored table.
// Design: notes/powered-descent-invariants.md. powered_descent.ks is the
// table-tracking predecessor and stays alongside for comparison; both fly
// the ellipse plan_doi.ks leaves (the allowance bias in plan_doi's lead is
// simply absorbed by the first look here).
//
// The braking burn is a gravity turn flown in reverse, and a gravity turn
// flies itself: hold thrust surface-retrograde and gravity rotates the path
// from level to vertical while the burn takes the speed. Because thrust is
// held retrograde, the trajectory is a one-parameter family — the current
// state plus a throttle determines the whole arc — so every few seconds the
// program integrates the arc forward FROM THE SHIP'S CURRENT STATE and asks
// one question: which throttle ends this arc over the site? That throttle
// is the command. The stored table, the trim gain, the overshoot allowance
// and its taper, and the one-sided ratchet of powered_descent.ks all
// existed to manage the staleness of a prediction made once at PDI;
// re-predicting retires them. What survives of the ratchet is the physical
// asymmetry behind it, stated once as an inequality: the throttle whose arc
// bottoms at the handoff altitude is the floor under the command, because
// stretching the arc below the gate is planning it into the ground.
//
// The same look is the safety invariant. The arc at f_max is the highest,
// shortest arc this craft has; if even its bottom falls below the handoff,
// no throttle closes the descent, and that is the abort — checked before
// the coast from the ellipse, and every look thereafter from the ship.
//
// Deliberately bare-bones: the reference implementation of the guidance
// design, not an operational autopilot. Steps a more complex mission would
// need are marked "OMITTED:" at the point they would go.

@lazyglobal off.

clearscreen.
print "=== POWERED DESCENT (LIVE) ===".

// common for engine_isp; kepler for orbital_speed, time_of_periapsis,
// geoposition_at and, through its own runoncepath, bisect. Both files
// define orbital_speed with different signatures; kepler runs last so its
// (altitude, orbit) form — the one called below — is the survivor.
run "common".
run "../core/kepler".

parameter target_lat is 0.
parameter target_lng is 0.
// The arc ends this far above the site's terrain and this slow. Below and
// after, the terminal rate controller flies. Every metre of it is spent in a
// near-hover, so it is gravity loss; the floor under it is terminal's room to
// flare, null drift, and stay clear of the ground.
parameter landing_height is 50.
// Speed at which the arc ends and terminal takes over. The gravity turn's
// turn rate carries speed in its denominator, so the arc must stop above
// zero; terminal's reference descent rate is capped at this same speed so
// the handoff is continuous.
parameter speed_handoff is 5.
// Ceiling on the commanded throttle. The reserve (1 - f_max) is the whole
// authority margin: the solve may command up to it, never past it, so a
// solution that needs more is the infeasibility signal.
parameter f_max is 0.85.
// Floor of the solve's bracket: a throttle this low burns long enough that
// the arc falls below the handoff on any ellipse worth flying, which is
// what bisection needs from that end.
parameter f_min is 0.05.
// Throttle resolution of the solve. Coarser than powered_descent.ks's
// 1e-4 on purpose: that solve ran once, so its answer had to hold to the
// endpoint; this one is repeated every few seconds and only has to beat
// the drift a single look accrues. 0.001 of throttle is metres of endpoint.
parameter f_epsilon is 0.001.
// Steps for a march spanning the whole burn. Each look scales this by the
// fraction of the speed span still to burn, so early and late looks draw
// their arcs at the same fidelity per second of flight. Smaller than the
// table rendition's 500 because a look's error only has to hold until the
// next look, not to the end of the burn.
parameter arc_steps is 150.
// Seconds between re-solves. The arc flies itself between looks — that is
// the retrograde hold's whole point — so the cadence only has to be short
// against how fast dispersion accumulates, and long against the time one
// look's marches take. A look costs a second or two of game time at the
// IPU setting below; the locks keep flying the ship while it runs.
parameter solve_period is 5.
// Stop re-solving when the predicted time to handoff falls below this.
// Metres of endpoint per unit of throttle collapse as the arc shortens, so
// late solves would slam the command between its bounds chasing metre-scale
// misses. The last solution rides to the handoff; terminal's drift cascade
// owns the last few metres by charter.
parameter t_go_freeze is 10.
// Fraction of the braking thrust the cross-track correction may spend
// steering off retrograde. The loss is 1 - cos(yaw), quadratic in the
// angle, so one percent buys about eight degrees of yaw. Demand past the
// cap saturates and the script warns: a plane that far off is the
// planner's error to fix, not the braking phase's to absorb.
parameter steering_loss_budget is 0.01.

local tgt is body:geopositionlatlng(target_lat, target_lng).
// The altitude, above the datum, where the arc ends and terminal begins:
// landing_height above the site's terrain. Every solve aims or measures
// against it.
local h_handoff is tgt:terrainheight + landing_height.
// Locked, not sampled: mass falls through the burn, so readers during it
// (the recorder, the steering law) see the live acceleration.
local lock a_max to ship:availablethrust / ship:mass.

// A dead stage cannot be planned around: every quantity below divides by
// the engine's thrust or its flow rate.
if ship:availablethrust <= 0 {
  print "ABORT: no live engine. Stage or activate the descent engine,".
  print "then rerun. Nothing has been committed.".
  wait until false.
}

// Mass leaves through the engine at thrust / (Isp * g0) at full throttle;
// the stepper scales it by the throttle. engine_isp reads the first live
// engine, so one engine type burns at a time.
local mdot_full is ship:availablethrust / (engine_isp() * constant:g0).

// The looks are hundreds of Euler steps each; run the processor at its
// ceiling for the duration and put the setting back on every exit path.
local ipu_prior is config:ipu.
set config:ipu to 2000.

// OMITTED: Phase 0 (plane alignment). We assume the descent ellipse already
// passes over the site; the planner owns that.

// === THE ARC ===
// The fuel-optimal airless descent is a gravity turn flown in reverse: hold
// thrust surface-retrograde and let gravity rotate the velocity vector from
// horizontal to straight down as the burn bleeds off speed, arriving vertical
// over the target. Retrograde is the minimum-Delta-v direction to null a
// velocity vector; it cancels the vertical component while the craft is still
// fast and centrifugal support makes vertical cheap; and it spends the least
// time slow, where gravity loss accrues fastest.
//
// Euler's method from an arbitrary seed: hold the state in a handful of
// numbers, compute how much each changes over a short interval dt, add the
// changes on, repeat. Only the endpoint returns — where the arc bottoms
// out, how far down-range, and when — because nothing here stores a path:
// the path is re-derived whenever it is wanted.
function integrate_arc {
  parameter f.            // throttle, as a fraction of full thrust
  // The state the march leaves from, as a lexicon: h (altitude above the
  // datum), speed (surface speed), pitch (of the velocity, degrees above
  // the horizon — negative descending), m (mass). Built by seed_from_orbit
  // before the burn and seed_from_ship during it; the integrator does not
  // care which, and that indifference is why PDI is not special here.
  parameter seed.
  // Step budget for the march.
  parameter steps_ is arc_steps.

  local h is seed["h"].
  local speed is seed["speed"].
  local pitch is seed["pitch"].
  local m is seed["m"].

  local thrust is f * ship:availablethrust.   // constant: the throttle holds
  local mdot is f * mdot_full.
  // The step: the burn's estimated duration over the step budget. The
  // estimate ignores the speed gravity feeds back along the path, so the
  // real burn runs longer; the loop cap below gives it four times the room.
  local dt is (speed - speed_handoff) * m / thrust / steps_.

  // Angle swept around the body's centre since the seed, radians, so that
  // theta * body:radius is ground distance — the quantity the solve
  // compares against the measured distance to the site.
  local theta is 0.
  local t is 0.
  local steps is 0.

  until speed <= speed_handoff or steps >= 4 * steps_ {
    // r_ trails an underscore because kOS reserves R() for rotations.
    local r_ is body:radius + h.
    local g is body:mu / r_ ^ 2.
    // The ship lightens as it burns, so the same thrust buys more braking
    // late in the arc.
    local a_thrust is thrust / m.

    // Every increment reads the state as it stands; they all apply together
    // at the bottom.
    //
    // Thrust points straight back along the velocity, so all of it changes
    // speed. Gravity splits: the part along the path adds speed as the nose
    // drops below the horizon, and the part across the path steers, on the
    // next line.
    local d_speed is (-a_thrust - g * sin(pitch)) * dt.
    // Two rates turn pitch. The local horizon rotates under the ship at
    // speed*cos(pitch)/r as it flies around the body, tipping the nose up.
    // Gravity pulls across the velocity at g*cos(pitch)/speed, tipping it
    // down. The two match at orbital speed and pitch holds. Below it gravity
    // wins and the path steepens toward vertical; above it the horizon wins
    // and the arc climbs. radtodeg converts the rate to degrees because kOS
    // trigonometry works in degrees.
    local d_pitch is (speed / r_ - g / speed) * cos(pitch)
                     * constant:radtodeg * dt.
    // The vertical and horizontal halves of the speed.
    local d_h is speed * sin(pitch) * dt.
    local d_theta is speed * cos(pitch) / r_ * dt.

    set speed to speed + d_speed.
    set pitch to pitch + d_pitch.
    set h     to h     + d_h.
    set theta to theta + d_theta.
    set m     to m     - mdot * dt.
    set t     to t     + dt.
    set steps to steps + 1.
  }

  // speed at or below speed_handoff marks a closed arc; above it, the march
  // spent its whole step budget with speed still to burn, and callers read
  // that off this endpoint.
  return lexicon("t", t, "speed", speed, "h", h, "x", theta * body:radius).
}

// === SEEDS ===

// The state at periapsis of an ellipse, for the looks that run before the
// burn: PDI's altitude is read off the orbit, the speed there is vis-viva
// less the motion of the ground underneath (the arc is flown against the
// ground, not the stars; equatorial per the Phase 0 assumption), and the
// velocity is level because periapsis is where it is level.
function seed_from_orbit {
  parameter orbit_ is ship:orbit.
  local h is orbit_:periapsis.
  local r_pe is orbit_:body:radius + h.
  return lexicon(
    "h", h,
    "speed", orbital_speed(h, orbit_)
             - 2 * constant:pi * r_pe / orbit_:body:rotationperiod,
    "pitch", 0,
    "m", ship:mass).
}

// The state the ship is actually in, for the looks that run during the
// burn. Ignition slop, DOI placement error, and mid-burn dispersion all
// enter the plan through here, identically: they are just the current
// state. Mass is live too, so depletion error resets every look instead of
// compounding from PDI.
function seed_from_ship {
  local speed is ship:velocity:surface:mag.
  return lexicon(
    "h", ship:altitude,
    "speed", speed,
    "pitch", arcsin(max(-1, min(1, verticalspeed / max(speed, 0.1)))),
    "m", ship:mass).
}

// === DISTANCES ===

// Great-circle ground distance between two positions: the angle between
// their radial directions, times the radius — the same measure as the
// march's x. A chord would understate it by tens of metres over a long
// descent.
function ground_distance {
  parameter p1, p2.
  return body:radius * constant:degtorad
       * vang(p1 - body:position, p2 - body:position).
}

function dist_to_site {
  return ground_distance(ship:position, tgt:position).
}

// === THE SOLVES ===
// The endpoint must satisfy two conditions — over the site, at the gate
// altitude — and there is one knob. Both endpoint coordinates are monotone
// in f: down-range falls as f rises (a harder burn is a shorter arc), the
// bottom climbs (a shorter burn gives gravity less time to pull the path
// down). So each condition pins its own throttle, and braking_look below
// orders them.

// The throttle whose arc, marched from seed, ends dist metres down-range.
// Callers establish the bracket before calling — the f_min arc reaches at
// least dist, the f_max arc falls short of it — so bisection always has
// its sign change.
function solve_f_to_site {
  parameter seed.
  parameter dist.
  parameter steps_.

  local miss is {
    parameter f.
    local e is integrate_arc(f, seed, steps_).
    // A march that spent its whole budget exited with speed still to burn:
    // wherever it stopped, the true arc runs longer. Report it far long,
    // which steers the search toward more throttle.
    if e["speed"] > speed_handoff { return 1e9. }
    return e["x"] - dist.
  }.
  return bisect(miss, f_min, f_max, f_epsilon).
}

// The throttle whose arc bottoms out exactly at the handoff altitude —
// powered_descent.ks's whole solve, demoted here to the floor under the
// command and the fallback when the site is out of reach. Same contract:
// callers establish the bracket (f_min bottoms below the gate, f_max at or
// above it) before calling.
function solve_f_to_gate {
  parameter seed.
  parameter steps_.

  local miss is {
    parameter f.
    local e is integrate_arc(f, seed, steps_).
    // Speed left means the arc is still falling: its true bottom is lower.
    if e["speed"] > speed_handoff { return -1e9. }
    return e["h"] - h_handoff.
  }.
  return bisect(miss, f_min, f_max, f_epsilon).
}

// One look: from a seed state and a measured distance to the site, choose
// the throttle. Returns f (the command, or -1 for infeasible), end (the
// endpoint of the arc that command flies), and mode. The two probe arcs at
// the bracket's ends classify the geometry before any solve runs, so every
// bisection below is called on a guaranteed sign change. The feasibility
// ordering:
//   1. The arc at f_max is the highest, shortest arc this craft has. If
//      even its bottom is below the gate — or it cannot finish inside the
//      step budget — nothing closes above the gate. That is the abort, and
//      the caller owns what abort means: re-plan before the coast,
//      emergency_land during the burn.
//   2. If even that arc ends at or past the site, every throttle books an
//      overshoot; fly f_max and eat the smallest one.
//   3. If even the f_min arc — the longest there is — lands short, no
//      throttle reaches the site: aim the bottom at the gate instead and
//      accept the short landing. (If every bottom already clears the gate,
//      fly f_min itself.)
//   4. Otherwise the site is bracketed; solve for it. A site solution whose
//      bottom dips below the gate is pulled up to the gate throttle: the
//      gate outranks the site. A wrong-place landing beats a right-place
//      crater — emergency_land's trade, applied continuously.
function braking_look {
  parameter seed.
  parameter dist.
  parameter steps_.

  local end_max is integrate_arc(f_max, seed, steps_).
  if end_max["speed"] > speed_handoff or end_max["h"] < h_handoff {
    return lexicon("f", -1, "end", end_max, "mode", "infeasible").
  }
  if end_max["x"] >= dist {
    return lexicon("f", f_max, "end", end_max, "mode", "long").
  }

  local end_min is integrate_arc(f_min, seed, steps_).
  // The step-budget guard in the solves reads a spent budget as "long", so
  // an unfinished f_min march lands in the bracketed case below, which is
  // where it belongs: its true arc runs past the site.
  if end_min["speed"] <= speed_handoff and end_min["x"] < dist {
    if end_min["h"] >= h_handoff {
      return lexicon("f", f_min, "end", end_min, "mode", "short").
    }
    local f_gate is solve_f_to_gate(seed, steps_).
    return lexicon("f", f_gate,
                   "end", integrate_arc(f_gate, seed, steps_),
                   "mode", "short").
  }

  local f_site is solve_f_to_site(seed, dist, steps_).
  local e is integrate_arc(f_site, seed, steps_).
  if e["h"] < h_handoff {
    // f_site's bottom is below the gate and f_max's is above it: bracketed.
    local f_gate is solve_f_to_gate(seed, steps_).
    if f_gate > f_site {
      return lexicon("f", f_gate,
                     "end", integrate_arc(f_gate, seed, steps_),
                     "mode", "gate").
    }
  }
  return lexicon("f", f_site, "end", e, "mode", "site").
}

// === ABORTS ===

// OMITTED: active aborts. An operational script would fly abort-to-orbit
// (thrust up and up-range back to a stable ellipse) from any powered phase
// rather than handing a falling ship to the pilot.

// Nothing has burned yet: the ship is on a stable ellipse and stays there.
// Used before the coast and at ignition, where stopping still costs nothing.
function abort_in_orbit {
  parameter why.
  set config:ipu to ipu_prior.
  print "ABORT: " + why.
  print "Nothing has been committed. Re-plan the descent ellipse.".
  wait until false.
}

// The descent has diverged with no altitude left to argue about it: stop
// solving, kill velocity at full thrust, and land where we are. The target
// is abandoned — a wrong-place landing beats a right-place crater. Ends
// parked: the mission sequence must not continue past this.
function emergency_land {
  parameter why.
  set config:ipu to ipu_prior.
  print "EMERGENCY: " + why.
  print "Abandoning target. Killing velocity; landing here.".
  lock steering to srfretrograde.
  lock throttle to 1.
  wait until ship:velocity:surface:mag < 10 or ship:status = "LANDED".
  terminal_descent(false).
  print "Down, off target. You have the ship.".
  wait until false.
}

// === PHASE 1: COAST TO PDI ===

function coast_to_pdi {
  local coast_margin is 60.      // stop warp this far before PDI (s): time
                                 // for the steering lock to swing the ship
                                 // retrograde, with margin

  // OMITTED: coast monitoring. On longer missions an on-screen abort
  // instrument belongs here — "if the engine never lights, where do I
  // hit?" (core/impact.ks predict_impact) — plus the option of a
  // meter-per-second trim, cheap now and expensive later.

  print "Coasting to PDI: " + round(eta:periapsis) + " s.".
  warpto(time:seconds + eta:periapsis - coast_margin).
  wait until eta:periapsis <= coast_margin.

  // Pre-orient for the braking burn while the last minute runs out.
  // 1 s is "at periapsis" for our purposes: the first look integrates from
  // whatever state ignition actually finds, so timing slop only needs to be
  // small against the burn, not against the plan.
  lock steering to srfretrograde.
  wait until eta:periapsis <= 1.
}

// === FLIGHT RECORDER ===
// One CSV row per second from the powered phases, so flights are analyzed
// from telemetry instead of remembered impressions. Lands in this directory
// (the kOS archive); overwritten on each run. Lines beginning '#' are
// metadata — the planning numbers each flight is judged against.
//
// Same columns as powered_descent.ks so the analysis tooling reads both.
// The signature to watch here is the throttle column during BRAKE: the
// solver's health is a staircase of small steps drifting smoothly;
// oscillation between looks is the failure mode the design note names.
local flightlog is "flight_log.csv".

function log_state {
  parameter phase, t_go, aim_geo, aim_alt, a_thrust.
  parameter cross is 0.
  local to_site is vxcl(up:vector, tgt:position):normalized.
  log round(time:seconds, 1) + "," + phase + "," + round(t_go, 1) + ","
      + round(altitude) + "," + round(alt:radar) + ","
      + round(vdot(ship:velocity:surface, to_site), 1) + ","
      + round(verticalspeed, 1) + ","
      + round(aim_geo:altitudeposition(aim_alt):mag) + ","
      + round(a_thrust:mag, 2) + "," + round(throttle, 3) + ","
      + round(vang(a_thrust, ship:facing:vector), 1) + ","
      + round(ship:mass, 3) + "," + round(ship:deltav:current, 1) + ","
      + round(90 - vang(up:vector, ship:facing:vector), 1) + ","
      + round(90 - vang(up:vector, a_thrust), 1) + ","
      + round(cross)
      to flightlog.
}

// === THE BRAKING PHASE ===
// Hold thrust along the commanded braking direction and let the arc fly
// itself. Every solve_period seconds, one look re-derives the throttle
// from live state; every second, the yaw law re-measures the site's offset
// from the flown plane and the recorder writes a row. Between looks the
// command holds — the arc is self-flying, and a deviation measured at the
// next look is corrected with the whole remaining burn.
function fly_braking {
  parameter look0.            // the ignition look, from the mission sequence

  local f_cmd is look0["f"].
  local a_cross is v(0, 0, 0).   // lateral acceleration demand, m/s^2
  local cross_warned is false.
  local miss_warned is false.
  local speed_pdi is ship:velocity:surface:mag.

  // t_go decrements from the last look's endpoint between looks; the yaw
  // law and the recorder read it. t_seed anchors the decrement at the
  // moment the look's seed was sampled, not when its marches finished.
  local t_seed is time:seconds.
  local t_go_solved is look0["end"]["t"].
  local frozen is false.

  // The commanded acceleration is retrograde braking plus the lateral
  // demand. Steering follows it, and the vang gate holds the throttle
  // closed unless the ship points near it. Nominally the gate never fires;
  // it exists to deny fuel to divergence, since mis-pointed thrust is the
  // energy source that sustains one. The locks run every physics tick, so
  // the ship keeps flying while a look's marches run in the mainline.
  lock steering to lookdirup(
      f_cmd * a_max * srfretrograde:vector + a_cross,
      ship:facing:topvector).
  lock throttle to choose f_cmd
      if vang(f_cmd * a_max * srfretrograde:vector + a_cross,
              ship:facing:vector) < 30 else 0.

  print "BRAKE: retrograde hold at f " + round(f_cmd, 4)
      + "; re-solving every " + solve_period + " s.".

  local t_logged is 0.
  until ship:velocity:surface:mag <= speed_handoff {
    local spd is ship:velocity:surface:mag.
    local t_go is max(1, t_go_solved - (time:seconds - t_seed)).

    // The look. Freezing is one-way: once the horizon is short, metres of
    // endpoint per unit of throttle have collapsed, and a solve would slam
    // the command chasing noise. The last solution rides to the handoff.
    if not frozen and time:seconds - t_seed >= solve_period {
      if t_go <= t_go_freeze {
        set frozen to true.
        print "BRAKE: t_go " + round(t_go) + " s; holding f "
            + round(f_cmd, 4) + " to the handoff.".
      } else {
        // Step budget in proportion to the speed still to burn: the same
        // fidelity per second of arc at every look, and cheaper every look.
        local steps_ is max(25, round(arc_steps * (spd - speed_handoff)
                            / max(speed_pdi - speed_handoff, 1))).
        local t_this is time:seconds.
        local look is braking_look(seed_from_ship(), dist_to_site(), steps_).
        if look["f"] < 0 {
          // The safety invariant, violated: even the f_max arc bottoms
          // below the gate. No throttle closes the descent from here.
          emergency_land("BRAKE: no throttle keeps the arc above the"
              + " handoff (f_max bottom "
              + round(h_handoff - look["end"]["h"]) + " m below it).").
        }
        if look["mode"] <> "site" and not miss_warned {
          print "WARNING: endpoint " + look["mode"] + " of the site; flying f "
              + round(look["f"], 4) + ". Expect to miss "
              + (choose "long." if look["mode"] = "long" else "short.").
          set miss_warned to true.
        }
        set f_cmd to look["f"].
        set t_go_solved to look["end"]["t"].
        set t_seed to t_this.
      }
    }

    // The 1 Hz work: the backstop, the yaw law, the recorder.
    if time:seconds - t_logged >= 1 {
      // Ground-proximity backstop. The planner's gamma certification
      // promises the arc at least landing_height of terrain clearance, so
      // radar below that with real speed still to burn means the flight
      // has left the certified envelope — a fact no forward prediction
      // reports, which is why this check reads the radar and not a march.
      if alt:radar < landing_height and spd > 3 * speed_handoff {
        emergency_land("BRAKE: radar " + round(alt:radar) + " m with "
            + round(spd) + " m/s still to burn.").
      }

      // Cross-track. The site's signed offset from the plane the ship is
      // flying in is the lateral miss the arc will book if nothing steers,
      // because an unforced gravity turn stays in its plane. The
      // constant-jerk profile that arrives centred with no drift asks for
      // 6 y / t_go^2 of lateral acceleration now; it is delivered by
      // yawing the thrust, and the cap keeps that yaw inside the steering
      // budget. Re-measuring against the live velocity each look absorbs
      // the correction's own progress and the body's rotation alike.
      local n_cross is vcrs(ship:velocity:surface, up:vector):normalized.
      local y_pred is vdot(tgt:position, n_cross).
      set a_cross to n_cross * (6 * y_pred / t_go ^ 2).
      // The cap fades with the fraction of the speed still horizontal: the
      // law steers by rotating the ground track, and as the path goes
      // vertical near handoff that leverage and n_cross's definition vanish
      // together — the same geometry, so no separate constant.
      local a_cross_max is f_cmd * a_max * sqrt(2 * steering_loss_budget)
                        * vxcl(up:vector, ship:velocity:surface):mag / spd.
      if a_cross:mag > a_cross_max {
        set a_cross to a_cross:normalized * a_cross_max.
        if not cross_warned {
          print "WARNING: cross-track demand saturated. The plane misses the".
          print "site by more than the steering budget corrects.".
          set cross_warned to true.
        }
      }

      log_state("BRAKE", t_go, tgt, h_handoff,
          f_cmd * a_max * srfretrograde:vector + a_cross, y_pred).
      set t_logged to time:seconds.
    }
    wait 0.
  }
}

// === TERMINAL DESCENT (P66) ===

// Rate-of-descent control, which is what Apollo's P66 was: the reference
// descent rate is a function of radar altitude, and the throttle servos the
// actual rate onto it around a gravity-cancelling feedforward.
//
// The reference profile is -min(speed_handoff, max(2, alt:radar / 10)):
//   - capped at speed_handoff so it is continuous with the arc's arrival
//     speed (uncapped, alt/10 would command -15 m/s at 150 m, beginning
//     the phase by speeding the descent back up);
//   - proportional to height (tau = 10 s) through the middle;
//   - floored at 2 m/s so touchdown happens rather than being approached
//     asymptotically.
function terminal_descent {
  // False when called from emergency_land: the target is abandoned, so the
  // position loop is zeroed and only the drift damper flies.
  parameter chase_site is true.

  print "TERMINAL: rate-of-descent control from " + round(alt:radar) + " m.".

  // OMITTED: site redesignation and slope/quality checks. The ground below
  // is trusted because it was surveyed before the mission — that trust is
  // the core design trade, not an oversight.

  local g0 is body:mu / body:radius ^ 2.
  local lock v_ref to -min(speed_handoff, max(2, alt:radar / 10)).
  local v_cap is choose 3 if chase_site else 0.

  // Gravity feedforward plus a proportional correction. The 0.3 gain is
  // in units of 1/s: an error of 1 m/s adds 0.3 m/s^2 of commanded accel,
  // a ~3.3 s closed-loop time constant — brisk against the 10 s reference
  // profile, yet 5 m/s of error demands only 1.5 m/s^2 over the
  // feedforward, well inside TWR-2 authority. max() guards flameout.
  lock throttle to (g0 + 0.3 * (v_ref - verticalspeed)) * ship:mass
                   / max(0.001, ship:availablethrust).

  // Two loops in cascade. The outer turns position error into a commanded
  // closing drift: 0.2 m/s per metre of horizontal offset to the site,
  // capped at v_cap, so beyond 15 m the ship closes at a constant 3 m/s and
  // inside that it eases off linearly. The inner is the drift damper: 0.1
  // of tilt per m/s of velocity error, ~6 degrees per m/s, so the cap also
  // bounds the lean this close to the ground at ~17 degrees. Over the site
  // the command is zero and this reduces to plain drift-nulling — which is
  // why the damper owns the inner loop: a lander that tips hard at 10 m to
  // chase 30 m of miss trades a wrong-place landing for a tipped-over one.
  function tilt {
    local off is vxcl(up:vector, tgt:position).
    local v_err is vxcl(up:vector, ship:velocity:surface)
                 - off * min(0.2, v_cap / max(0.001, off:mag)).
    return up:vector - 0.1 * v_err.
  }

  lock steering to lookdirup(tilt(), ship:facing:topvector).

  gear on.

  // LANDED is the real signal; the fallback catches a hover balanced just
  // above contact — 5 m is landing-leg scale, -0.1 m/s is "effectively
  // stopped." The recorder runs here too: a_cmd is the ROD servo's demand
  // along the commanded steering direction; t_go is not a quantity this
  // phase has, so it logs 0; aim_dist is live distance to the site — the
  // touchdown drift.
  local t_logged is 0.
  until ship:status = "LANDED"
      or (alt:radar < 5 and verticalspeed > -0.1) {
    if time:seconds - t_logged >= 1 {
      log_state("TERMINAL", 0, tgt, tgt:terrainheight,
          (g0 + 0.3 * (v_ref - verticalspeed)) * tilt():normalized).
      set t_logged to time:seconds.
    }
    wait 0.
  }
  lock throttle to 0.
  print "Contact. Settling.".
  wait 3.                          // settle on the legs before releasing control
  unlock steering.
  unlock throttle.
  set ship:control:pilotmainthrottle to 0.
}

// === MISSION SEQUENCE ===

if hasnode {
  abort_in_orbit("a maneuver node is still pending. Burn it first; this"
      + " script begins on the descent ellipse, not before it.").
}

// The pre-flight check is the flight's own first look, seeded from the
// ellipse instead of the ship: the coast is unpowered, so the state at
// periapsis is already knowable, and the distance from PDI to the site is
// the ground between the periapsis footprint and the target. An infeasible
// descent is caught here, with the whole coast still ahead and the orbit
// still raisable — by the same test every in-flight look will run.
local t_pdi is time_of_periapsis(time, ship:orbit).
local pdi_geo is geoposition_at(t_pdi, ship:orbit).
local dist_pdi is ground_distance(pdi_geo:position, tgt:position).
local seed0 is seed_from_orbit(ship:orbit).
local look0 is braking_look(seed0, dist_pdi, arc_steps).

if look0["f"] < 0 {
  abort_in_orbit("even f_max cannot keep the arc above the handoff. PDI is "
      + round(seed0["h"]) + " m over a site at " + round(tgt:terrainheight)
      + " m: this craft cannot fly this ellipse down.").
}
if look0["mode"] = "long" {
  // Short is flyable — the descent lands short and says so — but long at
  // full throttle before ignition means PDI is inside the site's minimum
  // braking distance, and every metre flown makes it worse.
  abort_in_orbit("even f_max overshoots the site by "
      + round(look0["end"]["x"] - dist_pdi) + " m. PDI is too close in;"
      + " re-plan with more lead.").
}
if look0["mode"] <> "site" {
  print "WARNING: the plan reaches the gate " + round(dist_pdi
      - look0["end"]["x"]) + " m short of the site. Expect to land short".
  print "unless terminal walks the difference.".
}

print "PDI predicted: " + round(seed0["h"]) + " m at "
    + round(seed0["speed"], 1) + " m/s, " + round(dist_pdi / 1000, 1)
    + " km from the site.".
print "First look: f " + round(look0["f"], 4) + " (" + look0["mode"] + "), "
    + round(look0["end"]["t"], 1) + " s, ending "
    + round(look0["end"]["h"] - h_handoff) + " m above the gate.".

coast_to_pdi().

// Ignition look, from the ship the coast actually delivered. The gap
// between it and look0 is the whole prediction error of the coast — worth
// a line in the log, and nothing more: the looks absorb it.
local look1 is braking_look(seed_from_ship(), dist_to_site(), arc_steps).
if look1["f"] < 0 {
  abort_in_orbit("at PDI, even f_max cannot keep the arc above the"
      + " handoff. The ellipse is still stable; re-plan.").
}
print "PDI arrival: f " + round(look1["f"], 4) + " (" + look1["mode"]
    + ") vs " + round(look0["f"], 4) + " planned.".

// The plane the coast delivered, measured the same way the yaw law will
// measure it: the site's signed offset from the plane of the surface
// velocity.
local n_pdi is vcrs(ship:velocity:surface, up:vector):normalized.
local cross_pdi is vdot(tgt:position, n_pdi).
print "Plane misses the site laterally by " + round(abs(cross_pdi)) + " m.".

if exists(flightlog) { deletepath(flightlog). }
log "# target " + round(target_lat, 4) + " " + round(target_lng, 4)
    + "  terrain " + round(tgt:terrainheight) + " m" to flightlog.
log "# h_pdi " + round(ship:altitude) + "  speed_pdi "
    + round(ship:velocity:surface:mag, 1)
    + "  planned " + round(seed0["h"]) + " / " + round(seed0["speed"], 1)
    + "  h_handoff " + round(h_handoff) to flightlog.
log "# f_ignition " + round(look1["f"], 4) + " (" + look1["mode"]
    + ")  f_planned " + round(look0["f"], 4)
    + "  twr_pdi " + round(a_max / (body:mu / (body:radius + ship:altitude) ^ 2), 1)
    to flightlog.
log "# arc  duration " + round(look1["end"]["t"], 1) + " s  downrange "
    + round(look1["end"]["x"]) + " m  end_h " + round(look1["end"]["h"])
    + "  solve_period " + solve_period + "  t_go_freeze " + t_go_freeze
    to flightlog.
log "# cross_pdi " + round(cross_pdi) + "  dv_at_pdi "
    + round(ship:deltav:current, 1) to flightlog.
log "t,phase,t_go,alt,radar,v_to_site,v_vert,aim_dist,a_cmd,throttle,facing_err,mass,dv_rem,pitch,cmd_pitch,cross"
    to flightlog.

fly_braking(look1).
terminal_descent().

set config:ipu to ipu_prior.

// The headline number: horizontal distance from the touchdown point to the
// target site.
local miss is vxcl(up:vector, tgt:position):mag.
print "Landed. Miss distance: " + round(miss) + " m.".
log "# landed  miss " + round(miss) + " m  dv_rem " + round(ship:deltav:current, 1)
    to flightlog.
