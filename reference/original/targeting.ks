// targeting.ks -- close a loop on where a ballistic trajectory meets the
// ground, and burn until that point is the one you wanted.
//
// One idea, two directions. A booster flying home and a spaceplane flying
// out to a destination are the same problem: thrust is the only thing that
// moves a ballistic impact point, so aim the burn along the miss and stop
// when the miss is gone. What differs between them is the attitude the
// thrust is commanded at -- flat for a booster already lofted by its
// ascent, lofted for a vehicle that has to buy its own arc -- and that
// belongs to the caller. This file never steers and never throttles; it
// answers where the burn should point, how hard, and when to stop.
//
// Everything rests on one property: **on a coasting arc the predicted
// impact point does not move.** It is an invariant of the orbit. Three
// things follow, and they are most of the design:
//   - Every metre the miss closes was bought by thrust, so the measured
//     closing rate is a direct reading of the throttle's range authority
//     on this vehicle in this geometry. Nothing has to model it.
//   - Noise in the miss is noise in the predictor, not in the world, so it
//     can be filtered hard without filtering away signal.
//   - A miss that has grown past its own best can only mean the burn is
//     making the landing worse, whatever the predictor thinks it is doing.
//
// State lives in a lexicon the caller holds rather than in this file, so
// nothing is left set between runs and two loops could run at once.

@lazyglobal off.

runoncepath("common").   // landing_time, landing_site
runoncepath("aero").     // compass_for, ground_distance

// A targeting state. The caller reads these keys and this file writes them:
//   miss      metres to the aim point, horizontal. -1 until the first
//             survey lands, and it stays at its last value through a
//             survey that finds no impact.
//   miss_rate metres per second, filtered. Negative while closing.
//   miss_min  the best miss seen so far, for the overshoot test.
//   close     metres of miss closed per second PER UNIT THROTTLE -- the
//             vehicle's range authority, and what sizes the taper.
//   cmd       unit vector, horizontal, along the miss: the direction to
//             burn. The caller decides what pitch to fly it at.
//   bias      the drag correction, a raw-frame displacement (see below).
//   d_kep     great-circle metres from the drag-free impact to the target,
//             or "" when there is no impact. Logged, never flown.
//   d_tr      the same for Trajectories' drag-aware impact, or "".
function targeting_new {
  parameter tgt.                   // a GeoCoordinates to hit
  // miss_bar: how near the target the predicted impact must come before
  // the burn is finished, metres. An accuracy bound; it belongs to the
  // mission, not to the craft or the body.
  parameter miss_bar is 250.
  // t_taper: seconds of full-throttle closing left when the throttle
  // starts down, so the cut lands on the bar instead of past it.
  parameter t_taper is 2.
  // tau_bias: seconds over which the drag correction is averaged.
  parameter tau_bias is 5.
  parameter use_tr is true.
  parameter range_bias is 0.
  return lexicon(
    "tgt", tgt,
    "miss_bar", miss_bar,
    "t_taper", t_taper,
    "tau_bias", tau_bias,
    "use_tr", use_tr,
    "range_bias", range_bias,
    "miss", -1,
    "miss_rate", 0,
    "miss_min", 2 ^ 30,
    "close", 0,
    "cmd", vxcl(up:vector, tgt:position):normalized,
    "bias", v(0, 0, 0),
    "d_kep", "",
    "d_tr", "").
}

// One cycle's reading of both predictors. Returns false when the arc has
// no terrain crossing to aim at -- which during a burn means the thrust
// has lifted periapsis clear of the ground and there is nothing left to
// close on.
//
// thr is the throttle that produced the motion being measured, not a
// command: `close` is only meaningful divided by it.
//
// steer is false past the burn, where the miss is still worth logging but
// there is no longer a direction to command from it.
function targeting_survey {
  parameter st.
  parameter dt.
  parameter thr is 0.
  parameter steer is true.
  set st["d_kep"] to "".
  set st["d_tr"] to "".

  // Trajectories is asked through the addon list, not through ADDONS:TR,
  // which raises for an addon kOS has not registered (notes/kos-facts.md).
  local tr_ok is false.
  local tr_pos is v(0, 0, 0).
  if st["use_tr"] and addons:available("TR") and addons:tr:hasimpact {
    local tr_geo is addons:tr:impactpos.
    set tr_ok to true.
    set tr_pos to tr_geo:position.
    set st["d_tr"] to round(ground_distance(st["tgt"], tr_geo)).
  }

  local t_land is landing_time().
  if t_land < 0 { return false. }
  local kep is landing_site(t_land).
  set st["d_kep"] to round(ground_distance(st["tgt"], kep)).

  if tr_ok {
    // The drag correction: the displacement from the modelled impact to
    // the drag-free one -- how far past the real landing an arc with no
    // air in it carries. Filtered, because it belongs to the vehicle and
    // the arc, which change over a whole burn, while Trajectories
    // recomputes on its own clock and its answer steps. A stepping input
    // inside the rate-driven taper below would be an oscillator; this is
    // what keeps it out.
    set st["bias"] to st["bias"]
        + ((kep:position - tr_pos) - st["bias"]) * min(1, dt / st["tau_bias"]).
  }

  // Aim at the target pushed out by that correction, so that driving the
  // drag-free impact onto the aim point puts the real one on the target.
  local aim is st["tgt"]:position + st["bias"].
  if st["range_bias"] <> 0 {
    set aim to aim
        + vxcl(up:vector, st["tgt"]:position):normalized * st["range_bias"].
  }

  // The miss, taken in the ship's own horizontal plane: its direction is
  // the burn direction wanted, and over the distances in play its length
  // and the great-circle distance agree to parts in ten thousand. d_kep
  // alongside is the honest great-circle number for the record.
  local m_vec is vxcl(up:vector, aim - kep:position).
  local m_new is m_vec:mag.
  if steer and m_new > 0 { set st["cmd"] to m_vec:normalized. }

  if st["miss"] >= 0 {
    // The rate exists to drive the taper, so the taper sizes its filter:
    // half of t_taper, which settles the measurement twice over inside the
    // window the throttle it drives has to act in. Choosing a number here
    // instead would be a second time constant bearing no relation to the
    // first, and the two would then have to be tuned against each other.
    local tau_rate is st["t_taper"] / 2.
    set st["miss_rate"] to st["miss_rate"]
        + ((m_new - st["miss"]) / dt - st["miss_rate"]) * min(1, dt / tau_rate).
  }
  set st["miss"] to m_new.
  if m_new < st["miss_min"] { set st["miss_min"] to m_new. }

  // What the miss closes at per unit throttle. Dividing by the throttle
  // that produced it is what stops the taper chasing its own tail: a rate
  // measured at half throttle predicts twice that at full, so the throttle
  // commanded below is a fixed point rather than a decaying one. Only
  // measured above a tenth of throttle, where the reading means something,
  // and never reset, so the taper cannot fall back to full throttle
  // halfway down and re-open the valve.
  if thr > 0.1 and st["miss_rate"] < 0 {
    set st["close"] to -st["miss_rate"] / thr.
  }
  return true.
}

// Full throttle until the miss is within t_taper seconds of closing at
// full throttle, then linearly down so the cut lands on the bar. Shut
// while the nose is outside align_bar, which makes the alignment wait a
// property of the loop instead of a phase of its own: the burn does not
// run until the turn is done, and stops again if the craft loses it.
function targeting_throttle {
  parameter st.
  parameter cmd_dir.        // the attitude actually commanded, as a vector
  parameter align_bar is 10.
  if vang(cmd_dir, ship:facing:vector) > align_bar { return 0. }
  if st["close"] <= 0 { return 1. }
  return max(0, min(1, st["miss"] / (st["close"] * st["t_taper"]))).
}

// The two stop conditions that belong to the loop rather than to the
// vehicle. Returns "" while the burn should go on. Everything else that
// ends a burn -- dry tanks, a floor, a deadline, the pilot -- is the
// caller's, because only the caller knows what it is flying.
function targeting_done {
  parameter st.
  if st["miss"] < 0 { return "". }
  if st["miss"] <= st["miss_bar"] { return "miss inside the bar". }
  if st["miss"] > st["miss_min"] + st["miss_bar"] {
    return "miss growing past its best".
  }
  return "".
}

// The miss as a log column: empty until the first survey lands, the same
// way d_kep and d_tr go empty when their predictor has no answer.
function targeting_miss_col {
  parameter st.
  if st["miss"] < 0 { return "". }
  return round(st["miss"]).
}

// === THE CLOSED FORMS ===
//
// The loop above needs none of these -- it measures instead. They are here
// because a burn should be able to say, before it lights, whether the
// vehicle can do the job at all, and where to point while the loop has not
// yet had anything to measure. Both are for a ballistic arc between two
// points at the same radius on a non-rotating sphere, so on Kerbin they
// are an aim and a lower bound, not an answer. The loop closes the rest.

// The central angle between the ship's ground position and a target,
// degrees. This is the argument both forms below are written in, because
// it is the only length scale a sphere has: range over radius.
function range_angle {
  parameter tgt.
  return ground_distance(tgt) / body:radius * constant:radtodeg.
}

// The flight-path angle above the local horizon that reaches range angle
// phi for the least energy: 45 - phi/4.
//
// Both ends check. At phi -> 0 it gives 45 degrees, the flat-ground answer
// everyone knows. At phi = 180, the antipode, it gives 0 -- launch
// horizontally, which is right, because the cheapest way to the far side
// of a sphere is a grazing circular orbit. Between them it interpolates
// linearly, which is the standard ballistic-missile result.
function ballistic_loft {
  parameter phi.
  return 45 - phi / 4.
}

// The speed that reaches range angle phi on that minimum-energy arc:
// v^2 = (mu/r) * 2*sin(phi/2) / (1 + sin(phi/2)).
//
// Both ends check again. At phi = 180, sin(90) = 1 and the expression
// collapses to v^2 = mu/r, circular speed -- the same grazing orbit the
// loft angle just described. At small phi, sin(phi/2) ~ phi/2 and it
// becomes v^2 = mu*phi/r = g*R for ground range R, which is the flat-earth
// maximum range at 45 degrees. A formula that lands on both textbook cases
// from opposite directions is one worth trusting as a lower bound.
//
// It is a lower bound and nothing more: no drag, no gravity losses, no
// rotating ground, and burnout is assumed at the impact radius rather than
// however high the vehicle actually is. A mission that cannot reach this
// speed cannot fly; one that can may still not.
function ballistic_speed {
  parameter phi.
  parameter r_ is body:radius + ship:altitude.
  local s is sin(phi / 2).
  return sqrt(body:mu / r_ * 2 * s / (1 + s)).
}
