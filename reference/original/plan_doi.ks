// plan_doi.ks — the DOI planner: one node that drops a parking orbit onto a
// descent ellipse whose periapsis is PDI, placed for a Klumpp-guided braking
// phase (see notes/klumpp-descent-redesign.md).
//
// The planner answers two questions with two instruments. How high is PDI:
// the reference arc — the constant-throttle gravity turn at f_cap — marched
// forward from a candidate periapsis until it slows to the gate speed, with
// the periapsis altitude solved so that stall lands at the gate altitude.
// How far up-range: not the reference arc's reach — the flight flies
// E-guidance, a different curve — but the lead at which the guidance
// profile's peak thrust demand sits exactly at f_cap, so the f_cap..f_max
// band is reserved whole for post-ignition dispersion.
//
// The DOI burn is tangential, so periapsis lands roughly half an orbit ahead
// of it. The placement asks the game where the node it just made puts
// periapsis, and moves the burn by the miss.
//
// The output is the node. Burn it, then run powered_descent.ks. f_max and
// h_gate here must match the values that program is run with, or the demand
// sized here is not the demand flown.
//
// Terrain: h_gate above the site's terrain is the one clearance input. The
// ground under the braking arc is uncertified (terrain-certification.md);
// the modelled E-guidance path clears the straight PDI-to-gate chord over
// its whole length, so a single offline march per placement could certify
// braking-arc clearance — noted, deliberately not built.

@lazyglobal off.

clearscreen.
print "=== PLAN DOI ===".

// kepler for time_to_longitude, time_of_periapsis, geoposition_at,
// wrap_longitude; bisect rides along. common for engine_isp and
// burn_duration.
run "../core/kepler".
run "common".

parameter target_lat is 0.
parameter target_lng is 0.
// High gate: the altitude over the site, metres above its terrain, where
// braking ends and the vertical descent begins. The design's single terrain
// clearance input; the corridor and the gate speed derive from it.
parameter h_gate is 300.
// The throttle ceiling the descent is flown at. Must equal
// powered_descent.ks's f_max, or the corridor priced here is not the one
// flown.
parameter f_max is 0.85.
// The share of the ceiling the plan leaves unspent: the nominal guidance
// demand is placed at f_cap = (1 - f_headroom) * f_max, and the band above
// it is command margin for dispersion arriving after ignition.
// Dimensionless, 0.1, provisional until a flight falsifies it.
parameter f_headroom is 0.1.

// A pending node is not ours to reason about — or to delete.
if hasnode {
  print "ABORT: a maneuver node is already pending. Burn or remove it first.".
  wait until false.
}
if ship:availablethrust <= 0 {
  print "ABORT: no live engine. Stage or activate the descent engine.".
  wait until false.
}

local tgt is body:geopositionlatlng(target_lat, target_lng).
// Gate altitude above the datum: h_gate above the site's terrain.
local alt_gate is tgt:terrainheight + h_gate.
local f_cap is (1 - f_headroom) * f_max.
// The flare height the arrest schedule stops above, and surface gravity.
// h_pad must match powered_descent.ks's.
local h_pad is 5.
local g_surf is body:mu / body:radius ^ 2.

// The descent needs thrust at f_max above weight, and a gate above the
// flare height, before any arc can be priced. Weight is taken at the
// ship's current mass — the heaviest the descent sees — so the test is
// conservative by the braking burn's propellant fraction.
if f_max * ship:availablethrust / ship:mass <= g_surf {
  print "ABORT: f_max thrust does not exceed weight on " + body:name
      + " — no arrest is possible.".
  wait until false.
}
if h_gate <= h_pad {
  print "ABORT: h_gate " + round(h_gate) + " m does not clear the flare"
      + " height h_pad " + round(h_pad) + " m.".
  wait until false.
}

// The march's accuracy bounds — locals because they are accuracy bounds,
// not craft or body numbers. pitch_tol caps the flight-path rotation per
// Euler step (degrees); v_frac caps the fractional speed change per step.
local pitch_tol is 1.
local v_frac is 0.02.
// How many times the placement below puts the node down and reads it back.
// The miss decays by roughly a decimal digit a pass and levels off near a
// thousandth of a degree from the sixth (measured; doi-planner.md).
local passes is 8.

// Mass leaves through the engine at thrust / (Isp * g0) at full throttle;
// the stepper scales it by the throttle.
local mdot_full is ship:availablethrust / (engine_isp() * constant:g0).

local ipu_prior is config:ipu.
set config:ipu to 2000.

print "target " + round(target_lat, 4) + " " + round(target_lng, 4)
    + ", terrain " + round(tgt:terrainheight) + " m; gate " + round(alt_gate)
    + " m (" + round(h_gate) + " over terrain).".

// Ground-relative periapsis speed of the descent ellipse with periapsis at
// datum altitude h: vis-viva with apoapsis at the parking orbit's
// semi-major-axis radius, less the ground's eastward motion, because the
// arc is flown against the ground.
function v_pe_at {
  parameter h.
  local r_pe is body:radius + h.
  local sma_desc is (r_pe + ship:orbit:semimajoraxis) / 2.
  return sqrt(body:mu * (2 / r_pe - 1 / sma_desc))
       - 2 * constant:pi * r_pe / body:rotationperiod.
}

// === THE REFERENCE ARC ===
// The constant-throttle gravity turn at f, marched forward from the periapsis
// of the ellipse a candidate h_pdi implies: pitch zero (a periapsis is
// horizontal), speed from v_pe_at, mass the ship's own — the coast burns
// nothing. The march ends where speed falls to v_stop (the arc's stall: past
// it the turn equations verticalize on the spot) or at the gate altitude,
// whichever comes first. x is the ground covered, metres; m the mass at the
// end — the gate mass, read off the march instead of iterated. The march
// flies ground-relative speed through the inertial turn equations; the
// rotating frame's 2*omega*v and omega^2*r terms are dropped, which solves
// the stall altitude of order 100 m high — toward terrain clearance, under
// 1 m/s of delta-v.
function ref_arc {
  parameter h_pdi_.
  parameter v_stop.
  parameter f.

  local h is h_pdi_.
  local speed is v_pe_at(h_pdi_).
  local pitch is 0.        // degrees above the horizon
  local m is ship:mass.
  local theta is 0.        // ground angle swept, radians
  local steps is 0.
  local thrust is f * ship:availablethrust.

  // step_cap: a scale for the march's length, not a bound on it. Each Euler
  // step retires at most one quantum of the binding constraint — v_frac of
  // speed, pitch_tol of turn — so ln(speed/v_stop)/v_frac + 90/pitch_tol is
  // the step count if every step retired a full quantum, and the factor 4
  // is headroom over that scale (measured; doi-planner.md). Hitting the
  // cap is a bug witness, and the verdict logs steps against cap.
  // max(1, ...) keeps the ln argument nonnegative if a future caller
  // hands a start already at or below v_stop.
  local step_cap is 4 * (ln(max(1, speed / v_stop)) / v_frac + 90 / pitch_tol).
  until speed <= v_stop or h <= alt_gate or steps >= step_cap {
    local r_ is body:radius + h.
    local g is body:mu / r_ ^ 2.
    local a_thr is thrust / m.
    local turn_rate is speed / r_ - g / speed.
    local dt_angle is pitch_tol
                    / (max(1e-6, abs(turn_rate)) * constant:radtodeg).
    local dt_speed is v_frac * speed / (a_thr + g).
    local dt is min(dt_angle, dt_speed).
    set h     to h     + speed * sin(pitch) * dt.
    set theta to theta + speed * cos(pitch) / r_ * dt.
    set speed to speed + (-a_thr - g * sin(pitch)) * dt.
    set pitch to pitch + turn_rate * cos(pitch) * constant:radtodeg * dt.
    set m     to m     - f * mdot_full * dt.
    set steps to steps + 1.
  }
  return lexicon("h", h, "speed", speed, "pitch", pitch, "m", m,
                 "x", theta * body:radius, "steps", steps, "cap", step_cap).
}

// === THE PDI ALTITUDE ===
// h_pdi is the altitude whose reference arc stalls to the gate speed exactly
// at the gate altitude: solved by bisection on (stall altitude - gate
// altitude), which changes sign once — an arc from too low reaches the gate
// altitude still fast, one from too high stalls above it. The gate speed
// depends on the gate mass through a_dec, and the gate mass on the march, so
// the solve runs twice: a vis-viva first guess for the burn's propellant
// seeds v_gate, and the second pass reads the mass off the first pass's arc.
// The residual v_gate change across the second pass is the loop's
// convergence witness, logged below.
local m_gate is ship:mass
    / constant:e ^ ((v_pe_at(alt_gate)) / (engine_isp() * constant:g0)).
local v_gate is sqrt(2 * (f_max * ship:availablethrust / m_gate - g_surf)
                     * (h_gate - h_pad)) / 2.
local h_pdi is 0.
local arc is 0.
local v_gate_prior is 0.
from { local pass_ is 0. } until pass_ >= 2 step { set pass_ to pass_ + 1. } do {
  // The bracket: from just above the gate (an arc from there falls into the
  // gate still fast) to the parking orbit's own periapsis (the ellipse
  // family's ceiling — above it there is no descent ellipse to place).
  // Signs checked before bisecting so a bracket failure aborts in this
  // program's own words.
  local lo is alt_gate + 1.
  local hi is ship:orbit:periapsis.
  local stall_alt is {
    parameter h_. return ref_arc(h_, v_gate, f_cap)["h"] - alt_gate. }.
  if stall_alt(lo) > 0 or stall_alt(hi) < 0 {
    set config:ipu to ipu_prior.
    print "ABORT: no periapsis in " + round(lo) + ".." + round(hi)
        + " m (gate to parking periapsis) puts the f_cap arc's stall at the"
        + " gate; raise h_gate, add thrust, or lower f_headroom.".
    wait until false.
  }
  // The solve stops when the bracket is narrower than v_frac times the
  // midpoint's height above the gate: the march carries roughly v_frac of
  // relative error, so a tighter root would be precision the function
  // does not have. The tolerance scales itself — metres when the root
  // sits close over the gate, hundreds of metres when it sits high.
  until hi - lo < v_frac * ((lo + hi) / 2 - alt_gate) {
    local mid is (lo + hi) / 2.
    if stall_alt(mid) > 0 { set hi to mid. } else { set lo to mid. }
  }
  set h_pdi to (lo + hi) / 2.
  set arc to ref_arc(h_pdi, v_gate, f_cap).
  if arc["steps"] >= arc["cap"] {
    set config:ipu to ipu_prior.
    print "ABORT: the reference march hit its step cap with the arc"
        + " unfinished — a bug witness, not a placement problem.".
    wait until false.
  }
  if arc["m"] <= ship:drymass {
    set config:ipu to ipu_prior.
    print "ABORT: the braking burn does not fit the propellant — the arc"
        + " needs " + round(ship:mass - arc["m"], 3) + " t and the tanks hold "
        + round(ship:mass - ship:drymass, 3) + " t.".
    wait until false.
  }
  set m_gate to arc["m"].
  set v_gate_prior to v_gate.
  set v_gate to sqrt(2 * (f_max * ship:availablethrust / m_gate - g_surf)
                     * (h_gate - h_pad)) / 2.
}
local a_dec is f_max * ship:availablethrust / m_gate - g_surf.
local v0 is v_pe_at(h_pdi).

// === THE LEAD ===
// The E-guidance profile the flight flies is linear in time, so its thrust
// demand peaks at an endpoint; the two endpoint demands cross once on the
// bracket, and that crossover — the minimax — is the profile's dip: the
// least peak demand any t_go buys at this lead. Each endpoint is priced at
// its own mass and local gravity. Closed form, exact under constant gravity,
// approximate here only through the ~11 degrees the arc subtends — the
// closed form under-reads the closed-loop peak by ~0.015 of throttle
// (modelled), inside the f_cap..f_max reserve.
function endpoint_demand {
  parameter tau.
  parameter x_lead.
  // The 2-D frame: x toward the site along the ground, y up. Boundary
  // conditions: from (0, h_pdi) at (v0, 0) to (x_lead, alt_gate) at
  // (0, -v_gate).
  local drx is x_lead - v0 * tau.
  local dry is alt_gate - h_pdi.
  local dvx is -v0.
  local dvy is -v_gate.
  local a0x is 6 * drx / tau ^ 2 - 2 * dvx / tau.
  local a0y is 6 * dry / tau ^ 2 - 2 * dvy / tau.
  local a1x is 4 * dvx / tau - 6 * drx / tau ^ 2.
  local a1y is 4 * dvy / tau - 6 * dry / tau ^ 2.
  local g0_ is body:mu / (body:radius + h_pdi) ^ 2.
  local g1_ is body:mu / (body:radius + alt_gate) ^ 2.
  return lexicon(
      "ign",  sqrt(a0x ^ 2 + (a0y + g0_) ^ 2) * ship:mass
            / ship:availablethrust,
      "gate", sqrt(a1x ^ 2 + (a1y + g1_) ^ 2) * m_gate
            / ship:availablethrust).
}

// The dip at a candidate lead: root of (ignition demand - gate demand) on
// the bracket [1.6, 2.6] * x / v0, which straddles the dip and excludes the
// short-burn wall and the hover branch beyond 3 * x / v0. The bracket is
// load-bearing (klumpp-descent-redesign.md, Open) and the demand curve's
// non-monotonicity is why no simpler rule serves.
function dip_at {
  parameter x_lead.
  local cross is { parameter tau.
    local pair is endpoint_demand(tau, x_lead).
    return pair["ign"] - pair["gate"]. }.
  local t_lo is 1.6 * x_lead / v0.
  local t_hi is 2.6 * x_lead / v0.
  // Same-signed gaps at the bracket ends mean the crossing — and the dip —
  // lies outside the bracket this design certifies: no profile exists at
  // this lead, and bisect's failure sentinel must not be read as a t_go.
  // "ok" false carries that outcome to the caller.
  if cross(t_lo) * cross(t_hi) > 0 {
    return lexicon("ok", false).
  }
  // The tolerance: v_frac of the profile's own timescale x_lead/v0 (the
  // bracket spans 1.6 to 2.6 of it). The flight re-solves t_go from the
  // delivered state at ignition, so this dip's t_go is a cross-check
  // against that solve, not the number flown.
  local tau is bisect(cross, t_lo, t_hi, v_frac * x_lead / v0).
  return lexicon("ok", true, "t_go", tau,
                 "demand", endpoint_demand(tau, x_lead)["ign"]).
}

// The lead X: where the dip demand equals f_cap. Demand falls as lead grows
// — more ground, gentler profile — so the root is bisected between the
// reference arc's own reach (dip above f_cap there) and a far end found by
// doubling the reach until the demand drops under it; the reach is the
// craft's own length scale, so the search grows in its units. The ceiling
// is geometric: a lead past a quarter of the body's circumference is no
// longer an approach. A dip already at or under f_cap at the arc's reach
// adopts the reach: extra lead would spend margin the plan already has.
local x_lead is arc["x"].
local dip is dip_at(x_lead).
if not dip["ok"] or dip["demand"] > f_cap {
  // A lead whose profile has no crossing in the certified bracket is a
  // lead this design cannot fly, so the search treats it exactly like
  // demand above the cap.
  local over is { parameter x_.
    local dip_ is dip_at(x_).
    return choose dip_["demand"] - f_cap if dip_["ok"] else 1. }.
  local x_far is 2 * arc["x"].
  local x_ceil is constant:pi * body:radius / 2.
  until over(x_far) <= 0 or x_far > x_ceil {
    set x_far to 2 * x_far.
  }
  if x_far > x_ceil {
    set config:ipu to ipu_prior.
    print "ABORT: guidance demand stays above f_cap " + round(f_cap, 3)
        + " for every lead out to a quarter of the body's circumference —"
        + " no lead fits this craft under its reserve. Add thrust or lower"
        + " f_headroom.".
    wait until false.
  }
  // The tolerance: v_frac of the reference arc's reach, arc["x"] — the
  // march's own length scale, and conservative here since the lead is at
  // least the reach. The delivered lead already scatters by ~1.5 km
  // (node-delivery-window.md), which the ignition t_go choice absorbs.
  set x_lead to bisect(over, arc["x"], x_far, v_frac * arc["x"]).
  set dip to dip_at(x_lead).
  if not dip["ok"] {
    set config:ipu to ipu_prior.
    print "ABORT: the endpoint-demand crossing lies outside the certified"
        + " t_go bracket at the solved lead " + round(x_lead) + " m.".
    wait until false.
  }
}
local lead_deg is x_lead / body:radius * constant:radtodeg.

// === THE NODE ===
// One retrograde node that drops periapsis to h_pdi over desired_lng. Each
// pass places the node, asks the game what that node actually does, and
// corrects the two numbers it was asked for: the burn slides west or east by
// the longitude miss, and the radius the sizing formula aims at absorbs the
// periapsis miss. Both corrections are applied at the top of a pass, so the
// last one computed is also flown into a placement.
function plan_node {
  parameter desired_lng.   // body-fixed longitude periapsis is wanted at
  parameter t_burn.        // seed burn time, roughly half an orbit before it

  // Degrees of longitude the ground track covers per second: 360 over the
  // synodic period, the body's own rotation already netted out. It turns a
  // longitude miss into the burn-time change that removes it.
  local lng_rate is 360 / synodic_period(ship:orbit).
  // The periapsis radius the vis-viva sizing aims at. It starts at the one
  // wanted and absorbs the difference between what a tangential burn off an
  // apsis delivers and what one here does.
  local r_aim is body:radius + h_pdi.
  // The previous pass's misses: metres of periapsis below h_pdi, and degrees
  // of longitude east of where periapsis was wanted. Zero on the first pass,
  // which places the node from the seed alone.
  local pe_miss is 0.
  local lng_miss is 0.

  local nd is node(t_burn:seconds, 0, 0, 0).
  add nd.
  from { local pass is 0. } until pass >= passes step { set pass to pass + 1. } do {
    set r_aim to r_aim + pe_miss.
    set t_burn to t_burn - lng_miss / lng_rate.

    // The ship's own radius and speed at the burn, read off the parking orbit.
    // positionat and velocityat follow the predicted trajectory through a
    // pending node, and at t_burn that is this node's own discontinuity.
    local st is orbit_at(t_burn, ship:orbit).
    local r_burn is st["position"]:mag.
    set nd:time to t_burn:seconds.
    // Vis-viva for the ellipse with apoapsis at the burn and periapsis at
    // r_aim, less the speed the ship already has there.
    set nd:prograde to sqrt(body:mu * (2 / r_burn - 2 / (r_burn + r_aim)))
                     - st["velocity"]:mag.

    local t_pe is time_of_periapsis(timestamp(nd:time), nd:orbit).
    set pe_miss to h_pdi - nd:orbit:periapsis.
    set lng_miss to wrap_longitude(
        geoposition_at(t_pe, nd:orbit):lng - desired_lng).
  }
  return nd.
}

// === THE PLAN ===
// Where periapsis is wanted, and the burn that seeds the search for it: half
// an orbit before that longitude, and further west by the rotation the body
// turns through while the ship coasts down.
local desired_lng is wrap_longitude(tgt:lng - lead_deg).
local sma_seed is (ship:orbit:semimajoraxis + body:radius + h_pdi) / 2.
local t_coast is constant:pi * sqrt(sma_seed ^ 3 / body:mu).
local burn_lng is wrap_longitude(desired_lng
    + t_coast * 360 / body:rotationperiod - 180).
local t_seed is time_to_longitude(burn_lng).
// A time in the past is time_to_longitude's sentinel for a root it could not
// bracket.
if (t_seed - time):seconds <= 0 {
  set config:ipu to ipu_prior.
  print "ABORT: found no crossing of the burn longitude "
      + round(burn_lng, 2) + " deg ahead.".
  wait until false.
}

local nd is plan_node(desired_lng, t_seed).
// The corrections move the burn by as much as tens of seconds, which from a
// seed already close to now can walk it into the past.
if nd:eta <= 0 {
  remove nd.
  set config:ipu to ipu_prior.
  print "ABORT: the DOI plan puts the burn in the past.".
  wait until false.
}
// Ignition leads the node by half the burn, plus a minute of orientation
// time to swing onto the burn vector — about three full reversals at the
// flown slew rate (doi-planner.md). A node closer than that burns late,
// which silently moves periapsis east. Failing is self-correcting: by the
// re-run this crossing has passed and the next is most of an orbit out.
if nd:eta < burn_duration(nd:deltav:mag) / 2 + 60 {
  remove nd.
  set config:ipu to ipu_prior.
  print "ABORT: the burn is only " + round(nd:eta) + " s away — too close to"
      + " orient and ignite. Re-run for the next crossing.".
  wait until false.
}

// === THE VERDICT ===
// Where periapsis lands and how high, against what was asked of it. These are
// the placement's residuals, not a drift it declined to correct: the loop
// aimed at both, so a search that did not converge shows up here rather than
// going unsaid.
local t_pdi is time_of_periapsis(timestamp(nd:time), nd:orbit).
local pe_lng is geoposition_at(t_pdi, nd:orbit):lng.
local pe_err is wrap_longitude(pe_lng - desired_lng).
local pe_miss is nd:orbit:periapsis - h_pdi.
local dv_doi is nd:deltav:mag.

// The plan, printed and kept: doi_plan.log is the witness the flight is judged
// against.
local planlog is "doi_plan.log".
if exists(planlog) { deletepath(planlog). }
function report {
  parameter line.
  print line.
  log line to planlog.
}
report("# PLAN DOI (KLUMPP)  target " + round(target_lat, 4) + " "
    + round(target_lng, 4) + "  terrain " + round(tgt:terrainheight) + " m").
report("# parking " + round(ship:orbit:periapsis) + " x "
    + round(ship:orbit:apoapsis) + " m  ecc "
    + round(ship:orbit:eccentricity, 4)).
report("# gate  h_gate " + round(h_gate) + " m over terrain (alt "
    + round(alt_gate) + ")  v_gate " + round(v_gate, 1) + " m/s  a_dec "
    + round(a_dec, 2) + " at m_gate " + round(m_gate, 3)
    + " t  (v_gate moved " + round(abs(v_gate - v_gate_prior), 2)
    + " on the last pass)").
report("# h_pdi " + round(h_pdi) + " m solved (node delivers "
    + round(nd:orbit:periapsis) + ", miss " + round(pe_miss, 1) + " m)").
report("# arc  reach " + round(arc["x"]) + " m at f_cap " + round(f_cap, 3)
    + "  stall pitch " + round(arc["pitch"], 1)
    + " deg (reference, not the lead)  steps " + round(arc["steps"]) + " of cap "
    + round(arc["cap"])).
report("# lead  X " + round(x_lead) + " m law-sized  dip "
    + round(dip["demand"], 3) + " at t_go " + round(dip["t_go"], 1) + " s").
report("# node  dv " + round(dv_doi, 1) + " m/s  eta " + round(nd:eta)
    + " s  pe_lng " + round(pe_lng, 2) + " want " + round(desired_lng, 2)
    + " (err " + round(pe_err, 3) + " deg)").

set config:ipu to ipu_prior.
print "Node placed. Eyeball the ellipse for terrain clearance, then burn and"
    + " run powered_descent.".
