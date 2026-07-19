// optimize_descent_angle.ks — mission planning, the half plan_doi.ks leaves
// to judgment: survey the approach and solve gamma, the descent angle.
// Design: notes/capability-driven-descent.md (piece 3, "the smart planner").
// gamma is an input to plan_doi.ks and an output of this script; everything
// downstream of gamma — h_pdi, the lead, the node — stays plan_doi's. Run
// this, read the slope and the obstacle that forced it, then run plan_doi
// with the answer.
//
// The optimum needs no search. Delta-v rises with gamma — a steeper
// approach buys its clearance with a higher PDI, a longer brake, and more
// gravity loss, a trend plan_doi's sweep prices every time it runs — so
// the cheapest certified plan is the shallowest certified slope, and
// "optimize" means "find the binding obstacle". gamma is the steepest ray
// any obstacle demands:
//
//   gamma = max over x of
//           arctan((terrain(x) + terrain_margin - h_handoff) / x)
//
// x being ground distance up-range from the site and h_handoff the arc's
// endpoint, landing_height above the site's terrain. The ray certifies
// the flown arc geometrically: the gravity turn leaves PDI level and
// steepens monotonically, so the arc is concave and lies above the
// straight line from the handoff up to PDI everywhere — terrain the line
// clears, the flight clears. The survey samples no trajectory, reads
// nothing from the ship but the body it stands on, and can run before the
// parking orbit exists.
//
// One assumption stands in for the design note's node coupling. The note
// wants the sweep to follow the real ground track, which depends on the
// node's timing — but that coupling only bites on an inclined orbit, and
// the whole stack assumes a prograde equatorial parking orbit (plan_doi
// seeds its ellipses with it; its verdict warns when the plane misses the
// site). Under that assumption the approach crosses the site along its
// own parallel, west to east, and that parallel is what this script
// walks. When an inclined approach is worth flying, the survey joins
// plan_doi's fixed point; until then the decoupling is what lets gamma
// exist before the node does.
//
// What this script does not certify: the coast. Up-range of PDI the ray
// says nothing useful — the coast leaves periapsis flat and climbs
// quadratically, so the ray rides far above it for most of a hemisphere
// and would happily certify a mountain the coast flies into. The coast's
// rule is a flat clearance walked along the placed ellipse (design note,
// open item 1), it binds for real — measured ten metres over the Great
// Flats — and it is a property of the node, so it lives with the node:
// plan_doi walks the placed ellipse from the burn to PDI and refuses any
// plan whose clearance dips under its coast_clearance. This survey
// certifies the arc's corridor and nothing more.

@lazyglobal off.

clearscreen.
print "=== OPTIMIZE DESCENT ANGLE ===".

// kepler for wrap_longitude; the survey needs nothing else from the stack.
run "../core/kepler".

parameter target_lat is 0.
parameter target_lng is 0.
// The seam with plan_doi: the ray is anchored landing_height above the
// site, exactly where plan_doi anchors it. Pass a non-default value on to
// plan_doi unchanged, or the slope solved here certifies a different line
// than the one planned there.
parameter landing_height is 50.
// How far the terrain model is trusted, metres: every sample is treated
// as this much taller than kOS reports. At the forcing obstacle the ray's
// clearance is exactly this number — the flown arc's concavity bonus
// above the ray, real elsewhere, goes to zero as the obstacle nears the
// site — so this is the whole of the margin there. Defaults to
// landing_height: up-range terrain gets the same benefit of the doubt as
// the clearance granted at the site — one judgment, not two — until the
// model earns a number of its own.
parameter terrain_margin is landing_height.
// The shallowest approach worth flying regardless of how flat the survey
// reads, degrees. A near-level ray puts PDI barely above the handoff
// altitude and lays the coast along the ground for the length of the
// approach — plan_doi's coast walk will refuse that plan, but only after
// the fixed point has spent its passes on it; the floor keeps the survey
// from proposing it in the first place. Some slope must be chosen for
// plan_doi to price; over the Great Flats, which demand nothing, this
// is it.
parameter gamma_floor is 1.
// Sample spacing, metres — the design note's open item 8: IPU budget
// against stepping over a spire.
parameter dx is 100.

local tgt is body:geopositionlatlng(target_lat, target_lng).
local h_handoff is tgt:terrainheight + landing_height.

// The survey is a few thousand terrain reads; run them at the processor's
// ceiling and put the setting back on the way out.
local ipu_prior is config:ipu.
set config:ipu to 2000.

// Every abort path: restore the processor setting and stop. Nothing else
// to unwind — this script places no node and touches no control.
function survey_abort {
  parameter why.
  set config:ipu to ipu_prior.
  print "ABORT: " + why.
  print "Nothing has been committed: this script only reads terrain.".
  wait until false.
}

if dx <= 0 {
  survey_abort("dx is " + dx + "; the survey cannot walk a non-positive"
      + " step.").
}
if terrain_margin < 0 {
  survey_abort("terrain_margin is " + terrain_margin + "; distrust of the"
      + " terrain model cannot be negative.").
}
if gamma_floor <= 0 or gamma_floor >= 90 {
  survey_abort("gamma_floor is " + gamma_floor + "; it is a descent slope"
      + " in degrees and must lie strictly between 0 and 90.").
}

print "target " + round(target_lat, 4) + " " + round(target_lng, 4)
    + ", terrain " + round(tgt:terrainheight) + " m; ray anchored "
    + landing_height + " m above it.".
if abs(target_lat) > 5 {
  print "WARNING: the site sits at latitude " + round(target_lat, 2)
      + " deg; the equatorial-track assumption this survey walks under is"
      + " strained, and plan_doi's plane check will say so too.".
}

// Metres of ground per degree of longitude along the site's parallel.
local m_per_deg is body:radius * cos(target_lat) * constant:degtorad.

// The walk's span: a quarter of the body, always. The straight ray cannot
// claim more — past 90 degrees of arc, "up-range along the approach" has
// stopped meaning anything it can certify — and it does not need to: a
// plausible PDI sits tens of kilometres up-range, and terrain past PDI is
// under the coast, whose rule this survey does not own. A body-wide
// terrain maximum could end the walk sooner, but kOS holds no such
// number, and on the bodies flown so far the quarter-body bound binds
// first anyway — a one-degree ray does not top Minmus's 5725 m peak until
// ~330 km out, most of the way around. So the span is derived, not
// supplied, and the price is a few thousand cheap samples.
local x_cap is constant:pi * body:radius / 2.
if x_cap / dx > 100000 {
  survey_abort("dx " + dx + " m means " + round(x_cap / dx) + " samples"
      + " to the quarter-body span; raise dx.").
}

// Reporting strides, derived from the span: about fifty profile lines
// and a handful of progress notes on any body.
local prof_step is max(dx, x_cap / 50).
local note_step is x_cap / 5.

local x is 0.
local g_run is 0.          // the steepest demand seen so far, degrees
local force_x is 0.        // where it was seen; 0 means nothing demanded
local force_h is 0.
local force_lng is 0.
local samples is 0.
local profile is list().   // decimated (x, terrain) pairs for the log
local next_prof is 0.
local next_note is note_step.

until x + dx > x_cap {
  set x to x + dx.
  local lng_i is wrap_longitude(target_lng - x / m_per_deg).
  local terr is body:geopositionlatlng(target_lat, lng_i):terrainheight.
  set samples to samples + 1.
  local demand is arctan((terr + terrain_margin - h_handoff) / x).
  if demand > g_run {
    set g_run to demand.
    set force_x to x.
    set force_h to terr.
    set force_lng to lng_i.
  }
  if x >= next_prof {
    profile:add(list(x, terr)).
    set next_prof to next_prof + prof_step.
  }
  if x >= next_note {
    print "  swept " + round(x / 1000) + " km; demand so far "
        + round(g_run, 2) + " deg.".
    set next_note to next_note + note_step.
  }
}

// === THE VERDICT ===

local gamma is max(g_run, gamma_floor).

// The survey, printed and kept: gamma_survey.log is the witness the
// judgment used to be — plan_doi's plan is only as good as the corridor
// this file certifies.
local surveylog is "gamma_survey.log".
if exists(surveylog) { deletepath(surveylog). }
function report {
  parameter line.
  print line.
  log line to surveylog.
}

report("# gamma " + round(gamma, 2) + " deg  target "
    + round(target_lat, 4) + " " + round(target_lng, 4) + "  terrain "
    + round(tgt:terrainheight) + " m").
report("# anchor " + round(h_handoff) + " m (landing_height "
    + landing_height + ")  margin " + terrain_margin + " m  floor "
    + gamma_floor + " deg  dx " + dx + " m").
if g_run >= gamma_floor {
  report("# bound: terrain — " + round(force_h) + " m at lng "
      + round(force_lng, 4) + ", " + round(force_x / 1000, 1)
      + " km up-range").
} else {
  report("# bound: gamma_floor — "
      + (choose "the steepest terrain demand was only "
             + round(g_run, 2) + " deg (" + round(force_h) + " m at "
             + round(force_x / 1000, 1) + " km)"
         if force_x > 0
         else "no terrain on the walk rose above the anchor at all")).
}
report("# walk " + samples + " samples over " + round(x / 1000, 1)
    + " km — a quarter of the body; terrain beyond is coast country,"
    + " not this survey's to certify").
if gamma > 30 {
  print "WARNING: gamma " + round(gamma, 1) + " deg is not an approach"
      + " corridor, it is a wall next to the site. Move the site rather"
      + " than flying this.".
}

// The corridor itself, log only: enough of the profile to plot the ray
// against the ground it clears.
log "# profile: x_m,terrain_m,ray_m" to surveylog.
for p in profile {
  log round(p[0]) + "," + round(p[1]) + ","
      + round(h_handoff + p[0] * tan(gamma)) to surveylog.
}

set config:ipu to ipu_prior.
print "Survey done. Fly it: run plan_doi(" + round(gamma, 2) + ", "
    + target_lat + ", " + target_lng + ")."
    + (choose "" if landing_height = 50
       else " Pass landing_height " + landing_height + " along too.").
