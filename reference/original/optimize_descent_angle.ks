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
// Flats — and it is a property of the node, so it belongs with the node,
// in plan_doi's verdict. It is not written yet; until it is, the coast is
// still the human's risk.

@lazyglobal off.

clearscreen.
print "=== OPTIMIZE DESCENT ANGLE ===".

// kepler for wrap_longitude; the survey needs nothing else from the stack.
run "../core/kepler".

// The tallest terrain anywhere on the body, metres above the datum — the
// one fact this script cannot read: kOS reports terrain per coordinate
// and has no body-wide maximum (Minmus peaks near 5725 m). It bounds the
// walk: once the ray tops it, nothing beyond can matter.
parameter max_terrain_height.
parameter target_lat is 0.
parameter target_lng is 0.
// The seam with plan_doi: the ray is anchored landing_height above the
// site, exactly where plan_doi anchors it. Pass a non-default value on to
// plan_doi unchanged, or the slope solved here certifies a different line
// than the one planned there.
parameter landing_height is 50.
// How far the terrain model is trusted, metres: every sample is treated
// as this much taller than kOS reports. At the forcing obstacle the ray's
// clearance is exactly this number — and the flown arc's concavity bonus
// above the ray, real elsewhere, goes to zero as the obstacle nears the
// site — so this is the whole of the margin there, not a topping-up of
// some other one.
parameter terrain_margin is 50.
// The shallowest approach worth flying regardless of how flat the survey
// reads, degrees. Terrain that demands less than this — the Great Flats
// demand nothing at all — gets this instead: plan_doi needs some slope to
// price, and a ray indistinguishable from level trusts the terrain model
// over hundreds of kilometres.
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
if max_terrain_height <= tgt:terrainheight {
  survey_abort("max_terrain_height is " + round(max_terrain_height)
      + " m but the site's own terrain is " + round(tgt:terrainheight)
      + " m; the body's peak cannot sit below the site.").
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

// The walk's far bound: a quarter of the body. Past 90 degrees of arc,
// "up-range along the approach" has stopped meaning anything a straight
// ray can certify, and on a small body a shallow floor would otherwise
// walk most of the way around — the one-degree ray tops Minmus 330 km
// out, most of its circumference. Terrain beyond the cap is the coast
// rule's problem, and hitting the cap is reported, not hidden.
local x_cap is constant:pi * body:radius / 2.
if x_cap / dx > 100000 {
  survey_abort("dx " + dx + " m means " + round(x_cap / dx) + " samples"
      + " to the quarter-body cap; raise dx.").
}

local x is 0.
local g_run is 0.          // the steepest demand seen so far, degrees
local force_x is 0.        // where it was seen; 0 means nothing demanded
local force_h is 0.
local force_lng is 0.
local samples is 0.
local ray_closed is false. // true: the ray topped the peak inside the cap
local profile is list().   // decimated (x, terrain) pairs for the log
local next_prof is 0.
local next_note is 20000.

until false {
  set x to x + dx.
  if x > x_cap { break. }
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
    set next_prof to next_prof + 2000.
  }
  if x >= next_note {
    print "  swept " + round(x / 1000) + " km; demand so far "
        + round(g_run, 2) + " deg.".
    set next_note to next_note + 20000.
  }
  // The walk ends itself: the ray, at the steeper of the running answer
  // and the floor, has climbed past the tallest terrain the body owns
  // (plus the same distrust every sample gets), so no obstacle beyond
  // can demand more — whatever stands out there tops out at
  // max_terrain_height, and the slope that reaches a bounded height
  // falls as 1/x.
  if h_handoff + x * tan(max(g_run, gamma_floor))
      > max_terrain_height + terrain_margin {
    set ray_closed to true.
    break.
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
    + gamma_floor + " deg  dx " + dx + " m  peak "
    + round(max_terrain_height) + " m").
if g_run >= gamma_floor {
  report("# bound: terrain — " + round(force_h) + " m at lng "
      + round(force_lng, 4) + ", " + round(force_x / 1000, 1)
      + " km up-range").
} else if force_x > 0 {
  report("# bound: gamma_floor — the steepest terrain demand was only "
      + round(g_run, 2) + " deg (" + round(force_h) + " m at "
      + round(force_x / 1000, 1) + " km)").
} else {
  report("# bound: gamma_floor — no terrain on the walk rose above the"
      + " anchor at all").
}
report("# walk " + samples + " samples over " + round(x / 1000, 1)
    + " km, " + (choose "closed: the ray topped the peak" if ray_closed
                 else "CAPPED at a quarter of the body")).
if not ray_closed {
  print "NOTE: the walk hit the quarter-body cap with the ray still below"
      + " the peak — routine on a shallow slope over a small body."
      + " Terrain beyond " + round(x / 1000) + " km is uncertified here;"
      + " it lies far up-range of any plausible PDI, under the coast,"
      + " whose clearance rule this survey does not own.".
}
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
