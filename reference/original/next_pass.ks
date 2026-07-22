// next_pass.ks — when does the current orbit next pass near a ground
// target? Prediction only: walks up to max_orbits revolutions ahead with
// core/kepler's ground_target_approach and reports the earliest pass
// within tolerance. Nothing here steers, burns, or warps, and a pending
// maneuver node is not consulted — the question is about the orbit as it
// stands.

@lazyglobal off.

parameter lat is 0.
parameter lng is 0.
parameter tolerance is 10000.   // metres of acceptable ground-track miss
parameter max_orbits is 8.

run "../core/kepler".

clearscreen.
print "=== NEXT PASS ===".

local tgt is body:geopositionlatlng(lat, lng).
print "target " + round(lat, 4) + " " + round(lng, 4) + ", terrain "
    + round(tgt:terrainheight) + " m; tolerance "
    + round(tolerance / 1000, 1) + " km.".

// The search is a few hundred Kepler solves per revolution walked; run
// it at the processor's ceiling and put the setting back.
local ipu_prior is config:ipu.
set config:ipu to 2000.
local pass is ground_target_approach(tgt, tolerance, max_orbits).
set config:ipu to ipu_prior.

if pass["distance"] < 0 {
  print "No pass found; is the orbit closed?".
} else {
  local geo is pass["closest"].
  print "closest pass: rev " + pass["rev"] + ", eta " + round(pass["eta"])
      + " s (" + round(pass["eta"] / 60, 1) + " min), miss "
      + round(pass["distance"] / 1000, 2) + " km.".
  print "ground track there: " + round(geo:lat, 4) + " "
      + round(geo:lng, 4) + ".".
  if pass["ok"] {
    print "Within tolerance.".
  } else {
    print "NOT within tolerance. A target latitude beyond the orbit's"
        + " inclination (" + round(ship:orbit:inclination, 1) + " deg)"
        + " can never do better; otherwise more revolutions might.".
  }
}
