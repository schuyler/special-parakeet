// next_pass.ks — how close does the ground track come to a prospective
// landing site, and how long until it gets there? Walks a search window
// and reports the closest approach inside it: time to overflight, the
// miss distance, and the point on the track where it happens. Prediction
// only; nothing here steers, burns, or warps.
//
// A pending maneuver node is honored. The node's orbit is the patch the
// ship flies after the burn, so the search starts at the node rather than
// now — asking the same question of the trajectory being planned.

@lazyglobal off.

parameter lat is 0.
parameter lng is 0.
parameter window is -1.         // seconds to search; -1 is five revolutions
parameter tolerance is 10000.   // metres of miss worth calling a pass

run "../core/kepler".

clearscreen.
print "=== NEXT PASS ===".

// Which trajectory the question is about, and the instant the ship is on
// it. A node's patch describes nothing until its burn is done.
local orbit_ is ship:orbit.
local t0 is time.
local traj is "current orbit".
if hasnode {
  set orbit_ to nextnode:orbit.
  set t0 to time + nextnode:eta.
  set traj to "orbit after the pending node".
}

local body_ is orbit_:body.
local tgt is body_:geopositionlatlng(lat, lng).
print "target " + round(lat, 4) + " " + round(lng, 4) + " on " + body_:name
    + ", terrain " + round(tgt:terrainheight) + " m.".

if orbit_:eccentricity >= 1 {
  print "The " + traj + " is not closed, so it has no revolution to walk.".
} else {
  local span is window.
  if span < 0 { set span to 5 * orbit_:period. }

  print "searching the " + traj + " for " + round(span / 60, 1) + " min ("
      + round(span / orbit_:period, 2) + " revolutions).".

  // The search is a few hundred Kepler solves per revolution walked; run
  // it at the processor's ceiling and put the setting back.
  local ipu_prior is config:ipu.
  set config:ipu to 2000.
  // Tolerance zero: no approach counts as good enough, so the walk runs
  // the whole window and returns the closest one rather than the first
  // acceptable one.
  local pass is ground_target_approach(tgt, 0, span, orbit_, t0).
  set config:ipu to ipu_prior.

  if pass["distance"] < 0 {
    print "No approach found.".
  } else {
    local geo is pass["closest"].
    print "closest approach: rev " + pass["rev"] + ", eta "
        + round(pass["eta"]) + " s (" + round(pass["eta"] / 60, 1)
        + " min), miss " + round(pass["distance"] / 1000, 2) + " km.".
    print "ground track there: " + round(geo:lat, 4) + " "
        + round(geo:lng, 4) + ".".
    if pass["distance"] <= tolerance {
      print "Within " + round(tolerance / 1000, 1) + " km tolerance.".
    } else {
      print "Outside the " + round(tolerance / 1000, 1) + " km tolerance."
          + " A target latitude beyond the orbit's inclination ("
          + round(orbit_:inclination, 1) + " deg) can never do better;"
          + " otherwise a longer window might.".
    }
  }
}
