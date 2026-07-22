@lazyglobal off.

clearscreen.
print "=== RETURN FROM MOON ===".

// Plan the burn that drops us out of a moon's SOI onto a return trajectory
// whose planet-relative periapsis matches a target altitude — a reentry or
// aerobrake corridor, usually. One knob: target_pe, the desired periapsis
// above the parent planet's sea level, in metres.
//
// The plan is a single prograde burn (moon frame), placed at the one point
// on the current orbit where it does the most good, then a bisection on its
// magnitude. Two ideas, living in two different frames:
//
//   WHERE to burn — the moon carries us around the planet at V_moon. Our
//   planet-relative velocity is V_moon + v_rel, v_rel being our velocity
//   around the moon. To sink the planet periapsis we want to shed planet-
//   relative speed, so we burn where v_rel points *against* V_moon: the
//   point on the orbit where we are moving retrograde to the moon's own
//   travel. A moon-frame prograde burn there grows v_rel in the anti-V_moon
//   direction and |V_moon + v_rel| shrinks. We find that point analytically
//   from the library's Keplerian state (orbit_at) — it is just the time
//   that minimises v_rel · V_moon — without touching the game's predictor.
//
//   HOW HARD to burn — the periapsis of the *next* patched conic is monotone
//   in the burn: more prograde dv, lower planet periapsis, until v_rel
//   overtakes V_moon. That objective is not something this library can model
//   — orbit_at lives inside one SOI and knows nothing of the crossing — so
//   we let KSP do it. Park a real maneuver node, set its prograde component,
//   and read nd:orbit:nextpatch:periapsis straight off the game's patched-
//   conic solver. Bisect dv until that periapsis is the target.
//
// The retrograde point is the near-optimal ejection for a low, near-circular
// moon orbit. It sets periapsis *altitude*, not the longitude or timing of
// reentry; if you need a specific corridor, refine.ks can tune the node
// against a live prediction afterwards.
//
// The node is the whole output. Then: run next. (to fly it).

run "../core/kepler".   // orbit_at (state vectors); pulls in optimize.ks: bisect
run "common".           // minimize_scan

parameter target_pe is 40000.   // desired periapsis above the planet, metres

function plan_return {
  parameter target_pe.

  // -- Preconditions ---------------------------------------------------

  if not ship:body:hasbody {
    print "Not orbiting a moon: " + ship:body:name
      + " has no parent body to return to. Nothing planned.".
    return.
  }

  local moon is ship:body.
  local planet is moon:body.

  if ship:orbit:hasnextpatch {
    print "WARNING: the current orbit already leaves " + moon:name
      + "'s SOI. This planner assumes a bound orbit; results may be off.".
  }

  print "Returning to " + planet:name + ", target periapsis "
    + round(target_pe / 1000, 1) + " km.".

  // The moon's velocity around the planet — our reference direction.
  // Sampled once: over one low-moon orbit it barely turns.
  local v_moon is moon:velocity:orbit.

  // -- WHERE: the retrograde-to-the-moon's-orbit point -----------------

  local t0 is time.
  local period is ship:orbit:period.

  // v_rel . V_moon over one orbit; its minimum (toward -1) is the point
  // where we move most nearly opposite the moon's travel.
  local alignment is {
    parameter dt.
    local v_rel is orbit_at(t0 + dt, ship:orbit):velocity.
    return vdot(v_rel:normalized, v_moon:normalized).
  }.

  local dt_burn is minimize_scan(alignment, 0, period, 1).
  local t_burn is t0 + dt_burn.
  local align is alignment(dt_burn).

  print "Burn point: +" + round(dt_burn) + "s, alignment "
    + round(align, 3) + " (-1 = perfectly retrograde).".
  if align > -0.9 {
    print "NOTE: best alignment is only " + round(align, 3)
      + "; the orbit is inclined to the moon's, so this ejection is off-optimal.".
  }

  // -- HOW HARD: bisect prograde dv on the next patch's periapsis -------

  // Clear existing nodes so nd:orbit:nextpatch reflects only our burn.
  until not hasnode { remove nextnode. }

  local nd is node(t_burn:seconds, 0, 0, 0).
  add nd.

  // The game's patched-conic periapsis for a given prograde dv. wait 0 lets
  // KSP resolve the new patch before we read it.
  local patch_pe is {
    parameter dv.
    set nd:prograde to dv.
    wait 0.
    if nd:orbit:hasnextpatch {
      return nd:orbit:nextpatch:periapsis.
    }
    return 1e12.   // still bound to the moon: treat as unreachably high
  }.

  local objective is {
    parameter dv.
    return patch_pe(dv) - target_pe.
  }.

  // Bracket. dv_escape puts moon-apoapsis at the SOI edge; a hair beyond it
  // a real next patch appears with periapsis near the moon's own orbital
  // radius (objective > 0). Growing dv sinks that periapsis, and shedding
  // |V_moon| more than zeroes the planet-relative speed, so the objective
  // turns negative well before then.
  local burn_state is orbit_at(t_burn, ship:orbit).
  local r_b is burn_state:position:mag.
  local v_b is burn_state:velocity:mag.
  local a_esc is (r_b + moon:soiradius) / 2.
  local v_edge is sqrt(moon:mu * (2 / r_b - 1 / a_esc)).
  local dv_escape is max(0, v_edge - v_b).

  local dv_lo is dv_escape + 2.
  local dv_hi is dv_escape + v_moon:mag.

  // Widen the top until the periapsis actually drops below target.
  local tries is 0.
  until objective(dv_hi) < 0 or tries >= 6 {
    set dv_hi to dv_hi + v_moon:mag.
    set tries to tries + 1.
  }

  if objective(dv_lo) < 0 {
    print "Even a bare escape already undershoots "
      + round(target_pe / 1000, 1)
      + " km. Target too low, or orbit already escaping.".
  } else if objective(dv_hi) > 0 {
    print "Could not sink periapsis to " + round(target_pe / 1000, 1)
      + " km. Check the game's conic patch limit and that an escape "
      + "actually reaches " + planet:name + ". Nothing usable planned.".
  } else {
    local dv_solution is bisect(objective, dv_lo, dv_hi, 0.5).

    set nd:prograde to dv_solution.
    wait 0.
    local final_pe is nd:orbit:nextpatch:periapsis.
    local patch_body is nd:orbit:nextpatch:body.

    print "----".
    print "Prograde dv: " + round(dv_solution, 1) + " m/s.".
    print "Escape burn in " + round(t_burn:seconds - time:seconds) + "s.".
    print "Next patch: " + patch_body:name + ", periapsis "
      + round(final_pe / 1000, 1) + " km (target "
      + round(target_pe / 1000, 1) + " km).".
    if patch_body <> planet {
      print "WARNING: next patch is " + patch_body:name + ", not "
        + planet:name + " — check for an intervening encounter.".
    }
    print "Next: run next.".
  }
}

plan_return(target_pe).
