clearscreen.

run common.
run orbital.

// === PLANE MATCHING ===
//
// Generalizes set_inclination.ks: instead of matching a given equatorial
// inclination, match the plane of the current target. Burn at the cheaper
// of the two relative node crossings (the one where we're moving slowest).

// orbit_normal and relative_inclination come from orbital.ks.

// Signed distance from the target's orbital plane, dt seconds from now.
// Its zero crossings are the relative ascending and descending nodes.
function plane_distance_at {
  parameter dt.
  local n_tgt is orbit_normal(target).
  return vdot(n_tgt, positionat(ship, time:seconds + dt) - body:position).
}

// Scan one orbit for sign changes in the plane distance, then refine each
// bracket by bisection. Returns a list of two times-from-now: the relative
// AN and DN, in whichever order we reach them.
function plane_crossings {
  local crossings is list().
  local samples is 24.
  local step is ship:orbit:period / samples.
  local f0 is plane_distance_at(0).
  local i is 1.
  until i > samples {
    local f1 is plane_distance_at(i * step).
    if (f0 < 0) <> (f1 < 0) {
      crossings:add(find_root(plane_distance_at@, (i - 1) * step, i * step, 0.1)).
    }
    set f0 to f1.
    set i to i + 1.
  }
  return crossings.
}

// Plane changes cost 2 * v * sin(di / 2), so burn wherever v is lowest.
function cheapest_crossing {
  local best is -1.
  local best_speed is 0.
  for dt in plane_crossings() {
    local speed is velocityat(ship, time:seconds + dt):orbit:mag.
    if best < 0 or speed < best_speed {
      set best to dt.
      set best_speed to speed.
    }
  }
  return best.
}

function match_planes {
  local dt is cheapest_crossing().
  local t is time:seconds + dt.
  local n_tgt is orbit_normal(target).
  local d_inc is relative_inclination(t).

  local v_ is velocityat(ship, t):orbit.
  local r_ is positionat(ship, t) - body:position.

  // Rotating our velocity about the radial direction by the relative
  // inclination tips our plane onto the target's. This beats a pure
  // normal burn of 2*v*sin(di/2): that's the chord, not the arc, and
  // for large di the correct burn picks up a retrograde component that
  // the rotation gives us for free. Rather than deriving which way is
  // "ascending" in a left-handed frame, try both directions and keep
  // the one that actually aligns the normals.
  local v_new is angleaxis(d_inc, r_) * v_.
  local v_alt is angleaxis(-d_inc, r_) * v_.
  if vang(vcrs(v_alt, r_), n_tgt) < vang(vcrs(v_new, r_), n_tgt) {
    set v_new to v_alt.
  }

  local nd is node_from_velocity(v_new - v_, t).
  add nd.
  return nd.
}

print "=== MATCH PLANES ===".

if not hastarget {
  print "No target set.".
} else {
  local d_inc is relative_inclination().
  print "Relative inclination: " + round(d_inc, 3) + " deg.".
  if d_inc < plan_inc_matched {
    print "Planes already matched.".
  } else {
    // Clear a pending node only now that we are actually planning one;
    // the matched branch above must stay a true no-op, not eat a node
    // some other planner (or a hand) left in the flight plan.
    if hasnode {
      remove nextnode.
    }
    local nd is match_planes().
    print "Node in " + round(nd:eta) + "s, dv " + round(nd:deltav:mag, 1) + " m/s.".
    // positionat/velocityat honor planned nodes, so this predicts the
    // post-burn plane before we commit any fuel to it.
    local check is vang(orbit_normal(ship, nd:time + 60), orbit_normal(target)).
    print "Predicted after burn: " + round(check, 3) + " deg.".
  }
}
