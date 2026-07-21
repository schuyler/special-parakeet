clearscreen.

// === NODE REFINEMENT ===
//
// Refine the next maneuver node to minimize the predicted closest approach
// to the target. Works on any node — a departure from intercept.ks, a
// phasing entry, or a small mid-course correction node added by hand —
// because positionat/velocityat honor planned nodes, so the objective is
// simply "predicted separation with this candidate node in the flight
// plan". Coordinate descent over node time, prograde, and radial, with
// shrinking steps; normal stays zero because match_planes owns the plane.
//
// This can take a minute of game time: every probe of a node parameter
// re-solves for the moment of closest approach.

parameter rounds is 3.

run common.
run orbital.  // separation_at

local nd is 0.
local t_ca is 0.

// The encounter time moves as the node is tuned, but only by seconds per
// probe, so track it between evaluations: one full-window scan up front,
// then each evaluation is a cheap local refinement around the last known
// encounter instead of another scan.
function closest_sep {
  set t_ca to minimize(separation_at@, t_ca - 300, t_ca + 300, 2).
  return separation_at(t_ca).
}

// Ternary-search one node parameter over [x - step, x + step] and leave
// the node set to the best value found. The bracket is small enough that
// the miss distance is unimodal on it.
function tune {
  parameter getter, setter, step.
  local x0 is getter().
  local f is {
    parameter x.
    setter(x).
    return closest_sep().
  }.
  setter(minimize(f, x0 - step, x0 + step, step / 10)).
}

if not hastarget {
  print "No target set.".
} else if not hasnode {
  print "No node to refine.".
} else {
  set nd to nextnode.
  set t_ca to minimize_scan(separation_at@, nd:eta, nd:eta + nd:orbit:period, 1).

  local coords is list(
    list({ return nd:time. },      { parameter x. set nd:time to x. },      60),
    list({ return nd:prograde. },  { parameter x. set nd:prograde to x. },  10),
    list({ return nd:radialout. }, { parameter x. set nd:radialout to x. }, 10)).

  print "=== REFINE ===".
  print "Initial closest approach: " + round(closest_sep()) + " m.".

  local round_i is 1.
  local scale is 1.
  until round_i > rounds {
    for c in coords {
      tune(c[0], c[1], c[2] * scale).
    }
    print "Round " + round_i + ": " + round(closest_sep()) + " m.".
    set scale to scale / 4.
    set round_i to round_i + 1.
  }

  print "Final: " + round(closest_sep()) + " m, " + round(t_ca / 60, 1) + " min from now.".
  print "Next: run next. run rendezvous.".
}
