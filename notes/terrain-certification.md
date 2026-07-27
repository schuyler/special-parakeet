# Terrain certification: deferred design

*Not built. `plan_doi.ks` reads terrain at the landing site alone and hands everything
else to the pilot's eye on the map. This note holds the analysis for putting terrain back
into the software, and names where it would attach. Companion: `doi-planner.md` (the
planner as it stands, including the `h_pdi` solve this would extend).*

## Two objects, two rules

The flown path either side of PDI is two different curves, and each needs its own
argument. PDI is the seam.

**The braking arc is unruled.** The old certificate was a chord: down-range of PDI, `h(x)`
lies above the straight line joining high gate to PDI, provably, for a gravity-turn arc at
any throttle. The flight does not fly a gravity turn — it flies E-guidance under a vector
law — and that argument does not cover this curve. One observation stands in for it, and
is deliberately not machinery: **modelled, the E-guidance path clears the straight
PDI→gate chord over its whole length** — minimum +0.5 m at the PDI end, +900 to +1,600 m
mid-arc — so a single offline march of the planned profile per placement could certify
braking-arc clearance the way the chord did, by sampling the marched path against terrain
instead of asserting a shape. That march is not built. What exists instead is
instrumentation: the flight logs a running minimum radar clearance over the arc
(`clear_min`), so the flights measure the gap rather than pretending it is closed. On
2026-07-27 that minimum was 223 m, three seconds after ignition, with high gate 1500 m
over the site — the tightest clearance of the whole descent was up-range, where nothing
rules.

**The coast must be walked.** Up-range of PDI a ray says nothing useful, and believing it
is dangerous. PDI is periapsis, so the coast leaves it flat (`dh/dx = 0`) and climbs
*quadratically*, while a ray climbs linearly. The ray therefore sits above the coast for
`2·r_pe·(1+e)/e · tan γ` — on Minmus, over a hundred kilometres, most of the up-range
hemisphere. It would happily certify a mountain the coast would fly into.

So the coast gets a flat clearance rule: sample it, and require `h − terrain ≥ clearance`
throughout. Flat is correct here for exactly the reason it was wrong over the arc — the
coast is not trying to land, so clearance under it is a pure hazard question, with no
feedback and no circularity. The walk runs from PDI outward to where the coast climbs past
anything on the body that could reach it; anchoring at PDI and stepping outward lets the
loop find its own end rather than being told where to start. The coast is on rails, so
this is deterministic *before the burn*: walk `nd:orbit`, the patch KSP itself predicts,
and adjust the node before anything is committed.

## The measurement that motivates it

On a 2937 m periapsis over the Great Flats — the flattest ground on Minmus — clearance at
PDI was 2937 m and at 240 s up-range **2947 m**. A ten-metre margin, over the most
forgiving terrain available. Tilt the approach onto any relief and the coast binds before
the arc does.

That is the whole case: the coast, not the braking arc, is the constraint that decides how
low PDI can sit, and it is invisible to a survey that only walks rays.

## Where it attaches

`h_pdi` is already a solve, on a criterion that has nothing to do with terrain: the
altitude whose reference arc stalls to the gate speed exactly at the gate altitude
(`doi-planner.md`). This design is an *extension* of that solve, not a replacement for it.
`h_pdi` becomes the largest altitude any demand returns, and the binding demand is named in
the plan's verdict — terrain binding means the site or the orbit is the problem, capability
binding means the craft is:

1. **The capability demand.** The existing solve. It returns an altitude directly, and it
   is the floor the others bid against; on a low-TWR craft it outbids both terrain demands
   already.
2. **The coast demand.** Walking the placed ellipse from the DOI burn to PDI, the minimum
   of altitude over terrain must be at least `coast_clearance`. At a frozen placement,
   raising periapsis with the burn radius fixed raises the ellipse at every point between
   (at fixed true anomaly, `∂r/∂r_pe > 0` with apoapsis held), so minimum clearance rises
   with `h_pdi` and the demand is a root.
3. **The braking-arc demand.** Unwritten, because nothing rules the arc. If the offline
   march above is ever built, it enters here as an altitude the same way: march the planned
   profile from a candidate PDI, sample terrain under it, and raise `h_pdi` until the
   minimum clearance meets `terrain_margin`. The old chord form —
   `h_pdi ≥ alt_gate + X · max over x of ((terrain(x) + terrain_margin − alt_gate)/x)` —
   is the shape the answer takes, with the marched path in place of the straight line.

The iteration: seed `h_pdi` at the capability solve's answer, the lowest any demand could
return. Each pass places a candidate node, re-runs the solve, sizes the lead X, evaluates
the terrain demands at that placement, and takes the max as the next `h_pdi`.

**Convergence is not proved and should not be claimed.** The arc demand's feedback through
X is `s_max · dX/dh_pdi`; the coast demand's is the pinch's local terrain slope times the
track shift per metre of `h_pdi`. Neither factor is bounded a priori — the one measured
iteration of this loop settled at an effective factor near 0.5 — and the `max()` can switch
binding branches between passes. Damping on reversal and a pass budget are the cover for
all three, and a solve that fails to settle must be reported, not hidden.

Run the in-solve walks coarse (the coast walk is the planner's measured hot spot and each
pass would contain one), then certify the settled geometry at full fidelity; a deficit the
coarse walks missed feeds back for one extra pass, and a deficit that survives that aborts.
The altitude tolerance `h_tol` belongs to the family of `pitch_tol` — an accuracy bound,
not a craft or body number — and is deliberately not tied to `coast_clearance`: it answers
to the solve's noise floor, which does not move when the pilot's caution does.

## What the floor has to cover

A solve driven by terrain drives the binding clearance to its floor *exactly*, where a
dial carries accidental slack. So the floor stops being pure terrain-model distrust and
must also cover the difference between the planned ellipse and the burned one.

Both halves are measured. The placement residual is small — the planner's verdict prints
`pe_err` at a thousandth of a degree — and the *delivery* residual is not: along-track
error of order ±1.5 km is structural, measured across three flights
(`node-delivery-window.md`). That is the number the floor has to cover, and on Minmus one
degree of track error is ~1 km of ground, which on a pinch flank at 10 % grade is ~100 m
of clearance. The verdict should therefore print the pinch's sensitivity — metres of
clearance lost per tenth of a degree of placement error, read off the walked terrain's
local slope — so the exposure is a number rather than an assumption. The stronger answer
is a post-burn re-walk of the *achieved* ellipse: the coast is on rails and it is the same
walk. That belongs to the coast phase, so it needs an owner that is not the flight
controller.

## What this design cannot do

A terrain survey that reads only the body can run before a parking orbit exists, and can
inform site selection with no ship at all. Both certificates above are properties of a
*placed* ellipse, so under this design nothing can be certified until an orbit exists to
place it from. The pre-orbit question — is this site approachable at all — has no owner
here. If it earns one, it is an advisory terrain-envelope walk that certifies nothing.

## The code that exists

`reference/original/optimize_descent_angle.ks` is a working corridor survey: it walks the
approach up-range from the site sampling `geopositionlatlng(lat, lng):terrainheight`,
keeps the steepest ray any obstacle demands, and logs a decimated `(x, terrain, ray)`
profile for plotting. It is the only terrain tooling in the repo — `plan_doi.ks` reads
terrain at the landing site alone.

It is orphaned: it wraps its sweep in a descent angle γ and prints an invocation
`plan_doi.ks` does not accept. To reuse it, the sweep is what survives — re-scoped from a
quarter-body walk to the placement's own X, feeding the arc demand as an altitude — while
γ becomes a derived output, `arctan((h_pdi − alt_gate)/X)`, reported because it is the
honest one-line summary of how steep an approach is, not because anything solves for it.
