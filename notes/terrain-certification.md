# Terrain certification: deferred design

*Not built. `plan_doi.ks` hands terrain clearance to the pilot's eye on the map, and
`pdi_height` is a dial. This note holds the analysis for putting terrain back into the
software, and names where it would attach. Companion: `doi-planner.md` (the planner as it
stands, including the chord certificate this builds on).*

## Two objects, two rules

The flown path either side of PDI is two different curves, and each needs its own
argument. PDI is the seam.

**The braking arc is certified by a chord.** Down-range of PDI, `h(x)` lies above the
straight chord joining the handoff point to PDI — the argument is in `doi-planner.md`, and
it is geometric, needing no sampling of the trajectory. Certifying the chord certifies
every arc the controller can fly.

**The coast must be walked.** Up-range of PDI the same ray says nothing useful, and
believing it is dangerous. PDI is periapsis, so the coast leaves it flat (`dh/dx = 0`) and
climbs *quadratically*, while a ray climbs linearly. The ray therefore sits above the
coast for `2·r_pe·(1+e)/e · tan γ` — on Minmus, over a hundred kilometres, most of the
up-range hemisphere. It would happily certify a mountain the coast would fly into.

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

`pdi_height` stops being a dial and becomes a solve. `h_pdi` is the smallest altitude
satisfying three demands, and the binding one is named in the plan's verdict — terrain
binding means the site or the orbit is the problem, capability binding means the craft is:

1. **The coast demand.** Walking the placed ellipse from the DOI burn to PDI, the minimum
   of altitude over terrain must be at least `coast_clearance`. At a frozen placement,
   raising periapsis with the burn radius fixed raises the ellipse at every point between
   (at fixed true anomaly, `∂r/∂r_pe > 0` with apoapsis held), so minimum clearance rises
   with `h_pdi` and the demand is a root.
2. **The chord demand.** The chord height at ground distance x is
   `h_handoff + (x/X)(h_pdi − h_handoff)`. Requiring it to clear `terrain(x) +
   terrain_margin` for all x in (0, X] gives
   `h_pdi ≥ h_handoff + X · max over x of ((terrain(x) + terrain_margin − h_handoff)/x)`
   — the old survey's formula, scoped to the placement's own X and rearranged to demand an
   altitude instead of a slope.
3. **The capability demand.** The throttle solve must find `f ≤ (1 − f_headroom)·f_max` on
   the ellipse `h_pdi` implies. Lower periapsis means a shorter descent and a higher solved
   throttle, so on a low-TWR craft this demand outbids both terrain demands. This one is
   already in the code as `f_headroom`; the other two are not.

The iteration: seed `h_pdi` at `h_handoff + coast_clearance`, the lowest any demand could
return. Each pass places a candidate node, solves the throttle, marches the reach X,
evaluates the three demands at that placement, and takes their max as the next `h_pdi`.

**Convergence is not proved and should not be claimed.** The chord demand's feedback
through X is `s_max · dX/dh_pdi`; the coast demand's is the pinch's local terrain slope
times the track shift per metre of `h_pdi`. Neither factor is bounded a priori — the
flight-7 fixed point converged at an effective factor near 0.5, empirically — and the
`max()` can switch binding branches between passes. Damping on reversal and a pass budget
are the cover for all three, and a solve that fails to settle must be reported, not hidden.

Run the in-solve walks coarse (the coast walk is the planner's measured hot spot and each
pass would contain one), then certify the settled geometry at full fidelity; a deficit the
coarse walks missed feeds back for one extra pass, and a deficit that survives that aborts.
The altitude tolerance `h_tol` belongs to the family of `pitch_tol` — an accuracy bound,
not a craft or body number — and is deliberately not tied to `coast_clearance`: it answers
to the solve's noise floor, which does not move when the pilot's caution does.

## What the floor has to cover

A solve drives the binding clearance to its floor *exactly*, where a dial carries
accidental slack. So the floor stops being pure terrain-model distrust and must also cover
the difference between the planned ellipse and the burned one.

The placement residual is measurable (`pe_lng_err`, ~0.04°); burn execution slop is not yet
measured. One degree of track error is ~1 km of ground on Minmus, which on a pinch flank at
10% grade is ~100 m of clearance. The verdict should therefore print the pinch's
sensitivity — metres of clearance lost per tenth of a degree of placement error, read off
the walked terrain's local slope — so the exposure is a number rather than an assumption.
The stronger answer is a post-burn re-walk of the *achieved* ellipse: the coast is on rails
and it is the same walk. That belongs to the coast phase, so it needs an owner that is not
the flight controller.

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

It is currently orphaned: it wraps its sweep in a descent angle γ and prints an invocation
the planner no longer accepts. To reuse it, the sweep is what survives — re-scoped from a
quarter-body walk to the placement's own X, feeding the chord demand as an altitude — while
γ becomes what it already is in `plan_doi.ks`: a derived output,
`arctan((h_pdi − h_handoff)/X)`, reported because it is the honest one-line summary of how
steep an approach is, not because anything solves for it.
