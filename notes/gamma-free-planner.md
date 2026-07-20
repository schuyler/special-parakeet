# The gamma-free planner: h_pdi from its binding constraints

*A design note: piece 2 and piece 3 of `capability-driven-descent.md` merged and
re-posed. `plan_doi.ks` takes gamma — a human-judged descent slope — and derives the PDI
altitude from it; `optimize_descent_angle.ks` was built to supply that gamma from a
terrain survey, and its first real answer (4.1 deg on the flight-7 orbit) was refused by
`plan_doi`'s coast walk, clearance −610 m. This note works out why that failure is
structural, what the telemetry says the fuel lever actually is, and re-poses the
planner's solve so gamma is a derived output and the survey script retires. Companion
registers: `powered-descent-invariants.md` (the flight controller this plans for — it is
untouched), `capability-driven-descent.md`, `planner-test-program.md`. The flight
controller's contract in `powered_descent_min.ks` stands as written.*

## The telemetry

Two facts from the flight-7-geometry pair (`doi_plan.log`, gamma 8; `flight_log.csv`,
flown by `powered_descent_min.ks` on the same save), and one from the survey. Surface
gravity is read off the log's own free-fall rows (TERMINAL, engine off): v_vert loses
0.49 m/s per second, Minmus's g0.

1. **The arc's overhead is small and is not a throttle problem.** From the PDI state —
   172.2 m/s at 2278 m — the least delta-v that brings the ship to rest at the site is
   `sqrt(v² + 2 g h) ≈ 178 m/s`. The flight spent 214 (1741 at PDI, 1527 landed). Braking
   harder does not close that gap: killing the 172 m/s at `f_max` takes ~175 m/s and
   leaves the ship slow at ~2.2 km, and arresting the fall from there costs another
   ~46–48 m/s — total ≈ 222, about 8 m/s *more* than the shallow arc flew. At a fixed
   periapsis the throttle trades a longer burn's support losses against a shorter burn's
   fall arrest, the two prices differ by a few percent of the descent, and the shallow
   arc the solve already flies is the cheap side. The term that moves the budget is
   `2 g h`. **The fuel lever is the PDI altitude, not the throttle.**
2. **The binding constraint on the PDI altitude is the coast.** The gamma-8 plan's coast
   walk found its minimum clearance 510 m over terrain 118 s up-range of PDI, against a
   50 m floor: ~460 m of h_pdi is available before the coast binds. The refused 4.1-deg
   plan is the same fact from the other side — the ray cleared its terrain, and the coast
   the ray implied was 610 m underground.
3. **The survey certifies the wrong object under the wrong constraint.** The flight is a
   retrograde-hold gravity turn; its one parameter is the throttle, re-solved from live
   state. No program flies the gamma ray. The ray exists to certify the arc's corridor by
   a chord bound — a sound idea, repaired and kept below — but the survey walks a
   quarter of the body to find the steepest ray demand, most of which lies under the
   coast, whose rule the ray cannot express.

## The chord certificate, repaired

The chord runs from the handoff point (0, h_handoff) up to PDI (X, h_pdi), x measured
along the ground from the site. The claim: every braking arc the flight controller can
fly from this PDI lies on or above it. The old argument — "the arc leaves PDI level and
steepens monotonically, so h(x) is concave" — is false at the start: at PDI the ship is
slightly super-circular (172.2 m/s against a circular 168 at that radius), the turn
rate `v/r − g/v` is positive, and the path pitches *up*; the flown log shows the first
~15 s climbing. The repaired argument runs in two spans:

- **While at or above h_pdi.** From ignition the path rises, peaks, and descends back
  through h_pdi. Throughout, h ≥ h_pdi, and the chord's greatest height is h_pdi at its
  PDI end, so the path clears the chord trivially.
- **Below h_pdi.** By the time the path re-crosses h_pdi it is descending and
  sub-circular, and sub-circularity is preserved for the rest of the descent: v falls
  under braking while the bound `sqrt(g r) = sqrt(mu/r)` rises as r falls, and even a
  zero-throttle segment gains only `2 g Δh` of v² against a bound thirty times larger
  at braking speeds. Sub-circular, the turn rate is negative at every throttle — the
  rate contains no throttle term — so pitch decreases monotonically, h(x) is concave,
  and the path lies above the straight segment joining its h_pdi re-crossing to its
  endpoint. Both of that segment's ends sit on or above the chord (the re-crossing at
  h_pdi; the endpoint per the next paragraph), and the chord is a line, so the segment
  — and the path above it — clears it.

The endpoint condition is the certificate's one non-geometric premise. The planner's
nominal arc ends at the chord's own anchor (h_handoff, by `solve_throttle`'s
construction). The flown arcs end earlier and higher: the flight controller hands off at
the attitude seam with its stopping distance still in hand — flight 7 reached the seam
at 627 m radar where the chord stood at ~140 m — and a seam that arrives *at* the chord
is a marginal handoff the flight already warns about. The certificate therefore covers
the braking family up to a condition the telemetry measures every flight (seam altitude
against chord height at the seam's ground distance — a verdict line worth adding to the
flight's own log analysis), with `terrain_margin` as the buffer behind it. Below the
seam the craft is in terminal's near-vertical cone over the site, whose terrain is the
anchor itself.

Because every quantity above is placement-derived, the chord walk shrinks from a
quarter-body sweep to the arc's own footprint, x in (0, X]. Up-range of PDI belongs to
the coast walk, as it always did.

## The solve, re-posed

Unknowns: h_pdi and the lead. Given h_pdi, everything downstream is already pinned by
`plan_doi.ks`'s existing machinery: the throttle f is the one value whose arc bottoms at
the handoff altitude (`solve_throttle`), the reach X is that arc's down-range
(`integrate_arc`), and the lead is X as an angle at the body's centre. Raising h_pdi
gives the arc more altitude to descend through to the same handoff, so the solved f
falls and the reach X grows — the direction the gamma sweep already observed. What
changes is the source of h_pdi. It is the smallest altitude satisfying three demands:

1. **The coast demand.** Walking the placed ellipse from the DOI burn to PDI, the
   minimum of altitude over terrain must be at least `coast_clearance`. At a frozen
   placement, raising the periapsis with the burn radius fixed raises the ellipse at
   every point between (at fixed true anomaly, `∂r/∂r_pe > 0` with apoapsis held), so
   the minimum clearance rises with h_pdi and the demand is a root. Across placement
   updates the ground track shifts and the terrain under it resamples; that coupling
   belongs to the outer iteration below, not to this root.
2. **The chord demand.** The chord height at ground distance x is
   `h_handoff + (x/X)(h_pdi − h_handoff)`. Requiring it to clear
   `terrain(x) + terrain_margin` for all x in (0, X] gives
   `h_pdi ≥ h_handoff + X · max over x of ((terrain(x) + terrain_margin − h_handoff)/x)`
   — the survey's formula, scoped to the placement's own X and rearranged to demand an
   altitude instead of a slope.
3. **The capability demand.** `solve_throttle` must find f ≤ (1 − f_headroom) · f_max on
   the ellipse h_pdi implies. Lower periapsis means a shorter descent and a higher
   solved throttle; on a low-TWR craft this demand can outbid both terrain demands.
   `f_headroom` is the fraction of the ceiling reserved for the flight controller's
   re-solve to shorten the arc — the existing 10%-of-f_max warning, promoted to a
   constraint. It is a dimensionless judgment multiplier: the share of authority the
   plan may not spend. The promotion has a price the warning did not: on craft where
   this demand binds, it raises h_pdi and spends delta-v to keep the reserve.

h_pdi is the max of the three demands; the binding one is named in the plan's verdict,
the way the survey named its forcing obstacle. Terrain binding means the site or the
orbit is the problem; capability binding means the craft is.

The iteration: seed h_pdi at `h_handoff + coast_clearance` — the lowest altitude any
demand could return — and the lead from the stop-distance reach `v²/(2 f_max a_max)`,
the same approach-from-below seed the old fixed point argued. Each pass places a
candidate node, solves f, marches X, evaluates the three demands at that placement, and
takes their max as the next h_pdi; the damped-on-reversal step and the pass budget carry
over unchanged, and a solve that fails to settle is reported, not hidden. The honest
convergence statement: the chord demand's feedback through X is
`s_max · dX/dh_pdi` (s_max the steepest slope demand), the coast demand's is the
pinch's local terrain slope times the track shift per metre of h_pdi, and neither
factor is bounded a priori — the flight-7 fixed point converged at an effective factor
near 0.5, not by a proved contraction — while the max() can also switch binding
branches between passes. The damping and the budget are the cover for all three, as
they were for the old map's overshoot; what is new is only which map is being damped.

In-solve coast and chord walks run at a coarse step (the existing `search_scale`
treatment: the walk step times the scale), because the coast walk is the planner's
measured hot spot and each pass now contains one. The settled geometry is then
certified at full fidelity — the verdict's coast walk at `coast_dx`, the chord walk at
its own step — and a deficit the coarse walks missed feeds back for one extra pass;
a deficit that survives it aborts.

The iteration's altitude tolerance, h_tol, is 5 m: an accuracy bound in the family of
`coast_dx` and `pitch_tol`, not a craft or body number — above the metre-scale slop the
node's delivered periapsis carries and the reach noise the throttle solve's eps admits,
and below the scale at which any clearance judgment changes meaning. It is deliberately
not tied to `coast_clearance`: the tolerance answers to the solve's noise floor, which
does not move when the pilot's caution does.

**Execution dispersion.** The solve drives the binding clearance to its floor exactly,
where the gamma plans carried accidental slack, so what the floor must cover is now
explicit: terrain-model distrust *plus* the difference between the planned ellipse and
the burned one. The placement residual is measured (pe_lng_err, ~0.04 deg); burn
execution slop is stage 2's to measure; and one degree of track error is ~1 km of
ground on Minmus, which on a pinch flank at 10% grade is ~100 m of clearance. The
verdict therefore reports the pinch's sensitivity — metres of clearance lost per tenth
of a degree of placement error, read off the walked terrain's local slope — so the
exposure is a printed number, not an assumption. A post-burn re-walk of the achieved
ellipse (the coast is on rails; the check is the same walk) is the stronger answer and
is left opened, because it belongs to the coast phase and the flight controller is
frozen by charter.

## What deletes, what survives, what appears

Deleted:
- **`optimize_descent_angle.ks`**, whole. The chord walk moves into `plan_doi.ks`
  (~30 lines, re-scoped to X), and the survey's plot-ready profile log moves with it:
  the verdict logs (x, terrain, chord) triples, the same audit `gamma_survey.log`
  served.
- **gamma as an input**, and `gamma_floor` with it. The floor guarded against a
  near-level plan laying the coast along the ground; `coast_clearance` guards the same
  hazard directly, in the metres terrain distrust is actually judged in, over the whole
  approach including PDI itself — the coast walk's clearance at periapsis *is* the
  pass height over the up-range terrain. On a high-TWR craft over flat ground the new
  solve will therefore plan a fast pass at roughly `terrain + coast_clearance`, every
  certificate satisfied at its floor; a pilot who wants a statelier approach raises
  `coast_clearance`, and the sweep prices what that costs.
- **`plan_from_reach`'s slope term.** The lead half survives; the `X tan(gamma)` half is
  replaced by the constraint solve.
- **The gamma sweep.** Replaced: the priced judgment is now `coast_clearance`, so the
  sweep prices the plan at {1/2, 1, 2, 4} × the floor — the sub-floor point is advisory
  only, the price of a caution the pilot declined to spend. The set brackets the
  judgment one octave down and two up; it is a reporting choice, not a derived one.
- **Gamma-phrased abort advice** ("steepen gamma or move the site", "re-think gamma").
  The coast abort's diagnostic split survives reworded: a dip near PDI indicts the
  demanded clearance or the site; a dip up-range toward the burn indicts the parking
  orbit.

Survives untouched: `integrate_arc`, `solve_throttle`, `arc_dv`, `plan_node`,
`place_node`, the coast walk (now run inside the solve as well as at the verdict), the
plane/cross-track verdict, the terminal-arrest check, the abort discipline, and all of
`powered_descent_min.ks` — its contract ("PDI is the periapsis, the corridor under the
arc is certified") is discharged differently, not changed.

Appears: `f_headroom` (dimensionless, default 0.1, the promoted warning) and
`terrain_margin` (metres the terrain model is distrusted by under the chord, defaulting
to `landing_height` — carried over from the survey with its argument: one benefit of
the doubt, not two). `gamma` is reported in the verdict as a derived quantity —
`arctan((h_pdi − h_handoff)/X)`, the chord's slope, the floor under everything the arc
does above it — because it remains the honest one-line summary of how steep the
approach is.

Lost, and accepted: the survey could run before a parking orbit existed and inform
site selection with no ship at all. Both certificates are now properties of a placed
ellipse, so nothing can be certified until an orbit exists to place it from. The
pre-orbit question — is this site approachable at all — has no owner in this
architecture; if it earns one, it is an advisory terrain-envelope walk that certifies
nothing.

## What this claims, and how a flight falsifies it

Re-planning the flight-7 save (stage-1 bridge run, no burn — the node is the only side
effect) should show, as signatures:

- The verdict names the **coast** as the binding demand, at the lng ≈ −33.6 pinch the
  gamma-8 plan measured; the chord demand stays near h_handoff (terrain under the
  shortened X is Great Flats, near 0 m).
- h_pdi settles **below 2263 m** by roughly the spendable clearance (~400–500 m), with
  the coast walk's minimum clearance equal to `coast_clearance` within the walk's
  sampling slop.
- f_solved rises (shorter descent, same craft), staying far under the capability
  demand on this TWR.
- Priced `dv_doi + dv_arc` comes in **below the gamma-8 plan's 209 m/s**, by single-digit
  metres per second. The prize is small on this body — the telemetry section's point —
  and the claim under test is the constraint structure, not a large saving.
- Determinism: re-running the same plan twice gives identical numbers.

A flown descent then tests the certificates: minimum radar clearance on the coast at or
above the floor net of the printed dispersion sensitivity, the braking track above the
chord throughout with the seam arriving above the chord's height at its ground
distance, and flown total delta-v tracking the priced total as it did for gamma 8
(209 priced, 214 flown including terminal).

f_headroom's falsification signatures: the BRAKE-phase throttle trace saturating at
f_max says 0.1 was too small a reserve; a whole campaign — stage 5's low-TWR flights
included, where the reserve is actually priced — in which the re-solve never enters the
reserved band says it was too large.

On the TWR-2 design craft the capability demand should bind instead — the planner
refusing to spend the coast's full clearance because the throttle ceiling arrives
first — which is the number stage 5 of the test program exists to see.

The test program re-bases with the parameter: stage 1's signatures transfer as above;
stages 2–3 (seam validation, first flight) are unchanged in structure; stage 4's flown
delta-v(gamma) campaign becomes flown delta-v(`coast_clearance`) — one flight per floor
from the same quicksave, judged against the sweep's priced curve, which now exists
before any of them fly.

## Settled and opened

Settles `capability-driven-descent.md`'s piece 3 (the smart planner is not a gamma
solver in front of piece 2; it is piece 2 with h_pdi sourced from the binding
constraints) and its open item 8 as far as the arc goes (the chord walk is scoped to X,
so its sample budget is bounded by the placement, not the body). Open item 1's judgment
(`coast_clearance`) is now the plan's single terrain judgment, and the re-targeted
sweep prices it.

Opens: the post-burn re-walk of the achieved ellipse (the clean answer to execution
dispersion; homeless while the flight controller is frozen); whether the coast-demand
root wants bisection or the one-sided correction (clearance deficit added to h_pdi, an
under-correction that approaches from below); and how coarse the in-solve walks can
run before the one-extra-pass recovery stops being enough.
