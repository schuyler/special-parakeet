# Planner test program

*The test campaign for `plan_doi.ks` (piece 2), to begin once `powered_descent.ks` has
been validated in flight. Companion to `capability-driven-descent.md`, whose process
rules apply throughout: one instrumented change per flight, predictions stated as
testable signatures before the test, and `doi_plan.log` + `flight_log.csv` as the paired
witness — every flight judged against its own plan.*

## Stage 1 — bridge runs, no burn

From the parking-orbit quicksave, `run plan_doi(gamma).` costs nothing: the node is the
only side effect and `remove nextnode` undoes it.

- **First run:** compiles; wall-clock of the solve (target ≤ ~10 s — this is where the
  `config:ipu` claims get checked); `doi_plan.log` written; node placed. Signatures:
  the fixed point settles inside its 12-pass budget and placement in ≤ 3 passes;
  placement error < 0.2°; the node's delivered periapsis within metres of `h_pdi`; the
  log's own numbers satisfy `h_pdi = terrain + landing_height + X·tan γ` to within
  `x_tol·tan γ` (the placement loop stops when the endpoint's move is inside the
  corridor slack, and reports the pair it certified as interchangeable, not a
  re-derived one).
- **Repeat the same γ:** identical numbers. Everything is on rails — nondeterminism
  means a bug.
- **γ sweep** (say 1°/2°/4°/8°), removing the node between runs: the first
  Δv-against-γ curve ever produced, priced rather than flown. Signature: `h_pdi`
  monotone in γ. The shape of `dv_total(γ)` is the payoff — record it, no prediction.
- **Abort-path probes:** absurd γ, a pre-existing node, engine shut down. Each must
  leave no node and restore IPU.
- **Watch item:** the fixed point seeds `X` at the stop distance at the throttle
  ceiling, `v²/(2·f_max·a_max)` — a lower bound on any arc's downrange — precisely so
  the first solve never prices a degenerate zero-descent ellipse whose bisection
  bracket can fail on a high-thrust craft. A bracket abort on the first coarse pass
  would mean that argument failed in practice; it is the regression to watch for.

## Stage 2 — seam validation: one burn, no landing required

Plan at a working γ, `execute_node`, run `powered_descent`, and read its pre-coast
printout against the plan. Four signatures:

1. `f_solved(flight) − f_solved(plan) ≈ −f·dv_doi/(Isp·g0)` — the mass-loss offset,
   small and negative: the planner prices the arc at pre-DOI mass, the flight
   controller re-solves at post-DOI mass. Larger disagreement means the duplicated
   steppers have diverged, which is the seam's one standing hazard.
2. `overshoot_pdi(flight) ≈ allowance(plan)` within the 0.2° placement slop (~200 m on
   Minmus) — the "arrive one allowance long" contract term, measured for the first
   time.
3. `cross_pdi(flight)` vs the planner's prediction — calibrates the footprint-based
   cross-track estimate.
4. Measured periapsis vs planned `h_pdi` — the node-execution slop that the fixed
   point's 1 m convergence criterion leans on.

## Stage 3 — first planned flight to touchdown

Same γ, no changes. Reads: miss; `dv_at_pdi − dv_rem` at landing against the plan's
`dv_arc`; total against the plan's total; the `throttle` and `cross` traces. First
head-to-head against the 244 m/s baseline.

## Stage 4 — the γ campaign

One γ per flight from the same quicksave, nothing else varied. Each flight adds a point
to flown-Δv(γ) and a flown-vs-priced residual. Output: the knee of the curve, and how
close the best γ gets to the ~180–200 m/s impulsive floor. This is the number the whole
redesign exists to produce.

## Stage 5 — the TWR-2 craft

Repeat stages 1–3 at the design point, where failures can't hide behind surplus
thrust: the `f_solved` gap to `f_max`, the headroom warning, arc duration, and
allowance scale all change character there. The stage-1 seed watch item flips regime
here too — low TWR makes the seed `X` large, testing the lower-bound argument from
the other side.

## Budget

Stages 2–3 are one or two flights, stage 4 about four, stage 5 two or three — roughly
eight flights to a defensible Δv(γ) curve on both craft.
