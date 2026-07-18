# Powered descent by live re-solve: the invariants

*A design note: piece 1 revisited. `powered_descent.ks` flies the braking burn against a
table integrated once at PDI; this note works out what the descent actually promises at
every instant, and concludes the table is a cache whose staleness costs more machinery than
the cache saves. It motivates `powered_descent_live.ks`, the rendition that re-integrates
the arc from live state. Companion registers: `capability-driven-descent.md` (the
architecture; this note settles its open items 3 and 4), `apollo-powered-descent.md`,
`klumpp-guidance-derivation.md`. All stand as written.*

## The premise

Hold thrust retrograde and the trajectory is a **one-parameter family**: the current state
(altitude, speed, flight-path angle, mass) plus a throttle `f` fully determines the arc,
because the gravity turn flies itself. So at any moment during braking, where the arc at
the current throttle bottoms out — altitude, down-range, time — is *computable from the
ship*, by the same Euler march that built the table. No stored plan is needed to know it.
There is no closed form (the gravity-turn ODE has none); "we know the trajectory" means we
know it by integration. The table versus live re-solve is therefore purely a caching
decision: the same integrator, run once at PDI or run every few seconds. The physics
content is identical. What differs is what the loops compare against — a frozen plan, or a
fresh prediction — and where the compute goes.

## The invariant set

The braking phase is these five statements and nothing more:

1. **Retrograde hold.** Thrust lies along −v_surface, ± a bounded yaw. This is the
   enabling invariant: it is what makes the trajectory a one-parameter family.
2. **Targeting.** `f_cmd` is the value whose arc ends over the site. Well-posed because
   the endpoint's down-range falls strictly monotonically as `f` rises — more throttle,
   shorter arc — so the solution is unique when it exists.
3. **Feasibility.** That `f` satisfies `f ≤ f_max`, and its arc bottoms out at or above
   the handoff altitude. Violated on the high side: the site is unreachable within
   authority — abort while altitude remains. Violated on the low side: reaching the site
   would plan the arc below the gate — accept the short landing, never plan into the
   ground.
4. **Cross-track.** The site lies in the plane of the surface velocity, nulled by yaw
   while the ship is fast, where a degree costs least.
5. **Handoff continuity.** The arc ends at `speed_handoff`, which is terminal's descent-
   rate cap: the phases meet at the same state.

Each phase of the whole descent is *named by* an invariant like these: the coast is
on-rails (the ellipse is the state — quicksave-able, abortable), braking is the family
above, terminal is a rate servo. The seams — the node, PDI, the handoff state — are where
the invariant changes. That is the phase structure; the throttle states of the older
scripts (see below) were never this.

## What the table is, and what deletes with it

A row of the table is a prediction made at PDI and trusted to the end of the burn. Every
piece of in-flight machinery in `powered_descent.ks` beyond the invariant set exists to
manage that trust:

- **The table and `table_at`** — replaced by re-integrating from live state.
- **`x_shrink_per_f`**, the finite-difference probe arc — a linearized gain for trimming
  toward a frozen reference. A re-solve has no gain; it solves the remaining problem
  whole, each look, seeded by the previous answer.
- **The overshoot allowance, `model_error_margin`, the half-step coarse arc, the linear
  taper** — a protected buffer sized to the one-shot prediction's self-measured error.
  Under re-solve the prediction error shrinks with the horizon: each look marches a
  shorter arc from a fresher state, and the looks that decide the landing are the
  shortest and truest. Nothing is left for the buffer to protect.
- **The one-sided ratchet.** The asymmetry behind it is physical and survives — but as an
  inequality, not a mechanism. Raising `f` is always safe (the arc rises and shortens);
  lowering `f` stretches the arc downward, and the floor is the `f` whose arc bottoms
  exactly at the gate. `f_cmd = max(f_site, f_gate)`, capped at `f_max`. The plan no
  longer needs to arrive deliberately long, because the solve corrects both signs of
  error — the gate floor is what keeps the down-range correction from ever spending
  altitude it doesn't have.
- **The "below the planned arc" emergency check** — its reference is gone, and its
  replacement is stronger: divergence now reads as *the required throttle climbing toward
  `f_max`*, remaining capability measured in the control variable's own units. The abort
  condition becomes "no throttle in range keeps the arc above the gate" — which is the
  same test the pre-flight feasibility check runs. One invariant, checked continuously
  from the parking orbit to the handoff, instead of two guards phrased differently.
- **PDI as a special state.** The integrator seeded from live state doesn't care that PDI
  is a periapsis; ignition slop, DOI placement error, and mid-burn dispersion are all
  just "the current state," absorbed identically. PDI remains special to the *planner* —
  it is where the Δv lives — but not to the flight controller.

What survives: the integrator, a scalar solve around it, the retrograde hold, the yaw law
(t_go read off the march instead of off the table), terminal descent, the recorder, the
aborts. The program becomes: *hold retrograde; every few seconds, solve for the one
throttle whose arc ends at the site; check it's feasible; yaw the plane onto the site.*

## Two conditions, one knob

The endpoint must satisfy two conditions — over the site, at the gate altitude — and there
is one knob. Both endpoint coordinates are monotone in `f`: down-range falls as `f` rises,
bottom altitude climbs. So the two conditions each pin their own throttle, `f_site` and
`f_gate`, and the command is the feasibility ordering:

1. If even the `f_max` arc — the highest, shortest arc the craft has — bottoms below the
   gate, nothing closes: **abort** (re-plan before the coast; emergency-land during the
   burn).
2. If even that arc ends at or past the site, every throttle books an overshoot: fly
   `f_max`, eat the smallest one, and say so.
3. Otherwise solve `f_site`. If no throttle reaches the site — every arc lands short —
   aim the bottom at the gate (`f_gate`) and accept the short landing.
4. If `f_site`'s arc dips below the gate, pull up to `f_gate`: **the gate outranks the
   site.** A wrong-place landing beats a right-place crater, the same trade
   `emergency_land` makes, applied continuously.

The planner's placement determines which case runs: a well-placed PDI keeps every look in
case 3's happy path, with `f_site` a little above the pre-flight solution and the endpoint
a little above the gate. The other cases are what the design does about a planner that
missed.

## The lesson in the older scripts

The old landing family (`landing.ks`, `land_at_periapsis.ks`, `deorbit.ks`,
`deorbit_simple.ks`, `drop_periapsis.ks`, `predict_landing.ks`) had pieces of everything
above and the use of none of it, and the reason is one inversion:

- **Old doctrine:** the *trajectory* intersects the site and the *burn* is timed to
  arrest it. Every planner in the family buried periapsis at or below the surface
  (−20,000 m, −5,000 m, `r = body:radius`). Consequences: the coast is a fall, timing is
  safety-critical, margins are bolted onto clocks (`burn_margin 1.01`, `+2 s`), vertical
  and horizontal velocity die in separate bang-bang burns, and there is no stable state
  to abort into or quicksave from.
- **Apollo doctrine (current):** the trajectory *misses the surface* — periapsis low,
  safe, up-range — and the *burn* is what brings the ship down. PDI is a chosen state,
  not a rescue. The coast is on rails, ignition slop is absorbed by feedback, an abort
  hands back an orbit, and periapsis becomes the seam between planning and flight.

Without that inversion the phases could not come apart: if the trajectory must hit the
site, "coast" and "fall" are the same thing and "deorbit" and "descent" are the same
burn. With it, each phase gets its invariant and the boundaries fall out on their own.

The near-misses are instructive. `drop_periapsis.ks` *was* a DOI planner — positive
periapsis over a chosen longitude — with no landing script able to consume it, because
they all wanted impact trajectories. `estimate_downrange_target` in `land_at_periapsis.ks`
is the lead angle in embryo, computed as a fudge on an impact point instead of as a
designed arc. And `predict_landing.ks`'s live "where do I end up if I burn from here,"
IPU-cached and recomputed on the fly, is exactly the re-solve structure — with a 1-D
speed-and-fall model predicting from a state too impoverished to close down-range or
cross-track, and no loop closed on the prediction. The evolution is a spiral: right
feedback structure with the wrong model (old scripts) → right model with frozen feedback
(`powered_descent.ks`) → right model, live feedback (this design). The old code was not
wrong to re-predict from current state; it was wrong about what the state *was*, because
without the phase distinctions "current state" could not include "I am on a designed
ellipse whose periapsis is my ignition point."

In guidance terms: the old scripts and this design are *explicit* guidance (compute the
trajectory onboard from current state, every cycle); the table is *implicit* guidance
(precompute, store, track a reference). `powered_landing.ks` was explicit with the
constant-jerk model and needed gates to keep the model honest; this is explicit with the
gravity-turn model, which needs no gates because the model is the trajectory actually
flown.

## Costs, priced

- **IPU.** A look is roughly a dozen marches (one `f_max` probe, a bisection, one
  endpoint confirmation). At 150 steps and `config:ipu 2000` that is a second or two of
  game time — acceptable at a 5 s look cadence because the locks keep flying the ship
  while the mainline marches, and the step budget scales down with the remaining speed
  span. The frozen table's in-flight loop was nearly free; this buys accuracy with
  compute we have. If the budget ever binds, the bisection becomes a secant seeded by the
  previous look (2–3 marches) before anything else changes.
- **Feedback through prediction.** The ratchet had an idempotence argument: repeating the
  computation against an unchanged measurement changes nothing. A re-solver's command
  changes the state its next prediction seeds from. Deadbeat receding-horizon control of
  a monotone one-knob system is about as benign as this gets, and the cadence is long
  against the throttle's effect appearing in the state — but per the process rule, that
  is a claim about flight behavior and only telemetry can settle it.
- **Gain inversion near the handoff.** As the arc shortens, metres of endpoint per unit
  of throttle collapse, so the solve would demand ever-larger `f` swings to correct
  ever-smaller misses. The design freezes `f_cmd` when predicted t_go falls below a
  threshold (~10 s) and lets the last solution ride; the residual miss is metres, and
  terminal's drift cascade owns the last few metres by charter.

## Predictions (testable signatures, not outcomes)

- `throttle` column, BRAKE phase: a staircase of small steps drifting smoothly — solver
  convergence. Oscillation between looks is the feedback-through-prediction failure mode
  and kills the design as specified.
- `aim_dist` at handoff: tens of metres or better without any allowance machinery, on the
  first flight, because the last looks solve short arcs from true state.
- Δv against the 244 m/s Minmus baseline: no worse than the table rendition on the same
  ellipse — the flown arc is the same arc; only the error management changed. Any
  improvement comes from retiring the deliberate overshoot bias, and should be small.

## Settled and opened

Settles `capability-driven-descent.md` open item 3 (`f` is an *output* of the flight
controller, continuously; `f_max` survives as the authority reserve, now a bound on the
solve) and item 4 (re-solve, not decrement; no arrival-acceleration scalar survives).

Opens: `solve_period` and `t_go_freeze` values (chosen, not derived); whether bisection
needs replacing with a seeded secant on IPU grounds; the radar backstop threshold
(`alt:radar < landing_height` while fast) leans on the planner's γ-certification and has
not flown.
