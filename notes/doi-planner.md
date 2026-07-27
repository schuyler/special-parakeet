# The DOI planner

*The design register for `reference/original/plan_doi.ks`: one node that drops a parking
orbit onto a descent ellipse whose periapsis is PDI, over the landing site.
Companions: `powered-descent-invariants.md` (the flight program this plans for),
`klumpp-descent-redesign.md` (the design and its full argument),
`terrain-certification.md` (the terrain analysis this planner defers, and where it would
attach), `node-delivery-window.md` (what the burn does with the node).*

## Why the placement is a loop

The DOI burn is tangential, and on a *circular* parking orbit that is the whole story:
the burn point is an apsis, periapsis lands exactly 180 degrees ahead, and vis-viva
sized at the semimajor axis delivers the altitude asked for. A parking orbit at
e = 0.012 is not circular enough for either claim. Measured over the Mun, with the burn
falling at true anomaly 109 degrees:

- **Periapsis lands 195.3 degrees ahead of the burn, not 180.** Away from an apsis the
  burn leaves radial speed in play, so the new eccentricity vector does not point back
  along the burn radius. That is 15 degrees, about 53 km — larger than the whole
  38.7 km braking arc, so it cannot be flown out.
- **The altitude comes in 1064 m low**, because the radius at the burn is 822 m above
  the semimajor axis the sizing used. Sizing at the true burn radius shrinks that to
  ≤ 422 m, and never upward — always toward the terrain.

So the planner searches. Each pass places the node, reads back what the game says that
node does, and corrects the two numbers it asked for: the burn slides by the longitude
miss at the ground track's synodic rate, and the radius the vis-viva sizing aims at
absorbs the periapsis miss. Nothing models the two offsets; the loop measures them.

A pass removes about nine tenths of its miss. It is not more because sliding the burn
also slides it to a different true anomaly, changing how far past the antipode
periapsis falls — roughly a tenth of a degree per degree, near this burn point. So the
miss decays about a decimal digit a pass (15.7 degrees, 1.9, 0.18, 0.015) and flattens
near a thousandth of a degree from the sixth. `passes` is 8 — the extra two are free
against everything else the planner does, and they keep the flattening measured rather
than assumed.

The alternative was solving the non-apsidal Δv exactly, which is available in closed
form: for a tangential impulse, `e_new = A·e_old + (A−1)·r̂` with `A = (1 − Δv/v)²`. It
agrees with KSP's own patch to 1 m and 0.01 degrees. It was not taken because inverting
it for a target periapsis still needs a numerical solve, it closes only the altitude
half of the loop, and the altitude half already converges in one pass. It is the version
to build if the planner ever has to run without a live node — for a vessel that is not
the active one, or with no game state to touch at all.

The placement refuses rather than hand over something unflyable. A maneuver node already
pending is not the planner's to reason about, or to delete. No live engine means no arc to
size. A burn longitude with no crossing ahead is a search that never started. A plan whose
corrections walked the burn into the past has nothing to fly. And a node closer than half
its own burn plus a minute of orientation time will burn late, which silently moves
periapsis east — aborting on it is self-correcting, since by the re-run that crossing has
passed and the next is most of an orbit out. The minute's grounding is the flown slew:
`facing_err` closed 20.1, 10.7, 1.7 degrees over the first three BRAKE rows of
`flight_log_20260727_klumpp.csv`, one second apart — the 20.1 already a one-second
residual, so the full-slew rate is unwitnessed — and a minute covers a full reversal
about three times at that closing rate.

## The fuel lever is the PDI altitude, not the throttle

This is the finding that decides where effort goes, and it is telemetry, not theory. From
a measured PDI state — 172.2 m/s at 2278 m over Minmus, `g0` 0.49 m/s² read off the log's
own engine-off free-fall rows — the least delta-v that brings the ship to rest at the site
is `sqrt(v² + 2·g·h) ≈ 178 m/s`. The flight spent 214.

Braking harder does not close that gap. Killing the 172 m/s at `f_max` takes ~175 m/s and
leaves the ship slow at ~2.2 km; arresting the fall from there costs another ~46–48, for
~222 total — about 8 m/s *more* than the shallow arc actually flew. At a fixed periapsis
the throttle only trades a longer burn's support losses against a shorter burn's fall
arrest, and those prices differ by a few percent. The term that moves the budget is
`2·g·h`.

So the planner owns the fuel, and it owns it through one number: how high PDI sits.

## The PDI altitude is a solve

The instrument is the **reference arc**: the constant-throttle gravity turn at `f_cap`,
marched forward by Euler steps from the periapsis a candidate `h_pdi` implies. Pitch zero,
because a periapsis is horizontal; speed from vis-viva with apoapsis at the parking
orbit's semi-major-axis radius, less the ground's eastward motion, because the arc is
flown against the ground; mass the ship's own, because the coast burns nothing. The march
ends where speed falls to the gate speed — the arc's stall, past which the turn equations
verticalize on the spot — or at the gate altitude, whichever comes first. It returns the
ground it covered and the mass left at the end, so the gate mass is read off the march
rather than iterated for its own sake.

`h_pdi` is then the altitude whose reference arc stalls to the gate speed *exactly at the
gate altitude*, bisected on `stall altitude − gate altitude`. That difference changes sign
once: an arc from too low reaches the gate altitude still fast, one from too high stalls
above it.

Four things about the solve are derived rather than chosen, and that is the point of it:

- **The bracket** runs from `alt_gate + 1` — an arc from just over the gate falls into the
  gate still fast — to `ship:orbit:periapsis`, the ellipse family's own ceiling, since
  above it there is no descent ellipse to place. Both signs are checked before bisecting,
  so a bracket failure aborts in the planner's own words instead of inside a root-finder.
- **The tolerance scales itself.** The solve stops when the bracket is narrower than
  `v_frac` times the midpoint's height above the gate. The march carries roughly `v_frac`
  of relative error, so a tighter root would be precision the function does not have —
  metres when the root sits close over the gate, hundreds of metres when it sits high. The
  same `v_frac` also sets the lead bisect's tolerance and the dip solve's `t_go`
  tolerance. No absolute tolerance appears among the solver tolerances — the bracket's
  own floor, `alt_gate + 1` above, is the one absolute metre among them.
- **The step cap is a scale with measured headroom.** Each Euler step retires at most one
  quantum of its binding constraint, so `ln(v0/v_gate)/v_frac + 90/pitch_tol` counts the
  steps a march would take if every step retired a full quantum, and the cap is four
  times that. The headroom is measured offline, not flown: a re-march of the 2026-07-27
  plan takes 110 steps against a 184-step budget, and a sweep across TWR at `f_max`
  0.98–6.2 and `h_pdi` 3.5–19.3 km runs 7–112 against budgets near 184, low-TWR arcs
  exiting early on the gate altitude rather than crawling. The verdict's `steps n of cap
  m` line witnesses each live plan. Hitting the cap aborts as a bug witness, not a
  placement problem.
- **The gate mass converges in two passes.** The gate speed depends on the gate mass
  through `a_dec`, and the gate mass comes off the march, so the solve runs twice: a
  vis-viva estimate of the burn's propellant seeds `v_gate`, and the second pass reads the
  mass off the first pass's arc. Each pass shrinks the mass error by roughly the
  propellant fraction. The residual `v_gate` change across the second pass is the loop's
  convergence witness and is printed in the verdict — 0.02 m/s on the 2026-07-27 plan.

The planner refuses before it plans: a pending node it will not touch; no live engine;
thrust at `f_max` at or below weight, taken at the ship's current mass — the heaviest the
descent sees; a gate at or below the flare height. Past those, five failures abort before
any node is placed: no periapsis in the bracket puts the `f_cap` arc's stall at the gate
(raise `h_gate`, add thrust, or lower `f_headroom`); the march hit its step cap with the
arc unfinished, which is a bug witness and not a placement problem; the arc's propellant
does not fit the tanks, reported in tonnes against what the tanks hold; the lead search
reaches a quarter of the body's circumference with the demand still above `f_cap`; the
solved lead's profile has no demand crossing inside the certified `t_go` bracket.

What the solve does not rule is the ground under the arc. `h_gate` above the site's
terrain is the one clearance input the planner carries; `terrain-certification.md` holds
the rest.

## The lead comes from the law, not from the arc

The reference arc's reach is the *gravity turn's* reach, and the flight does not fly a
gravity turn — it flies E-guidance, a different curve. So the lead X is sized from the law
the flight actually flies: X is where that law's cheapest profile demands exactly `f_cap`,
which reserves the whole `f_cap`..`f_max` band for what happens after ignition.

The profile's demand is closed form. Commanded acceleration is linear in time, so the
demand peaks at an endpoint; the two endpoint demands cross once on the bracket
`(1.5, 3)·X/v₀` — the walls where the law's endpoint accelerations flip sign
along-track, exact for any state — and that crossover is the dip: the least peak demand
any `t_go` buys at this lead, sitting at `2·X/v₀` (constant deceleration) in the planar
limit. Each endpoint is priced at its own mass and its own local gravity. The form
is exact under constant gravity and approximate here only through the ~11 degrees the arc
subtends; modelled, it under-reads the closed-loop peak by about 0.015 of throttle, inside
the reserve.

Demand falls as the lead grows — more ground, a gentler profile — so the placement
bisects `dip demand − f_cap` between the reference arc's own reach, where the dip sits
above `f_cap`, and a far end found by *doubling* the reach until the demand drops under
it. The search grows in the craft's own units, because the reach is the craft's own length
scale. The ceiling is geometric: a lead past a quarter of the body's circumference is no
longer an approach, and reaching it aborts. A lead whose profile has no demand crossing
inside the certified `t_go` bracket is read as demand above the cap. A dip already at or
under `f_cap` at the arc's reach adopts the reach, since extra lead would spend margin the
plan already has.

Worked, from `doi_plan_20260727_klumpp.log`: reach 16,373 m at `f_cap` 0.765, lead
18,128 m, dip 0.765 at `t_go` 62 s. The flight ignited 18,042 m out and chose `t_go`
62.9 s.

## `f_headroom`

The share of the throttle ceiling the plan leaves unspent. The nominal guidance demand is
placed at `f_cap = (1 − f_headroom)·f_max`, and the band above it is **command margin for
the feedback law** — what guidance holds for dispersion arriving after ignition: model
error, steering lag, the decrementing clock. It is not reserve for shortening an arc, and
it does not cover delivery error: the ignition `t_go` choice is made from the delivered
state, so delivery error and the delivered flight-path angle are absorbed into the
schedule instead of being booked here. Self-scaled to the craft's thrust and the body's
gravity instead of a fixed distance factor. Dimensionless, 0.1, provisional until a flight
falsifies it.

The 2026-07-27 demand trace peaked at 0.761 against the flight's own solved dip of 0.733
and the planned `f_cap` 0.765 — 0.028 of post-ignition excursion, 0.004 of margin to the
cap. The reserve went unentered because delivery came in easier than planned, not because
the excursion was small: the same 0.028 from a dip at `f_cap` would have reached ~0.793, a
third of the way in. Still provisional; one flight, one craft.

## Open

- Braking-arc terrain clearance is unruled: the chord certificate covered gravity-turn
  arcs and the flight does not fly one. `terrain-certification.md` holds the replacement
  and the reason it is deliberately unbuilt; the flight logs a running minimum-clearance
  witness in the meantime, which on 2026-07-27 bottomed at 223 m.
- Scope, standing: vacuum only, and prograde near-equatorial orbits. The ground-motion
  term in the periapsis-speed function and the lead's layout in longitude are both
  equatorial. Lifting either is designed separately.
