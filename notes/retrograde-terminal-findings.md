# The retrograde terminal: what its first flights measured

*A findings record for the 2026-07-26 redesign of `reference/original/powered_descent.ks`
(uncommitted in the working tree as of writing): the flights flown, the miss budget they
measured, and the fix set they argue. Registers it bears on:
`powered-descent-invariants.md`, `powered-descent-handoff-contract.md` (largely describes
the replaced terminal — see Register debt), `doi-planner.md`. Witnesses:
`reference/original/flight_log_20260726_klumpp_terminal.csv` (the last flight of the old
terminal), `flight_log_20260726_retrograde_terminal.csv` (first flight of the new one),
and `flight_log_20260726_retrograde_terminal_2.csv` (second flight, with lat/lng witness
lines). All flights: same craft (thrust 19.99 kN, Isp 319.7 s, 3.96 t at PDI), same DOI
plan (`doi_plan_20260726_baseline.log`: PDI 5992 m,
563.7 m/s, 39.7 km from the site, f_cap 0.765).*

## The redesign under test

Terminal is the braking law continued to the ground: hold surface retrograde through a
coast from high gate, ignite the arrest burn when f_max could just bring the total speed
to rest at the pad, throttle to carry the descent rate to v_floor, plumb below walking
speed. Klumpp's guidance, the plumb latch, the alignment gate, and the lateral-cap
machinery are deleted. `endpoint` integrates the whole descent — brake, coast, arrest —
to touchdown, and `solve_f` aims the predicted touchdown point at the site with no
stopping-distance term.

## What the flights measured

**The old terminal's defect, confirmed (klumpp CSV).** High gate arrived 2.5 s of fall
above suicide-burn ignition against the ~21 s the lateral guidance needed, so the ship
touched down still carrying 16 m/s of horizontal drift; miss 293 m (direction not
logged). The free-fall window the design assumed was never produced by anything.

**The new terminal works (both retro CSVs).** Drift 10.2–10.4 m/s at high gate decayed
to 0 at touchdown; vertical speed −2.7 m/s at the pad; upright; no slew before ignition;
throttle reserve untouched; the offset moved only ~17 m between high gate and touchdown.
The fast-horizontal-touchdown failure mode is gone by construction.

**But the miss grew: 605 m, then 612 m, and ~31 m/s more Δv than the old flight.** The
lat/lng witness lines (second retro flight) place the lander ~593 m past the site along
an eastward track (pe at lng −94.92 → site at −83.68, inclination ~1.2°). Refined against
the offline reproduction: **along-track ≈ 380 ± 50 m long, cross-range ≈ 440–500 m**
(measured; the split is limited by the cross column's normal rotating onto the
along-track axis once the drift verticalises).

## The miss budget (offline reproduction, Python re-implementation of `endpoint`)

The reproduction tracks the flown braking arc to ≤40 m altitude, 1–3 m/s speed, and
+90…+170 m of reach over 30.7 km — the arc model is sound. The budget, signed + = long:

| item | metres | status |
|---|---|---|
| Margin at PDI (root 0.755 vs ceiling 0.85) | −4482 available | modelled |
| 18.0 s flown at seed throttle 0.3449 while the first solve ran | +5276 | latency measured, reach modelled |
| ⟹ overshoot already unavoidable at f_max | +794 | modelled |
| Late relaxation 0.85 → 0.425 (last 5 s of braking) | +108 | modelled from flown throttle |
| Yaw bias's a·cos ψ reach the model does not predict | +82…+239 | modelled, ψ fit uncertain |
| Aim's cross-range term while throttle had authority | +23…+36 | measured geometry |
| Delivered along-track | +380 ± 50 | measured |

**The dominant cause is solve latency, not the plan and not the model.** Margin at f_max
drains at 315 m per second of latency when the seed is 0.345 (54 m/s at seed 0.755). Two
compounding mechanisms, both measured:

1. **The first in-loop solve consumed 18.0 s of flight** (10.4 s under the old model —
   the new endpoint runs 1.23× the cost per step, 1.8× the steps at high f, and its three
   f = 0 bracket probes from a near-orbital state each run to the 4000-step cap). The
   ship flew that whole time at the seed throttle.
2. **`solve_f`'s objective re-reads `dist_to_site()` on every evaluation** while the ship
   closes at ~536 m/s, so bisection chases a target that moved 9.4 km during 14
   evaluations. The returned 0.85 was a truncation artifact — the lower bracket walked up
   to the ceiling; at the state the solve ended on, no root existed. The 116 s pin and
   the bracket-failure spam follow.

The 0.3449 seed is a *legitimate* root of the new model at the pre-PDI state — the old
seed of 0.574 was inflated by the removed vh²/(2·a_eff) term, which at sub-high-gate
throttles added 17–170 km of fictitious stopping distance and was accidentally closer to
the correct PDI answer. The seed's real defect is semantic: it answers "what throttle if
I light now" from a state 60 s and 33 km up-range of ignition.

**`plan_doi` is exonerated.** From the true periapsis state the root was 0.755 against
the planner's f_cap 0.765 with 4.5 km of shortening margin — placement exactly per
`doi-planner.md`'s f_headroom design. The earlier hypothesis (plan infeasible under the
new contract) is overturned; the handoff-contract note's ripple check still deserves
restating, but no placement change is indicated by these flights.

Secondary model biases, both real but non-causal this flight (the throttle was
saturated): the sim books the high-gate horizontal speed as along-track reach through
coast and arrest (+115–200 m of phantom prediction; the flight's post-gate along-track
gain was ~40 m, the residual being ~90 % cross-track), and the aim compares in-plane
reach against the full great-circle distance including cross-range — negligible early,
dominant after t ≈ 100 s when the whole remaining distance is sideways, which is what
produced the harmful 0.425 relaxation.

## The yaw law's gain error (first retro CSV, separate offline analysis)

The plane-closing bias underdelivers by a factor of ≈ 2, and it is a geometric gain
error, not a lag. n is defined from the velocity, so v·n ≡ 0 identically: the offset y
closes only through the ground track's azimuth turning, ẏ = −Ω·X with X the downrange
distance remaining. The commanded pretend cross-speed y/τ buys Ω = a·(y/τ)/(|v|·v_h),
so the achieved closure is y/τ · G with G = a·X/(|v|·v_h) ≈ 0.48–0.58 across the burn
(τ_eff = τ·|v|·v_h/(a·X): predicted 93.7 s at ignition, fitted 93.6 s — a clean
first-order exponential at the wrong constant). G ≈ 0.5 is scale-invariant (X is itself
a stopping distance ~v²/2a), so the law leaves the same *fraction* everywhere — which is
why it looked fine on Minmus, where half of a small offset is small. Closure dies once X
collapses (below ~100 m of downrange the channel is dead). The structural fix commands
the bias as (y/τ*)·|v|·v_h/(a·X) — dividing the gain out so closure is y/τ* by
construction — with a floor or freeze as X collapses; a bare retune (τ ≈ 18–20 s,
data-calibrated 64 s/τ) also reaches tens of metres but fits a value where the
derived-constants rule wants an argument. Δv cost is quadratic in the PDI offset:
&lt; 1 m/s at this flight's 1184 m.

## Fix set, in argued order

1. **Freeze the solve.** Capture state and target once at the top of `solve_f`; solve
   against the frozen problem; account for the elapsed time when applying the answer.
   Removes the moving-target truncation entirely.
2. **Seed from the periapsis state, not the warp-out state.** Propagate to periapsis
   (or solve the frozen problem *for* the periapsis state) so ignition starts at a
   defensible throttle. Items 1+2 are worth the entire +5276 m.
3. **Cheapen the bracket.** The three f = 0 probes at 4000 steps dominated the 18 s.
   Options: seed the bracket from the previous solution, special-case f = 0, or cap the
   coast segment. Also restores the invariants register's IPU pricing (~1–2 s per look).
4. **Implement the feasibility ordering** from `powered-descent-invariants.md` (case 2:
   fly f_max, eat the smallest overshoot, say so once, quietly) instead of the
   accidental stale-f_cmd behavior plus bisect's four-line spam — both halves already
   documented as open issues in `doi-planner.md`.
5. **Aim at the in-plane component**: sqrt(dist² − cross²), cross being the quantity the
   log already carries. Kills the late sideways-chasing (+108 m here) for one line.
6. **Yaw gain fix** (structural form above). Largest single accuracy lever remaining
   (~440–500 m of this flight's miss).
7. **Model refinements** (secondary, after 1–6 fly): zero the post-gate θ accumulation
   (or scale by the along-track cosine), and model braking deceleration as a·cos ψ.
8. **One instrumentation line**: log tau_yaw and t_brake in the CSV header — the flown
   tau is uncertain (t_go implies ~50 s; the cmd_pitch fit says ~20 s) and the yaw-fix
   design needs it settled.

## Register debt

`powered-descent-handoff-contract.md` mostly registers the deleted terminal (d_handoff,
Klumpp, latch/gate/fence) and needs rewriting once this design settles.
`powered-descent-invariants.md` survives with amendments (invariant 2's endpoint is now
touchdown; invariant 5's continuity is now trivial — same attitude law across the
handoff). `doi-planner.md`'s chord-certificate premise ("flown arcs end earlier and
higher") needs restating for a flight that now descends to the ground — the descent
stays inside the near-vertical cone over the site, but the register should say so rather
than rely on it.

## Working-tree state (as of this note)

Uncommitted on branch `fix/doi-node-placement`: the `powered_descent.ks` redesign
(including the lat/lng witness lines and the v_switch-gated cosine factor from review);
`boot/main.ks` (AFBW disable) and `ssto2.ks` + untracked `ascent_record.ks`,
`ssto2_incl.ks` (a separate ascent-instrumentation stream) — disposition undecided.
Review found no critical defects; one cosmetic item open (the arrest-trigger formula
written out twice). The `v_switch` constant (5 m/s, plumb-below speed) is a chosen
tolerance without a derivation. The old flight's miss direction was never logged and
remains unknown.
