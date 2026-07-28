# Klumpp braking over a solved plan: the descent design

*A design document. Status: shipped — `plan_doi.ks` and `powered_descent.ks` implement it,
it has flown repeatedly, and it lands. Numbers below are marked modelled where they are
modelled; the flown ones name their witness. It designs `plan_doi.ks` and
`powered_descent.ks` together. Companions:
`node-delivery-window.md` (the delivery scatter this design takes as an input),
`retrograde-terminal-findings.md` (the flights that motivated it and the flights that
tested it), `apollo-powered-descent.md` and `klumpp-guidance-derivation.md` (the guidance
law and its `t_go` closures, normative sources — with three named departures, flagged
where they occur), `doi-planner.md` and `powered-descent-invariants.md` (the two registers
this design lands in). Terrain
certification of the up-range braking arc is out of scope: `h_gate` above the site's
terrain is the one clearance input this design carries, and certification of the ground
under the approach stays deferred to `terrain-certification.md`.*

## Motivation, from measurements

Four measured facts, all from 2026-07-26 flights of the same craft and plan:

- **Terminal-law fuel parity.** The Klumpp-terminal flight spent 626.2 m/s (931.5 minus
  dv_rem 305.3); the fixbatch flight — the retained retrograde terminal — spent 631.2
  (dv_rem 300.3); the first retrograde-terminal pair, which overshot 600 m, spent 657.7
  (dv_rem 273.8/274). All three flew the *same retrograde-hold braking phase*, so this
  measures terminal laws only: at equal braking, the terminal choice is a few-m/s line
  item. What these flights do not measure is Klumpp flown as the *braking* phase — the
  design here. The objection to it was a cosine-loss argument, and it is settled by
  computation below, not by these flights.
- **The crash.** The retrograde design flew into the ground still in its braking phase:
  +1,314 m of node delivery error pulled the solved throttle down to arcs whose high
  gate had no altitude under it, and the feasibility clause that would have refused
  them was design without code (`retrograde-terminal-findings.md`).
- **The delivery window.** PDI arrives ±1.5 km along-track of plan, structurally
  (`node-delivery-window.md`). The retrograde design answers delivery error with its
  throttle — the same knob that sets gate altitude — so the window points the arc at
  the ground. A feedback law with command margin answers it by re-timing the burn at
  ignition, for single-digit m/s.
- **The complexity audit.** The retrograde `powered_descent.ks` ran near twice
  `plan_doi.ks`'s length, and the excess was re-solver machinery — frozen solves, bracket
  seeding, case handling, a gain-corrected yaw law — all built to rescue off-plan states
  that a feedback law absorbs natively.

The criteria are unchanged: Δv is the criterion, 10 m accuracy is the bar,
terrain clearance is a design input, entering here solely as `h_gate`.

## The worked example

Every number below with a unit attached belongs to **one craft, one site and one gate**
unless another flight is named: the 2026-07-26 Mun lander — thrust 20.0 kN, Isp 319.7 s,
3.96 t at PDI, 3.31 t at the gate, so `a_dec` 3.51 m/s² and TWR about 3 — landing at a
1982 m site with `h_gate` 300 m, from PDI at 5982 m and 563.4 m/s. Its `f_cap` is 0.765,
its solved lead 41.2 km, its `v_gate` 23 m/s. None of those are design constants; they are
what this design returns when it is handed that craft and that gate.

The design's own quantities are the arguments, not the values: `h_gate` is a parameter of
both programs, `f_max` and `f_headroom` are the ceiling and its reserve, and everything
else — `h_pdi`, the lead, `v_gate`, `a_dec`, `t_go`, the search brackets — is solved from
the craft, the body and the gate. Checked offline across craft from TWR ~1.5 to ~15 on
airless bodies; the second flown craft (2026-07-27, `retrograde-terminal-findings.md`) is
a 58 kN, 4.25 t vehicle at `h_gate` 1500 m, which returns `a_dec` ~10 m/s², `v_gate`
86.4 m/s, `h_pdi` 4004 m and an 18.1 km lead from the same code.

**Standing scope limits.** Vacuum only — nothing models drag. Prograde, near-equatorial
orbits — the periapsis-speed function nets out the ground's eastward motion as a scalar
and the lead is laid out in longitude, both of which are equatorial constructions. Lifting
either is designed separately; they are recorded here as limits, not as open questions
this design will answer.

## The architecture

The nomenclature is settled and binding:

**DOI → descent coast → PDI → braking (Klumpp) → high gate → FALL → low gate (arrest
ignition) → arrest burn → touchdown → settle.**

Both gates are events, and neither is an attitude. High gate is *delivered at a designed
altitude* — over the site, `h_gate` above its terrain, descending inside the vertical
corridor defined below. Low gate is the arrest schedule firing: `f_max` can just bring the
speed to rest `h_pad` above the pad.

| phase / event | definition | law |
|---|---|---|
| DOI, descent coast | node from `plan_doi`, on-rails ellipse to periapsis | — |
| PDI | ignition at periapsis; `t_go` chosen, feasibility checked | — |
| braking (Klumpp) | one vector law flies position, velocity, and throttle toward the gate state | `a_cmd = 6·(r_tgt − r)/t_go² − (4·v + 2·v_tgt)/t_go` |
| high gate | event: braking exits over the site at `h_gate`, mid-corridor descent, drift nulled | `t_go` reaches the floor `t_go_floor` |
| FALL | engine off, retrograde attitude hold (≈ plumb at zero drift), vertical fall | — |
| low gate | event: the arrest schedule fires on `h_bot` | existing |
| arrest burn | commanded vector: vertical schedule to `v_floor`, plus the horizontal law run to rest over the site | `a_vert·up + a_lat`, `a_lat = 6·d/t_h² − 4·v_lat/t_h` |
| touchdown, settle | contact witness, settle trace | existing |

**FALL stays inert — decided.** No powered approach phase follows braking; the horizontal
correction lives in the arrest, which is already burning. The Apollo P64 approach phase
stays a *designed extension point*: the same guidance law with a second target set,
switched in at the gate rather than at low gate, and with a fresh `t_go` instead of the
arrest's leftovers. What promotes it is the arrest's horizontal law failing to hold the
miss inside `r_bar` — the arrest corrects late, after the fall has already multiplied the
gate residuals, and it corrects with whatever lean it can spare from the flare. The
trigger is the touchdown `miss`, not the gate residuals: a flight can sit inside the
gate's own drift and offset bounds and still land outside the bar, because the lever
between them is the fall. Apollo's other
reasons for P64 do not apply: its trajectory was shaped so the commander could see the
site out the window, and kOS reads state vectors with no window; it existed so a human
could redesignate the site mid-approach, and ours is a fixed geoposition chosen before
DOI; its profile kept thrust demand out of the DPS's forbidden 65–94 % throttle band,
and our engine throttles continuously; it managed the attitude transition to
windows-forward flight, and our pitch-over falls out of the vector law with no attitude
program.

FALL holds surface retrograde. With drift small at the gate, retrograde
and plumb coincide to within the drift residual, so — per the thrust-waits-for-attitude
lesson (`powered-descent-invariants.md`, Standing lessons) — there is no slew to pay at
arrest ignition: the nose is already on the thrust direction when low gate fires. The
alignment is bought at FALL entry, not free: braking's arrival attitude is the profile's,
near the horizon when the arrival acceleration is mostly braking, and the swing from it
to plumb spends the front of the FALL window (High gate placement, third bullet). The
terminal chain below the gate — FALL, the low-gate schedule on `h_bot`, the arrest, the
witness lines — has flown 33 m and 34 m misses under the retrograde braking phase and
15 m under this one, all three with an arrest that held retrograde and corrected sideways
only as a by-product. The `ship:bounds` altimeter has now flown a flare: `# bounds dh_core 1.52`
accepted by its guard, `h_bot` reading 0.2 m at contact against `alt:radar` 5.5, tilt 0.5°
and no tip-over (`retrograde-terminal-findings.md`).

## The vertical corridor: the design's feasibility invariant

Everything below the gate is vertical, so feasibility below the gate is one inequality,
written on the total surface speed — the variable the retained arrest trigger actually
reads (`powered_descent.ks`, the `burning` latch). With `a_dec = f_max·T/m − g` the net
arrest deceleration:

```
v_surf² ≤ 2·a_dec·(h_gate − h_pad)
```

At the gate `v_surf² = v_v² + drift²`, and with drift held to a few m/s the drift term
is under one percent. **If the inequality holds at the gate it holds thereafter,
provided only `a_dec > 0`** — thrust at `f_max` beats local weight, TWR > 1, true of
any craft that can land at all. Falling a metre grows `v²` by `2g` while the stopping
budget `2·a_dec·(h − h_pad)` shrinks by `2·a_dec`, so the margin between budget and
need falls monotonically at `2·(a_dec + g)` per metre and crosses zero exactly once —
that crossing is low gate, and it sits above the pad if and only if the corridor held
at the gate.

The formula is evaluated at the *gate* mass, not the PDI mass. Worked on the example
craft: the fixbatch_2 CSV shows 3.312 t at high gate (its a_cmd 5.08 m/s² at throttle
0.841 confirms thrust 20.0 kN), so `a_dec = 0.85·20.0/3.312 − 1.628 = 3.51 m/s²`,
corroborated by the flown arrest itself: v_vert −27.8 → −3.7 m/s over 7.7 s is
3.13 m/s² net at throttle 0.78–0.84. The hard wall at its 300 m gate is
`√(2·3.51·295) ≈ 45 m/s` — faster and no throttle avoids the ground. The soft wall is
hover: slower costs fuel at `g0·t`. The design targets arrival mid-corridor,
`v_gate = wall/2`, 23 m/s there, so delivery and guidance errors spend margin on either
side instead of finding a wall. The rule is the halving, not the speed: the 2026-07-27
craft at `h_gate` 1500 m gets a 173 m/s wall and an 88 m/s target from the same line, and
arrived at a measured corridor fraction of 0.25 — the fraction the halving predicts.

The corridor decides the law's shape. With horizontal-only Klumpp over a separate
vertical schedule, the gate's descent rate is uncommanded — verifiable on arrival, not
steered to. Full-vector (3D) Klumpp makes mid-corridor `v_v` a *commanded* boundary
condition, part of `v_tgt`, delivered by the same polynomial that delivers offset and
drift. That is the argument for 3D, and the old build's failure is its supporting
history: its horizontal law and vertical schedule kept separate clocks, and the 2.5 s
of fall the schedule left was no horizon for the 21 s the lateral law needed. One
vector law cannot disagree with itself about the horizon.

## The braking law's fuel and feasibility, modelled

The objection to flying Klumpp over the whole braking phase was that it spends cosine
losses where the state is expensive. Modelled, the objection does not hold on the
efficient branch — but the branch structure it missed is real and shapes the whole
design. Method: a one-page Python integration (2-D orbital plane,
inverse-square gravity, the law closed-loop at 20 Hz, mass by the rocket equation;
craft constants from the logs — thrust 20.0 kN, Isp 319.7 s, m_pdi 3.96 t; PDI
endpoints from the plan logs — h 5982 m, ground-relative speed 563.4 m/s, γ 0; gate
300 m over the 1982 m site at 23 m/s vertical; ground lead 39,198 m; the reference is
the same integrator flying the constant-throttle gravity turn at f_cap 0.765 from the
same state to the gate altitude). Modelled, not flown:

- **Peak thrust demand over the burn is not monotone in `t_go`.** It falls steeply from
  short burns to a **dip at `t_go` ≈ 2.0·X/v₀ (141.1 s here): Δv 591.0 m/s, peak
  demand 0.803**, rises to a local maximum (1.065 near 3·X/v₀ ≈ 209 s, where the
  along-track term `4v/t_go − 6X/t_go²` peaks), then falls again onto a hover branch — the
  far-side root of "demand = f_cap" is a *lofted* profile (t_go ≈ 492 s) that climbs
  5.4 km before descending and spends **1,032 m/s**. Any `t_go` selection rule that
  assumes monotonicity and takes the first root from above lands on the hover branch
  and wastes ~440 m/s.
- **On the dip, the fuel comparison is a wash — which settles the objection.** At
  matched arrival conditions the gravity-turn reference comes to **~592 m/s all-in
  against the law's 591.0**. (The reference march's raw 604.5 is a stalled-march
  artifact — it hits its speed guard 32 m above the gate, and the figure moves
  589–615 with step size; the matched-arrival ~592 is the comparable number.) The
  conclusion stands on the wash, not on a win: fuel is not a reason to reject the
  law. The dip profile is near-retrograde throughout — throttle runs a shallow U — so
  there is no expensive state for cosine losses to spend against.
- **The lead is a nearly free margin knob.** Sweeping the ground lead 39.2 → 45 km
  moves the dip's peak demand 0.803 → ~0.71 while Δv moves by ~4 m/s — a tenth of
  throttle for single-digit m/s. The dip crosses f_cap 0.765 at **X ≈ 41.2 km**
  (42 km gives 0.752, conservative), which is the placement condition the planner
  adopts below.

## `plan_doi`: the solved PDI altitude and the law-sized lead

The planner answers two questions with two instruments.

**How high is PDI: a forward march under bisection.** The reference arc is the
constant-throttle gravity turn at `f_cap`, marched *forward* from the periapsis a
candidate `h_pdi` implies — pitch zero because a periapsis is horizontal, speed from
vis-viva with apoapsis at `ship:orbit:semimajoraxis` (the source that reproduces the flown
563.7 m/s; "parking radius" is ambiguous by 5 km and moves the lead by hundreds of metres),
less the ground's eastward motion, mass the ship's own because the coast burns nothing.
The march ends at the arc's stall — speed down to `v_gate`, past which the turn equations
verticalize on the spot — or at the gate altitude, whichever comes first.

`h_pdi` is then the root of *stall altitude minus gate altitude*, bisected. The sign
changes once: an arc from too low reaches the gate altitude still fast, one from too high
stalls above it. Marching forward from a candidate is what makes the arc well-posed —
there is no seed degeneracy to patch, because a periapsis state is fully determined by its
altitude, and no self-check to run, because the solved arc *is* the forward arc.

Nothing in the solve is a magic number. The bracket runs from `alt_gate + 1` to
`ship:orbit:periapsis` — the gate below, the ellipse family's ceiling above — with both
signs checked before bisecting so a bracket failure aborts in the planner's own words. The
tolerance scales itself: stop when the bracket is narrower than `v_frac` times the
midpoint's height above the gate, because the march carries roughly `v_frac` of relative
error and a tighter root is precision the function does not have. The whole solve sits
inside a two-pass gate-mass contraction: `v_gate` depends on the gate mass through `a_dec`
and the gate mass is read off the march, so a vis-viva propellant estimate seeds the first
pass and the second reads the mass off the first pass's arc. A relative error `δm/m`
returns as a propellant error roughly the propellant fraction (~0.2) times smaller, so two
passes leave mass error below 0.1 %; the residual `v_gate` movement across the second pass
is printed in the verdict as the loop's convergence witness (0.02 m/s on 2026-07-27).

The planner refuses before it plans — no pending node, a live engine, thrust at `f_max`
above weight, a gate above the flare height — and three failures abort the march: no
periapsis in the bracket puts the `f_cap` arc's stall at the gate; the march hit its step
cap with the arc unfinished, which is a bug witness rather than a placement problem; the
arc's propellant exceeds what the tanks hold.

**How far up-range: the law's own feasibility.** The lead X is *not* the march's swept
ground — that is the gravity turn's reach, a different curve. X is sized from the law
the flight actually flies: place PDI at the X where the E-guidance dip demand equals
f_cap (41.2 km for the example craft and gate, against the march's 39.2). Demand falls as
the lead grows, so the placement bisects `dip − f_cap` between the arc's own reach and a
far end found by doubling the reach until the demand drops under it — the search grows in
the craft's own length scale — with a quarter of the body's circumference as the ceiling,
past which a lead is not an approach and the planner aborts. A lead whose profile has no
demand crossing inside the certified `t_go` bracket is read as demand above the cap, and
the solved lead is required to have one. Δv is indifferent at the m/s
level across the whole sweep, so the condition spends nothing. This is the first named
departure from `apollo-powered-descent.md`, which sizes `t_go` by a 90 %-authority
solve at a fixed aim point: that rule assumes demand falls monotonically with `t_go`,
and modelled here it does not — applied blindly it selects the hover branch.

`f_headroom` is **command margin for a feedback law**: the gap between the planned dip
demand (f_cap) and `f_max` is what guidance holds for dispersion arriving after ignition,
and nothing else is booked against it. 0.1, provisional until a flight falsifies it.

## `powered_descent`: the flight program

The flight carries no trajectory and no channels. Cross-track is not a channel: the
offset is a component of `r_tgt − r` and the law corrects it in vector, at a Δv cost
quadratic in the offset (< 1 m/s at the measured ~1.3 km PDI offset). Attitude is not a
phase boundary: the pitch-over falls out of the law, so no `tilt_max` defines the gate.
**No numerical integration runs in flight** — `a_cmd` is closed-form — so the
solve-latency and moving-target defect class has nothing to live in. `config:ipu` is still
raised to 2000: not for the guidance command, which is arithmetic, but for the ignition
bisection, where a second of game time at PDI is several hundred metres of along-track and
the raise buys the solve back down to a tenth of a second.

The five pieces:

- **Full-vector Klumpp, one law**, for the corridor argument above. Thrust demand is
  `a_cmd − g_vec`; throttle its magnitude over available thrust; attitude its
  direction; the pitch-over from near-retrograde falls out of the law
  (`apollo-powered-descent.md`, phases 3–4). `r_tgt` is the *virtual* gate (next
  bullet); `v_tgt` its matching velocity.
- **The virtual gate.** E-guidance does not taper: at any positive `t_go` it still
  commands its arrival acceleration — thrust demand 4.90 m/s², commanded `|a_end|`
  4.34 m/s² here — so exiting at a floor `t_go_floor` aimed at the real gate hands FALL a
  velocity error of roughly `|a_end|·t_go_floor`: **13.0 m/s at a 3 s floor** (8.74 at 2 s),
  against a 1–2 m/s drift bound — structurally the old design's 16 m/s
  touchdown-drift failure reborn one phase earlier. The law therefore aims at a
  virtual gate: the real gate state propagated `t_go_floor` forward along the profile's own
  linear acceleration, offsetting velocity as well as position. With
  `k = (a₁ − a₀)/t_go_ign` the planned profile's jerk,

  ```
  v_virt = v_gate + a₁·t_go_floor + ½·k·t_go_floor²
  p_virt = p_gate + v_gate·t_go_floor + ½·a₁·t_go_floor² + (1/6)·k·t_go_floor³
  ```

  The floor is solved at ignition, not chosen — the smaller of two demands:

  ```
  t_go_floor = min( √(6·r_bar/a_dec) ,  ½·h_gate/v_gate )
  ```

  The first is the **law's authority**: a residual `dR` at the exit draws `6·dR/t_go²`
  of commanded acceleration, so the floor is set where correcting a miss the size of
  the accuracy bar `r_bar` costs exactly the spare acceleration `a_dec` the craft has
  at gate mass. Under it the law commands authority it has not got, to chase an error
  smaller than the requirement — the precision floor a scale-free law bangs against.
  The second is the **gate's geometry**: the aim point descends at `v_gate` per second
  of floor, reaching the site's terrain at `h_gate/v_gate`, and the construction
  spends at most half of that. The cap is the wall the authority demand cannot see,
  and it binds only in the high-thrust, low-gate corner — ~1.7 s at TWR 15 with a
  100 m gate on the Mun. Across the swept envelope (TWR 1.5–15) the floor returns
  1.7–11.6 s: 4.1 s for the example craft, whose aim point then sits ~205 m above the
  terrain of its 300 m gate, and 2.4 s for the 2026-07-27 craft.

  Braking exits at `t_go = t_go_floor`, at which instant the ship occupies the *real* gate
  state to the law's tracking accuracy, the `1/t_go²` divergence never entered.
  The fallback if the virtual gate misbehaves is
  exit-on-state — leave braking when altitude, offset, and drift all sit inside gate
  bounds — simpler but a race among criteria. The escalation if flights falsify the
  drift bound is the P64 extension point, not a bigger floor.
- **`t_go` at ignition, from the dip.** A powered phase has no schedule intersection to
  read `t_go` off, so it is chosen. At ignition the program evaluates the planned profile's peak demand — closed form from
  the linear profile's endpoints, exact under constant gravity and approximate here
  only through the ~11° the arc subtends, so "no integration in flight" stays true —
  over a bracket of candidate `t_go`, and takes the minimum: the dip. Feasible means
  the dip demand fits under `f_max` with margin; the planner's lead puts the nominal
  dip at f_cap, and the ±1.5 km window moves it by ∓0.018 per km — ∓0.027 across the
  window, worst case ~0.79, inside `f_max` 0.85. Implementation is cheaper than a
  minimum-search: the dip is exactly the crossover of the two endpoint demands — a
  minimax — so the flight root-finds `d_ign = d_gate` rather than searching, and
  `max(d₀, d₁)` is unimodal on the bracket (validated numerically, not asserted).
  The bracket is `(1.5, 3)·X/v₀`, the walls of the two-ended braking class: the law's
  endpoint accelerations flip sign along-track at exactly `1.5·X/v₀` (ignition end) and
  `3·X/v₀` (gate end) for any delivered state — identities of the cubic form — so
  outside the walls the profile thrusts toward the site at one end: the short-burn wall
  below, the lofted hover branch above. The demands cross once between them, at
  `2·X/v₀` in the planar limit — constant deceleration, which is why the dip lives
  there; gravity pricing and the vertical channel move the crossing by percents
  (modelled 141.1 s against `2X/v₀` 139.2), never the walls. In
  flight `t_go` decrements by clock to `t_go_floor` — the second named departure from the
  source, which re-solves every ~10 s; re-anchoring the schedule re-admits the
  moving-target class. Flown, decrement-only sheds nothing worth chasing: the demand
  trace runs a shallow U from 0.735 through 0.701 to 0.761 against an `f_max` of 0.85,
  and `sat_s` finishes at 0.
- **Feasibility as refusal.** Two guards run before the coast, where declining costs
  nothing: a live engine with thrust at `f_max` above weight, and a gate above the flare
  height. At PDI, if the demand crossing falls outside the bracket, or the dip demand
  exceeds `f_max`, the program declines to ignite and says which. The ship is then still
  at the periapsis of a stable, quicksave-able ellipse: declining to ignite *is* the
  abort, and it is checked at the one moment aborting is free.

## High gate placement

`h_gate` is a parameter of both programs, and `v_gate = wall/2` follows from it
(mid-corridor by construction — symmetric margin, craft- and body-aware through `a_dec`,
free of both as a rule). The altitude's own argument:

- **Contain the arrest.** Low gate falls at
  `h_lg = h_pad + (v_gate² + 2·g0·(h_gate − h_pad))/(2·(a_dec + g0))`, and the gate has to
  sit far enough above it that the arrest fits underneath. On the example craft's 300 m
  gate entered at 23 m/s that is 149.9 m at gate mass — a ratio `h_gate/h_lg` of 2.0 — and
  the flown anchor is fixbatch_2's 283 m gate with its arrest firing at radar 126–127
  (the first ARREST row), ratio 2.23 against a predicted 131 m, a 4 m error. The ratio is
  *not* a constant: it carries `g0/a_dec`, so the 2026-07-27 craft's 1500 m gate predicts
  low gate at ~551 m — ratio 2.7 — and its arrest fired between the 536 m and 436 m radar
  rows, the row cadence being 100 m of fall at that speed. Two anchors, two ratios, one
  formula that fits both: `k_gate ≈ 2` is the example's number, not the design's. What
  falsifies `h_lg` is a flight whose arrest peaks at the throttle ceiling — 1.0, the
  engine's own limit — or reaches the pad still above `v_floor`; the band above `f_max` is
  the arrest's reserve, and flights are expected to enter it under planning slop.
- **Contain the plumb slew.** Braking exits at the profile's arrival attitude — pitch
  −15.8° flown, near the horizon, because the arrival acceleration is mostly braking —
  while FALL commands plumb, so FALL opens with a slew: 106° flown, closing
  105.8 → 99.6 → 77.2 → 46.8 → 17.4 → 0.4 of facing_err across five 1 s rows, peak
  ~30°/s, settled ~6 s before low gate fired. The budget is the `h_gate → h_lg` fall
  time: ~11 s at the flown 1500 m gate, ~4.5 s at the example craft's 300 m gate — the
  margin shrinks with the gate, and a slow-rotating craft at a low gate meets its arrest
  mid-slew, paying thrust-waits-for-attitude at the most expensive moment. No rule
  computes the slew or checks the window (Open); the FALL rows' facing_err and pitch
  columns witness it per flight.
  the retained arrest thrusts anti-velocity, and the fixbatch flights *measured* it
  closing 128 m of offset to 33 and 9.9 m/s of drift to 0.2. So gate drift is not a
  sentence, it is a cost: ~3–4 m of residual miss per m/s of gate drift as flown
  (33 m from 9.9 m/s), against ~15 m per m/s if nothing corrected. Against a 10 m bar
  that is not comfortable: the law's own 1–2 m/s gate-drift bound already spends 3–8 m
  of the budget, and the fixbatch drift of 9.9 m/s lands at 33 m, three times outside
  it. The bar leaves room for the drift term or for a second term of that size, not
  both.
- **Terrain enters here and only here.** `h_gate` is height above the site's terrain,
  the design's single clearance input; what the site demands adds to it directly.
  Fuel does not argue against it: the whole gate costs ~`√(2·g0·h_gate)` of arrest
  speed — 31 m/s at the example's 300 m — and moving it a hundred metres moves the
  budget by single-digit m/s.

## Failure modes, by phase, with pre-gate observables

**Braking.**

- *Infeasible at PDI.* The dip evaluation above, before ignition, on the ellipse.
  Margin accounting, coherently: delivery error and `γ*` are in the state the `t_go`
  choice is made from, so they are absorbed into the schedule, not the margin — the
  ignition demand lands at f_cap ± ~0.03 across the delivery window. The reserve to
  `f_max` (0.085 nominal, ≥ 0.055 at the window's short edge) covers what arrives
  *after* ignition: thrust/mass model error, steering lag, and whatever dispersion the
  decrementing clock accumulates. Nothing is booked twice.
- *Shared-magnitude coupling.* One thrust vector serves horizontal braking and
  vertical support, and thrust never pushes down — the vertical actuator is
  asymmetric. Saturation sheds the vertical component in effect, and unsupported
  seconds drive the descent rate toward the corridor's hard wall. Saturation duration
  is the pre-gate observable that predicts a wall-side arrival, logged per row.
- *Mis-modelled caps.* The law is indifferent to mass/Isp/thrust error — it commands
  acceleration and reads state — but `a_max`, `a_dec`, and the corridor are computed
  from the model. The CSV logs commanded against *achieved* acceleration each row, so
  a wrong `f_max` or stale mass shows as a persistent ratio error long before it shows
  as a saturated arrival.
- *`t_go` error.* A wrong `t_go` mis-scales every command; the header logs the chosen
  `t_go` against the planner's reference for the cross-check, and `t_go_floor` keeps the
  divergence out of the loop.

**High gate.** Two arrival checks, witnessed on the `# high gate` line before FALL
commits: the corridor inequality on total surface speed with its margin fraction, and
the offset/drift residuals against the law's bounds. A wall-side or saturated arrival
is visible *here*, with the engine still lit, not at the pad.

**FALL and arrest.** The corridor proof is the certificate: inside it at the gate, low
gate always fires above the pad, and the arrest's 0.85 ceiling leaves reserve that
covers ignition lag and discretization — flown: arrest peaks 0.840, 0.842 and 0.858
across the three landings, the last entering the band by 0.008, about five percent of
the 0.15 reserve. The flown set: the arrest chain landed 33 m and
34 m under retrograde braking and 15 m under this law; the bounds altimeter has flown a
flare and its guard accepted the datum; the fixbatch tip-over is *not fully explained* by
the radar-datum error — slope and leg geometry are entangled in it — and it has not
recurred, so the settle trace has nothing new to separate.

## Witnesses, and what they measured

The recorder carries `t_go` (the guidance clock), thrust demand as a fraction of
available (the saturation observable), achieved acceleration beside commanded (the
model-error observable), ZEM magnitude, and — instrumentation for the certification gap
named in Open — the running *minimum radar clearance* over the braking arc, logged so the
flights measure what no rule covers. The pe-longitude witness line from
`node-delivery-window.md` logs at warp-out.

Each signature was predicted as a column and a value before the first flight. Against
`flight_log_20260727_klumpp.csv` (`retrograde-terminal-findings.md` holds the full
reading):

- Header. Predicted: ignition `t_go` within ~10 s of the planner's reference. Flown:
  62.9 s against 62. The pe witness line measured delivery independently, 86 m short of
  the planned lead.
- BRAKE rows. Predicted (modelled): throttle a shallow U, never above `f_max`;
  commanded-vs-achieved within a few percent of 1; `facing_err` single digits, no
  slew-wait rows. Flown: 0.735 → 0.701 → 0.761 against `f_max` 0.85, `sat_s` 0; `ach`
  1.00 ± 0.01 from the third row; `facing_err` 20.1° and 10.7° on the first two rows —
  the one slew off surface retrograde onto the guidance vector — then under 4° for the
  rest of the burn.
- `# high gate` line. Predicted: offset ≤ 30 m, drift ≤ 1–2 m/s, speed within ±10 m/s of
  `v_gate`, corridor fraction ≤ 0.5, altitude within 20 m of `h_gate`. Flown, at a gate
  five times the modelled one: **offset 3 m, drift 1.5 m/s, speed 88.3 against `v_gate`
  88.2, corridor 0.25, radar 1498 against `h_gate` 1500.** The prediction held on every
  column.
- Touchdown. Predicted: miss ≤ 40 m, settle tilt < 5°. Flown: miss 15 m, tilt 0.5°,
  contact at −2.0 m/s and 0.1 m/s of drift. The Δv prediction is unread — the flight flew
  a different craft, so `dv_rem` 1073 has nothing to compare against, and the modelled
  591-vs-~592 braking wash is still modelled.

## What remains live

- The gentle-executor experiment (`node-delivery-window.md`): the pe-longitude witness
  line flies on every descent and has data, so the experiment is unblocked and runs on
  its own thread.
- `BOOK-PLAN.md`: the book follows the code; the design history is chapter material, not
  register material.

## Open

- **Braking-arc terrain clearance is unruled** between PDI and the gate until
  `terrain-certification.md`'s work happens — the chord certificate retired with the
  gravity turn and nothing replaces it yet. One observation for that future work,
  recorded as a comment and deliberately not built into machinery: the modelled
  E-guidance path clears the straight PDI→gate chord over its whole length (minimum
  +0.5 m at the PDI end, +900 to +1,600 m mid-arc), so a single offline march per
  placement could certify braking-arc clearance the way the retired certificate did.
  The flights instrument the gap (the minimum-clearance witness) without pretending to
  rule it, and they have measured how real it is: 223 m of clearance three seconds after
  ignition on 2026-07-27, the tightest point of the whole descent, up-range where nothing
  rules. `terrain-certification.md` remains the pointer for the general problem.
- The low-gate formula is anchored on two flown gate altitudes, 283 m and 1500 m, and
  fits both; the *ratio* `k_gate` does not transfer between them (2.2 and 2.7), so the
  constant-ratio form is falsified and the formula's `g0/a_dec` dependence is what
  survives. Its argument stays partial until the arrival scatter across craft is
  measured.
- The `t_go` bracket's walls are derived — the endpoint along-track sign-flip
  identities at `(1.5, 3)·X/v₀` — and the crossing's planar location `2·X/v₀` is exact
  (constant deceleration). What stays numerical is the gap's single sign change across
  the full span: validated numerically on the `[1.6, 2.6]` interior, unswept on the
  outer margins, where the flown and modelled crossings (both within 2 % of `2·X/v₀`)
  have never been. An offline sweep across certified geometries closes it.
- **Nothing owns the airframe's ability to fly the profile's final attitude.** The
  exit floor answers to the accuracy bar and the gate's geometry, and deliberately not
  to control authority, so the program can command a rotation the craft cannot follow —
  the gains rise toward exit and the nose may simply not arrive. Deferred on the
  grounds that these landers carry good attitude authority; it is a scope statement,
  not a certificate. The per-flight witness is the facing_err trend across the last
  braking rows — 0.7 → 3.7° over the final five seconds flown, the command outrunning
  the nose — and growth there is the signature. What it costs is paid in the same
  currency as everything else at the gate: late dispersion handed to FALL as offset
  and drift, which the arrest prices at ~3–4 m of miss per m/s. If it earns an owner
  it is the plumb-slew guard below, which needs the same angular acceleration and
  guards a larger angle one phase later; it is not the floor's job.
- **The 10 m bar is not met.** The design's one flight landed 15 m from the site, with
  1.5 m/s of gate drift worth ~5 m of that. No flight has hit the bar, and the drift
  term alone spends most of the budget (High gate placement), so meeting it is a
  design question and not a matter of tightening what is there.
- The FALL-entry slew is unbudgeted: the plumb-slew window (High gate placement, third
  bullet) is checked by no rule. The pieces of a guard exist without new physics: the
  swing angle is the angle between the profile's gate-end thrust direction — `a₁ − g`,
  already computed at the solve — and plumb; the window is the `h_gate → h_lg` fall
  time; the slew law is the steering manager's, `ω_max = MAXSTOPPINGTIME · torque/MOI`
  with `MAXSTOPPINGTIME` readable (default 2 s) — so the one craft number missing is
  the angular acceleration `torque/MOI`, which kOS exposes no suffix for and a step
  response measures (`steeringmanager:writecsvfiles` logs the six loops; a bridge
  attitude-step on orbit is the clean instrument). The flown swing is consistent with
  that model at 1 Hz resolution: ramp to a ~30°/s peak ≈ `2 s · torque/MOI`, then
  106° in 5–6 s. The knob for a craft that fails it is `h_gate`: a higher gate buys
  window at ~`√(2·g0·h_gate)` of arrest speed — single-digit m/s per hundred metres.
  Registered, deliberately unbuilt.
- Saturation response in braking — ride it or abort to orbit — is policy, Schuyler's
  call, and a named departure from the source's abort prescription.
- The arrest's horizontal law is **unflown**, and it is not built from a log: it is
  arithmetic over registered numbers, changing what the ship does in its last ten
  seconds over the ground. Nothing in it is chosen: the lean cone is `arccos(f_max)` —
  the thrust reserve read as an angle, 31.8° at `f_max` 0.85 — the clock is the vertical
  schedule's own, and its floor is the braking exit's authority argument applied
  sideways. The margin before contact is `h_pad`, which the flare already aims at, so the
  law comes to rest 5 m up rather than reserving a second one, and the cone closes over
  that same last `h_pad` so the ship reaches the pad upright even if it did not. The
  witnesses are the `# arrest` line at
  ignition — offset, drift, burn time, lean cap — and the touchdown `miss`/`drift` pair,
  which is where the legs' tolerance is measured. The signature of the law working is
  `drift` at contact near zero with `miss` inside `r_bar`; the signature of it being
  under-powered is `miss` outside `r_bar` with `drift` still near zero, which is the law
  correctly choosing rest over the last few metres.
- The flight carries **two changes at once**: the exit floor's new closed form and the
  arrest's horizontal law. That breaks one-instrumented-change-per-flight, and the two
  are separable — the floor moves gate residuals, the horizontal law moves what happens
  to them afterward. If the first flight disagrees with the plan, that ambiguity is the
  reason.
- The fixbatch tip-over's full mechanism (datum error vs slope vs leg geometry). It has
  not recurred; the settle trace and touchdown witness are the instruments if it does.
- Whether the reference arc's `h_pdi` — a gravity-turn altitude — is the right altitude
  for an E-guidance flight is a structural approximation: the two profiles' altitude
  histories agree at the few-percent level on the efficient branch (modelled), and the
  header's `t_go`-vs-reference cross-check witnesses it per flight (62.9 against 62 on
  2026-07-27).
