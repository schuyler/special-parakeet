# Klumpp braking over a backward-integrated plan: the descent redesign

*A design document. Status: proposed, unbuilt — nothing in the tree implements it, and
no flight has tested it; the fuel and feasibility numbers below are modelled, marked as
such. It redesigns `plan_doi.ks` and `powered_descent.ks` together. Companions:
`node-delivery-window.md` (the delivery scatter this design takes as an input),
`retrograde-terminal-findings.md` (the flights that motivated it),
`apollo-powered-descent.md` and `klumpp-guidance-derivation.md` (the guidance law and its
`t_go` closures, here promoted from book drafts to normative sources — with three named
departures, flagged where they occur), `doi-planner.md`, `powered-descent-invariants.md`,
`powered-descent-handoff-contract.md` (all carry ripples, listed at the end). Terrain
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
  item. What flights cannot yet measure is Klumpp flown as the *braking* phase — the
  proposal here — which `powered-descent-handoff-contract.md` argued against on cosine
  grounds; that question is settled by computation below, not by these flights.
- **The crash.** The retrograde design flew into the ground still in its braking phase:
  +1,314 m of node delivery error pulled the solved throttle down to arcs whose high
  gate had no altitude under it, and the feasibility clause that would have refused
  them was design without code (`retrograde-terminal-findings.md`).
- **The delivery window.** PDI arrives ±1.5 km along-track of plan, structurally
  (`node-delivery-window.md`). The retrograde design answers delivery error with its
  throttle — the same knob that sets gate altitude — so the window points the arc at
  the ground. A feedback law with command margin answers it by re-timing the burn at
  ignition, for single-digit m/s.
- **The complexity audit.** `powered_descent.ks` stands near twice `plan_doi.ks`'s
  length, and the growth is the re-solver machinery — frozen solves, bracket seeding,
  case handling, a gain-corrected yaw law — all built to rescue off-plan states that a
  feedback law absorbs natively.

The criteria are unchanged: Δv is the criterion, tens-of-metres accuracy is the bar,
terrain clearance is a design input, entering here solely as `h_gate`.

## The architecture

The nomenclature is settled and binding:

**DOI → descent coast → PDI → braking (Klumpp) → high gate → FALL → low gate (arrest
ignition) → arrest burn → touchdown → settle.**

Both gates are events. High gate is *delivered at a designed altitude* — over the site,
`h_gate` above its terrain, descending inside the vertical corridor defined below —
rather than defined by attitude as the retrograde design had it. Low gate is the arrest
schedule firing: `f_max` can just bring the speed to rest `h_pad` above the pad.

| phase / event | definition | law |
|---|---|---|
| DOI, descent coast | node from `plan_doi`, on-rails ellipse to periapsis | — |
| PDI | ignition at periapsis; `t_go` chosen, feasibility checked | — |
| braking (Klumpp) | one vector law flies position, velocity, and throttle toward the gate state | `a_cmd = 6·(r_tgt − r)/t_go² − (4·v + 2·v_tgt)/t_go` |
| high gate | event: braking exits over the site at `h_gate`, mid-corridor descent, drift nulled | `t_go` reaches the floor `τ_f` |
| FALL | engine off, retrograde attitude hold (≈ plumb at zero drift), vertical fall | — |
| low gate | event: the arrest schedule fires on `h_bot` | existing |
| arrest burn | throttle carries descent rate to `v_floor`; plumb below `v_switch` | existing |
| touchdown, settle | contact witness, settle trace | existing |

**FALL stays inert — decided.** No powered approach phase follows braking. The Apollo
P64 approach phase is recorded as a *designed extension point*: the same guidance law
with a second target set, switched in at the gate, to be built only if flights measure
gate drift beyond ~1–2 m/s or offset beyond the tens-of-metres bar. Apollo's other
reasons for P64 do not apply: its trajectory was shaped so the commander could see the
site out the window, and kOS reads state vectors with no window; it existed so a human
could redesignate the site mid-approach, and ours is a fixed geoposition chosen before
DOI; its profile kept thrust demand out of the DPS's forbidden 65–94 % throttle band,
and our engine throttles continuously; it managed the attitude transition to
windows-forward flight, and our pitch-over falls out of the vector law with no attitude
program.

FALL's attitude is the retained build's: hold surface retrograde above `v_switch`. With
drift nulled at the gate, retrograde and plumb coincide to within the drift residual, so
this keeps the flown behavior, and — per the handoff contract's thrust-waits-for-attitude
lesson — leaves no slew to pay at arrest ignition: the nose is already on the thrust
direction when low gate fires. The terminal chain below the gate — FALL (CSV label
changes from `COAST` to `FALL`), the low-gate schedule on `h_bot`, the arrest burn with
its lean-cosine factor, the witness lines — is retained from the current build: it flew
33 m and 34 m misses on the fixbatch flights. The `ship:bounds` altimeter was captured
once in anger on the crash flight's wrecked craft (`# bounds dh_core 0.91`, accepted by
its guard) but has never been flown through a flare; it is retained with that status.

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

The numbers, at the mass the formula is evaluated at — the *gate* mass, not the PDI
mass: the fixbatch_2 CSV shows 3.312 t at high gate (its a_cmd 5.08 m/s² at throttle
0.841 confirms thrust 20.0 kN), so `a_dec = 0.85·20.0/3.312 − 1.628 = 3.51 m/s²`,
corroborated by the flown arrest itself: v_vert −27.8 → −3.7 m/s over 7.7 s is
3.13 m/s² net at throttle 0.78–0.84. The hard wall at a 300 m gate is
`√(2·3.51·295) ≈ 45 m/s` — faster and no throttle avoids the ground. The soft wall is
hover: slower costs fuel at `g0·t`. The design targets arrival mid-corridor,
`v_gate = wall/2 ≈ 23 m/s`, so delivery and guidance errors spend margin on either
side instead of finding a wall.

The corridor decides the law's shape. With horizontal-only Klumpp over a separate
vertical schedule, the gate's descent rate is uncommanded — verifiable on arrival, not
steered to. Full-vector (3D) Klumpp makes mid-corridor `v_v` a *commanded* boundary
condition, part of `v_tgt`, delivered by the same polynomial that delivers offset and
drift. That is the argument for 3D, and the old build's failure is its supporting
history: its horizontal law and vertical schedule kept separate clocks, and the 2.5 s
of fall the schedule left was no horizon for the 21 s the lateral law needed. One
vector law cannot disagree with itself about the horizon.

## The braking law's fuel and feasibility, modelled

`powered-descent-handoff-contract.md` argued that Klumpp flown over the whole braking
phase spends cosine losses where the state is expensive. Modelled, the objection does
not hold on the efficient branch — but the branch structure it missed is real and
shapes the whole design. Method: a one-page Python integration (2-D orbital plane,
inverse-square gravity, the law closed-loop at 20 Hz, mass by the rocket equation;
craft constants from the logs — thrust 20.0 kN, Isp 319.7 s, m_pdi 3.96 t; PDI
endpoints from the plan logs — h 5982 m, ground-relative speed 563.4 m/s, γ 0; gate
300 m over the 1982 m site at 23 m/s vertical; ground lead 39,198 m; the reference is
the same integrator flying the constant-throttle gravity turn at f_cap 0.765 from the
same state to the gate altitude). Modelled, not flown:

- **Peak thrust demand over the burn is not monotone in `t_go`.** It falls steeply from
  short burns to a **dip at `t_go` ≈ 2.0·X/v₀ (141.1 s here): Δv 591.0 m/s, peak
  demand 0.803**, rises to a local maximum (1.065 near 3·X/v₀ ≈ 209 s, where the
  along-track term `4v/τ − 6X/τ²` peaks), then falls again onto a hover branch — the
  far-side root of "demand = f_cap" is a *lofted* profile (τ ≈ 492 s) that climbs
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

## `plan_doi`: the backward march and the law-sized lead

The planner answers two questions with two instruments.

**How high is PDI: the backward march.** The reference arc inverts `brake_reach`: the
same Euler equations, `dt` negated, marched *upward from the high gate* at
`f_ref = f_cap`. Backward, thrust adds speed, gravity's across-path part un-turns the
path toward horizontal, and mass increases at `f_ref·mdot`. The seed, pedantically: the
march works in datum altitude, and `h_gate` is height above the *site's terrain* — seed
position directly above the target at `tgt:terrainheight + h_gate` (1982 + 300 m here);
seed speed `v_gate`; seed direction a few degrees off vertical, because a
zero-horizontal state integrates backward to a vertical line — the reverse gravity turn
never bends away from plumb it never had. `tilt_seed` is a planner artifact, closed by
the flight's law as ordinary work; its argument is numerical, family of `pitch_tol`.
Seed mass: PDI mass less the burn's propellant. The mass iteration is a contraction:
a relative error `δm/m` in the gate-mass guess changes the marched Δv and returns as a
propellant error roughly the propellant fraction (~0.2) times smaller, so each pass
divides the error by ~5; two passes from a vis-viva first guess leave mass error below
0.1 %, which at first-order reach sensitivity (`δx/x ≈ δm/m`) is under 40 m of lead.

At each upward step the march compares its speed against the periapsis speed of the
ellipse that altitude implies — vis-viva with apoapsis taken from
`ship:orbit:semimajoraxis` (the source `brake_reach` uses, which reproduces the flown
563.7 m/s; "parking radius" is ambiguous by 5 km and moves the lead by hundreds of
metres) and periapsis at the current altitude, less the ground's eastward motion;
closed-form per step, no circularity. The crossing is PDI: that altitude is `h_pdi` —
solved, where it was a dial — and the arc's remaining pitch there, `γ*`, is reported in
the verdict; the flight's ignition `t_go` choice absorbs it, since the choice is made
from the delivered state, γ included. The march's failure paths are aborts before any
node is placed, in `plan_doi.ks`'s existing idiom (its arc-failure abort at lines
214–223): no ellipse crossing found by the step cap; the crossing altitude reaching the
apoapsis source (the ellipse family is exhausted); the backward mass exceeding what the
tanks hold (the burn does not fit the propellant). The planner also re-marches the
*forward* arc from the solved PDI state and requires it to land at the gate within
tolerance — one extra march; disagreement is a bug witness, not a tunable, and
`tilt_seed`'s reach bias must sit inside this check's tolerance or the seed needs a
real derivation.

**How far up-range: the law's own feasibility.** The lead X is *not* the march's swept
ground — that is the gravity turn's reach, a different curve. X is sized from the law
the flight actually flies: place PDI at the X where the E-guidance dip demand equals
f_cap (X ≈ 41.2 km for this craft and gate, versus the march's 39.2). The closed-form
profile evaluated at a dozen candidate leads settles it; Δv is indifferent at the m/s
level across the whole sweep, so the condition spends nothing. This is the first named
departure from `apollo-powered-descent.md`, which sizes `t_go` by a 90 %-authority
solve at a fixed aim point: that rule assumes demand falls monotonically with `t_go`,
and modelled here it does not — applied blindly it selects the hover branch.

`f_headroom` restates cleanly: no longer solver reserve for shortening an arc but
**command margin for a feedback law** — the gap between the planned dip demand (f_cap)
and `f_max` is what guidance holds for dispersion. Same constant, 0.1, still
provisional until a flight falsifies it.

## `powered_descent`: what is deleted, what replaces it

Deleted outright: the `endpoint` march and its 4000-step budget, `live_state`,
`pdi_state`, `solve_arc` and the OVERSHOOT/SHORT case machinery, `f_bracket`, the
`t_frozen`/`t_solved` clocks, `aim_distance`, and the whole yaw channel —
`braking_dir`, the gain correction, `k_yaw`, `tau_yaw`, `y_floor`. Cross-track stops
being a channel: the offset is a component of `r_tgt − r` and the law corrects it in
vector, at a Δv cost quadratic in the offset (< 1 m/s at the measured ~1.3 km PDI
offset). `tilt_max` retires with the attitude-defined handoff it served, and
`a_lat_max` with it — that constant lives in `plan_doi.ks`'s stopping-distance aim,
which the law-sized lead replaces. **No numerical integration remains in flight code**
— `a_cmd` is closed-form — so the solve-latency and moving-target defect class dies
with the march, and `config:ipu` no longer needs raising.

What replaces them:

- **Full-vector Klumpp, one law**, for the corridor argument above. Thrust demand is
  `a_cmd − g_vec`; throttle its magnitude over available thrust; attitude its
  direction; the pitch-over from near-retrograde falls out of the law
  (`apollo-powered-descent.md`, phases 3–4). `r_tgt` is the *virtual* gate (next
  bullet); `v_tgt` its matching velocity.
- **The virtual gate.** E-guidance does not taper: at any positive `t_go` it still
  commands its arrival acceleration — thrust demand 4.90 m/s², commanded `|a_end|`
  4.34 m/s² here — so exiting at a floor `τ_f` aimed at the real gate hands FALL a
  velocity error of roughly `|a_end|·τ_f`: **13.0 m/s at a 3 s floor** (8.74 at 2 s),
  against a 1–2 m/s drift bound — structurally the old design's 16 m/s
  touchdown-drift failure reborn one phase earlier. The law therefore aims at a
  virtual gate: the real gate state propagated `τ_f` forward along the profile's own
  linear acceleration, offsetting velocity as well as position. With
  `k = (a₁ − a₀)/τ` the planned profile's jerk,

  ```
  v_virt = v_gate + a₁·τ_f + ½·k·τ_f²
  p_virt = p_gate + v_gate·τ_f + ½·a₁·τ_f² + (1/6)·k·τ_f³
  ```

  At `τ_f` = 3 s the virtual target sits 68 m of profile past the real gate — the
  `v_gate·τ_f` coast term corrected by the 19.5 m `½·a₁·τ_f²` term and the jerk
  term, still ~230 m above the terrain — and carries a 13.1 m/s velocity offset.
  Braking exits at `t_go = τ_f`, at which instant the ship occupies the *real* gate
  state to the law's tracking accuracy, the `1/t_go²` divergence never entered.
  Modelled closed-loop through this construction: drift at the gate 0.08 m/s, Δv and
  peak demand unchanged. The fallback if the virtual gate misbehaves is
  exit-on-state — leave braking when altitude, offset, and drift all sit inside gate
  bounds — simpler but a race among criteria. The escalation if flights falsify the
  drift bound is the P64 extension point, not a bigger floor.
- **`t_go` at ignition, from the dip.** The old `t_gate` closure solved a free fall's
  intersection with the arrest schedule and does not apply to a powered phase. At
  ignition the program evaluates the planned profile's peak demand — closed form from
  the linear profile's endpoints, exact under constant gravity and approximate here
  only through the ~11° the arc subtends, so "no integration in flight" stays true —
  over a bracket of candidate `t_go`, and takes the minimum: the dip. Feasible means
  the dip demand fits under `f_max` with margin; the planner's lead puts the nominal
  dip at f_cap, and the ±1.5 km window moves it by ∓0.018 per km — ∓0.027 across the
  window, worst case ~0.79, inside `f_max` 0.85. Implementation is cheaper than a
  minimum-search: the dip is exactly the crossover of the two endpoint demands — a
  minimax — so the flight root-finds `d_ign = d_gate` rather than searching, and
  `max(d₀, d₁)` is unimodal on the bracket (validated numerically, not asserted).
  The bracket is `[1.6, 2.6]·X/v₀`, which straddles the
  modelled dip (2.0·X/v₀) and excludes both the short-burn wall and the hover branch
  beyond 3·X/v₀; the demand curve's non-monotonicity is established numerically for
  this geometry, not proved, so the bracket is load-bearing and stated in the code. In
  flight `t_go` decrements by clock to `τ_f` — the second named departure from the
  source, which re-solves every ~10 s; re-anchoring the schedule re-admits the
  moving-target class, and the first flight's demand trace decides whether
  decrement-only accumulates error worth shedding.
- **Feasibility at ignition, an abort that exists.** If the dip demand does not fit,
  the ship is still at the periapsis of a stable, quicksave-able ellipse: declining to
  ignite *is* the abort, checked while aborting is free. This replaces the retrograde
  design's two-conditions-one-knob ordering, most of which was design without code.

## High gate placement

`h_gate` sets the corridor; `v_gate = wall/2 ≈ 23 m/s` follows from it (mid-corridor
by construction — symmetric margin, craft- and body-aware through `a_dec`, free of
both as a rule). The altitude's own argument:

- **Contain the arrest.** From a 300 m gate entered at 23 m/s, low gate falls near
  150 m (`h_lg = h_pad + (v_gate² + 2·g0·(h_gate − h_pad))/(2·(a_dec + g0))` =
  149.9 m at gate mass) — the arrest fits with `h_gate/h_lg ≈ 2.0`. The proposed
  constant is that ratio, `k_gate ≈ 2.0`, dimensionless; the flown anchor is the
  fixbatch_2 flight's 283 m gate with its arrest firing at radar 127 (the first
  ARREST row) — ratio 2.23, and the formula predicts 131 there, a 4 m error. A
  flight whose arrest peaks over `f_max` falsifies it.
- **Bound what drift costs.** "No horizontal channel below the gate" would be false:
  the retained arrest thrusts anti-velocity, and the fixbatch flights *measured* it
  closing 128 m of offset to 33 and 9.9 m/s of drift to 0.2. So gate drift is not a
  sentence, it is a cost: ~3–4 m of residual miss per m/s of gate drift as flown
  (33 m from 9.9 m/s), against ~15 m per m/s if nothing corrected. The law's 1–2 m/s
  gate-drift bound keeps the drift term in single-digit metres either way; the
  fixbatch numbers say even a tenfold violation lands inside the bar.
- **Terrain enters here and only here.** `h_gate` is height above the site's terrain,
  the design's single clearance input; what the site demands adds to it directly.
  Fuel does not argue against it: the whole gate costs ~`√(2·g0·h_gate)` ≈ 31 m/s of
  arrest speed at 300 m, and moving it a hundred metres moves the budget by
  single-digit m/s.

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
  `t_go` against the planner's reference for the cross-check, and `τ_f` keeps the
  divergence out of the loop.

**High gate.** Two arrival checks, witnessed on the `# high gate` line before FALL
commits: the corridor inequality on total surface speed with its margin fraction, and
the offset/drift residuals against the law's bounds. A wall-side or saturated arrival
is visible *here*, with the engine still lit, not at the pad.

**FALL and arrest.** The corridor proof is the certificate: inside it at the gate, low
gate always fires above the pad, and the arrest's 0.85 ceiling leaves reserve that
covers ignition lag and discretization. The flown set is honest: the arrest chain
landed 33 m and 34 m twice; the bounds altimeter has been captured and guard-accepted
on a wrecked craft but never flown through a flare; and the fixbatch tip-over is *not
fully explained* by the radar-datum error — slope and leg geometry are entangled in
it, and the settle trace exists to separate them.

## Witnesses and testable signatures

The recorder keeps its shape; BRAKE columns change meaning with the law. New or
changed: `t_go` (the guidance clock), thrust demand and its fraction of available (the
saturation observable), achieved acceleration beside commanded (the model-error
observable), ZEM magnitude, and — instrumentation for the certification gap named in
Open — the running *minimum radar clearance* over the braking arc, logged so the first
flights measure what no rule yet covers. The pe-longitude witness line from
`node-delivery-window.md` logs at warp-out. Predicted signatures for the first flight,
column and value, modelled where marked:

- Header: ignition `t_go` within ~10 s of the planner's reference; the pe witness line
  measures delivery independently of everything below.
- BRAKE rows (modelled, at the adopted X = 41.2 km lead): throttle a shallow U —
  0.766 at ignition, sagging to 0.696 mid-burn, returning to 0.754 approaching the
  gate — never above `f_max`;
  commanded-vs-achieved ratio within a few percent of 1; `facing_err` single digits
  throughout (the burn is continuous — no slew-wait rows).
- `# high gate` line: offset ≤ 30 m, drift ≤ 1–2 m/s, total speed within ±10 m/s of
  `v_gate` 23 with the corridor fraction `v²/(2·a_dec·(h_bot − h_pad))` ≤ 0.5,
  altitude within 20 m of `h_gate`.
- Touchdown: miss ≤ 40 m — the retained terminal measured 33–34 m *with* 9.9 m/s of
  gate drift, and the law promises ≤ 2 — and dv_rem within ~20 m/s of the fixbatch
  flights' 300 (anchored there, not to the 274 of the overshooting pair, because
  fixbatch flew the retained terminal on-plan; the modelled braking wash of
  591-vs-~592 is what closes the loop if it holds). Settle tilt < 5° is the bounds
  altimeter's first flare-tested flight.

## Staged rollout

One instrumented change per flight, with one marked exception:

0. *No flight.* New planner checked offline: verdict against the three logged plans,
   forward/backward self-check, solved `h_pdi` against the 5982 m dial, law-sized lead
   against the modelled 41.2 km.
1. The pe-longitude witness line — one log line, rides whatever flies next. Whether
   the current retrograde build ever flies again is Schuyler's call; the
   recommendation is no — effort goes to the redesign.
2. First redesign flight: new planner, Klumpp braking, retained terminal. This flight
   carries **two** unflown elements — the braking law and the bounds altimeter's
   flare — a deliberate exception to one-change-per-flight: they occupy disjoint
   phases with disjoint witnesses (BRAKE columns vs the `# bounds`/`# touchdown`
   pair), and the altimeter has no safe vehicle to fly on alone. Schuyler's call.
3. The gentle-executor experiment (`node-delivery-window.md`) runs on its own thread
   once the witness line has data.

## Ripples

- `powered-descent-invariants.md`: rewritten. The one-parameter-family premise migrates
  to the planner's reference arc; the flight's invariants become the law, the gate
  state, the corridor, and ignition feasibility. The two-conditions-one-knob section
  retires with the knob.
- `powered-descent-handoff-contract.md`: the contract principle and the standing
  lessons survive; `d_handoff`, `a_eff`, the attitude handoff, the latch/gate/fence
  machinery, and the geometric-mean lean shaping all die with free-fall trim. Its
  cosine-loss argument against Klumpp braking is answered by the modelled comparison
  above and should be restated against those numbers. Rewrite after shipping.
- `retrograde-terminal-findings.md`: must be extended first — the fixbatch pair and
  the crash flight currently have no findings register, and this document leans on
  all three as witnesses.
- `doi-planner.md`: placement loop and fuel-lever finding unchanged; the `pdi_height`
  dial paragraph replaced by the backward solve; `f_headroom` restated as guidance
  margin; the lead's source moves from the march's reach to the law's dip condition.
  **The chord certificate does not survive as-is**: its concavity argument covers
  gravity-turn arcs, and Klumpp's flown path is not one. Its replacement is deferred
  terrain work (`terrain-certification.md`).
- `apollo-powered-descent.md`, `klumpp-guidance-derivation.md`: normative sources for
  the law; the departures from them are named where they occur (`t_go` by
  dip-minimization, not the 90 %-authority solve, whose monotonicity assumption fails
  here; decrement-only `t_go`, not the ~10 s re-solve; saturation response left as
  policy where the source prescribes abort).
- `BOOK-PLAN.md`: the book follows the code once this ships; the design history is
  chapter material, not register material.

## Open

- **Braking-arc terrain clearance is unruled** between PDI and the gate until
  `terrain-certification.md`'s work happens — the chord certificate retired with the
  gravity turn and nothing replaces it yet. One observation for that future work,
  recorded as a comment and deliberately not built into machinery: the modelled
  E-guidance path clears the straight PDI→gate chord over its whole length (minimum
  +0.5 m at the PDI end, +900 to +1,600 m mid-arc), so a single offline march per
  placement could certify braking-arc clearance the way the retired certificate did.
  For now the first flights instrument the gap (the minimum-clearance witness)
  without pretending to rule it; `terrain-certification.md` remains the pointer for
  the general problem.
- `k_gate ≈ 2.0` is anchored on one flown gate altitude; its argument is partial until
  the new law's arrival scatter is measured.
- The demand curve's dip structure is established numerically for this craft and
  geometry, not analytically; the `t_go` bracket `[1.6, 2.6]·X/v₀` is load-bearing.
  A derivation of the dip and its bounds would retire the bracket.
- `τ_f`, the `t_go` floor and virtual-gate offset, is family-of-`t_settle` and
  underived.
- `tilt_seed`, the backward march's degeneracy seed, is family-of-`pitch_tol` and
  underived; the forward/backward self-check bounds its consequence, not its value.
- Saturation response in braking — ride it or abort to orbit — is policy, Schuyler's
  call, and a named departure from the source's abort prescription.
- `v_switch` (5 m/s, plumb-below speed in the retained arrest) remains chosen, not
  derived, carried from the current build.
- The stage-2 two-unflown-elements exception — Schuyler's call.
- The fixbatch tip-over's full mechanism (datum error vs slope vs leg geometry) — the
  settle trace and touchdown witness are the instruments when it next flies.
- Whether the backward march's `h_pdi` — a gravity-turn altitude — is the right
  altitude for an E-guidance flight is a structural approximation: the two profiles'
  altitude histories agree at the few-percent level on the efficient branch
  (modelled), and the header's `t_go`-vs-reference cross-check witnesses it per
  flight.
