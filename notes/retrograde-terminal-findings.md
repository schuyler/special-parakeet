# Descent flights: what they measured

*The findings register for flights of `reference/original/powered_descent.ks`, across the
builds that have flown it. Registers it bears on: `powered-descent-invariants.md` (the
flight program), `doi-planner.md` (the placement), `klumpp-descent-redesign.md` (the
design these flights argued for and then tested), `node-delivery-window.md` (the delivery
scatter they measured). Witnesses live beside the scripts in `reference/original/` and are
named per flight below. They sit outside git — `*.csv` and `*.log` are ignored — so an
unarchived `flight_log.csv` or `doi_plan.log` survives only until the next run overwrites
it: copy it to a dated name before flying again, or the flight is a memory.*

## The flights

| flight | witness | high gate | miss | Δv spent |
|---|---|---|---|---|
| Klumpp terminal | `flight_log_20260726_klumpp_terminal.csv` | not logged | 293 m | 626.2 m/s |
| retrograde terminal | `flight_log_20260726_retrograde_terminal.csv` | radar 1024, drift 10.2 | 605 m | 657.7 |
| retrograde terminal 2 | `flight_log_20260726_retrograde_terminal_2.csv` | radar 1023, drift 10.4 | 612 m | 657.5 |
| fixbatch 1 | `flight_log_20260726_fixbatch_1.csv` | radar 282, drift 9.9, offset 128 | 34 m | 631.5 |
| fixbatch 2 | `flight_log_20260726_fixbatch_2_touchdown.csv` | radar 283, drift 9.9, offset 128 | 33 m | 631.2 |
| crash | lost | never reached | flew into the ground | — |
| Klumpp redesign, first | lost | — | 1 m | — |
| Klumpp redesign | `flight_log_20260727_klumpp.csv` | radar 1498, drift 1.5, offset 3 | 15 m | 702.2 |

The first seven rows are one craft (thrust 19.99 kN, Isp 319.7 s, 3.96 t at PDI) on one
plan family (`doi_plan_20260726_baseline.log`: PDI 5982 m, 563.7 m/s, 39.2 km of reach at
f_cap 0.765). The last is a different craft and a different site, so its Δv is not
comparable with the rest; what carries across is the arrival, not the budget.

## The retrograde terminal build

The build the 2026-07-26 flights flew, recorded here because its measurements are what
the current design was argued from. It is not in the tree.

Terminal was the braking law continued to the ground: hold surface retrograde through a
coast from high gate, ignite the arrest burn when f_max could just bring the total speed
to rest at the pad, throttle to carry the descent rate to v_floor, plumb below walking
speed. `endpoint` integrated the whole descent — brake, coast, arrest — to touchdown, and
`solve_f` aimed the predicted touchdown point at the site.

**The old terminal's defect, confirmed (klumpp CSV).** High gate arrived 2.5 s of fall
above suicide-burn ignition against the ~21 s the lateral guidance needed, so the ship
touched down still carrying 16 m/s of horizontal drift; miss 293 m (direction not
logged). The free-fall window that design assumed was never produced by anything.

**The retrograde terminal worked (both retro CSVs).** Drift 10.2–10.4 m/s at high gate
decayed to 0 at touchdown; vertical speed −2.7 m/s at the pad; upright; no slew before
ignition; throttle reserve untouched; the offset moved only ~17 m between high gate and
touchdown. The fast-horizontal-touchdown failure mode is gone by construction.

**But the miss grew: 605 m, then 612 m, and ~31 m/s more Δv than the old flight.** The
lat/lng witness lines (second retro flight) place the lander ~593 m past the site along
an eastward track (pe at lng −94.92 → site at −83.68, inclination ~1.2°). Refined against
the offline reproduction: **along-track ≈ 380 ± 50 m long, cross-range ≈ 440–500 m**
(measured; the split is limited by the cross column's normal rotating onto the
along-track axis once the drift verticalises).

### The miss budget (offline reproduction, Python re-implementation of `endpoint`)

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
2. **`solve_f`'s objective re-read `dist_to_site()` on every evaluation** while the ship
   closed at ~536 m/s, so bisection chased a target that moved 9.4 km during 14
   evaluations. The returned 0.85 was a truncation artifact — the lower bracket walked up
   to the ceiling; at the state the solve ended on, no root existed. The 116 s pin and
   the bracket-failure spam follow.

The 0.3449 seed was a *legitimate* root of that model at the pre-PDI state — the older
seed of 0.574 was inflated by the removed vh²/(2·a_eff) term, which at sub-high-gate
throttles added 17–170 km of fictitious stopping distance and was accidentally closer to
the correct PDI answer. The seed's real defect was semantic: it answered "what throttle if
I light now" from a state 60 s and 33 km up-range of ignition.

**`plan_doi` is exonerated.** From the true periapsis state the root was 0.755 against
the planner's f_cap 0.765 with 4.5 km of shortening margin — placement exactly per
`doi-planner.md`'s f_headroom design. The hypothesis that the plan was infeasible is
overturned; no placement change is indicated by these flights.

Secondary model biases, both real but non-causal that flight (the throttle was
saturated): the sim booked the high-gate horizontal speed as along-track reach through
coast and arrest (+115–200 m of phantom prediction; the flight's post-gate along-track
gain was ~40 m, the residual being ~90 % cross-track), and the aim compared in-plane
reach against the full great-circle distance including cross-range — negligible early,
dominant after t ≈ 100 s when the whole remaining distance is sideways, which is what
produced the harmful 0.425 relaxation.

### The yaw law's gain error (first retro CSV, separate offline analysis)

The plane-closing bias underdelivered by a factor of ≈ 2, and it is a geometric gain
error, not a lag. n was defined from the velocity, so v·n ≡ 0 identically: the offset y
closes only through the ground track's azimuth turning, ẏ = −Ω·X with X the downrange
distance remaining. The commanded pretend cross-speed y/τ buys Ω = a·(y/τ)/(|v|·v_h),
so the achieved closure is y/τ · G with G = a·X/(|v|·v_h) ≈ 0.48–0.58 across the burn
(τ_eff = τ·|v|·v_h/(a·X): predicted 93.7 s at ignition, fitted 93.6 s — a clean
first-order exponential at the wrong constant). G ≈ 0.5 is scale-invariant (X is itself
a stopping distance ~v²/2a), so the law left the same *fraction* everywhere — which is
why it looked fine on Minmus, where half of a small offset is small. Closure dies once X
collapses (below ~100 m of downrange the channel is dead). Δv cost is quadratic in the
PDI offset: < 1 m/s at that flight's 1184 m offset. The lesson the current design takes
from it: cross-track handled as its own channel needs its own gain derivation, and a
vector law that carries the offset as a component of `r_tgt − r` has none to get wrong.

## The fixbatch pair: the terminal chain that the current build kept

Two flights of the same plan (`doi_plan_20260726_fixbatch.log`, eta 284 s), the second
adding the settle trace. Both delivered the same braking arc: ignition throttle 0.7666,
t_go 158.7 and 158.1, 39.2 km from the site — within 75 m of plan, the tightest delivery
of the three measured (`node-delivery-window.md`).

**The arrest chain works, and it is the part the current build retains.** High gate at
radar 282–283 m carrying 9.9 m/s of drift and 128 m of offset; the arrest thrust
anti-velocity closed both, to 0.2 m/s of drift and 33–34 m of miss at contact. That is
the measurement behind the current design's drift pricing: **~3–4 m of residual miss per
m/s of gate drift**, against ~15 m per m/s if nothing corrected. Descent rate at
contact −3.1 m/s; the low-gate schedule fired at radar 126–127 against 283 at the gate,
still carrying 8.7 m/s of drift and −27.8 m/s of descent rate.
Δv spent 631.2 and 631.5 m/s, against the Klumpp-terminal flight's 626.2 on the same
braking arc — at equal braking, the terminal law is a few-m/s line item.

**Both flights tipped over, and the mechanism is not settled.** Contact was upright and
gentle — fixbatch 2 touched down at 4.9° of tilt, 0.2 m/s of drift, −3.1 m/s vertical —
and the settle trace then walks the craft over in five seconds: tilt 4.9 → 24.5 → 46.2 →
66.7 → 93.9 → 97°, angular rate 0.15 → 0.559 → 0.371 → 0.399 → 0.555 → 0.113 rad/s. Both
flights ended at tilt 96.8. The radar-datum error is one candidate — the schedule flew
`alt:radar`, which reads from the core, so the flare planned itself into the ground by
the craft's own core height — but slope and leg geometry are entangled in it, and the
trace cannot separate them alone. The current build's `ship:bounds` altimeter answers the
datum half only.

## The crash flight: witness lost

The retrograde build flew into the ground still in its braking phase. **Its CSV and its
plan log are gone** — both were the undated `flight_log.csv` / `doi_plan.log`, and later
runs wrote over them; neither was ever in git. What survives is second-hand and is
recorded in the registers that read them while they existed: +1,314 m of along-track
delivery error (`node-delivery-window.md`, from the header's `dist 40,555` against the
plan's 39,241), which pulled the throttle solve down to 0.739, whose arc put high gate
4.4 s of fall above touchdown (header `t_go 151`, `t_brake 146.6`), and a
`# bounds dh_core 0.91` line captured on the wrecked craft. `doi_plan_20260726_baseline.log`
stands in for the placement: the three 2026-07-26 plans were byte-identical but for the
node's eta.

The flight measured one thing cleanly, and it is the design input the current build
carries: the delivery window points a throttle-answered arc at the ground, because
throttle is also the knob that sets gate altitude. There was no feasibility abort to
refuse it.

## The Klumpp redesign flights

The current build: full-vector Klumpp braking to a virtual gate, the retained terminal
chain below it (`klumpp-descent-redesign.md`, `powered-descent-invariants.md`). It has
flown repeatedly and it lands.

**The first flight landed 1 m from the target.** Its witness was overwritten before it
was archived; the number is all that survives of it.

**The 2026-07-27 flight is the surviving witness** (`flight_log_20260727_klumpp.csv`,
`doi_plan_20260727_klumpp.log`), and it is a different craft and site from every flight
above: 58 kN, 4.25 t at the gate, a_dec ~10 m/s², landing beside a parked ship in the
Midlands at h_gate **1500 m**. Plan: h_pdi 4004 m solved, lead X 18,128 m law-sized,
dip demand 0.765 at t_go 62 s. Delivered: h_pdi 4011 m, 568.3 m/s, 18,042 m from the site
at ignition — 86 m short of the planned lead — t_go 62.9 s, dip demand 0.733.

- **Braking held the reserve.** Throttle ran a shallow U — 0.735 at ignition, sagging to
  0.701 mid-burn, back to 0.761 approaching the gate — never near f_max 0.85, and
  `sat_s` finished at 0. Achieved-over-commanded acceleration sat at 1.00 ± 0.01 from the
  third row on; the first two rows (0.179, 0.961) are the one-time slew off surface
  retrograde onto the guidance vector, with `facing_err` 20.1° then 10.7° and under 4°
  for the rest of the burn. ZEM fell monotonically 19,366 → 85 m.
- **The gate arrived where it was aimed.** Radar 1498 against h_gate 1500; speed 88.3
  against v_gate 88.2; drift 1.5 m/s; offset 3 m; corridor fraction 0.25 against the
  design's 0.25 and the wall's 1.0.
- **The terminal chain finished it.** FALL from 1496 m; the arrest fired between the
  536 m and 436 m radar rows, holding a net 10.6 m/s² of vertical deceleration; contact
  at −2.0 m/s, 0.1 m/s of drift, 0.5° of tilt, and no tip-over. Miss 15 m.
- **The bounds altimeter has now flown through a flare.** `# bounds dh_core 1.52`,
  accepted by the guard, and `h_bot` read 0.2 m at contact against `alt:radar` 5.5 —
  the box tracks attitude, and 1.52 is what it read while the craft was still leaning
  105.8° off plumb at FALL entry, not the plumb core height the touchdown pair measures.
  The capture feeds the accept/reject guard only; the flown height is read live.
- **The braking arc came within 223 m of the ground.** `clear_min` bottomed at 223 m
  three seconds after ignition — with the gate 1500 m over the site, the tightest
  clearance of the whole descent was up-range, over terrain nothing rules on
  (`terrain-certification.md`). The arc cleared it. Nothing in the plan knew it would.

## Open

- The fixbatch tip-over's full mechanism (datum error vs slope vs leg geometry). The
  settle trace and the touchdown witness are the instruments; the redesign flights have
  not reproduced it.
- The Klumpp-terminal flight's miss direction was never logged and is unknown.
- Δv across builds is measured on one craft and one plan family. The redesign flights fly
  a different craft, so the braking law's fuel is still compared against a model
  (`klumpp-descent-redesign.md`), not against a flight.
