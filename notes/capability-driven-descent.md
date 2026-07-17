# Capability-driven descent: working backwards from the suicide burn

*Design note, 2026-07-17. Successor to `targeting-redesign-checkpoint.md` — that note
inverted the planning dependency chain (BRAKE duration from the horizontal axis at design
throttle, vertical rides gravity) and fixed the BRAKE-loafs-at-21% half of the high-TWR
inefficiency. This note goes after the other half: the APPROACH hang and the 150 m
terminal tail. It reframes the whole profile around the fuel-optimal suicide burn and lets
the gates fall out of the craft's own thrust, instead of importing Apollo's crewed-mission
gate states.*

**Status: soft commitments only.** This is the reasoning and the architecture we agreed;
the exact trajectory model, the gate placement math, and the code blocks are to be worked
step-by-step in later sessions. Nothing here is applied to
`reference/original/powered_landing.ks`, and — unlike the checkpoint — this note does not
carry finished code blocks. It carries what we've decided and, just as importantly, what we
have deliberately left open.

## The idea in one paragraph

The fuel-optimal airless descent is essentially bang-bang: coast as long as possible, then
one hard braking arc that arrives at the surface at low speed. Apollo declined that
hoverslam for four reasons — crew visibility, abort coverage, human-in-the-loop, blind
terrain — and we share none of the first three. The only one that survives is terrain
uncertainty, and that is exactly the one our *surveyed target* removes, but only over the
site. So the suicide burn is available to us where it wasn't to Apollo. The move: design the
brake backwards from a margin-throttle burn that arrives at the surface, let low gate and
high gate be points on *that* trajectory rather than scaled-Apollo states, and clamp only
one thing by hand — how close to the terrain the approach corridor is allowed to run.

## Diagnosis: high TWR isn't the problem, the fixed gates are

High TWR is not inefficient here. Gravity loss is `∫ (g opposing thrust) dt`; the later,
harder, and shorter the brake, the smaller that integral, so efficiency *should improve*
with TWR. The current design inverts that because its gates are fixed and refuse the thrust.

The telemetry is unambiguous. Flight 7 (TWR-34 craft, 25 km circular, h_pdi 3000, 0 m miss)
spent 244.1 m/s: DOI+coast 11.9, **BRAKE 100.2, APPROACH 107.6**, TERMINAL 24.4. Flight 6
read the mechanism directly: BRAKE flew at **21% throttle** the whole way (its duration
pinned by the vertical geometry of a fixed 2 km high gate, not by the engine), and ~45 of
APPROACH's 109 m/s was **pure gravity hang** — 3.1 km covered at ~35 m/s while the engine
held against gravity. For that craft the fuel-optimal brake is ~10 s and drops a handful of
metres; the flown design spent ~135 s of engine-against-gravity doing a ~10–20 s job. The
~50–60 m/s gap between the flown 244 and the impulsive floor *is* that surplus engine time.

The checkpoint's horizontal-first redesign already recovers the BRAKE half (predicted ~10 s
at design throttle for this craft). What it explicitly defers — "hg_height 2000 is where the
remaining ~45 m/s of hang lives" — is the APPROACH tail. That tail is this note's target.

## Working backwards: the construction

From the ground up, the way the profile is actually designed:

1. **Touchdown / terminal (keep).** The P66 rate-of-descent controller costs a few m/s and
   is the robust part. The suicide burn does not target the ground; it targets **low gate**,
   the terminal handoff. Keep the controller.

2. **Low gate = the terminal handoff — now parameterized and lowered.** 150 m is Apollo's
   500 ft carried over unscaled, and its stated justification ("close enough that `alt:radar`
   is ground truth") does not hold as a *floor*: over the surveyed site radar is truth at any
   height. The real floor is the terminal phase's need for room to do its two jobs — flare
   the descent rate to a soft touchdown, and null any residual horizontal drift the guided
   leg hands it — plus a buffer to be clear of the law's divergent gains near `t_go → 0`.
   Both scale with how fast and how clean the handoff is. See "Low gate" below; the decision
   is to make it a parameter, default it well under 150, and walk it down with telemetry.

3. **The brake, integrated backwards from low gate.** At margin throttle `f·a_max`, run the
   checkpoint's own vertical model — `a_felt = g − v²/r`, "gravity first pays the centripetal
   cost of following the surface's curve; only the remainder descends the ship" — but to a
   *different endpoint*: horizontal speed `v_pe → ~0` at low gate, not `→ 60` at a high gate.
   The integral then yields the brake duration, the altitude it consumes, the downrange it
   covers (→ the lead angle), and the vertical rate it arrives with.

4. **High gate = the top of that brake, clamped by clearance.** Its velocity and height
   become *outputs* of the backward integration, not the scaled-Apollo `60 m/s @ 2 km`. The
   one input we keep by hand is the terrain-clearance floor `H_clear`: high gate's altitude —
   and, more precisely, the whole arc's minimum altitude — must stay `≥ terrain + H_clear`.

5. **The geometric gift.** Off a low periapsis, `v_pe > v_circular`, so `a_felt < 0` early:
   the craft is *lifted*, not falling, and the free trajectory rises just after PDI before it
   falls as the brake bleeds speed below circular. So the trajectory is **highest up-range,
   lowest late — directly over the target.** The clearance floor therefore protects the
   *up-range corridor*, where terrain is only modeled and the craft is naturally high; the
   low, terrain-trusting part is confined to over the surveyed site, where radar is truth.
   This is the suicide-burn version of the robustness Apollo bought with its steep final
   approach, and it falls out of the geometry instead of being scripted.

## The open question at the center: what does the brake actually fly to?

This is the crux, and the main thing to work step-by-step — recorded here so we don't lose
what the checkpoint's integral already tells us.

Run the checkpoint's forward model to the `v_pe → 0` endpoint and the arrival descent rate
at low gate is `vv_lg = T·[g − v_pe²/(3r)]` with `T = v_pe/a_h`. Evaluate it on the two
reference craft (Minmus, v_pe ≈ 170 m/s):

- **TWR-34 test craft** (`a_h ≈ 11.2`): `T ≈ 15 s`, `vv_lg ≈ 5 m/s`. Braking horizontally
  all the way to zero hands terminal a ~5 m/s descent — a *soft handoff with no APPROACH leg
  at all.* The single brake IS the whole powered descent; the phase eating 107 m/s today
  simply disappears.
- **TWR-2 design craft** (`a_h ≈ 0.51`): `T ≈ 333 s`, `vv_lg ≈ 114 m/s`. Braking horizontally
  to zero accumulates a descent rate no terminal controller can absorb. The model is telling
  us the truth: for a long brake, "hold vertical, brake horizontal to zero" is *not* the
  efficient path — thrust spent building descent rate must be spent again removing it. The
  efficient brake there is genuinely retrograde (thrust cancels the *velocity vector*, which
  rotates from near-horizontal toward down as speed bleeds off), and a managed vertical leg
  earns its keep.

So the architecture is **TWR-adaptive, and the same guidance law with derived gate states
spans both ends**: high TWR collapses toward a single brake straight to a low, slow gate;
low TWR keeps high gate higher/earlier with a managed descent below it. Both the clearance
floor and the accumulated descent rate push high gate *up* for low-TWR craft — consistently.
We don't choose which regime we're in; we let the brake integral report `vv_lg` and place
high gate accordingly.

The reassurance under all of this: **the guidance law flies the real trajectory; planning's
only jobs are to place PDI/lead and hand the law consistent gate states.** The planning model
does not have to be the exact fuel-optimal arc — it has to put high gate on *a* feasible
efficient brake and keep the boundary conditions consistent (lead, speed, duration
describing the same burn — the hard-won lead-consistency principle). The law absorbs the
rest, as it has since flight 1. So the step-by-step work is: decide how planning places high
gate on the brake (candidate: extend the checkpoint decomposition, switch to a
retrograde/velocity-vector model when `vv_lg` exceeds a soft-handoff threshold), and verify
by flight — never by argument.

## Terrain clearance: the efficiency knob, made to show its price

`H_clear` is the one hand-set input, and it is the efficiency-vs-safety trade in a single
number. Loose `H_clear` → a low, late hoverslam: minimal ΔV, terrain-trusting. Tight
`H_clear` → the brake must fly higher and earlier, costing ΔV. Two commitments:

- The clamp is on the **whole arc's minimum altitude**, not just high gate — for a hard
  brake the low point is late (near low gate, over surveyed ground), and the existing sag
  machinery already computes where. Raise `h_pdi` until the sagging arc clears `H_clear`.
- The planning block should **print the ΔV that the chosen clearance costs** against the
  unclamped optimum. That turns `clearance = 2000` from a magic number into a measured
  decision and fits the book's ledger habit — "measure, don't assume," applied to our own
  safety margin.

## Low gate: parameterize and walk it down

The decision from this session, concretely:

- **Make low-gate height a parameter**, sibling to `H_clear`. Default it well under 150 —
  ~50 m is defensible as a start — with ~20–30 m as the aggressive floor.
- **The floor is set by the terminal phase, not by radar truth.** Terminal needs room to
  taper toward its 2 m/s touchdown floor (`radar/10`, so ~20–30 m), to null residual drift
  (its steering tips only ~6°/m/s, so drift wants altitude to walk out), and to stay clear of
  the guided law's divergent gains near `t_go → 0` (at 50 m and ~5 m/s that's ~10 s in hand;
  at 20 m, ~4 s — tight).
- **The payoff is the same "don't hang" logic.** Terminal is a near-hover — throttle sits at
  the gravity feedforward the whole way down, so every metre is gravity loss, and it scales
  with body g (the same 150 m costs ~3× as much on the Mun as on Minmus). Flight 7's TERMINAL
  was 24.4 of 244. Every metre lowered is a metre the *efficient brake* carries instead.
- **It couples to the arrival rate** — a lower gate wants a slightly slower, cleaner arrival
  so the flare and drift-null fit the reduced room. Lower the gate and let the arrival rate
  come down with it; don't move it in isolation.
- **Walk it down with telemetry, don't pick it by argument.** Flight 7 now logs terminal
  rows, so touchdown `v_vert` and miss distance are the scores. Drop the gate a step per
  flight; stop when either degrades. That yields a real number instead of Apollo's inherited
  one.

## One leg or two

A pure suicide burn is one continuous brake. But the quadratic law assumes acceleration
linear-in-time over the leg, and a full brake from orbital speed is long enough that the
approximation degrades and the aim-point geometry wants re-targeting partway — the real
reason Apollo split P63/P64, and why flight 1's single gentle leg worked but reversed.

**Recommendation: keep two guided legs, but place high gate as a point *on the optimal
brake*, not as a chosen state.** Design the efficient brake backwards from low gate, then
split it where the clearance floor binds (or at a fixed fraction of `v_pe`). Both legs fly
pieces of the *same* efficient decel; neither hangs; high gate's velocity is whatever the
brake has there. For a high-TWR craft the split may collapse to nearly nothing (the second
leg is a few seconds) — which is correct, and is where the APPROACH fuel goes to die.

## Margin

"Planned margin" is `f < 1`, and it does double duty. Reserving `(1 − f)` of thrust is both
the control authority the law spends absorbing error *and* the ignition-timing cushion — if
the brake lights late or the thrust runs short, the law calls on the reserve to catch up. So
do **not** design a literal `f = 1` hoverslam; the campaign's whole thesis is that the law's
error-absorption is the robustness, and `f = 1` leaves it nothing to absorb with. Start at
`f ≈ 0.85`; it's a knob we sweep in flight if the law shows it has room. The saturation guard
and attitude gate stay exactly as they are.

## What changes vs the checkpoint (soft)

Held together so the delta is legible; all soft, all subject to the step-by-step work.

- **Kept from the checkpoint:** `h_pdi` as an output (`terrain + clearance`, floored);
  BRAKE pinned to design throttle, not geometry; exact endpoint feasibility checks; the
  `a_felt = g − v²/r` vertical model; the lead-consistency principle; DOI/coast/eccentricity
  feedback/kepler fix all untouched.
- **New here:** the brake's endpoint is low gate (`v → ~0`), not a 60 m/s high gate; high
  gate becomes a waypoint on that brake (velocity and height derived), clamped only by
  `H_clear` on the arc minimum; APPROACH as a distinct hang-phase collapses (fully for high
  TWR, partially for low); low-gate height becomes a parameter defaulted well under 150; the
  clearance ΔV cost is reported.
- **Open (the step-by-step work):** how planning places high gate on the brake (extend the
  decomposition vs. switch to a retrograde/velocity-vector model past a `vv_lg` threshold);
  where exactly the two-leg split falls; the low-gate/arrival-rate coupling law; the
  `vv_lg` soft-handoff threshold itself.

## Decisions / forks, with recommendations

- One leg or two? → **Two, high gate as a point on the optimal brake.** Robust; small delta.
- Design margin `f`? → **0.85** to start; sweep if the law has room.
- Low gate fixed or output? → **Parameter**, defaulted ~50 m, walked down with telemetry.
- Keep the P66 rate controller? → **Keep it.** The robust part; costs a few m/s.

## Process (unchanged, hard-won)

No claim about flight behavior without telemetry; predictions only as testable signatures.
One instrumented change per flight (this whole reframe is one planning-subsystem change).
Blocks presented and applied only on explicit approval, block-by-block. Comments carry
timeless principles, not war stories.

**Predicted signatures for the first flight of this design** (TWR-34 test craft, stated as
columns to check, not promised outcomes): BRAKE carries to low gate at ~5 m/s descent with
horizontal `v_to_site → ~0`; the APPROACH ledger collapses from flight 7's 107.6 m/s toward
a few-second remnant or nothing; TERMINAL shrinks with the lowered gate; total ΔV moves off
244 toward the ~180–200 impulsive floor; the arc's logged minimum altitude up-range stays
`≥ terrain + H_clear`; touchdown `v_vert` and miss distance hold at flight-7 quality.
