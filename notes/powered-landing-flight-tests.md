# powered_landing.ks flight-test campaign: state of the debugging

*Handoff note, 2026-07-16. `reference/original/powered_landing.ks` implements the design
in `apollo-powered-descent.md` + `klumpp-guidance-derivation.md` and has flown five
instrumented-or-logged test flights on Minmus. It lands (Kerbals survive; the guidance
law is remarkably hard to kill) but placement and Δv are still wrong, and the trail ends
at a specific, verifiable suspect in `core/kepler.ks`. Read this before touching
anything.*

All flights: Minmus, ~23 km near-circular-but-not-circular parking orbit (~25×21 km),
target (0, 0), TWR-34 test craft (far above the design point of local TWR ~2 — several
failures below only manifest on overpowered craft).

## What the script is

Apollo-style targeted descent: DOI burn → coast to a periapsis ("PDI") placed a lead
angle up-range of the site → BRAKE and APPROACH phases flying Klumpp's quadratic
guidance law (`a_cmd = 6R/t² − (4v+2v_tgt)/t`) to two "gates" → vertical rate-of-descent
TERMINAL phase. `t_go` for each gate is pinned by a "closure" (the seventh scalar
condition the six endpoint equations leave free). Flight telemetry goes to
`flight_log.csv` (one row/s from the gate flyer; overwritten each launch — archive
anything you want to keep, cf. `flight_log_baseline_margin_closure.csv`).

## Flight-by-flight: what we tried, what we discarded, why

**Flight 1 (success, deceptively).** Quadratic closure only, naive lead angle (1.2°,
auto-computed from braking distance at 75% throttle). Landed 19 m from target, 310 m/s.
Post-hoc analysis showed the horizontal profile had quietly reversed direction mid-BRAKE
(velocity through zero to ~−60 m/s) and the ~65 m/s excess over the ~245 m/s budget was
that contortion. It "worked" because the contortion was flown gently and continuously.

**Flight 2 (bad).** We blamed flight 1's Δv on the long low-throttle burn and switched
BRAKE's closure to thrust-margin (t_go such that demand = 90% of a_max), per the notes'
§6 recommendation. Violent dive-toward-the-ground at BRAKE start, flip-and-burn
oscillation, ~800+ m/s. **Discarded — two reasons, both confirmed by telemetry:**
1. The low-throttle-is-wasteful premise was wrong. At near-orbital horizontal speed the
   ship is still mostly in orbit (centrifugal support); gravity loss accrues only when
   lingering *slow*. A long gentle burn with *consistent geometry* costs ≈ the impulsive
   floor (~105 vs 101 m/s). Flight 1's excess was contortion, not hang.
2. The margin closure is structurally unfit as implemented: its defining equation
   `|a_cmd(t)| = 0.9·a_max` is non-monotonic in t → multiple roots → bisect picks one
   arbitrarily → successive 10 s re-solves hop between roots (telemetry: t_go 22.7 →
   55.3 in one re-solve), each hop commanding a contortion. Removed from the code; its
   only reliable job (authority feasibility) became a pre-flight check
   (`min_brake_duration` vs `brake_duration`, in the planning block).

**Also added after flight 2** (kept, both):
- *Attitude-gated throttle* in `fly_gate` (throttle 0 unless facing within 30° of the
  commanded vector): mis-pointed thrust during commanded flips is the energy source that
  sustains a guidance limit cycle. Nominally never fires.
- *Per-gate radar-altitude floors* → `emergency_land()` (abandon target, kill velocity,
  land here): below the floor there is no altitude left to hand a pilot.

**Flight 3 (bad, margin closure still in as max(margin, quadratic)).** Same qualitative
mess; the log showed the root-hopping directly. Led to the full retreat to
quadratic-only BRAKE, plus the *lead-consistency* principle: lead, speed, and duration
must describe the same burn (`distance = avg speed × time`); hand the law a
boundary-value problem violating that and it will still solve it — by dive or reversal.
The planning block was rebuilt to: solve the burn duration on the ground (same quadratic
the closure solves at PDI, vv=0), check engine feasibility against it, derive the lead
from it.

**Flight 4 (bad, >400 m/s).** Two *new* root causes found in telemetry, both real, both
fixed, both kept:
1. **Curvature sag.** The guidance law projects onto ship-local vertical; an aim point d
   down-range sits d²/2r below the local horizontal because the body curves away. On
   Minmus at 13 km range the sag (~1.3 km) exceeded the nominal drop (~1 km) and doubled
   the flown t_go vs the flat-earth plan (83.5 s vs 41 — the sag arithmetic reconciles
   both this and an unexplained "76 s" from flight 2 *exactly*). Fix: the planning block
   iterates duration↔distance↔sag (3 passes). The flight side was always correct — it
   sees true geometry through `altitudeposition`.
2. **Eccentric parking orbit.** DOI is ~10 m/s; the parking orbit's radial velocity
   (~3.5 m/s at e≈0.024) is *not* small against it, so the retrograde burn point is not
   the new apoapsis and periapsis is not 180° away — PDI shifted ~7° up-range,
   consistently. Fix: `perform_doi` now verifies the *planned* node's own predicted
   orbit (`nd:orbit`): find its periapsis longitude via kepler's `time_of_periapsis` +
   `geoposition_at`, compare to desired (`tgt − lead`), feed the error back into the
   burn longitude, re-plan (≤4 attempts, 0.2° tolerance). Measure-don't-model: catches
   any placement error visible in the patched-conics prediction.

**Flight 5 (tonight; landed, 148 m miss, ugly, ~400+ m/s).** The decisive one. Console +
CSV together:
- Planning was right: lead 19.63°, duration consistent.
- The DOI verification loop *converged*: `DOI plan 4: periapsis lng -19.69, want -19.63
  (err -0.05)`.
- The flight then hit PDI at **~36–39° down-range** (`BRAKE: t_go 350 s; site 38.7 km
  down-range`), roughly *double* the lead. BRAKE spent 230 s falling at 2–5% throttle
  toward an aim 42 km away (t_go 350 s is exactly right for that distance including
  sag — the flight math is self-consistent), handed APPROACH the ship at 236 m/s
  (spec: 60), which reversed to −128 m/s and oscillated before recovering. TERMINAL
  landed it.

So: the plan was right, the loop converged, and reality disagreed with the *prediction*
by ~20°. The loop actively steered the true periapsis 20° off while zeroing the
predicted error.

**Flight 6 (2026-07-16; the kepler fix + h_pdi 3000). Landed 1 m from target, ~250 m/s
total.** CSV (archived: `flight_log_flight6_kepler_fix_h3000.csv`) confirms every step-3
signature: PDI at alt 3017 with v_vert −0.1 (placement clean); BRAKE entry t_go 52.5 s
vs ~53 s reconstructed from the planning quadratic (terrain ≈ 0 at the flats); min
`v_to_site` −0.2 (no reversal — the −0.2 is touchdown noise, not a mid-phase sign flip);
max facing_err 7.8° (attitude gate never fired); max throttle 0.214 (never near
saturation). Handoff to APPROACH at 78 m/s vs 60 spec — the t_handoff=5 early exit,
inherited cleanly. Integrated thrust Δv: BRAKE ~95 m/s over 44 s, APPROACH ~109 m/s
over 91 s; with DOI ~10 and TERMINAL ~30 (unlogged, estimated) the books balance
against the reported ~250. The placement chain (curvature sag, lead consistency,
eccentricity feedback, epoch fix) is now *verified*, not believed.

**Flight 6's new fact: APPROACH is now the Δv hog.** 91 s × 0.49 m/s² ≈ 45 m/s of its
109 is pure gravity hang while covering 3.1 km at ~35 m/s average. BRAKE flew at 21%
throttle throughout — direct evidence that its duration is pinned by the wrong (vertical)
axis. Both point at the targeting redesign (below), not at guidance.

## The prime suspect — CONFIRMED AND FIXED (see flight 6)

`core/kepler.ks`, `body_rotation`:

```
return orbit_:body:rotationangle + (360 / orbit_:body:rotationPeriod) * (t - orbit_:epoch):seconds.
```

`rotationangle` is the body's rotation angle **now** (at call time), but the
extrapolation runs from **`orbit_:epoch`**. Those agree only when epoch ≈ now — true for
`ship:orbit` (KSP keeps its epoch current), which is every pre-existing caller. For a
maneuver node's `nd:orbit`, epoch is the **node time** (~2,241 s in the future tonight),
so `body_rotation` under-rotates by (node time − now):

2,241 s × 360°/40,400 s = **19.97°** — matching the observed ~20° discrepancy between
predicted (−19.69°) and actual (~−36 to −39°, chord-geometry slop plus the ~7°
eccentricity shift the loop was legitimately trying to remove account for the residual).

`body_longitude` subtracts `body_rotation`, so the predicted periapsis longitude came
out ~20° too far east; the loop then biased the real burn ~20° west to compensate.

Note the contrast with `mean_anomaly` (same file): anchoring *orbital elements* at
`orbit_:epoch` is correct — `meanAnomalyAtEpoch` is genuinely an epoch quantity. The bug
is mixing a *now*-sampled angle with an *epoch*-based Δt. (`mean_anomaly` had its own
bug — `orbit:period` for `orbit_:period` — already fixed this session.)

## What needs to happen next

1. **Verify the suspect**, cheaply and in isolation: in-game, with any maneuver node
   added, compare `geoposition_at(time_of_periapsis(timestamp(nd:time), nd:orbit),
   nd:orbit):lng` against where the ship actually ends up (or against
   `body:geopositionof(positionat(ship, t_pe))`, which uses KSP's own propagation and
   has no frame math to get wrong). Expect a discrepancy = (node ETA) × 360/rotation
   period. **Caveat (2026-07-16): the `geopositionof(positionat(...))` ground truth has
   its own frame trap** — `positionat` returns the future inertial position but
   `geopositionof` maps it through the body's *current* orientation, so its raw `:lng`
   must itself be corrected by −(t_pe − now) × 360/rotationPeriod before comparison
   (unambiguous — involves no orbit epoch). Run naively against the *fixed* code it
   shows a ~(node ETA)×ω discrepancy and falsely convicts the fix. Warping to t_pe and
   reading the ship's actual longitude avoids the trap entirely.
2. **Fix `body_rotation`** to extrapolate from the sample time of `rotationangle`, i.e.
   `(t - time):seconds`, not `(t - orbit_:epoch):seconds`. Audit the other consumers
   (`body_longitude`, `geoposition_at`, `time_to_longitude`) — all should be unaffected
   for ship:orbit and *corrected* for future-epoch orbits. `core/test_kepler.ks` exists.
   **[DONE 2026-07-16.** kOS source confirmed ROTATIONANGLE is a zero-arg passthrough to
   KSP's live `Body.rotationAngle` — no epoch input path exists. Fix applied; consumer
   audit clean (`time_to_longitude` passes `t=time`, now exact; `land_at_periapsis.ks`
   uses ship:orbit). Not yet verified in flight — steps 1/3 signatures still pending.]
3. **Re-fly. [DONE — flight 6, all signatures met; see above.]** The debugging campaign
   is closed. What follows is the optimization campaign.
4. **Telemetry gaps before optimizing** (the CSV must be able to say *why* a flight's
   Δv changed, not just that it did): (a) planning numbers (lead, brake_duration,
   planned t_go, desired PDI lng, DOI plan errors) into the CSV header — console is
   still the only witness; (b) log TERMINAL rows (currently `log_state` lives only in
   `fly_gate`, so the last ~150 m is dark); (c) a mass or `ship:deltav` column so true
   Δv per phase comes from the log, not from integrating `a_cmd` at 1 Hz.
5. **Targeting redesign (Δv-optimal planning), agreed 2026-07-16.** Criterion: minimize
   Δv subject to soft-and-on-target from a rough orbit; timing/path shape don't matter
   in KSP. Direction: invert the planning dependency chain — T_brake from the
   *horizontal* axis at design throttle (flight 6 flew BRAKE at 21%); the vertical
   cubic is then fully determined, so `a_arrival` becomes a planning *output* fed to
   the unchanged in-flight closure; h_pdi becomes an output too, parameterized as
   `tgt:terrainheight + clearance` (Schuyler: ~2000 m — the clearance is the real
   mission-planning input; his pre-project landers were all suicide burns and died by
   terrain-model optimism, so the floor is deliberate). Flight 6 says the tail is now
   the fuel hog: ~45 of APPROACH's ~109 m/s is gravity hang — tightening the gates
   (lower/faster high gate, shorter APPROACH leg) is the next lever after BRAKE.
6. Open questions, deliberately deferred: APPROACH lead/duration consistency when BRAKE
   hands over off-spec (flight 6 handed over at 78 vs 60 m/s spec via the t_handoff=5
   early exit, digested without reversal — but the targeting redesign should make the
   handoff state part of the plan); terminal-phase position feedback for sub-meter work
   (deferred: a Kerbal can walk).

## Process notes (hard-won)

- **We lost most of this session to guessing, and that must not continue.** The pattern,
  repeated four times: reason from a plausible mechanical model, implement a fix, predict
  a good flight, be wrong. Two of the "fixes" addressed non-problems (the thrust-margin
  closure was motivated by a gravity-loss premise that telemetry later refuted; the first
  lead-angle "fix" moved a number that was never the binding error). The flight recorder
  ended the guessing: every root cause actually found (root-hopping, curvature sag,
  eccentricity shift, the epoch suspect) came from reading the CSV or console, usually
  reconciling a specific logged number to the digit. **Rule for all future work: no claim
  about flight behavior — diagnosis or fix — without telemetry that supports it, and no
  fix implemented until the hypothesis it rests on has been checked against a log.
  Predictions are stated only as testable signatures for the next flight
  (which column, which value), never as promised outcomes.**
- One instrumented change per flight.
- The guidance law itself has never been the problem: every failure was in what we
  *asked* it to do (inconsistent boundary conditions, misplaced PDI, mispredicted
  frames). Its recovery from a backwards, 236 m/s handoff to a 148 m landing is the
  book's best advertisement for E-guidance.
- Work block-by-block with Schuyler: present each block, review, apply only on explicit
  approval. Comments state timeless principles, not test-run war stories.
