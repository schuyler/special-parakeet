# Capability-driven descent: a bare-bones general edition of Apollo's landing program

*Design note, 2026-07-17 (rev. 2). Successor to `targeting-redesign-checkpoint.md`. That
note inverted the planning chain (BRAKE duration from the horizontal axis at design
throttle) and fixed the BRAKE-loafs-at-21% half of the high-TWR inefficiency; this note
replaces the whole trajectory model with one continuous gravity turn and lets the gates —
and their number — fall out of it. Reframed this session around a design goal: a stripped-
down, general edition of the Apollo descent, sized for online computing power, ad-hoc
planning, and a wide range of spacecraft and bodies.*

**Status: soft commitments, worked step-by-step.** Nothing applied to
`reference/original/powered_landing.ks`. Math below is deliberately fed in small named
steps — the style of the existing `solve_t_go` / descent-design code (name each
intermediate, one physical idea per line), so a reader two semesters into calculus can
follow every line. No code blocks yet; those come block-by-block on approval.

## What this is, and what it strips out

Apollo's descent was shaped by three things we don't have to live with:

- **The AGC couldn't integrate the trajectory in flight**, so the reference path (position,
  velocity, *and acceleration* at each gate) was computed on the ground and flown as a
  stored table. We have compute to spare — we integrate the path live at ignition. **That
  deletes the trajectory tables.**
- **Missions were planned for months.** We plan ad hoc: point at a surveyed target from
  whatever orbit we're in, and solve the descent from live state. **That deletes the
  ground-planning loop** — everything is computed at PDI from `v_pe`, mass, thrust, and body
  constants.
- **There was a crew.** No crew means no pitchover-for-visibility, no manual short-final, no
  moded aborts around those. **That deletes the reasons Apollo's gates sat where they sat.**

What's left is small: one guidance law (Klumpp's quadratic, already flown), one ideal path
(a gravity turn), and a rule for chopping that path into as many legs as the law needs — one
on a gentle body, more on a harsh one. The generality is the point: the same code should
land a TWR-2 tug on Minmus and a TWR-30 probe on the Mun, because nothing in it is tuned to
a body or a vehicle. It reads `a_max`, `μ`, `r`, `g`, the target, and goes.

## Diagnosis: high TWR isn't the problem, the fixed gates are

Gravity loss is `∫ (g opposing thrust) dt` — later, harder, shorter braking makes it
smaller, so efficiency *should* rise with TWR. The current design inverts that because its
gates are fixed and refuse the thrust. Flight 7 (TWR-34, 0 m miss): 244.1 m/s, of which
**BRAKE 100.2, APPROACH 107.6**. Flight 6 read the mechanism: BRAKE at **21% throttle**
throughout, and ~45 of APPROACH's 109 m/s was **pure gravity hang** (3.1 km at ~35 m/s).
The fuel-optimal brake for that craft is ~10–15 s; the flown design spent ~135 s of engine-
against-gravity. That surplus is the ~50 m/s gap between the flown 244 and the impulsive
floor.

## The ideal: one continuous gravity turn

The fuel-optimal airless descent is a gravity turn flown in reverse — hold thrust exactly
**surface-retrograde** and let gravity rotate the velocity vector from horizontal to
straight-down as the burn bleeds off speed, arriving vertical over the target. Retrograde is
the rule for three separable reasons:

1. It's the minimum-ΔV direction to null a velocity vector — no thrust wasted turning it.
2. It cancels the vertical velocity **concurrently** with the horizontal, while the craft is
   still fast and centrifugal support makes vertical cheap — instead of deferring it to the
   slow terminal hover, where it's expensive. (This is why the checkpoint's brake-horizontal-
   then-clean-up-vertical is the *expensive* side of retrograde, and part of why APPROACH
   hangs.)
3. It minimizes time spent slow. Gravity loss accrues where `g − v²/r` is large, i.e. at low
   speed; retrograde is only slow briefly, at the very end.

Apollo declined this hoverslam for crew visibility, abort coverage, and blind terrain. We
share none of the first two, and the third — terrain uncertainty — is exactly what a
surveyed target removes over the site. The gravity turn's geometry even hands us the safety
for free: with `v_pe > v_circular` the arc *rises* just after PDI and is **highest up-range**
(where terrain is only modeled) and **lowest over the target** (where radar is truth).

**One correction to rev. 1:** the earlier note claimed a hard TWR regime split (a 114 m/s
arrival for the low-TWR craft). That number was an artifact of the *horizontal* model, which
lets the vertical free-fall. The gravity turn cancels vertical as it goes, so it closes
softly across essentially the whole TWR range. There is no regime wall — only a continuous
efficiency knob, and it is clearance, not TWR (see below).

## The model, in small steps

Everything is one short integration, run once at PDI. Carry four numbers and step them
forward by a small `dt`. This replaces both Apollo's stored trajectory and the checkpoint's
closed-form drop integral — we just let the computer walk the path.

**The four rates.** Let `γ` be the flight-path angle below horizontal (`γ = 0` level,
`γ = 90°` straight down), `v` the surface speed, `a_T = f·a_max` the retrograde thrust.
Each rate is one idea:

```
a_felt = g − v^2 / r          // the felt vertical accel: gravity minus the centrifugal
                              // support the ground-track speed already provides
v_dot     = −a_T + g·sin γ    // speed: thrust brakes it; gravity feeds a little back as
                              // the path tilts downhill
gamma_dot = a_felt·cos γ / v  // heading: ONLY gravity-minus-centrifugal turns the vector.
                              // Retrograde thrust is anti-parallel to v, so it cannot turn
                              // it — that is what keeps this line clean.
h_dot     = −v·sin γ          // altitude: falls at the downward part of the speed
x_dot     =  v·cos γ          // downrange: advances at the forward part of the speed
```

`gamma_dot` is the whole story of the arc: while `v > v_circular`, `a_felt < 0` and the
vector turns *up* (the craft rises); once braking drops `v` below circular, `a_felt > 0` and
it pitches toward vertical. Same `a_felt` the reader already met, now steering the heading.

**Walk it forward (Euler's method — the kinematic ladder, one rung at a time).** Seed at
PDI, step until the speed is nearly gone:

```
set v to v_pe.  set gamma to 0.  set h to h_pdi.  set x to 0.  set t to 0.
until v <= v_low {
  set a_felt to g - v^2 / r.
  set v     to v     - (a_T - g*sin(gamma)) * dt.
  set gamma to gamma + a_felt*cos(gamma)/v * dt.   // watch deg/rad: kOS trig is degrees
  set h     to h     - v*sin(gamma) * dt.
  set x     to x     + v*cos(gamma) * dt.
  set t     to t     + dt.
}
```

Out of one loop: the **drop** (`h_pdi − h` at the end), the **downrange** `x` (→ the lead
angle), the **duration** `t`, and — because we record `(v, gamma, h, x)` as we go — the ship's
state at *every* point on the arc, which is all we need to place gates.

**Place PDI.** The drop is what the arc needs; the floor is what the mission accepts:

```
set drop     to h_pdi_start - h_end.                 // from the integration
set h_needed to tgt:terrainheight + h_lg + drop.     // start high enough to reach low gate
set h_floor  to tgt:terrainheight + clearance.       // never fly the corridor below this
set h_pdi    to max(h_needed, h_floor).
```

Two cases fall straight out, and they are the efficiency knob:

- **Clearance loose** (`h_needed ≥ h_floor`): fly full margin throttle, brake low and late —
  the ideal suicide burn.
- **Clearance binds** (`h_floor` wins): now `h_pdi` is pinned, and **throttle becomes the
  free variable.** Lower `a_T` lengthens the brake and deepens the drop, so bisect `a_T` (the
  codebase already has `find_zero_crossing`) until `drop(a_T) = clearance − h_lg`. You throttle
  *down* to spend exactly the altitude the floor mandates.

This re-reads flight 6's 21% throttle: a high-TWR craft under a 2 km floor *should* throttle
down — its natural drop is metres, so clearance always binds and it must spend altitude. The
21% was roughly right; what was wrong was the *path* (flat-then-hover instead of one arc). So
the fix may barely move the throttle — it changes the shape and kills the hang. And the
bisection has a floor of its own: below some `a_T` the arc can't close (it would arrive still
fast, or need to hover off-retrograde). Hitting it is a clean pre-flight signal — *"clearance
too high for this craft, lower it or add thrust"* — caught on the ground.

## Selecting the gates: tessellate the arc

The gates are not places where anything physical happens — on a continuous arc, nothing does.
They are the resolution at which we resample the ideal path so the guidance law can track it.
The law flies a **chord**: a constant-jerk profile matching the two endpoint states but
cutting across whatever the true arc does between them. So a gate is where one chord ends and
the next begins, and we place them by bounding how far the chord may stray from the arc —
exactly like breaking a smooth curve into straight segments with a bounded gap.

Two small per-leg budgets, checked as we walk the arc; end the leg when **either** trips:

```
// (a) along-track sag: the chord's gap from the arc over a leg that sheds dv of speed.
set dv  to v_gate - v.                 // speed shed since the last gate
set sag to dv^2 / (4 * r).             // the note's own residual bound, reused
if sag > sag_budget { drop a gate. }   // authority spent on chord-vs-arc mismatch

// (b) heading change: how far the vector turned since the last gate.
set dgamma to gamma - gamma_gate.      // degrees turned
if dgamma > turn_budget { drop a gate. }  // caps thrust mispointing across the chord
```

Where each budget bites tells the whole story:

- **Sag `dv²/(4r)`** grows with speed shed and shrinks with body size. On Minmus (`r ≈ 60 km`,
  `v_pe ≈ 170`) the sag for the *entire* brake is only ~0.1 m/s² — so the law could fly the
  whole brake as **one chord** and the sag budget never trips. On the Mun (smaller `r`,
  faster `v_pe`) it tightens and forces a mid-brake gate or two.
- **Heading `dgamma`** trips at the **pitchover** — the last stretch before the target, where
  `gamma_dot` blows up (`v` in its denominator → 0) and the vector whips from shallow to
  vertical. That is where the honest analog of Apollo's "high gate" lands: they pitched over
  there for the commander's eyes; we cut a leg there because the Cartesian law needs
  re-anchoring. Same point on the arc, chosen by geometry either way.

So on a gentle body you get **one interior gate**, at the pitchover; on a harsh one, a couple
more up high. **Low gate** is separate — not a tessellation gate but the terminal handoff,
forced by the law's own divergence as `t_go → 0`, fixed at `terrain + h_lg`. The count is an
*output* of the body and craft, which is the generality we want; "high gate + low gate,
always two" is an Apollo inheritance the budget rule replaces.

**Continuous tracking, weighed and declined.** The fine-tessellation limit is trajectory
tracking — carry the arc, feed the law a moving target, no discrete gates. We chose against
it: `lock steering`/`lock throttle` already re-solve the *command* every physics tick, so
only the *target* is discrete, and keeping it so avoids re-importing the stored-path
dependence the campaign spent five flights removing — for a ΔV saving the sag budget shows is
negligible. The discrete gates are the design.

## Low gate: parameterize and walk it down

150 m is Apollo's 500 ft, unscaled, and "radar is truth" holds at any height over a surveyed
site — so it's no floor. The real floor is terminal's room to flare the descent to a soft
touchdown, null any residual drift, and stay clear of the law's divergent gains near
`t_go → 0`. All three scale with how fast and clean the handoff is. Terminal is a near-hover
(throttle at the gravity feedforward), so every metre is gravity loss, worse on higher-g
bodies — flight 7's TERMINAL was 24.4 of 244. Decision: **make it a parameter**, default well
under 150 (~50 m, ~20–30 m floor), let it couple to a slightly slower arrival, and **walk it
down with telemetry** (touchdown `v_vert` and miss distance are the scores) rather than
picking a number by argument.

## Margin

`f < 1` does double duty: the reserved `(1 − f)` is both the authority the closed-loop law
spends absorbing error and the ignition-timing cushion. Don't design a literal `f = 1`
hoverslam — the law's error-absorption *is* the robustness. Start `f ≈ 0.85`; sweep it if the
law shows room. Saturation guard and attitude gate stay.

## What this strips out

- **vs Apollo:** the ground-computed trajectory table (we integrate live); the fixed
  two-gate/visibility structure (tessellation picks the count); the crewed pitchover and
  manual short-final (guidance flies it).
- **vs the checkpoint:** the closed-form horizontal/vertical drop integrals (one Euler loop
  replaces both); the fixed `60 m/s @ 2 km` high gate and the separate APPROACH leg (gates
  are read off the arc, and the hang goes with them); the `114 m/s` regime framing (an
  artifact — dropped).
- **Kept:** the Klumpp quadratic guidance law and `solve_t_go` unchanged; `a_felt = g − v²/r`;
  lead-consistency; DOI/coast/eccentricity-feedback/kepler fix untouched; the terminal
  rate controller.

## Decisions / forks, with recommendations

- Sample the arc how? → **Tessellate to a sag + turn budget**; ≥1 gate, count is an output.
  Interior gate lands at the pitchover.
- Precompute the arc or track it continuously? → **Precompute at PDI, discrete gates.**
  Continuous tracking weighed and declined (above).
- Design margin `f`? → **0.85**, swept in flight.
- Low-gate height? → **Parameter**, ~50 m default, walked down with telemetry.
- Terminal controller? → **Keep it.**

## Open items

Named, not yet resolved. Each gets pinned as we build, in the campaign's measure-don't-assume
style — reason out a starting value, let a flight move it. Resolutions proposed in chat; folded
in on approval.

1. **Constant `g`/`r` in the rates vs. recomputing them from `h` each step.** Over the low-TWR
   craft's ~3 km drop on Minmus, `g` moves ~9%. Recomputing is nearly free but couples the arc
   shape to `h_pdi` (see 2).
2. **The `v_pe ↔ h_pdi` coupling → an outer iteration.** `v_pe` comes from vis-viva at
   `h_pdi`, which is itself an output of the integration; with 1, `g,r` couple too. This is
   what the "Place PDI" block's circular-looking `drop = h_pdi_start − h_end` is waiting on.
3. **Integration step `dt` and stop speed `v_low`.** `dt` trades accuracy against the IPU
   budget; `v_low` is where the arc ends (the pitchover is singular at exactly `v = 0`).
4. **The tessellation tolerances `sag_budget` and `turn_budget`.** These set the gate count and
   the pitchover split.
5. **The arc-sample → gate-lexicon read (the Block-4-equivalent — the actual build).** How a
   sample `(v, γ, h, x)` becomes a gate the existing `fly_gate` consumes, and how the total
   downrange feeds the existing DOI lead placement.

## Process and predicted signatures

One instrumented change per flight (this whole reframe is one planning-subsystem change);
predictions stated only as testable signatures; blocks applied on explicit approval,
block-by-block. First flight of this design, TWR-34 test craft, columns to check (not
promised outcomes): BRAKE carries to low gate as one arc with `v_to_site → 0`, no reversal;
the APPROACH ledger collapses from 107.6 toward a short pitchover remnant; total ΔV moves off
244 toward the ~180–200 floor; the logged arc-minimum altitude up-range stays
`≥ terrain + clearance`; touchdown `v_vert` and miss distance hold at flight-7 quality.
