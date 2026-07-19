# The braking→terminal handoff: a contract, not a coincidence

*A design note, downstream of `powered-descent-invariants.md`: it revises that note's
invariants 2 (targeting) and 5 (handoff continuity), first on paper and then through an
eight-flight campaign (2026-07-18/19) that ended with terminal flying Klumpp's guidance.
Everything in the invariants note still stands; this narrows two of its clauses and
records what the flights taught. Companion: the descent lives in `powered_descent_min.ks`;
the planner it contracts with is `plan_doi.ks`.*

## The founding diagnosis (flight 1: 71 m)

A TWR-27 landing missed by 71 m, all down-range overshoot, from two independent failures
hiding behind each other:

1. **Terminal's lateral authority was a fiction.** The whisper law commanded total thrust
   along a slightly tilted hold, so only `sin(θ)` of it landed horizontally — ~0.02 m/s²
   at the angles actually commanded. It could not null a walking pace.
2. **Braking aimed the arc's endpoint onto the site with the handoff velocity still
   live.** `reach == dist` reserved nothing for the residual, so ~5 m/s arrived directly
   over the target as pure overshoot.

Fixing only one buys either a strong controller cleaning up a mis-aimed arc, or a clean
handoff to a controller too weak to hold it. Both had to change, as separate
responsibilities that do not cover for each other.

## The principle: each phase hands the next a workable state

The whole descent is a chain of handoffs. DOI planning does not fly the landing; it hands
braking a periapsis braking can fly. Braking owes terminal the same kind of debt: not a
landing, but a state terminal can finish from — an up-range offset and a bounded closing
velocity that terminal, using only the authority it legitimately has, spends coming to
rest over the pad. The carrying quantity:

```
d_handoff = vh² / (2 · a_eff)        # stopping distance of the seam residual
```

with `vh` the horizontal speed at the seam and `a_eff = 0.8·a_lat_max` the budgeted
fraction of the lateral cap (`a_lat_max = g0·tan(tilt_max)`). Braking aims
`reach + d_handoff == dist`; the residual coasts in; terminal brakes it to rest. Derived,
body-aware, craft-free.

## The campaign: eight flights, 71 m → 5 m

Each flight falsified something specific. In order:

1. **71 m** (whisper law): the founding diagnosis above.
2. **9 m ×2** (first revision: attitude seam, up-range aim, P-controller null): two
   defects. The seam exit `pitch <= -tilt_max` was the *complement* of the intended
   angle — handoff at retrograde 60° from plumb, not 30°, with a 30° attitude slew at
   the "attitude-continuous" seam and 23.5 m/s of residual. And the P-loop tracking a
   decelerating reference carries a standing error of `a/k` (4.6 m/s at the old gain):
   the craft overflew the site at 1000 m and walked back too slowly. *Lesson: a pure
   P-loop cannot track a decelerating reference without feedforward; the overshoot was
   built into the law.*
3. **Hover-wobble, aborted**: over the pad, saturated commands flip-flopping, descent
   stalled into a dv-burning hover. Attributed first to the stopping law's cusp
   (`vc²/2d` demands `a_eff` at every scale — real, fixed by flooring the divisor at
   `h_pad`), then to thrust pumping through attitude slews (the alignment gate came from
   this theory; the new drift log column falsified its "circling" half).
4. **15 m** (floor + gate in): still dancing at the gate boundary. The actual root
   cause, present since flight 2: `k_trim` was derived from the *seed* solve's seam, and
   the seed's arc — integrated 60 s early, burning through what the craft actually
   coasts — hits `endpoint`'s ground exit, handing the gain a zero fall time. Gain 3.0
   instead of 0.024: a 125× error explaining every "yawing right over the target"
   symptom, with the ignition assertion's `WARN ... vs fall 0 s` as its fingerprint on
   every flight. Fixed by computing gain and workability from the live handoff state.
   *Lessons: a derived constant is only as good as the state it is derived from; and
   when a controller's behavior is arithmetically impossible under the intended
   constants, check the constants as flown before inventing dynamics.*
5. **17 m** (correct gain): the law finally parked — plumb, engine off, ~1 m offset at
   mid-fall. Then a whisper correction re-engaged at the deadzone boundary (no
   hysteresis) and rode its 30° lean into the *ungated* suicide burn: the first
   full-throttle second fired 25° off plumb and threw ~5 m/s of drift. The entire miss
   manufactured in the last 40 m. *Lesson: near the end of a descent, the cost of a
   correction is not its thrust but the attitude state it leaves behind for the next,
   less forgiving phase.*
6. **7 m** (consequence latch + plumb fence): lateral law latched on predicted miss
   rather than command size, and a fence ends the lateral game one attitude-swing before
   burn ignition. Burn fired at 1.3° facing error; the injection failure died here.
7. **5 m** (Klumpp): the guidance swap (below). Landed at the latch's designed
   tolerance with a plumb burn — but chattered visibly mid-fall because the latch's
   engage and release thresholds had been collapsed to the same line.
8. **2 m** (band + proportional lean): the design's promise, kept. One latch
   engagement for the whole fall; Klumpp's command tapered 0.45 → whisper while the
   lean walked 26° → plumb with `facing_err` ~0.1° throughout — no slews, no chatter;
   37 rows parked plumb; burn ignition at 1.3°; touchdown drift ~1 m/s; dv within
   4 m/s of the earlier flights. Every item on the falsification list passed.

## The architecture as flown (2026-07-19)

The phase structure is Apollo's, reappropriated wholeheartedly:

- **DOI** (plan_doi) sets **PDI**; coast; **PDI** ignites braking.
- **Braking burn to high gate** — the seam: retrograde within `tilt_max` of plumb, so
  the handoff is attitude-continuous by construction.
- **High gate → low gate**: unpowered fall; Klumpp's guidance (Apollo P63/P64,
  two-boundary form) trims the horizontal only:
  `a = 6·ZEM/t_go² − 2·ZEV/t_go`, target = over the pad at zero horizontal velocity,
  `t_go` = time to the low gate (closed-form intersection of the free fall with the
  ignition schedule). No gains, no schedule; `t_gate` scales everything.
- **Low gate** = suicide-burn ignition (`|vv|` meets `v_sched`); **terminal burn** to
  `v_floor` at the pad, vertical, owning the last metres itself.

One deliberate divergence from Apollo: braking is *not* Klumpp. A retrograde hold pays
zero steering loss — every newton-second kills velocity that must die anyway — and its
single degree of freedom, throttle, aims the endpoint down-range through the re-solve.
Klumpp buys trajectory shape and pays cosine losses for it; flying it over 170 m/s of
braking would spend that overhead where the state is expensive. Flying it from high gate
spends it on ~4 m/s of residual. The shaping cost scales with what is left to shape, so
the expensive law goes where the state is cheap. The coupling condition — high gate may
be anywhere, so long as low gate is reachable from it within the craft's authority — is
the code's contract in both directions: `solve_f` prices it into the aim, and the
terminal-entry assertion warns if the guidance ignites saturated.

Around the continuous law, three discrete guards, each earned by a flight:

- **The latch** (`corr_on`): correct only when coasting would miss by more than `h_pad`;
  release once driven under `h_pad/2`; coast plumb between. Consequence, not error.
- **The alignment gate** (`face_tol` 15°): free-fall thrust waits for attitude — a
  correction fired misaligned by `e` manufactures `sin(e)` of new drift. The suicide
  burn is never gated.
- **The plumb fence** (`t_settle` 3 s): one slew-time before the schedule crossing, the
  lateral game ends for good, so the burn always ignites plumb.

And the delivery: lean scales with command — `tan(lean) = sqrt(a_lat/a_lat_max) ·
tan(tilt_max)` — so `tilt_max` is a true maximum, reached only at saturation; the
vertical component stays ≤ `g0` (equality only at the cap), so corrections never climb;
whispers cost nods, not 30° slews. The fixed-at-max lean it replaced was the correct fix
for the `sin(θ)` suppression when every command was saturated, and pure attitude
overhead once Klumpp made every command gentle.

## Constants, and what argues each

Chosen margins (falsified by insensitivity — the landing should not care within a band):
`tilt_max` 30° (attitude margin kept to swing back and brake; also fixes the seam and
the lateral cap), `f_max` 0.85 (throttle reserve for the vertical), `a_eff` fraction 0.8
(lateral planning reserve, same argument one axis over), `t_settle` 3 s (one attitude
swing), `face_tol` 15° (sideways injection under a quarter of a correction), `h_pad` 5 m
(the flare radius, now also the latch band and the accepted lateral remainder),
`v_floor` 2 m/s, integrator tolerances `pitch_tol`/`v_frac`.

Everything else is derived: `a_lat_max`, `d_handoff`, `t_gate`, `tau_yaw = t_go/3`,
`a_dec`, `v_sched`, `a_req`, and Klumpp's coefficients, which are the mathematics.

## Ripples

`plan_doi.ks` remains deliberately untouched; its march still ends at its own threshold.
The divergence is tolerated because the seam ends braking earlier than a low-speed exit
would, so the planner's certified corridor remains conservative — restate this check if
the seam ever moves later. The aim shift (`d_handoff`, tens of metres against an
11–14 km arc) stays far below anything the planner would have to model.

## To falsify in flight

- Handoff attitude continuous: no `facing_err` spike at the BRAKE→TERMINAL row.
- Klumpp's command starts ≈ `6·ZEM/t²` under the cap and *tapers*; at the low gate both
  ZEM and drift are ≈ 0 without the fence having to clamp anything.
- At most one or two latch engagements per fall; lean during them proportional to the
  command, never `tilt_max` for a whisper.
- Burn ignition rows: `facing_err` ≈ 0, no drift jump across ignition.
- Touchdown: miss ≤ `h_pad`, horizontal velocity a few tenths; dv within a few m/s of
  the retrograde solve's prediction (the braking-optimality claim).

## Standing lessons

- A scale-free control law must be given a scale at which to stop caring, or it will
  bang-bang at the precision floor.
- With a steering loop in series, thrust must wait for attitude; an engine that burns
  through its own slews closes a positive feedback loop through the plant.
- A derived constant is only as good as the state it is derived from; never derive from
  a stale estimate what is about to be measurable.
- When observed behavior is arithmetically impossible under the intended constants,
  audit the constants as flown before inventing dynamics.
- Discrete guards (latch, gate, fence) are not fudges; they are the decisions a
  continuous law cannot make — when to care, when to fire, when to quit. But every
  discrete transition costs whatever attitude the delivery ties to it, so the delivery
  must make small commands cheap.
