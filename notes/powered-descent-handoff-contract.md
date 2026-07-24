# The braking→terminal handoff: a contract, not a coincidence

*A design note, downstream of `powered-descent-invariants.md`: it revises that note's
invariants 2 (targeting) and 5 (handoff continuity). Everything in the invariants note
still stands; this narrows two of its clauses. Companion: the descent lives in
`powered_descent.ks`; the planner it contracts with is `plan_doi.ks`.*

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

Two facts make the debt real, and they are separate responsibilities that cannot cover for
each other. Terminal's lateral authority is only `sin(θ)` of total thrust at the angles it
actually commands — a hundredth of a m/s² at a slight tilt, which will not null a walking
pace. And an arc aimed `reach == dist` reserves nothing for the residual, so the seam's
horizontal speed arrives directly over the target as pure overshoot. Braking has to buy the
offset; terminal cannot manufacture it.

## The architecture as flown

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
whispers cost nods, not 30° slews.

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
