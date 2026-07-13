# Companion note: jerk dynamics and the Apollo-style quadratic closure

*Working draft, companion to `apollo-powered-descent.md`. That guide states the guidance
law (phases 3–4) and its `t_go` solver in passing; this note derives both from scratch and
works out the alternative `t_go` closure — the Apollo acceleration-target form — as an
implementable variant. Same status as the sibling: implementation material, rewritten into
chapter prose (chapters 11–12) later. Math is plain-text/Unicode to match the sibling;
snippets are illustrative, not tested.*

Notation throughout: `r`, `v` are current position and velocity; `r_tgt`, `v_tgt` the aim
point's; `R = r_tgt − r`; `T = t_go`. Bold quantities are vectors handled per-axis. `τ` is
time measured forward from *now*, so `τ = 0` is this instant and `τ = T` is arrival.

---

## 1. The kinematic ladder: integrating up to jerk

The whole derivation rests on one move from first-year physics — *accumulate a rate over
time to get the quantity* — applied one rung higher than the usual constant-acceleration
case. The move follows a fixed pattern in the powers of time:

| Accumulate this over time τ | …and you get |
|---|---|
| a constant `c` | `c·τ` |
| `c·τ` | `½·c·τ²` |
| `½·c·τ²` | `⅙·c·τ³` |

The denominators are `1, 2, 6` — factorials — and each step bumps the power of `τ` up by
one. That table is the *entire* machinery below.

**Constant acceleration (the physics-class case).** Freeze acceleration at `a₀`. Velocity
accumulates it (`v₀ + a₀τ`); position accumulates that:

```
v(τ) = v₀ + a₀·τ
r(τ) = r₀ + v₀·τ + ½·a₀·τ²
```

**Constant jerk (one rung up).** *Jerk* is the rate of change of acceleration. Let it be a
constant `j`. Now acceleration is no longer frozen — it ramps linearly — and everything
below it gains one extra term straight off the table:

```
a(τ) = a₀ + j·τ
v(τ) = v₀ + a₀·τ + ½·j·τ²
r(τ) = r₀ + v₀·τ + ½·a₀·τ² + ⅙·j·τ³
```

These are the constant-acceleration formulas plus a `½·j·τ²` on velocity and a `⅙·j·τ³` on
position. Set `j = 0` and you fall back to the frozen case. That extra term — the
contribution of a linearly-ramping acceleration — is the only new physics in Klumpp's law.

**Why constant jerk?** It is an *assumption*, not a fact about the engine. A linear
acceleration profile (`a(τ) = c₀ + c₁·τ`) is the lowest-order shape with **two** free
coefficients per axis — exactly enough to satisfy two endpoint demands, arrival *position*
and arrival *velocity*. A frozen acceleration (one coefficient) can force one or the other,
never both. The engine does not produce constant jerk; it produces whatever thrust the law
commands. The constant-jerk assumption lives entirely in the math, and (see §2, closing
paragraph) it is discarded and re-solved every cycle, so it only ever has to hold for one
tick.

---

## 2. The guidance law: solve the boundary-value problem

Model the vehicle as a double integrator — `ṙ = v`, `v̇ = a_total` — with the total
acceleration `a(τ)` taken to be linear in time (§1). Using the current state as the initial
condition (`r₀ = r`, `v₀ = v`, `a₀ = c₀`, `j = c₁`):

```
v(τ) = v + c₀·τ + ½·c₁·τ²
r(τ) = r + v·τ + ½·c₀·τ² + ⅙·c₁·τ³
```

Impose the two endpoint conditions at `τ = T`:

```
v(T) = v_tgt   →   c₀·T + ½·c₁·T²          = v_tgt − v
r(T) = r_tgt   →   ½·c₀·T² + ⅙·c₁·T³ + v·T = R
```

Two linear equations, two unknowns. Solving (eliminate `c₁` from the velocity equation,
substitute) gives:

```
c₀ = 6·R/T² − (4·v + 2·v_tgt)/T
c₁ = 6·(v + v_tgt)/T² − 12·R/T³
```

The commanded acceleration *now* is the profile at `τ = 0`, which is just `c₀`:

```
a_cmd = c₀ = 6·(r_tgt − r)/t_go² − (4·v + 2·v_tgt)/t_go
```

That is the law in `apollo-powered-descent.md:140`. Evaluating the *same* profile at the
far end `τ = T` collapses to a clean mirror image (used in §4b):

```
a(T) = c₀ + c₁·T = −6·R/T² + (2·v + 4·v_tgt)/T
```

**Gravity.** `a(τ)` here is *total* acceleration — the actual second derivative of
position, thrust plus gravity. To realise `a_cmd` the engine must supply `a_cmd − g`, since
gravity already contributes `g`. That is the `a_cmd − g_vec` line in the sibling's
`guidance_step` (`apollo-powered-descent.md:160`). This "subtract a roughly-constant g at
the end" is the simple convention; Klumpp folded a varying-gravity term into the position
equation instead, which matters only when `g` changes appreciably over the arc.

**The assumption is never flown.** After solving, we keep only `c₀` and throw `c₁` (the
jerk) away, re-measuring the true state and re-solving next cycle. The constant-jerk
trajectory is a prediction discarded before it can be wrong; only its value at `τ = 0` is
used, where the jerk term `c₁·τ` contributes nothing. Every error — late ignition, thrust
dispersion, mass-model slop — becomes part of the next cycle's measured state and is
absorbed. This is what makes the law closed-loop (Cherry's E-guidance: compute from present
state, do not track a stored trajectory).

---

## 3. Reading the law: it is a PD controller

Write the position error `e = r_tgt − r`. For a fixed target its rate is `ė = −v`. In the
arrive-at-rest case (`v_tgt = 0`) the law is exactly:

```
a_cmd = (6/t_go²)·e + (4/t_go)·ė       ← proportional + derivative
        └── Kp ──┘      └── Kd ──┘
```

- **Kp = 6/t_go²**, **Kd = 4/t_go** — the position and velocity terms *are* the P and D
  gains. The `v_tgt` term (`−2·v_tgt/t_go`) is a **feedforward** so you arrive *moving*
  rather than stopped.
- **No integral term.** It is PD, not PID — so no bias rejection. A persistent unmodelled
  disturbance (thrust bias, gravity-model error) is not integrated away; what saves it is
  the escalating gains below. Real implementations sometimes add a small trim; on our side
  this is why authority margin (§4a) matters.
- **Gains are derived, not tuned, and they escalate.** `6/t_go²` and `4/t_go` fall out of
  the boundary-value solve as the *unique* gains that hit the target state in exactly
  `t_go`. As the clock runs out they blow up (`Kp → ∞` like `1/t_go²`), enforcing a
  *deadline* rather than an asymptotic decay. That is the signature of terminal guidance /
  finite-horizon optimal control, not of a regulator.
- **Implied damping is constant.** Freeze `t_go` and treat it as constant-gain PD on a
  double integrator: `ω_n = √Kp = √6/t_go`, and

  ```
  ζ = Kd / (2·√Kp) = (4/t_go) / (2·√6/t_go) = 2/√6 ≈ 0.82
  ```

  The `t_go` cancels — the loop keeps the same well-damped character (`ζ ≈ 0.82`, just shy
  of critical) while its bandwidth `ω_n` rises as arrival nears. That constant `ζ` is a
  direct fingerprint of the coefficients `6` and `4`, which came from the `⅙` and `½` in
  the kinematic ladder. (Heuristic — the gains really are moving — but it is why the law
  flies smoothly instead of ringing.)

---

## 4. Pinning down t_go

The law gives `a_cmd` *for a given* `t_go`. Nothing so far fixes `t_go` itself. Count
degrees of freedom: per axis the profile `c₀ + c₁·τ` has two coefficients → six across
three axes; the endpoint constraints (position + velocity) are six. So for any given `T`
the profile is fully determined — but `T` is a **seventh** scalar unknown with no equation.
Pinning it down requires exactly **one more scalar condition**. There are two natural
choices.

### 4a. Closure by thrust margin — the current method

`solve_t_go` (`apollo-powered-descent.md:181`) picks `T` so the commanded thrust *now* sits
at 90% of what the engine can give:

```
|a_cmd(T)| = 0.9 · a_max
```

Because that is the magnitude of a vector (three components under one root), squaring to
clear it makes it a **quartic** in `T` with the axes coupled through the norm — no clean
closed form, so it is solved numerically by bisection over a bracket (`find_zero_crossing`,
`[20, 1200]` s).

- **+** Needs no design inputs — reads `a_max` off the engine. As mass burns off `a_max`
  rises, and re-reading it automatically keeps the 10% reserve intact. Directly guards the
  failure that kills a braking burn: throttle saturation and loss of authority.
- **−** Numeric (bracket, tolerance, "no crossing = abort" edge case). Says nothing about
  the *state* you arrive in.

### 4b. Closure by target acceleration — the Apollo-style quadratic

Specify instead a **target acceleration at arrival**, so the ship reaches the gate with the
engine in a chosen state (smooth hand-off to the next phase). Set the arrival acceleration
`a(T)` — the mirror expression from §2 — equal to a target `a_tgt`:

```
−6·R/T² + (2·v + 4·v_tgt)/T = a_tgt
```

Multiply through by `T²` and collect into standard form:

```
a_tgt·T² − (2·v + 4·v_tgt)·T + 6·R = 0        ← quadratic in T
```

Working one scalar axis (all quantities projected onto it):

```
        (2v + 4v_tgt) ± √[ (2v + 4v_tgt)² − 24·a_tgt·R ]
T  =  ──────────────────────────────────────────────────
                        2·a_tgt
```

- Take the **smallest strictly-positive root** (first time the arrival condition can be
  met). A negative-or-complex result means *no feasible time-to-go for this `a_tgt`* — a
  genuine abort signal, not a nuisance.
- Degenerate case `a_tgt = 0` (arrive at a chosen velocity with zero net acceleration —
  thrust exactly cancelling gravity): the quadratic drops to linear, `T = 3·R/(v + 2·v_tgt)`.

**Over-determination.** The boxed equation is a *vector* equation — three quadratics for
one unknown `T` — which will not agree in general. Resolve it the way the AGC did:
designate one axis (the local **vertical**) as the `t_go`-defining axis, solve `T` from its
target acceleration, and let the other two axes take whatever arrival acceleration the
profile then gives. "A constraint on one component" is literally which scalar quadratic you
solve.

**Why the real AGC equation is a cubic.** Our law matches *two* endpoint conditions
(position, velocity) → linear-in-time acceleration → quadratic `t_go`. Apollo's law matched
*three* (position, velocity, **and** acceleration) as part of every gate's target state,
which needs acceleration quadratic-in-time (position quartic) — one more coefficient. Run
the identical clear-the-denominators step on that higher-order profile and the leading
power comes out one higher: a **cubic** in `t_go`. Same crank, one rung further up the
ladder — exactly as constant-jerk was one rung up from constant-acceleration.

**Does it need precomputed accelerations? No.** Apollo stored a full reference trajectory
(target position, velocity, *and* acceleration at each gate) computed on the ground —
because the AGC could not afford to compute a good `a_tgt` online and mission planning
wanted a verified path. Neither constraint is inherent to the method: the quadratic just
wants *a number* for `a_tgt`, which you can synthesise live. `a_tgt` here is the desired
*net* (total) vertical acceleration at the gate; the thrust to realise it is `a_tgt + g`,
which `guidance_step` already forms. So a small positive (upward) `a_tgt` means "arrive
gently slowing," `a_tgt = 0` means "arrive holding descent rate," and neither needs a
stored trajectory — only local gravity (`body:mu / r²`), computed each cycle. The gate's
`r_tgt`/`v_tgt` are likewise computed live from the site geoposition
(`apollo-powered-descent.md:203`), so this closure keeps the design trajectory-table-free.

---

## 5. Reference implementation (kOS): the quadratic closure

Drop-in alternative to `solve_t_go`. Solves the §4b quadratic on the local vertical and
returns the time-to-go, or `-1` when no feasible positive root exists (an abort, to be
handled the same way `solve_t_go`'s "no crossing" is).

```
// Apollo-style t_go closure. Returns the time-to-go that makes the ship arrive
// at the aim point with net vertical acceleration a_v_tgt (up-positive, m/s^2),
// or -1 if no feasible positive root exists.
function solve_t_go_accel {
  parameter aim_geo.    // geoposition of the aim point
  parameter aim_alt.    // altitude of the aim point above the datum
  parameter v_tgt.      // desired velocity at the aim point (surface frame)
  parameter a_v_tgt.    // desired NET vertical accel at arrival (up-positive)

  local u   is up:vector.                          // local vertical, unit
  local rER is aim_geo:altitudeposition(aim_alt).  // r_tgt - r, ship-relative
  local v   is ship:velocity:surface.

  // project onto the vertical axis -> scalars
  local Rv  is vdot(rER,   u).
  local vv  is vdot(v,     u).
  local vtv is vdot(v_tgt, u).

  local A is a_v_tgt.
  local B is -(2*vv + 4*vtv).
  local C is 6*Rv.

  if abs(A) < 1e-6 {                 // degenerate: a_v_tgt = 0 -> linear
    if abs(B) < 1e-9 { return -1. }
    local t is -C / B.
    if t > 0 { return t. }
    return -1.
  }

  local disc is B*B - 4*A*C.
  if disc < 0 { return -1. }         // no real arrival time -> abort
  local s is sqrt(disc).

  // smallest strictly-positive root of A t^2 + B t + C = 0
  local best is -1.
  for t in list((-B - s) / (2*A), (-B + s) / (2*A)) {
    if t > 0 and (best < 0 or t < best) { set best to t. }
  }
  return best.
}
```

It slots into the gate loop exactly where the thrust-margin solver did — `guidance_step`
is unchanged, only the line that seeds `t_go` differs:

```
// a_v_tgt: net vertical accel at the gate. +0.5 = arrive gently slowing;
// 0 = arrive holding descent rate. No precomputed trajectory — just a design scalar.
local a_v_tgt is 0.5.
set t_go to solve_t_go_accel(aim_geo, aim_alt, v_tgt, a_v_tgt).
if t_go < 0 { /* abort: gate unreachable with this arrival accel */ }

lock steering to lookdirup(a_thrust, ship:facing:topvector).
until t_go < t_handoff {
  set a_thrust to guidance_step(aim_geo, aim_alt, v_tgt, t_go).
  lock throttle to min(1, a_thrust:mag * ship:mass / ship:availablethrust).
  set t_go to t_go - dt.            // decrement by wall-clock; re-solve every ~10 s
  wait 0.
}
```

**Cross-guard.** This closure does not watch `a_max`, so after solving, cheaply check the
implied thrust does not saturate (`a_thrust:mag * mass < availablethrust`); conversely, the
thrust-margin closure should sanity-check its implied arrival acceleration. One extra
evaluation converts a silent failure into a caught one.

---

## 6. Which closure, when

| | Thrust margin (§4a, current) | Target accel (§4b, Apollo) |
|---|---|---|
| Per-solve cost | bisection (~dozen evals) | closed form (quadratic) |
| Design inputs | none (reads engine) | one scalar `a_tgt` per gate |
| Directly guarantees | control authority in reserve | smooth arrival dynamics |
| Silent about | arrival state | whether thrust saturates |
| Adapts to mass burn-off | yes (re-reads `a_max`) | no (ignores `a_max`) |
| Determinism / debug | lower (bracket, no-crossing) | high (formula, one root) |

Neither is globally "better"; they servo `t_go` against different scalars and guard
different failures. On kOS the per-solve cost gap is irrelevant (compute is cheap), so pick
by *what can kill the phase*:

- **Braking (P63)** is authority-limited — the dominant failure is throttle saturation.
  Keep the **thrust-margin** closure; its mass-adaptivity is a real advantage here.
- **Approach (P64)** cares about arrival geometry more than raw thrust. A **target
  vertical acceleration** is the natural closure, and the closed form is cleaner to reason
  about.
- **Terminal (P66)** already uses the velocity-level cousin of this idea (a rate-of-descent
  controller, `apollo-powered-descent.md:214`).

This phase-dependent split *is* what Apollo did — P63/P64/P66 were distinct programs with
distinct targets and closures for exactly this reason.

**Teaching value.** For the chapter, the quadratic is the more teachable closure: a clean
derivation (§4b) with a formula and a single root, no bracket to tune and no "no crossing"
edge case to hand-wave. Even if the shipped flight code keeps the bisection for its
mass-adaptivity, deriving the quadratic is the better way to *show the reader how `t_go`
gets pinned down*. The two goals need not use the same method.

---

## 7. Caveats

- **`t_go → 0` singularity.** The `1/t_go²` gains blow up and the closure equations grow
  ill-conditioned as time runs out. Never fly the law to zero — hand off to the next gate a
  few seconds early (`t_handoff`). This is the same singularity seen from both sides: the
  law diverges, and the equation defining `t_go` loses meaning.
- **Re-solve vs decrement.** `t_go` is *solved* only occasionally (~10 s) and *decremented*
  by wall-clock (`t_go − dt`) in between — not interpolated. The periodic re-solve is what
  sheds accumulated model error; the decrement just runs the clock down from the last
  anchor.
- **Gravity convention.** §2 treats `a(τ)` as total acceleration and subtracts a
  constant `g` at the end. Fine while `g` is roughly constant over the arc (true for a Mun
  braking burn). If it varies appreciably, move to Klumpp's form with a gravity term inside
  the position equation.

---

## References

See the bibliography in `apollo-powered-descent.md:289` (Klumpp *Automatica* 10 / Draper
R-695; Cherry AIAA 64-638; GSOP R-567 §5; D'Souza AIAA GNC 1997). The `t_go` cubic and the
P63/P64 targeting live in Klumpp R-695 and GSOP §5; D'Souza has the cleanest modern `t_go`
derivation and the near-optimality argument for the polynomial law. The guidance cycle time
`Δt = 2 s` (verify against the scans before quoting) is a GSOP §5 / R-695 figure.
