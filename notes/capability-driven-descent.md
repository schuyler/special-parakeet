# Capability-driven descent

*A spike, not a book chapter. `reference/original/powered_landing.ks` is a working,
inefficient lander flown to 0 m on Minmus; this note is the plan for making it efficient,
and the register of every engineering choice that plan rests on. Companions:
`apollo-powered-descent.md` (what Apollo's programs did) and `klumpp-guidance-derivation.md`
(the guidance law's math). Both stand as written; this note assumes them.*

## The goal

Minimize Δv for a soft, on-target landing from whatever orbit we're in.

**Baseline to beat: 244 m/s, 0 m miss, on Minmus. The impulsive floor is ~180–200.** That ~50
m/s gap is the whole job, and it is the only number here — a design goal of "minimize Δv" with
nothing attached to it cannot be failed.

244 is already down from **310**, so it is a worked number rather than a first attempt: the
cheap Δv has been taken, and what remains is the structural half.

The gap has one cause: **an arbitrarily chosen PDI altitude is inefficient on a high-TWR craft.**
Pick PDI high and the braking burn is pinned by the drop rather than by the engine, so a craft
with thrust to spare throttles down, flies a long shallow brake, and pays for the extra minutes
in gravity loss. That is why mission planning splits out — to solve the optimal PDI instead of
guessing it.

The design point is a **TWR ~2** craft; testing happens on a TWR-34 one. Failures hide on
overpowered craft, so both ends of the range matter when reading a result.

The guidance law is not the problem and never has been. Every failure has been in what it was
asked to do.

## The shape: two programs, one seam

**A planner** runs before DOI, from the parking orbit. It surveys the terrain up-range of the
site, computes the shallowest approach that clears it, solves the PDI altitude that implies,
and **adds a maneuver node**. That is its whole output.

**A flight controller** starts **any time after the DOI burn is executed**. It reads the descent
ellipse it is on, coasts to periapsis, integrates the gravity turn from live state, tessellates
it into gates, flies them, hands off to terminal.

The seam is the node, and after the burn, the orbit itself.

---

## Choices: architecture

**Split planner from flight controller** because they need different information at different
times, and the split makes `h_pdi` an *observation* — the flight controller reads
`ship:orbit:periapsis` rather than being told. Nobody types a PDI altitude again.

**The seam is the node** because it lets the flight controller start from a quicksave taken
mid-coast, which makes the descent re-runnable through the bridge without a launch or a burn.
A campaign that costs one flight per number learned stops costing that.

**The planner owns the survey** because it is the program that knows the terrain profile, and
a flight controller that surveys is a flight controller with a survey bolted to it.

## Choices: the arc

**Fly a gravity turn in reverse** — hold thrust surface-retrograde and let gravity rotate the
velocity vector from horizontal to straight down, arriving vertical over the target — because
retrograde is the minimum-Δv direction to null a velocity vector; because it kills vertical
speed *concurrently* with horizontal, while the craft is fast and centrifugal support makes
vertical cheap; and because it spends the least time slow, which is where gravity loss accrues.

That the arc *rises* just after PDI is a gift: it is highest up-range where terrain is only
modeled, and lowest over the site where radar is truth. The margin is real: at 170 m/s over
Minmus, `v²/r ≈ 0.46` against `g ≈ 0.445`, so just after PDI free fall barely descends at all.

**Integrate the arc live, once, at PDI** (Euler, `dt_arc`) because we have compute Apollo
didn't, and integrating gives the drop, the down-range, the duration, and the state at every
point on the path from one loop. Everything downstream is a read off those samples.

**Recompute `g` and `r` from `h` every step** because it costs two lines and `g` moves ~9% over
the low-TWR craft's drop.

**Carry down-range as the ground-track angle `theta`, reported as `theta·body:radius`,** because
the lead is an angle at the body's centre; the difference from `∫v·cos γ dt` is ~5% on Minmus.

**Stop the arc at `speed_handoff` above zero** because the turn rate carries `speed` in its
denominator and is singular at zero; stopping early keeps it finite and hands the rest to the
terminal controller.

**`max_steps` bounds the walk**, and **the arc closed iff `arc:length < max_steps`** — the loop
exits on speed or on budget, and the length is what distinguishes them. (The last *sample*'s
speed is always above `speed_low`: the sample is taken at the top of the loop and the step
applied after.)

## Choices: the planner

**Certify a chord, not the path.** `dh/dx = tan(pitch)`, and the gravity turn leaves PDI at
`pitch = 0` and steepens monotonically, so `h(x)` is concave and **lies above the chord joining
its endpoints**. Certifying the straight ray from the site to PDI certifies the flown arc —
geometrically, not statistically. This is what lets the planner stop comparing trajectories to
terrain.

**The corridor is a descent angle γ**, because γ is a number about *approaches* and means the
same thing at Minmus, the Mun, or Tylo, where a PDI altitude is a number about one body.

**γ is the steepest ray any obstacle demands:**

```
γ = max over x of  atan( (terrain(x) − site_elev − landing_height) / x )
```

Walk the ground track back from the site, sample `terrainheight`, keep the running max. This is
the obstacle clearance surface from instrument approach design. It needs no arc, no orbit, and
no node, and it ends itself once the ray climbs past `max_terrain_height`.

**`max_terrain_height` is a planner parameter** because kOS reports `TERRAINHEIGHT` per
`GeoCoordinates` but has no body-global maximum, and hard-coding one per body in the *flight*
script would break the generality rule. Minmus peaks ≈ 5.7 km.

**PDI falls out of γ:** `h_pdi = tgt:terrainheight + landing_height + X·tan γ`, with `X` the
arc's down-range — a one-dimensional fixed point over pure arithmetic. The survey supplies the
slope; the software still solves the altitude. That is where the Δv lives.

**The DOI node verifies itself.** Plan the burn, read the *predicted* orbit's periapsis
longitude (`nd:orbit` through `time_of_periapsis` + `geoposition_at`), compare to desired, feed
the error back into the burn longitude, re-plan (≤4 attempts, 0.2°). This is needed because the
parking orbit's radial velocity (~3.5 m/s at e≈0.024) is not small against a ~10 m/s DOI burn:
the burn point isn't the new apoapsis and periapsis isn't 180° away. Without it PDI sat ~7°
up-range, consistently.

**Lead, speed and duration must describe the same burn.** Hand the law a boundary-value problem
that violates `distance = speed × time` and it will still solve it — by diving or by reversing.
The lead comes from the arc's own down-range, so it is consistent by construction.

## Choices: the gates

**A gate is a sample off the arc.** Nothing physical happens at a gate; it is the resolution at
which the ideal path is resampled so the law can track it. `h` gives the aim altitude, `x` the
aim offset, and `speed` with `pitch` give the arrival velocity as `speed·cos(pitch)` and
`speed·sin(pitch)`. The count is an **output**, one gate on a gentle body and more on a harsh
one, because the law flies a chord between endpoints and the only question is how far that
chord may stray from the arc.

**Two per-leg budgets, whichever trips first:** `a_error`, the measured gap between the law's
command and what the arc needs; and `turn`, the pitch swept since the last gate, which caps
thrust mispointing across the chord. The heading budget trips at the **pitchover**, which is
where Apollo put its high gate for the commander's eyes — same point on the arc, chosen by
geometry instead.

**`t_go` for a segment is a difference of arc sample times**, because the arc carries `t`.

**Discrete gates, not continuous tracking**, because `lock steering`/`lock throttle` already
re-solve the *command* every physics tick — only the target is discrete, and keeping it so
avoids re-importing the stored-path dependence this design exists to remove.

**The terminal handoff is not a gate.** It is where the law's gains diverge (`t_go → 0`) and the
rate-of-descent controller takes over. `landing_height` is a parameter, default well under
Apollo's unscaled 150 m — ~50 m, floor ~20–30 — because terminal is a near-hover and every metre
is gravity loss — terminal costs ~10% of the descent's budget. It is the *same* number the
planner's ray clears the site by: one quantity, two programs.

## Choices: flight safety

**Attitude-gated throttle**: closed unless the ship faces within 30° of the commanded thrust
vector, because mis-pointed thrust is the energy source that sustains a guidance limit cycle.
Nominally never fires.

**Per-gate radar floors → `emergency_land`**: abandon the target, kill velocity, land where you
are — because below the floor there is no altitude left to hand a pilot. (The floors themselves
were chosen when PDI was 10 km and need rederiving against a solved PDI.)

**Feasibility is `closed`, checked during the coast**, because integrating from the *achieved*
post-burn state and aborting with the whole coast still ahead is a better position than
aborting pre-DOI on a prediction.

**Saturation cross-guard** stays: sustained full-thrust demand means the gate is unreachable.

## Choices: margin

**`f`** is the design throttle as a fraction of `a_max`. The reserved `(1 − f)` is both the
authority the closed-loop law spends absorbing error and the ignition-timing cushion — the
law's error-absorption *is* the robustness, so there is no literal `f = 1` hoverslam. Start
`f ≈ 0.85` and walk it with telemetry. **`brake_throttle` is deleted**; `f` is the one name.

## Conventions

- `pitch` is degrees **above** the horizon (`aero.ks:42`; kOS's `heading()` agrees).
- `d_`/`d` means **difference**, never derivative. The arc carries increments; `dt` is family.
- State is `speed`, not `v`, because `dv` means delta-v everywhere.
- `a_error`, not `sag`.
- Pass **orbit objects**, not scalars — `orbital_speed(h, orbit)` reads the sma off the orbit.
- Comments state what a quantity *is*, for a reader one semester into calculus.

## Process

- **No claim about flight behavior without telemetry that supports it**, and no fix implemented
  until its hypothesis has been checked against a log. This rule exists because reasoning from a
  plausible model and skipping the log has produced fixes for problems that were not there.
- **Predictions are testable signatures** — which column, which value — never promised outcomes.
- **One instrumented change per flight.**
- **The CSV is the witness.** `flight_log.csv`, one row/s, planning numbers as `#` metadata, so
  a flight is auditable afterward instead of remembered. The corridor is now the design's stated
  efficiency knob and should be the best-instrumented number in the planner: γ and the point
  up-range that forced it, the solved `h_pdi` and the `X` behind it, and the periapsis the
  flight controller actually observed against the one the planner intended.

## kOS facts, verified 2026-07-17

Settled from the docs and from the live game through `util/kos_bridge.py`. Don't re-litigate.

- `nd:ORBIT` is the post-burn orbit patch; it **survives into and out of a `lexicon`**, and
  `remove` works on a node fetched back out.
- `orbit_at(t, orbit_)` — kepler's, which wins by load order — takes a **TimeStamp, not a
  scalar**. It works on a node's orbit, walked backward. `place_doi` returns `t_pdi` as a
  TimeStamp already.
- `geoposition_at(t, orbit_, pos)` works before now; passing `pos` matches letting it recompute.
- `body:angularvel` is **radians**/sec, so `|ω|·r ≡ 2π·r/T`.
- `geopositionlatlng(lat, lng)` works for arbitrary lat/lng, off-rails.
- **No body-global max terrain suffix exists.**
- Function forward references are fine.

**Two live hazards, not this design's to fix:** `orbital_speed` is defined twice with reversed
signatures (`common.ks:93` `(orbit_, altitude_, apo, peri)` and `kepler.ks:90` `(alt_, orbit_)`)
and `powered_landing.ks` runs both — kepler's wins. And `common.ks:93`'s default `apo is
orbit:apoapsis` reads the *global* `orbit`, not its own parameter.

## Open

1. **The up-range wedge.** The certified ray climbs at `tan γ` past PDI; the coast leaves
   periapsis horizontally and curves up quadratically, so terrain just up-range could be
   certified yet struck. Measured on a 2937 m periapsis over the Great Flats: clearance at PDI
   2937 m, at −240 s **2947 m** — a ten-metre margin over the flattest ground on the body. The
   coast genuinely competes for the binding constraint. Bounded, computable, unbuilt.
2. **Does the planner sample the real ground track, or assume the equator?** The flight script
   assumes equatorial. The honest version needs the orbit, and then the planner isn't purely
   terrain geometry.
3. **The tessellation budgets** `a_error_budget` and `turn_budget` set the gate count. Unbuilt,
   and the claim that they "fall out" of a budget rule has the same shape as several claims that
   turned out to be wrong. Distrust it until it's measured.
4. **Does the in-flight closure re-solve the quadratic, or decrement plan time?** Decides whether
   an arrival-acceleration scalar survives at all.
5. **`integrate_arc` has no ground floor** — it integrates until the speed is gone regardless of
   altitude.
6. **Constant `a_thrust` across the arc.** `f·a_max` is sampled once, but mass drops through a
   ~240 m/s descent, so real acceleration climbs and the arc under-predicts. Empirical; the bridge
   can answer it.
7. **`dt_arc` and `speed_handoff`** trade accuracy against the IPU budget.
8. **Planner track sample spacing** trades IPU budget against stepping over a spire.
9. **Throttle deadzone.** Pinned by Schuyler, still undiscussed.
10. **Radar floors** need rederiving against a solved PDI rather than a 10 km one.
