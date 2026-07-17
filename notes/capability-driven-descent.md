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

## The shape: two jobs, one seam

**Planning** runs before DOI, from the parking orbit. It works out the shallowest approach that
clears the terrain, solves the PDI altitude that implies, and **adds a maneuver node**. That is
its whole output.

**Flight control** starts **any time after the DOI burn is executed**. It reads the descent
ellipse it is on, coasts to periapsis, integrates the gravity turn from live state, chops it
into legs, flies them, hands off to terminal.

The seam is the node, and after the burn, the orbit itself.

## Order of work: three scripts, in this order

### 1. The flight controller — `powered_landing.ks`

| | |
|---|---|
| **Given** | `target_lat`, `target_lng` |
| **Arc contract** | `f`, `landing_height`, `speed_handoff`, `turn_budget`, `dt_arc`, `max_steps` |
| **Reads** | its own orbit — periapsis *is* PDI, and `h_pdi` is an observation |
| **Precondition** | the DOI burn is already executed; the ship is on a descent ellipse |
| **Produces** | a landed craft; `flight_log.csv`; miss distance |

It is mostly written. The coast, the guidance law, `solve_t_go`, the leg flyer with its
saturation and attitude guards, terminal descent and the recorder have all flown, and
`integrate_arc` is committed. The work is mostly subtraction — the closed-form planner, the
fixed gates and their constants, and Phase 1's DOI machinery all leave — plus one new function
to chop the arc on pitch, and reading `ship:orbit:periapsis` instead of a parameter. ~800 lines
to ~450, of which one function is unproven.

It depends on neither planner: a node placed by hand will do. So it is testable through the
bridge immediately, from a quicksave mid-coast, which is where the unproven function should be
hammered.

### 2. The simple planner

| | |
|---|---|
| **Given** | `target_lat`, `target_lng`, **`γ`** — the human's judgment |
| **Arc contract** | `f`, `landing_height`, `speed_handoff`, `dt_arc`, `max_steps` |
| **Reads** | the parking orbit |
| **Produces** | **a maneuver node**; the solved `h_pdi`, `X` and lead, logged |

No terrain analysis: the human owns the survey and the risk. Worth building for itself, not as
scaffolding — it makes γ a knob that can be *felt*. Nobody has measured Δv against γ, and the
claim that the corridor is this design's efficiency knob is still an assertion. It also flies
real landings that don't have to be optimal, and it prices open item 1: once a degree of γ costs
something, so does the `clearance` demanded under the coast.

### 3. The smart planner

| | |
|---|---|
| **Given** | `target_lat`, `target_lng`, `max_terrain_height`, `clearance` |
| **Arc contract** | `f`, `landing_height`, `speed_handoff`, `dt_arc`, `max_steps` |
| **Reads** | the parking orbit, and the terrain under the real ground track |
| **Produces** | **a maneuver node** — plus **`γ`**, and the point up-range that forced it |

**γ is an input to piece 2 and an output of piece 3. Everything else is identical**, which is
why 3 extends 2 rather than replacing it: piece 2 already does γ → `h_pdi` → lead → node, and
piece 3 puts a γ solver in front of the same machinery. The flight controller's contract never
changes, so it is flown and trusted before either planner exists.

### The seam is the node *and* the arc contract

The arc parameters appear in all three, and they are not incidental — they must agree. The
planner places PDI a lead angle up-range, and that lead is `X / body:radius` where `X` is the
arc's down-range. If the planner computes `X` for one `f` and the flight controller flies
another, PDI is in the wrong place and the descent misses by the difference. The node alone does
not carry this; it is a shared contract riding beside it.

Which raises a question the design has not answered: `X` is a *consequence* of the state at PDI
and the thrust, not something the ship chooses. Any DOI error moves PDI, and then the arc
integrated from the real state ends somewhere other than the site. Either the planner's `X` must
be exact — it will not be — or the flight controller solves `f` at PDI so that the arc lands on
the site, making `f` an output of the flight controller rather than a parameter of it. That
collides with `f`'s other job as the authority reserve, which wants it pinned near 0.85. See
open item 3.

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

**Two regions, two rules.** The flown path either side of PDI is two different curves, and
each needs its own argument. PDI is the seam.

**The arc is certified by a chord.** `dh/dx = tan(pitch)`, and the gravity turn leaves PDI at
`pitch = 0` and steepens monotonically, so `h(x)` is concave and **lies above the chord joining
its endpoints**. Certifying the straight ray from the site to PDI certifies the flown arc —
geometrically, not statistically, and with no sampling of the trajectory at all.

**The coast is certified by walking it.** Up-range of PDI the ray says nothing useful: PDI is
periapsis, so the coast leaves it flat (`dh/dx = 0`) and climbs quadratically while the ray
climbs linearly. The ray sits above the coast for `2·r_pe·(1+e)/e · tan γ` — on Minmus, over a
hundred kilometres, most of the up-range hemisphere. It would certify a mountain the coast would
fly into.

So the coast gets a **flat clearance rule**: sample it, and require `h − terrain ≥ clearance`
throughout. Flat is correct here for the same reason it was wrong over the arc — the coast is
not trying to land, so clearance under it is a pure hazard question with no feedback and no
circularity. The walk runs between PDI and the point where the coast climbs past
`max_terrain_height`, which on the flight-7 geometry is ~50 km, about a quarter of the coast;
beyond it nothing on the body can reach the ship. The scan's direction is an implementation
choice, not a design one — anchoring at PDI and stepping outward lets the loop find its own end
rather than being told where to start.

The coast is on rails, so this is deterministic *before the burn*: the planner walks `nd:orbit`,
the patch KSP predicts, and adjusts the node before anything is committed.

**The corridor is a descent angle γ**, because γ is a number about *approaches* and means the
same thing at Minmus, the Mun, or Tylo, where a PDI altitude is a number about one body.

**γ is the steepest ray any obstacle demands:**

```
γ = max over x of  atan( (terrain(x) − site_elev − landing_height) / x )
```

Walk the ground track back from the site, sample `terrainheight`, keep the running max. This is
the obstacle clearance surface from instrument approach design, and it ends itself once the ray
climbs past `max_terrain_height`.

**The sweep follows the real ground track**, not an assumed equator, because the track is what
the ship actually flies over and an inclined approach crosses different country. That couples γ
to the node: the ground track depends on the orbital plane and on *when* the ship passes, so it
depends on the node's timing, which depends on `h_pdi`, which depends on γ. The sweep therefore
joins the same fixed point as everything else rather than standing outside it. It also means the
body's rotation must be carried explicitly — `geoposition_at`, not the bare ellipse — since the
ground runs east underneath an inertial track.

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
`speed·sin(pitch)`.

**Chop the arc on pitch, every `turn_budget` degrees.** Not on down-range: the arc ends
vertical, so `x` stops advancing while the ship is still descending and the last chord would
never close. Not on time either: the arc's *duration* scales with 1/thrust — ~12 s on a TWR-34
craft, ~200 s on the TWR-2 design craft, a 17× spread — while both sweep the same 90°. A fixed
`dt` would buy two chords of 45° on one craft and forty of 2° on the other, and a 45° chord is
not an approximation of that arc, it is a different trajectory. Pitch is the variable a chord's
fidelity actually depends on, so step on pitch and the spread disappears.

**`turn_budget` is 15°, so every descent has six chords** — every body, every craft, because
every gravity turn sweeps the same 90° from level at PDI to vertical at the end. The count is a
constant we chose, not an output of the vehicle.

**Where 15° comes from, and why there is only one budget.** The arc's virtue is thrust exactly
retrograde. Across a leg the required acceleration holds magnitude `a_T` while its direction
rotates by `turn_budget`, so the tip of that vector traces a circular arc of radius `a_T`; the
law, running acceleration linearly in time, flies its chord. The largest gap between chord and
arc is the sagitta:

```
a_error = a_T · (1 − cos(turn_budget / 2))
```

`a_error` is therefore not a second knob — it falls out of `turn_budget` and the thrust. The
bracketed term is also the fraction of thrust not going retrograde, which is what the chord
costs: 0.9% at 15°, 3.4% at 30°, 7.6% at 45°, 29% if the whole arc is flown as one chord. The
job is to recover ~20%, so 15° spends under 1% of the braking to buy six well-behaved legs, and
30° starts charging real money for two fewer.

**`t_go` for a segment is a difference of arc sample times**, because the arc carries `t`.

**Discrete legs and continuous tracking are the same machine**, separated only by `dt`. What was
rejected was managing guidance through `lock` and `when`, which is a different question.

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

1. **`clearance` for the coast rule.** How much gap to demand under the coast is the one number
   still owned by judgment rather than derived — it is how far the terrain model is trusted. The
   coast genuinely competes with PDI for the binding constraint: measured on a 2937 m periapsis
   over the Great Flats, clearance at PDI was 2937 m and at −240 s up-range **2947 m**, a
   ten-metre margin over the flattest ground on the body. Tilt onto any relief and the coast
   binds first.
2. **Does the sagitta argument hold in flight?** `turn_budget = 15°` assumes the law's command
   really does interpolate linearly between a leg's endpoint accelerations, as constant jerk
   says it should — but the law also feeds back on live state every tick, so the real command is
   not exactly that chord. This is cheap to settle: `pitch` and `cmd_pitch` are already in the
   recorder, have never flown, and are precisely the chord-versus-arc error. Fly 15° and read
   them.
3. **Is `f` a parameter or an output of the flight controller?** The arc's down-range `X` is a
   consequence of the PDI state and the thrust, so a DOI that misses by a little leaves an arc
   that ends off-site. Solving `f` at PDI to close the gap is the obvious lever, and it is the
   surviving half of the old "throttle becomes the free variable" idea — but `f` is also the
   authority reserve the closed-loop law spends absorbing error, and that job wants it pinned.
   Both cannot be true without bounds on the solve.
4. **Does the in-flight closure re-solve the quadratic, or decrement plan time?** The arc carries
   `t`, so plan time is known — but the arc is nominal and the ship is actual. Re-solving keeps
   the design honest and makes plan-time-versus-solved-time a free divergence check; decrementing
   re-imports the stored path. Decides whether an arrival-acceleration scalar survives at all.
5. **`integrate_arc` has no ground floor** — it integrates until the speed is gone regardless of
   altitude.
6. **Constant `a_thrust` across the arc.** `f·a_max` is sampled once, but mass drops through a
   ~240 m/s descent, so real acceleration climbs and the arc under-predicts. Empirical; the bridge
   can answer it.
7. **`dt_arc` and `speed_handoff`** trade accuracy against the IPU budget.
8. **Planner track sample spacing** trades IPU budget against stepping over a spire.
9. **Throttle deadzone.** Pinned by Schuyler, still undiscussed.
10. **Radar floors** need rederiving against a solved PDI rather than a 10 km one.
