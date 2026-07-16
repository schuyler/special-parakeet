# Implementation guide: targeted powered descent, the Apollo way

*Working draft feeding chapters 11–12. Structured like a book section but written as an
implementation guide; the prose gets rewritten to fit the narrative later. Snippets are
illustrative, not tested code — the real routines land in `lib/` when the chapters are
drafted.*

## Why copy Apollo

Every earlier attempt in `reference/` ran into the same wall: on an efficient, shallow
deorbit trajectory, the script has to predict the terrain height at its impact point, and
that impact point is (a) at the end of a shallow path where small timing errors move it
kilometers downrange, and (b) moving, because the braking burn itself drags the impact
point around while the prediction chases it. `script/landing.ks` caches and refreshes
terrain lookups; `original/land_at_periapsis.ks` offsets a pseudo-target by half the burn
time. Both are patches on the same geometry.

Apollo's descent profile makes the problem disappear instead of solving it:

1. The descent orbit's periapsis stays **above** the surface, so during the coast there is
   no impact point and nothing to predict.
2. The braking phase doesn't aim at the ground at all. It aims at an **aim point in the
   sky** — a position, velocity, and time above the landing site. Terrain never enters the
   guidance equations.
3. By the time the ship is low enough for terrain to matter, it is descending steeply over
   the one patch of ground whose elevation is known *exactly* before the mission starts:

   ```
   local tgt is body:geopositionlatlng(0.7, -23.4).
   print tgt:terrainheight.    // known before we deorbit. no prediction.
   ```

   And `alt:radar` is now ground truth, because the ground directly below *is* the site.

The price is a few tens of m/s of gravity loss against a theoretically optimal shallow
suicide burn — call it 5% of a ~650 m/s Mun descent budget. The shallow profile spends at
least that much carrying timing margin against terrain uncertainty, so the trade is fake
efficiency for real robustness.

## The Apollo sequence, and ours

| Apollo | What it did | Our phase | kOS state |
|---|---|---|---|
| Plane alignment | CSM plane change so the orbit passed over the site | 0 | `ALIGN` |
| DOI (Descent Orbit Insertion) | −23 m/s retrograde, half an orbit before the site, dropping perilune to 15.2 km | 1 | `DOI` |
| Coast | Half an orbit in the descent ellipse | 2 | `COAST` |
| PDI + braking (P63) | Ignition at perilune, ~480 km up-range; ~8.5 min throttled burn from 1,695 m/s, guided to "high gate" | 3 | `BRAKE` |
| Approach (P64) | High gate (~2,200 m) to low gate (~150 m); pitch-over so the crew could see the site | 4 | `APPROACH` |
| Terminal descent (P66) | Vertical, radar-driven rate-of-descent control to touchdown | 5 | `TERMINAL` |

Phases 3 and 4 run the *same guidance law* with different targets — that was true in the
AGC too. The phase boundaries ("gates") are just target states handed to the law.

## Two kOS facts the whole design leans on

**Fact 1: `geoposition` vectors are body-fixed and free.** `tgt:position` and
`tgt:altitudeposition(h)` return ship-relative vectors to a point that rotates with the
body. During any closed-loop phase, guidance recomputes every tick against these vectors
and against `ship:velocity:surface` — the rotating frame is handled implicitly, no
longitude bookkeeping. Rotation only has to be reasoned about explicitly for *long
predictions* (placing the DOI burn), which is exactly where `landing_v2`'s
rotation-corrected machinery already works.

**Fact 2: the guidance loop replaces the ignition-time problem.** The impulsive
`burn_duration`-style suicide-burn formula fails for the braking phase because a 2–8
minute burn covers a large fraction of the descent arc. Rather than integrate the powered
trajectory to find a perfect open-loop ignition time, we recompute the commanded
acceleration continuously; ignition timing only needs to be roughly right, and every
error — timing, thrust, mass model — is absorbed as it occurs.

## Phase 0 — Get the ground track over the site

On the Mun and Minmus (slow rotators, equatorial parking orbits for now): either burn a
small normal-direction correction, or simply wait — the body rotates the site under the
orbital plane twice per rotation. For an equatorial orbit and a low-latitude site, skip
the plane change entirely and treat it as a longitude-timing problem, which DOI placement
handles. The general inclined-target case is chapter 12 exercise material, not core path.

## Phase 1 — DOI

A single retrograde burn placed so the resulting periapsis (the PDI point) sits a chosen
lead angle *up-range* of the target. Vis-viva gives the magnitude; the placement is the
same `time_to_longitude` / `time_to_closest_approach` machinery from `reference/core/`
and `reference/landing_v2/`:

```
// Drop periapsis to h_pdi, placed lead_deg up-range (west, for a prograde
// equatorial orbit) of the target's longitude.
function plan_doi {
  parameter tgt_geo, h_pdi, lead_deg.

  // The burn happens half an orbit before periapsis.
  local burn_lng is wrap_longitude(tgt_geo:lng - lead_deg - 180).
  local t_burn is time_to_longitude(burn_lng).   // absolute TimeStamp, not a duration

  local r_burn is (positionat(ship, t_burn) - body:position):mag.
  local r_pe is body:radius + h_pdi.
  local sma is (r_burn + r_pe) / 2.
  local v_new is sqrt(body:mu * (2 / r_burn - 1 / sma)).
  local v_old is velocityat(ship, t_burn):orbit:mag.

  return node(t_burn:seconds, 0, 0, v_new - v_old).
}
```

Two refinements, both already prototyped in `reference/`:

- **Rotation during the coast.** The site moves east while we coast half an orbit, so aim
  at where it *will* be: compute the coast duration from the node, advance `tgt_geo:lng`
  by `coast_time * 360 / body:rotationperiod`, and re-plan once. One iteration converges
  for anything as slow as the Mun.
- **Fine-tuning by search.** For sub-degree placement, wrap the whole plan in
  `minimize()` over `lead_deg`, scoring the eventual predicted PDI point against the
  desired one — the exact structure of `landing_v2/calculate_deorbit_burn.ks`, except the
  cost function measures the *periapsis* location, not a ballistic impact point.

**Choosing the numbers.** Perilune ~15 km scaled to KSP: 8–12 km works on the Mun
(terrain reaches 7 km; 12 km clears everything), 4–6 km on Minmus. Lead angle: PDI far
enough up-range that the braking burn's downrange travel ends at the site. Braking
travel ≈ v_pe² / (2·a_horiz); on the Mun (v_pe ≈ 570 m/s, local TWR 2 → a ≈ 3.3 m/s²)
that's ≈ 50 km, or ~14° of arc — so a lead angle of ~15° is the right starting guess, and
guidance absorbs the slop.

## Phase 2 — Coast, monitor, correct

Nothing burns here. The old `predict_impact` from `core/impact.ks` gets demoted to an
instrument: it answers "if the engine never lights, where do I hit?" — which is the abort
question, worth having on screen, not the targeting mechanism. If the predicted PDI point
drifts (it shouldn't, on rails), a meter-per-second trim is cheap here and expensive
later.

## Phases 3–4 — The guidance law (P63/P64)

This is the heart, and it's Klumpp's Apollo descent guidance in its simplest form.
Assume commanded acceleration varies linearly over the remaining time-to-go `t_go`.
Requiring position and velocity to match the aim point at `t_go` fixes the polynomial,
and evaluating it *now* gives the commanded total acceleration:

```
a_cmd = 6·(r_tgt − r)/t_go² − (4·v + 2·v_tgt)/t_go
```

per axis, as vectors. Subtract gravity to get the thrust demand. In kOS, with the aim
point expressed as a geoposition plus altitude, the frame handling collapses to almost
nothing:

```
// One tick of quadratic guidance. Returns the thrust-acceleration vector.
function guidance_step {
  parameter aim_geo.   // geoposition of the aim point
  parameter aim_alt.   // altitude of the aim point above the datum
  parameter v_tgt.     // desired velocity at the aim point (surface frame)
  parameter t_go.      // time remaining to reach the aim point

  local r_err is aim_geo:altitudeposition(aim_alt).      // ship-relative, body-fixed
  local vel is ship:velocity:surface.    // "v" would shadow the builtin V()
  local a_cmd is 6 * r_err / t_go^2 - (4 * vel + 2 * v_tgt) / t_go.

  local g_vec is body:position:normalized * (body:mu / body:position:mag^2).
  return a_cmd - g_vec.
}
```

The loop that flies it:

```
lock steering to lookdirup(a_thrust, ship:facing:topvector).
until t_go < t_handoff {
  set a_thrust to guidance_step(aim_geo, aim_alt, v_tgt, t_go).
  lock throttle to min(1, a_thrust:mag * ship:mass / ship:availablethrust).
  set t_go to t_go - dt.   // decremented each tick; re-solved occasionally
  wait 0.
}
```

**Solving for t_go.** The AGC solved a cubic; we can be lazier. At phase start, pick
`t_go` so the commanded thrust sits at ~90% of what the engine can give (the 10% is the
authority margin guidance spends absorbing errors — Apollo throttled P63 at ~94%):

```
function solve_t_go {
  parameter aim_geo, aim_alt, v_tgt.
  local a_max is ship:availablethrust / ship:mass.

  function overdrive {
    parameter t.
    return guidance_step(aim_geo, aim_alt, v_tgt, t):mag - 0.9 * a_max.
  }
  return find_zero_crossing(overdrive@, 20, 1200, 0.5).
}
```

Then decrement `t_go` by wall-clock each tick and re-solve every ~10 s to shed
accumulated error. Two guard rails: (1) the law divergences as `t_go → 0` (division by
`t_go²`), so hand off to the next phase when `t_go` drops below a few seconds, never let
it reach zero; (2) if the throttle demand saturates at 1 for more than a few seconds,
`t_go` is too short or the craft is under-powered — that's an abort condition, not
something to ride out.

**The gates are just parameter sets.** P63 targets high gate; on arrival, P64 is the same
loop with new targets and a fresh `t_go`:

| Gate | Position | Velocity target | Apollo analog |
|---|---|---|---|
| High gate | ~2,000 m above `tgt:terrainheight`, ~2 km short of the site | ~60 m/s forward, −30 m/s vertical | P63 → P64, ~2,200 m |
| Low gate | directly above the site, 150 m up | 0 horizontal, −5 m/s vertical | P64 → P66, ~150 m |

Offsetting high gate *short* of the site keeps the approach-phase path steep (Apollo flew
15–25° so the commander could see the site out the window; we keep it for terrain
clearance and because a steep path makes the final phase nearly vertical). The pitch-over
between braking attitude (near-retrograde) and approach attitude falls out of the law on
its own — no attitude scripting.

## Phase 5 — Terminal descent (P66)

Horizontal velocity is already near zero, altitude is small, the ground below is the
surveyed site. This is the phase the old scripts already did well, and the impulsive
suicide-burn math is finally valid here if wanted — but a rate-of-descent controller is
simpler and is what P66 actually was:

```
local g0 is body:mu / body:radius^2.
local lock v_ref to -min(5, max(2, alt:radar / 10)).   // 10% of height, floor at 2 m/s,
                                                       // capped at 5 m/s for continuity
                                                       // with the low gate's arrival rate
lock throttle to (g0 + 0.3 * (v_ref - verticalspeed)) * ship:mass
                 / max(0.001, ship:availablethrust).
lock steering to lookdirup(
  up:vector - 0.1 * vxcl(up:vector, ship:velocity:surface), ship:facing:topvector).
```

The steering line holds the nose up while gently tipping against any residual horizontal
drift (`vxcl` projects it out of the vertical). Gear out at low gate, throttle to zero at
contact (`ship:status = "LANDED"` or `verticalspeed > -0.1` with `alt:radar < 5`), wait
for settle, release controls.

## The state machine

Each phase is a function that runs until its exit condition and returns; the mission
script is just the sequence. No nested `when` triggers — the earlier scripts showed how
hard those are to reason about (four levels deep in `original/landing.ks`).

```
runoncepath("lib/descent_guidance.ks").

local tgt is body:geopositionlatlng(0.7, -23.4).

align_plane(tgt).
execute_node(plan_doi(tgt, 10000, 15)).
coast_to_pdi(tgt).                       // warp + monitor; abort instrument on screen
fly_gate(high_gate(tgt)).                // BRAKE   (P63)
fly_gate(low_gate(tgt)).                 // APPROACH (P64)
terminal_descent().                      // TERMINAL (P66)
```

## Numbers to design against

| | Mun | Minmus |
|---|---|---|
| μ | 6.514×10¹⁰ m³/s² | 1.766×10⁹ m³/s² |
| Radius / surface g | 200 km / 1.63 m/s² | 60 km / 0.49 m/s² |
| Parking orbit | 30 km, v ≈ 532 m/s | 20 km, v ≈ 149 m/s |
| DOI (to PDI altitude) | ≈ −12 m/s (pe 10 km) | ≈ −8 m/s (pe 5 km) |
| Speed at PDI | ≈ 569 m/s | ≈ 173 m/s |
| Braking downrange (TWR 2) | ≈ 50 km (~14°) | ≈ 15 km (~14°) |
| Descent budget w/ margin | ~700 m/s | ~240 m/s |

(Local TWR ≥ 2 is the design floor — below ~1.5 the braking phase has no vertical
authority left after canceling horizontal velocity, and `solve_t_go` will tell you so by
finding no crossing.)

## Test ladder

Measure, don't assume — each stage logs predicted-vs-actual before the next is trusted:

1. **Guidance law on a hover rig** (Minmus, launch to 2 km, fly to an aim point 1 km
   away): does `guidance_step` converge on a target state at all?
2. **Terminal descent alone** from a low hover — this is also the chapter 11 untargeted
   landing, minus targeting.
3. **Full profile on Minmus**, log the state error at each gate (position, velocity at
   high gate and low gate) — these residuals are the score.
4. **Same script on the Mun** — the difficulty step is gravity, not new code.
5. **Precision run**: land, plant flag, re-fly, land within sight of the flag. Miss
   distance is the chapter 12 headline number.

Failure modes worth demonstrating on purpose (book material): PDI ignition 30 s late
(guidance absorbs it — compare throttle traces); perilune below a mountain range (the
coast-phase abort instrument catches it); TWR 1.3 (t_go solver fails at DOI time, on the
ground, before anything is committed).

## References

Primary and secondary sources on the Apollo descent, ordered by usefulness to an
implementer. Between them, Klumpp + Bennett + Eyles can source essentially every Apollo
claim the chapters will make. Scans: NTRS (ntrs.nasa.gov) for the NASA reports, the
Virtual AGC document library (ibiblio.org/apollo) for the Draper reports, GSOPs, and
flight code.

**The guidance law**

- Klumpp, A. R., **"Apollo Lunar-Descent Guidance,"** *Automatica* 10, pp. 133–146
  (1974). Longer version: MIT Charles Stark Draper Laboratory report **R-695** (1971).
  The source for everything in phases 3–4 of this guide: the polynomial guidance
  derivation, the t_go cubic (we bisect instead), P63/P64 targeting, the gate structure,
  throttle margin. Readable; start here.
- Cherry, G. W., **"A General, Explicit, Optimizing Guidance Law for Rocket-Propelled
  Spaceflight,"** AIAA 64-638 (1964). Origin of "E-guidance" — explicit computation from
  current state rather than tracking a stored trajectory. Klumpp's law descends from it.

**The trajectory design**

- Bennett, F. V., **"Apollo Lunar Descent and Ascent Trajectories,"** NASA TM X-58040
  (1970), and the Apollo Experience Report **"Mission Planning for Lunar Module Descent
  and Ascent,"** NASA TN D-6846 (1972). The real profile numbers — DOI, perilune, PDI
  range, gate states — and the steepness-vs-terrain-vs-δv reasoning. Sanity-check the
  numbers table above against these.

**The spec and the code**

- MIT Instrumentation Laboratory, **GSOP R-567, Section 5 (Guidance Equations)** — the
  flight software spec for P63/P64/P66; every equation the AGC flew, in implementable
  form.
- **LUMINARY source code**, Virtual AGC project (ibiblio.org/apollo; mirrored on
  GitHub) — `THE_LUNAR_LANDING.agc`, `BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc`. The
  servicer loop and t_go logic in real code.

**Context (sidebar material)**

- Eyles, D., ***Sunburst and Luminary: An Apollo Memoir*** (2018); also **"Tales from
  the Lunar Module Guidance Computer,"** AAS 04-064. By the author of the landing
  phases of the flight software; the 1201/1202 alarms and the Apollo 14 abort-switch
  patch, from the inside.
- O'Brien, F., ***The Apollo Guidance Computer: Architecture and Operation*** (Springer
  Praxis, 2010). P63→P66 mode logic and displays at the systems level.
- Mindell, D., ***Digital Apollo*** (MIT Press, 2008). The human/automation split; why
  P66 existed at all.

**Modern follow-ons**

- D'Souza, C., **"An Optimal Guidance Law for Planetary Landing,"** AIAA GNC (1997).
  Shows the Apollo polynomial law is near-optimal; clean t_go derivation; bridge to the
  modern literature.
- Açıkmeşe, B. & Ploen, S., **"Convex Programming Approach to Powered Descent Guidance
  for Mars Landing,"** *JGCD* (2007). What superseded this family of laws (SpaceX-style
  divert). Beyond the book's scope; "what came next" sidebar.

## What this reuses from `reference/`

- `core/kepler.ks` — `time_to_longitude`, `wrap_longitude`, `orbital_speed`, `bisect`/
  `find_zero_crossing` (phases 0–2, `solve_t_go`).
- `landing_v2/time_to_closest_approach.ks`, `minimize.ks`, `orbit_after_maneuver.ks` —
  DOI placement and fine-tuning.
- `core/impact.ks` `predict_impact` — demoted to the coast-phase abort instrument.
- `original/common.ks` `execute_node` — DOI execution.
- `script/landing.ks` — terminal-descent logic and the on-screen status display survive
  nearly intact; its nested-trigger state machine is replaced by sequential phase
  functions.
