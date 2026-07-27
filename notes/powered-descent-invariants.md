# The powered descent: the invariants

*The design register for `reference/original/powered_descent.ks`: what the flight program
promises at every instant, and the control law those promises imply. Companion registers:
`doi-planner.md` (the planner that places PDI and owns the reference arc),
`klumpp-descent-redesign.md` (the design and its full argument),
`retrograde-terminal-findings.md` (the flights), `apollo-powered-descent.md` and
`klumpp-guidance-derivation.md` (the guidance law's normative sources).*

## The premise

The program flies a feedback law, and it carries no trajectory. Commanded acceleration is
a closed form in the state, the gate state, and one clock:

```
a_cmd = 6·dR/t_go² − 2·dV/t_go        dR = r_tgt − r − v·t_go,  dV = v_tgt − v
```

— Klumpp's guidance, Apollo P63's law, in three dimensions with no channel split. Thrust
demand is `a_cmd − g`; throttle is its magnitude over available thrust; attitude is its
direction; the pitch-over from near-retrograde falls out of the law with no attitude
program. **No numerical integration runs in flight.** The one-parameter family of
gravity-turn arcs, and the march that walks it, live in the planner (`doi-planner.md`):
the flight is handed a periapsis and a lead, and answers everything else from state.

The taxonomy word is *explicit* guidance, but the state it is explicit about is the
boundary condition, not the path: the law never asks where the trajectory goes, only what
acceleration carries the two errors to zero in the time remaining.

## The invariant set

The flight is these five statements and nothing more.

1. **One law flies braking.** Position error and velocity error against the gate state,
   both, in vector. Cross-track is not a channel: the offset is a component of
   `r_tgt − r` and the law corrects it in vector, at a Δv cost quadratic in the offset
   (< 1 m/s at the ~1.3 km PDI offsets measured). There is no yaw law, no gain, and
   nothing to tune.

2. **`t_go` is chosen once, at ignition, at the minimax.** The commanded acceleration is
   linear in time, so the profile's demand peak sits at an endpoint; equalizing the two
   endpoint demands minimizes the peak. That crossover is the dip — the cheapest profile
   the bracket holds — and the program root-finds `d_ign = d_gate` on
   `[1.6, 2.6]·X/v₀` rather than searching a minimum. Each endpoint is priced at its own
   mass, so the solve wraps a two-pass gate-mass contraction: the gate mass sets both
   `v_gate` (through `a_dec`) and the gate-end demand, and each pass shrinks the mass
   error by roughly the propellant fraction. In flight `t_go` decrements by clock. It is
   never re-solved: re-anchoring the schedule re-admits the moving-target defect class
   that solve latency produced on the retrograde build.

3. **The law aims at a virtual gate.** E-guidance does not taper — at any positive `t_go`
   it still commands its arrival acceleration — so a law aimed at the real gate and cut
   at a floor `τ_f` hands FALL a velocity error of about `|a_end|·τ_f`. The aim point is
   therefore the real gate state propagated `τ_f` forward along the profile's own linear
   acceleration, position and velocity both, with `k = (a₁ − a₀)/τ` the planned profile's
   jerk:

   ```
   v_virt = v_gate + a₁·τ_f + ½·k·τ_f²
   p_virt = p_gate + v_gate·τ_f + ½·a₁·τ_f² + (1/6)·k·τ_f³
   ```

   Braking exits at `t_go = τ_f`, at which instant the ship occupies the *real* gate
   state to the law's tracking accuracy, and the `1/t_go²` divergence never enters. Only
   the offsets are frozen at the solve; the gate's position and its vertical are rebuilt
   from the live target each tick, so the aim point rides the rotating ground.

4. **High gate opens the vertical corridor.** Everything below the gate is vertical, so
   feasibility below the gate is one inequality on total surface speed — the variable the
   arrest trigger actually reads — with `a_dec = f_max·T/m − g` the net arrest
   deceleration:

   ```
   v_surf² ≤ 2·a_dec·(h_gate − h_pad)
   ```

   **If it holds at the gate it holds thereafter, provided only `a_dec > 0`** — thrust at
   `f_max` beating local weight, true of any craft that can land at all. Falling a metre
   grows `v²` by `2g` while the stopping budget `2·a_dec·(h − h_pad)` shrinks by
   `2·a_dec`, so the margin between budget and need falls monotonically at
   `2·(a_dec + g)` per metre and crosses zero exactly once. That crossing is low gate, and
   it sits above the pad if and only if the corridor held at the gate. The design targets
   arrival mid-corridor, `v_gate = wall/2`, so delivery and guidance error spend margin on
   either side instead of finding a wall; `v_gate` is a *commanded* boundary condition,
   part of `v_tgt`, delivered by the same polynomial that delivers offset and drift.

5. **Terminal is a chain of three states, not a controller.** FALL: engine off, holding
   surface retrograde. With drift nulled at the gate, retrograde and plumb coincide to
   within the residual, so the nose is already on the thrust direction when the arrest
   ignites — thrust waits for attitude, and this is how it is paid for. Low gate: the
   arrest schedule fires when `f_max` could just bring the total speed to rest `h_pad`
   above the ground. Arrest burn: hold the vertical deceleration that carries the descent
   rate to `v_floor` at the pad, restoring the vertical share the retrograde lean sends
   sideways, plumb below `v_switch`. Then settle, then SAS.

The seams are where the invariant changes: the node, PDI, the gate, low gate. Each phase
hands the next a state it can finish from — the coast hands braking a periapsis, braking
hands FALL a gate state inside the corridor, FALL hands the arrest an altitude the burn
fits under.

The phases only come apart because the trajectory *misses* the surface: periapsis is low,
safe and up-range, and the burn is what brings the ship down. PDI is a chosen state, not a
rescue. Plan the trajectory to intersect the site instead and the coast is a fall, timing
goes safety-critical, and there is no stable state to abort into or quicksave from.

## Feasibility: refusal, before it costs anything

The program assumes the plan is good; its envelope protection is refusal. Two guards run
before the coast, where declining costs nothing: a live engine with thrust at `f_max`
above weight, and a gate above the flare height. Two checks run at PDI, where declining
to ignite *is* the abort — the ship sits at the periapsis of a stable, quicksave-able
ellipse.

1. **The demand crossing left the bracket.** The gate-end demand dominates at short `τ`
   and the ignition end at long, so a same-signed pair at the bracket ends means the
   crossing, and the dip with it, sits outside the interval this design certifies. Logged
   with both gaps and the distance flown to.
2. **The dip demand exceeds `f_max`.** The cheapest profile available does not fit under
   the ceiling. Logged with the demand and the ceiling.

Margin is booked once. Delivery error and the delivered flight-path angle are in the
state the `t_go` choice is made from, so they are absorbed into the schedule, not the
margin. The reserve from the planner's dip (at `f_cap`) up to `f_max` covers only what
arrives *after* ignition: thrust and mass model error, steering lag, and whatever the
decrementing clock accumulates.

## What the witnesses are

`flight_log.csv`, one row a second through braking, four a second through the arrest burn,
with planning numbers as `#` lines. The columns that carry the invariants:

- **`t_go`** — the guidance clock. It runs down to `τ_f` and nowhere else; a `t_go` that
  disagrees with the planner's reference by more than ~10 s at ignition is the placement
  cross-check failing.
- **`dem`** — commanded demand as a fraction of available thrust. Above `f_max` is
  saturation. `sat_s` on the `# high gate` line accumulates seconds spent there:
  saturation sheds the vertical component in effect, because thrust never pushes down, so
  saturation *duration* is the pre-gate observable that predicts a wall-side arrival.
- **`ach`** — achieved thrust acceleration over commanded, from the velocity difference
  since the last row. The law is indifferent to mass, Isp and thrust error — it commands
  acceleration and reads state — but `a_dec` and the corridor are computed from the model,
  so a persistent ratio off 1 is a wrong `f_max` or a stale mass, visible long before it
  is a saturated arrival. The ratio needs two consecutive burning rows, so it reads 1 on
  any row where this row or the last had the engine off.
- **`zem`** — the position miss a pure coast would book at the virtual gate, the law's own
  error measure, zero when the profile is converged.
- **`cross`** — the lateral state `v_to_site` is blind to: the site's signed cross-track
  offset in metres during braking, the full horizontal speed in m/s below the gate.
- **`clear_min`** — running minimum radar altitude over the arc. Instrumentation, not a
  rule: braking-arc terrain clearance is unruled (`terrain-certification.md`), and this
  measures the gap the design does not close.

And the `#` lines: the delivery witness at warp-out (`node-delivery-window.md`), the
ignition header, the `# high gate` arrival — radar, speed, drift, offset, corridor
fraction, `sat_s` — the `# bounds` datum with its accept/reject, `# touchdown`,
the settle trace, and `# landed`.

Two arrival checks are witnessed at the gate with the engine still lit, not at the pad:
the corridor fraction, which must be ≤ 1 and is designed near 0.25, and the offset and
drift residuals against what the law promised.

## Costs, priced

- **IPU.** `config:ipu` is raised to 2000 for the flight and restored on every exit path.
  The cost it buys is the ignition bisection: at the default rate the solve takes about a
  second of game time, and a second at PDI is several hundred metres of along-track — a
  large fraction of the delivery window spent on arithmetic. At 2000 it fits in a tenth of
  a second. Nothing else in the flight is expensive; the guidance command is closed form.
- **The bounds altimeter.** `ship:bounds` is obtained once, after the gear has deployed,
  because the box goes stale when the craft changes shape and the docs price obtaining it
  as expensive. The gear therefore deploys at ignition, where the braking burn gives the
  animation two minutes; in vacuum the deployed gear costs nothing. `bottomaltradar` off
  the stored box is the cheap per-tick read, and it tracks attitude — so the one-time
  `dh_core` capture is a guard input, not a flown number, and the height the flare
  actually plans against is read live.
- **Feedback through a decrementing clock.** The command changes the state the next
  command reads, but the horizon shrinks on a clock the plant cannot move, so there is no
  loop through the schedule itself.

## Standing lessons

Carried forward; each was earned by a flight.

- A scale-free control law must be given a scale at which to stop caring, or it will
  bang-bang at the precision floor.
- With a steering loop in series, thrust must wait for attitude; an engine that burns
  through its own slews closes a positive feedback loop through the plant. FALL holding
  retrograde is this lesson's cheapest possible form — the attitude is already right when
  the burn starts, so nothing waits.
- A derived constant is only as good as the state it is derived from; never derive from a
  stale estimate what is about to be measurable.
- When observed behavior is arithmetically impossible under the intended constants, audit
  the constants as flown before inventing dynamics.
- Discrete guards are not fudges; they are the decisions a continuous law cannot make —
  when to fire, when to quit, when to stop steering. The arrest latch and the `v_switch`
  plumb transition are the two this program keeps. But every discrete transition costs
  whatever attitude the delivery ties to it, so the transitions are placed where the
  attitude is already where it needs to be.
- A solve whose objective re-reads the world on every evaluation is chasing a moving
  target. Freeze the problem, or remove the solve from the loop.

## Open

- `τ_f` (3 s) is family-of-settle-time and underived; the virtual gate's construction
  bounds its consequence, not its value.
- `v_switch` (5 m/s) is a chosen tolerance, carried from the earlier build.
- The `[1.6, 2.6]·X/v₀` bracket is load-bearing: the demand curve's dip structure is
  established numerically for this geometry, not proved. A derivation retires it.
- Braking-arc terrain clearance is unruled. `clear_min` measures it; nothing checks it.
  The 2026-07-27 flight's minimum was 223 m, three seconds after ignition, with the gate
  1500 m up.
- Saturation response in braking — ride it or abort — is policy, and a named departure
  from the source's abort prescription (`apollo-powered-descent.md`).
