# SSTO ascent: replacing hand-tuned knobs with measured aero quantities

*An analysis of `reference/original/ssto.ks`, `ssto2.ks`, and `ssto4.ks`, and how the
functions in `reference/original/aero.ks` could optimize the ascent in a more principled
way. Analysis only — no code yet. The short version: the scripts' heuristics are all
shadows of three measurable quantities — dynamic pressure, specific excess power, and the
lift/drag force balance — and `aero.ks` measures all three.*

## What's actually hand-tuned, and what each knob is a proxy for

Looking across the three scripts, the magic numbers fall into a few families:

- **`speed_factor is 28`** (ssto.ks) — the `airspeed <= altitude / speed_factor` climb
  schedule. This is really a crude **dynamic pressure corridor**: it's forcing speed to
  grow with altitude in rough proportion to falling density.
- **`twr_factor is 13.33`** (ssto2.ks, `pitch = twr_factor * twr`) — a proxy for "climb at
  the steepest angle the current excess thrust supports."
- **"Airspeed stopped increasing" as the mode-switch trigger** (all three) — this is
  detecting the moment **specific excess power hits zero** in the current engine mode, but
  only after the fact.
- **`rocket_ascent` = 21/22/23° and the level-off hack**
  (`ascent - (eta:apoapsis - 30) / 2`, which the comment itself calls "a TOTAL hack...
  probably wants some computation based on drag") — a stand-in for the pitch that balances
  drag losses against gravity losses on the way out of the atmosphere.
- **"21° doesn't cause much heating"** — an implicit thermal-flux constraint, tuned by
  watching parts glow.

Each of these has a physical quantity behind it, and `aero.ks` can measure most of them
live: `dynamic_pressure()`, `mach_number()`, `angle_of_attack()`, and — the crown jewels —
`drag_vector(accel)` and `lift_vector(accel)`, which back out the actual aero forces from
the force balance m·a = T + W + L + D using the `accelerometer()` closure. That's an
in-flight wind tunnel.

## The principled framing: energy-based ascent

The classic aircraft-performance way to pose this (and it fits an SSTO beautifully) is in
terms of **specific energy** E = h + v²/2g and **specific excess power**
P_s = (T − D)·v / (mg). An ascent is just a trajectory through the altitude–velocity
plane, and the near-optimal strategy is to climb along the path that maximizes energy gain
per unit fuel. Everything the scripts do by feel becomes computable:

1. **Air-breathing climb: control on measured excess power, not a speed schedule.**
   Instead of `altitude / speed_factor`, compute P_s directly each tick — thrust from
   `thrust_vector()`, drag from `drag_vector()`. Pitch is then the control variable in a
   loop that seeks maximum P_s (or holds a target dynamic pressure from
   `dynamic_pressure()`, which is the physically meaningful version of `speed_factor` and
   transfers between vehicles). Since RAPIER thrust in KSP is a strong function of Mach
   and density, you don't even need a model — a slow extremum-seeking dither on pitch,
   reading the filtered P_s response, will ride the ridge.

2. **Mode switching: predictive instead of reactive.** All three scripts wait until the
   ship actually stops accelerating, which means mushing along at zero excess power before
   switching. But you can *predict* the crossover: measured drag + known closed-cycle
   thrust (`availablethrustat(p)` is already being used in ssto2's telemetry) gives you
   rocket-mode P_s at any moment. Switch when rocket-mode energy-gain-per-kg-of-fuel
   exceeds air-breathing mode's — comparing P_s/ṁ using `ispat(p)`. That turns the
   wet-mode and staging triggers from tuned event detectors into a computed decision.

3. **Rocket phase / level-off: compute the pitch from the force balance.** The thing the
   eta:apoapsis hack is groping toward is "hold a shallow climb where
   T·sin(pitch) + L ≈ W minus the vertical acceleration you want." Every term there is
   available: weight from `weight_vector()`, lift from `lift_vector()`, thrust known.
   Solve for pitch as feedforward, then wrap a small feedback loop on eta:apoapsis to trim
   it. The 21°-vs-23° tuning disappears into physics, and it adapts when you fly a
   different airframe.

4. **AoA, not pitch, as the control variable.** ssto.ks has the TODO already. Lift scales
   roughly as Q·AoA, and `angle_of_attack()` + `lift_vector()` let you fit the lift-curve
   slope in flight after a few seconds of data. Then "I need this much vertical force"
   maps directly to a commanded AoA regardless of flight-path angle. This also naturally
   caps drag-due-to-lift, which is a real fuel cost the current scripts can't see.

5. **Heating as an explicit constraint.** Aerodynamic heating goes roughly like ρv³ —
   computable from `air_density()`. Instead of tuning the ascent angle until parts stop
   exploding, pitch up whenever ρv³ approaches a limit measured once per airframe.

There's also a nice offline complement: use the kOS bridge to `log` (altitude, velocity,
measured thrust, measured drag, fuel flow) on a few flights, build the P_s map over the
h–v plane on the ground, and precompute the optimal climb corridor — then the flight
script just tracks it. That's the energy-state approximation from real aircraft trajectory
optimization, and it would make a terrific chapter arc: hand-tuned heuristic → in-flight
measurement → offline optimization.

## Caveats before trusting aero.ks in a control loop

- `drag_vector`/`lift_vector` differentiate velocity numerically, so they're **noisy** —
  they need a low-pass filter before feeding a controller, and the `// sign?` comments in
  both functions suggest they haven't been fully validated yet. A calm gliding test flight
  (thrust = 0, so drag is unambiguous) would be the way to check them.
- `thrust_vector()` assumes actual thrust = available thrust × throttle along facing —
  fine for most setups, wrong during flameouts or with off-axis engines. ssto2's
  per-engine flameout handling matters here.
- KSP's drag model is per-part and strongly Mach-dependent, so don't try to fit one global
  drag polar; trust the live measurement locally and let feedback do the rest.

One drive-by observation: in `ssto4.ks`, `level_off` is used in the triggers around lines
94–96 but never declared in the file — it only works if a previous script left a global
behind, and would throw on a cold run.

Replacing the tuned constants with control loops on measured quantities would make the
scripts vehicle-independent, and the progression maps naturally onto the book's "measure,
then compute, then optimize" pedagogy.
