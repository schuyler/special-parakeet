# Powered descent by live re-solve: the invariants

*The design register for `reference/original/powered_descent.ks`: what the braking phase
promises at every instant, and the control law those promises imply. Companion registers:
`doi-planner.md` (the planner that places PDI), `powered-descent-handoff-contract.md`
(the braking→terminal seam), `apollo-powered-descent.md`, `klumpp-guidance-derivation.md`.*

## The premise

Hold thrust retrograde and the trajectory is a **one-parameter family**: the current state
(altitude, speed, flight-path angle, mass) plus a throttle `f` fully determines the arc,
because the gravity turn flies itself. So at any moment during braking, where the arc at
the current throttle bottoms out — altitude, down-range, time — is *computable from the
ship*. There is no closed form (the gravity-turn ODE has none); "we know the trajectory"
means we know it by integration — an Euler march, re-run every few seconds from live state.
In the standard taxonomy that makes this *explicit* guidance: the trajectory is computed
onboard from current state every cycle rather than precomputed and tracked against a stored
reference (see `apollo-powered-descent.md`).

## The invariant set

The braking phase is these five statements and nothing more:

1. **Retrograde hold.** Thrust lies along −v_surface, ± a bounded yaw. This is the
   enabling invariant: it is what makes the trajectory a one-parameter family.
2. **Targeting.** `f_cmd` is the value whose arc ends over the site. Well-posed because
   the endpoint's down-range falls strictly monotonically as `f` rises — more throttle,
   shorter arc — so the solution is unique when it exists.
3. **Feasibility.** That `f` satisfies `f ≤ f_max`, and its arc bottoms out at or above
   the handoff altitude. Violated on the high side: the site is unreachable within
   authority — abort while altitude remains. Violated on the low side: reaching the site
   would plan the arc below the gate — accept the short landing, never plan into the
   ground.
4. **Cross-track.** The site lies in the plane of the surface velocity, nulled by yaw
   while the ship is fast, where a degree costs least.
5. **Handoff continuity.** The arc ends at `speed_handoff`, which is terminal's descent-
   rate cap: the phases meet at the same state.

Each phase of the whole descent is *named by* an invariant like these: the coast is
on-rails (the ellipse is the state — quicksave-able, abortable), braking is the family
above, terminal is a rate servo. The seams — the node, PDI, the handoff state — are where
the invariant changes.

The phases only come apart because the trajectory *misses* the surface: periapsis is low,
safe and up-range, and the burn is what brings the ship down. PDI is a chosen state, not a
rescue. Plan the trajectory to intersect the site instead and the coast is a fall, timing
goes safety-critical, vertical and horizontal speed have to die in separate burns, and
there is no stable state to abort into or quicksave from.

## What the program is

*Hold retrograde; every few seconds, solve for the one throttle whose arc ends at the site;
check it's feasible; yaw the plane onto the site.* That is the whole flight controller: the
integrator, a scalar solve around it, the retrograde hold, the yaw law (t_go read off the
march), terminal descent, the recorder, the aborts.

PDI is not special to it. The integrator seeded from live state doesn't care that PDI is a
periapsis; ignition slop, DOI placement error, and mid-burn dispersion are all just "the
current state," absorbed identically. PDI stays special to the *planner* — it is where the
Δv lives.

## Two conditions, one knob

The endpoint must satisfy two conditions — over the site, at the gate altitude — and there
is one knob. Both endpoint coordinates are monotone in `f`: down-range falls as `f` rises,
bottom altitude climbs. So the two conditions each pin their own throttle, `f_site` and
`f_gate`, and the command is the feasibility ordering:

1. If even the `f_max` arc — the highest, shortest arc the craft has — bottoms below the
   gate, nothing closes: **abort** (re-plan before the coast; emergency-land during the
   burn).
2. If even that arc ends at or past the site, every throttle books an overshoot: fly
   `f_max`, eat the smallest one, and say so.
3. Otherwise solve `f_site`. If no throttle reaches the site — every arc lands short —
   aim the bottom at the gate (`f_gate`) and accept the short landing.
4. If `f_site`'s arc dips below the gate, pull up to `f_gate`: **the gate outranks the
   site.** A wrong-place landing beats a right-place crater, the same trade
   `emergency_land` makes, applied continuously.

The planner's placement determines which case runs: a well-placed PDI keeps every look in
case 3's happy path, with `f_site` a little above the pre-flight solution and the endpoint
a little above the gate. The other cases are what the design does about a planner that
missed.

## Costs, priced

- **IPU.** A look is roughly a dozen marches (one `f_max` probe, a bisection, one
  endpoint confirmation). At 150 steps and `config:ipu 2000` that is a second or two of
  game time — acceptable at a 5 s look cadence because the locks keep flying the ship
  while the mainline marches, and the step budget scales down with the remaining speed
  span. If the budget ever binds, the bisection becomes a secant seeded by the previous
  look (2–3 marches) before anything else changes.
- **Feedback through prediction.** A re-solver's command changes the state its next
  prediction seeds from. Deadbeat receding-horizon control of a monotone one-knob system
  is about as benign as this gets, and the cadence is long against the throttle's effect
  appearing in the state.
- **Gain inversion near the handoff.** As the arc shortens, metres of endpoint per unit
  of throttle collapse, so the solve would demand ever-larger `f` swings to correct
  ever-smaller misses. The design freezes `f_cmd` when predicted t_go falls below a
  threshold (~10 s) and lets the last solution ride; the residual miss is metres, and
  terminal's drift cascade owns the last few metres by charter.

## Open

- `solve_period` and `t_go_freeze` are chosen, not derived.
- Whether bisection needs replacing with a seeded secant, on IPU grounds.
- The radar backstop (`alt:radar < landing_height` while fast) has not flown, and the
  terrain certification it leaned on is now the pilot's eye rather than a survey.
