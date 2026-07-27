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
2. **Targeting.** `f_cmd` is the value whose arc reaches touchdown at the site: the
   endpoint integration runs brake, coast, and arrest burn to the ground, aimed at the
   in-plane component of the ground distance to the site. Well-posed because the
   endpoint's down-range falls strictly monotonically as `f` rises — more throttle,
   shorter arc — so the solution is unique when it exists.
3. **Feasibility.** That `f` satisfies `f ≤ f_max`, and its arc bottoms out at or above
   the handoff altitude. Violated on the high side: the site is unreachable within
   authority — abort while altitude remains. Violated on the low side: reaching the site
   would plan the arc below the gate — accept the short landing, never plan into the
   ground. This is the design's ordering, not a check the flying program makes: the
   integrating `endpoint` returns no bottom altitude to test it against, and the program
   carries no abort of any kind (see "Two conditions, one knob").
4. **Cross-track.** The site lies in the plane of the surface velocity, nulled by yaw
   while the ship is fast, where a degree costs least. The yaw law delivers closure
   `y / tau_yaw` by construction (the commanded bias is gain-corrected), with `tau_yaw =
   t_brake / k_yaw` and `k_yaw = ln(offset / y_floor)` clamped to `[1, 5]`, both fixed at
   ignition. The residual at high gate is therefore ≈ `y_floor`. Terminal carries no
   lateral channel, so the high-gate offset is the landing miss, to within terminal's own
   wander.
5. **Handoff continuity.** Both sides hold retrograde; terminal keeps the same direction
   braking flew. The one step at the gate is braking's yaw bias dropping out — a few
   degrees at the offset the yaw law leaves, since terminal carries no lateral channel.
   Continuity is structural, not a state the two phases have to be steered to match.

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

The design's decision framework: the endpoint must satisfy two conditions — over the
site, at the gate altitude — and there is one knob. Both endpoint coordinates are
monotone in `f`: down-range falls as `f` rises, bottom altitude climbs. So the two
conditions each pin their own throttle, `f_site` and `f_gate`, and the ordering below is
the command:

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

The flying program implements case 2 outright and a degraded case 3, described below.
Case 1's abort, case 4's pull-up, and invariant 3's low-side clause are this ordering's
design intent without code: they wait on the same bottom-altitude probe case 3's
degradation explains, and the program carries no abort of any kind.

Case 2 is implemented as specified: `solve_arc` tries the `f_max` arc first, and if it
still overshoots the site, flies `f_max`, books the smallest reachable overshoot, and
says so once per case change — a screen notice and a `# case` line in the CSV, so the
case is a logged fact rather than a silent choice.

Case 3's `f_gate` is not computable against an `endpoint` that integrates brake, coast,
and arrest all the way to touchdown and returns no bottom altitude to solve against. A
short landing instead holds the throttle already in hand: at ignition, where the search
runs the full `[0, f_max]` bracket, that throttle is `f_max` itself and the logged reach
is that arc's own — the largest reachable shortfall, the safety choice over the accuracy
one. In the braking loop SHORT is excluded from adoption, so the throttle actually flying
stays the previous look's `f_cmd`, not `f_max` and not predicted by the solve at all; the
notice and the `# case` line report `x_best`, the smallest shortfall any throttle in
`[0, f_max]` could have booked, beside the reach of the search bracket's own top end —
neither of which is the flown throttle's own reach. Implementing `f_gate` means giving
the march a bottom-altitude probe.

## Costs, priced

- **IPU.** A look is about 11 marches: two endpoint evaluations at the bracket's ends,
  the bisection closing a 0.1-wide bracket to 0.001 tolerance, and one confirmation march
  at the root. Seconds per look is a prediction, not yet a flown number — this
  frozen-solve build has not flown — but early-burn looks (the widest brackets, the
  highest speeds) model out at roughly 3–4.5 s of game time at `config:ipu 2000`, an
  early duty cycle of about 25–45 % against the 5 s look cadence, tapering as the bracket
  narrows and the remaining arc shortens; the next flight's CSV is what falsifies or
  confirms it. Each look solves one frozen problem — state and aim distance captured once
  at the top of the look — and two clocks track it: `t_frozen`, the instant the adopted
  solution's state was captured, against which the readout ages `t_go`; `t_solved`, the
  instant the look returned, which gates the 5 s cadence so a costly look still hands the
  ship 5 s of flown, logged flight before the next one starts. If the budget ever binds,
  the bisection becomes a secant seeded by the previous look (2–3 marches) before
  anything else changes.
- **Feedback through prediction.** A re-solver's command changes the state its next
  prediction seeds from. Deadbeat receding-horizon control of a monotone one-knob system
  is about as benign as this gets, and the cadence is long against the throttle's effect
  appearing in the state.
- **Gain inversion near the handoff.** As the arc shortens, metres of endpoint per unit
  of throttle collapse, so the solve would demand ever-larger `f` swings to correct
  ever-smaller misses. The design freezes `f_cmd` once retrograde comes within 10 degrees
  of high gate and lets the last solution ride; the residual miss is metres, and
  terminal's own wander owns the last few metres by charter.

## Open

- The 5 s look cadence and the 10-degree freeze cutoff near high gate are chosen, not
  derived.
- The radar backstop (`alt:radar` against a terrain-clearance threshold while fast) has
  not flown, and the terrain certification it leaned on is now the pilot's eye rather
  than a survey.
- `f_gate` — case 3's accuracy answer to a short landing — is not computable against an
  endpoint that integrates to touchdown and exposes no bottom altitude. SHORT holds the
  throttle already in hand instead (`f_max` at ignition, the previous look's `f_cmd` in
  the braking loop), the safety choice, not the accuracy one; the shortfall it books is
  not the smallest one available, and in the braking loop neither the notice nor the
  `# case` line predicts the flown throttle's own reach.
