# Cruise optimizer: a control loop that finds minimum-fuel level flight

*Design note, no code yet. Extends the `autopilot` branch's five-loop cascade
(`reference/original/autopilot.ks`) with a sixth, outermost loop that chooses the altitude
and airspeed setpoints instead of holding whatever the ship had at startup. Companion to
`ssto-aero-optimization.md`, which proposes the same technique — extremum seeking on a
measured aero quantity — for the ascent; this is the cruise-phase version.*

## The objective, and how to measure it

Fuel per metre of ground covered:

    J = ṁ_fuel / v_ground        (kg/m — smaller is better)

Two ways to get J each tick, one primary and one cross-check:

1. **Direct (primary): watch the ship lose weight.** In cruise nothing leaves the ship but
   burned fuel, so `ṁ = −d(ship:mass)/dt`. Over a measurement window: fuel mass burned
   divided by distance covered (`ship:groundspeed` integrated, or `ground_distance()` from
   the window's start point). Model-free — no thrust model, no drag model, no Isp lookup.
2. **Derived (cross-check): drag over airspeed.** In trim T = D, and KSP jet fuel flow is
   T/(Isp·g₀) with constant Isp, so J = D/(Isp·g₀·v). `drag_vector(accel)` from `aero.ks`
   measures D live. Faster than the windowed mass measurement but noisier (numerical
   differentiation) and resting on the still-unvalidated `// sign?` force balance — use it
   as telemetry to sanity-check the direct measurement, not as the loop's input.

No wind in stock KSP, so groundspeed and airspeed agree in level flight; the distinction
is bookkeeping, not physics.

## Why there is something to find

Level flight pins lift to weight, so at a given airspeed the density (altitude) sets the
required lift coefficient: climb and parasite drag falls while induced drag rises. At a
given altitude, speed trades the same two terms the other way. J is a surface over the
(altitude, speed) plane with a ridge structure set by the Mach drag rise.

Where the minimum sits is airframe-dependent, and the trim throttle at convergence is the
diagnostic:

- **Interior optimum** — throttle settles strictly below 1. The Mach drag rise caps the
  "climb forever" logic of the incompressible textbook model; best Mach plus best lift
  coefficient picks a specific density, hence a specific altitude, and the engine has
  margin there. Expected for overpowered airframes (Whiplash/RAPIER on a light hull).
  There may be two local minima — subsonic below the drag rise, supersonic past it — and
  the optimizer finds whichever basin it starts in.
- **Boundary optimum** — throttle pegs at 1 and the optimizer keeps asking for more
  altitude. The thrust envelope intersects the ridge before the stationary point; the
  optimal altitude is the highest the engine can hold at best lift coefficient. Expected
  for underpowered airframes. The optimizer must detect this and stop pushing.

Either way the answer is one number per airframe, and one instrumented flight settles it.

## The loop architecture

The existing cascade is untouched; the optimizer only writes the two setpoints that
`autopilot.ks` currently captures once at startup:

    +--------------------------------------------------------------------+
    |  CRUISE OPTIMIZER            period: minutes                       |
    |  measure J over a window; adjust h_cmd, v_cmd to walk downhill     |
    +--------------+---------------------------------+-------------------+
                   | h_cmd (alt setpoint)            | v_cmd (spd setpoint)
                   v                                 v
        alt_pid: alt err -> pitch cmd     spd_pid: spd err -> throttle
                   |                                 |
        pitch_pid: pitch -> elevator            mainthrottle
                   |
               elevator

    (roll/heading chain unchanged: hdg_pid -> roll_pid -> aileron; the
     optimizer runs only while wings are level, so it composes with the
     waypoint layer by pausing during turns)

Timescale ladder, each level well separated from the next: attitude loops every tick;
altitude/speed loops settle in ~10–20 s; one optimizer measurement window ~30 s; one
optimizer step per window or two; convergence over tens of minutes of cruise. The
separation is load-bearing — a step reflects in J only after the trim loops have finished
chasing it, and measuring during the chase reads the cost of maneuvering, not the cost of
cruising.

## First rendition: step-and-compare

Not the sinusoidal-dither extremum seeker — a discrete one-axis-at-a-time hill descent
("twiddle"). Same idea, but every decision is a legible A/B comparison between two numbers
logged seconds apart, which makes it debuggable from the flight log and teachable on the
page. The sinusoid version (dither both setpoints at incommensurate slow periods,
demodulate, integrate) is the refinement if step-and-compare proves too slow; it may never
be needed.

State machine, sketched:

    SETTLE:   wait until trimmed -- |verticalspeed| and |spd err| small for
              several seconds, wings level. Then open a window.
    MEASURE:  over ~30 s accumulate fuel burned (mass delta) and distance.
              At window end, J = fuel / distance.
    DECIDE:   if J improved on the best so far: keep the setpoint, keep the
              best J, step the SAME direction again (momentum).
              else: revert the setpoint, try the other direction; if both
              directions of an axis fail, switch axis (h <-> v); if both
              axes fail, halve the step sizes.
              Converged when steps shrink below thresholds (say 25 m, 2 m/s).

First-cut numbers: altitude step ±150 m, speed step ±8 m/s — big enough that the J delta
clears measurement noise, small enough that the settle transient is brief and the fuel
spent maneuvering (which the windows deliberately exclude, but which is a real cost) stays
negligible. All four numbers are tunables to be set by flying, not by argument.

## Constraints the optimizer must respect

- **Throttle saturation = envelope boundary.** If mean trim throttle over a window exceeds
  ~0.95, mark the boundary: reject steps that need more thrust (up in altitude or speed),
  and report the pegged throttle — that is the boundary-optimum diagnosis, not a failure.
- **Flameout ceiling.** Hard clamp on h_cmd below the airframe's known flameout altitude;
  the optimizer must not discover engine-out empirically.
- **Stall floor.** Reject speed steps that push measured `angle_of_attack()` past a limit;
  induced drag near stall also makes J blow up, so the optimizer avoids it naturally, but
  the clamp keeps a noisy window from stepping through the floor.
- **Turns corrupt J.** Bank raises induced drag; suspend SETTLE/MEASURE whenever the
  heading loop is commanding bank beyond a few degrees, resume on the next straight leg.

## Caveats

- The mass-delta measurement assumes fuel is the only mass leaving the ship — true in
  cruise, false the moment anything decouples or a resource transfers. Windowing plus the
  trim gate makes this hard to violate by accident.
- Intake air cycles through the tanks but its stored quantity is roughly constant in
  steady flight, so it cancels across a window.
- Physics warp compresses the wall-clock cost of convergence and changes nothing in the
  math; all rates are in game seconds already.
- Step-and-compare finds a local minimum. With a transonic ridge in the middle of the
  plane there are plausibly two; if the subsonic and supersonic answers both matter,
  run the optimizer twice from seeds on either side and compare the converged Js.

## Book fit

Part V's arc is measure lift and drag (ch. 14), map the engine (ch. 15), then PID control
and the cruise autopilot (ch. 16). This optimizer is the payoff exercise sitting right at
the seam with Part VI: the autopilot stops holding the altitude you gave it and starts
finding the altitude the airframe wants, using nothing but the telemetry discipline the
book has been building since chapter 2. The converged throttle reading — physics-chosen
altitude versus engine-chosen altitude — is exactly the kind of question Part VI's energy-
management framing needs the reader already comfortable asking. Verification is a flight:
log (h_cmd, v_cmd, J, throttle) through the bridge and watch the staircase walk downhill.
