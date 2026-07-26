# The docking approach's monopropellant budget

*Design register for `reference/original/dock3.ks`, the budgeted approach.
`reference/original/dock2.ks` flies the same corridor on fixed gains and keeps no log;
it is the simpler spike, still there to fly when propellant is not the binding
constraint. This note records what the budgeted approach spends, what sets the size of
each spend, and what the flight log has to show for the design to be believed.*

## Why the budget is tight in the first place

RCS delta-v comes out of the rocket equation like any other, and for a small propellant
fraction it is very nearly linear:

    dv ≈ Isp · g₀ · m_prop / m

Monopropellant thrusters run at a couple of hundred seconds of Isp, so `Isp·g₀` is about
2000–2500 m/s. A twenty-tonne spaceplane carrying sixty kilos of monopropellant therefore
has on the order of **six or seven m/s in the tank** — for the whole approach, the
attitude holding through it, a second attempt if the first misses, and undocking
afterwards. That is the entire reason this script needs a design note. An orbital
manoeuvre budget is measured in hundreds of m/s and nobody counts; a docking budget is
measured in single figures and everything counts.

## The one lever that matters: time

Moving d metres and stopping costs delta-v twice — once to start the motion, once to end
it. On a proportional law `v = d/τ` the arithmetic is exact: accelerating to the initial
commanded speed costs `d₀/τ`, and the decay that follows integrates to `d₀/τ` again, so

    dv_total = 2 d₀ / τ

**The cost is inversely proportional to the time taken, with no other term.** Halving the
budget doubles the clock and changes nothing else about the trajectory. So the script
takes a delta-v budget as its argument, measures the initial separation, and solves

    τ = 2 (axial₀ + lateral₀) / budget

once at entry. That τ is the only gain in the controller: the lateral centring speed is
`lateral/τ`, the closing speed is `axial/τ`, and the velocity deadband is
`corridor/τ`. There is nothing else to tune, and the tuning is stated in the units the
pilot actually budgets in.

Axial and lateral are summed rather than combined in quadrature because RCS thrusters are
axis-aligned: propellant tracks the sum of the components, not the magnitude of the
vector. The estimate is therefore an upper bound, which is the right side to err on.

## The three places propellant goes

1. **Translation.** Sized by the budget above. Predictable, and the only spend the pilot
   is really buying.
2. **Attitude.** An attitude, once reached, costs nothing to keep in free space — only to
   change. A continuously held `lock steering` pays anyway, because the steering manager
   answers every disturbance the moment it appears, and on a craft whose only torque is
   RCS that answer is monopropellant. So the ship is steered only outside a **deadband**:
   engage at `align` degrees, release at a third of that once the error is already coming
   down, coast in between. Releasing is the whole point; the hysteresis and the
   error-is-shrinking test only stop it chattering at the boundary.
3. **Fighting itself.** Translation and attitude are interleaved, never simultaneous:
   while the ship is slewing, translation is commanded to zero and the ship coasts, which
   is free. Behind the target port's plane the script hands translation back to the pilot
   entirely rather than holding zero relative velocity against whatever they do to fix it.

## Deadbands are sized, not picked

The velocity deadband is `max(0.02, corridor/τ)` — the velocity error that would carry the
ship out of the approach corridor no faster than the approach is closing it. It widens
with range for the reason a fixed one cannot: 5 cm/s is nothing at a hundred metres and
everything at one. The 2 cm/s floor is where the physics tick's own jitter lives; nulling
below it is propellant spent on noise.

## Constants, and what argues each

| Constant | Value | Argument |
|---|---|---|
| `budget` | 2 m/s | A light spaceplane's whole load is often under ten m/s; two buys the approach and leaves the rest for attitude, a retry, and undocking. |
| `align` | 5° | Stock ports capture from well outside ten degrees, so five is margin, not a limit. Release at a third of it makes the duty cycle a function of drift rate, not of this number. |
| `v_touch` | 0.1 m/s | Fast enough that the magnets reach across their capture range before the corridor controller stalls inside its own deadband; slow enough that a missed latch is a nudge. |
| `v_max` | 2 m/s | Collision limit, not an optimisation target: above this a port bounces or breaks instead of latching. Caps the closing speed whatever the budget asks for. |
| corridor | `0.2·axial + 0.2` m | A cone widening one metre per five of range, with 20 cm of slack at the port. Inherited unchanged; it gates the closing burn, not the spend. |

Roll gets no budget of its own. It rides along whenever steering engages, and stock ports
snap to whatever roll they meet.

## What the log has to show

`dock_log.csv`, one row per second, `#` metadata carrying the planning numbers:
`t,phase,axial,lateral,relv,v_cmd,dv_err,att_err,steer,mono`. `phase` is `SLEW`, `COAST`,
`TRANS` or `BEHIND`, so every row is attributable to one of the three spends above.

Three testable signatures, none of them yet flown:

- **`mono` is flat across every `COAST` row.** This is the deadband claim: released
  attitude and a velocity error inside the band should cost exactly nothing. Any drain in
  `COAST` means something is still commanding thrust, and the design is wrong rather than
  merely mistuned.
- **`mono` used, on the closing `#` line, is within about a factor of two of the budget**
  once converted to delta-v. The estimate is an upper bound on translation only, so it
  should come in under the budget for translation and over it once attitude is included;
  the gap between them is the attitude spend, which nothing currently predicts.
- **`dv_err` does not change sign on successive rows.** The translation command still
  carries the inherited gain of 1.5 on a 0.1 s loop, which was never checked against a
  log. Alternating signs would be that gain overshooting and paying for it twice; a
  monotone `dv_err` says the gain is fine and should be left alone.

Until those rows exist, the design's claim is arithmetic about `2d/τ` and about what a
deadband cannot cost, not a claim about how this ship flies.
