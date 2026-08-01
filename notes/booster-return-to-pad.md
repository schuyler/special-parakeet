# Booster return to pad

*The design register for `reference/original/boostback.ks`: a spent first stage flips
downrange, burns back toward the space centre on what is left in its tanks, and lands on
parachutes. Companions: `ballistic-targeting.md` (the loop itself, which now lives in
`reference/original/targeting.ks` and is shared with `ballistic_hop.ks` — read it for the
survey, the drag correction, the taper and the closed forms), `reference/original/reentry.ks`
and its `entry_flight.csv` (the other recorder that logs two impact predictors side by side,
and the script that inherits the `landing_site` fix below), `entry-s-curve.md` (range control
inside the atmosphere, which this script does not attempt), `kos-facts.md` (CHUTESSAFE, and
the rotation term `landing_site` was missing).*

*What is left in `boostback.ks` after the loop moved out is everything the loop deliberately
does not own: the flat burn attitude, the vehicle's own reasons for stopping, and the
descent. Sections below that describe the loop describe `targeting.ks`.*

**Nothing in this register has been flown.** Every tunable in the script says so in its own
comment, and the signatures below are predictions, not results. The file is at step 3 of the
flight pipeline — pre-flight review — and stops there.

## What this optimises, and why it is not Δv

The descent scripts optimise Δv against a 10 m accuracy bar. This one optimises neither.
Stock recovery pays a fraction of a vessel's cost that falls off with its distance from the
space centre, so the quantity being bought is **funds**, the currency is **distance**, and
the scale over which distance matters is **kilometres**. That resets three things at once:

- **Pinpoint is not the bar.** `miss_bar` is 250 m because the two impact predictors this
  script carries disagree by far more than that, and because burning past the point where
  the descent's own dispersion swamps the correction spends propellant on precision that
  is discarded a minute later.
- **All the fuel is available.** `dv_reserve` is 0. There is no landing burn to hold back
  for — the canopies do that work — and propellant carried home is recovered at its own
  value, not at the booster's.
- **A short burn is not a failure.** A booster that separates fast and far often cannot
  close the miss at all. That is not an abort; it is a partial return, and it is worth
  flying, because every kilometre the burn buys is funds. The loop is written so that
  running out of fuel is an ordinary exit with a name, not an error.

## The control variable is the impact point

The boostback Δv is not available in closed form. Finding it means a Lambert solve on a
rotating body against a drag model the script does not have, and the answer would be stale
the moment the first kilogram of propellant left the tank. So nothing solves for it.

Instead the loop closes on where the trajectory says the booster meets the ground, and it
can, because of one property: **on a coasting ballistic arc the predicted impact point does
not move.** It is an invariant of the orbit. Three things follow, and they are most of the
design:

- Every metre the miss closes was bought by thrust, so the measured closing rate is a
  direct reading of what the throttle is worth on this vehicle, in this geometry, right
  now. Nothing has to model it.
- Noise in the miss is noise in the predictor, not in the world, so it can be filtered
  hard without filtering away signal.
- The overshoot test is trivial and safe. The miss can only grow under thrust, so a miss
  that has grown past its own best by more than the bar means the burn is now making the
  landing worse — whatever the predictor thinks it is doing — and the loop cuts.

## The burn direction is the miss direction

The horizontal component of the vector from the predicted impact to the aim point, taken in
the ship's own horizontal plane. Burning along it moves the impact along it, which is the
whole argument; there is nothing to prove about a decomposition into downrange and
cross-range, because both fall out. Burning west off an eastward arc pulls the impact west;
a northward miss puts a northward component in the command. Cross-range correction is not a
second loop, it is the same one.

The burn is horizontal (`heading(azimuth, 0)`). At boostback the velocity is nearly
horizontal and the impact point's sensitivity to horizontal Δv dominates its sensitivity to
vertical Δv, so the horizontal axis is where the propellant does the most good per unit
spent. A vertical component would buy range too, by lengthening the fall, but it buys it at
the price of a higher, faster, hotter entry.

The flip is not a phase. `throttle_for` returns zero while the nose is outside `align_bar`,
so the burn starts the instant the turn finishes and shuts again if the craft loses the
attitude — which a spent booster on residual authority may well do. `align_bar` is 10
degrees rather than a node burn's 0.25: cos(10 deg) = 0.985, so at most 1.5% of the burn
goes somewhere other than commanded, and that part is still steering the impact point
rather than being lost. Surface retrograde is within a few degrees of the boostback
direction on a mostly-horizontal arc, so the settle wait does most of the flip before the
throttle is ever asked for.

## The taper divides the rate by the throttle that produced it

The cut has to land on the bar, not past it, so the throttle comes down over the last
`t_taper` seconds of closing. The naive form — throttle proportional to the miss over its
measured closing rate — chases its own tail: throttling down slows the closing, which
lengthens the estimated time to go, which raises the throttle again.

Dividing the measured rate by the throttle that produced it removes the loop. `close` is
metres of miss per second **per unit throttle**, so a rate measured at half throttle
predicts twice that at full, and the commanded throttle `miss / (close · t_taper)` is a
fixed point rather than an oscillation. It is also the most useful single number the flight
records: it is this booster's range authority, in units anyone can check.

`close` is only updated above a tenth of throttle, where the measurement means something,
and it is never reset, so the taper cannot fall back to the full-throttle branch halfway
down and re-open the valve.

## The drag correction, and why it is filtered rather than flown directly

The impact predictor built on `landing_time` has no atmosphere in it. A booster returning
from 60 km at better than a kilometre a second sheds most of its range in the last twenty,
so the drag-free arc overshoots the real landing by something on the order of tens of
kilometres — which, on a problem whose whole prize is kilometres, is the entire problem.

When Trajectories is loaded it supplies a drag-aware impact. The obvious move is to fly the
loop on that number directly, and the script does not, for one reason: Trajectories
recomputes on its own clock, so its answer **steps**, while the loop wants a derivative. A
stepping input inside a rate-driven taper is exactly the way to build an oscillator.

So the two predictors are used for the two things each is good at. The drag-free one is
solved from the live orbit every cycle and is lag-free by construction, so it carries the
fast loop. The difference between them — `bias`, the displacement from the modelled impact
to the drag-free one — is the slow drag correction, and it is filtered over `tau_bias`
because it is a property of the vehicle and the arc, both of which change over the whole
burn rather than between ticks. The aim point is the target pushed out by `bias`, so
driving the drag-free impact onto the aim point puts the real one on the target.

Algebraically the unfiltered version of this collapses to flying Trajectories directly. The
filter is the entire content of the idea, and `bias` is logged so a flight says whether it
was worth the two lines.

With no Trajectories, `bias` stays zero, the loop aims drag-free, and the booster lands
short of the target along its direction of travel by exactly the amount the flight then
measures. That is the calibration case, and `use_tr false` selects it deliberately.

`range_bias` is the fallback that flight would feed: metres to aim beyond the target along
the approach. It is **the one number in the script that is not free of the craft** — it
stands in for a ballistic coefficient the drag-free predictor cannot know — and the
register says so rather than dressing it up. It should stay 0 whenever Trajectories is
answering.

## What the burn refuses, and where it stops

Two refusals, both cases where nothing the script does helps: a body with no atmosphere, so
there is no parachute landing to fly; and a periapsis above the atmosphere, which is an
orbit rather than a booster arc and wants a deorbit burn first.

Everything else is coped with rather than refused, on one principle: **a booster that does
not deploy its parachutes is not recovered at any distance.** No live engine skips the burn
and flies the descent. A burn that cannot close the miss still flies the descent. Every
named exit from the boost loop falls through to entry:

- `miss inside the bar` — the burn did its job.
- `miss growing past its best` — the overshoot guard above.
- `engine dry` / `reserve reached` — the propellant is spent.
- `below the burn floor` — `alt_floor`, under which a booster held broadside to the
  airflow is a structural and control problem this script has no model for, and the
  drag-free arc has stopped describing the flight at all.
- `no terrain crossing to aim at`, held for two seconds before it counts — a burn that
  lifted periapsis clear of the terrain has no impact point to close on, and continuing
  would be steering on a stale number.
- `burn deadline` — twice the propellant's own full-throttle burn time plus two minutes.
  The two minutes is `aerobrake.ks`'s flip allowance and its argument: a craft that cannot
  make the turn in two minutes will not make it.

## Chutes: ask early, ask often, and keep one backstop

`CHUTESSAFE` deploys only what can safely deploy at that moment and is a no-op otherwise
(`kos-facts.md`). That makes the arming policy trivial and removes a tunable that would
otherwise have been a guess: ask once a second from the atmospheric interface all the way
down. Asking early cannot open a canopy early, and it cannot miss the first safe moment.

The one thing `CHUTESSAFE` cannot cover is a parachute module kOS's safe check does not
know about. So `alt_release` is a hard deploy, unconditional, plus the handover: steering
released, controls neutralised, SAS back on, so nothing is fighting the canopies. Its
argument is that down there the booster is at terminal velocity in dense air — the
condition stock chutes are rated to open in — with tens of seconds of fall left.

Retrograde is held all the way to that point. It is the attitude that puts the most drag in
the way, which makes for the shortest, slowest, coolest entry, and it is the one the
canopies want when they open. Whether a spent booster's airframe will actually hold it is
**not assumed** — `steer_err` against `q` is the column that answers, and it is the single
most likely place this design is wrong.

One trap belongs to the craft rather than the script: retrograde points the *control
point's* nose backward along the flight path. A booster controlled from its upper end
therefore falls engine-first, mass ahead of drag, which is the stable way round and the
one this design assumes. Controlled from the other end it flies the unstable way, and no
steering command fixes it — the fix is to reverse the control-from part.

## `landing_site` was missing the body's rotation

`GEOPOSITIONOF` reads a position against the body's orientation *now*. `landing_site` fed
it a position seconds-to-minutes in the future and returned the answer unshifted, so the
longitude it reported was the one the ground had already turned away from — 175 m of
Kerbin's equator for every second of fall, 35 km over a three-minute ballistic arc.

For a column logged next to a Trajectories reading that error is a puzzling offset. For a
targeting loop it is larger than the entire problem, so the fix landed in `common.ks` with
the correction factored into `geoposition_ahead` and applied in all three places that
needed it — `landing_site`, `above_terrain`'s terrain sample, and `time_to_surface`. It is
the same correction, for the same reason, as `core/kepler.ks`'s `geoposition_at`; that one
is handed an orbit to propagate, this one is handed what `positionat` already returned.

`reentry.ks` reads `landing_site` for its `kep_lng` column and inherits the fix without
having flown it. Its comparison against `tr_lng` and the final `lng` gets more truthful, not
less, but the change is unwitnessed there and this register is where that is on the record.

## Pre-flight: the signatures

The log is `boostback_<t>.csv`: four rows a second through the burn, one a second either
side of it, and one console line a second throughout. The burn's rate is not decoration.
The throttle comes down over `t_taper` — two seconds — so at one row a second the taper
would arrive as two samples, and the taper is the part of this design most likely to be
wrong. `powered_descent.ks` splits its clocks the same way and for the same reason: a
console printing four times a second is not a console.

What a flight has to show, column by column:

1. **`miss` is flat across the SETTLE rows and falls only once `thr` goes above zero.** This
   is the invariant the whole design rests on. If `miss` drifts while the engine is cold,
   the predictor is not describing a ballistic arc and everything downstream is built on
   sand.
2. **`close` settles to a roughly constant value during the burn** — within a factor of two
   across the full-throttle portion. It is metres of miss per second per unit throttle; if
   it wanders by an order of magnitude, the taper is sized off noise and `t_taper` means
   nothing.
3. **`thr` holds at 1, then falls monotonically over roughly `t_taper` seconds, and the
   cutoff row reads `miss inside the bar` with `miss` ≤ 250.** A cutoff that reads
   `miss growing past its best` instead means the taper overshot and `t_taper` is too
   short. A `thr` that chatters between 0 and 1 means the airframe is losing `align_bar`,
   which shows up as `steer_err` above 10 in the same rows.
4. **`d_kep` and `d_tr` separate during ENTRY, and the final `dist` is nearer `d_tr` at the
   interface than `d_kep` is.** This is the claim that the drag correction earns its
   complexity. If the final `dist` sits nearer `d_kep`, the correction is aiming the wrong
   way and `bias` should be zeroed rather than tuned.
5. **`steer_err` during ENTRY stays bounded rather than growing without limit.** An
   unbounded `steer_err` with rising `q` is a booster tumbling, and the retrograde hold is
   the wrong design for this airframe.

The headline number is the last row's `dist`, and the honest comparison for it is the first
SETTLE row's `d_kep` — where the booster was going to land before anything was done.

## Open

- **Every tunable is unflown.** `t_taper`, `tau_bias`, `alt_floor`, `alt_release` and
  `t_settle` are arguments with numbers attached, not measurements.
- **No mid-course correction.** The boostback is the only burn, and it is committed before
  entry has measured any drag at all. Reserving a little propellant for a correction once
  the entry deceleration is readable is the obvious next lever, and it is the one that
  would make `range_bias` unnecessary. It is deliberately not built, because there is no
  flight yet to say how much of the miss survives the burn.
- **The entry attitude is a hold, not a control.** Nothing here trims the entry for range —
  a booster with any lift at all could steer some of the remaining miss out on the way
  down, which is what `entry-s-curve.md` is about for a different vehicle.
- **`compass_for` now lives in `aero.ks`, and `reentry.ks` still carries its own copy.**
  Folding that one in is behaviour-identical but touches a working entry script, so it
  waits for a flight that would witness it.
- **The pilot's abort ends the burn but not the parachutes.** An abort hands the controls
  back and stops the boost; `CHUTESSAFE` goes on being asked for every second, and the hard
  deploy at `alt_release` is tracked on its own flag so an abort high up cannot consume it.
  That is a deliberate choice about whose job the canopies are, and a pilot who wants them
  held has no way to say so.
- **Scope, standing:** Kerbin, an easterly launch, a booster that separates downrange and
  above `alt_floor`. The geometry is general — nothing in the loop assumes a direction —
  but nothing off that profile has been thought about.
