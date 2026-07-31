# level_flight.ks: what is still to tune

Current gains, flown on the SSTO from 6.6 km to 9.0 km ASL:

    alt   (m -> deg)  kp 0.05   ki 0.001  kd 0
    pitch (deg -> elevator)  kp 0.05  ki 0.025  kd 0.005
    roll  (deg -> aileron)   kp 0.01  ki 0  kd 0.005

The cascade holds altitude and, from every capture condition flown, drives
error to a small residual within a few minutes. Everything below is a known
shortfall or open question, not a suspicion.

## The standing-error defect is fixed (`pitch_ki` 0.025)

`pitch_pid` was proportional plus derivative: holding an attitude needs a
non-zero elevator deflection, and a PD loop can only produce one by carrying
an error. `pitch_ki` 0.025 gives it a 2 s integral time -- fast against the
phugoid, slow against the one-second short-period oscillation the derivative
term damps, and a 0.2 deflection against a +/-1 clamp leaves room to wind.
kOS's `PIDLoop` clamps `ITERM` when the output saturates, so windup is
already bounded by the existing limits.

Settled rows of `level_flight_86066.csv` read `cmd - pitch` = 0.00 with
`elev` holding 0.182 -- the integral carries the full deflection with no
error at all. `cmd` is therefore a pitch command with no hidden offset,
`pitch_range` clamps the nose attitude (see below for a new wrinkle), and
there is no standing gap left to convert into sag on engagement.

For the record, what the defect looked like -- settled elevator against a
standing gap, and sag against vertical speed at capture:

| tas (m/s) | elev  | cmd - pitch |
|-----------|-------|-------------|
| 232 †     | 0.168 | 3.36        |
| 291.5 †   | 0.152 | 3.04        |
| 335 †     | 0.107 | 2.14        |
| 531 †     | 0.043 | 0.86        |
| 533 †     | 0.045 | 0.90        |

| kp   | v/s at capture | peak err | time to peak |
|------|-----------------|----------|---------------|
| 0.01 | 6.8 m/s sink  † | 499 m    | --            |
| 0.05 | 2.1 m/s climb   | 126.3 m  | 12 s          |
| 0.05 | 5.4 m/s sink  † | 41 m     | 6-8 s         |
| 0.05 | 13.1 m/s sink † | 107 m    | 6-8 s         |
| 0.05 | 18.1 m/s sink † | 78 m     | 6 s           |

† no surviving CSV. Three rows in each table were never logged as CSVs.
Two more (533 m/s, 18.1 m/s sink) were logged to `level_flight.csv`, but
that file has since been overwritten by an unrelated flight -- a second
file lost to the destroy-on-run behavior the script no longer has (it now
writes a timestamped file per flight). The 2.1 m/s climb row is still
reproducible, from the recovered partial level-engagement log.

Of three candidate fixes for this defect, the integral term is the one
built, and it works. `autotrim.ks` trim and an in-loop bias remain valid,
untested alternatives. A derivative term on `alt_pid`, which would have
reached the required command sooner, is now moot: the integral supplies the
command outright, so there is no gap left to reach sooner.

## Engagement now starts from the angle of attack already being flown, separately

`level_flight.ks` captures `aoa_at_engage is pitch_angle() - angle_of_ascent()`
at setup, and the outer loop commands a deviation from it rather than an
absolute attitude. An aircraft in level flight sits nose-up -- it needs
angle of attack to make lift -- so a command of zero degrees was a command
to dive.

The datum is angle of attack rather than pitch because pitch is angle of
attack plus flight path angle, so sampling pitch imports whatever the
flight path was doing at that instant -- and since `pitch_range` bounds the
*deviation* from the datum, a datum inflated by a climb leaves little
authority in the other direction. `level_flight_82698.csv`, flown before
this change, shows why: engaged at 26 deg of ascent, it took a pitch datum
of 28.49 deg, leaving only `28.49 - 30` = 1.5 deg of nose-down authority
against the 30 deg clamp. Recovery was authority-limited, not
gain-limited: over one representative 45 s stretch mid-recovery, `err`
improved by only about 100 m (156 to 59), roughly 2 m/s. Angle of attack
stays within a few degrees whatever the flight path is doing, so it doesn't
have this failure mode.

Four flights to the same target (~8980 m ASL, same aircraft) separate the
pitch-datum change from `pitch_ki` (above) -- **but all four flew with the
pitch datum**, not the current angle-of-attack one; the rename came after:

| configuration              | peak err |
|-----------------------------|----------|
| neither (`_83092.csv`)      | 118.8 m  |
| neither (`_85328.csv`)      | 144.4 m  |
| pitch datum only            | 75.1 m   |
| pitch datum + `pitch_ki`    | 20.8 m   |

Both "neither" flights targeted the same 8984 m; 118.8 and 144.4 are 21%
apart, which is the scatter any one of these figures carries on its own --
the progression is only meaningful as a trend against that noise floor, not
as exact numbers. The last configuration settles to -0.2 m. This is the structural version of
a pilot observation from before either change: starting from level flight
with SAS on, switching SAS off and engaging the script produced no large
dip, because a craft already trimmed needs almost no elevator and has
nothing to build. Capturing the attitude already being flown gets the same
effect without depending on the pilot having trimmed first -- but the case
that distinguishes the angle-of-attack datum from the pitch datum
(engaging while climbing or descending) is still unflown under the current
code. Every angle-of-attack-datum flight on record engaged within a couple
of degrees of level, where the two definitions coincide, so this
progression is evidence for the pitch datum, not yet for its replacement.

## Dynamic pressure resolves the scaling anomaly

The anomaly wasn't physical. `elev x tas^2` is the wrong invariant, because
`tas^2` is not dynamic pressure -- density is missing. `level_flight.ks` now
logs `q` (kPa) from `dynamic_pressure()`: `aero.ks`'s `air_density()` works
in kg/L (it converts pressure with `constant:atmtokpa` before the ideal-gas
division, so sea level comes out 1.225e-3), and `dynamic_pressure()` is
`air_density() x tas^2 / 2` off that, which comes out in kPa, not Pa. With
real `q`:

|              | alt    | tas   | deflection | q (kPa) | deflection x q |
|--------------|--------|-------|------------|---------|-----------------|
| autotrim     | 6634 m | 278.5 | 0.118      | 19.7    | 2.32            |
| level_flight | 8980 m | 282.6 | 0.182      | 14.07   | 2.56            |

Nearly identical airspeed, 2350 m apart. Raw deflections differ by 54%;
multiplied by `q` they agree to 10% -- `k` ~ 2.4, provisional (the units
cancel in the ratio, so this conclusion doesn't depend on kPa vs. Pa). Two
points, and the level_flight row's trim state is unrecorded
(`pilotpitchtrim` wasn't yet in the settings line when it flew, which is
why it is now). The 232 m/s row from the old elevator table has no `q` on
record and cannot be rechecked; it stays flagged, unresolved, for that
reason alone.

**Open**: `q` is a model, not a measurement. `dynamic_pressure()` is a
hand-rolled ideal-gas calculation from the altitude tables; it isn't
cross-checked against anything kOS measures directly. kOS exposes `SHIP:Q`
(documented as sea-level Kerbin atmospheres -- multiply by `ATMtokPa` for
kPa), and `reentry.ks` already logs it. Right now the `q` column is an echo
of our own model rather than an independent witness. Logging both in the
same flight would settle whether they agree and pin the units; not done
yet.

The cheapest remaining test of `deflection x q` doesn't need a code change:
with `hold_throttle` false, sweeping the throttle through its range in a
single flight, `q` logged every second, would check the constant across
the whole speed range instead of resting on two points at similar speed.

## Throttle: the AFBW latch is fixed, and the script no longer holds the axis by default

A rebuilt kOS-AFBW bridge clears AFBW's latched throttle offset when
`ENABLED` goes false. Every flight since reads `thr` = `cthr` = 0.5,
confirmed across `level_flight_85328.csv`, `_85487.csv`, and `_86066.csv`.
The mechanism (disabling AFBW stopped it polling the joystick but left a
latched value applied additively to `mainThrottle`) can now be stated as
established rather than a working explanation. The entry for `kos-facts.md`
has not been written yet; that file is not edited from here.

`hold_throttle` is a new tunable, default `false`: the script no longer
commands the throttle unless asked, and the pilot's lever is live during a
run. The script also neutralizes every control axis once at setup, before
taking pitch and roll -- the raw control structure belongs to the vessel
and keeps whatever a previous run last wrote to it, including a stale
`mainthrottle` applied to a script that has since stopped writing that
axis. Both are recorded in the settings line.

Consequence for the tuning work: with `hold_throttle` false, airspeed is no
longer held constant across a flight by the script, so two flights are only
comparable through `deflection x q`, not through raw `elev` or `tas` alone
-- which is why the previous section's open item (whether `q` itself is
trustworthy) matters more now than it did.

## autotrim: trim reaches the surfaces and survives NEUTRALIZE

`autotrim_82920.csv`, `max_pass 1`: the pass limit stopped it
(`stop_reason no_convergence` is the expected outcome at one pass, not a
search failure), trim 0.1183.

The release window is the discriminator. Elevator goes to zero and stays
there for 3 s while `gamma` holds flat, -2.33 to -2.26, and `pitch` keeps
rising, 0.14 to 0.22 -- the nose held on trim alone with no active pitch
control. `trim` (`pilotpitchtrim`, read back) holds 0.118 on every release
row, including through whatever reset NEUTRALIZE performs elsewhere in the
sequence.

A design prediction checks out: the settle phase should rest at
`gamma = -elev / pitch_kp`, predicted -2.36 from the settle row's elev
0.118; measured -2.34.

One unpredicted effect: `thr` reads 0 through the release window, because
`NEUTRALIZE` drops the throttle along with everything else. Cost 4.6 m/s
and 2.5% of `q` over the 3 s here; a longer release window would measure a
decelerating aircraft, not a steady one.

Be precise about what this licenses: trim written to `pilotpitchtrim`
survives `NEUTRALIZE` and reaches the surfaces. It does not establish that
trim holds while a script is actively writing raw `ship:control:pitch` --
the release window neutralizes exactly that.

## Open: engaging while climbing or descending still targets the altitude being left

With the engagement datum in place, the aircraft holds its attitude first
and pulls over gradually rather than reversing hard -- but it still travels
further from the target before turning around, because `target_alt` is
still captured at the instant of engagement regardless of the vertical
speed at that instant. Two candidate answers: refuse to engage above a
vertical-speed bound, or fly a level-off phase that drives vertical speed
to zero before capturing `target_alt`. A level-off phase needs its own
integral action on vertical speed, for the same reason the pitch loop
needed one on attitude.

## Smaller items

- **`pitch_range` clamps deviation from the engagement datum**, not
  absolute pitch, so reachable attitude is `aoa_at_engage +/- 30` (was
  `pitch_at_engage +/- 30` on the flights in the table above). Engaging
  with an inflated datum eats the loop's authority -- see
  `level_flight_82698.csv` above for what that costs. Not live on any
  angle-of-attack-datum flight logged so far, because none has engaged far
  from level; peak commanded deviation on the pitch-datum flights above
  fell from 7.13 deg (no datum) to 4.35 deg (datum only) to 1.11 deg
  (datum + `pitch_ki`), all far under 30.
- **`pitch_kd` is unchanged at 0.005** while `kp` went 0.01 -> 0.05, so the
  derivative time `kd/kp` fell from 0.5 s to 0.1 s. Nothing rings at 0.05,
  so no action; raise `kd` toward 0.025 first if a further `kp` increase
  oscillates.
