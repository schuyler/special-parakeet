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

## Engagement now starts from the attitude already being flown, separately

`level_flight.ks` captures `pitch_at_engage` at setup, and the outer loop
commands a deviation from it rather than an absolute attitude. An aircraft
in level flight sits nose-up -- it needs angle of attack to make lift -- so
a command of zero degrees was a command to dive. This is independent of the
integral term above, and three flights to the same target (~8979 m ASL,
same aircraft) separate their contributions:

| configuration          | peak err |
|-------------------------|----------|
| neither                 | 118.8 m  |
| engagement datum only   | 75.1 m   |
| datum + `pitch_ki`      | 20.8 m   |

The last configuration settles to -0.2 m. This is the structural version of
a pilot observation from before either change: starting from level flight
with SAS on, switching SAS off and engaging the script produced no large
dip, because a craft already trimmed needs almost no elevator and has
nothing to build. Capturing the attitude already being flown gets the same
effect without depending on the pilot having trimmed first.

## Dynamic pressure resolves the scaling anomaly

The anomaly wasn't physical. `elev x tas^2` is the wrong invariant, because
`tas^2` is not dynamic pressure -- density is missing. `level_flight.ks` now
logs `q` from `dynamic_pressure()`. With real `q`:

|              | alt    | tas   | deflection | q     | deflection x q |
|--------------|--------|-------|------------|-------|-----------------|
| autotrim     | 6634 m | 278.5 | 0.118      | 19.7  | 2.32            |
| level_flight | 8980 m | 282.6 | 0.182      | 14.07 | 2.56            |

Nearly identical airspeed, 2350 m apart. Raw deflections differ by 54%;
multiplied by `q` they agree to 10% -- `k` ~ 2.4, provisional. Two points,
and the level_flight row's trim state is unrecorded (`pilotpitchtrim` wasn't
yet in the settings line when it flew, which is why it is now). The 232 m/s
row from the old elevator table has no `q` on record and cannot be
rechecked; it stays flagged, unresolved, for that reason alone.

## Throttle: the AFBW latch is fixed

A rebuilt kOS-AFBW bridge clears AFBW's latched throttle offset when
`ENABLED` goes false. Every flight since reads `thr` = `cthr` = 0.5,
confirmed across `level_flight_85328.csv`, `_85487.csv`, and `_86066.csv`.
The mechanism (disabling AFBW stopped it polling the joystick but left a
latched value applied additively to `mainThrottle`) can now be stated as
established rather than a working explanation. The entry for `kos-facts.md`
has not been written yet; that file is not edited from here.

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

- **`pitch_range` now clamps deviation from the engagement attitude**, not
  absolute pitch, so reachable attitude is `pitch_at_engage +/- 30`.
  Engaging in a steep attitude eats the loop's authority -- a new way to
  lose it that did not exist before. Not live in any flight logged so far:
  peak commanded deviation fell from 7.13 deg (no datum) to 4.35 deg
  (datum only) to 1.11 deg (datum + `pitch_ki`) across the three flights
  above, all far under 30.
- **`pitch_kd` is unchanged at 0.005** while `kp` went 0.01 -> 0.05, so the
  derivative time `kd/kp` fell from 0.5 s to 0.1 s. Nothing rings at 0.05,
  so no action; raise `kd` toward 0.025 first if a further `kp` increase
  oscillates.
