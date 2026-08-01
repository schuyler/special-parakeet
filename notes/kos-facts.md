# kOS facts, verified

*Settled from the docs and from the live game through `util/kos_bridge.py`. Don't
re-litigate these; if one turns out to be wrong, fix it here.*

- `nd:ORBIT` is the post-burn orbit patch. It **survives into and out of a `lexicon`**,
  and `remove` works on a node fetched back out.
- `orbit_at(t, orbit_)` (`reference/core/kepler.ks`) takes a **TimeStamp, not a scalar**.
  It works on a node's orbit, walked backward.
- A node patch answers `ORBIT:TRUEANOMALY` out of its cached state at `ORBIT:EPOCH`, and
  **that epoch is the node's own time**, while `ORBIT:POSITION` is the position *now*.
  Mix the two and the prediction comes out rotated by the true anomaly the patch sweeps
  between now and the node. Take the anomaly from `true_anomaly(time, orbit_)` instead —
  the same number on a live orbit. Measured 2026-07-25 over the Mun: eight nodes walked
  once around one orbit, the error tracked the sweep to 0.01 deg the whole way, 27 deg
  at a 200 s lead and 174 deg at 1200 s.
- `geoposition_at(t, orbit_, pos)` works for times before now; passing `pos` matches
  letting it recompute.
- **`BODY:GEOPOSITIONOF` reads a position against the body's orientation *now*.** It has no
  time argument and cannot: hand it a point t seconds in the future and the longitude that
  comes back is the one the ground will have turned away from by then, short by
  `(360/rotationperiod)·t`. Latitude is unaffected — the spin never moves it. On Kerbin the
  term is 175 m of equator per second, so a three-minute ballistic fall lands 35 km from
  where the uncorrected read says. Both propagators in this repo now carry the correction:
  `core/kepler.ks`'s `geoposition_at`, which is handed an orbit, and `common.ks`'s
  `geoposition_ahead`, which is handed what `positionat` already returned.
- `body:angularvel` is in **radians**/sec, so `|ω|·r ≡ 2π·r/T`.
- `geopositionlatlng(lat, lng)` works for arbitrary lat/lng, off-rails.
- **No body-global maximum-terrain suffix exists.** `TERRAINHEIGHT` is per
  `GeoCoordinates` only. Minmus peaks ≈ 5.7 km.
- Function forward references are fine.
- `orbital_speed` is defined once, in `reference/core/kepler.ks`. Pass orbit objects, not
  scalars — it reads the sma off the orbit.
- `r(pitch, yaw, roll)` applies its angles **roll, then pitch, then yaw**, and a
  **positive first angle turns the nose down** — negate it to pitch up. The docs give the
  order but not the sign; the sign is from a flight.
- A `Direction`'s top vector is only defined for the ones you build: `prograde` and friends
  carry an undocumented roll reference, so an `r()` about them turns about an unnamed axis.
  `lookdirup(fore, top)` fixes both vectors and makes the pitch axis the one you meant.
- kOS binds a script's `parameter` values *before* `run common.`, so a tunable whose
  default depends on an imported constant needs a sentinel (this repo uses -1) resolved
  after the imports.
- `ship:bounds` (a `Bounds`) is **expensive to obtain and should be captured once**:
  computing it walks every part's own bounds and re-runs the coordinate transforms.
  Suffixes read off an already-captured box are cheap — the "absolute" ones
  (`bottomaltradar`, `bottomalt`, `absmin`, `absmax`, `abscenter`, `absorigin`, `facing`,
  `furthestcorner`) **recompute themselves from the ship's current position/attitude on
  every read**, so a box captured once still tracks rotation and translation correctly.
  The box **goes stale on a shape change** — gear, solar panels, and cargo bay doors
  deploying or retracting, robotic parts moving, docking/undocking, staging, or a control-
  orientation change — so capture only after the shape that matters has settled.
  `bottomaltradar` is the radar-altitude reading from the box's lowest corner: the height
  of the craft's lowest point above the ground, as opposed to `alt:radar`, which reads
  from the part-tree origin and can sit metres above where the craft actually touches.
- A `Bounds`'s `SIZE` suffix is the vector from `RELMIN` to `RELMAX` — the ray diagonally
  across the whole box (equal to `EXTENTS * 2`), expressed in the box's own reference
  frame, so its `:mag` is a frame-independent diagonal length
  (ksp-kos.github.io/KOS/structures/vessels/bounds.html, verified 2026-07-27).
- **Not writing a raw control axis is not releasing it.** `SHIP:CONTROL` belongs to the
  vessel, not the program, so an axis keeps whatever was last written into it — including
  by an earlier run — and kOS goes on applying that value to any script holding raw
  control. A `mainthrottle` of 0.5 left by a previous run still reached the engines of a
  script that had stopped writing the axis. `SET SHIP:CONTROL:NEUTRALIZE TO TRUE` is what
  releases; writing zero is a command, not a release
  (ksp-kos.github.io/KOS/commands/flight/raw.html).
- **`and` short-circuits.** "kerboscript doesn't bother calculating the righthand side once
  the lefthand side is false", so `if i >= 2 and list[i-2] …` is safe on the first pass
  (ksp-kos.github.io/KOS/language/features.html, read 2026-07-30). Pre-1.0 builds did not,
  so don't make it load-bearing where a nested `if` costs nothing.
- **Ask the addon *list* whether an addon is available, not the addon.** `ADDONS:AVAILABLE("X")`
  returns false for an unregistered id; `ADDONS:X:AVAILABLE` **raises**, because `ADDONS:X`
  is itself a suffix that only exists for addons kOS has registered. Both suffixes exist —
  the base `Addon` class registers `AVAILABLE` on every addon — but only the list form is
  safe when the addon's own DLL may be absent.
- **`PIDLoop` differentiates the *measurement*, not the error**: `dTerm = -ChangeRate * Kd`
  where `ChangeRate` is `(input − last input) / dt`. So a moving setpoint never enters the
  derivative path, and `DTERM` reads as `Kd` times the rate of the measured quantity.
  `MAXOUTPUT`/`MINOUTPUT` are the anti-windup mechanism — kOS adjusts `ITERM` when the
  output saturates. `ITERM` is **read-only**, so an integrator cannot be seeded for a
  bumpless handover; `RESET()` clears `ERRORSUM`, `ITERM` and `LASTSAMPLETIME`
  (ksp-kos.github.io/KOS/structures/misc/pidloop.html, read 2026-07-30).
- **`dynamic_pressure()` in `reference/original/aero.ks` returns kPa, not Pa**, because
  `air_density()` divides a pressure already converted with `constant:atmtokpa` and so
  returns kg/L — 1.225e-3 at sea level. kOS also supplies `SHIP:Q` directly (sea-level
  Kerbin atmospheres; multiply by `ATMtokPa`), which `reentry.ks` already logs. A `q`
  column taken from `aero.ks` is our own model read back, not an independent witness.
- **Disabling AFBW does not release the throttle.** Its `rightClickDisabled` flag guards
  only the joystick-polling loop; `UpdateFlightProperties` runs unguarded and applies a
  latched throttle offset additively as `mainThrottle = clamp(commanded + latch, 0, 1)`,
  and `ZeroOutFlightProperties` clears the rotational axes' value but only the throttle's
  velocity and acceleration. Measured across three flights commanding 0.5: 0.975, 1.000
  and 0.989, the last with AFBW confirmed switched off by the script. The kOS-AFBW bridge
  now clears the latch inside its `ENABLED` setter, and only on a write — so `afbw.ks`
  writes `ENABLED` unconditionally rather than skipping when it already reads false.
- **`CHUTESSAFE` is safe to ask for repeatedly, and asking is the whole arming policy.** It
  "deploys all the parachutes than can be safely deployed in the current conditions (only
  ON command has effect)", so setting it true when nothing can safely deploy is a no-op
  rather than a torn canopy — which means a descent can simply ask once a second from the
  atmospheric interface down and never need an altitude or a dynamic-pressure threshold of
  its own. `CHUTES` is the unconditional form and will deploy into conditions that destroy
  the canopy. **Do not read `CHUTESSAFE` as "are the chutes out":** it "returns false only
  if there are disarmed parachutes chutes which may be safely deployed, and true if all
  safe parachutes are already deployed including any time where there are no safe
  parachutes" — so it reads true both when everything is deployed and when nothing may be
  (ksp-kos.github.io/KOS/commands/flight/systems.html, read 2026-08-01).
- **Trim goes through `ship:control:pilotpitchtrim`, not `ship:control:pitchtrim`.** The
  raw-control one "has no real effect and is just here for completeness" — KSP reads trim
  only off the *pilot's* control structure, never an autopilot's. The pilot ones are
  settable, scalar [-1,1], and unlike `pilotpitch` are explicitly meant for autopilot use.
  Trim written to `pilotpitchtrim` **survives `NEUTRALIZE` and reaches the surfaces**:
  flown 2026-07-30 (`autotrim_82920.csv`), where releasing the elevator to zero left the
  nose held and `gamma` flat, −2.33 to −2.26 across a 3 s window, with the `trim` column
  reading back the written 0.118 on every row. Whether trim also acts while a script is
  *actively writing* raw `ship:control:pitch` is still untested — that flight neutralized
  the raw axes first, which is exactly the case it does not cover
  (ksp-kos.github.io/KOS/commands/flight/raw.html and .../pilot.html, read 2026-07-30).
