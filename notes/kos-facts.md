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
