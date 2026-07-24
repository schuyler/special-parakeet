# kOS facts, verified

*Settled from the docs and from the live game through `util/kos_bridge.py`. Don't
re-litigate these; if one turns out to be wrong, fix it here.*

- `nd:ORBIT` is the post-burn orbit patch. It **survives into and out of a `lexicon`**,
  and `remove` works on a node fetched back out.
- `orbit_at(t, orbit_)` (`reference/core/kepler.ks`) takes a **TimeStamp, not a scalar**.
  It works on a node's orbit, walked backward.
- `geoposition_at(t, orbit_, pos)` works for times before now; passing `pos` matches
  letting it recompute.
- `body:angularvel` is in **radians**/sec, so `|ω|·r ≡ 2π·r/T`.
- `geopositionlatlng(lat, lng)` works for arbitrary lat/lng, off-rails.
- **No body-global maximum-terrain suffix exists.** `TERRAINHEIGHT` is per
  `GeoCoordinates` only. Minmus peaks ≈ 5.7 km.
- Function forward references are fine.
- `orbital_speed` is defined once, in `reference/core/kepler.ks`. Pass orbit objects, not
  scalars — it reads the sma off the orbit.
- kOS binds a script's `parameter` values *before* `run common.`, so a tunable whose
  default depends on an imported constant needs a sentinel (this repo uses -1) resolved
  after the imports.
