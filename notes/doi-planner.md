# The DOI planner

*The design register for `reference/original/plan_doi.ks`: one node that drops a circular
parking orbit onto a descent ellipse whose periapsis is PDI, over the landing site.
Companions: `powered-descent-invariants.md` (the flight controller this plans for),
`powered-descent-handoff-contract.md` (the braking→terminal seam),
`terrain-certification.md` (the terrain analysis this planner defers, and where it would
attach).*

## Why nothing here searches

The parking orbit must be circular, and that assumption earns the closed form. On a
circular orbit the DOI burn is tangential wherever it fires, so periapsis lands exactly
180 degrees ahead and the node follows without feedback — no placement loop, no fixed
point. Where periapsis actually lands is reported, not corrected: a drift means the
parking orbit was not circular enough, and the pilot sees the number.

`pdi_height` is a dial. The planner marches the braking arc once, prices it, and places
one node.

## The fuel lever is the PDI altitude, not the throttle

This is the finding that decides where effort goes, and it is telemetry, not theory. From
a measured PDI state — 172.2 m/s at 2278 m over Minmus, `g0` 0.49 m/s² read off the log's
own engine-off free-fall rows — the least delta-v that brings the ship to rest at the site
is `sqrt(v² + 2·g·h) ≈ 178 m/s`. The flight spent 214.

Braking harder does not close that gap. Killing the 172 m/s at `f_max` takes ~175 m/s and
leaves the ship slow at ~2.2 km; arresting the fall from there costs another ~46–48, for
~222 total — about 8 m/s *more* than the shallow arc actually flew. At a fixed periapsis
the throttle only trades a longer burn's support losses against a shorter burn's fall
arrest, and those prices differ by a few percent. The term that moves the budget is
`2·g·h`.

So the planner owns the fuel, and it owns it through one number: how high PDI sits.

## The chord certificate

The pilot certifies terrain by eye on the map. What makes that tractable is that a
*straight line* suffices for the braking arc: the chord from the handoff point `(0,
h_handoff)` to PDI `(X, h_pdi)`, with x measured along the ground from the site, lies
under every braking arc the flight controller can fly from that PDI. Clear the chord and
the arc is clear — geometrically, with no sampling of the trajectory.

The obvious argument for this is wrong, and the repair matters. "The arc leaves PDI level
and steepens monotonically, so `h(x)` is concave" is false at the start: at PDI the ship
is slightly super-circular (172.2 m/s against a circular 168 at that radius), so the turn
rate `v/r − g/v` is positive and the path pitches *up* — the flown log shows the first
~15 s climbing. The certificate runs in two spans instead:

- **At or above `h_pdi`.** From ignition the path rises, peaks, and descends back through
  `h_pdi`. Throughout, `h ≥ h_pdi`, and the chord's greatest height is `h_pdi` at its PDI
  end, so the path clears it trivially.
- **Below `h_pdi`.** By the time the path re-crosses `h_pdi` it is descending and
  sub-circular, and sub-circularity is preserved for the rest of the descent: `v` falls
  under braking while the bound `sqrt(mu/r)` rises as `r` falls, and even a zero-throttle
  segment gains only `2·g·Δh` of `v²` against a bound thirty times larger at braking
  speeds. Sub-circular, the turn rate is negative at every throttle — it contains no
  throttle term — so pitch decreases monotonically, `h(x)` is concave, and the path lies
  above the straight segment joining its `h_pdi` re-crossing to its endpoint. Both ends of
  that segment sit on or above the chord, and the chord is a line, so the segment clears
  it and the path above it does too.

The one non-geometric premise is the endpoint. The planner's nominal arc ends at the
chord's own anchor by construction; flown arcs end earlier and *higher*, because the
flight controller hands off at the attitude seam with its stopping distance still in hand
— flight 7 reached the seam at 627 m radar where the chord stood at ~140 m. A seam
arriving *at* the chord is a marginal handoff the flight already warns about. Below the
seam the craft is in terminal's near-vertical cone over the site, whose terrain is the
anchor itself.

The certificate covers the braking arc only. Up-range of PDI it says nothing useful — see
`terrain-certification.md`, which is also where the certificate would become machinery
rather than a pilot's straightedge.

## `f_headroom`

The share of the throttle ceiling the plan may not spend. PDI is placed at the reach of a
brake at `(1 − f_headroom)·f_max` rather than at `f_max`, so the extra ground that lower
throttle covers is the flight controller's reserve for shortening the arc. Self-scaled to
the craft's thrust and the body's gravity instead of a fixed distance factor.
Dimensionless, 0.1, provisional until a flight falsifies it.

## Open

- `pdi_height` is a dial, so nothing prices whether it is the *right* altitude. Making it
  a solve is `terrain-certification.md`'s subject.
- The ignition fallback `f_cmd = f_max` conflates `bisect`'s two bracket failures — safe
  either way, but the log cannot tell which happened.
- `bisect`'s failure path prints four lines, which would tear the fixed-row readout
  mid-burn.
