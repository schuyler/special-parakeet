# The node delivery window: what three identical plans measured

*A findings register for DOI node execution. The planner's placement is exonerated; its
delivery is not. Witnesses: `doi_plan_20260726_baseline.log`,
`doi_plan_20260726_fixbatch.log` and `doi_plan_20260727_klumpp.log` (runs of
`plan_doi.ks`), and the flight CSV headers of
`flight_log_20260726_klumpp_terminal.csv`, `flight_log_20260726_fixbatch_1.csv` and
`flight_log_20260727_klumpp.csv`. **The crash flight's own witnesses are gone** — its CSV
and plan log were the undated `flight_log.csv` and `doi_plan.log`, later runs overwrote
them, and neither was ever in git; its row below is what was read off them while they
existed. Companions: `doi-planner.md` (the planner),
`retrograde-terminal-findings.md` (the flights), `klumpp-descent-redesign.md` (the design
that takes this window as an input).*

## The measurement

Three flights flew the same craft from the same parking orbit against three runs of the
same plan. The plan logs are byte-identical except the node's ETA (117, 284, 340 s —
the same orbit re-planned from different phases): parking 19.4 × 24.5 km, e 0.0115;
h_pdi 5982 m, delivered by the node with 0 m of planning miss all three times; arc reach
39,198 m at f_cap 0.765, plus a 43 m stop term, so the plan expects ignition
39,241 m from the site. (The headers' `dist` is great-circle where the plan's reach is
in-plane; at the measured ~1.3 km cross-range the difference is 18–21 m, common to all
rows and negligible against the errors below.)

What each flight measured at ignition, from its CSV header:

| flight | plan eta | h_pdi delivered | speed_pdi | dist at ignition | error |
|---|---|---|---|---|---|
| klumpp terminal | 117 s | 5992 m | 563.7 | 39,718 m | **+477 m** |
| fixbatch | 284 s | 5992 m | 563.7 | 39,166 m | **−75 m** |
| crash | 340 s | 6008 m | 563.6 | 40,555 m | **+1,314 m** |

The signature: periapsis *altitude* delivered within 10–26 m of plan and speed within
0.1 m/s, every time, while along-track *position* scattered across 1.4 km. The orbit
shape arrives right; the apsis line does not point where it was planned to.

A fourth delivery, on a different craft and site (2026-07-27, an 11 m/s node at eta 300 s
against a planned lead of 18,128 m): h_pdi 4011 m delivered against 4004 m planned, speed
568.3, **18,042 m at ignition — 86 m short**. The pe-longitude witness reads it
independently: −88.828° delivered against the plan's −88.870°, 0.042° east, ~147 m of
ground track at the Mun's radius. The two measures disagree by the cross-range and
latitude terms the great-circle `dist` carries and the longitude difference does not;
both say the same thing, that this delivery was an order of magnitude tighter than the
2026-07-26 spread. One sample on one craft does not move the ±1.5 km bound, but it is the
first delivery measured by the witness rather than inferred from a distance column.

## Attribution — models consistent with the data, none yet a measurement

`next.ks` executes the 9.9 m/s DOI node as a ~2 s full-throttle burn, steering at the
live node vector with a 0.25° alignment tolerance, and writes no file witness of when it
burned or what residual it left. The mechanisms that move the apsis line while leaving
periapsis altitude nearly untouched — the observed signature — sized:

- **Radial thrust error.** The burn sits at true anomaly ν ≈ 165° of the descent
  ellipse (per `doi-planner.md`'s measured 195.3° burn-to-periapsis arc). Gauss's
  equation gives the apsis rotation per radial impulse as
  Δω = |cos ν|·√(p/μ)·δv_r/e; with the logs' ellipse (r_pe 205,982 m, r_ap ≈ 223,073 m
  → e ≈ 0.0398, p ≈ 214,190 m) that is 0.044 rad per m/s, or **~90 m of ground track
  per cm/s** of radial error. The 0.25° alignment tolerance permits
  9.9·sin(0.25°) ≈ 4 cm/s, ~380 m.
- **Burn-centroid timing.** Delivering the impulse centred δt late moves the burn point
  δθ = (v/r)·δt along the orbit, and the apsis line with it: **~470 m of ground track
  per second**. The planned asymmetries — taper onset, engine spool — shift the
  centroid by ~0.26 s, roughly 120 m, perhaps 190 m with spool.

**Sized like this, the mechanisms fall about 3× short of the observed +1,314 m**
(~500–570 m combined at their tolerance limits, and the taper's lateness is systematic
while the fixbatch flight delivered −75 m, so the signs do not fit either). The
candidate that does cover the spread is untested: the taper delivers half the impulse —
5 of 9.9 m/s — over ~6 s while `lock steering to np` chases `nd:deltav`, a vector whose
direction degrades as its magnitude collapses toward the cutoff. Five m/s delivered at
an average 3° of radial pointing error is ~26 cm/s of radial impulse, ~2.3 km of apsis
motion — ample, and of either sign depending on where the residual vector wanders.
Nothing records that pointing history today, which is the strongest argument for the
witness line below. Periapsis altitude corroborates the overall scale: the +10 to
+26 m altitude misses correspond to ~0.7–1.8 cm/s of prograde residual (sensitivity
~1,468 m of pe altitude per m/s at this burn), consistent with the taper's 0.01 m/s
cutoff plus slop.

## The design consequence

Holding pe placement to ±100 m would need the burn centroid held to ~0.2 s and radial
thrust to ~1 cm/s. No executor in this repo is built to that, and small manual burns
will be worse. **Along-track delivery error of order ±1.5 km is structural. The descent
must tolerate the window; the planner cannot remove it.**

The crash flight is what intolerance looks like: +1,314 m of delivery pushed the
throttle solve down to 0.739, whose arc's high gate sat 4.4 s above touchdown from
ignition (header `t_go 151`/`t_brake 146.6`), and ordinary model error spent the rest —
see `retrograde-terminal-findings.md` and the invariants register's unimplemented
low-side feasibility clause. A feedback law with command margin absorbs the same error
by bending the trajectory early, for single-digit m/s; `klumpp-descent-redesign.md`
carries that accounting.

## The witness

`powered_descent.ks` logs one line at warp-out, before anything ignites: the body-fixed
longitude periapsis will arrive at, the site's longitude, and the lead between them. Given
the planner's wanted longitude as its `plan_pe_lng` parameter it also logs `want` and the
signed error; the parameter defaults to 999, which means not supplied and logs the
delivered longitude alone. Every descent is therefore a measurement of this window,
whatever the architecture below it.

Two gaps in the instrument as it stands. The runner scripts do not pass `plan_pe_lng`, so
the `err` term is computed by hand against the plan log rather than logged. And the plan
log is `doi_plan.log`, undated: archive it beside the CSV or the comparison has nothing to
run against — which is exactly how the crash flight's measurement was lost.

## Open

- **The taper-pointing mechanism is the leading candidate and is untested.** The
  witness line measures its net effect; if the executor is ever instrumented directly,
  the pointing history of the last 5 m/s is the record to keep.
- **A gentler executor is a testable hypothesis, and it is now unblocked.** Capping the
  executor's throttle so the burn takes ~20 s shrinks every mechanism: the taper becomes
  a small fraction of the burn, the centroid error shrinks with it, and pointing slop
  averages down over a longer arc. Signature if flown: the pe-longitude witness across
  three deliveries lands within ±0.1° (~350 m) instead of the current spread. The witness
  exists and has data, so the hypothesis can be falsified; it is the live item on the
  design's staged rollout (`klumpp-descent-redesign.md`).
- The sample is small and now spans two craft; the ±1.5 km figure is a bound argued from
  mechanism sizes, not a distribution. The witness line turns every flight into a sample,
  but only the flights whose plan logs are archived can be read as one.
