# S-curve entry: range control by bank, held in reserve

*Not built, and not next. The first range-control instrument for the spaceplane entry is
pitch within a bounded band, designed against `entry_flight.csv` (the recorder in
`reference/original/reentry.ks`); this note registers the strategy behind it in case the
pitch band proves too narrow. Companions: `plan_entry.ks` (the aim the entry flies from)
and the recorder's column commentary.*

## What the strategy is

Hold angle of attack fixed and roll the lift vector instead. At bank angle φ the lift
`L` splits into `L·cos φ` vertical and `L·sin φ` horizontal:

- **Downrange** rides the vertical part. A gliding entry's range scales with the
  effective vertical L/D, so banking spills it continuously — `cos φ` runs from full
  glide at wings level to none at 90° — without touching the pitch attitude at all.
  Spilled lift also means a faster sink into denser air, so drag rises and the
  shortening compounds.
- **Crossrange** accumulates from the horizontal part, which is the cost: a banked
  entry turns. The remedy is to alternate the bank's sign each time crossrange error
  crosses a deadband, which draws the S on the ground track and bounds the excursion.

This is the Shuttle's solution, and the reason it is elegant is the decoupling: the
thermal and stability problem lives entirely in AoA, which never moves, while the energy
and range problem lives entirely in bank. Pitch modulation spends exactly the quantity
that is scarce during entry — pitch margin against departure — to buy range authority.
Bank modulation leaves that margin alone and spends lift direction instead.

## Why it is deferred here

Four arguments, one of them a craft fact.

1. **The craft says no, for now.** The Mk3 SSTO departs — tumbles — unless the control
   surfaces are held steady through the hypersonic phase. That is a flight report, not
   yet a logged measurement, but it is the pilot's own airframe speaking, and a bank
   reversal is precisely a large commanded attitude excursion at the worst part of the
   envelope. Until `steer_err` and `angvel` against `q` show margin where the S-turns
   would fly, hypersonic banking is off the table. The report is falsifiable the same
   way the pitch band is: the recorder flies free on every entry.
2. **`reentry.ks` flies wings-level by design.** Roll 0 with the wing axis parallel to
   the ground is part of the attitude-stability posture that keeps the craft pointed
   through the plasma; banking abandons it.
3. **Crossrange is a second controlled state.** Pitch-in-a-band leaves the ground track
   where the deorbit plan put it. A banked entry must actively repay every degree of
   crossrange it borrows, which means a reversal logic, a deadband, and a failure mode
   (an unrepaid S lands beside the target, not short of it) that the pitch instrument
   simply does not have.
4. **Below 800 m/s the pilot has the stick.** Subsonic S-turns to bleed leftover energy
   on final are manual technique and need no program. The strategy's whole value is
   hypersonic and supersonic, which is exactly where arguments 1 and 2 bind.

## When it earns a build

The pitch band's authority and the aim's residual are both measurable, and the decision
is their comparison:

- The residual: touchdown longitude minus `plan_entry`'s aim, across flights of one
  craft — the scatter left after the per-craft dial has been walked in.
- The authority: range moved per degree of pitch, from the doublet flight the recorder's
  design anticipates, integrated over the band the margin columns permit.

If the scatter exceeds what the band can absorb, pitch has failed as the sole
instrument, and bank is the successor — not a steeper pitch band, because the departure
bound is a cliff, not a dial.

## Qualification path, if that day comes

Same recorder, same one-change-per-flight discipline, and the q-ladder is climbed from
the bottom:

1. Passive margins first: the entries already flown map `steer_err` and `angvel`
   against `q` at wings level. No new flight needed.
2. A single bank doublet (±10°, held a few seconds, auto-abort to wings level on a
   `steer_err` threshold) at low supersonic, where the craft is known docile. Measures
   crossrange rate per degree of bank and the attitude cost of the reversal.
3. Walk the doublet up the q ladder one flight at a time until the margin columns
   object. The highest quiet rung is `q_max` for banking; the S-curve controller, if
   ever written, operates below it and stands down above it, exactly as the pitch
   controller respects its band.

Testable signatures for the doublet: `angvel` spikes at the reversal and decays within
seconds (docile) or grows (departure — abort fired); the ground-track longitude drift
per second of held bank gives the crossrange gain; the Trajectories column `tr_lng`
should shift west during the doublet by the spilled-lift range loss.

φ_max gets set the way the pitch bounds are: from the measured attitude margin, per
craft, a dial with an argument rather than a fitted constant.
