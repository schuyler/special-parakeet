# Ballistic targeting

*The design register for `reference/original/targeting.ks` — the loop that burns a
ballistic impact point onto a chosen place on the ground — and for
`reference/original/ballistic_hop.ks`, its second consumer. The first is
`boostback.ks`, whose own register is `booster-return-to-pad.md`; read that one for the
descent half of the problem, this one for the part the two scripts share.*

**Nothing here has been flown.** Every tunable in both files says so, and the signatures
below are predictions. Both files sit at step 3 of the flight pipeline.

## The two problems are one problem

A booster coming home and a spaceplane going somewhere look like opposites and are not.
Both are asking the same question — *where does this arc meet the ground, and how do I move
that point* — and both have the same and only answer: thrust. So one file owns the loop and
neither script owns a copy of it.

The property it rests on is worth stating on its own, because everything else is a
consequence:

> **On a coasting ballistic arc the predicted impact point does not move.** It is an
> invariant of the orbit.

- Every metre the miss closes was bought by thrust. So the *measured* closing rate is a
  direct reading of the vehicle's range authority, in this geometry, at this instant, and
  nothing has to model the vehicle at all. `close` — metres of miss per second per unit
  throttle — is the number the taper is sized from, and it is an observation.
- Noise in the miss is noise in the predictor, not in the world, so it can be filtered hard
  without filtering away signal.
- The overshoot test is trivial. A miss that has grown past its own best can only mean the
  burn is making things worse.

`targeting.ks` therefore owns the survey, the drag correction, the miss, the closing rate,
the taper, and exactly two stop conditions — the two that are about the miss. It never
steers and never throttles. Everything a vehicle stops a burn for — dry tanks, a floor, a
deadline, the pilot — stays with the script, because only the script knows what it is
flying.

## What actually differs: the loft

> **A booster arrives already lofted. A vehicle leaving the ground has to buy its own arc.**

That is the whole difference, and it lands in one place: the attitude the burn is flown at.

`boostback.ks` burns flat, `heading(azimuth, 0)`. Its arc was paid for by the ascent it just
flew; what it needs now is to reverse horizontal velocity, and at those speeds the impact
point's sensitivity to horizontal Δv dominates its sensitivity to vertical. A vertical
component would buy range too — a longer fall is a longer flight — but at the price of a
higher, faster, hotter entry.

`ballistic_hop.ks` burns at the minimum-energy loft for the range it has left. It has no arc
yet and must buy one, and buying it at the wrong angle costs propellant the vehicle does not
have.

## The loft rule, and why it is two closed forms rather than a solve

For a ballistic arc between two points at the same radius on a non-rotating sphere,
separated by central angle Φ, the minimum-energy trajectory is:

    γ = 45° − Φ/4                          (flight-path angle at burnout)
    v² = (μ/r) · 2·sin(Φ/2) / (1 + sin(Φ/2))

These are the standard ballistic-missile results, and they are in the file rather than a
numerical solve because both of them **land on a textbook answer from each end**, which is
about as much verification as a formula can carry before a flight:

- **Φ → 0.** The loft gives 45°, the flat-ground answer. The speed gives
  v² = μΦ/r = g·R for ground range R — which is exactly the flat-earth maximum range at
  45°. Both degenerate correctly, together.
- **Φ = 180°, the antipode.** The loft gives 0° — launch horizontally. The speed gives
  v² = μ/r, circular speed. Those are the same trajectory said twice: the cheapest way to
  the far side of a sphere is a grazing circular orbit, and both formulas independently say
  so.
- **In between**, Φ = 90° on Kerbin (about 940 km) asks 22.5° of loft at 2208 m/s, a little
  under orbital speed. Long range wants a shallow arc, which is the right shape.

What they are *not* is an answer. No drag, no gravity losses, no rotating ground, and
burnout is assumed at the impact radius rather than 20 km above it. So the script uses them
for the only two things a lower bound and an aim are good for:

- **A refusal before anything is committed.** The least speed any arc to that range needs,
  against the most this vehicle could reach if every remaining metre per second went
  straight into its current velocity. Failing that comparison is a certainty; passing it
  proves nothing, since every neglected term is on the other side. That asymmetry is the
  point — it is a test that never refuses a feasible mission.
- **Where to point** while the loop closes the rest.

## The flight-path correction

The loft rule prescribes the *velocity* angle at burnout. Commanding the *thrust* at it
would only reach that angle asymptotically, and a boost of a minute is not asymptotic.

So the thrust leads the wanted direction by exactly as much as the current velocity lags it:
`pitch = loft + (loft − γ)`, with γ from `aero.ks`'s `angle_of_ascent()`. Nothing is chosen
in that gain. Leading by the error is the symmetric statement of the problem, it converges
for any positive gain, and one is the gain that spends the least cosine getting there. Φ is
re-read every cycle, so the loft steepens toward 45° on its own as the ground closes.

The band `pitch_min`/`pitch_max` is the only number in it, and it exists because the
correction — not the rule — is what could run away if γ starts far from the loft. `pitch`
pinned at a band edge in the log is that happening.

The azimuth is not part of this at all: it comes from the loop's miss vector, which points
wherever the impact is wrong, sideways included. Cross-range correction is not a second
loop.

## The boundary, and why it is drawn where it is

`ballistic_hop.ks` owns the boost and the coast. It does not own the climb and it does not
own the landing, and both refusals are deliberate:

- **WAIT takes no controls.** The vehicle climbs under a pilot's hand or `autopilot.ks`'s,
  and the script engages when ambient pressure falls below `p_boost`. Stated as a pressure
  rather than an altitude so it means the same thing on any body with air. It logs through
  the wait, so the climb is on the record even though nothing is flying it.
- **It lets go at `alt_handoff`**, above the air that matters, and says what to run next.
  The entry belongs to `reentry.ks` and the terminal to `autopilot.ks` with the target as
  its last waypoint. A script that tried to own those would be reimplementing both, badly.

The consequence is that the hop's product is *an arc aimed at a target*, and its witness is
the miss at handoff — not a touchdown distance. That is honest about what it did. It is also
why `miss_bar` is 2000 m against boostback's 250: between this arc and the ground sits an
entry flown by a script that is not this one, and any range control that entry exercises is
dispersion this bar cannot see. Aiming tighter than the handoff is precise buys nothing.

## What came from the aircraft scripts

The autopilot work turned out to carry three things this pair needed, and all three are
cases where the aeroplane side had already met a problem the rocket side was about to.

- **AFBW.** `afbw.ks` records that AFBW writes the control axes every tick and wins
  arbitration against kOS, throttle included, and that a script locking throttle while AFBW
  holds it reads its own commanded value back while the vessel runs something else. For
  these two scripts that is worse than flying the burn wrong: `close` is the measured
  closing rate *divided by the throttle believed to have produced it*, so a throttle that is
  a fiction makes the taper a fiction and the cut lands anywhere. Both scripts now release
  AFBW before taking the axes and restore it after. The hop releases it after WAIT rather
  than at the top, because the climb it just watched may have been flown on that stick.
- **One row, two ways.** `columns.ks` renders a list of values for the console while the
  same list goes to the CSV through `join`, so the two cannot disagree. Both scripts had
  been hand-writing a console line separate from the CSV row — exactly the drift that file
  exists to prevent. `subset()` moved out of `autopilot.ks`'s private scope into
  `columns.ks` alongside `columns()`, since a console narrower than the full row is the
  normal case rather than one script's problem.
- **The abort action group as the pilot's stop**, and the lesson that a run ended with it
  leaves it latched, so the next run stops on its first tick unless it is cleared. In
  `boostback.ks` an abort ends the burn and hands the controls back but does **not** stop
  the parachutes being asked for, because a pilot taking the controls still wants the
  canopies out and `CHUTESSAFE` cannot open one early. The hard deploy is tracked on a
  separate flag from the handback for the same reason: an abort high up must not consume the
  backstop that is due at 2500 m.

`aero.ks` supplied `angle_of_ascent()` for the flight-path correction and `ground_distance()`
for everything, and gained `compass_for()` — `compass_heading()` is now a call to it — and an
optional second point, which is what an impact-to-target distance needs.

## Pre-flight: the signatures

The log is `hop_<t>.csv`, one row per second from WAIT to handoff.

1. **`miss` is flat through WAIT and falls only once `thr` goes above zero.** The invariant,
   same as boostback's. If it drifts while the engine is cold, nothing downstream is sound.
2. **`gamma` converges on `loft`, with `pitch` between them and inside the band.** This is
   the correction working. `pitch` at a band edge is it running away; `gamma` flat while
   `pitch` leads hard is an airframe that cannot rotate its velocity vector as fast as the
   rule wants, which is a real answer and means the loft should be commanded open-loop
   instead.
3. **`close` settles within a factor of two across the full-throttle portion**, as
   boostback's must. If it does not, the taper is sized off noise.
4. **The cutoff row reads `miss inside the bar`** with the burn ending before `engine dry`.
   Ending on `engine dry` with a large `miss` is the feasibility check having passed a
   mission the losses then took away — which is the expected first failure, and the gap
   between `v_need` and what the flight actually spent is the number that sizes the margin
   the check is missing.
5. **`d_kep` and `d_tr` agree closely at cutoff and separate through COAST.** At boost
   altitude there is little air left to disagree about; the separation is the entry being
   predicted, and its size at handoff is what the entry script inherits.

The headline is the handoff row's `miss` against the WAIT rows' `d_kep`.

## Open

- **Every tunable in both files is unflown**, and `close` — the quantity the taper is built
  on — has never been measured on anything.
- **The loft rule's radius assumption is unbudgeted.** Burnout at 20 km on a 600 km body is
  a 3% radius error against a formula that assumes burnout at the impact radius. The loop
  absorbs it, but nobody has measured how much of the loop's work is spent doing so.
- **No mid-course correction, in either script.** Both commit before entry has measured any
  drag. This is the same open item in both registers and it has one fix — reserve a little
  and correct once the entry deceleration is readable.
- **The hop has no terminal.** It hands off an aimed arc, and the two scripts that should
  catch it have never been run in sequence with it. Whether `autopilot.ks` can be handed a
  vehicle at 25 km doing most of a kilometre a second is not known; most likely `reentry.ks`
  has to bleed it first, which is what the handoff message says to do.
- **`core/impact.ks` is a dead spike** — nothing imports it, and it ends in a bare call to a
  test function that loops forever, so nothing can. It is the natural home for a predictor
  that has to work on an orbit that is not the ship's live one (a maneuver node's patch, for
  instance), which is what a *planned* hop rather than a flown one would need. Left alone
  here because this pair needs neither.
- **Scope, standing:** a body with an atmosphere, a target reachable on one arc, and a
  vehicle that can hold an attitude under thrust. Nothing about the loop is Kerbin-specific;
  nothing about it has been tried anywhere else either.
