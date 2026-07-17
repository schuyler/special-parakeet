# Targeting redesign: session checkpoint, 2026-07-16

*Handoff for the next session. Read `powered-landing-flight-tests.md` first (campaign
history, flight 6/7 results, agreed design in its §5a). This file carries what that
note doesn't: the exact proposed code blocks awaiting Schuyler's review, the derivation
behind them, and the loose ends. Nothing below has been applied to
`reference/original/powered_landing.ks` — the blocks are proposals.*

## Where the session ended

- Debugging campaign CLOSED: kepler `body_rotation` epoch bug confirmed against kOS
  source, fixed, committed (`af4b25b`); `powered_landing.ks` + campaign notes committed
  (`068a3c3`). Flight 6: 1 m miss, ~250 m/s. Flight 7 (fully instrumented baseline,
  25 km circular equatorial, h_pdi 3000): **0 m miss, 244.1 m/s** — DOI+coast 11.9,
  BRAKE 100.2, APPROACH 107.6, TERMINAL 24.4. Archived:
  `flight_log_flight7_baseline_25km.csv`.
- **Uncommitted in the working tree:** the telemetry upgrades to `powered_landing.ks`
  (CSV `#` metadata header, DOI plan lines, mass/dv_rem columns, TERMINAL logging —
  flown in flight 7) plus the pitch/cmd_pitch columns (added after flight 7, NOT yet
  flown). Also untracked: flight logs, `kos_bridge.py`, `ssto2_incl.ks`.
- Targeting redesign designed and agreed (campaign note §5a); code drafted as four
  blocks, below, **pending review**. Schuyler asked for the free-fall integrals to be
  explained slowly and the explanation folded into the script comments — done in the
  Block 2 revision below; he had not yet approved anything when the session ended.

## The design in one paragraph

BRAKE is designed horizontal-first: design throttle pins the deceleration a_h, which
pins duration T = (v_pe − v_hg)/a_h, which pins the lead (midpoint rule — exact,
because constant decel means zero horizontal jerk). The vertical axis then *rides
gravity*: with v_h(t) linear, the zero-thrust vertical profile is closed-form, and the
gate descent rate (vv_gate) and PDI altitude (h_pdi = gate_alt + drop, floored by
terrain + clearance) are read off it instead of chosen. BRAKE's closure scalar
a_arrival_brake is then derived so the flight-side quadratic reproduces T at PDI.
Feasibility is checked exactly, pre-flight, at the endpoints of both legs (per-axis
commanded accel is linear in time, so |demand| maxes at an endpoint). `clearance`
replaces `h_pdi` as the third script parameter — **call-signature change**:
`powered_landing(0, 0, 2000)` now means clearance. hg_offset / hg_height /
hg_ground_speed / low gate / APPROACH's a_arrival (0.5) stay fixed this round.

## The derivation (for the comments and, later, a notes/ write-up)

1. **Felt vertical acceleration** of a ship at ground speed v over a body of radius r:
   `a_felt = g − v²/r` (down-positive). Gravity first pays the centripetal cost of
   following the surface's curve; only the remainder descends the ship. Anchors:
   v = 0 → full g (hover). v = sqrt(g·r) → zero (circular orbit: free fall that never
   descends). v > circular — which periapsis speed always is — → net lift (past
   periapsis, an ellipse climbs). So at BRAKE entry gravity builds no descent; it
   reappears as the burn sheds speed.
2. **v_h(t) = v_pe − a_h·t is known**, so a_felt(t) is a known (quadratic-in-t)
   function; integrate once for the gate descent rate, twice for altitude lost.
3. **First integral.** v_h falls linearly, so the burn spends equal time in each slice
   of speed: time-averages are speed-averages, and mean(v²) over [v_hg, v_pe] is the
   mean-square `(v_pe² + v_pe·v_hg + v_hg²)/3`. Then
   `vv_gate = T · [g − (v_pe² + v_pe·v_hg + v_hg²)/(3r)]`
   (identical to the difference-of-cubes form `g·T − (v_pe³ − v_hg³)/(3·r·a_h)`, but
   reads as physics: average felt acceleration × duration).
4. **Second integral.**
   `drop = g·T²/2 − v_pe³·T/(3·r·a_h) + (v_pe⁴ − v_hg⁴)/(12·r·a_h²)`
   — the /3 becomes /12 and a fourth power because the same change of variables is
   integrated twice. Short hard brakes drop metres (and can rise mid-leg); the TWR-2
   design craft drops ~3.4 km — which is exactly why h_pdi must be an output.
5. **Why the law can fly it:** the law flies linear-per-axis acceleration; a_felt(t)
   is quadratic; the flown cubic matches the endpoints and the engine pays only the
   residual, bounded by `(v_pe − v_hg)²/(4r)` ≈ 0.05 m/s² here. Also stated as an
   approximation: g is evaluated at h_pdi and held constant over the leg.
6. **Consistency scalar:** with D = (h_pdi − gate_alt) + sag (law-frame vertical
   distance), `a_arrival_brake = (6D − 4·vv_gate·T)/T²` is the arrival acceleration
   that makes `solve_t_go` return exactly T at PDI. If the clearance floor raised
   h_pdi above the free-fall drop, D grows and this scalar absorbs it.

## Worked numbers (cross-checks for review)

- Flight-7 craft (TWR 34, f = 0.75): a_h ≈ 11.24, T ≈ 9.8 s, brake_distance ≈ 1.13 km,
  vv_gate ≈ 2.1 m/s, drop ≈ 7 m, h_pdi ≈ terrain + 2007 (clearance floor 2000 not
  binding), a_arrival_brake ≈ +0.2, PDI endpoint demand ≈ 11.24 ≈ f·a_max. APPROACH
  ground solve t_apch ≈ 74 s.
- Design craft (TWR 2, f = 0.75): a_h ≈ 0.51, T ≈ 216 s, brake_distance ≈ 24.9 km,
  sag ≈ 5.0 km, vv_gate ≈ 47 m/s, drop ≈ 3.37 km, h_pdi ≈ terrain + 5.4 km.
- Predicted signatures for the first redesign flight (f = 0.75, TWR-34 craft): BRAKE
  ≈ 10 s at ~75% throttle (vs flight 7's 44 s at 21%); cmd_pitch near level through
  BRAKE; APPROACH entry v_vert ≈ −2; APPROACH ledger < flight 7's 107.6 m/s. Also
  test with brake_throttle turned DOWN (e.g. 0.1) to stretch T into the design-craft
  regime on the same vehicle.

## Proposed code blocks (NOT applied; review pending)

### Block 1 — parameters and constants

```
// clearance: the terrain-clearance floor, m — minimum height of the descent
// path above the surveyed terrain. This replaces h_pdi as the caller's knob:
// PDI altitude is an OUTPUT of the descent design, and the only number a
// mission planner legitimately owns is how much landscape risk to accept.
// The floor is deliberate: a low, late burn stakes the crew on the terrain
// model being exact.
parameter clearance is 2000.
parameter lead_deg is 0.          // PDI point this far up-range of the target; 0 = compute.
parameter brake_throttle is 0.75. // Design throttle for braking: sets how hard, and
                                  // therefore how briefly and how low, the ship brakes.
                                  // The reserve (1 - brake_throttle) is the in-flight
                                  // authority margin.

// === DESCENT DESIGN CONSTANTS ===
local hg_offset is 2000.        // aim point's up-range offset from the site, m
local hg_height is 2000.        // aim point's height above the site terrain, m
local hg_ground_speed is 60.    // arrival speed toward the site, m/s
local a_arrival_apch is 0.5.    // APPROACH's closure scalar: arrive gently slowing.
```

TWR pre-check stays but evaluates at surface gravity g0 (h_pdi unknown at that point;
conservative in the right direction). `hg_descent_rate` and the shared `a_arrival`
constant are deleted (derived / split per-gate below).

### Block 2 — the descent design loop (revised with derivation comments)

```
// === DESCENT DESIGN ===
// Designed horizontal-first: the engine's job is to shed ground speed, so
// the design throttle pins the deceleration, the deceleration pins the
// duration, the duration pins the lead. The vertical axis then RIDES
// GRAVITY: with the horizontal profile fixed, the zero-thrust vertical is
// known in closed form, and the gate's descent rate and PDI's altitude are
// read off it. Thrust spent building descent rate must be spent again
// removing it; the cheapest vertical profile is the one gravity flies.
//
// Two approximations, both stated: g is evaluated at h_pdi and treated as
// constant over the leg (the altitude change is small against r), and the
// law will fly the closest LINEAR acceleration profile through these
// endpoints while the free-fall acceleration is quadratic in time — the
// engine pays the residual, bounded by (v_pe - v_hg)^2/(4r), centimeters
// per second squared here.
//
// h_pdi and v_pe depend on each other (weakly): iterate, a correction of a
// correction, three passes.
local h_pdi is tgt:terrainheight + clearance.   // seed
local r_pe is 0. local g_pdi is 0. local v_pe is 0. local a_h is 0.
local brake_duration is 0. local brake_distance is 0. local sag is 0.
local vv_gate is 0. local drop is 0.
local pass is 0.
until pass >= 3 {
  set r_pe to body:radius + h_pdi.
  set g_pdi to body:mu / r_pe ^ 2.
  local sma_park is (ship:orbit:semimajoraxis + r_pe) / 2.
  // Surface-relative periapsis speed: ground-track speed is what both the
  // lead and the burn have to cover.
  set v_pe to sqrt(body:mu * (2 / r_pe - 1 / sma_park))
            - 2 * constant:pi * r_pe / body:rotationperiod.
  // Horizontal decel budget at design throttle, reserving authority to
  // hold the vertical axis.
  set a_h to sqrt(max(0.001, (brake_throttle * a_max) ^ 2 - g_pdi ^ 2)).
  set brake_duration to (v_pe - hg_ground_speed) / a_h.
  // Constant decel: the midpoint rule is exact (zero horizontal jerk).
  set brake_distance to (v_pe + hg_ground_speed) / 2 * brake_duration.
  // An aim point d down-range sits d^2/2r below local horizontal; the
  // law's vertical axis sees that sag as descent to be flown.
  set sag to brake_distance ^ 2 / (2 * r_pe).
  // Free-fall vertical over the burn — the design's central move. The felt
  // vertical acceleration of a ship moving at ground speed v over a body of
  // radius r is not g but
  //     a_felt = g - v^2/r,
  // because gravity must first supply the centripetal acceleration that
  // follows the surface's curve; only the remainder descends the ship.
  // Anchors: v = 0 gives full g (hovering); v = sqrt(g*r) gives zero (a
  // circular orbit: free fall that never descends); v above circular —
  // which periapsis speed always is — gives net LIFT (past periapsis, an
  // ellipse climbs). So at BRAKE entry gravity builds no descent at all,
  // and it reappears only as the burn sheds speed.
  //
  // Because v_h falls LINEARLY, the burn spends equal time in each slice
  // of speed, so time-averages are speed-averages. The average of v^2 over
  // the burn is the mean-square over [v_hg, v_pe]:
  //     mean(v^2) = (v_pe^2 + v_pe*v_hg + v_hg^2)/3
  // and the gate descent rate is average felt acceleration x duration:
  set vv_gate to brake_duration
      * (g_pdi - (v_pe ^ 2 + v_pe * hg_ground_speed + hg_ground_speed ^ 2)
                 / (3 * r_pe)).
  // Integrating once more gives the altitude free fall loses over the
  // burn: g*T^2/2 from gravity, minus the centrifugal accumulation (the
  // same change of variables, integrated twice — hence the /3 becomes a
  // /12 and a fourth power). Short hard brakes lose almost nothing and can
  // even rise mid-leg; long brakes lose kilometers, which is exactly why
  // PDI's altitude must be derived: PDI sits 'drop' above the gate so that
  // free fall arrives AT the gate.
  set drop to g_pdi * brake_duration ^ 2 / 2
      - v_pe ^ 3 * brake_duration / (3 * r_pe * a_h)
      + (v_pe ^ 4 - hg_ground_speed ^ 4) / (12 * r_pe * a_h ^ 2).
  // PDI is where the free-fall drop meets the gate, floored by the
  // clearance the caller staked out.
  set h_pdi to max(tgt:terrainheight + clearance,
                   tgt:terrainheight + hg_height + drop).
  set pass to pass + 1.
}
// A short brake can arrive still ascending: past periapsis, speed exceeds
// circular and the free path rises before it falls. The gate must hand
// APPROACH a descent; 1 m/s is "descending" without buying thrust-down.
set vv_gate to max(1, vv_gate).
```

### Block 3 — consistency scalar, exact feasibility, lead

```
// BRAKE's closure scalar, derived instead of chosen: the arrival
// acceleration that makes the flight-side quadratic reproduce this
// design's duration when solved at PDI. D is the law-frame vertical
// distance — true drop plus sag. (If the clearance floor raised h_pdi
// above the free-fall drop, D grows and this value absorbs it.)
local D is (h_pdi - (tgt:terrainheight + hg_height)) + sag.
local a_arrival_brake is (6 * D - 4 * vv_gate * brake_duration)
                         / brake_duration ^ 2.

// Engine feasibility, exact: per axis the commanded acceleration is
// linear in time, so the thrust demand over a leg maxes at an endpoint.
// BRAKE leg — both ends against full thrust (the design nominally sits at
// brake_throttle; failing here means the margin is gone on the ground):
local acmd_v0 is -6 * D / brake_duration ^ 2 + 2 * vv_gate / brake_duration.
local demand0 is sqrt(a_h ^ 2 + (acmd_v0 + g_pdi) ^ 2).
local demand1 is sqrt(a_h ^ 2 + (a_arrival_brake + g_pdi) ^ 2).
if max(demand0, demand1) > a_max {
  print "ABORT: braking demands " + round(max(demand0, demand1), 2)
      + " m/s^2; the engine has " + round(a_max, 2)
      + ". Lower brake_throttle or raise clearance.".
  wait until false.
}

// APPROACH leg — this check is the clamp on the derived gate state: solve
// the approach quadratic from the designed handoff and test ITS endpoint
// demands. A gate too hot for the approach geometry fails here, on the
// ground, not at 2 km.
local Da is hg_height - 150
    + hg_offset ^ 2 / (2 * (body:radius + tgt:terrainheight)).
local qb_a is 2 * vv_gate + 20.            // -(2*vv + 4*v_tgt_v), up-positive
local disc_a is qb_a ^ 2 + 24 * a_arrival_apch * Da.
local t_apch is (qb_a + sqrt(disc_a)) / (2 * a_arrival_apch).
local ah_a0 is 6 * hg_offset / t_apch ^ 2 - 4 * hg_ground_speed / t_apch.
local av_a0 is -6 * Da / t_apch ^ 2 + (4 * vv_gate + 10) / t_apch.
local demand_a0 is sqrt(ah_a0 ^ 2 + (av_a0 + g_pdi) ^ 2).
local demand_a1 is sqrt((2 * hg_ground_speed / t_apch) ^ 2
                        + (a_arrival_apch + g_pdi) ^ 2).
if max(demand_a0, demand_a1) > a_max {
  print "ABORT: approach demands " + round(max(demand_a0, demand_a1), 2)
      + " m/s^2 from the designed gate; the engine has " + round(a_max, 2)
      + ". Lower brake_throttle (a slower brake arrives shallower).".
  wait until false.
}

// Lead: ground covered during the burn plus the gate's own offset. Lead,
// speed, and duration must describe the SAME burn: hand the law a
// boundary-value problem violating distance = speed x time and it will
// still solve it — by dive or reversal, whichever contortion fits.
if lead_deg <= 0 {
  set lead_deg to (brake_distance + hg_offset) / r_pe * constant:radtodeg.
}
```

Plus: console print and the CSV `#` metadata updated to log the derived h_pdi,
vv_gate, drop, a_arrival_brake, t_apch, and the four endpoint demands;
`min_brake_duration` disappears from code, print, and metadata.

### Block 4 — gate constructors (described, not yet drafted in full)

- `high_gate`: `"v_vert", -vv_gate`; closure calls `solve_t_go(..., a_arrival_brake)`.
  Rewrite its comment: 2000/2000/60 provenance stays; the "60:30 = ~27° arrival path"
  paragraph is obsolete (arrival steepness is now whatever gravity built over the
  brake).
- `low_gate`: closure uses `a_arrival_apch`. Otherwise unchanged.
- No other flight-side changes. fly_gate, closures, t_handoff, floors, TERMINAL, DOI
  all stay as flown in flight 7.

## Open items for the next session

1. Complete Schuyler's review of the blocks (he was mid-digestion of the Block 2
   derivation; the mean-square form of vv_gate replacing difference-of-cubes was
   proposed but not yet approved).
2. Draft Block 4 in full; apply blocks on explicit approval only, then fly.
3. Commit the telemetry upgrades (flown in flight 7 + pitch columns) — currently
   uncommitted; decide whether they ride with the redesign commit or precede it.
4. Write the free-fall derivation up properly in notes/ (sibling to
   klumpp-guidance-derivation.md) once the code lands.
5. **Pinned by Schuyler, undiscussed: throttle deadzone.**
6. Deferred: APPROACH-side optimization (hg_height 2000 is where the remaining ~45 m/s
   of hang lives — ledger will say what it's worth after this flight); re-solve cadence
   scaled to t_go; terminal position feedback.

## Process (unchanged, hard-won)

No claim about flight behavior without telemetry; predictions only as testable
signatures. One instrumented change per flight (the redesign counts as one planning-
subsystem change). Blocks applied only on explicit approval. Comments carry timeless
principles, not war stories.
