# Handoff, 2026-07-17

State and next actions. The design is `capability-driven-descent.md`; the work order is its
"Order of work" section. This file is state only — when it stops being true, delete it.

## Use the bridge

`python3 util/kos_bridge.py --log --attach 1 &`, then `echo 'run foo.' > /tmp/kos_cmd`. This
directory is the kOS archive volume, so a script written here runs in the live game. Read
results by having the script `log` to a file here; `/tmp/kos_out` is a repainting terminal and
is for watching, not parsing. See CLAUDE.md.

Two things learned the hard way:

- **kOS only executes while KSP is unpaused and focused.** A paused game still answers telnet
  and still echoes what you type — it just never runs it. If commands vanish silently, that is
  the first thing to check, not the script.
- **A quickload detaches the terminal back to the CPU menu**, and `--attach` only fires on
  connect, so it does not recover. Send `1` again by hand. Worth fixing in the bridge.

## What is committed

- `util/kos_bridge.py` — works. Needed telnet option negotiation, which it never had.
- `notes/capability-driven-descent.md` — the design and every engineering choice behind it.
- `CLAUDE.md` — corrected; it had described `reference/` as frozen and named two directories
  that do not exist.

`b64921a` still holds blocks 1–3 inside `powered_landing.ks`, unwired. **Block 1 has never
compiled** — `local r` clobbers kOS's builtin `R()`. Block 3's forward arc loop is superseded
by the chord argument, but its backward coast walk is the surviving rule for the coast and
should be mined for piece 3, not deleted. `powered_landing.ks` itself still flies; it is the
fallback.

## Piece 1: `reference/original/powered_descent.ks`

New, uncommitted at time of writing, **never flown**. It begins on a descent ellipse someone
else planned: it warps to PDI, integrates the gravity turn from live state, chops it into legs,
flies them, hands off to terminal. Standalone apart from `../core/optimize` for `bisect`.

**Tested through the bridge, on Probe I at Minmus, no burn** — seeded with flight 7's geometry
(`h_pdi` 3000, `speed_pdi` 170.6, terrain 0):

```
solve: 44.5 s   f = 0.0609
arc: dur 245 s  X 17174 m  end_h 17  end_speed 4.98  closed True
gamma = 9.85 deg
legs: 6   aim_alt 2126 / 1353 / 796 / 398 / 122 / 17
```

Six legs from a 15° turn budget, exactly as the sagitta argument predicts, and they bunch at
the pitchover: LEG1 alone is 124 s of the 245 s arc, the last five crowd into the final 80.

**What this says about the campaign.** A PDI of 3000 m forces a TWR-28 craft down to **6%
throttle for four minutes across 17 km**, on a craft that could stop in twelve seconds. γ comes
out at 9.85° over ground that demands roughly zero. So piece 1 alone **cannot beat 244 m/s** —
handed a high PDI it will correctly fly the same expensive arc, because that is the only arc
that fits. The throttle was never the disease. The Δv is in pieces 2 and 3, which lower PDI
until the solve returns a high throttle.

**Untested:** everything from `fly_leg` onward. No leg has been flown, and `tessellate` has
never handed a lexicon to the guidance law.

**Known-pending:** `f_epsilon` was moved to 0.0001 after the last run and has **not been run at
that value**. At 0.005 the arc ended at 17 m against a `landing_height` of 50, because the
handoff height moves ~120 m per 0.001 of throttle. Expect `end_h` ≈ 50 ± 12 and a solve nearer
70 s. **Verify this first** — it is one run and everything downstream assumes it.

## Next

1. **Re-run the solve at `f_epsilon` 0.0001.** Confirm `end_h` ≈ 50. One bridge run.
2. **Fly piece 1.** It needs a descent ellipse and does not care who made it — place a node by
   hand, or let `powered_landing.ks` do its DOI and start this instead. Then quicksave
   mid-coast: that save makes the descent re-runnable with no launch and no burn, which is the
   iteration loop this campaign has never had. Read `pitch` vs `cmd_pitch` first — that pair is
   the chord-versus-arc error the 15° budget is chosen against, and it has never flown.
3. **Then piece 2**, the simple planner: takes γ from the human, places the node. It prices γ
   against Δv for the first time, which piece 3 cannot be designed honestly without.

## Decisions still open in piece 1

- **The endpoint feasibility check is proposed and unbuilt.** The saturation cross-guard is
  gone — its 5 s window was arbitrary and did not scale (40% of a TWR-34 arc, 2.5% of a TWR-2
  one). The principled replacement: acceleration is linear in time along a leg, so its demand
  peaks at an endpoint; check both against `f_max · a_max` when the leg is built, exactly, and
  abort before flying it rather than watching a clock. That also gives `f_max` one meaning in
  two places — capping the throttle solve and guarding leg feasibility.
- **The `10` in `fly_leg`'s re-solve cadence is the same crime as the deleted `5`.** A wall-clock
  constant in a script whose only natural timescale is `t_go`.
- **`CONFIG:IPU` is the real lever on speed.** kOS ships at 200 instructions/tick and allows up
  to 2000. Everything here is throttled by that, not by the algorithm. A 70 s solve is fine
  before a coast and painful in a test loop.
- Open items 1–10 in the design note, unchanged.
