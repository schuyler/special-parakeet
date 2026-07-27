# Project: "Flight by Wire" — a KSP/kOS aerospace engineering book

This repository hosts:

1. **Reference material to mine** — `reference/landing_v2/`, `reference/wip/`,
   `reference/script/` are main_v2's draft library. Frozen: read them, don't edit them.
   `reference/core/` is different: it's the shared kOS library the descent scripts
   actually run, and it is **not frozen** — edit it when the work calls for it.
2. **Working scripts** — `reference/original/` holds Schuyler's own kOS scripts (formerly
   root `*.ks` files). These are **not frozen**. They are spikes he flies in the game, and
   the descent pair is under active development: `plan_doi.ks` places the DOI node,
   `powered_descent.ks` flies the braking arc and terminal down from it. Edit them when
   the work calls for it.
3. **The book** — reader-facing chapters in `wiki/`.
4. **Tools** — `util/kos_bridge.py` exposes kOS's telnet server (port 5410) as
   `/tmp/kos_cmd` and `/tmp/kos_out`. This directory is the kOS archive volume, so a script
   written here runs in the live game: `python3 util/kos_bridge.py --attach 1 &`, then
   `echo 'run foo.' > /tmp/kos_cmd`. Attaching touches a live CPU — ask first. Get data out
   by having the script `log` to a file here and reading it, not by scraping the terminal.

Design notes live in `notes/`; verified kOS API facts live in `notes/kos-facts.md`, so check
there before re-deriving one. Nothing in this repo runs outside KSP and there is no test
framework; verification is a flight, or the bridge.

## What the flight code optimises for

Δv, not timing or path shape. The landing accuracy bar is 10 m; precision beyond it is
deliberately deferred — a Kerbal can walk. Terrain clearance is a design input, never
something to optimise away.

## Working agreements

- **No claim about flight behaviour without telemetry that supports it**, and no fix
  implemented until its hypothesis has been checked against a log. Reasoning from a
  plausible model and skipping the log has produced fixes for problems that were not there.
- **Predictions are testable signatures** — which column, which value — never promised
  outcomes.
- **One instrumented change per flight.**
- **The CSV is the witness.** `flight_log.csv`, one row per second, planning numbers as `#`
  metadata, so a flight is auditable afterward instead of remembered.
- **Records describe the current state, not how it got there.** Comments say what the code
  does; design notes register the shipped design. Git holds the history — don't narrate
  supersession in prose, and don't keep a script or a note alive only because another
  document cites it.

## Code conventions

- `pitch` is degrees **above** the horizon (`aero.ks:42`; kOS's `heading()` agrees).
- `d_`/`d` means **difference**, never derivative. `dt` is family.
- State is `speed`, not `v`, because `dv` means delta-v everywhere.
- Pass **orbit objects**, not scalars.
- Append `_` to any local or parameter colliding with a kOS builtin (`v`, `r`, …).
- Comments state what a quantity *is*, for a reader one semester into calculus.
- Every tunable needs an argument that is free of the craft and the body; flights falsify
  arguments, they never fit values.

**Before doing any work on the book, read `BOOK-PLAN.md`.** It is the authoring design
document: audience, pedagogical principles, the decided structure and the reasons behind it,
source-material map, style notes, working agreements, and a Status section tracking where
work left off. Update its Status section before ending a session.

Development branch: `claude/kerbal-aerospace-tutorial-q6gs9o`.
