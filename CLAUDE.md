# Project: "Flight by Wire" — a KSP/kOS aerospace engineering book

This repository hosts:

1. **Reference material to mine** — `reference/core/`, `reference/landing_v2/`,
   `reference/wip/`, `reference/script/` are main_v2's draft library. Frozen: read them,
   don't edit them.
2. **Working scripts** — `reference/original/` holds Schuyler's own kOS scripts (formerly
   root `*.ks` files). These are **not frozen**. They are spikes he flies in the game, and
   `powered_landing.ks` is under active development. Edit them when the work calls for it.
3. **The book** — reader-facing chapters in `wiki/`.
4. **Tools** — `util/kos_bridge.py` exposes kOS's telnet server (port 5410) as
   `/tmp/kos_cmd` and `/tmp/kos_out`. This directory is the kOS archive volume, so a script
   written here runs in the live game: `python3 util/kos_bridge.py --attach 1 &`, then
   `echo 'run foo.' > /tmp/kos_cmd`. Attaching touches a live CPU — ask first. Get data out
   by having the script `log` to a file here and reading it, not by scraping the terminal.

Design notes live in `notes/`. Nothing in this repo runs outside KSP and there is no test
framework; verification is a flight, or the bridge.

**Before doing any work on the book, read `BOOK-PLAN.md`.** It is the authoring design
document: audience, pedagogical principles, the decided structure and the reasons behind it,
source-material map, style notes, working agreements, and a Status section tracking where
work left off. Update its Status section before ending a session.

Development branch: `claude/kerbal-aerospace-tutorial-q6gs9o`.
