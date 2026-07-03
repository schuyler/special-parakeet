# Book design: "Flight by Wire"

This is the **authoring document** for the tutorial book being written in `wiki/`. It records
what we're writing, for whom, in what order, and — most importantly — *why* the structure is
the way it is, so that work can continue across many sessions without re-litigating decisions.
The reader never sees this file. Keep it current: update the **Status** section at the end of
every working session.

## What this is

A multi-chapter tutorial on aerospace engineering and orbital mechanics, using Kerbal Space
Program + the kOS mod as a hands-on laboratory. Markdown chapters in wiki format (`wiki/`,
flat pages, relative links) so it can later be transformed into an ebook if that seems worth
the effort.

## Audience

KSP players who want to learn the math and science. They are game-literate (know the VAB/SPH,
have flown to the Mun by instinct) but are not assumed to know physics beyond algebra, or any
programming. The emotional register: "you are already a test pilot; this book makes you an
engineer."

## The three-part chapter contract

Every chapter does all three, no exceptions:

1. **Teaches the science** — real derivations, gently, with Kerbin numbers plugged in.
2. **Builds the library** — adds tested, reusable routines to `lib/`. Nothing is a throwaway
   example; the code accretes across the book. The library IS the book's spine.
3. **Flies a mission** — chapters end with flying something, and later missions are
   compositions of the library built so far.

## Pedagogical principles (decided, with reasons)

- **The destination is SSTO.** The book's central problem is single-stage-to-orbit spaceplane
  engineering — chosen because it is the hardest problem in the game, which makes every topic
  load-bearing rather than survey material. The rocket equation is the villain; air-breathing
  engines are the loophole; everything else is in service of the assault.
- **Rockets are lab instruments, not the subject.** Parts I–III fly simple uncrewed rockets
  because they are the "frictionless inclined plane" of the book: a minimal apparatus that
  isolates one phenomenon at a time. Two specific reasons this beats planes-from-chapter-1:
  (a) a rocket ascent is a legible demonstration of exactly the concepts being taught, where a
  spaceplane ascent entangles five subsystems the reader hasn't met; (b) some experiments are
  *unmeasurable* on a jet — notably measuring Tsiolkovsky's equation from the reader's own
  telemetry, which requires fixed Isp and no lift/intake-air confounds. The reader's identity
  is spaceplane engineer from page one; rockets are how we take measurements.
- **Measure, don't assume.** The telemetry library arrives in chapter 2 and everything after
  is scored against flight data: the rocket equation is measured, the drag polar is measured,
  the engine envelope is mapped empirically, and SSTO design iteration is judged by logged
  numbers. This is the book's method and its deepest habit.
- **Concepts enter when a mission demands them.** Airless landing enters when the surface base
  demands it; patched conics enter when the expedition demands it. Nothing is taught "because
  it's next in the textbook."
- **Precision is the real landing curriculum.** Powered descent is harder than any orbital
  transfer (no in-game scaffold like maneuver nodes; no closed-form math — numerical methods
  and feedback control; failures are terminal and fast). *Precision* powered descent is harder
  still, and it is what motivates surface bases. Difficulty ladder: Minmus landing (0.49 m/s²,
  gentlest classroom) → landing within sight of the first flag → Mun (real gravity) → optional
  Vall encore during the expedition.
- **The bases are load-bearing.** The Minmus/Mun outposts of Part IV are the fuel
  infrastructure (ISRU) that the Laythe expedition refuels at in Part VIII. Not a detour.
- **Uncrewed first.** The computer is the crew — that's the premise. Kerbals ride along once
  the software has earned their trust (roughly how NASA did it).
- **Full engineering scope.** Vehicle design (SPH/VAB) is in scope throughout: wing sizing,
  engine count per ton, CoM/CoL, intake area, thermal margins — with the library used to
  measure whether a design change worked.
- **Failure is data.** Honest iteration, wrong turns left in where instructive (the existing
  `ssto.ks`→`ssto4.ks` progression is the model for chapter 18).
- **"In the real world" sidebars** connect each technique to actual spaceflight history
  (Apollo AGC / Margaret Hamilton in ch. 1; Falcon boostback in Part IV; etc.).

## The arc (8 parts, ~25 chapters)

The reader-facing table of contents with per-chapter science/code columns lives in
`wiki/Home.md` — keep the two in sync. Authoring intent per part:

- **Part I — Ground School (1–3):** kOS basics; telemetry library; rocket equation derived
  *and measured*. Ch. 3 plants the seed: "here is why a single-stage spaceplane shouldn't be
  possible." Chapter 1 is drafted.
- **Part II — Getting to Orbit (4–6):** ascent problem, what an orbit is (vis-viva from energy
  conservation), maneuver nodes. Mission: probe to circular orbit, hands off.
- **Part III — Orbital Operations (7–10):** transfers, anomalies/timing (Kepler's equation),
  rendezvous, docking. Mission: fuel depot in LKO with automated fuel transfer.
- **Part IV — The Outpost (11–13):** powered descent (Minmus), precision landing (second
  landing within sight of the first flag), base + ISRU + Mun difficulty step.
- **Part V — Flight School (14–16):** lift/drag measured in flight, engine envelope mapped
  empirically, PID control and a cruise autopilot. Mission: autonomous cross-Kerbin flight.
- **Part VI — The SSTO Problem (17–19):** ascent corridor as energy management; telemetry-
  driven design iteration (the honest chapter, failures left in); circularizing on fumes,
  payload fraction as the only score. Mission: runway to orbit, single stage, with payload.
- **Part VII — The Logistics Company (20–22):** reentry energy management targeting KSC;
  approach/autoland; graduation flight = repeatable hands-off station resupply.
- **Part VIII — The Expedition (23–25):** patched conics/windows; Minmus springboard + Jool
  aerocapture; Laythe landing and return; optional Vall encore.
- **Appendices:** A. body constants; B. library reference by chapter; C. further reading
  (Braeunig; Bate/Mueller/White).

Chapter count may compress during drafting (e.g., 7+8 could merge); that's fine, but keep the
part structure and mission milestones.

## Repository layout

- `wiki/` — reader-facing chapters. Flat pages, `Chapter-NN-Title.md` naming, relative links,
  `Home.md` as front page/TOC. Wiki-portable, ebook-able later.
- `lib/` — the canonical library as it exists at the current frontier of the book. Routines
  are introduced by chapter; appendix B tracks which chapter introduced what.
- `missions/chNN/` — per-chapter mission scripts (the scripts the reader writes/runs in that
  chapter, at that chapter's level of sophistication).
- Root `*.ks` files — **Schuyler's original working scripts.** Source material, kept for
  reference ("here for the nonce"); the book code is a fresh, pedagogically-ordered rebuild.
  Do not edit these; mine them.

### Source-material map (root scripts → chapters)

| Existing script(s) | Feeds chapter(s) | Notes |
|---|---|---|
| `telemetry.ks`, `resources.ks` | 2 | logging approach |
| `common.ks` (`burn_duration`, rocket eq.) | 3, 6 | has a "confirm this math" TBD — becomes an exercise |
| `launch.ks`, `launch_vacuum.ks`, `launch_correction.ks` | 4 | ascent profiles |
| `common.ks` (`orbital_speed` vis-viva), `orbital.ks` | 5, 8 | |
| `common.ks` (`execute_node`, `node_from_velocity`), `circularize.ks` | 6 | node execution incl. coordinate-frame rotation |
| `set_periapsis.ks`, `move_periapsis.ks` | 7 | |
| `kepler.ks`, `orbital.ks` (anomalies) | 8 | `kepler.ks` header ("don't need it, see orbit.ks") is itself a good pedagogical beat: derive by hand, then learn what the API gives you |
| `intercept.ks`, `next.ks`, `wait_for_launch.ks` | 9 | |
| `dock.ks`, `dock2.ks`, `fuelxfer.ks` | 10 | |
| `deorbit*.ks`, `landing.ks`, `land_at_periapsis.ks`, `common.ks` (`time_to_surface`, `landing_time` Newton iteration) | 11–12 | the terrain-height Newton iteration is exactly the "numerical methods where closed form runs out" lesson |
| `boostback.ks` | 12 (divert/targeting), sidebar Falcon | |
| `aero.ks` | 14 | lift/drag computation |
| `ssto.ks`…`ssto4.ks` | 17–18 | the iteration story of ch. 18 |
| `reentry.ks`, `aerobrake.ks` | 20 | |

## Style

- Voice target: **Schuyler's `schuyler-docs` skill** — direct, conversational, honest about
  limitations, dry humor that never crowds out technical content. STATUS: the skill is enabled
  on claude.ai but NOT loadable from remote sessions. TODO: add it to this repo under
  `.claude/skills/schuyler-docs/` (or paste contents in-session), then restyle `wiki/Home.md`
  and Chapter 1, which were drafted before the skill was known.
- kOS code shown with trailing periods; terminal-typed commands shown as typed.
- Worked examples use Kerbin numbers (radius 600 km, μ = 3.5316×10¹² m³/s², g = 9.81 m/s²);
  compare to real-world values where it illuminates.
- Math level: algebra + gentle derivations; every derivation lands in a number the reader can
  check at the terminal.
- Chapters end with **exercises** (some load-bearing for later chapters) and a **What's next**
  hook.

## Working agreements (for future sessions)

- Branch: `claude/kerbal-aerospace-tutorial-q6gs9o`. Commit and push at natural checkpoints.
- Workflow per chapter: draft → Schuyler reviews → revise. Don't mass-produce chapters ahead
  of review; voice calibration is still in progress.
- When a chapter introduces library code, add it to `lib/` and the mission scripts to
  `missions/chNN/` in the same commit as the chapter text.
- Keep `wiki/Home.md`'s TOC consistent with this document; if structure changes, update both
  and record the *reason* here.
- Update the Status section below before ending a session.

## Status

*Last updated: 2026-07-03*

- **Done:** outline settled through the decisions recorded above; `wiki/Home.md` (front page +
  full TOC); `wiki/Chapter-01-The-Flight-Computer.md` drafted (uncrewed OKTO trainer; gravity
  experiment; `liftoff.ks`).
- **Pending review:** Chapter 1 voice — awaiting `schuyler-docs` skill contents before restyle.
- **Next up:** Chapters 2 (telemetry library — first real `lib/` code) and 3 (rocket equation,
  derived and measured). Create `lib/` and `missions/` alongside.
- **Open questions:** none blocking.
