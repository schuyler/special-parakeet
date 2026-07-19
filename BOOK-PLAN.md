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
- `reference/` — **frozen source material; do not edit, only mine.**
  - `reference/original/` — Schuyler's original working kOS scripts (formerly the root
    `*.ks` files). The book code is a fresh, pedagogically-ordered rebuild from these.
  - `reference/core/`, `reference/landing_v2/`, `reference/wip/`, `reference/script/` —
    main_v2's draft library: Schuyler's first pass at Keplerian mechanics and precision
    landing with Claude's help. Treated as a draft of the trail the book now blazes
    deliberately; mined, never edited. Full commit history for all reference material
    lives on the `main` and `main_v2` branches.
  - `reference/variants/` — divergent versions of scripts that were independently modified
    on other branches: `next.ks` is the tutorial-branch refinement (auto-staging block
    removed, finer maneuver control); `land_at_periapsis.ks` is main's shorter
    pre-main_v2 version (131 lines vs. main_v2's 273-line rewrite). Kept because the
    progression between drafts is book material.

### Source-material map (scripts → chapters)

Unless otherwise noted, paths below are relative to `reference/original/`. Where
`reference/original/` and `reference/core/` cover the same topic, `core/` is the
later, better draft; the book can show the progression where instructive.

| Script(s) | Feeds chapter(s) | Notes |
|---|---|---|
| `telemetry.ks`, `resources.ks` | 2 | logging approach |
| `common.ks` (`burn_duration`, rocket eq.) | 3, 6 | has a "confirm this math" TBD — becomes an exercise |
| `launch.ks`, `launch_vacuum.ks`, `launch_correction.ks` | 4 | ascent profiles |
| `common.ks` (`orbital_speed` vis-viva), `orbital.ks` | 5, 8 | |
| `common.ks` (`execute_node`, `node_from_velocity`), `circularize.ks` | 6 | node execution incl. coordinate-frame rotation |
| `set_periapsis.ks`, `move_periapsis.ks` | 7 | |
| `set_inclination.ks` | 7 | plane-change maneuver; fits ch. 7 (orbital transfers) rather than ch. 9 (rendezvous) because inclination change is a purely Hohmann-adjacent maneuver with no phasing |
| `kepler.ks`, `orbital.ks` (anomalies) | 8 | `kepler.ks` header ("don't need it, see orbit.ks") is itself a good pedagogical beat: derive by hand, then learn what the API gives you |
| `intercept.ks`, `next.ks`, `wait_for_launch.ks` | 9 | |
| `dock.ks`, `dock2.ks`, `fuelxfer.ks` | 10 | |
| `deorbit.ks`, `deorbit_simple.ks`, `deorbit_node.ks`, `drop_periapsis.ks`, `landing.ks`, `land_at_periapsis.ks`, `common.ks` (`time_to_surface`, `landing_time` Newton iteration) | 11–12 | the terrain-height Newton iteration is exactly the "numerical methods where closed form runs out" lesson |
| `predict_landing.ks` | 11–12 | early landing-site prediction; compare to `reference/landing_v2/` for the improved approach |
| `boostback.ks` | 12 (divert/targeting), sidebar Falcon | |
| `aero.ks` | 14 | lift/drag computation |
| `ssto.ks`…`ssto4.ks` | 17–18 | the iteration story of ch. 18 |
| `reentry.ks`, `aerobrake.ks` | 20 | |
| `reference/core/kepler.ks`, `reference/core/test_kepler.ks` | 8, 11 | `time_to_altitude` and `free_fall_time` rewrites; `test_kepler.ks` is the companion test harness |
| `reference/core/maneuver.ks`, `reference/core/optimize.ks`, `reference/core/rocket.ks`, `reference/core/impact.ks` | 3, 6, 11–12 | maneuver planning and impact/landing math from the main_v2 draft |
| `reference/landing_v2/` (all files) | 11–12 | complete deorbit-burn calculation; datum/terrain impact prediction via Newton iteration; `minimize.ks`; `time_to_closest_approach.ks`; reworked landing state machine — this is the deeper draft of the ch. 11–12 material |
| `reference/wip/test_free_fall.ks` | 11 | test harness for `free_fall_time`; shows expected vs. measured values for the Newton-iteration approach |

## Style

- Voice target: **`schuyler-docs`** — direct, conversational, honest about limitations, dry
  humor that never crowds out technical content. Vendored into this repo at
  `.claude/skills/schuyler-docs/SKILL.md`, adapted for discursive tutorial prose (the global
  original targets READMEs/reference docs). TODO: restyle `wiki/Home.md` and Chapter 1,
  which were drafted before the skill was known.
- kOS code shown with trailing periods; terminal-typed commands shown as typed.
- Worked examples use Kerbin numbers (radius 600 km, μ = 3.5316×10¹² m³/s², g = 9.81 m/s²);
  compare to real-world values where it illuminates.
- Math level: algebra + gentle derivations; every derivation lands in a number the reader can
  check at the terminal.
- Chapters end with **exercises** (some load-bearing for later chapters) and a **What's next**
  hook.

## Drafting: process and voice

How chapter prose gets written. Adapted from the drafting conventions of Schuyler's theremin
tutorial wiki, which were developed against the same failure mode: prose composed *as a
document* goes turgid — define-first, nominal, stiff — no matter what the style guide says.
The fix is structural, not exhortative.

**Chapters are drafted collaboratively, a section at a time.** Propose the chapter's structure
first, then work section by section — draft, Schuyler reads and asks questions, refine, move
on. Don't draft a whole chapter cold and hand it over.

**Code before prose.** The lib/ routines and mission scripts a chapter presents are written
and tested first. The prose walks through working code; it never narrates code that doesn't
exist yet.

**The one instruction: write it the way you'd say it out loud** — explain the section to a
sharp friend across a table, then tidy that into prose. A clear spoken explanation and a
stiff page are the same content written two ways; the difference is whether it was said or
composed.

### Answer, don't author

The core protocol. **Never generate prose from an instruction containing "write the
section/chapter/draft"** — that phrasing triggers document register and the output stiffens
regardless of style instructions. Generate every section as a spoken answer instead. The
template, filled in per section:

> A reader has just finished [previous chapter/section] and asks: **"‹the question this
> section exists to answer›"**
>
> Below are the closing paragraphs of what they just read. Answer their question in ‹2–3›
> paragraphs, in the same voice, continuing directly from this opening sentence:
>
> **"‹one in-voice opening sentence, agreed in chat›"**
>
> ‹final 2–3 paragraphs of the previous chapter/section, pasted verbatim›
>
> No headings, no links, no closing summary. When you would stop talking, stop writing.

When a section presents code, paste the tested script (or excerpt) into the prompt as part
of the materials — the answer walks through it. Code blocks may appear in the answer;
headings and links may not.

Then two mechanical follow-ups, as separate turns — never merged into the generation:

1. **Say-aloud pass:** "Rewrite any sentence you wouldn't say out loud to a colleague.
   Change nothing else."
2. **Structure pass** (after approval): add headings, relative links, exercises,
   "In the real world" sidebars, and the "What's next" hook to the approved text without
   altering a sentence.

The question slot supplies the addressee, the pasted paragraphs and opening sentence prime
the register by continuation rather than instruction, and quarantining structure keeps the
"document" trigger out of the composition step entirely.

### The process

1. **Read the neighbors first.** Before drafting a line, read the previous chapter (all of
   it if practical; at minimum its back half) — the committed prose, not its TOC row. It carries what's
   already established and, just as much, the *voice* to match. Reconstructing the tone from
   this plan instead of from the actual chapters stiffens the prose every time.
2. **Anchor to the mission.** Say what the reader needs the chapter for in the build toward
   SSTO. That decides scope — keep what's load-bearing, cut the true-but-tangential.
3. **Draft by ear.** Each section: concrete scene first (a flight, a number from the
   terminal, a thing that just exploded), name the abstraction once it's visible, stop when
   the point is made.
4. **Tidy lightly.** Clean the spoken version into prose; don't recompose it into page voice.
5. **Read it back as a reader** — would I say this sentence to a person? If not, it's genre
   voice. Cut or rewrite.

### Diagnostics

For when a sentence clunks — not a pre-flight checklist. Drafting with this list in mind is
itself the self-consciousness that stiffens prose. Write by ear; reach for these only to
name a fault already felt:

- **Concrete before abstract.** Show an instance; name the concept after. Never open a
  paragraph by characterizing what an abstract thing *is*.
- **Only as deep as the job needs.** Knotted prose usually means discharging a subtlety the
  chapter doesn't require — cut the ambition and the sentence relaxes.
- **Don't narrate the prose, teach the subject.** No "this chapter does X," no "this unlocks
  Y later," no who-this-is-for openers, no teacherly signposting ("for now, just the
  framing"). Headings are the transitions — don't restate them. The tell: if a sentence is
  just as true with the subject swapped out, it's describing, not teaching.
- **Negation must clear a *real* misconception.** Correct a model the reader actually
  arrives with (more-boosters-fixes-everything ✓); don't invent one to swat. The tell: can
  you name someone who believed it?
- **Don't certify your own honesty or emphasis.** No "the honest picture," "to be clear,"
  "genuinely," "this matters." Earn it in the statement; don't assert it.
- **Defer with an inline link on a real claim,** not a standalone pointer. "That falloff is
  [the rocket equation](Chapter-03…), next chapter" works; "See Chapter 3 for more" doesn't.
- **Person splits by location.** *You* at the keyboard and in the game — the reader's
  actions on their own rocket in their own save ("you stage," "your log"). *We* at the
  whiteboard — the shared intellectual work of derivations and design reasoning ("we need
  momentum," "we can budget this burn"). Procedural imperatives stay imperative ("stage,"
  "lock steering to up"). What's banned in *both* registers is the promissory, teacherly
  address the theremin wiki's we-only rule was actually targeting: "you'll build," "you
  will learn," narrating the reader's future experience instead of teaching. (The we-only
  rule itself is not imported: the theremin wiki documents a shared bench, where "our hand
  near the antenna" is nearly literal; this book's reader is alone in the cockpit, and the
  spoken answers the drafting protocol produces naturally address a "you.")

## Chapter stubs

Every undrafted chapter at the frontier (the next one or two chapters) gets a stub page in
`wiki/`, linked from the TOC. A stub carries:

- **Position** — part and neighbors in the arc.
- **Scope** — the science/code/mission contract, from the TOC row, in a sentence or two.
- **Reader's question** *(provisional)* — the question the chapter exists to answer, posed
  by a reader who just finished the previous chapter. This fills the addressee slot in the
  answer-don't-author template.
- **Likely follow-ups** *(candidates)* — one fills the addressee slot per section at draft
  time; not required coverage.
- **Deferred here** — debts this chapter pays: claims and forward references from other
  chapters that land on this page for their justification.

Stubs roll forward as the frontier moves; a stub graduates by being drafted over. Don't stub
the whole TOC — reader's questions written twenty chapters ahead of the prose are
speculation, not planning.

## Working agreements (for future sessions)

- Work lands on `main`: sizable work happens on short-lived branches squash-merged in after
  Schuyler's review; small reviewed-in-conversation changes may commit to `main` directly.
  Push at natural checkpoints. (The old `claude/kerbal-aerospace-tutorial-q6gs9o` branch is
  retired — its content was squash-merged into `main` on 2026-07-03; kept frozen, don't
  build on it.)
- Workflow per chapter: stub → section-by-section drafting per "Drafting: process and voice"
  above → Schuyler reviews → revise. Don't mass-produce chapters ahead of review; voice
  calibration is still in progress.
- When a chapter introduces library code, add it to `lib/` and the mission scripts to
  `missions/chNN/` in the same commit as the chapter text.
- Keep `wiki/Home.md`'s TOC consistent with this document; if structure changes, update both
  and record the *reason* here.
- Update the Status section below before ending a session.

## Status

*Last updated: 2026-07-19*

- **Done (2026-07-19, later):** `reference/original/optimize_descent_angle.ks` — piece 3's
  front half, new. The terrain survey that stood behind gamma as "the human's judgment" in
  `plan_doi.ks` now exists as code: walk the approach up-range from the site and take the
  steepest ray any obstacle demands, `gamma = max arctan((terrain + margin − h_handoff)/x)`.
  No search: Δv rises with gamma (the trend plan_doi's sweep prices), so the optimum is the
  shallowest certified slope — "optimize" means "find the binding obstacle". Two scoping
  decisions, both argued in the header: (1) the design note's survey-joins-the-fixed-point
  coupling is deferred — it only bites on an inclined orbit, and under the stack's standing
  equatorial assumption the track through the site is the site's own parallel, so the
  survey is pure geography: reads nothing from the ship but the body, places no node, runs
  before the parking orbit exists, and needs no third copy of the arc march. (2) The coast
  clearance rule (open item 1) is explicitly NOT here — it is a property of the placed
  ellipse, so it belongs in plan_doi's verdict (walking `nd:orbit` before declaring
  victory), which is where it should land next; until then the coast is still the human's
  risk, and the script's header says so. Parameters: `max_terrain_height` (the one body
  fact kOS cannot read — no default; Minmus ≈ 5725 m), `terrain_margin` (how far the
  terrain model is trusted, ray-side twin of open item 1's number), `gamma_floor` (1°, the
  shallowest approach flown regardless of how flat the survey reads), `dx` (open item 8's
  knob, 100 m). The walk self-terminates once the ray tops the peak, capped at a quarter
  of the body — the cap is routine on the Minmus flats (a 1° ray tops the peak ~330 km
  out) and is reported as coast country, not hidden. Witness: `gamma_survey.log` — gamma,
  the forcing obstacle, walk stats, and a decimated corridor profile (x, terrain, ray) for
  plotting. **Unflown** — but unlike the flight scripts it is dry-runnable: it only reads
  terrain, so a bridge run from any save on or around Minmus exercises it end to end.

- **Done (2026-07-19):** `reference/original/plan_doi.ks` revised in place to plan for
  `powered_descent_min.ks` instead of the table-flying controller. **Flight news first**:
  the min rendition has now flown — pinpoint Minmus landings at TWR ~37 and the same craft
  thrust-limited to TWR ~2 (Schuyler's report), so the invariants note's predictions have
  telemetry behind them and the min design is the one the planner should serve. Review
  confirmed the controller has no fuel knob left: the braking Δv is fixed by PDI placement
  (higher solved throttle = shorter burn = less gravity loss), terminal is free fall plus a
  kinematic `f_max` arrest, and the optimal coast-then-`f_max` descent is the limit the
  controller already contains — so efficiency lives entirely in the plan, which is what the
  revision leans into. Changes: (1) the fixed-step `integrate_arc` replaced with min's
  adaptive-step `endpoint`, nearly verbatim — seed from the candidate ellipse instead of
  the live ship, accuracy bounds (`pitch_tol`, `v_frac`) as parameters so the coarse tier
  can loosen them; still duplicated by choice, not yet a shared library. (2) The overshoot
  allowance, half-step error probe, `x_shrink_per_f` trim gain, headroom-vs-allowance
  check, and the deliberate arrive-long lead all deleted — the live re-solve corrects both
  signs of error, so arriving long just holds the flown throttle above the solved one; the
  endpoint is placed AT the site. (3) Cross-track check recast for min's τ = 20 s yaw law:
  bias angle at PDI and the residual a τ-closure leaves at handoff, replacing the old
  `6y/t_go²` capacity integral. (4) `f_solved` margins reported against both bounds
  (authority to shorten and to stretch). (5) New gamma sweep: the coarse fixed point run
  at 0.75×/1×/1.5× the asked slope, each priced (DOI + arc Δv), printed and logged — the
  one fuel judgment, priced instead of sloganized. Parameter list 11 → 7. **Unflown**: the
  revised planner. Open items from the review of min itself (not acted on): the ignition
  fallback `f_cmd = f_max` conflates bisect's two bracket failures (safe either way, but
  the log can't tell which happened), and `bisect`'s failure path prints four lines that
  would tear the fixed-row readout mid-burn. **Follow-up (same day)**, after a parameter
  census with Schuyler: `f_min` retired from the arc contract — the solve brackets at zero
  throttle in both files (a no-thrust arc runs into the terrain floor, a real undershoot, so
  the bottom end needs no tuned floor); terminal keeps its 0.05 as `f_idle`, its own idle
  threshold, not the solve's (note: `powered_descent_min`'s positional parameter list
  shrank — `f_max` is now 4th). `plan_doi`'s coarse tier and `f_eps` also retired: one
  fixed point at flight fidelity (the tiers were rent paid to the fixed-step integrator),
  bisection tolerance derived from the bracket (`f_max/4096`), and the duplicated march
  synced to min's new terrain floor (`h <= tgt:terrainheight`). Parameter list now 6; the
  duplicated integrator's departures from min's copy are down to one (the seed). The two
  min-side derivations then landed as well, **both unflown**: `a_lat_max = g0·tan(tilt_max)`
  (0.3 is revealed as Minmus's instance of exactly that — near-no-op there, real elsewhere)
  and yaw `tau = t_go/3` frozen at ignition (closure becomes e^-3 of the PDI offset by
  construction; the planner's verdict check updated to match, its residual warning now
  reading "five percent of this offset is N m"). The schedule check landed too: the
  planner's verdict now warns when terminal would ignite behind schedule at handoff
  (`speed_handoff` vs `sqrt(2·a_dec·landing_height)` — the planner is the only program
  holding both numbers, which is why the check lives there and not in flight). That closes
  the parameter census; nothing from it remains unapplied. **Flight plan (Schuyler)**: next
  session flies `powered_descent_min` first — verifying the derived `a_lat_max` (expect
  free-fall `a_cmd` capping at ~0.28 in the log, otherwise identical) and the `t_go/3` yaw
  law (watch the `cross` column's decay) — then the revised `plan_doi` end to end.

- **Done (2026-07-18):** `notes/level-flight-fuel-optimization.md` — design note for a
  cruise optimizer atop the `autopilot` branch's cascade: a sixth, outermost loop that
  chooses the altitude/airspeed setpoints by minimizing measured fuel-per-metre
  (J = ṁ/v, primary measurement is the ship's own mass delta over ~30 s windows;
  `drag_vector()`-based D/v kept as cross-check only until its signs are validated).
  Key decisions: step-and-compare (twiddle) rendition first, sinusoidal extremum seeking
  only if needed; trim-gated measurement windows with turns suspended; converged trim
  throttle as the interior-vs-boundary-optimum diagnostic (throttle < 1 ⇒ the Mach drag
  rise picked the altitude; throttle pegged ⇒ the engine did). Companion to
  `ssto-aero-optimization.md` (same technique, ascent phase). Feeds the Part V/VI seam
  (ch. 16 payoff exercise). Analysis only — no code, unflown.

- **Done (2026-07-18):** `notes/powered-descent-invariants.md` + `reference/original/
  powered_descent_live.ks` — the descent's invariants worked out with Schuyler, and the
  rendition they imply. The note establishes that the retrograde hold makes the braking
  trajectory a one-parameter family (state + throttle → arc), so the descent table in
  `powered_descent.ks` is a cache of a computation that can run live; everything downstream
  of the cache's staleness (trim gain, overshoot allowance, taper, ratchet) deletes, and
  the safety invariant collapses to one test — "does any throttle keep the arc above the
  gate?" — run identically pre-coast and every look in flight. The script re-solves the
  throttle from live state every few seconds (bisection over the same Euler march, seeded
  from the ship instead of periapsis), with the gate as the floor under the command and
  `f_max` as the ceiling. Settles capability-driven-descent.md open items 3 and 4
  (annotated there). Also traced the lesson through the old landing family: the old
  scripts buried periapsis below the surface (trajectory hits the site, burn is timed);
  Apollo's inversion — periapsis safe and up-range, the burn brings you down — is what let
  the DOI/coast/PDI/terminal phases come apart. Strong chapter spine for Part IV: the
  phases discovered by getting them wrong. **Unflown**: the note's Predictions section
  lists the telemetry signatures to check on first flight (throttle staircase vs
  oscillation, handoff miss, Δv vs the 244 m/s baseline). Companion spike
  `powered_descent_min.ks`: the same design with all envelope protection removed — ~80
  statements that fly plus the flight recorder (kept on Schuyler's call: telemetry is the
  working agreement, so the recorder is part of the minimum, not scaffolding; same CSV
  columns as the siblings). Five ideas, no orbital mechanics (it only ever integrates
  from the live ship), `landing_height` absent because the planner spent it into the
  ellipse. Candidate for the chapter's presented listing, with `_live` as the "what a
  flyable version adds" follow-on. Its lateral law (velocity bias, tau = 20 s) is simpler
  than `_live`'s constant-jerk law and equally unflown.

- **Done (2026-07-13):** `notes/klumpp-guidance-derivation.md` — companion to the powered-
  descent guide, derived from a reader Q&A thread. Two things the sibling guide only stated
  in passing, now worked from scratch: (1) the **jerk dynamics** behind the guidance law —
  the constant-jerk kinematic ladder (one rung up from the constant-acceleration physics-
  class formulas), the boundary-value solve that produces `a_cmd = 6R/t² − (4v+2v_tgt)/t`,
  and the reading of that law as a *PD controller* with derived, escalating gains and a
  constant implied damping ratio `ζ = 2/√6 ≈ 0.82`; (2) an **alternative `t_go` closure** —
  the Apollo-style target-acceleration form, which is a closed-form *quadratic*
  (`a_tgt·T² − (2v+4v_tgt)T + 6R = 0`) versus the current guide's thrust-margin bisection.
  Includes a drop-in kOS `solve_t_go_accel`, a which-closure-when comparison (thrust-margin
  for braking, target-accel for approach — the phase split Apollo actually used), and the
  point that the acceleration target needs **no precomputed trajectory** (synthesised live
  from local `g`). Flagged for the chapter as the more *teachable* closure. Both notes stay
  working material until Part IV drafting reaches them.

- **Done (2026-07-12):** `notes/apollo-powered-descent.md` — implementation guide for the
  chapters 11–12 targeted powered-descent script, modeled explicitly on the Apollo
  sequence (DOI → coast → P63 braking → P64 approach → P66 terminal). Key design
  decisions recorded there: aim-point guidance (Klumpp quadratic law) instead of
  ballistic-impact prediction, which removes the terrain-height prediction problem the
  `reference/` attempts fought; terrain height at the *target* is known exactly via
  `geopositionlatlng`; sequential phase functions replace nested `when` triggers. The
  guide is working material — it gets rewritten into chapter prose (and `lib/` code)
  when Part IV drafting reaches it. New `notes/` directory holds authoring working
  papers that are neither reader-facing (`wiki/`) nor frozen (`reference/`).

- **Done (2026-07-05):** drafting conventions adapted from the theremin tutorial wiki's
  drafting process (its `Agent_Guide` and `Tutorial/Overview` pages):
  - New "Drafting: process and voice" section above — the answer-don't-author protocol
    (the load-bearing piece), the five-step process, and the clunk diagnostics. The
    template ordering follows the theremin source verbatim; a reviewer argued the pasted
    paragraphs should precede the opening sentence — declined for fidelity, revisit if
    chapter 2 drafting shows the ordering matters.
  - New "Chapter stubs" convention (frontier-only); stubs created for chapters 2–3 and
    linked from the TOC and chapter 1's footer.
  - **Person rule decided** (litigated with Schuyler): *you* at the keyboard, *we* at the
    whiteboard, imperatives stay imperative, promissory "you will learn/build" address
    banned in both. The theremin we-only rule deliberately not imported; reasoning
    recorded in the Diagnostics section.
  - Not adapted (out of scope by decision): the theremin *(verify)*/*(judgment)* honesty
    flags, the verify-backlog page, and the design-variable-numbers convention.
  - `schuyler-docs` vendored at `.claude/skills/schuyler-docs/SKILL.md`, adapted for
    discursive prose: README section patterns, length tables, and the reference-example
    corpus dropped; voice principles kept with book-flavored examples; process and chapter
    structure defer to this document rather than being duplicated.
- **Pending:** restyle of `wiki/Home.md` and Chapter 1 against the vendored skill (the
  Style-section TODO). The Home.md restyle must also add the front-matter AI disclosure
  the vendored skill's honesty principle calls for — no home for it exists yet. Note the
  restyle is partly *structural*, not just sentence-level:
  Chapter 1's "Mission briefing" opener is a learning-objective list in promissory "you
  will" address — banned twice over by the new conventions. Whether "Mission briefing"
  survives as a chapter convention is Schuyler's call at restyle time.

- **Done:** outline settled through the decisions recorded above; `wiki/Home.md` (front page +
  full TOC); `wiki/Chapter-01-The-Flight-Computer.md` drafted (uncrewed OKTO trainer; gravity
  experiment; `liftoff.ks`).
- **Done (this session):** repository reorganization — legacy code frozen under `reference/`:
  - `reference/original/` holds Schuyler's original root `*.ks` scripts (removed from root).
  - `reference/core/`, `reference/landing_v2/`, `reference/wip/`, `reference/script/` bring
    in main_v2's draft library. Decision: main_v2 is a draft to mine, not finished code —
    it represents Schuyler's first pass at Keplerian mechanics and precision landing with
    Claude. The book now blazes that trail deliberately, using main_v2 as a reference draft.
  - `reference/wip/test_free_fall.ks` added as a tracked file (was untracked before).
  - Source-material map updated to reflect the new paths and the additional files.
  - This commit will land on `main` via squash merge after Schuyler's review.
- ~~Pending review: Chapter 1 voice — awaiting `schuyler-docs` skill contents before
  restyle.~~ *(Resolved 2026-07-05: skill vendored; restyle itself still pending, tracked
  above.)*
- **Next up:** Chapters 2 (telemetry library — first real `lib/` code) and 3 (rocket equation,
  derived and measured). Create `lib/` and `missions/` alongside.
- **Open questions:** none blocking.
