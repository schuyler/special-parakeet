# Flight by Wire: Aerospace Engineering with Kerbal Space Program and kOS

You have flown to the Mun on instinct. You have eyeballed a gravity turn, wiggled a maneuver node until the dotted line kissed Duna, and quicksaved before every landing. You are, in other words, a test pilot.

This book is about becoming an engineer.

We are going to rebuild your piloting instincts as *math you can compute* and *code you can run* — the same math that flew Apollo and lands Falcon boosters — using [kOS](https://ksp-kos.github.io/KOS/), a mod that puts a programmable flight computer on your vessel. Every chapter does three things:

1. **Teaches the science.** Real physics, derived gently, with numbers plugged in for Kerbin. No hand-waving, but no prerequisites beyond algebra and a willingness to meet a square root in a dark alley.
2. **Builds the library.** Each chapter adds tested, reusable routines to `lib/` — your personal flight software stack. Nothing is a throwaway example; the code accretes.
3. **Flies a mission.** Chapters end on the pad (or the runway, or the Mun). The missions get more ambitious precisely as fast as your library does, because the missions *are* the library.

By the last page, your library will fly a complete mission — launch, rendezvous, voyage, landing, recovery — while you sip coffee and watch the telemetry, which is how actual aerospace engineers experience spaceflight.

## How to use this book

- **Play along.** Every chapter assumes you're at the keyboard with KSP running. The code is in this repository under `missions/chNN/` (chapter scripts) and `lib/` (the shared library, shown at its state as of each chapter).
- **Do the exercises.** They're not decoration; several later chapters assume you did them.
- **Expect failure.** Rockets explode. Scripts have bugs. Both are data.

**Requirements:** KSP 1.x, the [kOS mod](https://ksp-kos.github.io/KOS/) (installable via CKAN), and a stock-ish install. No other mods are required.

## The chapters

### Part I — Ground School

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 1 | [The Flight Computer](Chapter-01-The-Flight-Computer.md) | Newton's law of gravitation; what *g* really is | First scripts; the terminal; `LOCK`; staging under program control |
| 2 | Reading the Instruments | Position, velocity, and acceleration as vectors; reference frames (why "surface speed" and "orbital speed" disagree); the atmosphere as a function of altitude | `lib/telemetry.ks` — logging flight data to files and graphing it |
| 3 | The Tyranny of the Rocket Equation | Momentum, mass ratio, specific impulse; deriving Tsiolkovsky's equation; delta-v as the currency of spaceflight | `lib/rocket.ks` — TWR, stage delta-v, burn durations |

### Part II — Getting to Orbit

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 4 | The Ascent Problem | Gravity losses, drag losses, dynamic pressure (max-Q); why the gravity turn is shaped the way it is | `lib/ascent.ks` — a parameterized launch autopilot |
| 5 | What Is an Orbit? | Falling and missing; Kepler's laws; conic sections; deriving vis-viva from energy conservation | `lib/orbits.ks` — orbital speed, period, semi-major axis |
| 6 | Maneuver Nodes | The impulsive-burn approximation; burn timing; why you start the burn *before* the node | `lib/maneuver.ks` — create and execute nodes autonomously. **Mission:** launch to a circular orbit, hands off from pad to parking orbit |

### Part III — Orbital Operations

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 7 | Reshaping Orbits | The Hohmann transfer, derived and budgeted; plane changes and why they're expensive; the Oberth effect | `lib/transfer.ks` — change apoapsis/periapsis, full Hohmann transfers |
| 8 | Where Will You Be? | Mean, eccentric, and true anomaly; Kepler's equation; predicting your position (and altitude) at a future time | `lib/kepler.ks` — anomaly conversions, time-to-altitude |
| 9 | Rendezvous | Phasing orbits, synodic periods, closest approach; why you catch up by slowing down | `lib/rendezvous.ks` — intercept planning and velocity matching |
| 10 | Docking | Relative motion, RCS translation, port alignment | `lib/docking.ks`. **Mission:** assemble a two-launch fuel depot in orbit, with automated fuel transfer |

### Part IV — Coming Home

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 11 | Reentry | Deorbit targeting; drag revisited at Mach 5; heating; aerobraking; predicting your landing site on a rotating planet | `lib/reentry.ks` — deorbit burns and impact prediction |
| 12 | Powered Descent | The suicide burn (hoverslam), derived; terrain-relative altitude; boostback | `lib/landing.ks`. **Mission:** fly a booster back to a propulsive landing near the launch site, SpaceX-style |

### Part V — Voyages

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 13 | To the Mun | Spheres of influence and patched conics; transfer windows; capture burns; landing where there is no air | **Mission:** an automated Mun landing and return |
| 14 | Interplanetary | Phase angles and ejection angles; launch windows; porkchop plots (in spirit); mid-course corrections | **Mission:** a probe to Duna |

### Part VI — Wings

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 15 | Flying on Air | Lift, drag, and the lift-to-drag ratio; the flight envelope; air-breathing engines and why they change everything | `lib/aero.ks` — measuring L/D in flight |
| 16 | Single Stage to Orbit | The SSTO ascent profile as an energy-management problem; the capstone | **Mission:** runway to orbit and back to the runway, autonomously |

### Appendices

- **A. Kerbin and Friends** — physical constants for every body you'll visit (radius, μ, rotation, atmosphere)
- **B. The Library Reference** — every routine in `lib/`, by chapter of introduction
- **C. Further Reading** — where to go when Kerbin feels small (Braeunig's Rocket & Space Technology, Bate/Mueller/White's *Fundamentals of Astrodynamics*, and friends)

## Conventions

- Code blocks are kOS (kerboscript). Anything you type at the terminal is shown with its trailing period, because kOS demands it and forgetting it is a rite of passage.
- Worked examples use **Kerbin numbers**: radius 600 km, μ = 3.5316×10¹² m³/s², sea-level gravity 9.81 m/s². Where the real-world value illuminates, we compare (Kerbin is one-tenth the size of Earth but has the same surface gravity — a fact with consequences we'll keep tripping over).
- Sidebars titled **"In the real world"** connect each technique to actual spaceflight history and practice.

---

*Next: [Chapter 1 — The Flight Computer](Chapter-01-The-Flight-Computer.md)*
