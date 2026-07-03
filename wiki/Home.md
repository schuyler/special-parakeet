# Flight by Wire: Aerospace Engineering with Kerbal Space Program and kOS

You have flown to the Mun on instinct. You have eyeballed a gravity turn, wiggled a maneuver node until the dotted line kissed Duna, and quicksaved before every landing. You are, in other words, a test pilot.

This book is about becoming an engineer.

We are going to rebuild your piloting instincts as *math you can compute* and *code you can run* — the same math that flew Apollo and lands Falcon boosters — using [kOS](https://ksp-kos.github.io/KOS/), a mod that puts a programmable flight computer on your vessel. Every chapter does three things:

1. **Teaches the science.** Real physics, derived gently, with numbers plugged in for Kerbin. No hand-waving, but no prerequisites beyond algebra and a willingness to meet a square root in a dark alley.
2. **Builds the library.** Each chapter adds tested, reusable routines to `lib/` — your personal flight software stack. Nothing is a throwaway example; the code accretes.
3. **Flies a mission.** Chapters end on the pad (or the runway, or the Mun). The missions get more ambitious precisely as fast as your library does, because the missions *are* the library.

## The arc

The book has a destination: a **single-stage-to-orbit spaceplane** — the hardest engineering problem in the game — and, eventually, one that flies to **Laythe** and back. Everything is in service of that goal, and everything is earned in order:

- **Rockets come first, as lab equipment.** Uncrewed probes on simple rockets are the frictionless inclined plane of this book: the cleanest possible apparatus for measuring gravity, the rocket equation, and the shape of an orbit. You'll fly them instrumented, log the telemetry, and tune your designs against your own flight data. The reader's identity is spaceplane engineer from page one; the rockets are just how we take measurements.
- **Then you learn to operate in space** — transfers, rendezvous, docking — because a launch system with nowhere to go is a firework.
- **Then you learn to land where you meant to.** Powered descent on an airless moon is harder than any orbital transfer; *precision* powered descent — within walking distance of the fuel depot you built last mission — is harder still, and it's what a surface base demands. The Mun and Minmus outposts you establish here are not a detour: they're the fuel infrastructure the expedition runs on.
- **Then wings**, and the SSTO problem in full: aerodynamics measured in flight, engines mapped empirically, the ascent corridor flown as an energy-management problem, the design iterated against telemetry until the payload fraction stops being a joke.
- **Then the graduation flight** — a fully reusable, fully autonomous Kerbin logistics system — and finally **the expedition**: Minmus springboard, Jool system, Laythe, home.

Vehicle design is in scope the whole way. This is aerospace engineering in the full sense: wing sizing, engine count per ton, center of mass versus center of lift, intake area, thermal margins — with the library used to measure whether a design change actually worked, because that's what the telemetry is *for*.

## How to use this book

- **Play along.** Every chapter assumes you're at the keyboard with KSP running. The code is in this repository under `missions/chNN/` (chapter scripts) and `lib/` (the shared library, shown at its state as of each chapter).
- **Do the exercises.** They're not decoration; several later chapters assume you did them.
- **Expect failure.** Rockets explode. Scripts have bugs. Both are data.

**Requirements:** KSP 1.x, the [kOS mod](https://ksp-kos.github.io/KOS/) (installable via CKAN), and a stock-ish install. No other mods are required.

## The chapters

### Part I — Ground School

*Uncrewed probes, simple rockets, and the habit of measuring instead of guessing.*

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 1 | [The Flight Computer](Chapter-01-The-Flight-Computer.md) | Newton's law of gravitation; what *g* really is | First scripts; the terminal; `LOCK`; staging under program control |
| 2 | Reading the Instruments | Position, velocity, and acceleration as vectors; reference frames (why "surface speed" and "orbital speed" disagree); the atmosphere as a function of altitude | `lib/telemetry.ks` — logging flight data to files and graphing it |
| 3 | The Tyranny of the Rocket Equation | Momentum, mass ratio, specific impulse; deriving Tsiolkovsky's equation — then *measuring* it from your own sounding-rocket telemetry. Also: the first hint of why a single-stage spaceplane shouldn't be possible | `lib/rocket.ks` — TWR, stage delta-v, burn durations |

### Part II — Getting to Orbit

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 4 | The Ascent Problem | Gravity losses, drag losses, dynamic pressure (max-Q); why the gravity turn is shaped the way it is | `lib/ascent.ks` — a parameterized launch autopilot |
| 5 | What Is an Orbit? | Falling and missing; Kepler's laws; conic sections; deriving vis-viva from energy conservation | `lib/orbits.ks` — orbital speed, period, semi-major axis |
| 6 | Maneuver Nodes | The impulsive-burn approximation; burn timing; why you start the burn *before* the node | `lib/maneuver.ks` — create and execute nodes. **Mission:** a probe to circular orbit, hands off from pad to parking orbit |

### Part III — Orbital Operations

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 7 | Reshaping Orbits | The Hohmann transfer, derived and budgeted; plane changes and why they're expensive; the Oberth effect | `lib/transfer.ks` — change apoapsis/periapsis, full Hohmann transfers |
| 8 | Where Will You Be? | Mean, eccentric, and true anomaly; Kepler's equation; predicting your position (and altitude) at a future time | `lib/kepler.ks` — anomaly conversions, time-to-altitude |
| 9 | Rendezvous | Phasing orbits, synodic periods, closest approach; why you catch up by slowing down | `lib/rendezvous.ks` — intercept planning and velocity matching |
| 10 | Docking | Relative motion, RCS translation, port alignment | `lib/docking.ks`. **Mission:** assemble a fuel depot in Kerbin orbit, with automated fuel transfer |

### Part IV — The Outpost

*Powered descent, precision, and the surface bases the expedition will run on.*

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 11 | Falling with Style | Deorbit targeting; the suicide burn, derived; why there is no maneuver node for landing — numerical methods and feedback control where closed-form math runs out | `lib/descent.ks`. **Mission:** a powered landing on Minmus (0.49 m/s² — the gentlest classroom in the system) |
| 12 | Landing Where You Meant To | Landing-site prediction over a rotating body; terrain; the divert problem; error budgets | `lib/landing.ks` — targeted descent. **Mission:** land twice on Minmus, within sight of your first flag |
| 13 | The Base | Higher gravity as a difficulty step; ISRU and the economics of off-world fuel; surface rendezvous | **Missions:** a Minmus refueling base; a targeted Mun landing to prove the technique where the gravity is real |

### Part V — Flight School

*Wings at last.*

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 14 | Flying on Air | Lift, drag, and the lift-to-drag ratio; angle of attack; measuring your plane's actual drag polar in flight | `lib/aero.ks` — L/D measured, not assumed |
| 15 | The Air-Breathing Engine | Thrust versus speed and altitude; flameout; the flight envelope, mapped empirically | Engine survey scripts; envelope charts from your own data |
| 16 | Autopilot | Feedback control and PID loops; holding pitch, speed, and altitude | `lib/flight.ks`. **Mission:** an autonomous cross-Kerbin flight |

### Part VI — The SSTO Problem

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 17 | The Ascent Corridor | The climb as energy management; dynamic pressure and thermal limits; the airbreathing speedrun; engine mode switching | `lib/ssto.ks` — ascent guidance, v1 |
| 18 | Iterating Under Instruments | Telemetry-driven design: change one thing, fly, measure, repeat. Wing area, engine count, intake sizing, CoM/CoL — every change scored against the data | Ascent guidance, v2 through v*n* — the honest version, with the failures left in |
| 19 | On Fumes | Closed-cycle finish and circularization; fuel margins; payload fraction as the only score that matters | **Mission:** runway to a stable orbit, single stage, with payload |

### Part VII — The Logistics Company

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 20 | Reentry as Energy Management | Deorbit targeting to hit KSC; drag at Mach 5; heating; managing the energy you spent nineteen chapters acquiring | `lib/reentry.ks` |
| 21 | Approach and Autoland | Glide slopes; runway alignment; flare | `lib/autoland.ks` |
| 22 | Graduation | Putting the whole library together | **Mission:** a repeatable, hands-off station resupply flight — runway to depot to runway. The coffee-sipping chapter |

### Part VIII — The Expedition

| # | Chapter | The science | The code |
|---|---------|-------------|----------|
| 23 | Patched Conics | Spheres of influence; transfer windows; phase and ejection angles; mid-course corrections | `lib/interplanetary.ks` |
| 24 | The Long Way Round | The Minmus springboard (your Part IV base, earning its keep); Jool aerocapture | **Mission:** outbound leg |
| 25 | Laythe | The only other place in the system where wings work | **Mission:** land on Laythe, fly home. Optional encore: a Vall landing, for those who found Minmus too polite |

### Appendices

- **A. Kerbin and Friends** — physical constants for every body you'll visit (radius, μ, rotation, atmosphere)
- **B. The Library Reference** — every routine in `lib/`, by chapter of introduction
- **C. Further Reading** — where to go when Kerbin feels small (Braeunig's Rocket & Space Technology, Bate/Mueller/White's *Fundamentals of Astrodynamics*, and friends)

## Conventions

- Code blocks are kOS (kerboscript). Anything you type at the terminal is shown with its trailing period, because kOS demands it and forgetting it is a rite of passage.
- Worked examples use **Kerbin numbers**: radius 600 km, μ = 3.5316×10¹² m³/s², sea-level gravity 9.81 m/s². Where the real-world value illuminates, we compare (Kerbin is one-tenth the size of Earth but has the same surface gravity — a fact with consequences we'll keep tripping over).
- Early flights are **uncrewed**. The computer is the crew; that's the premise. Kerbals ride along once the software has earned their trust, which is also roughly how NASA did it.
- Sidebars titled **"In the real world"** connect each technique to actual spaceflight history and practice.

---

*Next: [Chapter 1 — The Flight Computer](Chapter-01-The-Flight-Computer.md)*
