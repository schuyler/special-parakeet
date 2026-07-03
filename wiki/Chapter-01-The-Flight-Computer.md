# Chapter 1 — The Flight Computer

*In which you stop touching the controls.*

## Mission briefing

By the end of this chapter you will have:

- installed a programmable flight computer on a rocket;
- used it, while still bolted to the launchpad, to verify Newton's law of universal gravitation;
- written a script that flies a rocket — countdown, ignition, ascent, and parachute — with your hands folded in your lap.

That last item is the point of the whole book. Every real spacecraft since about 1963 has been flown by computer, because the physics of rocket flight happens faster and more precisely than human reflexes can follow. The Apollo Guidance Computer that landed astronauts on the Moon had about 4 kilobytes of RAM — less memory than this paragraph takes to store — and it still flew the descent better than a human could have, right up until Neil Armstrong took over for the last few hundred meters because the computer was steering him into a boulder field. We'll aim for the computer's consistency *and* the pilot's judgment: yours, expressed in code, written before the engines light.

## Setting up

Install **kOS** (it's on CKAN, or [GitHub](https://ksp-kos.github.io/KOS/)). It adds:

- **Computer parts.** In the VAB you'll find them under *Control* — the **CX-4181 Scriptable Control System** is the little one you'll bolt to everything for the rest of this book.
- **A terminal.** Right-click the part in flight, choose *Open Terminal*, and you get a green-on-black console that would make a 1970s NASA engineer feel at home.
- **A place for scripts.** Files live in `Ships/Script/` inside your KSP folder — that's the *archive*, which your ships can read from at the KSC. Write code in any text editor you like.

One more thing before we build: nobody is riding this rocket. Flights in this book are **uncrewed** until the software has earned a passenger — the computer is the crew, which is the whole premise. (This is also roughly how NASA did it.)

Build the simplest possible test stand: an **OKTO probe core, a small battery, a parachute, and the CX-4181**. No engines yet. Put it on the launchpad and open the terminal.

## Conversations with a spacecraft

The terminal is live — it runs commands the moment you type them. Try:

```
PRINT "Hello, Kerbin.".
```

Note the period. Every kOS statement ends with one, like a telegram STOP. Forgetting it is a rite of passage; the error message will become an old friend.

Now ask the ship something:

```
PRINT SHIP:ALTITUDE.
```

That number — around 74 meters, the height of the pad above sea level — came from the vessel itself. `SHIP` is a structure describing your vessel, and the colon reaches into it. The ship knows a *lot*:

```
PRINT SHIP:MASS.          // tonnes
PRINT SHIP:VELOCITY:SURFACE:MAG.   // zero, hopefully, for now
PRINT SHIP:BODY:NAME.     // "Kerbin"
```

That last one is interesting. The computer doesn't just know about the ship — it knows about the *world*. Which means we can do science without leaving the pad.

## First experiment: weighing a planet

Newton's law of universal gravitation says the gravitational acceleration at distance $r$ from a body's center is:

$$g = \frac{GM}{r^2}$$

$G$ is the universal gravitational constant and $M$ is the body's mass. In practice nobody uses them separately — the product $GM$ is what you can actually measure from orbits, and astronomers call it $\mu$ (mu), the **gravitational parameter**. It is the single most important number about any planet you will ever fly near, and kOS will hand it to you:

```
PRINT SHIP:BODY:MU.       // 3.5316E+12
PRINT SHIP:BODY:RADIUS.   // 600000
```

So Kerbin's $\mu$ is 3.5316×10¹² m³/s², and its radius is 600 km. Let's predict the strength of gravity where you're sitting:

$$g = \frac{\mu}{r^2} = \frac{3.5316 \times 10^{12}}{(600{,}000)^2} = \frac{3.5316 \times 10^{12}}{3.6 \times 10^{11}} = 9.81 \ \text{m/s}^2$$

Have the computer check:

```
PRINT SHIP:BODY:MU / SHIP:BODY:RADIUS^2.
```

**9.81 m/s²** — exactly Earth's surface gravity, on a planet one-tenth Earth's size. (Kerbin is implausibly dense; this is a game balance decision with real physical consequences we'll exploit later, like short orbital periods and merciful delta-v budgets.)

Here's the part that should feel like a superpower. You're sitting on a launchpad on Kerbin, but the computer knows *every* body in the system:

```
PRINT BODY("Mun"):MU / BODY("Mun"):RADIUS^2.     // 1.63 m/s²
PRINT BODY("Minmus"):MU / BODY("Minmus"):RADIUS^2. // 0.49 m/s²
```

You just computed the surface gravity of two moons you haven't visited, from a formula written in 1687. Every landing script in this book ultimately rests on that one line of algebra.

## Scripts: writing it down

Typing at the terminal is flying by conversation. A *script* is a flight plan. Create `Ships/Script/hello.ks` in your text editor:

```
// hello.ks — first script
PRINT "Flight computer online.".
PRINT "Body:    " + SHIP:BODY:NAME.
PRINT "Gravity: " + ROUND(SHIP:BODY:MU / SHIP:BODY:RADIUS^2, 2) + " m/s2".
```

Then, at the terminal:

```
RUNPATH("0:/hello.ks").
```

`0:` is the archive — mission control's disk back at the KSC. (Your ship has its own tiny local disk, `1:`, which matters once you fly out of antenna range; we'll get there.)

Two ideas make kOS scripts feel like *flight software* rather than a to-do list:

**`SET` stores a value once.**

```
SET g TO SHIP:BODY:MU / SHIP:BODY:RADIUS^2.
```

**`LOCK` stores a *formula* that re-computes every time it's used** — and, crucially, the ship's controls are lockable:

```
LOCK THROTTLE TO 1.0.
LOCK STEERING TO HEADING(90, 90).   // compass east, pitch straight up
```

A `LOCK` is a little machine that runs continuously. When you lock the steering, you are not setting a value — you are hiring an autopilot and handing it a rule. This distinction between *state* and *behavior* is the heart of every control program we'll write.

## First flight: `liftoff.ks`

Time for engines. Build the classic trainer:

- **OKTO probe core**, with a **Mk16 Parachute** on top
- **Z-200 battery** (dead probes take no data)
- **CX-4181** computer (radially, anywhere)
- **FL-T400 fuel tank**
- **LV-T45 "Swivel"** engine

Check the staging: stage 1 fires the engine, stage 0 the parachute. Then save this as `Ships/Script/liftoff.ks`:

```
// liftoff.ks — first automated flight
// Straight up, engine to depletion, chute on the way down.

PRINT "Flight program loaded.".
LOCK STEERING TO HEADING(90, 90).   // straight up
LOCK THROTTLE TO 1.0.

FROM {local n is 5.} UNTIL n = 0 STEP {set n to n - 1.} DO {
  PRINT "T-" + n.
  WAIT 1.
}

PRINT "Ignition.".
STAGE.                              // light the engine

WAIT UNTIL SHIP:AVAILABLETHRUST = 0.   // fuel exhausted
PRINT "Burnout at " + ROUND(SHIP:ALTITUDE) + " m.".
LOCK THROTTLE TO 0.
UNLOCK STEERING.                    // let it tumble; it's fine

WAIT UNTIL SHIP:VERTICALSPEED < 0.  // wait for the top of the arc
PRINT "Apex: " + ROUND(SHIP:ALTITUDE) + " m. Descending.".

WAIT UNTIL SHIP:ALTITUDE < 5000.
STAGE.                              // parachute
PRINT "Chute deployed.".

WAIT UNTIL SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED".
PRINT "Touchdown. Vertical speed at landing was survivable, probably.".
```

Run it:

```
RUNPATH("0:/liftoff.ks").
```

And now — this is the important part — **don't touch anything.** Watch. The countdown counts, the engine lights, the rocket climbs, and at some point you will feel the itch to *do something*. That itch is the pilot in you. Let the program fly.

What you're watching is a chain of `WAIT UNTIL` statements: the program declares the *conditions* that define each phase of flight — burnout, apex, deployment altitude — and the computer watches for them far more attentively than you would. Congratulations: you have written an event-driven flight program, which is structurally the same thing running on every launch vehicle currently flying.

### When it goes wrong

Something will eventually go wrong — a forgotten period, a staging mishap, a chute deployed at Mach 1. Good. **Every failure in this book is data.** Real flight test works exactly this way, minus the quickload. Read the error, form a hypothesis, change one thing, fly again.

## In the real world

The Apollo Guidance Computer's software was woven — literally, by hand, wires threaded through magnetic cores — under the direction of Margaret Hamilton at MIT. During the Apollo 11 landing it threw the famous 1202 alarm: it was being asked to do more work per cycle than it had cycles. Because Hamilton's team had designed it to shed low-priority tasks and *keep flying*, the landing continued. Your `liftoff.ks` has no such grace under pressure yet — but "keep flying the vehicle" is a design principle we will return to, and by the landing chapters your programs will handle surprises too.

## Exercises

1. **Flight recorder (preview of Chapter 2).** Add a loop that prints altitude and speed once per second during ascent. Hint: `UNTIL SHIP:VERTICALSPEED < 0 { PRINT ... . WAIT 1. }` — but think about where in the script it has to go.
2. **Gravity survey.** Write `gravity.ks` that prints the surface gravity of *every* body orbiting Kerbol. Hint: `LIST BODIES IN bs.` gives you a list to loop over with `FOR b IN bs { ... }`.
3. **Altitude-targeted shutdown.** Modify `liftoff.ks` to shut the engine down (`LOCK THROTTLE TO 0.`) when `SHIP:APOAPSIS` exceeds 20,000 m instead of running to depletion. Your rocket now has a *guidance target* rather than just an appetite. How repeatable is the final apex across three flights? (Write the three numbers down. That's engineering.)
4. **Thought experiment.** Kerbin has Earth's surface gravity at one-tenth the radius. Using $g = \mu/r^2$, how does Kerbin's $\mu$ compare to Earth's ($3.986 \times 10^{14}$)? What does that ratio suggest about how much easier it is to leave?

## What's next

The ship told us its altitude and speed, and we took the numbers at face value. But *which* speed? Speed relative to what? In [Chapter 2 — Reading the Instruments](Chapter-02-Reading-the-Instruments.md) we build the first real piece of our library — a telemetry recorder — and discover that "how fast am I going" is a surprisingly deep question whose answer depends on who's asking.

---

*[Home](Home.md) · Next: Chapter 2 — Reading the Instruments*
