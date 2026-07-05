# Chapter 2 — Reading the Instruments

**Stub** — planned, not yet drafted. Part I (Ground School), between
[Chapter 1 — The Flight Computer](Chapter-01-The-Flight-Computer.md) and
[Chapter 3 — The Tyranny of the Rocket Equation](Chapter-03-The-Tyranny-of-the-Rocket-Equation.md).

**Scope:** position, velocity, and acceleration as vectors; reference frames (why "surface
speed" and "orbital speed" disagree, even on the pad); the atmosphere as a function of
altitude. Code: `lib/telemetry.ks` — the first real library file — logging flight data to
files and graphing it. Mission: an instrumented sounding-rocket flight whose data Chapter 3
will mine.

**Reader's question** *(provisional)*: "My flight printed its numbers to the screen and they
scrolled away. How do I make the rocket write down what actually happened — and which of
those two speeds is the real one?"

**Likely follow-ups** *(candidates — one fills the addressee slot per section at draft time;
not required coverage)*:

- "Chapter 1 left me hanging: why do surface speed and orbital speed disagree while the
  rocket is sitting still on the pad?"
- "Velocity is three numbers now, not one — what do the components mean, and which way is
  'up'?"
- "How do I get the log file out of the game and into a graph?"
- "The air gets thinner as I climb — can I see that in my own data?"
- "How often should I log? Every tick? Once a second?"

**Deferred here:** Chapter 1's exercise 1 (the flight-recorder loop) graduates into
`lib/telemetry.ks`. The book's "measure, don't assume" habit starts on this page — every
later chapter is scored against flight data logged with this library, beginning with
Chapter 3's measurement of the rocket equation.
