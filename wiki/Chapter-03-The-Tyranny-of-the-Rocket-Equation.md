# Chapter 3 — The Tyranny of the Rocket Equation

**Stub** — planned, not yet drafted. Part I (Ground School), between
[Chapter 2 — Reading the Instruments](Chapter-02-Reading-the-Instruments.md) and Part II
(Getting to Orbit).

**Scope:** momentum, mass ratio, specific impulse; Tsiolkovsky's equation derived — then
*measured* from the reader's own sounding-rocket telemetry. Code: `lib/rocket.ks` — TWR,
stage delta-v, burn durations. Plants the seed of the book's central problem: why a
single-stage spaceplane shouldn't be possible.

**Reader's question** *(provisional)*: "I can log a whole flight now. My rocket burned out
at 18 km — if I want to double that, do I just double the fuel?"

**Likely follow-ups** *(candidates — one fills the addressee slot per section at draft time;
not required coverage)*:

- "What is specific impulse actually measuring — and why is it in seconds?"
- "Where does the logarithm come from, and what does it do to my fuel budget?"
- "Can I measure my engine's Isp and stage delta-v from my own telemetry — and will it match
  the VAB readout?"
- "Why does dropping an empty tank beat carrying one big one?"
- "So why exactly shouldn't a single-stage spaceplane be possible — and what would the
  loophole be?"

**Deferred here:** the SSTO problem is planted on this page; Parts V–VI collect. The
burn-duration math in `lib/rocket.ks` (including the "confirm this math" TBD inherited from
the reference scripts — it becomes an exercise) is what Chapter 6's node execution leans on
for burn timing.
