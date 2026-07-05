---
name: schuyler-docs
description: Use when writing or revising any reader-facing prose for this book — chapters in wiki/, the front page, appendices, exercises, sidebars. Encodes Schuyler's writing voice — direct, conversational, honest about limitations, dry humor that never crowds out technical content — adapted for discursive tutorial prose from the README-oriented original. Read BOOK-PLAN.md's "Drafting: process and voice" section first; it governs how prose gets generated, while this skill governs how it should sound. Do NOT use for code comments or commit messages.
---

# Writing Book Prose in Schuyler's Style

Vendored from Schuyler's global `schuyler-docs` skill and adapted for this project:
the original targets READMEs and reference documentation; this book is discursive
tutorial prose. The README section patterns, length tables, and reference-example
corpus are dropped — chapter structure and the drafting *process* (the
answer-don't-author protocol, the diagnostics) live in `BOOK-PLAN.md` and are not
duplicated here. What remains is the voice.

## Core Principles

These are non-negotiable. Every piece of reader-facing prose must follow them.

### 1. The first sentence does work

Open with something that is already teaching — a scene, a number, a claim the
chapter will earn. Never with throat-clearing about the prose itself.

Good:
- "You have flown to the Mun on instinct."
- "Somewhere around 12 km, the acceleration curve bends upward even though the throttle never moved."

Bad:
- "In this chapter, we will explore the fundamentals of..."
- "Before we begin, it's worth taking a moment to..."
- "Orbital mechanics is a fascinating subject that..."

### 2. Length tracks complexity

A point that takes a paragraph gets a paragraph. Never pad a section to feel
chapter-sized, and never compress a real derivation to feel brisk. Cut anything
that doesn't earn its place.

### 3. Show, don't describe

The concrete thing comes early and the abstraction is named after it's visible: a
flight, a log excerpt, a number the reader can check at the terminal, a script
that runs. Code examples are working code — the same code in `lib/` and
`missions/` — never toy fragments that exist only on the page.

### 4. Be honest

- If an approximation is in play, say so and quantify: "the impulsive-burn
  approximation is good to a few m/s here; we'll measure how wrong it gets."
- If the game's physics diverges from the real world, say which one you're
  teaching at that moment.
- If a technique is a simplification the reader will outgrow, say what breaks
  and when.
- Credit sources, and point to deeper or better treatments where they exist
  (Braeunig, Bate/Mueller/White) — recommending someone else's better material
  is a signature trait, not a weakness.
- If AI helped write it, say so. For this book that disclosure lives once, in
  the front matter — where a reader would look for it — not on every page.

### 5. No marketing language

Never: "powerful", "robust", "seamless", "cutting-edge", "next-generation",
"innovative". No mission statements about the book. No aspirational language —
the SSTO is hard, and the prose says *how* it's hard, not how exciting it is.

### 6. Conversational register

Write like explaining to a sharp friend across the table — which is literally how
the prose gets generated (see BOOK-PLAN.md). Colloquialisms are fine when
natural: "a launch system with nowhere to go is a firework"; "your rocket now has
a guidance target rather than just an appetite." But don't force it. Derivations
and procedures are clear and precise; the humor is incidental, not the point.

### 7. No boilerplate

Skip: tables of contents inside chapters, summaries that restate the headings,
"in this section we saw" recaps, learning-objective lists. Chapter-ending
structure is fixed by BOOK-PLAN.md (exercises, "What's next" hook) — beyond
that, include only prose that carries information.

## Anti-Patterns to Avoid

1. **Sycophancy toward the reader** — no "Congratulations, you've mastered...",
   no cheerleading. Respect the reader by handing them the next hard thing.
   (One "Congratulations" per book is plenty, and chapter 1 already spent it —
   on a claim, not a compliment.)
2. **Filler sections** — if a section has nothing to say, delete it.
3. **Premature formality** — this is not a textbook with a review board; don't
   write like one. But the math is real, so don't write like a hype thread
   either.
4. **Emoji in prose** — not part of this style.
5. **Over-explaining** — the reader is game-literate and at the keyboard; trust
   them to run the code and read an error message.
6. **Template voice** — if any sentence sounds like it came from a tutorial
   template ("Now that we've covered X, let's turn to Y"), rewrite it as
   something you would actually say.

## Process

Prose generation is governed by BOOK-PLAN.md's "Drafting: process and voice" —
the answer-don't-author protocol, the say-aloud and structure passes, the clunk
diagnostics, and the person rule (*you* at the keyboard, *we* at the whiteboard).
This skill is the answer to "what should it sound like"; that section is the
answer to "how do we get it to sound that way." Use both: read the previous
chapter for the voice, BOOK-PLAN.md for the process, and this skill as the check
when reading a draft back.
