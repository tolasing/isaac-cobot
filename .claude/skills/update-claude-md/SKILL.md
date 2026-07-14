---
description: Conventions for editing CLAUDE.md in this repo. Use whenever you're about to document a finding, fix, or new open issue there — the user has corrected bloat in this file before and wants it kept lean.
---

# Keeping CLAUDE.md lean

CLAUDE.md's own header states the goal directly: "Kept to the essentials
for working in the sim day-to-day — full history, forensic detail, and
'why' narratives live in `docs/`... rather than here." The user has
explicitly asked for trims when this slipped ("claude.md looks bloated.
trim it" → follow-up: "add 10 lines max").

## Where a new finding goes

- **Section structure to match**: `What this repo is` → `Where things
  live` → active-script status (`Active script + current state` for
  mefron/Franka, or the equivalent for whatever branch you're on) →
  `Currently open issues` (bullet list) → branch-specific validation
  sections (e.g. `CR5 validation (dobot branch)`) → `Must-know gotchas`
  (cross-cutting, not scene-specific) → `Pinned versions`.
- A **root-caused bug with a real fix** goes as a numbered item in the
  relevant validation section (continue the existing numbering, don't
  restart), or a bullet in `Currently open issues` if still unresolved.
- A **repo-wide gotcha that would bite any future script**, not specific
  to one asset/branch, goes in `Must-know gotchas`.
- **Forensic detail, full bug-hunt narratives, abandoned approaches,
  screenshots-worth-of-diagnosis** — put in `docs/mefron-history.md` (or
  create a new `docs/*.md`) and link it from CLAUDE.md with one line, not
  inline.

## How much to write

- Default budget: **~10 lines per finding**, tight enough that CLAUDE.md
  stays skimmable top to bottom in one sitting.
- Prefer editing an existing bullet/paragraph over appending a new one if
  the finding updates something already documented (mark it **RESOLVED**
  rather than leaving both the old "still open" text and a new "fixed"
  note both present).
- Cut, don't just add: if a fix makes an earlier caveat obsolete, remove
  the caveat rather than leaving stale text alongside the update — a
  reader shouldn't have to figure out which of two contradicting
  paragraphs is current.
- State the fix and the *why* in one pass, not "here's what I tried" —
  CLAUDE.md is not a changelog. Save the "what I tried and ruled out"
  narrative for `docs/` if it's worth keeping at all.

## Before committing an edit

- Re-read the surrounding section after editing — confirm it still reads
  coherently in context and that you haven't left a dangling reference to
  something you just renamed/removed elsewhere in the file.
- `grep` the repo for any other tracked file (scripts, other docs, yaml
  configs) referencing a path/name you just changed in CLAUDE.md, so the
  doc and the code don't drift apart.
- Double check any factual claim about a specific file/function/constant
  by actually reading that file — don't state what a config or script
  does from memory of an earlier turn in the conversation.
