# ADR-0001: Record architecture decisions in this directory

- **Status:** Accepted
- **Date recorded:** 2026-08-02
- **Requirements:** none directly
- **Related:** all of them

## Context

SlipX accumulated a set of decisions during P0 that constrain almost every
change made afterwards: no Eigen, a strictly downward dependency graph, a
`const` `step`, NaN rather than zero in diagnostics, reference hashes keyed by
build. Each was reasoned about carefully. None of the reasoning was written
anywhere a contributor could read it.

It lived in three places, and all three are the wrong place:

- `CLAUDE.local.md`, which is gitignored and not checked in.
- `docs/spec/SRS.md`, which is also gitignored, so the authoritative
  specification is invisible to readers of the published repository.
- Code comments, which are excellent for the line they sit above and hopeless
  for a decision that spans a build system, a licence file and a lint script.

The failure mode is specific and has already nearly happened once. A decision
whose reasoning is not written down does not get reversed by an argument. It
gets reversed by somebody making a small, locally sensible change, because
nothing told them there was anything to argue with. "Temporarily make L2 fall
back to L1" is a reasonable-sounding sentence to anybody who has not read why
it is not.

The alternative considered was a single `DECISIONS.md`. It was rejected for the
same reason a changelog is not a git log: one file grows until nobody reads it,
edits to it lose the previous reasoning, and there is no natural place to
record that a decision was superseded rather than deleted.

## Decision

Architectural decisions are recorded as numbered ADRs in `docs/adr`, in
Michael Nygard's format with a `Requirements` field added for SRS IDs.

Records are immutable. A decision that changes is superseded by a new record;
the old one keeps its text and gains a `Superseded by` status. The reasoning at
the time is the artefact, including reasoning that turned out to be wrong,
because "we considered that and here is what we thought" is more useful to the
next person than a document that has always agreed with the present.

`docs/adr` is not gitignored. That is the point of choosing it over the two
existing homes.

## Consequences

A decision now costs a file. That is the intended friction: if a change is
worth constraining future work, it is worth twenty minutes of writing, and if
nobody can be bothered to write the record, that is evidence about how load
bearing the decision really was.

`CLAUDE.local.md` and the SRS stop being the authority on decisions and become
pointers to this directory. Where they overlap, the ADR is authoritative for
*why* and the SRS remains authoritative for *what is required*. Keeping three
documents consistent was already a cost; this reduces it to keeping one
summary line consistent per decision.

The existing P0 decisions were written up after the fact, from
`CLAUDE.local.md`, the SRS and the code comments. They are accurate about the
reasoning and are not contemporaneous. Anything written from here on is.
