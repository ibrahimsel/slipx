# ADR-0034: A track is geometry plus a declared surface, and friction stays in the tyre file

- **Status:** Proposed
- **Date recorded:** 2026-08-15 (decision taken during P1)
- **Requirements:** the P1 track deliverables. No SRS ID is cited: `docs/spec`
  is not present in this checkout, and a guessed ID is worse than none.
- **Related:** [ADR-0003](0003-dependencies-point-downward.md),
  [ADR-0009](0009-mf-lite-over-full-pacejka.md),
  [ADR-0010](0010-tyres-are-compound-surface-pairs.md),
  [ADR-0013](0013-provenance-labels-are-printed.md),
  [ADR-0025](0025-c-kappa-enters-the-core-ahead-of-the-schema.md)

## Context

`slipx_scene` is the first new component since the core, and the roadmap
describes its first slice as loading a centreline CSV with the columns
`s, x, y, w_left, w_right, banking, mu`, TUM racetrack database compatible,
with a per-segment surface identifier resolving the tyre `(compound, surface)`
pair, and refusing to run when the declared surface has no matching tyre.

Two things in that sentence do not fit together, and both are worth settling
before any code exists rather than after somebody has written a parser around
them.

**The format does not have those columns.** The TUM racetrack database and the
F1TENTH set derived from it carry four columns, `x_m, y_m, w_tr_right_m,
w_tr_left_m`, with the column names in a leading comment line. There is no
arc length, no banking and no surface. "TUM compatible" and "seven columns"
are two different files. A loader that accepts the four-column form and
invents the other three is exactly the silent defaulting ADR-0025 forbids,
and it would invent the three that matter most: banking and friction are not
geometry, and a wrong one is not visibly wrong.

**A `mu` column contradicts ADR-0010.** Peak friction already exists, per
`(compound, surface)` pair, in a tyre file, as `mu_y0` and `mu_x0` with a load
sensitivity exponent beside them, and it is the parameter the whole
identification programme is built to measure. A friction number in the track
file is a second source for the same quantity, arriving from a track author
who measured nothing, and whichever of the two wins, the other is silently
discarded. The failure is not hypothetical: it is the same one ADR-0010 was
written to prevent, with the venue in the track file instead of the car file.

Options considered:

**Take the seven-column format literally and consume `mu`.** Rejected. It
makes the track the authority on friction, which puts an unidentified number
in front of an identified one and makes a fitted tyre file's most valuable
parameter inert whenever a track is loaded.

**Take it literally but treat `mu` as a multiplier on the tyre's friction.**
Tempting, and it is what a damp patch or a dusty line physically is. Rejected
for the first slice, not forever: a multiplier is still a number nobody
measured, and it has no identification manoeuvre behind it. It can be added
later as a named local-conditions factor whose provenance is stated, which is
a different thing from a column called `mu`.

**Extend the CSV with our own columns.** Rejected. A file that is TUM
compatible until it is not is the worst of both: existing files fail to load
for reasons that read as bugs, and our files are rejected by every other tool
in the ecosystem.

## Decision

A track is two files. Geometry is a centreline CSV in the four-column TUM
form, unmodified and unextended. Everything that is not geometry lives beside
it in a track manifest, which names the track, states the provenance of its
geometry, and declares the surface.

The manifest declares a **surface identifier**, a string such as `carpet` or
`asphalt`, and never a friction coefficient. Friction is resolved by pairing
that identifier with the car's tyre compound, exactly as ADR-0010 already
requires, and it comes from the tyre file or the run does not start.

`slipx_scene` refuses to build a scene when a declared surface has no matching
tyre entry, and the refusal names the surface, the compounds it was offered
and the file the manifest came from, in the style ADR-0025 established for the
loader. The check runs at construction, once, not per step.

The refusal lives in `slipx_scene` and not above it, and it works on a list of
available `(compound, surface)` pairs supplied by the caller. `slipx_scene`
therefore learns nothing about YAML, about `slipx_schema` or about how a car
directory is laid out, and the dependency direction of ADR-0003 is preserved
without the check migrating upward into whatever happens to be holding both
objects at the time.

Arc length is derived, not read. `s` is the cumulative chord length along the
centreline, computed in file order from the first point, because file order is
the only ordering the format guarantees and a derived quantity cannot
disagree with the geometry it was derived from. Banking is absent from the
first slice entirely: a banking of zero is a claim about the track, and
declaring one we did not measure would be the defaulting this record was
written to avoid. A manifest that wants banking gets a refusal naming it until
the tier that consumes it exists.

## Consequences

A track directory is not a single file, which is a real cost in
convenience. It buys the same thing ADR-0010 bought: the surface is a
first-class, visible part of an experiment's description, and it cannot be
carried into a new venue by copying a directory.

Every existing TUM and F1TENTH centreline loads unchanged, and every one of
them needs a manifest written for it before it will run. That is the intended
friction. Somebody has to state what surface the track is, and there is no
value they can supply by accident.

The first slice cannot represent a track whose surface changes along its
length, even though the manifest is the place that would express it. Per
segment surfaces are the obvious second slice and the manifest format should
be shaped so that adding them is not a breaking change: the first slice
declares one surface for the whole track, in a field that later accepts a
list of ranges.

Numeric parsing is a determinism surface, and this is the first component in
the tree that parses floating point at all. `std::stod` and the default
`istringstream` are locale sensitive, so a machine with a comma decimal
separator would read `1.5` as `1`, silently, and produce a plausible wrong
track rather than an error. Parsing in `slipx_scene` imbues the classic
locale explicitly and the test suite asserts a value that a comma locale
would misread. This is a note against a real failure, not a theoretical one:
it costs one line and it is invisible until CI runs somewhere else.

Reversing this record means deciding that a track may state friction. That is
a decision about where an identified parameter lives, so it supersedes this
record and ADR-0010 together, and it should not be taken because a file format
happened to have a column.
