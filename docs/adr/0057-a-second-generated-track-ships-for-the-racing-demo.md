# ADR-0057: A second generated track ships, sized for racing rather than CI

- **Status:** Proposed
- **Date recorded:** 2026-08-20
- **Requirements:** the P1 track deliverables and the P3 racing demos. No SRS
  ID is cited: `docs/spec` is not present in this checkout.
- **Related:** [ADR-0035](0035-track-geometry-is-converted-never-redistributed.md),
  which this supersedes in part;
  [ADR-0034](0034-a-track-is-geometry-plus-a-declared-surface.md),
  [ADR-0043](0043-contact-is-one-impulse-between-declared-footprints.md)

## Context

ADR-0035 decided that SlipX redistributes no third-party track geometry and
that "one small track that we generate ships in the tree". The stadium that
ships under that record was shaped for machinery, and its own generator says
so: a lap counter needs it closed, a pure pursuit controller needs curvature
that changes, a wall follower needs a straight to settle on. Two identical
corners and a 34.85 m lap deliver exactly that and nothing more.

The bridge now runs fields that bump wheels: footprints appear in each
other's scans (ADR-0045) and contact is real (ADR-0043, ADR-0055). Watching
twenty cars on the stadium is watching a queue: a twenty-car grid at 1.5 m
spacing occupies 28.5 m of a 34.85 m lap, the field never spreads, and both
corners are the same corner, so a driver that can take one can take the
other and nobody earns a pass. The demo the racing layer deserves needs a
track with unequal corners, a braking zone worth attacking into and room for
a field to string out. The stadium cannot be stretched into that without
breaking every suite that asserts its exact dimensions.

Options considered:

**Run the demo on a converted real track.** Rejected. The demo must work
from a fresh clone with no network step, which is the property ADR-0035
bought deliberately; a demo whose first instruction is "fetch somebody
else's geometry" gives that property back.

**Grow the stadium.** Rejected. Its dimensions are load-bearing fixtures in
the C++ and Python suites and in the published performance numbers, and a
bigger stadium is still two identical corners.

**Ship a second generated track.** Taken. The substance of ADR-0035 was
never the count; it was that no third-party geometry enters the tree and
that the shipped, tested path stays network-free. Both are per-track
invariants, and a second track from the same generator, with the same
asserted geometry and the same provenance honesty, preserves them.

## Decision

A second generated track, `paddock_gp`, ships beside the stadium, produced
by the same `examples/tracks/make_tracks.py` and held to the same standard:
standard library only, every dimension exact, every property of the output
asserted by the generator rather than eyeballed, provenance stating plainly
that it is generated and has no real-world counterpart.

The circuit is shaped by what a race needs rather than what the machinery
needs: a long start straight, a fast sweeper, a bus-stop chicane that pinches
to single file, a hairpin behind a wide braking zone, and a return elbow,
about 101 m a lap. It is the first shipped track whose width varies along
the lap, which exercises the per-point width columns that the stadium's
constant 0.75 m never has.

The stadium remains what CI and the test suites assert against; nothing
migrates. The circuit is what the racing demos open by default.

## Consequences

The "one small track" sentence of ADR-0035 is superseded by this record.
The converter half of that decision, and every word of its licensing
reasoning, stands untouched.

Two tracks cost more than twice one: the generator now carries a general
segment walk beside the stadium's bespoke construction, and a reader must
know which track a number belongs to. The generator asserts both, so a
regression in either fails at generation time rather than at load time.

The circuit is a fixture nothing in the C++ suites asserts against. A
loader bug that only varying width triggers would be caught by the
generator's own checks and by the Python tests that load the shipped
directory, not by ctest. That gap is accepted rather than closed, because
closing it means a second set of dimension fixtures to keep true.

Reversing this record means deleting a directory and a generator function;
nothing else consumes the circuit by name except the demos that exist for
it.
