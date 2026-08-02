# ADR-0008: Reference trajectory hashes are keyed by build, and the reference runners are pinned

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0; the pinning was
  added after CI failed)
- **Requirements:** NFR-02, NFR-03
- **Related:** [ADR-0007](0007-determinism-is-scoped-to-a-build.md),
  [ADR-0020](0020-wheels-assert-nothing-about-their-hash.md)

## Context

[ADR-0007](0007-determinism-is-scoped-to-a-build.md) scopes the determinism
promise to one build. That makes the obvious implementation wrong.

A single pinned hash in a unit test would pass on the machine it was recorded
on and fail on every other. The only way to make it pass everywhere is to
weaken it to a tolerance, which is exactly the weakening the whole design is
arranged to avoid.

## Decision

`conformance/reference_hashes.tsv` is keyed by `(system_processor, compiler_id,
compiler_major, build_type, tier, integrator)`. A run is compared against the
row matching its own build, and:

| Case | Meaning |
|---|---|
| matching row, hashes agree | the promise held |
| matching row, hashes differ | NFR-02 is broken; this is a bug |
| no matching row | nothing was claimed; print the row to add and exit 0 |

The third case is the honest outcome for a contributor on an unrecorded
machine, and `--require-row` turns it into a failure on the designated
reference runners, where a missing row means the file needs updating.

Compiler **major** only. A point release is not supposed to change
floating-point results and in practice does not; requiring an exact match would
mean a new row every time a CI image updated, and a check nobody maintains is a
check nobody believes.

The determinism job is pinned to `ubuntu-24.04` and `g++-13`. This was learned
rather than designed: the job ran on `ubuntu-latest`, GitHub rotated the image
from GCC 11 to GCC 13, the lookup key changed, and the job failed under
`--require-row` for a reason with nothing to do with determinism. Every other
job stays on `ubuntu-latest` so a new toolchain is still exercised, just not by
the job whose purpose is to hold one build fixed.

Rows for compilers that agree are kept separate rather than collapsed. The
moment they are collapsed the file stops being able to express a compiler that
disagrees, and NFR-03 exists precisely because one eventually will.

## Consequences

Moving the pins is a deliberate act requiring new reference rows recorded in the
same commit. That is the intended cost, and it is written at the top of the TSV.

Changing an existing hash invalidates every result anybody has compared against
it. It is a release event, it belongs in the changelog, and it needs a reason in
the commit message: a physics change, an integrator change, a change to the
conformance scenario, or a change to the state layout. "The test was failing" is
not a reason.

The file will grow one row set per platform anyone cares to publish. That is
fine; it is a small file and each row is a claim someone can check.

The hashes certify that the arithmetic is reproducible. They certify nothing
about a real car: the conformance scenario uses the `VehicleParams` defaults,
which are provisional
([ADR-0013](0013-provenance-labels-are-printed.md)).
