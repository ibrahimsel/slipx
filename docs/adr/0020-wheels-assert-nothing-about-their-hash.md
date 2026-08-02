# ADR-0020: A released wheel asserts nothing about its own trajectory hash

- **Status:** Accepted
- **Date recorded:** 2026-08-02
- **Requirements:** NFR-02, NFR-03
- **Related:** [ADR-0007](0007-determinism-is-scoped-to-a-build.md),
  [ADR-0008](0008-reference-hashes-are-keyed-by-build.md),
  [ADR-0018](0018-wheel-coverage.md)

## Context

The P0 exit gate is a sentence about a person: a third party can `pip install`
the package, load a car directory, integrate a step-steer manoeuvre and get the
same trajectory hash as CI. Publishing wheels makes that sentence achievable for
the first time, and immediately raises the question of what each wheel should
check about itself.

The published rows in `conformance/reference_hashes.tsv` are keyed by
architecture, compiler major and build type
([ADR-0008](0008-reference-hashes-are-keyed-by-build.md)). None of them
describes a manylinux, macOS or Windows wheel-building image.

Three options were considered.

**Assert the published hash in every wheel.** Wrong on the face of it: NFR-03
does not promise these builds match, so the assertion would fail on platforms
where nothing was ever claimed, and the pressure would be to weaken it to a
tolerance.

**Publish reference rows for the wheel-building images.** Superficially the
tidy answer, and it is the one already learned against. A cibuildwheel image's
compiler version moves whenever the image is rebuilt, which changes the lookup
key. This is exactly the `ubuntu-latest` failure from
[ADR-0008](0008-reference-hashes-are-keyed-by-build.md), except on an image
nobody here controls at all. It would mean publishing claims whose validity
depends on a third party not updating a container.

**Assert nothing, and verify what does hold everywhere.**

## Decision

A released wheel makes no claim about its trajectory hash.

cibuildwheel's `test-command` runs `tools/exit_gate.py` **without** `--expect`.
That verifies the three clauses of the exit gate that hold on every platform:
the wheel installs, the reference car loads out of it, and a step steer
integrates to a finite trajectory. The hash is printed for the record and is not
graded.

The only build compared against a published row remains the `wheel` job in
`ci.yml`, which runs on a pinned image with a pinned compiler for exactly the
same reason the determinism job does.

The README states the consequence directly: `slipx-conformance` printing
something other than `d44a9a68616ec899` on your machine is the expected outcome
unless you are on x86-64 with one of the compilers in that file, and that is the
promise working rather than failing.

## Consequences

The P0 exit gate closes only for a third party on a build with a published row.
Everyone else can report that it installed, ran and produced a finite number,
which is worth having and is explicitly not the gate. The README says which is
which.

The release workflow surfaces the per-platform hashes as evidence. If a
manylinux hash is ever to become a published row, that is a separate deliberate
commit with a reason in the message, exactly as the header of the TSV requires
of any new row. A release job does not get to publish a claim on its own.

This is the fourth place the scope of the determinism promise has to be restated
(README, reference file, `check_conformance.py`, cibuildwheel config). That
repetition is the cost of a promise whose most likely failure is a user assuming
the stronger version.
