# ADR-0033: The C library is part of the build a reference hash is keyed by

- **Status:** Accepted
- **Date recorded:** 2026-08-13
- **Requirements:** NFR-02, NFR-03
- **Supersedes:** [ADR-0008](0008-reference-hashes-are-keyed-by-build.md)
- **Related:** [ADR-0007](0007-determinism-is-scoped-to-a-build.md),
  [ADR-0020](0020-wheels-assert-nothing-about-their-hash.md)

## Context

[ADR-0008](0008-reference-hashes-are-keyed-by-build.md) keys
`conformance/reference_hashes.tsv` by `(system_processor, compiler_id,
compiler_major, build_type, tier, integrator)` and gives the lookup three
outcomes: hashes agree, hashes differ (a determinism bug), or no matching row
(nothing was claimed, exit 0).

That key has no C library column, and it needs one. The trajectory hash tracks
`libm`, not the compiler. `sin`, `cos`, `atan` and `exp` are not correctly
rounded, glibc's implementations change between versions, and none of that is
visible in any field the key records. The evidence is not hypothetical: one
wheel, unchanged, byte for byte the same binary, produced two different
trajectory hashes on glibc 2.28 and glibc 2.39. Every other column of the key
was identical across those two runs.

So the key can be satisfied by two runs that were never entitled to agree, and
the checker reports the disagreement as case two: `MISMATCH`, exit 1, with a
message saying something changed the numerical result and pointing at physics,
the integrator, the scenario or the state layout. All four are wrong. A
contributor on an older distribution is told they have a determinism bug in
code that is behaving exactly as designed, and the only clue is a sentence in
the TSV header they have no reason to be reading.

The same gap makes `"within_build": "bit-identical"` in every run manifest an
overstatement. For a build compiled and run in one place it is true. For a
redistributed dynamically linked wheel, which is what SlipX ships, "the build"
is not one thing: the binary is fixed at build time and the C library is not
chosen until the wheel is installed on somebody's machine.

Three options were considered.

**Record the C library in the manifest, but leave the key alone.** Cheap, and
it puts the diagnostic where somebody debugging can find it. It does not fix
the false failure: the lookup still matches a row it should not match, and the
checker still exits 1 with the wrong explanation. A field nobody is required to
read does not stop a wrong answer being printed.

**Reword only.** Fix `within_build` to say what a build includes and leave
everything else. This is the cheapest and it records nothing machine-readable,
so the checker keeps mis-attributing, and the wording change is a documentation
fix applied to what is really a key defect.

**Put the C library in the key.**

## Decision

The C library is part of the build a reference hash is keyed by.

`conformance/reference_hashes.tsv` gains a `libc` column between
`compiler_major` and `build_type`, so the key is `(system_processor,
compiler_id, compiler_major, libc, build_type, tier, integrator)`. The run
manifest gains `libc_id` and `libc_version` in its `build` block, and both feed
the configuration digest. `tools/check_conformance.py` composes the column from
the two manifest fields as `id-version`, or `id` alone where no version is
available.

The identity is read at **run** time, not configure time. `gnu_get_libc_version()`
is what the process is actually linked against; a value baked in by CMake would
describe the machine that compiled the wheel, which is precisely the machine
whose C library did not matter. Where the platform offers no runtime version
(Apple's libc, the Windows UCRT, musl) the id is recorded and the version is
empty, because an invented version is worse than an absent one and the id alone
is still enough to keep two platforms in different rows.

glibc is keyed on its full `2.39`, not a major, unlike the compiler. glibc's
major has been 2 since 1997, so a major-only key would be a column of constants.
The compiler argument for majors was that point releases do not move results;
for glibc, minor releases demonstrably do, which is the whole reason for this
record.

The eighteen existing rows are re-keyed, not re-measured into new values: every
hash in the file is unchanged and gains `glibc-2.39`, the C library they were
in fact recorded under, on this machine and on the pinned `ubuntu-24.04`
runners alike. This is not a hash movement and no published result becomes
incomparable. Anyone who compared against these numbers compared against them
on glibc 2.39 or got a mismatch they could not explain.

## Consequences

A contributor on a different glibc now gets the honest third outcome: no
matching row, exit 0, and the row printed for them to add. That is the
behaviour [ADR-0008](0008-reference-hashes-are-keyed-by-build.md) always
intended for a build nobody has recorded, and it was reachable for a different
architecture or compiler but not for a different C library.

The file grows a row set per C library anyone cares to publish, on top of one
per platform. The rows are cheap and each is a claim someone can check, but the
combinatorics are now four dimensions rather than three, and the honest reading
is that the file records the builds we have measured rather than the builds
that exist.

The `wheel` job in `ci.yml` compares an installed wheel against a literal
hash, and that comparison is only valid because the job pins `ubuntu-24.04`
and therefore pins glibc. That was already true and was already relied upon;
it is now written down at the pin. The same job on a runner image with a
different glibc would fail for a reason that has nothing to do with the wheel.
Nothing about [ADR-0020](0020-wheels-assert-nothing-about-their-hash.md)
changes: a released wheel still asserts nothing about its own hash, and this
record is part of why it cannot.

Configuration digests move. Two runs on different C libraries now have
different digests, which is the point, and a digest recorded before this change
will not match one recorded after even for an otherwise identical setup. No
digest is published anywhere, and none is pinned in a test; they are compared
against each other, never against a literal.

The manifest says `"same binary and same C library"` where it used to say
`"bit-identical"`. That is a longer sentence and a narrower promise, and it is
the promise that was actually being kept. The manifest strings also stop citing
requirement IDs, which a reader of the published repository cannot follow.

Reversing this means deciding that libm agreement across C library versions is
close enough to key on, which the measured two-hash result says it is not.
