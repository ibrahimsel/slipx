# ADR-0007: Determinism is promised within one build and not across platforms

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0)
- **Requirements:** NFR-02, NFR-03
- **Related:** [ADR-0004](0004-step-is-const-and-stateless.md),
  [ADR-0008](0008-reference-hashes-are-keyed-by-build.md),
  [ADR-0020](0020-wheels-assert-nothing-about-their-hash.md)

## Context

Three of the intended uses rest on reproducibility. A competition leaderboard
has to survive an appeal. A replay has to be the run it claims to be. A graded
assignment has to produce the same answer on the marker's machine as on the
student's.

The maximal promise, bit-identical results everywhere, cannot be kept. `libm`
is not correctly rounded, and `sin`, `cos`, `exp` and `atan2` differ in the last
bits between platforms, between libc versions, and sometimes between
optimisation levels within one compiler. A project that promises cross-platform
bit-identity is promising something it does not control, and the pressure to
keep the promise turns into a tolerance, which is where nondeterminism hides.

The minimal promise, "results are approximately reproducible", is worthless for
all three uses.

## Decision

Bit-identity is promised within one fixed (platform, compiler, flag set) triple
and is explicitly **not** promised across them. Across platforms, agreement is
asserted to a stated and published tolerance on x86-64 and aarch64.

What makes the in-build promise achievable is set explicitly rather than
assumed, because each of these defaults differently between compilers:

- `-ffp-contract=off`. Fusing `a*b+c` into an FMA is not rounding neutral, and
  whether it happens depends on optimisation level and on which expressions the
  vectoriser reached first. GCC defaults this to `fast` for C++.
- never `-ffast-math` or `-funsafe-math-optimizations`, in any configuration,
  including release.
- never `-march=native` or `-mtune=native`, which make the binary depend on the
  machine that compiled it.
- fixed reduction order, no multithreading inside the integrator.

`cmake/SlipxDeterminism.cmake` fails configuration if a forbidden flag is
present, rather than producing a build whose results cannot be compared.

Every run writes a manifest hashing schema versions, parameter files, seeds,
integrator, git SHA, compiler ID and flags, so a replay cannot compare equal to
a run it does not match.

## Consequences

The scope of the promise has to be restated every time a hash is mentioned, in
the README, in the reference file, in `check_conformance.py` and in the release
process. That repetition is deliberate: the failure this guards against is a
user assuming the stronger promise, and the only defence is saying it
everywhere the weaker one appears.

Some optimisations are permanently off the table. The cost is real and has not
been measured as significant at 1 kHz over a state vector of at most fifteen
doubles.

A user on an unrecorded platform gets a number about which nothing has been
claimed. That is the honest outcome and it is designed for in
[ADR-0008](0008-reference-hashes-are-keyed-by-build.md).

As an observation and not a promise: on x86_64, GCC 11, GCC 13 and Clang 18
produce identical hashes for all four conformance cases.
