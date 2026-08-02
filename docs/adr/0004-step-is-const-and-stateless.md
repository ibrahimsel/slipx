# ADR-0004: `step` is `const` and stateless

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0)
- **Requirements:** CORE-03, CORE-04, NFR-02
- **Related:** [ADR-0003](0003-dependencies-point-downward.md),
  [ADR-0007](0007-determinism-is-scoped-to-a-build.md)

## Context

Two of the intended uses put hard constraints on the shape of the integration
call before any physics is written.

RL rollouts want thousands of instances stepping in parallel, and want
snapshot and restore between them. Competition replay wants a run to be
reproducible from a manifest and an input log, months later, on another
machine.

Both are trivial if the model owns no state and reads nothing ambient, and both
are difficult if it does. A model that caches the previous slip angle, or reads
a wall clock, or draws from a global RNG, cannot be snapshotted with a `memcpy`
and cannot be replayed without also replaying whatever it read.

The usual counter-argument is performance: caching across steps saves work.
Measured against a 1 kHz fixed step over a state vector of at most fifteen
doubles, there is nothing worth caching.

## Decision

`VehicleModel::step` is `const`. All mutable state lives in the caller's
`VehicleState`, which is a plain struct.

Consequently, and by rule rather than by accident:

- no hidden state and no memoisation across calls
- no wall clock, no ambient RNG, no environment reads
- no allocation inside `step`
- no unordered iteration anywhere in a numerical path (CORE-04), because
  iteration order over a hash container is a reduction order, and a reduction
  order is a floating-point result

Randomness, where a tier needs it, is passed in from a per-agent seeded
generator owned by the orchestrator.

## Consequences

N instances parallelise with no locking and no thread-local anything, because
there is nothing to share. Snapshot and restore is a struct copy. Replay needs
the manifest and the input log and nothing else.

A tier that genuinely needs history, such as a tyre relaxation length carrying a
lagged slip angle, must put that history in `VehicleState` where it is visible,
snapshotted and hashed. This is more verbose than a private member and it is
the correct verbosity: a state that is not in the state vector is a state that
silently breaks replay.

The no-allocation rule is tested rather than trusted; the core has a dedicated
no-allocation suite. The no-unordered-iteration rule is the one most likely to
be broken by a well-meaning refactor, since `std::unordered_map` is the obvious
container right up to the point where it changes a sum.
