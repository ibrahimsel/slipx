# ADR-0006: Diagnostics report NaN, not zero, for what a tier cannot represent

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0)
- **Requirements:** CORE-06, CORE-07
- **Related:** [ADR-0005](0005-tiers-throw-rather-than-fall-back.md),
  [ADR-0003](0003-dependencies-point-downward.md)

## Context

`StepDiagnostics` is the optional output of a step: slip angles, slip ratios,
per-tyre forces, load transfer terms, actuator saturation flags. It is what
lets a student plot exactly why the car spun.

One struct serves every tier, and no tier fills all of it. L0 is kinematic and
has no tyres, so it has no slip angles. L1 is a single-track model and has no
mechanism for load transfer, so it has no load transfer terms.

Something has to go in those fields. Zero is the default a struct gets for
free, and zero is a number somebody will plot, average, feed to a controller,
or compare against a measurement. A flat zero line for front slip angle looks
exactly like a car that is not cornering.

The core cannot log, cannot throw per field, and cannot return an optional per
field without making the hot path expensive
([ADR-0003](0003-dependencies-point-downward.md)).

## Decision

Any diagnostic a tier cannot represent is NaN, never zero.

NaN propagates. It cannot be silently averaged into a plausible mean, it makes
a plot empty rather than flat, and any comparison against it is false, so a
threshold check on an unrepresented quantity fails rather than passing by
accident.

## Consequences

A consumer must handle NaN. That is the point, and it is a one-time cost paid
while writing the plotting code rather than a permanent risk of believing a
zero.

The core's test suite asserts NaN in the unrepresented fields per tier, so the
promise is checked and not merely documented. There is also an invariant suite
asserting that no parameter set in the sweep produces NaN in a field the tier
*does* represent, which is the other half: NaN must mean "not representable
here" and never "the arithmetic went wrong".

Naive numerical code downstream will produce NaN where it previously produced a
wrong number. That is a better failure and it will still be reported as a bug at
least once.
