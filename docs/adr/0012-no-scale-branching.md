# ADR-0012: No `if (scale == ...)` branching in code

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0)
- **Requirements:** SCH-07
- **Related:** [ADR-0009](0009-mf-lite-over-full-pacejka.md)

## Context

SlipX targets 1/10-scale cars. Formula Student scale is on the roadmap for P5
as an optional schema extension, and there is standing pressure to support it
sooner, because the physics is the same physics.

The physics is the same physics. That is the whole argument for the decision.
A double-track model with load transfer and a Magic Formula tyre does not know
how large the car is; it knows masses, lengths, inertias and coefficients. If a
scale flag ever becomes necessary, the reason will be that some quantity which
should have been a parameter was hard-coded as an assumption.

The failure mode is well documented in other simulators and is progressive.
One branch appears for a limit. Then a second for a default. Then a tyre model
selection, then a solver tolerance, and eventually there are two codebases
sharing a file, only one of which is tested.

## Decision

There is no scale branching in code. No `if (scale == ...)`, no scale enum, no
per-scale defaults compiled in.

Formula Student scale is an optional set of schema extension fields and nothing
more. A larger car is a car with larger numbers in its parameter files.

A scale branch appearing in a diff is treated as a design failure and as
evidence that a parameter is missing, and the fix is to add the parameter.

## Consequences

Supporting a new scale is a parameter file, which is the outcome that makes the
core embeddable in contexts nobody anticipated. It is also the outcome that
makes "does SlipX support X scale" answerable with "does X scale have a mass and
a wheelbase".

Anything genuinely scale-dependent must be found and made explicit rather than
branched around. The one already known is the slip-angle speed floor
(`numerics.v_eps`), which is a numerical mitigation rather than a property of
the car, is optional per car, and is recorded in the run manifest either way.

This is listed as a named risk in SRS section 8, because the pressure to add
the first branch will come with a deadline attached and will look small.
