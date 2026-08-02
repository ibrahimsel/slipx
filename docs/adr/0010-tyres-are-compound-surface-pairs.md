# ADR-0010: Tyres are referenced as a `(compound, surface)` pair

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0)
- **Requirements:** SCH-05
- **Related:** [ADR-0009](0009-mf-lite-over-full-pacejka.md),
  [ADR-0011](0011-schema-refuses-a-newer-minor.md)

## Context

The obvious schema puts tyre coefficients in the car file, next to the mass and
the wheelbase. It is one file, it is what most simulators do, and it is wrong
in a way that shows up immediately at this scale.

A 1/10-scale car races on carpet in a sports hall, on polished concrete in a
lab, and on asphalt outdoors, often in the same week. The peak friction of a
sponge tyre on carpet and the same tyre on polished concrete differ by more
than most of the parameters anyone bothers to tune. The car has not changed.

If the coefficients live in the car file, the surface is invisible. The most
likely failure is somebody copying a car directory to a new venue and quietly
carrying asphalt coefficients into a sports hall, with nothing in the file
recording that a change of venue should have changed a number.

## Decision

Tyres are a separate schema, referenced from `dynamics.yaml` as a
`(compound, surface)` pair, never embedded in the car file.

The same car on carpet and on polished concrete is two different vehicles, and
the schema says so rather than leaving it to a convention.

## Consequences

A car directory is not self-contained in the naive sense; it references tyre
files, resolved from `tyres/` by convention. The loader reports that resolution
in its summary so the indirection is visible rather than magic.

Contributing a tyre measurement is decoupled from contributing a car. A team
that identifies sponge-on-carpet has produced something every other team on
carpet can use, which is the behaviour the registry in P2 is meant to
encourage. Embedding would have made every measurement car-specific and
therefore unshareable.

The surface becomes a first-class part of an experiment's description, so a
result is comparable only against results on the same surface. That is a
constraint on leaderboards and it is a true one.
