# ADR-0005: An unimplemented tier throws rather than falling back

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0)
- **Requirements:** CORE-05
- **Related:** [ADR-0006](0006-diagnostics-report-nan-not-zero.md),
  [ADR-0013](0013-provenance-labels-are-printed.md)

## Context

Tiers L0 to L3 are fidelity levels selected at construction behind one
`VehicleModel` interface. P0 implements L0 (kinematic bicycle, 4 states) and L1
(dynamic bicycle with linear tyres, 6 states). L2, the double-track model with
load transfer and MF-lite, is the default tier and the one the tier system
identification fits against, and it arrives in P1. L3 is later still.

So for the whole of P0 there is a public enum with two values that work and two
that do not, and the question is what `VehicleModel::create(Tier::L2, ...)`
does.

The tempting answer is to return L1 with a warning. It keeps demos working, it
keeps a caller's code compiling across the P0/P1 boundary, and the warning
discharges the obligation to be honest.

It does not. A warning is seen once, by the person who ran it interactively,
and never again by the CI job, the batch of ten thousand rollouts, or the
person reading the resulting plot six weeks later. What survives is a
trajectory labelled L2. Anyone comparing it against a real L2 result is
comparing two different models and has no way to tell.

Below L2 nothing represents CoG height, weight distribution, differential or
tyre compound. A silent fallback means a user varies those parameters, observes
no change, and concludes the car is insensitive to them.

## Decision

`VehicleModel::create` throws for L2 and L3. It does not fall back, and it does
not warn and continue.

A trajectory labelled L2 that is actually L1 is worse than no trajectory.

## Consequences

Asking for the default tier fails loudly for the whole of P0. That is
inconvenient, it is visible in the README, and it is the intended cost.

Downstream code written against L2 during P0 cannot run until P1 lands. It also
cannot silently produce results that will need retracting, which is the trade.

Parameters below L2 that have no effect (CoG height, weight distribution,
differential, compound) stay in the schema and stay inert. That is deliberate
and is documented as the teaching artefact: the tier determines which
parameters can possibly matter, and discovering that is the point rather than a
bug to fix.
