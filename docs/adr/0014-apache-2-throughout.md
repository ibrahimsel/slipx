# ADR-0014: Apache-2.0 throughout, and no copyleft anywhere near the core

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0)
- **Requirements:** NFR-01
- **Related:** [ADR-0002](0002-no-eigen-in-the-core.md),
  [ADR-0003](0003-dependencies-point-downward.md)

## Context

The adoption goal is `slipx_core` compiled into simulators maintained by other
people, some of which are commercial and some of which are university projects
with no legal department at all.

A copyleft licence anywhere in the core makes that a legal question. For a
commercial simulator it is a question answered by not adopting; for a student
project it is a question nobody asks until it is a problem. Either way the
embedding strategy is defeated, and it is defeated silently, because a team that
declines for licensing reasons does not file an issue about it.

The risk is not primarily the project's own licence, which is easy to choose
once. It is a transitive dependency acquiring one later, in a change whose
author is thinking about an algorithm rather than a licence.

## Decision

Apache-2.0 throughout, and no copyleft dependency anywhere in the core.

Apache-2.0 rather than MIT for the explicit patent grant, which is the clause a
commercial adopter's review actually looks for.

This is checked rather than asserted. `tools/licence_scan.py` runs in CI, checks
that every declared dependency is permissive, and separately checks that none of
them appears in `slipx_core`. [ADR-0002](0002-no-eigen-in-the-core.md) makes the
second check trivially satisfiable today, since the core has no dependency at
all.

## Consequences

A GPL or LGPL library is off the table for the core no matter how good it is.
That has already been priced in and will be felt when a well-known solver turns
out to be GPL.

The licence scan is one of the checks that fails for reasons a compiler never
would, alongside the dependency lint and the version check. It costs seconds and
catches a class of mistake that is otherwise found by an adopter's lawyer.

Test-only dependencies are treated separately. GoogleTest is permissive anyway,
and the dependency lint checks that it never reaches `slipx_core`'s link line,
so the distinction is enforced rather than trusted.
