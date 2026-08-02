# ADR-0002: `slipx_core` depends only on the C++ standard library

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0, recorded as D-02)
- **Requirements:** CORE-01, NFR-06, NFR-01
- **Related:** [ADR-0003](0003-dependencies-point-downward.md),
  [ADR-0014](0014-apache-2-throughout.md)

## Context

The original design had Eigen as `slipx_core`'s linear algebra dependency. It is
the default choice for vehicle dynamics code and the reflex is a good one in
most projects.

The two-year goal is `slipx_core` turning up as a dependency in a simulator we
do not maintain. That makes the cost of a dependency different from what it is
in an application. Every transitive dependency is a `find_package` in somebody
else's build, a version to pin per platform, a licence for somebody else's
lawyer to read, and one more variable in the determinism argument.

Against that, what the core actually computes was measured. Every tier is a
small explicit expression in two and three dimensions. There is no solver, no
decomposition, no dynamic sizing, and no expression tree deep enough for
expression templates to matter. What Eigen would have contributed to this
particular code is notation.

Alternatives considered:

- **Keep Eigen.** Familiar, well tested, and the notation is genuinely nicer.
- **Header-only alternative (GLM or similar).** Trades a large dependency for a
  small one and keeps every structural cost.
- **Hand-rolled types.** More code we own and must test.

## Decision

`slipx_core` depends on the C++ standard library and nothing else.
`slipx/math.hpp` provides `Vec2`, `Vec3` and `Mat3` in about 180 lines.

Embedding SlipX therefore costs two lines of CMake and no transitive
dependency:

```cmake
add_subdirectory(slipx)
target_link_libraries(your_simulator PRIVATE slipx::core)
```

That is the adoption strategy stated as a build system fact rather than as an
intention, and it is enforced by `tools/dep_lint.py` rather than by good
manners.

## Consequences

We own and test the maths. That is a real cost and it is bounded: the header is
small, it has no algorithmic content, and the core's test suite exercises it
through every tier.

The zero-dependency claim becomes load bearing. It appears in the README, in
NOTICE, in the licence scan and in the adoption argument. Reinstating Eigen, or
adding any other dependency to the core, is not a build system change; it is a
change to the reason the project expects anyone to adopt it. It would need
`tools/dep_lint.py`, the README and NOTICE edited together, and a reason strong
enough to give up the claim.

A future tier that genuinely needs linear algebra (a solver for an implicit
integrator, say) would reopen this. The right response then is a new ADR
superseding this one, not a quiet `find_package`.
