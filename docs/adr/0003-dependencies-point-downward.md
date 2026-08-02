# ADR-0003: Dependencies point strictly downward, and the binding layer sits above what it binds

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0)
- **Requirements:** CORE-01, NFR-06
- **Related:** [ADR-0002](0002-no-eigen-in-the-core.md),
  [ADR-0004](0004-step-is-const-and-stateless.md)

## Context

The library is the product, not the simulator. There are already four or five
1/10-scale simulators and shipping a sixth is a poor bet; being the physics
layer inside the existing ones is a better one. That only works if the physics
layer can be lifted out, which means it must not depend on anything above it.

This is the kind of rule that is broken by accident rather than on purpose. A
logging call added while debugging, a parameter struct that grows a
`std::filesystem::path`, an include added to fix a build: each is locally
reasonable and each ends the embedding story.

The SRS complicates one edge. Its stack order in section 2.1 places the `slipx`
Python package *below* `slipx_sim`, which would forbid the bindings from
exposing the orchestrator. The same document describes `slipx` as a Gymnasium
adapter, and a Gymnasium environment is a fixed-step loop over agents, which is
exactly what the orchestrator is. The two statements cannot both hold.

## Decision

Dependencies point strictly downward through this stack, top depending on
bottom and never the reverse:

```
slipx_registry -> slipx_id -> slipx_ros -> slipx_sim -> slipx_scene / slipx_sense
               -> slipx (Python) -> slipx_schema -> slipx_core
```

`slipx_core` may depend only on the C++ standard library. No ROS, no threads,
no I/O, no logging, no allocation inside `step`. It must build and pass its
full test suite with `slipx_schema` absent: parameters arrive as a plain
`VehicleParams` struct and parsing lives elsewhere.

The binding layer sits **above** everything it binds, not below. The SRS is
wrong on this point and should be amended; `tools/dep_lint.py` implements the
corrected order and says why in a comment.

Both halves are executable rather than asserted. `tools/dep_lint.py` reads the
link lines and the Python imports and fails the build on a violation.
`-DSLIPX_CORE_ONLY=ON` configures the core with every layer above it switched
off and runs its full test suite, so "builds without `slipx_schema`" is a CI
job rather than a claim.

## Consequences

Some things are more awkward than they would otherwise be. The core cannot log
a warning, so it returns diagnostics
([ADR-0006](0006-diagnostics-report-nan-not-zero.md)). It cannot read a file,
so somebody else parses. It cannot own a thread, so batching is the caller's
problem. Each of those is the correct trade for the adoption goal and each will
feel wrong at least once.

A change that violates the direction is wrong even if it compiles and even if
the tests pass. That is a strong statement and it is deliberate: this is the
single most load bearing decision in the design, and the cost of finding out
late that the core cannot be lifted out is the project's whole thesis.

The SRS and `tools/dep_lint.py` disagree on paper until the SRS is amended.
The lint script is the operative version, since it is the one that fails the
build.
