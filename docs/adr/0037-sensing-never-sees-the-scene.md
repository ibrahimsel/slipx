# ADR-0037: Sensing never sees the scene, and the seeded RNG moves down to meet it

- **Status:** Proposed
- **Date recorded:** 2026-08-15 (decision taken during P1)
- **Requirements:** SIM-03, NFR-02, NFR-06. No sensing requirement ID is
  cited: `docs/spec` is not present in this checkout.
- **Related:** [ADR-0003](0003-dependencies-point-downward.md),
  [ADR-0004](0004-step-is-const-and-stateless.md),
  [ADR-0034](0034-a-track-is-geometry-plus-a-declared-surface.md)

## Context

A 2D LiDAR needs two things the components around it own separately. It needs
geometry, to know what a ray hits, and it needs randomness, for noise,
dropouts and latency jitter. Neither is where it can reach.

**Geometry.** `tools/dep_lint.py` lists `slipx_sense` below `slipx_scene`, so
sensing may not include a scene header. That ordering is not a decision
anybody took. The dependency diagram in the spec shows the two as a pair on
one line, `slipx_scene / slipx_sense`, and the lint had to put them in some
order to compare them, so it put them in the order they were written down.
Taken literally it forbids a LiDAR from seeing a track, which is close to
forbidding a LiDAR.

**Randomness.** `Rng` lives in `slipx/sim/rng.hpp`, above both, although its
own opening comment says it exists for "sensor noise, dropouts, actuator
jitter and scenario generation". It was written for a consumer that did not
exist yet and parked in the component that did.

The obvious repair to the first problem is to reorder the lint so scene sits
below sense. It works, and it makes the sensor model depend on the track
representation, so a change to how a centreline is stored becomes a change
that can break a LiDAR. That coupling is not wanted in either direction.

## Decision

**Scene and sense are siblings, and neither depends on the other.**
`tools/dep_lint.py` keeps a total order because comparing two layers needs
one, and the order between these two is now recorded as arbitrary rather than
meaningful, with the real rule being that neither includes the other.

A sensor model is about timing, noise and dropouts. What a ray hits is a
question for whoever owns geometry, so the LiDAR receives the world through a
caller-supplied function: given a ray, return a distance. `slipx_scene`
provides an implementation of that function for a track, `slipx_sim` puts the
two together, and neither component names the other in an include.

The cost is an indirect call per ray, several thousand times a second per
agent. That is a real cost and it is unmeasured; M5.9 measures it, and if it
matters the fix is to template the call rather than to collapse the layers.

**The seeded RNG moves to `slipx/sense/rng.hpp`.** It goes to the lowest C++
layer above the core that needs it, which is sensing, and `slipx_sim`
includes it from there. The alternative homes were both worse: duplicating it
would give two generators that must produce identical streams forever with
nothing enforcing it, and putting it in `slipx_core` would contradict the
rule that randomness enters above the core, in a header the core would never
call.

The type, the algorithm and the seed derivation are unchanged. This moves a
file; it does not touch a number.

## Consequences

A public header path changes, `slipx/sim/rng.hpp` to `slipx/sense/rng.hpp`,
with no compatibility shim left behind. That is a break, it is pre-1.0, and
the affected surface is C++ callers of `slipx_sim`, which is not what the
distribution publishes. The Python package reimplements the same generator
and is untouched.

No trajectory hash moves. The generator produces the same stream from the
same seed and nothing about the numerical path changed, which is asserted
rather than assumed.

The injected range function is the seam every later sensor goes through, so
it is worth it being awkward now rather than later: an occupancy grid, a 3D
scene in P4, or a unit test's circular wall all satisfy the same signature,
and the LiDAR cannot tell them apart. The test suite uses that directly,
which is how motion distortion gets asserted against a world with a closed
form rather than against a track.

Reversing this means deciding that a sensor may know what it is looking at.
The place that would show up first is a LiDAR that special-cases track
geometry for speed, which is exactly the optimisation ADR-0004's determinism
argument makes expensive to reason about.
