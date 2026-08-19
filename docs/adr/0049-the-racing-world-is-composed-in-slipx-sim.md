# ADR-0049: the racing world is composed once, in slipx_sim, from the walls and the footprints

- **Status:** Proposed
- **Date recorded:** 2026-08-19
- **Requirements:** SENSE-01, SENSE-04, SIM-09, NFR-02
- **Related:** ADR-0037 (sensing never sees the scene), ADR-0043 (contact
  footprints), ADR-0045 (the agent overlay), ADR-0047 (the sensor rig)

## Context

The sensor rig takes the world as a function, and until now every caller
assembled that function by hand: the reference stack closes over the wall
raycast, the benchmark builds its own overlay, and nothing anywhere lets a
LiDAR see another car even though the agent overlay was built for exactly
that. The ROS bridge forces the question, at scale: twenty agents, each
with a 1080-ray scanner, need rays answered by "the nearer of the wall and
the nearest other car", and a world assembled in Python would put a million
interpreter calls per simulated second on the hot path.

Where the composition lives is constrained from three sides. It reads the
scene (walls, overlay), so it cannot live in `slipx_sense` (ADR-0037
forbids that direction). It reads the simulation's states and footprints,
so it cannot live in `slipx_scene` (which sits below the sim and must not
know what an agent is). And `slipx_race` is above the bindings' needs: a
lone car lapping a track wants this world with no referee anywhere in
sight. The alternatives considered were a component of its own between
scene and sim (a whole layer for one class), keeping per-caller assembly
(the status quo that never produced cars-seeing-cars), and `slipx_race`
(which would drag the ruleset into every sensored simulation).

## Decision

`slipx_sim` gains `TrackWorld`: the track's walls and the simulation's own
agent footprints, composed into the rig's world function. `slipx_sim` now
depends on `slipx_scene` as well as `slipx_sense`, which is the dependency
ADR-0037 anticipated in so many words ("they meet in slipx_sim, which is
above both and is allowed to know about each").

The choices inside it:

- A ray is answered with the nearer of the grid-accelerated wall cast and
  the agent overlay cast with the asking car skipped: an emitter does not
  see its own body, and the skip is by agent index, which is why the rig's
  world signature carries one (ADR-0047).
- The boxes are the same footprints the contact pass collides, centred
  between the axles (ADR-0043), gathered once at construction because a
  spec cannot change after `add_agent`. What a LiDAR sees touching is what
  the physics says is touching, and a car with no footprint is invisible to
  sensors exactly as it is untouchable by bumpers: one rule, not two. A
  DNF'd car keeps its box, because a wreck is an obstacle (ADR-0042).
- The overlay refits lazily, once per simulation step, on the first ray
  after an advance: every ray within one step sees one consistent world,
  and other agents appear at step resolution, ADR-0047's stated
  approximation. The lazy refit writes scratch under a const call, so the
  object is not thread-safe, the same trade the wall grid documents.
- `max_range` is a constructor parameter, not a guess: the accelerated
  casts want a bound, and the honest bound is the longest range any sensor
  will ask for, which only the caller knows.
- Agents added after construction are refused at the next cast rather than
  silently unseen: a world missing a car is the invisible-obstacle bug,
  and it would otherwise appear only as a stack driving through someone.

The bindings expose the same composition natively (a `SensorRig`
constructed from a `TrackWorld` never crosses into Python per ray), with
the scene's `Track` bound just far enough to build one from a track
directory the Python loader has already validated.

## Consequences

- `slipx_sim` depends on `slipx_scene`. Embedders of the orchestrator now
  link the scene as well; embedders of `slipx_core` are untouched, and the
  core's promise (standard library and nothing else) is not involved.
- The dependency lint's layer order already permitted this edge, so the
  operative statement of the stack order does not move; what moves is that
  the edge is now real rather than latent.
- A caller with a world of their own (a custom arena, a synthetic test
  shape) keeps the function seam: `TrackWorld` is a convenience standing
  beside it, not a new requirement.
- Sensors still cannot perturb: `TrackWorld` holds const references, and
  the trajectory of a sensored run remains bit-identical to its bare twin.
  No reference hash can move and none did.
