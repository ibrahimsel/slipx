# ADR-0055: Walls are immovable contact segments in the simulation

- **Status:** Proposed
- **Date recorded:** 2026-08-20
- **Requirements:** SIM-04 in spirit; `docs/spec` is not present in this
  checkout, so no ID is cited as authority.
- **Related:** [ADR-0043](0043-contact-is-one-impulse-between-declared-footprints.md),
  [ADR-0046](0046-race-control-is-a-layer-that-mechanises-the-ruleset.md),
  [ADR-0049](0049-the-racing-world-is-composed-in-slipx-sim.md),
  [ADR-0054](0054-the-map-is-the-raycasters-walls-latched-once.md)

## Context

ADR-0043 deliberately excluded car-to-wall contact: "what a wall does to a
car is a different decision for the slice that needs it." The slice that
needs it arrived with the ROS bridge. In the RViz race demo, walls exist
only to the LiDAR: a car shoved sideways by a pair impulse near a wall, or
one that simply steers badly, passes through the polyline with no
resistance, and once outside the track its scan sees nothing and it drives
off the map. Race control's border rule (a wall crash places the car at
rest, ADR-0046) covers C++ races, but the bridge does not run race control
and race control's rule is an adjudication, not a physical surface; a
stack under test should feel a wall the way its LiDAR sees one.

The alternatives actually considered:

- **A border rule in the bridge**, mirroring race control's: detect the
  crossing, place the car at rest. Rejected as the primary answer because
  it duplicates an adjudication into a layer whose job is message assembly
  (ADR-0050), because a car that teleports to rest is not what a stack
  under test should experience mid-lap, and because every embedder of
  `slipx_sim` would face the same hole and write the same rule again.
- **Walls as degenerate rectangles through `rectangle_contact`**, reusing
  the pair path with a zero-width box. Rejected on geometry: SAT with a
  minimum-overlap normal pushes a body out whichever side is nearer, so a
  car whose centre has crossed the line is pushed out the far side. That
  is the squeeze-through failure wearing the fix's own clothes, and near
  segment ends the minimum axis can point along the wall, snagging the car
  sideways.
- **A spring penalty at the wall.** Rejected for ADR-0043's original
  reason: a stiff term couples the step size to the contact stiffness and
  puts the stiffness into the trajectory.

## Decision

**Wall polylines are immovable contact geometry in `Simulation`
(`add_wall`, latched before the first advance), resolved by a pure
segment-versus-footprint function in the core (`segment_contact`) through
the same impulse and positional projection the pair pass uses, with the
penetration always resolved to the side of the wall line the car's centre
is on.**

The pieces:

1. **The core owns the mathematics, the sim owns the collision**, exactly
   as ADR-0043 split it. `segment_contact` is closed form and header-only:
   a Liang-Barsky clip of the segment against the footprint decides
   touching, the contact point is the clipped midpoint, and the normal is
   the segment line's unit normal oriented toward the footprint's centre.
   The existing `resolve_contact` then applies, with the wall as a body
   whose inverse mass and inverse inertia are zero: immovability is the
   standard spelling it already had for DNF'd cars.
2. **The centre-side rule is the anti-squeeze device.** A zero-thickness
   wall has no inside, so the resolution side must be chosen, and it is
   chosen as the side the centre currently occupies. The full positional
   projection removes the penetration every step, so for the centre to
   cross the line it would have to travel more than the footprint's
   half-width in one step: 150 mm for the reference car against 4 mm per
   step at 4 m/s and 1 kHz, or 40 mm at an unreachable 40 m/s. A step-size change must
   re-check this arithmetic, exactly as ADR-0043's tunnelling note says.
3. **Walls are scenery, latched before the green flag.** `add_wall`
   throws after the first advance; `reset()` keeps the walls. A run's
   walls are configuration, so the manifest records the segment count and
   a digest over the coordinates in order, both folded into the
   configuration digest: a race with walls is not the race without them.
4. **No walls, no change.** The wall pass over an empty segment list
   touches nothing, so every trajectory recorded before walls existed is
   bit-identical, conformance rows included. That is asserted by test and
   by the conformance check, not hoped.
5. **Fixed order, one pass.** Pairs resolve first, then walls in ascending
   (agent, segment) order, so a car shoved into a wall by a pair impulse
   is pushed back out in the same step. Sequential application within the
   pass is order-dependent but deterministic, the same trade the pair pass
   made. A cheap bounding-box reject (comparisons only, so rejected
   segments leave no trace in the trajectory) keeps the pass at
   microseconds against a 696-segment track.
6. **The bridge hands over the raycaster's own polylines.**
   `TrackWorld.wall_left` and `wall_right` go straight into `add_wall`, so
   the physics walls, the scans and the map are one geometry, never a
   re-derived offset (the same rule ADR-0054 set for the map).
7. **Wall contacts are reported, not adjudicated.** A `WallContactEvent`
   (agent, segment, point, normal, impulse, approach speed) is the
   single-sided sibling of `ContactEvent`, for race control or an event
   stream to interpret; the sim holds no opinion about fault (ADR-0046).

## Consequences

- The restitution and friction of a wall strike come from the same
  `ContactParams` as car-to-car contact, and are as unfitted as they were
  in ADR-0043: plausible and deterministic, not validated, and every
  document that touches contact keeps saying so. A separate wall material
  would be a second unidentifiable knob; if a real need appears it is a
  new decision.
- A car resting against a wall chatters at the restitution floor exactly
  as two rubbing cars do; the existing anti-jitter threshold covers both.
- An open polyline has ends, and a car can drive around an end: walls
  block what they geometrically cover, nothing more. No invented closure
  is added, for ADR-0024's reason (nothing is drawn, or now enforced, that
  is not in the recorded geometry).
- The centre-side rule is stateless within a step but not within a
  trajectory: a car teleported (by a state write) to the far side of a
  wall will be resolved to that far side. Scenario setup owns that
  correctness, as it already owns teleporting cars onto grid slots.
- Race control's border rule (2.5.3) stays: physics stops the car at the
  wall, the ruleset decides what a wall strike means for the race. The
  two answer different questions and neither replaces the other.
- Every simulation embedder who wants walls must feed polylines in; the
  sim still knows nothing about tracks, only segments, which keeps the
  dependency direction of ADR-0003 untouched.
