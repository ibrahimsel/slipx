# ADR-0054: the bridge latches the raycaster's walls as the occupancy map

- **Status:** Proposed
- **Date recorded:** 2026-08-20
- **Requirements:** the ROS bridge requirements in spirit; `docs/spec` is
  not present in a fresh clone, so no ID is cited as authority. The
  operative constraint is ADR-0050's exit shape: an existing stack
  connects with a remap file and no code change, and the standard F1TENTH
  stack subscribes to a map.
- **Related:** ADR-0050 (the bridge), ADR-0049 (the composed world whose
  walls these are), ADR-0024 and ADR-0028 (nothing drawn that is not in
  the recorded state), ADR-0025 (refuse rather than default, for the
  resolution refusal)

## Context

The bridge published scans, odometry, ground truth and a TF tree, and
nothing about the track. RViz on fixed frame `map` showed every scan and
pose floating over an empty grid, and a stack that localises or plans
against a map had nothing to subscribe to: the F1TENTH convention, which
the gym bridge and every map server follow, is a latched
`nav_msgs/OccupancyGrid` on `/map`. Without one, the bridge could host
ground-truth followers and reactive drivers and nothing else, against
ADR-0050's stated goal.

The rule that nothing is drawn that is not in the recorded state, in
particular no invented track (ADR-0024, ADR-0028), was written when no
track existed and a drawn kerb would have asserted geometry the
simulation did not have. The situation has inverted: the walls exist,
every lidar ray is answered by them, and the map question is not whether
to invent geometry but which statement of the existing geometry to put on
the wire.

Alternatives considered:

- **Wall polylines as RViz markers.** Exact geometry, no rasterisation
  choices, and only RViz can consume it: a localiser cannot march rays
  through a marker array. It decorates the visualisation without serving
  the stacks, which is the goal backwards.
- **Rasterise from the centreline and widths in the bridge.** The wall
  offset would be derived a second time, in Python, and two derivations
  can disagree. A map that disagrees with the scans, even by one cell at
  a mitred corner, is a localisation bug handed to every user; it is
  worse than no map.
- **Gate the map on ground truth.** Wrong category: the map is geometry,
  not truth-telling. A real car has a map without having a perfect
  localiser, because SLAM gave it one before the race.

## Decision

`TrackWorld` exposes the wall polylines rays are cast against
(`wall_left`, `wall_right`), and the bridge rasterises exactly those
polylines into a `nav_msgs/OccupancyGrid`, latched once on `/map`
(reliable, transient local, depth 1, frame `map`, stamp zero).

The rasterisation is a supercover grid walk over every wall segment,
including the closing segment exactly when the raycaster includes it:
every cell a segment passes through is occupied. Free space is a
four-connected flood fill from the first centreline sample, which is
inside the drivable band by construction; everything the fill cannot
reach without crossing a wall stays unknown. That is the shape a SLAM
map of the same walls would have: walls occupied, observed track free,
the world beyond the walls unknown. A supercover barrier cannot be
crossed diagonally by a four-connected fill, so the unknown region is
structural, not luck.

`--no-map` declines the topic for a stack that runs its own map server,
`--map-resolution` sets the cell size (default 0.05 m, the F1TENTH map
convention), and the bridge manifest records both. A resolution so
coarse that a wall lands in the seed cell is refused by name, never
published as a map that is all wall (ADR-0025's rule, on the wire).

## Consequences

- The map contains walls and only walls. Opponents are dynamic: they
  appear in the scans and never in the map, exactly as they would in a
  SLAM map of an empty track. A planner that treats the map as the whole
  world will drive through parked cars; the scans are the authority on
  the present.
- A stack tuned against the gym bridge's pgm map gets the same shape and
  different pixels: this map is rasterised from exact polylines, not
  scanned from a driven session, so it has no speckle, no partial
  observations and no closed-off pit areas.
- The map publishes once per run. Anything that changes during a run
  belongs to other topics; a future dynamic layer would be a new topic,
  never a republished `/map`.
- On an open track the free region runs out of the open ends to the grid
  border, because there genuinely is no wall there; the raycaster
  reports misses through the same gap.
- `info.resolution` is float32 on the wire. A consumer doing exact cell
  arithmetic against the declared resolution should use the declared
  value, not reconstruct it from cell coordinates.
- The binding surface grew by two read-only properties on `TrackWorld`.
  They exist so that no consumer ever has a reason to re-derive the
  offset; a second derivation appearing anywhere downstream is the
  failure this record exists to prevent.
