# ADR-0056: The race direction is the centreline's traversal, and reversal races the reversed track

- **Status:** Proposed
- **Date recorded:** 2026-08-20
- **Requirements:** RACE-level requirements in spirit; `docs/spec` is not
  present in this checkout, so no ID is cited as authority.
- **Related:** [ADR-0034](0034-a-track-is-geometry-plus-a-declared-surface.md),
  [ADR-0046](0046-race-control-is-a-layer-that-mechanises-the-ruleset.md),
  [ADR-0050](0050-the-ros-bridge-is-rclpy-above-the-bindings.md),
  [ADR-0054](0054-the-map-is-the-raycasters-walls-latched-once.md)

## Context

Every signed quantity in the racing stack already presumes a direction: arc
length increases along the centreline as the file orders its points, the lap
counter counts that direction positive, `pose_at` faces along it, and the
grid start places cars facing it. But the direction was nowhere stated, in
the way a thing everyone assumes never is. Three problems follow.

First, nothing could race the other way round. Real organisers reverse a
track between heats; here that would have meant editing the geometry file,
which is the venue, not the race. Second, nothing told a running stack which
way the race goes. A scan-only driver has no way to sense direction (a real
one does not either; race control announces it and the grid faces it), and
the pursuit demo answered the question by reading the track's own CSV off
disk, which only works when the driver lives in the simulator's checkout.
Third, nothing noticed a car driving the wrong way: a spun follow-the-gap
car will happily race backwards forever, and race control had no word for
it.

Alternatives considered for the reversal mechanism:

- **A `direction:` field in the track manifest.** Rejected: the point
  ordering already carries the direction, so a declared copy could
  contradict it, and which way a heat runs is a property of the race, not
  the venue. The manifest records what a surveyor could check; a race
  decision does not belong in it.
- **A sign flag threaded through race control.** A `reversed` flag consulted
  at every consumer: lap deltas negated, grid headings rotated, restart
  poses flipped, obstacle gaps mirrored. Rejected because every consulting
  site is a place to forget, and the first missed site (a restart placing a
  car facing the wrong way) is exactly the class of bug the flag exists to
  prevent.
- **Reversing the track object.** One function produces the same venue
  traversed the other way, and every consumer stays direction-blind.
  Chosen.

For the wrong-way judgment, penalising was considered and rejected: the
pinned ruleset revision (ADR-0046) has no wrong-way rule, and inventing a
penalty would be adjudicating by a rule that does not exist, the same
reasoning that abandons a never-ending match rather than ruling on it.

## Decision

The race direction is the direction of increasing arc length along the
centreline as declared, and a race run the other way is run on the reversed
track.

- `scene::Track::reversed()` returns the same venue traversed the other
  way: point order reversed, left and right widths swapped, arc length
  re-derived, manifest and tyres unchanged, and on a closed track the first
  point kept first, because reversing a lap must not move the start line.
  The centreline origin gains a "(reversed)" suffix so manifests say which
  way the geometry was walked. A closed centreline whose last point repeats
  its first is refused rather than silently repaired, naming the duplicate
  row.
- `RaceConfig` gains `reversed`. Each race procedure (time trial, head to
  head round, obstacle test) owns a copy of the track, reversed when the
  flag is set, so the grid, the counters, the corridor and every restart
  measure one direction and none of them carries a sign. The flag travels
  in the event stream metadata, because which way the races ran is part of
  what race it was.
- Race control watches for wrong-way driving: a car whose progress falls
  `wrong_way_distance` behind its own furthest point draws a `wrong_way`
  event, once per excursion, re-armed when the ground is made back.
  Teleports (border restarts, crash set-backs) rebase the monitor instead
  of being ruled on: the referee moved the car, the car did not drive. The
  event records and never penalises; a competition adopting this decides
  what it costs.
- The bridge announces the direction the way race control does, before the
  heat rather than in-band: the centreline latched once on
  `/race/centreline` as a `nav_msgs/Path` whose pose order and tangents are
  the direction, in the map frame, with the map's latching QoS. On a closed
  track the first pose is repeated at the end, because a Path carries no
  closed flag and a subscriber should not guess whether two nearby ends
  join. `--reversed` turns the announcement and the grid together;
  `--no-centreline` declines the topic for a stack that brings its own
  racing line, recorded in the bridge manifest like the map's decline.

## Consequences

- Reversal costs one track copy per race procedure, a few hundred points.
  Accepted for the deleted alternative: no sign logic exists anywhere, so
  no consumer can get it half right.
- `line_s` and every other arc length a caller passes into a reversed race
  are coordinates of the reversed track. On a closed track s = 0 is the
  same start line both ways; any other landmark must be re-projected, which
  is the honest cost of the coordinate actually turning round.
- A scan-only driver remains direction-blind, deliberately. Its direction
  is the grid placement, exactly as on a real grid; making the sensors
  carry direction would falsify what a LiDAR can know. The announcement is
  ground-truth-side information, like `/map`.
- The `wrong_way` event carries no penalty, so a wrong-way car loses
  nothing under the mechanised rules except the ground it drove. If a
  future ruleset revision adds a rule, the event is the hook and the
  threshold is already a named `RaceConfig` field.
- The published Path duplicates the closing pose, so a consumer counting
  samples must not assume it equals the file's row count. That is the price
  of making closure explicit on the wire.
- Reversing this decision means teaching every consumer a direction flag;
  the record above lists the sites that would each need it.
