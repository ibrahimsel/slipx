# ADR-0045: The broadphase is a static BVH plus a refit overlay, and the grid keeps the wall rays

- **Status:** Proposed
- **Date recorded:** 2026-08-19
- **Requirements:** SIM-09 and the P1 performance goals in spirit;
  `docs/spec` is not present in this checkout, so no ID is cited as
  authority.
- **Related:** [ADR-0037](0037-sensing-never-sees-the-scene.md),
  [ADR-0043](0043-contact-is-one-impulse-between-declared-footprints.md)

## Context

Racing adds two geometric populations the grid index was never built for:
moving cars that rays must be able to hit (a LiDAR that cannot see the
opponent is not a racing sensor) and moving cars that must be paired up for
contact without testing every pair at every step forever. The raycast grid
(raycast.hpp) covers neither: it is prebuilt over static walls and rebuilding
it per step for moving boxes would be exactly the per-step work a broadphase
exists to avoid.

Separately, the 20-agent performance target stood missed by 24 per cent,
all of it in the sensing, and the recorded position (M5.12) was that closing
it meant a different acceleration structure, "which is an ADR and the racing
phase's broadphase". So one structure was on the hook for two jobs: the
racing broadphase, and the last candidate for the missing 24 per cent.

## Decision

**The broadphase is two structures in `slipx/scene/broadphase.hpp`, matched
to two populations. `SceneBvh` is a bounding-volume hierarchy over the wall
segments, built once and never touched again; `AgentOverlay` is a flat array
of oriented boxes whose bounds are refit in place each step, answering ray
queries (with a self-skip) and axis-aligned pair queries. Nothing rebuilds
per step. And the wall rays stay on the grid, because the BVH was measured
against it and lost.**

The load-bearing choices:

1. **Refit only, and flat.** At racing agent counts a linear scan over
   twenty fattened boxes beats any tree it would pay to maintain, so the
   overlay is honestly what it is: an array. The pair query is a sort-and-
   sweep along x with a fully specified tie-break, conservative (kissing
   bounds count; the contact narrowphase decides), and asserted equal to
   the brute-force definition.
2. **The BVH build is fully specified.** Sorted median splits with an index
   tie-break, never `nth_element`, so every standard library builds the same
   tree; the traversal is ordered nearer-child-first and pruned against the
   best hit, and unlike the grid it needs no scratch stamps, so the query is
   genuinely thread-safe. Every bounding box is fattened by a nanometre,
   for the grid's corner-margin reason: a ray through the exact corner of a
   wall can pass the exact segment test while a zero-slack slab test rejects
   it by an ulp. Padding adds tests and can never change an answer, because
   the exact test stays the authority, shared with the grid in one header
   (`src/ray_intersect.hpp`) so the two accelerators cannot drift apart.
3. **The grid keeps the wall rays, by measurement.** On the workload that
   matters, short rays from on-track poses in a 1.5 m corridor, alternated
   in one session on the named machine: 95 ns per ray through the grid,
   280 ns through the BVH. The corridor is the reason: the grid enters two
   or three cells and stops, while a from-the-root descent pays a dozen node
   slab tests to reach the same handful of segments. The benchmark prints
   both costs per commit so the decision stays re-checkable if tracks or
   sensors change shape.
4. **The 20-agent target is renegotiated to over 7x.** With the
   acceleration-structure route measured shut, the standing decision of
   2026-08-19 applies. The case measures 7.3x today and 8.4x four days
   earlier on the same machine with the same code (pre- and post-racing
   binaries alternate identically), so the target becomes the number every
   session clears. The record is in `docs/reference/performance.md`.
5. **Composition stays above** (ADR-0037): the overlay knows boxes, not
   agents; the BVH knows segments, not tracks' meanings; and "what does this
   LiDAR see" remains a function the orchestrator composes from wall hits
   and overlay hits, taking the nearer.

## Consequences

- Two acceleration structures for rays now exist, and the fact that the
  slower one exists at all needs this record: it was built as the racing
  broadphase and as the measured answer to "would a BVH close the gap", and
  the answer, no, is worth exactly as much as having tried. Deleting it
  would reopen the question; keeping it costs one build per scene.
- The overlay's pair query is quadratic in the worst case (every box
  overlapping every box), which for cars on a track cannot happen without
  the contact model having already failed. The sweep is for the common
  sparse case, not an asymptotic guarantee.
- The renegotiated 7x is a target on one named machine, like every number
  on the performance page. The honest cross-machine statement stays
  "measure it yourself with the shipped benchmark".
- Sensing that wants cars visible pays the overlay on top of the wall ray
  (about 124 ns for twenty boxes today, printed by the benchmark); scenario
  authors choose whether opponents are visible, and the benchmark's
  published 20-agent case remains walls-only so its history stays
  comparable.
- Multi-core stepping is the one lever this page leaves untouched, and it
  stays untouched deliberately: single-threaded lockstep is a determinism
  decision, and revisiting it is its own ADR, not a performance patch.
