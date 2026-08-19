// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The racing broadphase (ADR-0045): a prebuilt BVH over the static scene,
// and a flat refit overlay of moving boxes on top of it.
//
// Two structures because the two populations could not be less alike. The
// walls are thousands of segments that never move, so they get a real tree,
// built once with some care and never touched again. The agents are a few
// dozen boxes that move every step, so they get a flat array whose bounds
// are refit in place: at these counts a linear scan beats any tree it would
// pay to maintain, and "refit only" is the honest name for what the
// structure is. Nothing here rebuilds per step.
//
// Neither structure knows what a track or an agent is. The BVH is built
// from the Walls' polylines and answers rays; the overlay holds oriented
// rectangles and answers rays and pair queries. Composing them into "what
// does this LiDAR see" stays the orchestrator's job (ADR-0037), exactly as
// it was for the grid.

#ifndef SLIPX_SCENE_BROADPHASE_HPP
#define SLIPX_SCENE_BROADPHASE_HPP

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "slipx/scene/raycast.hpp"

namespace slipx {
namespace scene {

// A bounding-volume hierarchy over the wall segments, built once from a
// Walls object so the geometry is byte-identical to what the grid index
// accelerates; the two must agree with cast_brute_force, and therefore with
// each other.
//
// The build is fully specified (sorted median splits with an index
// tie-break, never nth_element), so the same walls produce the same tree on
// every standard library. The traversal is ordered and pruned: children are
// visited nearer-first and a subtree whose entry distance is past the best
// hit is skipped, which is the same early-out argument the grid makes cell
// by cell.
//
// Unlike the grid, the query is genuinely thread-safe: the traversal stack
// lives on the caller's stack and there is no scratch stamp array.
class SceneBvh {
 public:
  explicit SceneBvh(const Walls& walls);

  // Distance to the first wall the ray meets, or a miss; the same contract
  // as Walls::cast, and tested to agree with Walls::cast_brute_force
  // bit for bit.
  RayHit cast(double x, double y, double bearing, double max_range) const;

  // Diagnostic, for the benchmark and for anybody wondering whether the
  // tree is doing anything.
  std::size_t node_count() const { return nodes_.size(); }

 private:
  struct Segment {
    double ax, ay, ex, ey;
  };
  struct Node {
    double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
    // A leaf holds `count` segments starting at `first` in the reordered
    // arrays. An internal node holds 0, its left child is the next node
    // (the build is pre-order), and `first` is the RIGHT child's index:
    // the left subtree's size is not one, which is the classic mistake a
    // left-child pointer invites.
    std::uint32_t first = 0;
    std::uint32_t count = 0;
  };

  std::vector<Node> nodes_;
  // Reordered at build so each leaf's segments are contiguous.
  std::vector<Segment> segments_;
  std::vector<std::uint8_t> segment_left_;
};

// One moving box of the overlay: an oriented rectangle, the same footprint
// shape contact uses (ADR-0043). An inactive box is invisible to every
// query; that is how a caller expresses "this car has no footprint" or
// "skip the wreck" without renumbering anything.
struct OverlayHit {
  bool hit = false;
  double range = 0.0;      // [m]
  std::size_t index = 0;   // which box
};

class AgentOverlay {
 public:
  // Slots are stable: box i is agent i for whoever composes the queries.
  void resize(std::size_t count);
  std::size_t size() const { return boxes_.size(); }

  // Refit one slot in place: pose in, bounds recomputed, nothing allocated.
  void set(std::size_t i, double x, double y, double yaw, double half_length,
           double half_width, bool active = true);

  // The nearest active box the ray meets, `skip` excluded (the emitter does
  // not see its own body). A ray starting inside a box hits it at range
  // zero: two overlapping cars are touching, not invisible to each other.
  static constexpr std::size_t kNone = static_cast<std::size_t>(-1);
  OverlayHit cast(double x, double y, double bearing, double max_range,
                  std::size_t skip = kNone) const;
  // The definition: the exact oriented test against every active box, no
  // bounds prefilter. The accelerated query is only correct insofar as it
  // agrees with this.
  OverlayHit cast_brute_force(double x, double y, double bearing,
                              double max_range,
                              std::size_t skip = kNone) const;

  // Candidate pairs whose axis-aligned bounds overlap (touching counts:
  // the broadphase is conservative and the narrowphase decides), ascending
  // (i, j) with i < j, into `pairs`, which is cleared first. Sweep along x
  // after a sort; allocation-free once `pairs` and the overlay have their
  // capacity.
  void overlapping_pairs(
      std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs) const;
  // The definition: every unordered pair, tested directly.
  void overlapping_pairs_brute_force(
      std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs) const;

 private:
  struct Box {
    double x = 0.0, y = 0.0;    // centre, world                       [m]
    double c = 1.0, s = 0.0;    // cos(yaw), sin(yaw), computed at set
    double hl = 0.0, hw = 0.0;  // half extents                        [m]
    double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
    bool active = false;
  };

  std::vector<Box> boxes_;
  // Scratch for the sweep, sized with the overlay. Mutable for the same
  // reason the grid's stamps are: scratch for a const query, holding no
  // answer. Not thread-safe across concurrent pair queries; rays do not
  // touch it.
  mutable std::vector<std::uint32_t> order_;
};

}  // namespace scene
}  // namespace slipx

#endif  // SLIPX_SCENE_BROADPHASE_HPP
