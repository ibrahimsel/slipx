// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Casting a ray at a track.
//
// The walls of a track in this representation are not stored anywhere: the
// centreline carries a width to each side, so the left wall is the polyline
// offset by w_left and the right wall the polyline offset by w_right, both
// implied. This builds those two polylines once and intersects rays against
// them.
//
// Built once and reused, rather than offsetting on the fly, because a wall is
// a property of the track and not of the ray.
//
// There is a spatial index, and there is one because a measurement asked for
// it rather than because it seemed likely. Testing every segment for every
// ray put a 1080-ray scan at three quarters of a million intersection tests,
// and a single car with a LiDAR at sixteen times real time against a P1
// target of a hundred. The index is a uniform grid, which is the least clever
// structure that fixes it; the racing phase has a broadphase of its own to
// build later and this is deliberately not that.
//
// This is the geometry half of ADR-0037: sensing never includes this header,
// and gets at it through a function the orchestrator supplies.

#ifndef SLIPX_SCENE_RAYCAST_HPP
#define SLIPX_SCENE_RAYCAST_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include "slipx/scene/track.hpp"

namespace slipx {
namespace scene {

// What a ray found, in the terms slipx_sense's world function needs. The
// duplication of shape with sense::Hit is deliberate: the two components do
// not include each other, so they do not share a type either, and the
// orchestrator translates between them in one place.
struct RayHit {
  bool hit = false;
  double range = 0.0;  // distance from the origin to the wall          [m]

  // Which wall, for a caller that wants to know: true for the left-hand
  // wall, in the sense that "left" has everywhere else in SlipX.
  bool left_wall = false;
};

// The two walls of a track, as polylines, built once.
class Walls {
 public:
  explicit Walls(const Track& track);

  // Distance to the first wall a ray meets, or a miss.
  //
  // `max_range` bounds the search. A miss is a genuine answer here: a ray
  // down a straight on an open track leaves the end of the walls and hits
  // nothing, and reporting a range of zero for it would be a wall against
  // the sensor.
  RayHit cast(double x, double y, double bearing, double max_range) const;

  // The same query, testing every wall segment. Kept because it is the
  // definition of the answer: the accelerated cast above is only correct
  // insofar as it agrees with this, and the tests assert that over thousands
  // of rays rather than trusting the traversal.
  RayHit cast_brute_force(double x, double y, double bearing,
                          double max_range) const;

  const std::vector<double>& left_x() const { return left_x_; }
  const std::vector<double>& left_y() const { return left_y_; }
  const std::vector<double>& right_x() const { return right_x_; }
  const std::vector<double>& right_y() const { return right_y_; }

  bool closed() const { return closed_; }

  // Diagnostic, for the benchmark and for anybody wondering whether the
  // index is doing anything.
  std::size_t cell_count() const { return cells_.size(); }

 private:
  // A uniform grid over the walls, built once.
  //
  // Measured, not guessed. Before it, a 1080-ray scan against this track
  // tested three quarters of a million segment intersections and a single
  // car with a LiDAR ran at 16 times real time against a target of 100. A
  // grid is the least clever structure that fixes that, which is what makes
  // it the right one here: a BVH would be faster still and would have to be
  // right about more things, and the racing phase has a broadphase of its
  // own to build later.
  struct Segment {
    double ax, ay, bx, by;
    bool left_wall;
  };

  void build_index();
  void gather(double x, double y, double dx, double dy, double max_range,
              std::vector<std::uint32_t>& candidates) const;

  std::vector<double> left_x_, left_y_;
  std::vector<double> right_x_, right_y_;
  bool closed_ = false;

  std::vector<Segment> segments_;

  // Grid geometry. cells_ holds, for each cell, the segments whose bounding
  // box overlaps it.
  double origin_x_ = 0.0, origin_y_ = 0.0;
  double cell_size_ = 1.0;
  std::size_t nx_ = 1, ny_ = 1;
  std::vector<std::vector<std::uint32_t>> cells_;

  // Stamps, so a segment reached through two cells is tested once. Mutable
  // because it is scratch space for a const query and holds no answer.
  mutable std::vector<std::uint32_t> stamp_;
  mutable std::uint32_t visit_ = 0;
};

}  // namespace scene
}  // namespace slipx

#endif  // SLIPX_SCENE_RAYCAST_HPP
