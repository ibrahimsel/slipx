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
  //
  // Const but not thread-safe: it writes the scratch stamps declared below.
  // One Walls per thread if two threads ever cast at once.
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
  std::size_t cell_count() const {
    return cell_start_.empty() ? 0 : cell_start_.size() - 1;
  }

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
  //
  // The traversal tests each cell's segments as it reaches that cell, rather
  // than collecting every candidate along the ray and testing the lot at the
  // end, so that a hit inside the current cell ends the walk. That is also
  // measured: it is most of what took the sensing from a 90x single agent to
  // one comfortably past the P1 target.
  // A segment as the intersection test wants it rather than as the wall
  // describes it: the start point and the edge vector, so the subtraction
  // happens once at construction instead of once per ray per segment. Four
  // doubles, so two segments to a cache line.
  struct Segment {
    double ax, ay, ex, ey;
  };

  void build_index();

  std::vector<double> left_x_, left_y_;
  std::vector<double> right_x_, right_y_;
  bool closed_ = false;

  std::vector<Segment> segments_;

  // Which wall each segment belongs to, kept apart from the geometry because
  // a ray reads it at most once, on the segment it finally hits, and putting
  // it in Segment would cost every segment it tests a wider stride.
  std::vector<std::uint8_t> segment_left_;

  // Grid geometry, and the grid itself in compressed form: cell_items_ holds
  // every cell's segment indices end to end, and cell_start_ says where each
  // cell's run begins, with a tail entry so the last cell needs no special
  // case.
  //
  // A vector of vectors is the obvious shape and costs a pointer chase and a
  // likely cache miss per cell. A scan walks a handful of cells per ray and
  // several hundred thousand rays a second, so that pointer chase is a real
  // fraction of the work rather than a tidiness argument.
  double origin_x_ = 0.0, origin_y_ = 0.0;
  double cell_size_ = 1.0;
  std::size_t nx_ = 1, ny_ = 1;
  std::vector<std::uint32_t> cell_start_;
  std::vector<std::uint32_t> cell_items_;

  // Stamps, so a segment reached through two cells is tested once. Mutable
  // because it is scratch space for a const query and holds no answer.
  //
  // Only one segment in nine that a ray is offered is a repeat, so this looks
  // like bookkeeping that costs more than it saves. Measured, it is the other
  // way round by about a seventh: a stamp is a load and a compare against a
  // line that is already hot, and the test it skips is six multiplies and a
  // division.
  mutable std::vector<std::uint32_t> stamp_;
  mutable std::uint32_t visit_ = 0;
};

}  // namespace scene
}  // namespace slipx

#endif  // SLIPX_SCENE_RAYCAST_HPP
