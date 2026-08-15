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
// a property of the track and not of the ray. It is also where a spatial
// index would go when there is a measurement saying one is needed; there is
// not, and a broadphase built on a guess is a broadphase whose bugs nobody
// has a reason to look for (M7.4 has the racing version of this problem).
//
// This is the geometry half of ADR-0037: sensing never includes this header,
// and gets at it through a function the orchestrator supplies.

#ifndef SLIPX_SCENE_RAYCAST_HPP
#define SLIPX_SCENE_RAYCAST_HPP

#include <cstddef>
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

  const std::vector<double>& left_x() const { return left_x_; }
  const std::vector<double>& left_y() const { return left_y_; }
  const std::vector<double>& right_x() const { return right_x_; }
  const std::vector<double>& right_y() const { return right_y_; }

  bool closed() const { return closed_; }

 private:
  std::vector<double> left_x_, left_y_;
  std::vector<double> right_x_, right_y_;
  bool closed_ = false;
};

}  // namespace scene
}  // namespace slipx

#endif  // SLIPX_SCENE_RAYCAST_HPP
