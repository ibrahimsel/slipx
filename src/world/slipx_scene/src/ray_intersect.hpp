// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The one ray-against-segment intersection, shared by the grid index
// (raycast.cpp) and the BVH (broadphase.cpp) so the two accelerators cannot
// drift apart on the arithmetic they are both accelerating. Internal to
// slipx_scene: this header lives in src/ and is not installed.

#ifndef SLIPX_SCENE_SRC_RAY_INTERSECT_HPP
#define SLIPX_SCENE_SRC_RAY_INTERSECT_HPP

namespace slipx {
namespace scene {
namespace detail {

// Where a ray from (ox, oy) along (dx, dy) crosses the segment starting at
// (ax, ay) with edge vector (ex, ey), as a distance along the ray, or a
// negative number for no crossing. `limit` is the distance beyond which the
// caller has already found something nearer and does not care.
//
// The two divisions are ordered rather than paired: most of the segments a
// ray is offered are behind it or further away than the wall it has already
// found, and those are rejected on t alone and never pay for u. `limit` is
// folded in here so that the rejection happens before the second division
// rather than after it.
inline double ray_segment(double ox, double oy, double dx, double dy,
                          double ax, double ay, double ex, double ey,
                          double limit) {
  const double denominator = dx * ey - dy * ex;
  if (denominator == 0.0) return -1.0;  // parallel, including collinear

  const double px = ax - ox;
  const double py = ay - oy;

  const double t = (px * ey - py * ex) / denominator;  // along the ray
  if (t < 0.0) return -1.0;              // behind the emitter
  if (t >= limit) return -1.0;           // something nearer is already known

  const double u = (px * dy - py * dx) / denominator;  // along the segment
  if (u < 0.0 || u > 1.0) return -1.0;   // past an end of the wall
  return t;
}

}  // namespace detail
}  // namespace scene
}  // namespace slipx

#endif  // SLIPX_SCENE_SRC_RAY_INTERSECT_HPP
