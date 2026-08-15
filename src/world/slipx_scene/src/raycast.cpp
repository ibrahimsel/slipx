// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/scene/raycast.hpp"

#include <cmath>

namespace slipx {
namespace scene {
namespace {

// The offset direction at a centreline point: a unit left-hand normal, scaled
// so that multiplying it by a width lands the wall at exactly that
// perpendicular distance from BOTH segments meeting at the point.
//
// The scale is the part that is easy to leave out. Averaging the two
// segments' normals gives the bisector, which is the right direction, but
// moving along it by w puts the corner at w*cos(half-angle) from each edge
// rather than at w. On a track sampled every 0.1 m the two segments are
// nearly parallel and the difference is nothing; on a square it is 29 per
// cent, and the wall visibly cuts the corner. This is the standard mitre
// join, and the factor is 1 / (bisector . segment normal).
void offset_normal_at(const std::vector<CentrelinePoint>& points,
                      std::size_t i, bool closed, double& nx, double& ny) {
  const std::size_t count = points.size();

  // Left-hand normals of the incoming and outgoing segments. The left normal
  // of a direction (dx, dy) is (-dy, dx), a quarter turn anticlockwise, which
  // is the same sense of "left" the projection uses.
  double ax = 0.0, ay = 0.0;
  double bx = 0.0, by = 0.0;

  const bool has_incoming = closed || i > 0;
  const bool has_outgoing = closed || i + 1 < count;

  if (has_incoming) {
    const CentrelinePoint& previous = points[(i + count - 1) % count];
    const double dx = points[i].x - previous.x;
    const double dy = points[i].y - previous.y;
    const double length = std::sqrt(dx * dx + dy * dy);
    if (length > 0.0) { ax = -dy / length; ay = dx / length; }
  }
  if (has_outgoing) {
    const CentrelinePoint& next = points[(i + 1) % count];
    const double dx = next.x - points[i].x;
    const double dy = next.y - points[i].y;
    const double length = std::sqrt(dx * dx + dy * dy);
    if (length > 0.0) { bx = -dy / length; by = dx / length; }
  }

  // At an end of an open track there is only one segment, so the bisector is
  // that segment's normal and the mitre factor is 1.
  const double reference_x = has_incoming ? ax : bx;
  const double reference_y = has_incoming ? ay : by;

  nx = ax + bx;
  ny = ay + by;
  const double length = std::sqrt(nx * nx + ny * ny);

  // The two normals cancelled, which means the centreline reverses on itself
  // here. There is no offset that makes sense, so the wall follows the
  // outgoing segment and the fold is left visible rather than papered over
  // with a number that would be a guess.
  if (!(length > 1e-12)) {
    nx = reference_x;
    ny = reference_y;
    return;
  }

  nx /= length;
  ny /= length;

  const double cosine = nx * reference_x + ny * reference_y;
  if (cosine > 1e-6) {
    nx /= cosine;
    ny /= cosine;
  }
}

// Where a ray from (ox, oy) along (dx, dy) crosses the segment from (ax, ay)
// to (bx, by), as a distance along the ray, or a negative number for no
// crossing.
double ray_segment(double ox, double oy, double dx, double dy, double ax,
                   double ay, double bx, double by) {
  const double ex = bx - ax;
  const double ey = by - ay;

  const double denominator = dx * ey - dy * ex;
  if (denominator == 0.0) return -1.0;  // parallel, including collinear

  const double px = ax - ox;
  const double py = ay - oy;

  const double t = (px * ey - py * ex) / denominator;  // along the ray
  const double u = (px * dy - py * dx) / denominator;  // along the segment

  if (t < 0.0) return -1.0;              // behind the emitter
  if (u < 0.0 || u > 1.0) return -1.0;   // past an end of the wall
  return t;
}

}  // namespace

Walls::Walls(const Track& track) {
  const std::vector<CentrelinePoint>& points = track.centreline().points();
  const std::size_t count = points.size();
  closed_ = track.is_closed();

  left_x_.reserve(count);
  left_y_.reserve(count);
  right_x_.reserve(count);
  right_y_.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    double nx = 0.0, ny = 0.0;
    offset_normal_at(points, i, closed_, nx, ny);

    left_x_.push_back(points[i].x + nx * points[i].w_left);
    left_y_.push_back(points[i].y + ny * points[i].w_left);
    right_x_.push_back(points[i].x - nx * points[i].w_right);
    right_y_.push_back(points[i].y - ny * points[i].w_right);
  }
}

RayHit Walls::cast(double x, double y, double bearing, double max_range) const {
  const double dx = std::cos(bearing);
  const double dy = std::sin(bearing);

  RayHit best;
  double best_range = max_range;

  const std::size_t count = left_x_.size();
  const std::size_t segments = closed_ ? count : count - 1;

  for (std::size_t i = 0; i < segments; ++i) {
    const std::size_t j = (i + 1) % count;

    const double left = ray_segment(x, y, dx, dy, left_x_[i], left_y_[i],
                                    left_x_[j], left_y_[j]);
    if (left >= 0.0 && left < best_range) {
      best_range = left;
      best.hit = true;
      best.left_wall = true;
    }

    const double right = ray_segment(x, y, dx, dy, right_x_[i], right_y_[i],
                                     right_x_[j], right_y_[j]);
    if (right >= 0.0 && right < best_range) {
      best_range = right;
      best.hit = true;
      best.left_wall = false;
    }
  }

  best.range = best.hit ? best_range : 0.0;
  return best;
}

}  // namespace scene
}  // namespace slipx
