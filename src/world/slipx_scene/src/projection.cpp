// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/scene/projection.hpp"

#include <cmath>
#include <stdexcept>

namespace slipx {
namespace scene {
namespace {

struct SegmentFit {
  double distance_squared;
  double t;
};

// The closest point on the segment from `a` to `b`, as a fraction along it.
// Clamped to the segment, so a point beyond either end projects to that end
// rather than off the line the segment lies on.
SegmentFit fit(const CentrelinePoint& a, const CentrelinePoint& b, double x,
               double y) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_squared = dx * dx + dy * dy;

  // The loader refuses coincident consecutive points, so this cannot be zero
  // for a Centreline that exists. The guard is here because the closing
  // segment of a closed track joins the last point to the first, and nothing
  // refuses a track whose ends coincide.
  double t = 0.0;
  if (length_squared > 0.0) {
    t = ((x - a.x) * dx + (y - a.y) * dy) / length_squared;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
  }

  const double px = a.x + t * dx;
  const double py = a.y + t * dy;
  const double ex = x - px;
  const double ey = y - py;
  return {ex * ex + ey * ey, t};
}

// Positive when (x, y) lies to the left of the direction from `a` to `b`.
// This is the z component of the cross product of the segment direction with
// the vector to the point, which is the same "left is positive" convention
// the rest of SlipX uses.
double side(const CentrelinePoint& a, const CentrelinePoint& b, double x,
            double y) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return dx * (y - a.y) - dy * (x - a.x);
}

double lerp(double a, double b, double t) { return a + t * (b - a); }

}  // namespace

Projection project(const Track& track, double x, double y) {
  const std::vector<CentrelinePoint>& points = track.centreline().points();
  const std::size_t count = points.size();
  const std::size_t segments = track.is_closed() ? count : count - 1;

  Projection best;
  double best_distance_squared = 0.0;
  bool have_best = false;

  for (std::size_t i = 0; i < segments; ++i) {
    const CentrelinePoint& a = points[i];
    const CentrelinePoint& b = points[(i + 1) % count];

    const SegmentFit candidate = fit(a, b, x, y);

    // Strictly less than, so an exact tie keeps the earlier segment. A tie
    // resolved by whichever comparison ran first is a different trajectory on
    // a different machine.
    if (!have_best || candidate.distance_squared < best_distance_squared) {
      have_best = true;
      best_distance_squared = candidate.distance_squared;
      best.segment = i;
      best.t = candidate.t;
    }
  }

  const CentrelinePoint& a = points[best.segment];
  const CentrelinePoint& b = points[(best.segment + 1) % count];

  // Arc length along the segment. The last segment of a closed track runs
  // from the final point back to the first, and its length is the closing
  // chord rather than a difference of two stored values.
  const double span = (best.segment + 1 == count)
                          ? track.centreline().closing_chord()
                          : b.s - a.s;
  best.s = a.s + best.t * span;

  // The signed distance, from the unsigned one the search produced. Taking
  // the magnitude from the search and the sign from the cross product keeps
  // the two consistent even when the point projects onto a segment end,
  // where the perpendicular distance and the cross product disagree.
  const double magnitude = std::sqrt(best_distance_squared);
  best.lateral = side(a, b, x, y) < 0.0 ? -magnitude : magnitude;

  return best;
}

Widths widths_at(const Track& track, const Projection& where) {
  const std::vector<CentrelinePoint>& points = track.centreline().points();
  const std::size_t count = points.size();
  const CentrelinePoint& a = points[where.segment];
  const CentrelinePoint& b = points[(where.segment + 1) % count];

  return {lerp(a.w_left, b.w_left, where.t),
          lerp(a.w_right, b.w_right, where.t)};
}

LimitStatus check_limits(const Track& track, const Projection& where,
                         double tolerance) {
  if (!std::isfinite(tolerance)) {
    throw std::invalid_argument(
        "track limit tolerance must be a finite number of metres. Zero is a "
        "rule, not an absence, so there is no default to fall back on.");
  }

  const Widths widths = widths_at(track, where);

  // Left is positive, so a car to the left is measured against the left
  // width. The margin is the distance still available before the nearer edge,
  // which is the number somebody adjudicating a run wants, rather than a bare
  // flag saying it was close.
  const double edge = where.lateral >= 0.0 ? widths.left : widths.right;
  const double margin = edge + tolerance - std::fabs(where.lateral);

  LimitStatus status;
  status.margin = margin;
  status.inside = margin >= 0.0;
  return status;
}

}  // namespace scene
}  // namespace slipx
