// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/scene/raycast.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

// The grid cell an ordinate falls in, clamped to the grid.
std::size_t cell_of(double value, double origin, double size,
                    std::size_t count) {
  if (value <= origin) return 0;
  const std::size_t index = static_cast<std::size_t>((value - origin) / size);
  return index >= count ? count - 1 : index;
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

  build_index();
}

void Walls::build_index() {
  const std::size_t count = left_x_.size();
  const std::size_t spans = closed_ ? count : count - 1;

  segments_.reserve(spans * 2);
  for (std::size_t i = 0; i < spans; ++i) {
    const std::size_t j = (i + 1) % count;
    segments_.push_back(
        Segment{left_x_[i], left_y_[i], left_x_[j], left_y_[j], true});
    segments_.push_back(
        Segment{right_x_[i], right_y_[i], right_x_[j], right_y_[j], false});
  }

  if (segments_.empty()) return;

  double min_x = segments_[0].ax, max_x = segments_[0].ax;
  double min_y = segments_[0].ay, max_y = segments_[0].ay;
  double total_length = 0.0;
  for (const Segment& s : segments_) {
    min_x = std::fmin(min_x, std::fmin(s.ax, s.bx));
    max_x = std::fmax(max_x, std::fmax(s.ax, s.bx));
    min_y = std::fmin(min_y, std::fmin(s.ay, s.by));
    max_y = std::fmax(max_y, std::fmax(s.ay, s.by));
    total_length += std::sqrt((s.bx - s.ax) * (s.bx - s.ax) +
                              (s.by - s.ay) * (s.by - s.ay));
  }

  // A cell a few segments across. Smaller cells mean fewer segments tested
  // per cell and more cells walked, and the product has a broad minimum, so
  // this is not a number worth tuning: it is worth being roughly right and
  // not being zero.
  const double mean_length =
      total_length / static_cast<double>(segments_.size());
  cell_size_ = std::fmax(mean_length * 4.0, 1e-6);

  origin_x_ = min_x - cell_size_;
  origin_y_ = min_y - cell_size_;

  const double width = (max_x - min_x) + 2.0 * cell_size_;
  const double height = (max_y - min_y) + 2.0 * cell_size_;

  nx_ = static_cast<std::size_t>(width / cell_size_) + 1;
  ny_ = static_cast<std::size_t>(height / cell_size_) + 1;

  // A track sampled very finely would otherwise ask for a grid with more
  // cells than there are segments to put in them, which costs memory and
  // buys nothing.
  const std::size_t kMaxCells = 1u << 20;
  while (nx_ * ny_ > kMaxCells) {
    cell_size_ *= 2.0;
    nx_ = static_cast<std::size_t>(width / cell_size_) + 1;
    ny_ = static_cast<std::size_t>(height / cell_size_) + 1;
  }

  cells_.assign(nx_ * ny_, {});

  // Each segment goes into every cell its bounding box touches. Conservative,
  // so a ray never misses a segment it should have tested; the cost is that a
  // long diagonal segment is registered in cells it does not actually cross,
  // which is a few extra intersection tests and never a wrong answer.
  for (std::uint32_t index = 0; index < segments_.size(); ++index) {
    const Segment& s = segments_[index];
    const std::size_t x0 = cell_of(std::fmin(s.ax, s.bx), origin_x_, cell_size_, nx_);
    const std::size_t x1 = cell_of(std::fmax(s.ax, s.bx), origin_x_, cell_size_, nx_);
    const std::size_t y0 = cell_of(std::fmin(s.ay, s.by), origin_y_, cell_size_, ny_);
    const std::size_t y1 = cell_of(std::fmax(s.ay, s.by), origin_y_, cell_size_, ny_);
    for (std::size_t cy = y0; cy <= y1; ++cy) {
      for (std::size_t cx = x0; cx <= x1; ++cx) {
        cells_[cy * nx_ + cx].push_back(index);
      }
    }
  }

  stamp_.assign(segments_.size(), 0);
}

void Walls::gather(double x, double y, double dx, double dy, double max_range,
                   std::vector<std::uint32_t>& candidates) const {
  candidates.clear();
  if (cells_.empty()) return;

  ++visit_;
  if (visit_ == 0) {  // the stamp wrapped, so start the numbering again
    std::fill(stamp_.begin(), stamp_.end(), 0);
    visit_ = 1;
  }

  // Grid traversal, the Amanatides and Woo way: step from cell boundary to
  // cell boundary and visit each cell the ray passes through, exactly once.
  //
  // The tempting shortcut is to sample the ray every half cell and collect
  // whatever is under each sample. It is shorter and it is wrong: a ray
  // crossing a corner clips a cell for less than a sample interval and the
  // sample never lands in it, so the wall in that cell is invisible. It fails
  // on a few rays out of thousands, at angles that depend on the grid, which
  // is the worst possible way for a raycast to be wrong.
  const double far_x = origin_x_ + static_cast<double>(nx_) * cell_size_;
  const double far_y = origin_y_ + static_cast<double>(ny_) * cell_size_;

  // Clip the ray to the grid. A ray can start well outside the track and
  // cross it, so where it enters is not always where it starts.
  double enter = 0.0;
  double leave = max_range;

  const auto clip = [&](double start, double direction, double lo, double hi) {
    if (std::fabs(direction) < 1e-300) {
      if (start < lo || start > hi) leave = -1.0;  // parallel and outside
      return;
    }
    double t0 = (lo - start) / direction;
    double t1 = (hi - start) / direction;
    if (t0 > t1) std::swap(t0, t1);
    enter = std::fmax(enter, t0);
    leave = std::fmin(leave, t1);
  };

  clip(x, dx, origin_x_, far_x);
  clip(y, dy, origin_y_, far_y);

  if (leave < enter) return;  // the ray never touches the grid

  // Start exactly at the entry point, with no nudge along the ray.
  //
  // A nudge looks like it avoids a boundary ambiguity and instead creates
  // one. A ray that begins precisely on a cell boundary, which is what a ray
  // from a wall corner does, gets pushed into the next cell along and the
  // cell it actually starts in is never visited, so the wall it is touching
  // is invisible. Landing on a boundary is not ambiguous here: whichever side
  // the floor picks, the first step is of zero length and visits the other.
  const double start_t = enter;
  const double px = x + dx * start_t;
  const double py = y + dy * start_t;

  auto cx = static_cast<std::ptrdiff_t>((px - origin_x_) / cell_size_);
  auto cy = static_cast<std::ptrdiff_t>((py - origin_y_) / cell_size_);
  if (cx < 0) cx = 0;
  if (cy < 0) cy = 0;
  if (cx >= static_cast<std::ptrdiff_t>(nx_)) cx = static_cast<std::ptrdiff_t>(nx_) - 1;
  if (cy >= static_cast<std::ptrdiff_t>(ny_)) cy = static_cast<std::ptrdiff_t>(ny_) - 1;

  const std::ptrdiff_t step_x = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
  const std::ptrdiff_t step_y = dy > 0.0 ? 1 : (dy < 0.0 ? -1 : 0);

  const double huge = std::numeric_limits<double>::infinity();

  // Distance along the ray to the next boundary in each axis, and the
  // distance between consecutive boundaries.
  double next_x = huge;
  double delta_x = huge;
  if (step_x != 0) {
    const double boundary =
        origin_x_ + static_cast<double>(cx + (step_x > 0 ? 1 : 0)) * cell_size_;
    next_x = start_t + (boundary - px) / dx;
    delta_x = cell_size_ / std::fabs(dx);
  }

  double next_y = huge;
  double delta_y = huge;
  if (step_y != 0) {
    const double boundary =
        origin_y_ + static_cast<double>(cy + (step_y > 0 ? 1 : 0)) * cell_size_;
    next_y = start_t + (boundary - py) / dy;
    delta_y = cell_size_ / std::fabs(dy);
  }

  double travelled = start_t;
  while (travelled <= leave) {
    for (std::uint32_t index :
         cells_[static_cast<std::size_t>(cy) * nx_ + static_cast<std::size_t>(cx)]) {
      if (stamp_[index] == visit_) continue;
      stamp_[index] = visit_;
      candidates.push_back(index);
    }

    if (next_x < next_y) {
      cx += step_x;
      travelled = next_x;
      next_x += delta_x;
    } else {
      cy += step_y;
      travelled = next_y;
      next_y += delta_y;
    }

    if (cx < 0 || cy < 0 || cx >= static_cast<std::ptrdiff_t>(nx_) ||
        cy >= static_cast<std::ptrdiff_t>(ny_)) {
      break;
    }
  }
}

RayHit Walls::cast(double x, double y, double bearing, double max_range) const {
  const double dx = std::cos(bearing);
  const double dy = std::sin(bearing);

  static thread_local std::vector<std::uint32_t> candidates;
  gather(x, y, dx, dy, max_range, candidates);

  RayHit best;
  double best_range = max_range;

  for (std::uint32_t index : candidates) {
    const Segment& s = segments_[index];
    const double t = ray_segment(x, y, dx, dy, s.ax, s.ay, s.bx, s.by);
    if (t >= 0.0 && t < best_range) {
      best_range = t;
      best.hit = true;
      best.left_wall = s.left_wall;
    }
  }

  best.range = best.hit ? best_range : 0.0;
  return best;
}

RayHit Walls::cast_brute_force(double x, double y, double bearing,
                               double max_range) const {
  const double dx = std::cos(bearing);
  const double dy = std::sin(bearing);

  RayHit best;
  double best_range = max_range;

  for (const Segment& s : segments_) {
    const double t = ray_segment(x, y, dx, dy, s.ax, s.ay, s.bx, s.by);
    if (t >= 0.0 && t < best_range) {
      best_range = t;
      best.hit = true;
      best.left_wall = s.left_wall;
    }
  }

  best.range = best.hit ? best_range : 0.0;
  return best;
}

}  // namespace scene
}  // namespace slipx
