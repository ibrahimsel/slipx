// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The reference stack: a wall follower and a pure pursuit controller.
//
// These exist to validate the simulator, not to win anything, and the
// distinction is worth keeping in front of the reader. A competitive
// F1TENTH stack plans a racing line, brakes on a friction budget and
// overtakes; these two do none of that. What they do is exercise the
// simulator end to end, which is a different job and one they are better at
// for being simple enough to reason about when they fail.
//
// The pair is chosen so that the two consume different halves of what P1
// built. The wall follower sees only a LiDAR scan, so a lap of it says the
// sensor chain is coherent: rays hit the walls the track implies, at
// distances that mean something, often enough to steer on. Pure pursuit
// takes ground truth against the centreline, so a lap of it says the
// geometry and the vehicle model agree about where the car is. When one lap
// fails and the other passes, the failure is already half localised.
//
// Both are header-only and live in examples/ rather than in a component,
// because nothing in the library should depend on them.

#ifndef SLIPX_EXAMPLES_REFERENCE_STACK_HPP
#define SLIPX_EXAMPLES_REFERENCE_STACK_HPP

#include <cmath>
#include <cstddef>
#include <limits>

#include "slipx/scene/projection.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/sense/lidar.hpp"
#include "slipx/state.hpp"

namespace slipx {
namespace examples {

// Wrap an angle to (-pi, pi].
inline double wrap_angle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle <= -kPi) angle += 2.0 * kPi;
  return angle;
}

// ---------------------------------------------------------- pure pursuit
//
// The textbook geometric controller: pick a point on the path a fixed
// distance ahead, and steer along the arc that reaches it. The steering law
// is exact for a kinematic bicycle, which is why it works at all on a car
// that is not one, and why it understeers as the tyres start to matter. That
// is the correct behaviour to see from it and not a defect to tune out.
class PurePursuit {
 public:
  PurePursuit(const scene::Track& track, double lookahead_m, double wheelbase_m,
              double target_speed)
      : track_(&track),
        lookahead_(lookahead_m),
        wheelbase_(wheelbase_m),
        target_speed_(target_speed) {}

  DriveInput drive(const VehicleState& state) const {
    const scene::Projection here =
        scene::project(*track_, state.pos.x, state.pos.y);

    // The point a lookahead ahead along the centreline. Along the centreline
    // rather than "the first path point more than L away", because the second
    // formulation picks a point behind the car whenever the path doubles
    // back, and a stadium's ends do exactly that.
    const double target_s = here.s + lookahead_;
    const auto goal = point_at(target_s);

    const double dx = goal.first - state.pos.x;
    const double dy = goal.second - state.pos.y;

    // Into the body frame, where the geometry is the textbook one.
    const double cos_yaw = std::cos(state.yaw);
    const double sin_yaw = std::sin(state.yaw);
    const double forward = dx * cos_yaw + dy * sin_yaw;
    const double left = -dx * sin_yaw + dy * cos_yaw;

    const double distance = std::sqrt(forward * forward + left * left);
    double steer = 0.0;
    if (distance > 1e-9) {
      // The curvature that reaches the goal point, and the steer angle a
      // bicycle of this wheelbase needs to hold it.
      const double curvature = 2.0 * left / (distance * distance);
      steer = std::atan(wheelbase_ * curvature);
    }

    DriveInput input;
    input.steer_cmd = steer;
    input.accel_cmd = 2.0 * (target_speed_ - state.vel_body.x);
    return input;
  }

 private:
  // The centreline point at an arc length, wrapping on a closed track.
  std::pair<double, double> point_at(double s) const {
    const auto& points = track_->centreline().points();
    const std::size_t count = points.size();
    const double length = track_->length();

    if (track_->is_closed()) {
      s = std::fmod(s, length);
      if (s < 0.0) s += length;
    } else {
      if (s < 0.0) s = 0.0;
      if (s > points.back().s) s = points.back().s;
    }

    // Linear scan. A binary search would be faster and this is not the
    // measurement that matters yet.
    for (std::size_t i = 0; i + 1 < count; ++i) {
      if (s <= points[i + 1].s) {
        const double span = points[i + 1].s - points[i].s;
        const double t = span > 0.0 ? (s - points[i].s) / span : 0.0;
        return {points[i].x + t * (points[i + 1].x - points[i].x),
                points[i].y + t * (points[i + 1].y - points[i].y)};
      }
    }

    // On the closing segment, from the last point back to the first.
    const double span = length - points.back().s;
    const double t = span > 0.0 ? (s - points.back().s) / span : 0.0;
    return {points.back().x + t * (points.front().x - points.back().x),
            points.back().y + t * (points.front().y - points.back().y)};
  }

  const scene::Track* track_;
  double lookahead_;
  double wheelbase_;
  double target_speed_;
};

// ----------------------------------------------------------- wall follower
//
// Sees nothing but a scan. The classic F1TENTH construction: two rays to one
// side, one square to the car and one swept forward, give the angle of the
// wall relative to the car by trigonometry, and from that the distance to the
// wall a little way ahead rather than the distance to it now. Steering on the
// distance now is the version that oscillates, because by the time the error
// is visible the car is already alongside it.
class WallFollower {
 public:
  WallFollower(double target_distance_m, double target_speed)
      : target_(target_distance_m), target_speed_(target_speed) {}

  // `scan` is a scan from the car's own LiDAR. Rays that were dropped or out
  // of range arrive as NaN and are skipped rather than being read as zero,
  // which would be a wall against the mast.
  DriveInput drive(const sense::Scan& scan, double speed) const {
    constexpr double kPi = 3.14159265358979323846;

    // Follow the right-hand wall: square to the car is -90 degrees, and the
    // swept ray is 40 degrees forward of it.
    const double square_angle = -kPi / 2.0;
    const double swept_angle = square_angle + 0.6981317;  // 40 degrees

    const double b = range_at(scan, square_angle);
    const double a = range_at(scan, swept_angle);

    DriveInput input;
    input.accel_cmd = 2.0 * (target_speed_ - speed);

    if (!std::isfinite(a) || !std::isfinite(b)) {
      // No wall on that side within range. Hold the wheel straight rather
      // than inventing a distance; a controller that guesses here is one that
      // turns hard into a gap.
      input.steer_cmd = 0.0;
      return input;
    }

    const double theta = 0.6981317;

    // The wall's angle relative to the car, from the two ranges.
    const double alpha =
        std::atan2(a * std::cos(theta) - b, a * std::sin(theta));

    // Perpendicular distance now, and a little way ahead. Projecting forward
    // is what stops the loop oscillating: without it the car reacts to an
    // error only once it is already beside it.
    const double distance_now = b * std::cos(alpha);
    const double lookahead = 0.35;
    const double distance_ahead = distance_now + lookahead * std::sin(alpha);

    // Following the right wall, so a distance larger than the target means
    // the car has drifted left and must steer right. Positive steer is left
    // (ISO 8855), so the correction carries the sign of the error as written
    // and is not negated: too far from the wall gives a negative error and a
    // right-hand correction.
    const double error = target_ - distance_ahead;
    input.steer_cmd = 1.2 * error;
    return input;
  }

 private:
  // The range of the valid ray closest to a bearing.
  static double range_at(const sense::Scan& scan, double angle) {
    double best = std::numeric_limits<double>::quiet_NaN();
    double best_gap = std::numeric_limits<double>::infinity();
    for (const sense::Ray& ray : scan.rays) {
      if (!ray.valid) continue;
      const double gap = std::fabs(wrap_angle(ray.angle - angle));
      if (gap < best_gap) {
        best_gap = gap;
        best = ray.range;
      }
    }
    return best;
  }

  double target_;
  double target_speed_;
};

}  // namespace examples
}  // namespace slipx

#endif  // SLIPX_EXAMPLES_REFERENCE_STACK_HPP
