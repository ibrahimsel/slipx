// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/race/starts.hpp"

#include <algorithm>
#include <cmath>

namespace slipx {
namespace race {

TrackPose pose_at(const scene::Track& track, double s) {
  const auto& points = track.centreline().points();
  const double total = track.length();

  double sm = s;
  if (track.is_closed()) {
    sm = std::fmod(sm, total);
    if (sm < 0.0) sm += total;
  } else {
    sm = std::max(0.0, std::min(sm, total));
  }

  // The segment holding sm: points carry cumulative arc length, and on a
  // closed track the wrap segment (last point back to the first) covers
  // [points.back().s, total).
  const std::size_t count = points.size();
  std::size_t i = 0;
  {
    // First point with .s strictly greater than sm, then step back one.
    std::size_t lo = 0, hi = count;
    while (lo < hi) {
      const std::size_t mid = lo + (hi - lo) / 2;
      if (points[mid].s <= sm) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    i = (lo == 0) ? 0 : lo - 1;
  }

  std::size_t j;
  if (i + 1 < count) {
    j = i + 1;
  } else if (track.is_closed()) {
    j = 0;   // the wrap segment
  } else {
    // The very end of an open track: stand on the last segment's end.
    i = count - 2;
    j = count - 1;
  }

  const double seg_start = points[i].s;
  const double seg_len = (j == 0) ? (total - seg_start)
                                  : (points[j].s - seg_start);
  const double t =
      seg_len > 0.0 ? std::min(1.0, (sm - seg_start) / seg_len) : 0.0;

  const double dx = points[j].x - points[i].x;
  const double dy = points[j].y - points[i].y;

  TrackPose pose;
  pose.x = points[i].x + t * dx;
  pose.y = points[i].y + t * dy;
  pose.heading = std::atan2(dy, dx);
  return pose;
}

void place_on_track(sim::Simulation& sim, std::size_t agent,
                    const scene::Track& track, double s, double lateral,
                    double speed) {
  const TrackPose pose = pose_at(track, s);
  // Left of the direction of travel is the left normal, a quarter turn
  // anticlockwise, the same "left" the projection reports.
  const double nx = -std::sin(pose.heading);
  const double ny = std::cos(pose.heading);

  VehicleState& state = sim.state(agent);
  state.pos.x = pose.x + lateral * nx;
  state.pos.y = pose.y + lateral * ny;
  state.pos.z = 0.0;
  state.yaw = pose.heading;
  state.vel_body = Vec3{speed, 0.0, 0.0};
  state.rates = Vec3{};
  state.steer = 0.0;
  state.steer_rate = 0.0;
  for (double& lag : state.alpha_lag) lag = 0.0;
  const double radius = sim.model(agent).params().wheel_radius;
  for (double& omega : state.omega_w) omega = speed / radius;
}

void grid_start(sim::Simulation& sim, const scene::Track& track,
                std::size_t left_car, std::size_t right_car, double line_s,
                double gap) {
  // Side by side at the line, one car width apart (2.5.1.9.1-2): half the
  // gap to each side of the centreline, at rest.
  place_on_track(sim, left_car, track, line_s, 0.5 * gap, 0.0);
  place_on_track(sim, right_car, track, line_s, -0.5 * gap, 0.0);
}

void rolling_start(sim::Simulation& sim, const scene::Track& track,
                   std::size_t left_car, std::size_t right_car, double line_s,
                   double gap, double speed) {
  place_on_track(sim, left_car, track, line_s, 0.5 * gap, speed);
  place_on_track(sim, right_car, track, line_s, -0.5 * gap, speed);
}

}  // namespace race
}  // namespace slipx
