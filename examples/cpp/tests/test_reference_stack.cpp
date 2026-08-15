// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// A lap of the shipped track, headless, by each of the two reference
// controllers.
//
// This is the closest thing in the tree to an end-to-end test, and it is
// worth being clear about what passing it means. It does not mean the
// simulator is right. It means the pieces built in P1 are connected: the
// track loads, the walls are where the widths say, the projection agrees with
// the geometry, the LiDAR hits the walls at distances that mean something,
// the vehicle model turns steering into motion, and the lap counter notices
// when a lap has happened. Any one of those being wrong shows up here as a
// car in a wall.

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "reference_stack.hpp"
#include "slipx/scene/lap.hpp"
#include "slipx/scene/raycast.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/sense/lidar.hpp"
#include "slipx/sense/rng.hpp"
#include "slipx/vehicle_model.hpp"

namespace {

using slipx::DriveInput;
using slipx::Tier;
using slipx::VehicleModel;
using slipx::VehicleParams;
using slipx::VehicleState;
using slipx::examples::PurePursuit;
using slipx::examples::WallFollower;
using slipx::scene::Centreline;
using slipx::scene::LapCounter;
using slipx::scene::Track;
using slipx::scene::TrackManifest;
using slipx::scene::Walls;
using slipx::sense::Hit;
using slipx::sense::Lidar;
using slipx::sense::LidarSpec;
using slipx::sense::Pose;
using slipx::sense::Rng;
using slipx::sense::Scan;

Track shipped_track() {
  Centreline geometry = Centreline::from_file(
      std::string(SLIPX_EXAMPLE_TRACK_DIR) + "/centreline.csv");

  TrackManifest manifest;
  manifest.name = "paddock_stadium";
  manifest.surface = "carpet";
  manifest.closed = true;
  manifest.geometry_source = "examples/tracks/make_tracks.py";
  manifest.geometry_licence = "Apache-2.0";
  manifest.provenance_label = "provisional";

  return Track::build(geometry, manifest, {{"sponge", "carpet"},
                                           {"sponge", "carpet"}});
}

// The car starts on the centreline at the beginning of the bottom straight,
// pointing along it, which is where the generator put arc length zero.
VehicleState on_the_grid(double speed) {
  VehicleState state;
  state.pos.x = -4.0;
  state.pos.y = -3.0;
  state.yaw = 0.0;
  state.vel_body.x = speed;
  return state;
}

struct LapResult {
  int laps = 0;
  bool left_the_track = false;
  double worst_margin = 0.0;
  double seconds = 0.0;
};

TEST(ReferenceStack, PurePursuitDrivesALapOfTheShippedTrack) {
  const Track track = shipped_track();
  const VehicleParams params;
  const auto model = VehicleModel::create(Tier::L2_DoubleTrack, params);

  const double speed = 3.0;
  const PurePursuit controller(track, 0.6, params.lf + params.lr, speed);

  VehicleState state = on_the_grid(speed);
  LapCounter counter(track, 0.0);
  counter.update(state.pos.x, state.pos.y);

  LapResult result;
  const double dt = 1.0e-3;
  const int steps = 30000;  // thirty seconds, ample for a 34.85 m lap
  for (int i = 0; i < steps && counter.laps() < 1; ++i) {
    model->step(state, controller.drive(state), dt, nullptr);
    counter.update(state.pos.x, state.pos.y);
    result.seconds = (i + 1) * dt;
  }

  result.laps = counter.laps();
  result.left_the_track = counter.has_left_the_track();
  result.worst_margin = counter.worst_margin();

  EXPECT_EQ(result.laps, 1) << "no lap in " << result.seconds << " s";
  EXPECT_FALSE(result.left_the_track)
      << "worst margin " << result.worst_margin << " m";

  // A 34.85 m lap at 3 m/s is 11.6 s, and the controller is not fast, so
  // anything near that is the car having driven rather than cut the corner.
  EXPECT_GT(result.seconds, 10.0);
  EXPECT_LT(result.seconds, 20.0);
}

TEST(ReferenceStack, TheWallFollowerDrivesALapOnLidarAlone) {
  const Track track = shipped_track();
  const Walls walls(track);
  const VehicleParams params;
  const auto model = VehicleModel::create(Tier::L2_DoubleTrack, params);

  // A modest scan: enough rays that the two the controller wants exist, and
  // no noise, because this test is about the chain being connected rather
  // than about robustness to a noisy one.
  LidarSpec spec;
  spec.rate_hz = 40.0;
  spec.rays = 360;
  spec.range_max = 10.0;
  spec.noise_base_m = 0.0;
  spec.noise_per_metre = 0.0;
  spec.dropout_probability = 0.0;
  const Lidar lidar(spec);
  Rng rng(1);

  // The seam of ADR-0037: the scene's raycast, handed to the sensor as a
  // plain function. Neither component knows the other exists.
  const auto world = [&walls, &spec](const Pose& origin, double bearing) {
    Hit hit;
    const auto found = walls.cast(origin.x, origin.y, bearing, spec.range_max);
    hit.hit = found.hit;
    hit.range = found.range;
    return hit;
  };

  const double speed = 2.5;
  const WallFollower controller(0.75, speed);

  VehicleState state = on_the_grid(speed);
  LapCounter counter(track, 0.05);
  counter.update(state.pos.x, state.pos.y);

  const double dt = 1.0e-3;
  const double scan_period = 1.0 / spec.rate_hz;
  double next_scan_at = 0.0;
  DriveInput input;

  double seconds = 0.0;
  for (int i = 0; i < 40000 && counter.laps() < 1; ++i) {
    const double now = i * dt;

    if (now >= next_scan_at) {
      // The pose function is the car's current pose held constant across the
      // revolution. A stack that wanted the distortion would interpolate a
      // real pose history here; holding it is the simplification, and it is
      // stated rather than hidden.
      const VehicleState frozen = state;
      const auto pose_at = [frozen](double) {
        Pose pose;
        pose.x = frozen.pos.x;
        pose.y = frozen.pos.y;
        pose.yaw = frozen.yaw;
        return pose;
      };
      const Scan scan = lidar.sample(now, pose_at, world, rng);
      input = controller.drive(scan, state.vel_body.x);
      next_scan_at += scan_period;
    }

    model->step(state, input, dt, nullptr);
    counter.update(state.pos.x, state.pos.y);
    seconds = now + dt;
  }

  EXPECT_EQ(counter.laps(), 1) << "no lap in " << seconds << " s";
  // A wall follower on a 1.5 m corridor holding 0.75 m from one wall is
  // driving the centreline, so it should stay inside a corridor it is aiming
  // at the middle of. The tolerance is 5 cm, which is the transient at the
  // two corner entries and not a licence to leave the track.
  EXPECT_FALSE(counter.has_left_the_track())
      << "worst margin " << counter.worst_margin() << " m";
}

// Both controllers are deterministic functions of what they were given, so
// two runs of the same one are the same run. That is not a property of the
// controllers so much as a check that nothing in the chain beneath them
// reached for a clock.
TEST(ReferenceStack, ALapIsReproducible) {
  const Track track = shipped_track();
  const VehicleParams params;

  const auto run = [&] {
    const auto model = VehicleModel::create(Tier::L2_DoubleTrack, params);
    const PurePursuit controller(track, 0.6, params.lf + params.lr, 3.0);
    VehicleState state = on_the_grid(3.0);
    for (int i = 0; i < 5000; ++i) {
      model->step(state, controller.drive(state), 1.0e-3, nullptr);
    }
    return state;
  };

  const VehicleState one = run();
  const VehicleState two = run();

  EXPECT_DOUBLE_EQ(one.pos.x, two.pos.x);
  EXPECT_DOUBLE_EQ(one.pos.y, two.pos.y);
  EXPECT_DOUBLE_EQ(one.yaw, two.yaw);
}

}  // namespace
