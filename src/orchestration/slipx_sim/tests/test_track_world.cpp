// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The track world (ADR-0049): the walls and the simulation's own cars,
// composed into the rig's world function. The claims under test: a ray is
// answered by the nearer of wall and car, the asker is skipped, the box is
// the contact pass's box (axle-centred, so an asymmetric car's offset
// shows), a car with no footprint is invisible while a wreck is not, rays
// within one step see one consistent world, and a world missing a car
// refuses to answer rather than hiding an obstacle.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "slipx/scene/track.hpp"
#include "slipx/sim/mailbox.hpp"
#include "slipx/sim/sensor_rig.hpp"
#include "slipx/sim/simulation.hpp"
#include "slipx/sim/track_world.hpp"

namespace {

using slipx::DriveInput;
using slipx::Tier;
using slipx::VehicleState;
using slipx::scene::Track;
using slipx::sense::Hit;
using slipx::sense::Pose;
using slipx::sim::Simulation;
using slipx::sim::TrackWorld;

constexpr double kHalfPi = 1.57079632679489661923;

Track shipped_track() {
  slipx::scene::Centreline geometry = slipx::scene::Centreline::from_file(
      std::string(SLIPX_EXAMPLE_TRACK_DIR) + "/centreline.csv");
  slipx::scene::TrackManifest manifest;
  manifest.name = "paddock_stadium";
  manifest.surface = "carpet";
  manifest.closed = true;
  manifest.geometry_source = "examples/tracks/make_tracks.py";
  manifest.geometry_licence = "Apache-2.0";
  manifest.provenance_label = "provisional";
  return Track::build(geometry, manifest, {{"sponge", "carpet"}});
}

// A car on the bottom straight (which runs along y = -3, heading +x), with
// the usual 0.50 by 0.30 footprint. The default parameters have the CoG
// mid-wheelbase, so the box centre offset is zero unless the test moves it.
slipx::sim::AgentSpec car_at(double x, double vx = 0.0) {
  slipx::sim::AgentSpec spec;
  spec.tier = Tier::L1_Bicycle;
  spec.initial_state.pos.x = x;
  spec.initial_state.pos.y = -3.0;
  spec.initial_state.vel_body.x = vx;
  spec.footprint_length = 0.50;
  spec.footprint_width = 0.30;
  spec.policy = [](const VehicleState&, double, slipx::sense::Rng&) {
    return DriveInput{};
  };
  return spec;
}

Pose pose_at(double x) {
  Pose pose;
  pose.x = x;
  pose.y = -3.0;
  pose.yaw = 0.0;
  return pose;
}

TEST(TrackWorld, TheNearerOfWallAndCarAnswers) {
  const Track track = shipped_track();
  Simulation sim;
  sim.add_agent(car_at(-4.0));
  sim.add_agent(car_at(-2.0));  // two metres ahead of the asker
  const TrackWorld world(track, sim, 30.0);

  // Straight ahead: the opponent's rear face, at the gap minus its half
  // length, long before the wall at the end of the straight.
  const Hit ahead = world(0, pose_at(-4.0), 0.0);
  ASSERT_TRUE(ahead.hit);
  EXPECT_NEAR(ahead.range, 2.0 - 0.25, 1.0e-9);

  // Sideways: the wall at the half width, with no car in the way. The
  // asker sits on the centreline of a 1.5 m corridor. The wall is a
  // polyline offset from a 0.1 m-sampled centreline, so near the corner
  // transition it sits a tenth of a millimetre off the nominal width;
  // the tolerance is the discretisation, not slack in the cast.
  const Hit left = world(0, pose_at(-4.0), kHalfPi);
  ASSERT_TRUE(left.hit);
  EXPECT_NEAR(left.range, 0.75, 1.0e-3);
}

TEST(TrackWorld, TheBoxIsTheContactBoxAxleCentredAndTheAskerIsSkipped) {
  const Track track = shipped_track();
  Simulation sim;
  sim.add_agent(car_at(-4.0));
  // An asymmetric opponent: CoG behind the wheelbase midpoint, so its box
  // sits 0.04 m ahead of where a CoG-centred box would, exactly as the
  // contact pass places it (ADR-0043).
  slipx::sim::AgentSpec ahead = car_at(-2.0);
  ahead.params.lf = 0.20;
  ahead.params.lr = 0.12;
  sim.add_agent(ahead);
  const TrackWorld world(track, sim, 30.0);

  const Hit hit = world(0, pose_at(-4.0), 0.0);
  ASSERT_TRUE(hit.hit);
  const double offset = 0.5 * (0.20 - 0.12);
  EXPECT_NEAR(hit.range, 2.0 - 0.25 + offset, 1.0e-9);

  // The asker does not see its own body: the ray starts inside its own
  // box, and an unskipped box answers at range zero.
  EXPECT_GT(hit.range, 1.0);
}

TEST(TrackWorld, NoFootprintIsInvisibleToSensorsAsItIsToBumpers) {
  const Track track = shipped_track();
  Simulation sim;
  sim.add_agent(car_at(-4.0));
  slipx::sim::AgentSpec ghost = car_at(-2.0);
  ghost.footprint_length = 0.0;   // declares no footprint (ADR-0043)
  ghost.footprint_width = 0.0;
  sim.add_agent(ghost);
  const TrackWorld world(track, sim, 30.0);

  // The ray passes through where the ghost stands and reaches a wall far
  // beyond it: one rule for contact and sensing, not two.
  const Hit hit = world(0, pose_at(-4.0), 0.0);
  ASSERT_TRUE(hit.hit);
  EXPECT_GT(hit.range, 3.0);
}

TEST(TrackWorld, AWreckKeepsItsBox) {
  const Track track = shipped_track();
  Simulation sim;
  sim.add_agent(car_at(-4.0));

  slipx::sim::AgentSpec doomed = car_at(-2.0);
  doomed.policy = nullptr;
  doomed.mailbox = std::make_shared<slipx::sim::CommandMailbox>();
  doomed.timeout_policy = slipx::sim::TimeoutPolicy::kDnf;
  sim.add_agent(doomed);

  const TrackWorld world(track, sim, 30.0);
  sim.advance();  // no command ever arrives: the miss rules it out
  ASSERT_FALSE(sim.agent_running(1));

  const Hit hit = world(0, pose_at(-4.0), 0.0);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.range, 2.0 - 0.25, 1.0e-9)
      << "a wreck is an obstacle to sensors exactly as it is to bumpers";
}

TEST(TrackWorld, RaysWithinAStepShareOneWorldAndTheNextStepMoves) {
  const Track track = shipped_track();
  Simulation sim;
  sim.add_agent(car_at(-4.0));
  sim.add_agent(car_at(-2.0, 2.0));  // driving away at 2 m/s
  const TrackWorld world(track, sim, 30.0);

  const Hit before = world(0, pose_at(-4.0), 0.0);
  const Hit again = world(0, pose_at(-4.0), 0.0);
  ASSERT_TRUE(before.hit);
  EXPECT_EQ(before.range, again.range)
      << "two rays in one step see one consistent world";

  sim.run(500);  // half a second: the coasting opponent moves away
  const Hit after = world(0, pose_at(-4.0), 0.0);
  ASSERT_TRUE(after.hit);
  EXPECT_GT(after.range, before.range + 0.5);
  EXPECT_LT(after.range, before.range + 1.2);
}

TEST(TrackWorld, AWorldMissingACarRefusesToAnswer) {
  const Track track = shipped_track();
  Simulation sim;
  sim.add_agent(car_at(-4.0));
  const TrackWorld world(track, sim, 30.0);
  ASSERT_TRUE(world(0, pose_at(-4.0), kHalfPi).hit);

  sim.add_agent(car_at(-2.0));
  EXPECT_THROW(world(0, pose_at(-4.0), 0.0), std::logic_error)
      << "a world missing a car is an invisible obstacle";
}

TEST(TrackWorld, MaxRangeBoundsTheWorldAndIsValidated) {
  const Track track = shipped_track();
  Simulation sim;
  sim.add_agent(car_at(-4.0));
  sim.add_agent(car_at(-2.0));  // a car and a wall both beyond max_range

  const TrackWorld near_sighted(track, sim, 0.5);
  EXPECT_FALSE(near_sighted(0, pose_at(-4.0), 0.0).hit)
      << "the car at two metres does not exist to a half-metre world";
  EXPECT_TRUE(near_sighted(0, pose_at(-4.0), kHalfPi).hit == false ||
              near_sighted(0, pose_at(-4.0), kHalfPi).range <= 0.5);

  EXPECT_THROW(TrackWorld(track, sim, 0.0), std::invalid_argument);
  EXPECT_THROW(TrackWorld(track, sim, -1.0), std::invalid_argument);
}

TEST(TrackWorld, ThroughTheRigTheOpponentAppearsInTheScan) {
  const Track track = shipped_track();
  Simulation sim;
  sim.add_agent(car_at(-4.0));
  sim.add_agent(car_at(-2.0));
  const TrackWorld world(track, sim, 30.0);

  slipx::sim::SensorRig rig(sim, world.function(), 3);
  slipx::sim::LidarSensor lidar;
  lidar.name = "scan";
  lidar.spec.rate_hz = 40.0;
  lidar.spec.rays = 360;
  lidar.spec.range_max = 30.0;
  lidar.spec.noise_base_m = 0.0;
  lidar.spec.noise_per_metre = 0.0;
  lidar.spec.dropout_probability = 0.0;
  slipx::sim::AgentSensors sensors;
  sensors.lidars.push_back(lidar);
  rig.attach(0, sensors);

  for (int i = 0; i < 30; ++i) {
    sim.advance();
    rig.collect();
  }
  const auto scans = rig.take_scans(0, "scan");
  ASSERT_FALSE(scans.empty());

  // The ray closest to straight ahead measures the opponent; the rays
  // closest to either side measure the walls.
  double best_forward = 1.0e9, forward_range = 0.0;
  double best_left = 1.0e9, left_range = 0.0;
  for (const auto& ray : scans.front().rays) {
    if (!ray.valid) continue;
    if (std::fabs(ray.angle) < best_forward) {
      best_forward = std::fabs(ray.angle);
      forward_range = ray.range;
    }
    if (std::fabs(ray.angle - kHalfPi) < best_left) {
      best_left = std::fabs(ray.angle - kHalfPi);
      left_range = ray.range;
    }
  }
  EXPECT_NEAR(forward_range, 1.75, 0.05);
  EXPECT_NEAR(left_range, 0.75, 0.05);
}

}  // namespace
