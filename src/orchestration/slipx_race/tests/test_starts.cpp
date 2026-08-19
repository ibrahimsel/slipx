// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Start procedures (ruleset 2.5.1.9) and the track-pose helper they are
// built on.

#include <gtest/gtest.h>

#include <cmath>

#include "slipx/race/ruleset.hpp"
#include "slipx/race/starts.hpp"
#include "race_test_support.hpp"

namespace {

using slipx::race::pose_at;
using slipx::race::TrackPose;
using slipx::scene::Track;

TEST(Starts, TheRulesetIsStated) {
  // The build states its ruleset revision: repository and a full commit id.
  const std::string statement = slipx::race::ruleset_statement();
  EXPECT_NE(statement.find("roboracer_rules"), std::string::npos);
  EXPECT_NE(statement.find(slipx::race::kRulesetRevision),
            std::string::npos);
  EXPECT_EQ(std::string(slipx::race::kRulesetRevision).size(), 40u);
}

TEST(Starts, PoseAtWalksAndWrapsTheTrack) {
  const Track track = race_test::shipped_track();
  const double total = track.length();

  // Progression: two nearby arc lengths are two nearby points, about the
  // right distance apart along the track.
  const TrackPose p0 = pose_at(track, 2.0);
  const TrackPose p1 = pose_at(track, 2.5);
  const double moved = std::hypot(p1.x - p0.x, p1.y - p0.y);
  EXPECT_NEAR(moved, 0.5, 0.05);

  // The wrap: one full lap later is the same pose.
  const TrackPose wrapped = pose_at(track, 2.0 + total);
  EXPECT_NEAR(wrapped.x, p0.x, 1e-9);
  EXPECT_NEAR(wrapped.y, p0.y, 1e-9);

  // And a projected pose lands back at its own arc length.
  const auto projected = slipx::scene::project(track, p0.x, p0.y);
  EXPECT_NEAR(projected.s, 2.0, 1e-6);
}

TEST(Starts, TheGridIsSideBySideOneCarWidthApartAtRest) {
  const Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  sim.add_agent(race_test::race_car({}));
  sim.add_agent(race_test::race_car({}));

  slipx::race::RaceConfig config;
  slipx::race::grid_start(sim, track, 0, 1, 3.0, config.grid_gap);

  const auto& left = sim.state(0);
  const auto& right = sim.state(1);

  // One car width apart (2.5.1.9.1-2), at rest, both on the line.
  const double gap = std::hypot(left.pos.x - right.pos.x,
                                left.pos.y - right.pos.y);
  EXPECT_NEAR(gap, 0.30, 1e-9);
  EXPECT_DOUBLE_EQ(left.speed(), 0.0);
  EXPECT_DOUBLE_EQ(right.speed(), 0.0);

  // The left car is to the LEFT of the direction of travel: positive
  // lateral in the projection's sign convention.
  EXPECT_GT(slipx::scene::project(track, left.pos.x, left.pos.y).lateral,
            0.0);
  EXPECT_LT(slipx::scene::project(track, right.pos.x, right.pos.y).lateral,
            0.0);

  // Both heading along the track.
  const TrackPose line = pose_at(track, 3.0);
  EXPECT_NEAR(left.yaw, line.heading, 1e-9);
  EXPECT_NEAR(right.yaw, line.heading, 1e-9);
}

TEST(Starts, ARollingStartIsTheSameGeometryAtSpeed) {
  const Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  sim.add_agent(race_test::race_car({}));
  sim.add_agent(race_test::race_car({}));

  slipx::race::rolling_start(sim, track, 0, 1, 3.0, 0.30, 2.5);
  EXPECT_DOUBLE_EQ(sim.state(0).vel_body.x, 2.5);
  EXPECT_DOUBLE_EQ(sim.state(1).vel_body.x, 2.5);
  // Wheel speeds consistent with the roll, not left at zero.
  EXPECT_GT(sim.state(0).omega_w[0], 0.0);
}

}  // namespace
