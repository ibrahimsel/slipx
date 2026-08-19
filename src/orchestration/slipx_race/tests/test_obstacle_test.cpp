// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The obstacle avoidance test (ruleset 2.5.1.6): pass without touching and
// without coming to a complete stop; stopping in front of it is explicitly
// not a pass.

#include <gtest/gtest.h>

#include <cmath>

#include "slipx/race/obstacle_test.hpp"
#include "slipx/race/starts.hpp"
#include "race_test_support.hpp"

namespace {

using slipx::race::ObstacleOutcome;
using slipx::race::ObstacleTest;
using slipx::race::RaceConfig;
using slipx::scene::Track;

// A car at the line and an obstacle 3 m up the road, laterally where the
// scenario wants it. The obstacle is an ordinary footprinted agent at rest
// with no policy: the sim's contact model is what makes touching it a fact.
// The policy is built by a factory against the fixture's own track, so the
// lambda's captured reference outlives the test.
struct Fixture {
  Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;

  template <typename PolicyFactory>
  Fixture(double car_speed, double obstacle_lateral,
          PolicyFactory make_policy) {
    sim.add_agent(race_test::race_car(make_policy(track)));
    sim.add_agent(race_test::race_car({}));
    slipx::race::place_on_track(sim, 0, track, 1.0, 0.0, car_speed);
    slipx::race::place_on_track(sim, 1, track, 4.0, obstacle_lateral, 0.0);
  }
};

slipx::sim::Policy follower(const Track& track) {
  return race_test::follow_centreline(track, 2.0);
}

TEST(ObstacleTest, ACleanPassAtSpeedPasses) {
  Fixture fixture(2.0, -0.45, follower);

  RaceConfig config;
  ObstacleTest test(fixture.sim, fixture.track, 0, 1, config);
  test.run(6000);

  EXPECT_EQ(test.outcome(), ObstacleOutcome::kPassed);
  ASSERT_FALSE(test.events().empty());
  EXPECT_EQ(test.events().back().type,
            slipx::race::EventType::kObstaclePassed);
}

TEST(ObstacleTest, DrivingIntoTheObstacleFails) {
  Fixture fixture(2.0, 0.0, follower);

  RaceConfig config;
  ObstacleTest test(fixture.sim, fixture.track, 0, 1, config);
  test.run(6000);

  EXPECT_EQ(test.outcome(), ObstacleOutcome::kFailed);
  EXPECT_EQ(test.failure_code(), slipx::race::kObstacleFailContact);
}

TEST(ObstacleTest, StoppingInFrontOfItFailsToo) {
  // Drive for a second, then brake to a stand: caution is exactly what the
  // test exists to fail (2.5.1.6.3).
  Fixture fixture(2.0, 0.0, [](const Track&) {
    return slipx::sim::Policy(
        [](const slipx::VehicleState& s, double t, slipx::sense::Rng&) {
          slipx::DriveInput input;
          input.steer_cmd = 0.0;
          input.accel_cmd = t < 1.0 ? 4.0 * (2.0 - s.vel_body.x) : -8.0;
          return input;
        });
  });

  RaceConfig config;
  ObstacleTest test(fixture.sim, fixture.track, 0, 1, config);
  test.run(6000);

  EXPECT_EQ(test.outcome(), ObstacleOutcome::kFailed);
  EXPECT_EQ(test.failure_code(), slipx::race::kObstacleFailStopped);
}

TEST(ObstacleTest, ThePassDistanceWrapsAcrossTheStartLine) {
  // The car starts just before the line and the obstacle sits just after
  // it: the forward gap wraps around the lap, and a test that measured it
  // as a negative number would declare a pass before the car moved.
  Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  sim.add_agent(race_test::race_car(race_test::follow_centreline(track, 2.0)));
  sim.add_agent(race_test::race_car({}));
  slipx::race::place_on_track(sim, 0, track, track.length() - 2.0, 0.0, 2.0);
  slipx::race::place_on_track(sim, 1, track, 1.5, -0.45, 0.0);

  RaceConfig config;
  ObstacleTest test(sim, track, 0, 1, config);
  test.run(8000);

  EXPECT_EQ(test.outcome(), ObstacleOutcome::kPassed);
  ASSERT_FALSE(test.events().empty());
  EXPECT_GT(test.events().back().time, 0.5)
      << "an instant pass means the wrap was measured as a negative gap";
}

TEST(ObstacleTest, AStandingStartIsNotAStop) {
  // The car begins at rest; the stop check must arm only once it moves, or
  // every standing start would fail at step one.
  Fixture fixture(0.0, -0.45, follower);

  RaceConfig config;
  ObstacleTest test(fixture.sim, fixture.track, 0, 1, config);
  test.run(8000);

  EXPECT_EQ(test.outcome(), ObstacleOutcome::kPassed);
}

}  // namespace
