// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The wall pass in the orchestrator (ADR-0055): walls are latched scenery,
// a wall that is never touched changes nothing, a car cannot end a step on
// the far side of one, the bounce replays bit for bit, and a frozen car
// against a wall stays byte-identical.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "slipx/sim/simulation.hpp"

namespace {

using slipx::Vec2;
using slipx::Tier;
using slipx::VehicleState;
using slipx::sim::AgentSpec;
using slipx::sim::Simulation;
using slipx::sim::SimulationConfig;
using slipx::sim::TimeoutPolicy;
using slipx::sim::WallContactEvent;

constexpr double kLength = 0.55;
constexpr double kWidth = 0.30;
constexpr double kHalfLength = 0.5 * kLength;

// A coasting L1 car with a footprint, as the pair tests use: what is under
// test is the collision, not a controller's response to it.
AgentSpec coaster(double x, double y, double yaw, double speed) {
  AgentSpec spec;
  spec.tier = Tier::L1_Bicycle;
  spec.initial_state.pos.x = x;
  spec.initial_state.pos.y = y;
  spec.initial_state.yaw = yaw;
  spec.initial_state.vel_body.x = speed;
  spec.footprint_length = kLength;
  spec.footprint_width = kWidth;
  return spec;
}

// A vertical wall at x, long enough that nothing drives around it.
std::vector<Vec2> vertical_wall(double x) {
  return {Vec2{x, -50.0}, Vec2{x, 50.0}};
}

TEST(WallSim, RefusalsAreNamed) {
  Simulation sim;
  EXPECT_THROW(sim.add_wall({Vec2{0.0, 0.0}}, false), std::invalid_argument);
  EXPECT_THROW(
      sim.add_wall({Vec2{0.0, 0.0},
                    Vec2{std::numeric_limits<double>::quiet_NaN(), 1.0}},
                   false),
      std::invalid_argument);
  EXPECT_THROW(
      sim.add_wall({Vec2{0.0, 0.0}, Vec2{0.0, 0.0}, Vec2{1.0, 0.0}}, false),
      std::invalid_argument);
  // A closed polyline that repeats its first point would close with a
  // zero-length segment.
  EXPECT_THROW(
      sim.add_wall({Vec2{0.0, 0.0}, Vec2{1.0, 0.0}, Vec2{0.0, 0.0}}, true),
      std::invalid_argument);
  EXPECT_EQ(sim.wall_segment_count(), 0u);

  // Walls are latched before the green flag; reset() reopens the latch
  // along with the clock.
  sim.add_agent(coaster(0.0, 0.0, 0.0, 1.0));
  sim.advance();
  EXPECT_THROW(sim.add_wall(vertical_wall(5.0), false),
               std::invalid_argument);
  sim.reset();
  EXPECT_NO_THROW(sim.add_wall(vertical_wall(5.0), false));
  EXPECT_EQ(sim.wall_segment_count(), 1u);
}

TEST(WallSim, ClosedJoinsTheLastPointBackToTheFirst) {
  Simulation sim;
  const std::vector<Vec2> square = {Vec2{-1.0, -1.0}, Vec2{1.0, -1.0},
                                    Vec2{1.0, 1.0}, Vec2{-1.0, 1.0}};
  sim.add_wall(square, true);
  EXPECT_EQ(sim.wall_segment_count(), 4u);

  Simulation open;
  open.add_wall(square, false);
  EXPECT_EQ(open.wall_segment_count(), 3u);
}

// The assertion the slice was designed around, walls edition: a wall that
// is never touched changes nothing, bit for bit.
TEST(WallSim, AWallThatIsNeverTouchedChangesNothing) {
  Simulation bare;
  bare.add_agent(coaster(0.0, 0.0, 0.0, 5.0));
  Simulation walled;
  walled.add_wall(vertical_wall(100.0), false);
  walled.add_agent(coaster(0.0, 0.0, 0.0, 5.0));
  bare.run_for(2.0);
  walled.run_for(2.0);
  EXPECT_EQ(bare.trajectory_hash(), walled.trajectory_hash());
}

TEST(WallSim, ACarCannotEndAStepAcrossAWall) {
  const double wall_x = 2.0;
  Simulation sim;
  sim.add_wall(vertical_wall(wall_x), false);
  sim.add_agent(coaster(0.0, 0.0, 0.0, 8.0));

  bool touched = false;
  for (int step = 0; step < 3000; ++step) {
    sim.advance();
    // The projection removes the whole penetration between steps, so the
    // recorded trajectory never shows the footprint past the line.
    EXPECT_LE(sim.state(0).pos.x + kHalfLength, wall_x + 1e-9);
    if (!sim.wall_contacts().empty()) {
      touched = true;
      const WallContactEvent& event = sim.wall_contacts().front();
      EXPECT_EQ(event.step, sim.step_count());
      EXPECT_EQ(event.agent, 0u);
      EXPECT_EQ(event.segment, 0u);
      EXPECT_DOUBLE_EQ(event.normal.x, -1.0);
      EXPECT_DOUBLE_EQ(event.normal.y, 0.0);
      EXPECT_NEAR(event.point.x, wall_x, 1e-9);
      if (event.jn > 0.0) EXPECT_GT(event.approach, 0.0);
    }
  }
  ASSERT_TRUE(touched);
  // And it bounced rather than stuck: the car ends the run moving away
  // from, or at rest against, the wall it hit.
  EXPECT_LE(sim.state(0).vel_body.x, 1e-9);
}

TEST(WallSim, AClosedBoxKeepsACarInsideFromAnyHeading) {
  // A car loose inside a closed square, coasting from a diagonal heading:
  // corners, joints and repeated bounces, and it must still be inside at
  // every step. This is the RViz escape, mechanised as a regression.
  const double half = 2.0;
  Simulation sim;
  sim.add_wall({Vec2{-half, -half}, Vec2{half, -half}, Vec2{half, half},
                Vec2{-half, half}},
               true);
  sim.add_agent(coaster(0.0, 0.0, 0.6, 6.0));
  for (int step = 0; step < 4000; ++step) {
    sim.advance();
    EXPECT_LT(std::abs(sim.state(0).pos.x), half);
    EXPECT_LT(std::abs(sim.state(0).pos.y), half);
  }
}

TEST(WallSim, APairShoveIntoAWallIsPushedBackTheSameStep) {
  // The observed failure, in miniature: car B overlaps car A and the pair
  // projection shoves A toward the wall. Walls resolve after pairs, so the
  // same step pushes A back out; without that ordering A would end the
  // step inside the wall.
  const double wall_x = 1.0;
  Simulation sim;
  sim.add_wall(vertical_wall(wall_x), false);
  sim.add_agent(coaster(0.7, 0.0, 0.0, 0.0));
  sim.add_agent(coaster(0.24, 0.0, 0.0, 0.0));
  sim.advance();
  EXPECT_FALSE(sim.wall_contacts().empty());
  EXPECT_LE(sim.state(0).pos.x + kHalfLength, wall_x + 1e-9);
}

TEST(WallSim, TheRejectPadCoversWithinPassMotion) {
  // The reject circle is tested against the centre where the pass found
  // it, but an earlier segment's resolution moves the car by up to one
  // depth before a later segment is reached. Here that motion matters: a
  // long diagonal wall (segment 0) pushes the resting car toward a short
  // second wall (segment 1) whose tight bounding box sits just beyond ONE
  // bounding radius of the starting centre, and the second wall's own
  // pushback is perpendicular to the first wall's normal, so a correct
  // pass slides the car along the first wall and ends flush against
  // both lines. An unpadded reject skips segment 1 and leaves the car
  // across it at the end of the step.
  Simulation sim;
  // Line x + y = -0.1422, passing 0.1006 below-left of the origin.
  sim.add_wall({Vec2{-1.4853, 1.3431}, Vec2{1.3431, -1.4853}}, false);
  // Line x - y = 0.22, short: bounding box min_x = 0.32, beyond the
  // bounding radius 0.3133 of the (0, 0) centre.
  sim.add_wall({Vec2{0.32, 0.10}, Vec2{0.532, 0.312}}, false);
  sim.add_agent(coaster(0.0, 0.0, 0.0, 0.0));
  sim.advance();

  ASSERT_EQ(sim.wall_contacts().size(), 2u);
  EXPECT_EQ(sim.wall_contacts()[0].segment, 0u);
  EXPECT_EQ(sim.wall_contacts()[1].segment, 1u);

  // Flush against both lines, penetrating neither. The box reach along
  // either diagonal normal is (half_length + half_width) / sqrt(2).
  const double off = sim.footprint_centre_offset(0);
  const double bx = sim.state(0).pos.x + std::cos(sim.state(0).yaw) * off;
  const double by = sim.state(0).pos.y + std::sin(sim.state(0).yaw) * off;
  const double root_two = std::sqrt(2.0);
  const double reach = (sim.footprint_half_length(0) +
                        sim.footprint_half_width(0)) / root_two;
  const double clear_of_first = (bx + by + 0.1422) / root_two - reach;
  const double clear_of_second = (0.22 - (bx - by)) / root_two - reach;
  EXPECT_GE(clear_of_first, -1e-9);
  EXPECT_GE(clear_of_second, -1e-9);
}

TEST(WallSim, TheBounceReplaysBitForBit) {
  Simulation sim;
  sim.add_wall(vertical_wall(2.0), false);
  sim.add_agent(coaster(0.0, 0.0, 0.0, 6.0));
  sim.set_input_logging(true);
  sim.run_for(1.5);
  const std::string live = sim.trajectory_hash();
  const std::vector<slipx::DriveInput> log = sim.input_log();

  sim.replay(log);
  EXPECT_EQ(sim.trajectory_hash(), live);

  // And reset alone reproduces it too: walls survive the rewind.
  sim.reset();
  sim.run_for(1.5);
  EXPECT_EQ(sim.trajectory_hash(), live);
}

TEST(WallSim, AFrozenCarAgainstAWallStaysByteIdentical) {
  // A DNF'd car teleported into overlap with a wall: the wall pass must
  // skip it entirely, because a frozen state is a promise (ADR-0042).
  SimulationConfig config;
  config.barrier_timeout = 0.0;
  Simulation sim(config);
  sim.add_wall(vertical_wall(2.0), false);
  AgentSpec spec = coaster(0.0, 0.0, 0.0, 0.0);
  spec.mailbox = std::make_shared<slipx::sim::CommandMailbox>();
  spec.timeout_policy = TimeoutPolicy::kDnf;
  sim.add_agent(spec);
  sim.advance();   // no command ever arrives: timeout-DNF on the spot
  ASSERT_FALSE(sim.agent_running(0));

  sim.state(0).pos.x = 2.0;   // the footprint straddles the wall
  VehicleState frozen = sim.state(0);
  sim.run(100);
  EXPECT_EQ(std::memcmp(&frozen, &sim.state(0), sizeof(VehicleState)), 0);
  EXPECT_TRUE(sim.wall_contacts().empty());
}

TEST(WallSim, TheManifestRecordsTheWalls) {
  Simulation bare;
  bare.add_agent(coaster(0.0, 0.0, 0.0, 1.0));

  Simulation walled;
  walled.add_wall(vertical_wall(2.0), false);
  walled.add_agent(coaster(0.0, 0.0, 0.0, 1.0));

  const auto bare_manifest = bare.manifest();
  const auto walled_manifest = walled.manifest();
  EXPECT_EQ(bare_manifest.wall_segments, 0u);
  EXPECT_TRUE(bare_manifest.walls_digest.empty());
  EXPECT_EQ(walled_manifest.wall_segments, 1u);
  EXPECT_FALSE(walled_manifest.walls_digest.empty());

  // Different walls are a different race; the same walls are the same.
  EXPECT_NE(bare_manifest.configuration_digest(),
            walled_manifest.configuration_digest());
  // A moved wall keeps the segment count, so this is the coordinates
  // reaching the configuration digest, not just the count.
  Simulation moved;
  moved.add_wall(vertical_wall(3.0), false);
  moved.add_agent(coaster(0.0, 0.0, 0.0, 1.0));
  EXPECT_NE(moved.manifest().walls_digest, walled_manifest.walls_digest);
  EXPECT_NE(moved.manifest().configuration_digest(),
            walled_manifest.configuration_digest());

  EXPECT_NE(walled_manifest.to_json().find("\"wall_segments\": 1"),
            std::string::npos);
}

TEST(WallSim, EventsAreCurrentNotCumulative) {
  // A touching step reports its contact; the next step, with the car
  // pushed clear and leaving, reports nothing. The list is the most
  // recent step's, never a history.
  Simulation sim;
  sim.add_wall(vertical_wall(2.0), false);
  // Overlapping the wall but already separating: the projection applies
  // (and is reported), the impulse does not.
  sim.add_agent(coaster(1.8, 0.0, 0.0, -3.0));
  sim.advance();
  ASSERT_EQ(sim.wall_contacts().size(), 1u);
  EXPECT_EQ(sim.wall_contacts().front().jn, 0.0);
  sim.advance();
  EXPECT_TRUE(sim.wall_contacts().empty());
}

}  // namespace
