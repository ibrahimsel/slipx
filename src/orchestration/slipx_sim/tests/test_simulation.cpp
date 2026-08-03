// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// SIM-01, SIM-02, SIM-09: the fixed-step in-process orchestrator.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "slipx/sim/manoeuvres.hpp"
#include "slipx/sim/simulation.hpp"

namespace {

using slipx::DriveInput;
using slipx::Tier;
using slipx::VehicleParams;
using slipx::VehicleState;
using slipx::sim::AgentSpec;
using slipx::sim::Rng;
using slipx::sim::Simulation;
using slipx::sim::SimulationConfig;

AgentSpec straight_runner(const char* name, double speed) {
  AgentSpec spec;
  spec.name = name;
  spec.tier = Tier::L1_Bicycle;
  spec.initial_state.vel_body.x = speed;
  spec.policy = [speed](const VehicleState& s, double, Rng&) {
    return DriveInput{0.0, slipx::sim::hold_speed(s, speed)};
  };
  return spec;
}

TEST(Simulation, DefaultsToOneKilohertz) {
  EXPECT_DOUBLE_EQ(SimulationConfig{}.dt, 1.0e-3);
  Simulation sim;
  EXPECT_DOUBLE_EQ(sim.dt(), 1.0e-3);
}

TEST(Simulation, RejectsANonPositiveStep) {
  SimulationConfig config;
  config.dt = 0.0;
  EXPECT_THROW(Simulation{config}, std::invalid_argument);
  config.dt = -1.0e-3;
  EXPECT_THROW(Simulation{config}, std::invalid_argument);
}

// Time is steps * dt, not a running sum. Ten thousand additions of 0.001 do
// not give 10.0, and a clock that disagrees with its own step count cannot be
// replayed against a recorded input sequence.
TEST(Simulation, TimeIsExactlyStepsTimesDt) {
  Simulation sim;
  sim.add_agent(straight_runner("a", 3.0));
  sim.run(10000);

  EXPECT_EQ(sim.step_count(), 10000u);
  EXPECT_DOUBLE_EQ(sim.time(), 10.0);

  double accumulated = 0.0;
  for (int i = 0; i < 10000; ++i) accumulated += 1.0e-3;
  EXPECT_NE(accumulated, 10.0) << "which is exactly why time() does not do "
                                  "it that way";
}

TEST(Simulation, RunForRoundsToWholeSteps) {
  Simulation sim;
  sim.add_agent(straight_runner("a", 3.0));
  sim.run_for(0.0104);
  EXPECT_EQ(sim.step_count(), 10u);
  EXPECT_DOUBLE_EQ(sim.time(), 0.010);
}

TEST(Simulation, AnAgentWithNoPolicyCoasts) {
  Simulation sim;
  AgentSpec spec;
  spec.tier = Tier::L1_Bicycle;
  spec.initial_state.vel_body.x = 5.0;
  sim.add_agent(spec);  // no policy

  sim.run_for(1.0);
  EXPECT_LT(sim.state(0).vel_body.x, 5.0) << "resistance slows it";
  EXPECT_GT(sim.state(0).vel_body.x, 4.0) << "but nothing brakes it";
  EXPECT_DOUBLE_EQ(sim.state(0).steer, 0.0);
}

// SIM-09: no compile-time or architectural bound on the agent count.
TEST(Simulation, SupportsManyAgents) {
  Simulation sim;
  const std::size_t n = 200;
  for (std::size_t i = 0; i < n; ++i) {
    sim.add_agent(straight_runner("car", 2.0 + 0.01 * static_cast<double>(i)));
  }
  ASSERT_EQ(sim.agent_count(), n);

  sim.run_for(0.5);
  for (std::size_t i = 0; i < n; ++i) {
    EXPECT_GT(sim.state(i).pos.x, 0.0);
  }
  EXPECT_GT(sim.state(n - 1).pos.x, sim.state(0).pos.x)
      << "the faster car went further";
}

// The lockstep property. Every policy sees the world as it was at the start of
// the step, so the result cannot depend on the order agents were added in. The
// probe below records what agent 1 saw of agent 0; if commands were collected
// lazily during the movement phase, it would see a state that had already
// advanced.
TEST(Simulation, CommandsAreCollectedBeforeAnyAgentMoves) {
  Simulation sim;

  sim.add_agent(straight_runner("leader", 5.0));

  std::vector<double> observed_leader_x;
  std::vector<double> leader_x_at_step_start;

  AgentSpec follower;
  follower.name = "follower";
  follower.tier = Tier::L1_Bicycle;
  follower.initial_state.vel_body.x = 5.0;
  follower.policy = [&](const VehicleState&, double, Rng&) {
    observed_leader_x.push_back(sim.state(0).pos.x);
    return DriveInput{0.0, 0.0};
  };
  sim.add_agent(follower);

  for (int i = 0; i < 100; ++i) {
    leader_x_at_step_start.push_back(sim.state(0).pos.x);
    sim.advance();
  }

  ASSERT_EQ(observed_leader_x.size(), leader_x_at_step_start.size());
  for (std::size_t i = 0; i < observed_leader_x.size(); ++i) {
    EXPECT_EQ(observed_leader_x[i], leader_x_at_step_start[i])
        << "at step " << i;
  }
}

// Adding an agent must not change the numbers any other agent draws, or a
// scenario cannot be extended without invalidating every result that came
// before it.
TEST(Simulation, AddingAnAgentDoesNotDisturbTheOthersRandomStreams) {
  SimulationConfig config;
  config.master_seed = 777;

  Simulation one(config);
  one.add_agent(straight_runner("a", 3.0));
  const std::uint64_t a_first = one.rng(0).next_u64();

  Simulation two(config);
  two.add_agent(straight_runner("a", 3.0));
  two.add_agent(straight_runner("b", 4.0));
  EXPECT_EQ(two.rng(0).next_u64(), a_first);
  EXPECT_NE(two.rng(1).next_u64(), a_first);
}

TEST(Simulation, ResetRestoresStatesClockAndStreams) {
  SimulationConfig config;
  config.master_seed = 5;
  Simulation sim(config);
  sim.add_agent(straight_runner("a", 4.0));

  const VehicleState initial = sim.state(0);
  const std::uint64_t first_draw = sim.rng(0).next_u64();

  sim.run_for(2.0);
  ASSERT_GT(sim.state(0).pos.x, 1.0);

  sim.reset();
  EXPECT_EQ(sim.step_count(), 0u);
  EXPECT_DOUBLE_EQ(sim.time(), 0.0);
  EXPECT_EQ(std::memcmp(&sim.state(0), &initial, sizeof(VehicleState)), 0);
  EXPECT_EQ(sim.rng(0).next_u64(), first_draw);
}

TEST(Simulation, OutOfRangeAgentAccessThrows) {
  Simulation sim;
  sim.add_agent(straight_runner("a", 3.0));
  EXPECT_THROW(sim.state(1), std::out_of_range);
  EXPECT_THROW(sim.diagnostics(3), std::out_of_range);
  EXPECT_NO_THROW(sim.state(0));
}

// The core's refusal to substitute an unimplemented tier has to survive the
// trip through the orchestrator (CORE-02).
TEST(Simulation, RefusesUnimplementedTiersAndBadParameters) {
  Simulation sim;

  AgentSpec l3;
  l3.tier = Tier::L3_Extended;
  EXPECT_THROW(sim.add_agent(l3), std::invalid_argument);

  AgentSpec impossible;
  impossible.tier = Tier::L1_Bicycle;
  impossible.params.mass = -1.0;
  EXPECT_THROW(sim.add_agent(impossible), std::invalid_argument);

  EXPECT_EQ(sim.agent_count(), 0u) << "a refused agent must not be half-added";
}

TEST(Simulation, DiagnosticsAreAvailablePerAgent) {
  Simulation sim;
  AgentSpec spec;
  spec.tier = Tier::L1_Bicycle;
  spec.initial_state.vel_body.x = 5.0;
  spec.policy = [](const VehicleState& s, double, Rng&) {
    return DriveInput{0.1, slipx::sim::hold_speed(s, 5.0)};
  };
  sim.add_agent(spec);
  sim.run_for(0.5);

  EXPECT_GT(sim.diagnostics(0).ay, 0.0);
  EXPECT_EQ(sim.diagnostics(0).tier, static_cast<int>(Tier::L1_Bicycle));
  EXPECT_EQ(sim.model(0).tier(), Tier::L1_Bicycle);
}

// The step steer is the P0 exit-gate manoeuvre, so its shape is pinned:
// straight until the step, then a settled left turn.
TEST(Manoeuvres, StepSteerIsStraightThenTurning) {
  Simulation sim = slipx::sim::make_conformance_run();

  sim.run_for(0.9);  // before the step at t = 1.0
  EXPECT_NEAR(sim.state(0).pos.y, 0.0, 1e-12);
  EXPECT_NEAR(sim.state(0).yaw, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(sim.state(0).steer, 0.0);

  sim.run_for(4.0);
  EXPECT_DOUBLE_EQ(sim.state(0).steer, 0.10);
  EXPECT_GT(sim.state(0).pos.y, 0.0) << "and it turned left";
  EXPECT_GT(sim.state(0).rates.z, 0.0);

  // The speed hold is proportional and therefore settles with an offset
  // against whatever resists it, which in a 0.6 g turn is drag, rolling
  // resistance, the longitudinal component of the front tyre force and the
  // vy*r transport term together. Roughly a fifth of a metre per second at
  // this condition. Tightening this tolerance would mean tuning the speed
  // controller, which would put its dynamics into every manoeuvre built on
  // it; the conformance run does not need the speed held exactly, it needs it
  // held reproducibly.
  EXPECT_NEAR(sim.state(0).vel_body.x, 5.0, 0.3);
}

TEST(Manoeuvres, HoldSpeedActsOnTheBodyLongitudinalComponent) {
  VehicleState s;
  s.vel_body.x = 3.0;
  EXPECT_DOUBLE_EQ(slipx::sim::hold_speed(s, 5.0), 8.0);
  EXPECT_DOUBLE_EQ(slipx::sim::hold_speed(s, 3.0), 0.0);
  EXPECT_DOUBLE_EQ(slipx::sim::hold_speed(s, 5.0, 1.0), 2.0);
}

}  // namespace
