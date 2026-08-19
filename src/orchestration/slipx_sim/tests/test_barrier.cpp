// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The barrier and its timeout policies (ADR-0044): commands arrive through a
// step-tagged mailbox, a miss is answered by the agent's policy, one hung
// agent cannot hang a race unless waiting is what was asked for, and the
// input log reproduces whatever the wall clock decided.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include "slipx/sim/manoeuvres.hpp"
#include "slipx/sim/simulation.hpp"

namespace {

using slipx::DriveInput;
using slipx::Tier;
using slipx::VehicleState;
using slipx::sim::AgentSpec;
using slipx::sim::CommandMailbox;
using slipx::sim::DnfCause;
using slipx::sense::Rng;
using slipx::sim::Simulation;
using slipx::sim::SimulationConfig;
using slipx::sim::TimeoutPolicy;

AgentSpec mailbox_car(std::shared_ptr<CommandMailbox> mailbox,
                      TimeoutPolicy policy) {
  AgentSpec spec;
  spec.tier = Tier::L1_Bicycle;
  spec.initial_state.vel_body.x = 4.0;
  spec.mailbox = std::move(mailbox);
  spec.timeout_policy = policy;
  return spec;
}

// The command every barrier test drives with: state-independent, so a
// synchronous twin policy can reproduce it exactly and the comparison is a
// hash equality rather than a tolerance.
DriveInput test_command() { return DriveInput{0.03, 0.5}; }

TEST(Barrier, WaitMeansTimingCannotChangeTheAnswer) {
  // A poster thread that dawdles: the kWait barrier blocks until each
  // step's command arrives, so the trajectory must be bit-identical to the
  // synchronous twin however the scheduler interleaves them.
  const std::uint64_t steps = 400;

  auto mailbox = std::make_shared<CommandMailbox>();
  Simulation sim;
  sim.add_agent(mailbox_car(mailbox, TimeoutPolicy::kWait));

  std::thread poster([mailbox, steps] {
    for (std::uint64_t s = 0; s < steps; ++s) {
      if (s % 97 == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      mailbox->post(s, test_command());
    }
  });
  sim.run(steps);
  poster.join();

  Simulation twin;
  AgentSpec spec;
  spec.tier = Tier::L1_Bicycle;
  spec.initial_state.vel_body.x = 4.0;
  spec.policy = [](const VehicleState&, double, Rng&) {
    return test_command();
  };
  twin.add_agent(std::move(spec));
  twin.run(steps);

  EXPECT_EQ(sim.trajectory_hash(), twin.trajectory_hash());
}

TEST(Barrier, AHungAgentCoastsAndTheRaceGoesOn) {
  SimulationConfig config;
  config.master_seed = 3;

  Simulation sim(config);
  sim.add_agent([] {
    AgentSpec spec;
    spec.tier = Tier::L1_Bicycle;
    spec.initial_state.vel_body.x = 4.0;
    spec.policy = [](const VehicleState& s, double, Rng&) {
      return DriveInput{0.05, slipx::sim::hold_speed(s, 4.0)};
    };
    return spec;
  }());
  // Hung from the first step: nothing is ever posted.
  sim.add_agent(mailbox_car(std::make_shared<CommandMailbox>(),
                            TimeoutPolicy::kCoast));
  sim.run_for(1.0);

  // The hung car behaved exactly like a car with no source at all...
  Simulation coaster(config);
  AgentSpec bare;
  bare.tier = Tier::L1_Bicycle;
  bare.initial_state.vel_body.x = 4.0;
  coaster.add_agent(std::move(bare));
  coaster.run_for(1.0);
  EXPECT_EQ(sim.agent_trajectory_hash(1), coaster.agent_trajectory_hash(0));

  // ...and the running car never noticed: its trajectory is its solo one.
  Simulation solo(config);
  solo.add_agent([] {
    AgentSpec spec;
    spec.tier = Tier::L1_Bicycle;
    spec.initial_state.vel_body.x = 4.0;
    spec.policy = [](const VehicleState& s, double, Rng&) {
      return DriveInput{0.05, slipx::sim::hold_speed(s, 4.0)};
    };
    return spec;
  }());
  solo.run_for(1.0);
  EXPECT_EQ(sim.agent_trajectory_hash(0), solo.agent_trajectory_hash(0));
}

TEST(Barrier, AHungAgentFreezesAndResumesWhereItPaused) {
  auto mailbox = std::make_shared<CommandMailbox>();
  Simulation sim;
  sim.add_agent(mailbox_car(mailbox, TimeoutPolicy::kFreeze));

  for (std::uint64_t s = 0; s < 100; ++s) {
    mailbox->post(s, test_command());
    sim.advance();
  }
  ASSERT_TRUE(sim.agent_running(0));

  // The pause: no commands for 200 steps. The state, velocities included,
  // is byte-identical across it; this is suspension, not a crash.
  VehicleState paused{};
  std::memcpy(&paused, &sim.state(0), sizeof(VehicleState));
  EXPECT_GT(paused.speed(), 0.0) << "paused mid-motion, on purpose";
  sim.run(200);
  EXPECT_EQ(std::memcmp(&paused, &sim.state(0), sizeof(VehicleState)), 0);
  EXPECT_TRUE(sim.agent_running(0)) << "a pause is not a DNF";

  // Commands return, tagged with the current step, and it moves again.
  for (std::uint64_t s = 300; s < 400; ++s) {
    mailbox->post(s, test_command());
    sim.advance();
  }
  EXPECT_NE(std::memcmp(&paused, &sim.state(0), sizeof(VehicleState)), 0);
}

TEST(Barrier, AHungAgentIsDisqualified) {
  auto mailbox = std::make_shared<CommandMailbox>();
  Simulation sim;
  AgentSpec spec = mailbox_car(mailbox, TimeoutPolicy::kDnf);
  spec.footprint_length = 0.55;
  spec.footprint_width = 0.30;
  sim.add_agent(std::move(spec));

  for (std::uint64_t s = 0; s < 50; ++s) {
    mailbox->post(s, test_command());
    sim.advance();
  }
  ASSERT_TRUE(sim.agent_running(0));

  sim.advance();   // the first unanswered barrier
  ASSERT_FALSE(sim.agent_running(0));
  EXPECT_EQ(sim.dnf(0)->cause, DnfCause::kTimeout);
  EXPECT_EQ(sim.dnf(0)->step, 51u);
  EXPECT_EQ(sim.state(0).speed(), 0.0) << "out means a stationary obstacle";

  const auto manifest = sim.manifest();
  EXPECT_EQ(manifest.agents[0].status, "dnf");
  EXPECT_EQ(manifest.agents[0].dnf_cause,
            "barrier timeout: no command arrived");

  // Terminal, exactly as a rollover is: posting again resurrects nothing.
  mailbox->post(60, test_command());
  VehicleState frozen{};
  std::memcpy(&frozen, &sim.state(0), sizeof(VehicleState));
  sim.run(100);
  EXPECT_EQ(std::memcmp(&frozen, &sim.state(0), sizeof(VehicleState)), 0);
}

TEST(Barrier, ReplayReproducesThePause) {
  auto mailbox = std::make_shared<CommandMailbox>();
  Simulation sim;
  sim.add_agent(mailbox_car(mailbox, TimeoutPolicy::kFreeze));
  sim.set_input_logging(true);

  for (std::uint64_t s = 0; s < 60; ++s) {
    mailbox->post(s, test_command());
    sim.advance();
  }
  sim.run(80);   // the pause, recorded as NaN-tagged slots
  for (std::uint64_t s = 140; s < 200; ++s) {
    mailbox->post(s, test_command());
    sim.advance();
  }
  const std::string hash = sim.trajectory_hash();
  const std::vector<DriveInput> log = sim.input_log();

  sim.replay(log);
  EXPECT_EQ(sim.trajectory_hash(), hash);
}

TEST(Barrier, ReplayReproducesTheTimeoutDnf) {
  auto mailbox = std::make_shared<CommandMailbox>();
  Simulation sim;
  sim.add_agent(mailbox_car(mailbox, TimeoutPolicy::kDnf));
  sim.set_input_logging(true);

  for (std::uint64_t s = 0; s < 30; ++s) {
    mailbox->post(s, test_command());
    sim.advance();
  }
  sim.run(50);
  ASSERT_FALSE(sim.agent_running(0));
  const auto event = *sim.dnf(0);
  const std::string hash = sim.trajectory_hash();

  sim.replay(sim.input_log());
  ASSERT_FALSE(sim.agent_running(0));
  EXPECT_EQ(sim.dnf(0)->step, event.step);
  EXPECT_EQ(sim.dnf(0)->cause, DnfCause::kTimeout);
  EXPECT_EQ(sim.trajectory_hash(), hash);
}

TEST(Barrier, AMissMarkerForAnAgentThatCannotMissIsRefused) {
  const double nan = std::numeric_limits<double>::quiet_NaN();

  // A policy agent's log can never contain a miss.
  Simulation sim;
  AgentSpec spec;
  spec.tier = Tier::L1_Bicycle;
  spec.policy = [](const VehicleState&, double, Rng&) {
    return test_command();
  };
  sim.add_agent(std::move(spec));
  EXPECT_THROW(sim.replay({DriveInput{nan, nan}}), std::invalid_argument);

  // Nor can a coasting mailbox agent's: a coasted miss is logged as the
  // zeros it applied, so a marker claims a policy that never writes one.
  Simulation sim2;
  sim2.add_agent(mailbox_car(std::make_shared<CommandMailbox>(),
                             TimeoutPolicy::kCoast));
  EXPECT_THROW(sim2.replay({DriveInput{nan, nan}}), std::invalid_argument);
}

TEST(Barrier, ProtocolErrorsAreRefusedByName) {
  const double nan = std::numeric_limits<double>::quiet_NaN();

  CommandMailbox mailbox;
  EXPECT_THROW(mailbox.post(0, DriveInput{nan, 0.0}), std::invalid_argument);
  mailbox.post(3, test_command());
  EXPECT_THROW(mailbox.post(3, test_command()), std::invalid_argument)
      << "tags strictly increase";
  EXPECT_THROW(mailbox.post(1, test_command()), std::invalid_argument);

  // One command source per agent.
  Simulation sim;
  AgentSpec spec = mailbox_car(std::make_shared<CommandMailbox>(),
                               TimeoutPolicy::kCoast);
  spec.policy = [](const VehicleState&, double, Rng&) {
    return test_command();
  };
  EXPECT_THROW(sim.add_agent(std::move(spec)), std::invalid_argument);

  // A policy returning NaN is refused loudly, not integrated: NaN in the
  // log is reserved as the missed-step marker.
  Simulation sim2;
  AgentSpec bad;
  bad.tier = Tier::L1_Bicycle;
  bad.policy = [nan](const VehicleState&, double, Rng&) {
    return DriveInput{nan, 0.0};
  };
  sim2.add_agent(std::move(bad));
  EXPECT_THROW(sim2.advance(), std::invalid_argument);
}

TEST(Barrier, AnAckHoldsTheLastCommandAndStaleEntriesAreDiscarded) {
  // ack(N) means "alive, no new command": the held command applies, and
  // before anything was posted the held command is the neutral coast.
  auto mailbox = std::make_shared<CommandMailbox>();
  Simulation sim;
  sim.add_agent(mailbox_car(mailbox, TimeoutPolicy::kCoast));
  mailbox->post(0, test_command());
  mailbox->ack(1);
  mailbox->ack(2);
  sim.run(3);

  Simulation twin;
  AgentSpec spec;
  spec.tier = Tier::L1_Bicycle;
  spec.initial_state.vel_body.x = 4.0;
  spec.policy = [](const VehicleState&, double, Rng&) {
    return test_command();
  };
  twin.add_agent(std::move(spec));
  twin.run(3);
  EXPECT_EQ(sim.trajectory_hash(), twin.trajectory_hash());

  // An entry tagged behind the barrier is stale and is discarded, not
  // applied late: the whole run below coasts.
  auto late = std::make_shared<CommandMailbox>();
  Simulation slow;
  slow.add_agent(mailbox_car(late, TimeoutPolicy::kCoast));
  slow.run(5);                       // five misses, five coasted steps
  late->post(2, DriveInput{0.4, 4.0});   // for a step long gone
  slow.run(5);

  Simulation coaster;
  AgentSpec bare;
  bare.tier = Tier::L1_Bicycle;
  bare.initial_state.vel_body.x = 4.0;
  coaster.add_agent(std::move(bare));
  coaster.run(10);
  EXPECT_EQ(slow.trajectory_hash(), coaster.trajectory_hash());

  // And discarded means gone: a fresh entry behind a stale one still
  // reaches the barrier. A queue the stale entry poisons would coast here.
  late->post(10, DriveInput{0.4, 4.0});
  slow.advance();
  coaster.advance();
  EXPECT_NE(slow.trajectory_hash(), coaster.trajectory_hash())
      << "the current-step command behind the stale entry must apply";
}

TEST(Barrier, ATimeoutBudgetWaitsLongEnoughForALateCommand) {
  // A generous wall-clock budget and a deliberately slow poster: the
  // barrier must wait the command in rather than ruling an instant miss.
  SimulationConfig config;
  config.barrier_timeout = 5.0;   // seconds of wall clock, worst case

  auto mailbox = std::make_shared<CommandMailbox>();
  Simulation sim(config);
  sim.add_agent(mailbox_car(mailbox, TimeoutPolicy::kCoast));

  std::thread poster([mailbox] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    mailbox->post(0, DriveInput{0.4, 4.0});
  });
  sim.advance();
  poster.join();

  Simulation coaster;
  AgentSpec bare;
  bare.tier = Tier::L1_Bicycle;
  bare.initial_state.vel_body.x = 4.0;
  coaster.add_agent(std::move(bare));
  coaster.advance();
  EXPECT_NE(sim.trajectory_hash(), coaster.trajectory_hash())
      << "the late command must have been applied, not ruled a miss";
}

TEST(Barrier, TheManifestNamesTheCommandSource) {
  Simulation sim;
  AgentSpec runner;
  runner.tier = Tier::L1_Bicycle;
  runner.policy = [](const VehicleState&, double, Rng&) {
    return test_command();
  };
  sim.add_agent(std::move(runner));
  AgentSpec idle;
  idle.tier = Tier::L1_Bicycle;
  sim.add_agent(std::move(idle));
  sim.add_agent(mailbox_car(std::make_shared<CommandMailbox>(),
                            TimeoutPolicy::kFreeze));
  sim.run(5);

  const auto m = sim.manifest();
  EXPECT_EQ(m.agents[0].command_source, "policy");
  EXPECT_EQ(m.agents[1].command_source, "coast");
  EXPECT_EQ(m.agents[2].command_source, "mailbox");
  EXPECT_EQ(m.agents[2].timeout_policy, "freeze");
  EXPECT_TRUE(m.timing_dependent_commands);

  // The determinism block narrows its promise to the input log, in the
  // manifest itself and not only in documentation nobody reads mid-dispute.
  const std::string json = m.to_json();
  EXPECT_NE(json.find("replayed from the input log"), std::string::npos);
  EXPECT_NE(json.find("\"timeout_policy\": \"freeze\""), std::string::npos);

  // A wait-policy mailbox keeps the strong promise: the barrier blocks, so
  // timing cannot reach the trajectory.
  Simulation strict;
  strict.add_agent(mailbox_car(std::make_shared<CommandMailbox>(),
                               TimeoutPolicy::kWait));
  const auto sm = strict.manifest();
  EXPECT_FALSE(sm.timing_dependent_commands);
  EXPECT_NE(sm.to_json().find("bit-identical for the same binary"),
            std::string::npos);

  // The timeout policy is configuration: change it and the digest moves.
  Simulation other;
  other.add_agent(mailbox_car(std::make_shared<CommandMailbox>(),
                              TimeoutPolicy::kCoast));
  EXPECT_NE(strict.manifest().configuration_digest(),
            other.manifest().configuration_digest());
}

}  // namespace
