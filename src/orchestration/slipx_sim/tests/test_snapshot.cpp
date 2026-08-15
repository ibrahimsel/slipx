// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Snapshot, restore and the two run modes (SIM-08).
//
// The claim being tested is narrow and worth stating exactly: a snapshot
// taken mid-run and restored puts the simulation back where it was, and
// running on from there reproduces the run that was never interrupted, bit
// for bit. Not "close to", and not "the states match": the trajectory hash
// has to match too, because the hash is a running fold and a resumed run that
// forgot to resume the fold would agree about every state and disagree about
// the run.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "slipx/sim/manoeuvres.hpp"
#include "slipx/sim/simulation.hpp"

namespace {

using slipx::DriveInput;
using slipx::Tier;
using slipx::VehicleState;
using slipx::sense::Rng;
using slipx::sim::AgentSpec;
using slipx::sim::RunMode;
using slipx::sim::Simulation;
using slipx::sim::SimulationConfig;
using slipx::sim::SimulationSnapshot;

// A policy that draws from its own random stream every step, so that the
// generator's position is part of the state a snapshot has to carry. A policy
// that never drew would let a broken snapshot pass.
AgentSpec noisy_runner(const char* name, double speed) {
  AgentSpec spec;
  spec.name = name;
  spec.tier = Tier::L2_DoubleTrack;
  spec.initial_state.vel_body.x = speed;
  spec.policy = [speed](const VehicleState& s, double, Rng& rng) {
    // A normal, deliberately: normal() keeps a spare value between calls, so
    // an agent that has drawn an odd number of them is half a draw out of
    // step with one that has not. That is the part of the generator's state a
    // snapshot is most likely to forget.
    const double wobble = rng.normal() * 0.01;
    return DriveInput{wobble, slipx::sim::hold_speed(s, speed)};
  };
  return spec;
}

Simulation build(RunMode mode = RunMode::kDeterministic) {
  SimulationConfig config;
  config.master_seed = 12345;
  config.mode = mode;
  Simulation sim(config);
  sim.add_agent(noisy_runner("a", 5.0));
  sim.add_agent(noisy_runner("b", 3.0));
  return sim;
}

// ---------------------------------------------------------------- snapshots

TEST(Snapshot, RestoringMidRunReproducesTheUninterruptedRunBitForBit) {
  // The reference: eight hundred steps, straight through.
  Simulation reference = build();
  reference.run(800);
  const std::string wanted = reference.trajectory_hash();

  // The same run, snapshotted at three hundred, taken somewhere else
  // entirely, then restored and finished.
  Simulation interrupted = build();
  interrupted.run(300);
  const SimulationSnapshot saved = interrupted.snapshot();

  interrupted.run(250);  // a divergent future, thrown away

  interrupted.restore(saved);
  EXPECT_EQ(interrupted.step_count(), 300u);

  interrupted.run(500);

  EXPECT_EQ(interrupted.step_count(), 800u);
  EXPECT_EQ(interrupted.trajectory_hash(), wanted);

  for (std::size_t i = 0; i < reference.agent_count(); ++i) {
    EXPECT_EQ(interrupted.agent_trajectory_hash(i),
              reference.agent_trajectory_hash(i))
        << "agent " << i;
    EXPECT_EQ(0, std::memcmp(&interrupted.state(i), &reference.state(i),
                             sizeof(VehicleState)))
        << "agent " << i << " state differs";
  }
}

// The specific failure a snapshot of the engine word alone would produce. It
// resumes correctly until something asks for a normal, and then diverges for
// a reason that looks like anything except a missing bool.
TEST(Snapshot, CarriesTheGeneratorsSpareNormal) {
  Simulation sim = build();

  // An odd number of steps, so each agent has drawn an odd number of normals
  // and is holding a spare.
  sim.run(101);
  const SimulationSnapshot saved = sim.snapshot();

  const double next_after_snapshot = sim.rng(0).normal();

  sim.restore(saved);
  const double next_after_restore = sim.rng(0).normal();

  EXPECT_DOUBLE_EQ(next_after_restore, next_after_snapshot);
}

TEST(Snapshot, CarriesTheRunningHashAndNotJustTheStates) {
  Simulation sim = build();
  sim.run(200);
  const std::string at_snapshot = sim.trajectory_hash();

  const SimulationSnapshot saved = sim.snapshot();
  sim.run(50);
  ASSERT_NE(sim.trajectory_hash(), at_snapshot);

  sim.restore(saved);
  EXPECT_EQ(sim.trajectory_hash(), at_snapshot)
      << "the hash is a running fold and has to be resumed with the states";
}

TEST(Snapshot, IsAMemcpyOfTheStateAndNothingClever) {
  Simulation sim = build();
  sim.run(10);

  const SimulationSnapshot saved = sim.snapshot();
  ASSERT_EQ(saved.agents.size(), 2u);
  EXPECT_EQ(saved.steps, 10u);

  // The state is trivially copyable by design (CORE-03), which is what makes
  // this cheap enough to do often.
  EXPECT_TRUE(std::is_trivially_copyable<VehicleState>::value);
  EXPECT_EQ(0, std::memcmp(&saved.agents[0].state, &sim.state(0),
                           sizeof(VehicleState)));
}

TEST(Snapshot, TruncatesTheInputLogSoAResumedRunLogsWhatItWouldHave) {
  Simulation reference = build();
  reference.set_input_logging(true);
  reference.run(400);

  Simulation interrupted = build();
  interrupted.set_input_logging(true);
  interrupted.run(200);
  const SimulationSnapshot saved = interrupted.snapshot();
  interrupted.run(200);
  interrupted.restore(saved);
  interrupted.run(200);

  ASSERT_EQ(interrupted.input_log().size(), reference.input_log().size());
  for (std::size_t i = 0; i < reference.input_log().size(); ++i) {
    EXPECT_DOUBLE_EQ(interrupted.input_log()[i].steer_cmd,
                     reference.input_log()[i].steer_cmd)
        << "entry " << i;
  }
}

TEST(Snapshot, RefusesASnapshotFromADifferentlyShapedRun) {
  Simulation two = build();
  two.run(10);
  const SimulationSnapshot from_two = two.snapshot();

  SimulationConfig config;
  Simulation one(config);
  one.add_agent(noisy_runner("only", 5.0));

  EXPECT_THROW(one.restore(from_two), std::invalid_argument);
}

// ---------------------------------------------------------------- run modes

TEST(RunMode, IsDeterministicUnlessAskedOtherwise) {
  // A default that had drifted to validation would make every run in the
  // library quietly unreproducible, which is the one failure the whole design
  // is arranged around.
  EXPECT_EQ(SimulationConfig{}.mode, RunMode::kDeterministic);
  EXPECT_EQ(Simulation{}.mode(), RunMode::kDeterministic);
}

TEST(RunMode, ValidationRunsAreStillTheSameArithmetic) {
  // Pacing changes when steps happen, not what they compute. With no sensor
  // latency in the loop yet, a validation run of the same scenario produces
  // the same trajectory; what it stops promising is that it will keep doing
  // so on a loaded machine, which is what the manifest says.
  SimulationConfig config;
  config.master_seed = 12345;
  config.mode = RunMode::kValidation;
  config.real_time_factor = 1000.0;  // so the test does not take real seconds

  Simulation paced(config);
  paced.add_agent(noisy_runner("a", 5.0));
  paced.add_agent(noisy_runner("b", 3.0));
  paced.run(100);

  Simulation fast = build();
  fast.run(100);

  EXPECT_EQ(paced.trajectory_hash(), fast.trajectory_hash());
}

TEST(RunMode, TheManifestSaysWhichModeProducedIt) {
  Simulation deterministic = build();
  deterministic.run(10);
  const auto plain = deterministic.manifest();

  EXPECT_EQ(plain.run_mode, "deterministic");
  EXPECT_NE(plain.to_json().find("bit-identical"), std::string::npos);

  SimulationConfig config;
  config.mode = RunMode::kValidation;
  config.real_time_factor = 1000.0;
  Simulation validation(config);
  validation.add_agent(noisy_runner("a", 5.0));
  validation.run(10);
  const auto paced = validation.manifest();

  EXPECT_EQ(paced.run_mode, "validation");
  const std::string json = paced.to_json();
  EXPECT_NE(json.find("NOT REPRODUCIBLE"), std::string::npos)
      << "a manifest that claimed bit-identity for a paced run would be the "
         "most damaging line in this library";
  EXPECT_EQ(json.find("\"within_build\": \"bit-identical"), std::string::npos);
}

TEST(RunMode, IsPartOfTheConfigurationDigest) {
  // Two runs in different modes are not comparable whatever else matches, so
  // the digest that says "these two runs are the same experiment" has to
  // separate them.
  Simulation deterministic = build();
  deterministic.run(10);

  SimulationConfig config;
  config.master_seed = 12345;
  config.mode = RunMode::kValidation;
  config.real_time_factor = 1000.0;
  Simulation validation(config);
  validation.add_agent(noisy_runner("a", 5.0));
  validation.add_agent(noisy_runner("b", 3.0));
  validation.run(10);

  EXPECT_NE(deterministic.manifest().configuration_digest(),
            validation.manifest().configuration_digest());
}

}  // namespace
