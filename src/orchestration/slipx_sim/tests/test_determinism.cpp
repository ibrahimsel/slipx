// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// SIM-03, SIM-07 and NFR-02 at the level the promise is actually made.
//
// The core's own tests establish that one model is a deterministic function of
// its arguments. That is necessary and not sufficient: what a competition
// organiser, a course instructor and a leaderboard rely on is that a whole
// run, with N agents, seeded noise and a recorded input sequence, reproduces
// bit for bit. This file tests that, and tests the ways it could plausibly
// stop being true.

#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "slipx/sim/manoeuvres.hpp"
#include "slipx/sim/simulation.hpp"

namespace {

using slipx::DriveInput;
using slipx::Tier;
using slipx::VehicleState;
using slipx::sim::AgentSpec;
using slipx::sim::Rng;
using slipx::sim::Simulation;
using slipx::sim::SimulationConfig;

// A scenario with everything that could plausibly introduce nondeterminism:
// several agents, differing parameters, policies that consume random numbers,
// and a manoeuvre that saturates the tyres.
Simulation build_race(std::uint64_t seed = 314159) {
  SimulationConfig config;
  config.master_seed = seed;
  Simulation sim(config);

  for (int i = 0; i < 5; ++i) {
    AgentSpec spec;
    spec.name = "car" + std::to_string(i);
    spec.tier = (i % 2 == 0) ? Tier::L1_Bicycle : Tier::L0_Kinematic;
    spec.params.mass = 3.0 + 0.2 * i;
    spec.params.c_alpha_f = 110.0 + 5.0 * i;
    spec.initial_state.vel_body.x = 4.0 + 0.1 * i;
    spec.initial_state.pos.y = 0.5 * i;
    const double phase = 0.3 * i;
    spec.policy = [phase](const VehicleState& s, double t, Rng& rng) {
      // Noise drawn every step, so a policy that consumed a shared or
      // unseeded stream would show up here immediately.
      const double jitter = 0.002 * rng.normal();
      const double steer = 0.15 * std::sin(2.0 * t + phase) + jitter;
      return DriveInput{steer, slipx::sim::hold_speed(s, 5.0)};
    };
    sim.add_agent(std::move(spec));
  }
  return sim;
}

TEST(Determinism, TwoIdenticalRunsAgreeBitForBit) {
  Simulation a = build_race();
  Simulation b = build_race();
  a.run_for(5.0);
  b.run_for(5.0);

  EXPECT_EQ(a.trajectory_hash(), b.trajectory_hash());
  for (std::size_t i = 0; i < a.agent_count(); ++i) {
    EXPECT_EQ(a.agent_trajectory_hash(i), b.agent_trajectory_hash(i));
    EXPECT_EQ(std::memcmp(&a.state(i), &b.state(i), sizeof(VehicleState)), 0)
        << "agent " << i;
  }
}

TEST(Determinism, ResetAndRerunReproducesTheSameHash) {
  Simulation sim = build_race();
  sim.run_for(5.0);
  const std::string first = sim.trajectory_hash();

  sim.reset();
  sim.run_for(5.0);
  EXPECT_EQ(sim.trajectory_hash(), first);
}

TEST(Determinism, ADifferentSeedGivesADifferentRun) {
  Simulation a = build_race(1);
  Simulation b = build_race(2);
  a.run_for(2.0);
  b.run_for(2.0);
  EXPECT_NE(a.trajectory_hash(), b.trajectory_hash())
      << "the seed must actually reach the agents";
}

// SIM-07: replay from a recorded input sequence, with the policies gone.
TEST(Determinism, ReplayFromTheInputLogIsBitIdentical) {
  Simulation sim = build_race();
  sim.set_input_logging(true);
  sim.run_for(5.0);

  const std::string original = sim.trajectory_hash();
  const std::vector<DriveInput> log = sim.input_log();
  ASSERT_EQ(log.size(), 5000u * sim.agent_count());

  std::vector<VehicleState> final_states;
  for (std::size_t i = 0; i < sim.agent_count(); ++i) {
    final_states.push_back(sim.state(i));
  }

  sim.replay(log);
  EXPECT_EQ(sim.trajectory_hash(), original);
  EXPECT_EQ(sim.step_count(), 5000u);
  for (std::size_t i = 0; i < sim.agent_count(); ++i) {
    EXPECT_EQ(std::memcmp(&sim.state(i), &final_states[i],
                          sizeof(VehicleState)), 0)
        << "agent " << i;
  }
}

// The adjudication case: a fresh process, with the policies unavailable,
// reproducing a disputed result from the log alone.
TEST(Determinism, AFreshSimulationReplaysAnotherRunsLog) {
  Simulation recorded = build_race();
  recorded.set_input_logging(true);
  recorded.run_for(3.0);
  const std::string expected = recorded.trajectory_hash();

  // Rebuilt with no policies at all: same cars, same seeds, no controllers.
  SimulationConfig config;
  config.master_seed = 314159;
  Simulation adjudicator(config);
  for (int i = 0; i < 5; ++i) {
    AgentSpec spec;
    spec.name = "car" + std::to_string(i);
    spec.tier = (i % 2 == 0) ? Tier::L1_Bicycle : Tier::L0_Kinematic;
    spec.params.mass = 3.0 + 0.2 * i;
    spec.params.c_alpha_f = 110.0 + 5.0 * i;
    spec.initial_state.vel_body.x = 4.0 + 0.1 * i;
    spec.initial_state.pos.y = 0.5 * i;
    adjudicator.add_agent(std::move(spec));
  }

  adjudicator.replay(recorded.input_log());
  EXPECT_EQ(adjudicator.trajectory_hash(), expected);
}

TEST(Determinism, ReplayRejectsALogFromADifferentScenario) {
  Simulation sim = build_race();
  sim.set_input_logging(true);
  sim.run_for(0.1);
  std::vector<DriveInput> log = sim.input_log();
  log.pop_back();  // no longer a whole number of steps for five agents
  EXPECT_THROW(sim.replay(log), std::invalid_argument);
}

// The hash has to be sensitive to the things that make two runs different,
// or it certifies nothing. Each of these is a change somebody could make
// while believing it harmless.
TEST(Determinism, TheHashNoticesEveryConfigurationChange) {
  const auto hash_of = [](void (*mutate)(SimulationConfig&, AgentSpec&)) {
    SimulationConfig config;
    config.master_seed = 42;
    AgentSpec spec;
    spec.tier = Tier::L1_Bicycle;
    spec.initial_state.vel_body.x = 5.0;
    spec.policy = slipx::sim::step_steer();
    mutate(config, spec);

    Simulation sim(config);
    sim.add_agent(std::move(spec));
    sim.run_for(3.0);
    return sim.trajectory_hash();
  };

  const std::string base = hash_of([](SimulationConfig&, AgentSpec&) {});

  EXPECT_NE(base, hash_of([](SimulationConfig& c, AgentSpec&) {
              c.integrator = slipx::Integrator::kSemiImplicitEuler;
            }));
  EXPECT_NE(base, hash_of([](SimulationConfig& c, AgentSpec&) {
              c.dt = 5.0e-4;
            }));
  EXPECT_NE(base, hash_of([](SimulationConfig&, AgentSpec& s) {
              s.params.mass += 0.001;
            }));
  EXPECT_NE(base, hash_of([](SimulationConfig&, AgentSpec& s) {
              s.tier = Tier::L0_Kinematic;
            }));
  EXPECT_NE(base, hash_of([](SimulationConfig&, AgentSpec& s) {
              s.initial_state.vel_body.x = 5.000001;
            }));
}

// A larger stride must not change what the hash sees at the steps it does
// sample, only how many it samples.
TEST(Determinism, HashStrideSamplesLessWithoutChangingTheRun) {
  const auto run = [](std::uint64_t stride) {
    SimulationConfig config;
    config.hash_stride = stride;
    Simulation sim(config);
    AgentSpec spec;
    spec.tier = Tier::L1_Bicycle;
    spec.initial_state.vel_body.x = 5.0;
    spec.policy = slipx::sim::step_steer();
    sim.add_agent(std::move(spec));
    sim.run_for(3.0);
    return sim;
  };

  Simulation dense = run(1);
  Simulation sparse = run(10);

  EXPECT_NE(dense.trajectory_hash(), sparse.trajectory_hash())
      << "fewer samples is a different hash";
  EXPECT_EQ(std::memcmp(&dense.state(0), &sparse.state(0),
                        sizeof(VehicleState)), 0)
      << "but the same trajectory";
}

TEST(Determinism, HashStrideOfZeroIsRefused) {
  SimulationConfig config;
  config.hash_stride = 0;
  EXPECT_THROW(Simulation{config}, std::invalid_argument);
}

// The published conformance run. Its hash is pinned per build in
// conformance/reference_hashes.tsv and checked by tools/check_conformance.py
// on every commit; here it is only required to be stable within the process.
// Hard-coding a value in this test would make the suite fail on a
// legitimately different compiler or architecture, which is behaviour NFR-03
// documents rather than forbids, and would turn a real guarantee into a
// nuisance that somebody eventually deletes.
TEST(Determinism, ConformanceRunIsStableAndReportsItsIdentity) {
  Simulation a = slipx::sim::make_conformance_run();
  a.run_for(5.0);
  Simulation b = slipx::sim::make_conformance_run();
  b.run_for(5.0);

  EXPECT_EQ(a.trajectory_hash(), b.trajectory_hash());
  EXPECT_EQ(a.trajectory_hash().size(), 16u);

  const auto manifest = a.manifest();
  EXPECT_EQ(manifest.steps, 5000u);
  EXPECT_EQ(manifest.integrator, "rk4");
  EXPECT_EQ(manifest.agents.size(), 1u);
  EXPECT_EQ(manifest.agents[0].tier, "L1_Bicycle");
  EXPECT_FALSE(manifest.configuration_digest().empty());
}

}  // namespace
