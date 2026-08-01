// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// SIM-06: the run manifest.
//
// The manifest is the artefact somebody reads when a result does not
// reproduce and they need to work out whether that is a bug, a different car,
// or the documented cross-platform limitation. These tests assert that the
// fields needed to make that determination are all actually populated, which
// is the failure mode a manifest has: it is written once, never read until it
// matters, and then found to be missing the one field in question.

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "slipx/sim/manifest.hpp"
#include "slipx/sim/manoeuvres.hpp"
#include "slipx/sim/simulation.hpp"

namespace {

using slipx::Tier;
using slipx::sim::AgentSpec;
using slipx::sim::RunManifest;
using slipx::sim::Simulation;
using slipx::sim::SimulationConfig;

bool contains(const std::string& haystack, const std::string& needle) {
  return haystack.find(needle) != std::string::npos;
}

Simulation two_car_run() {
  SimulationConfig config;
  config.master_seed = 2026;
  config.schema_version = "0.1.0";
  Simulation sim(config);

  AgentSpec a;
  a.name = "alice";
  a.tier = Tier::L1_Bicycle;
  a.initial_state.vel_body.x = 5.0;
  a.policy = slipx::sim::step_steer();
  sim.add_agent(std::move(a));

  AgentSpec b;
  b.name = "bob";
  b.tier = Tier::L0_Kinematic;
  b.params.mass = 4.2;
  b.initial_state.vel_body.x = 4.0;
  sim.add_agent(std::move(b));

  sim.run_for(1.0);
  return sim;
}

TEST(Manifest, RecordsEverythingNeededToReproduceTheRun) {
  const RunManifest m = two_car_run().manifest();

  // Identity of the software.
  EXPECT_FALSE(m.slipx_core_version.empty());
  EXPECT_EQ(m.schema_version, "0.1.0");

  // Identity of the discretisation.
  EXPECT_DOUBLE_EQ(m.dt, 1.0e-3);
  EXPECT_EQ(m.steps, 1000u);
  EXPECT_EQ(m.integrator, "rk4");
  EXPECT_EQ(m.master_seed, 2026u);

  // Identity of the build (NFR-02: bit-identity is scoped to this).
  EXPECT_FALSE(m.compiler_id.empty());
  EXPECT_FALSE(m.compiler_version.empty());
  EXPECT_FALSE(m.build_type.empty());
  EXPECT_FALSE(m.system_name.empty());
  EXPECT_FALSE(m.system_processor.empty());
  EXPECT_FALSE(m.git_sha.empty());

  // The flag set has to include the one flag the determinism claim rests on.
  EXPECT_TRUE(contains(m.cxx_flags, "-ffp-contract=off")) << m.cxx_flags;

  // Identity of the cars.
  ASSERT_EQ(m.agents.size(), 2u);
  EXPECT_EQ(m.agents[0].name, "alice");
  EXPECT_EQ(m.agents[0].tier, "L1_Bicycle");
  EXPECT_EQ(m.agents[1].tier, "L0_Kinematic");
  EXPECT_NE(m.agents[0].seed, m.agents[1].seed);
  EXPECT_NE(m.agents[0].params_digest, m.agents[1].params_digest)
      << "different cars must have different digests";
  EXPECT_EQ(m.agents[0].params_digest.size(), 16u);

  // And the result.
  EXPECT_EQ(m.trajectory_hash.size(), 16u);
  ASSERT_EQ(m.agent_trajectory_hashes.size(), 2u);
  EXPECT_NE(m.agent_trajectory_hashes[0], m.agent_trajectory_hashes[1]);
}

// Per-agent hashes exist so that a failure says which car diverged. A
// whole-run hash alone would only ever say "something changed".
TEST(Manifest, PerAgentHashesLocaliseADivergence) {
  Simulation a = two_car_run();
  const RunManifest ma = a.manifest();

  SimulationConfig config;
  config.master_seed = 2026;
  config.schema_version = "0.1.0";
  Simulation b(config);
  AgentSpec alice;
  alice.name = "alice";
  alice.tier = Tier::L1_Bicycle;
  alice.initial_state.vel_body.x = 5.0;
  alice.policy = slipx::sim::step_steer();
  b.add_agent(std::move(alice));
  AgentSpec bob;
  bob.name = "bob";
  bob.tier = Tier::L0_Kinematic;
  bob.params.mass = 9.9;  // only bob differs, and only in a way L0 ignores
  bob.initial_state.vel_body.x = 4.0;
  b.add_agent(std::move(bob));
  b.run_for(1.0);
  const RunManifest mb = b.manifest();

  EXPECT_EQ(ma.agent_trajectory_hashes[0], mb.agent_trajectory_hashes[0])
      << "alice was not touched";
  EXPECT_EQ(ma.agent_trajectory_hashes[1], mb.agent_trajectory_hashes[1])
      << "and L0 ignores mass, so bob's trajectory is unchanged too";

  // But the configuration digest must still notice, because the car on paper
  // is a different car even where the tier cannot tell.
  EXPECT_NE(ma.configuration_digest(), mb.configuration_digest());
}

TEST(Manifest, ConfigurationDigestSeparatesSetupFromResult) {
  Simulation sim = two_car_run();
  const std::string early = sim.manifest().configuration_digest();
  sim.run_for(1.0);
  const RunManifest later = sim.manifest();

  EXPECT_EQ(later.configuration_digest(), early)
      << "running further does not change the setup";
  EXPECT_NE(later.trajectory_hash, "")
      << "but it does change the result";
}

TEST(Manifest, ConfigurationDigestNoticesASeedChange) {
  RunManifest a;
  a.master_seed = 1;
  RunManifest b;
  b.master_seed = 2;
  EXPECT_NE(a.configuration_digest(), b.configuration_digest());
}

TEST(Manifest, JsonIsWellFormedAndCarriesTheKeyFields) {
  const std::string json = two_car_run().manifest().to_json();

  // Balanced braces and brackets: the cheapest useful well-formedness check
  // that does not pull in a JSON parser to test a JSON writer.
  int braces = 0;
  int brackets = 0;
  for (const char c : json) {
    if (c == '{') ++braces;
    if (c == '}') --braces;
    if (c == '[') ++brackets;
    if (c == ']') --brackets;
    ASSERT_GE(braces, 0);
    ASSERT_GE(brackets, 0);
  }
  EXPECT_EQ(braces, 0);
  EXPECT_EQ(brackets, 0);

  EXPECT_TRUE(contains(json, "\"trajectory_hash\""));
  EXPECT_TRUE(contains(json, "\"configuration_digest\""));
  EXPECT_TRUE(contains(json, "\"integrator\": \"rk4\""));
  EXPECT_TRUE(contains(json, "\"alice\""));
  EXPECT_TRUE(contains(json, "\"bob\""));
  EXPECT_TRUE(contains(json, "\"git_sha\""));

  // NFR-03 is restated in the artefact, not only in the documentation.
  EXPECT_TRUE(contains(json, "across_platforms"));
  EXPECT_TRUE(contains(json, "not guaranteed"));
}

// The step size must round-trip exactly, or the manifest cannot reproduce the
// run it describes.
TEST(Manifest, NumbersArePrintedToFullPrecision) {
  SimulationConfig config;
  config.dt = 1.0 / 3000.0;  // not representable in a few digits
  Simulation sim(config);
  AgentSpec spec;
  spec.tier = Tier::L1_Bicycle;
  sim.add_agent(std::move(spec));

  const std::string json = sim.manifest().to_json();
  const std::size_t at = json.find("\"dt\": ");
  ASSERT_NE(at, std::string::npos);
  const double parsed = std::strtod(json.c_str() + at + 6, nullptr);
  EXPECT_EQ(parsed, config.dt) << "dt did not round-trip";
}

TEST(Manifest, WritesToDiskAndReportsFailureWithoutThrowing) {
  const RunManifest m = two_car_run().manifest();

  const std::string path =
      std::string(::testing::TempDir()) + "slipx_test.manifest.json";
  ASSERT_TRUE(m.write(path));

  std::ifstream in(path);
  ASSERT_TRUE(in.good());
  std::ostringstream buffer;
  buffer << in.rdbuf();
  EXPECT_EQ(buffer.str(), m.to_json());
  in.close();
  std::remove(path.c_str());

  // A run that finished should report its result even if the manifest cannot
  // be written, so this returns false rather than throwing.
  EXPECT_FALSE(m.write("/nonexistent-directory/slipx.manifest.json"));
}

TEST(Manifest, EmptyRunStillProducesAValidManifest) {
  Simulation sim;
  const RunManifest m = sim.manifest();
  EXPECT_EQ(m.agents.size(), 0u);
  EXPECT_EQ(m.steps, 0u);
  const std::string json = m.to_json();
  EXPECT_TRUE(contains(json, "\"agents\": [\n  ]"));
}

}  // namespace
