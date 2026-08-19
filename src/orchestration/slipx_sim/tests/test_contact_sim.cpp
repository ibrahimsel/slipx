// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The contact pass in the orchestrator (ADR-0043): footprints are declared,
// pairs are resolved between steps in fixed order, a scenario without
// footprints reproduces its pre-contact trajectory bit for bit, and a DNF'd
// car really is the immovable obstacle ADR-0042 promised.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include "slipx/sim/manoeuvres.hpp"
#include "slipx/sim/simulation.hpp"

namespace {

using slipx::DriveInput;
using slipx::Tier;
using slipx::VehicleState;
using slipx::sim::AgentSpec;
using slipx::sense::Rng;
using slipx::sim::Simulation;
using slipx::sim::SimulationConfig;

constexpr double kLength = 0.55;
constexpr double kWidth = 0.30;

// A coasting L1 car with a footprint. Coasting on purpose: a policy would
// keep driving the cars back into each other, and what is under test is the
// collision, not the controller's response to it.
AgentSpec coaster(double x, double y, double yaw, double speed,
                  bool footprint = true) {
  AgentSpec spec;
  spec.tier = Tier::L1_Bicycle;
  spec.initial_state.pos.x = x;
  spec.initial_state.pos.y = y;
  spec.initial_state.yaw = yaw;
  spec.initial_state.vel_body.x = speed;
  if (footprint) {
    spec.footprint_length = kLength;
    spec.footprint_width = kWidth;
  }
  return spec;
}

TEST(ContactSim, FootprintsAreBothOrNeitherAndNeverNegative) {
  Simulation sim;
  AgentSpec spec = coaster(0.0, 0.0, 0.0, 1.0);
  spec.footprint_width = 0.0;
  EXPECT_THROW(sim.add_agent(spec), std::invalid_argument);
  spec.footprint_width = -0.3;
  EXPECT_THROW(sim.add_agent(spec), std::invalid_argument);

  SimulationConfig config;
  config.contact.restitution = 1.5;
  EXPECT_THROW(Simulation{config}, std::invalid_argument);
  config.contact.restitution = 0.3;
  config.contact.friction = -0.1;
  EXPECT_THROW(Simulation{config}, std::invalid_argument);
}

// The assertion the whole slice was designed around: contact that never
// happens changes nothing. A single footprinted agent, and a footprinted
// pair too far apart to touch, produce the trajectories their footprint-free
// twins produce, bit for bit. The published conformance rows are the same
// fact checked against history by tools/check_conformance.py.
TEST(ContactSim, AFootprintThatTouchesNothingChangesNothing) {
  Simulation bare;
  bare.add_agent(coaster(0.0, 0.0, 0.0, 5.0, false));
  Simulation shod;
  shod.add_agent(coaster(0.0, 0.0, 0.0, 5.0, true));
  bare.run_for(2.0);
  shod.run_for(2.0);
  EXPECT_EQ(bare.trajectory_hash(), shod.trajectory_hash());

  Simulation far_bare;
  far_bare.add_agent(coaster(0.0, 0.0, 0.0, 5.0, false));
  far_bare.add_agent(coaster(0.0, 10.0, 0.0, 5.0, false));
  Simulation far_shod;
  far_shod.add_agent(coaster(0.0, 0.0, 0.0, 5.0, true));
  far_shod.add_agent(coaster(0.0, 10.0, 0.0, 5.0, true));
  far_bare.run_for(2.0);
  far_shod.run_for(2.0);
  EXPECT_EQ(far_bare.trajectory_hash(), far_shod.trajectory_hash());
}

TEST(ContactSim, GhostsPassThroughAndFootprintsDoNot) {
  // Head-on: one car eastbound, one westbound on the same line.
  const auto build = [](bool footprints) {
    Simulation sim;
    sim.add_agent(coaster(0.0, 0.0, 0.0, 5.0, footprints));
    sim.add_agent(coaster(4.0, 0.0, slipx::kPi, 5.0, footprints));
    return sim;
  };

  // Without footprints they occupy the same space and carry on: the
  // pre-contact behaviour, preserved because nothing was declared.
  Simulation ghosts = build(false);
  ghosts.run_for(1.0);
  EXPECT_GT(ghosts.state(0).pos.x, ghosts.state(1).pos.x)
      << "the ghosts drove through each other";

  // A mixed pair is a ghost pair: contact needs a footprint on BOTH sides,
  // because an impulse against a car that cannot be touched back would
  // break momentum conservation before it broke anything else.
  Simulation mixed;
  mixed.add_agent(coaster(0.0, 0.0, 0.0, 5.0, true));
  mixed.add_agent(coaster(4.0, 0.0, slipx::kPi, 5.0, false));
  mixed.run_for(1.0);
  EXPECT_GT(mixed.state(0).pos.x, mixed.state(1).pos.x);

  // With footprints the gap never closes below the two half-lengths.
  Simulation solid = build(true);
  double min_gap = 1e9;
  for (int i = 0; i < 1000; ++i) {
    solid.advance();
    min_gap = std::min(min_gap, solid.state(1).pos.x - solid.state(0).pos.x);
  }
  EXPECT_LT(min_gap, 1.5 * kLength) << "they never met, so the scenario "
                                       "asserts nothing";
  EXPECT_GT(min_gap, kLength - 1e-9)
      << "two footprinted cars overlapped in an observed state";
}

TEST(ContactSim, ACollisionIsDeterministicAndReplays) {
  const auto build = [] {
    Simulation sim;
    // A glancing rear-end: offset laterally so the impulse carries a yaw
    // moment and the friction term is nonzero.
    sim.add_agent(coaster(0.0, 0.00, 0.0, 6.0));
    sim.add_agent(coaster(2.0, 0.15, 0.0, 3.0));
    return sim;
  };

  Simulation first = build();
  first.set_input_logging(true);
  first.run_for(1.5);

  Simulation second = build();
  second.run_for(1.5);
  EXPECT_EQ(first.trajectory_hash(), second.trajectory_hash());

  // The collision happened (the slow car was shoved) or this test asserts
  // nothing.
  EXPECT_GT(second.state(1).vel_body.x, 3.0 - 1e-9);
  EXPECT_NE(second.state(1).rates.z, 0.0) << "the offset hit must yaw it";

  const auto log = first.input_log();
  first.replay(log);
  EXPECT_EQ(first.trajectory_hash(), second.trajectory_hash());
}

TEST(ContactSim, MirrorSymmetryHoldsThroughACollision) {
  const auto build = [](double side) {
    Simulation sim;
    sim.add_agent(coaster(0.0, side * 0.00, side * 0.0, 6.0));
    sim.add_agent(coaster(2.0, side * 0.15, side * 0.1, 3.0));
    return sim;
  };

  Simulation left = build(1.0);
  Simulation right = build(-1.0);
  left.run_for(1.5);
  right.run_for(1.5);

  for (std::size_t i = 0; i < 2; ++i) {
    const VehicleState& l = left.state(i);
    const VehicleState& r = right.state(i);
    // EXPECT_EQ on doubles, deliberately: a mirrored race is the same race
    // bit for bit, or the invariant is not held at all.
    EXPECT_EQ(l.pos.x, r.pos.x);
    EXPECT_EQ(l.pos.y, -r.pos.y);
    EXPECT_EQ(l.yaw, -r.yaw);
    EXPECT_EQ(l.vel_body.x, r.vel_body.x);
    EXPECT_EQ(l.vel_body.y, -r.vel_body.y);
    EXPECT_EQ(l.rates.z, -r.rates.z);
  }
}

TEST(ContactSim, AFrozenCarIsAnImmovableCollidableObstacle) {
  // Roll a car (the ADR-0042 scenario), then drive another into the wreck.
  SimulationConfig config;
  Simulation sim(config);

  AgentSpec roller;
  roller.tier = Tier::L2_DoubleTrack;
  roller.params.h_cog = 0.18;
  roller.params.tyre_front.mu_y0 = 1.6;
  roller.params.tyre_front.mu_x0 = 1.7;
  roller.params.tyre_rear.mu_y0 = 1.6;
  roller.params.tyre_rear.mu_x0 = 1.7;
  roller.initial_state.vel_body.x = 6.0;
  // Started already past a quarter turn, so the wreck freezes with cos(yaw)
  // negative. That is deliberate: byte-stability of the frozen state under
  // sustained contact is only a sharp assertion where the world-frame
  // round-trip of an exact zero would launder a -0.0 into the state, and
  // that needs this quadrant.
  roller.initial_state.yaw = 2.0;
  roller.footprint_length = kLength;
  roller.footprint_width = kWidth;
  roller.policy = [](const VehicleState&, double t, Rng&) {
    return DriveInput{std::min(0.15 * t, 0.40), 0.0};
  };
  sim.add_agent(roller);
  // Parked far away until the wreck exists; teleported in afterwards.
  sim.add_agent(coaster(100.0, 100.0, 0.0, 0.0));

  for (int i = 0; i < 3000 && sim.agent_running(0); ++i) sim.advance();
  ASSERT_FALSE(sim.agent_running(0));

  // Ram the wreck broadside: the second car starts 1.5 m off the wreck's
  // left flank, heading straight at it. A side impact (rather than one
  // along the wreck's heading) is what makes the byte-stability assertion
  // sharp; see the yaw note above.
  const VehicleState wreck = sim.state(0);
  VehicleState& rammer = sim.state(1);
  const double lx = -std::sin(wreck.yaw);
  const double ly = std::cos(wreck.yaw);
  rammer.pos.x = wreck.pos.x + 1.5 * lx;
  rammer.pos.y = wreck.pos.y + 1.5 * ly;
  rammer.yaw = wreck.yaw - 0.5 * slipx::kPi;
  rammer.vel_body.x = 3.0;

  VehicleState frozen{};
  std::memcpy(&frozen, &sim.state(0), sizeof(VehicleState));

  bool bounced = false;
  for (int i = 0; i < 1000; ++i) {
    sim.advance();
    if (sim.state(1).vel_body.x < 0.0) bounced = true;
  }

  EXPECT_TRUE(bounced) << "the rammer must rebound off the wreck";
  EXPECT_EQ(std::memcmp(&frozen, &sim.state(0), sizeof(VehicleState)), 0)
      << "no impulse moves a DNF'd car";

  // And the wreck's slot in the manifest still says why it stopped.
  EXPECT_EQ(sim.manifest().agents[0].status, "dnf");
}

TEST(ContactSim, TheManifestRecordsFootprintsAndContactConstants) {
  SimulationConfig config;
  config.contact.restitution = 0.25;
  config.contact.friction = 0.6;
  Simulation sim(config);
  sim.add_agent(coaster(0.0, 0.0, 0.0, 1.0, true));
  sim.add_agent(coaster(5.0, 0.0, 0.0, 1.0, false));
  sim.run(10);

  const auto m = sim.manifest();
  EXPECT_DOUBLE_EQ(m.contact_restitution, 0.25);
  EXPECT_DOUBLE_EQ(m.contact_friction, 0.6);
  EXPECT_DOUBLE_EQ(m.agents[0].footprint_length, kLength);
  EXPECT_DOUBLE_EQ(m.agents[0].footprint_width, kWidth);
  EXPECT_DOUBLE_EQ(m.agents[1].footprint_length, 0.0);

  // Configuration, so the digest must move with it: a run that could not
  // be hit was a different race.
  SimulationConfig other = config;
  other.contact.restitution = 0.35;
  Simulation changed(other);
  changed.add_agent(coaster(0.0, 0.0, 0.0, 1.0, true));
  changed.add_agent(coaster(5.0, 0.0, 0.0, 1.0, false));
  EXPECT_NE(changed.manifest().configuration_digest(),
            m.configuration_digest());

  Simulation reshod(config);
  reshod.add_agent(coaster(0.0, 0.0, 0.0, 1.0, true));
  reshod.add_agent(coaster(5.0, 0.0, 0.0, 1.0, true));
  EXPECT_NE(reshod.manifest().configuration_digest(),
            m.configuration_digest());
}

}  // namespace
