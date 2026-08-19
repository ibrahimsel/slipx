// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Rollover as a discrete event (ADR-0042): detected by the orchestrator from
// the step's diagnostics, both wheels of one side at zero vertical load, and
// terminal. The scenarios here drive a slow steering ramp at constant speed,
// so the lateral acceleration rises quasi-statically and the event should
// fire where the static analysis says wheels lift, which is what makes the
// threshold assertable rather than merely observed.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

#include "slipx/load_transfer.hpp"
#include "slipx/sim/manoeuvres.hpp"
#include "slipx/sim/simulation.hpp"

namespace {

using slipx::DriveInput;
using slipx::Tier;
using slipx::VehicleParams;
using slipx::VehicleState;
using slipx::sim::AgentSpec;
using slipx::sim::DnfCause;
using slipx::sim::DnfEvent;
using slipx::sense::Rng;
using slipx::sim::Simulation;
using slipx::sim::SimulationConfig;

// A steering ramp while coasting from speed, on a sticky tyre. Each choice
// is load-bearing, and all three were found by probing rather than assumed:
//
//   Sticky (mu_y0 1.6): rollover is the sticky-surface, tall-CoG failure.
//   On the default 1.1 the tyres let go near one g and the car slides,
//   which is the honest and much safer outcome the load-transfer notes
//   promise.
//   Coasting: drive thrust transfers load rearward and props up the rear
//   inner wheel, so a powered car three-wheels indefinitely instead of
//   lifting the whole side. Rollover wants the lateral transfer unpolluted.
//   From speed (6 m/s): the threshold is then reached at a small steering
//   angle, well inside the servo travel; at car-park speed the steering
//   runs out before the lateral acceleration gets there.
//
// The default h_cog of 0.06 m cannot roll (threshold two g against a tyre
// budget near 1.45 g on this compound); raising it is what brings the
// threshold down into what the tyres can sustain, which is exactly the
// sweep the event has to be tested over.
AgentSpec ramp_corner(double h_cog, double steer_sign = 1.0,
                      double speed = 6.0, double ramp_rate = 0.15) {
  AgentSpec spec;
  spec.name = "roller";
  spec.tier = Tier::L2_DoubleTrack;
  spec.params.h_cog = h_cog;
  spec.params.tyre_front.mu_y0 = 1.6;
  spec.params.tyre_front.mu_x0 = 1.7;
  spec.params.tyre_rear.mu_y0 = 1.6;
  spec.params.tyre_rear.mu_x0 = 1.7;
  spec.initial_state.vel_body.x = speed;
  spec.policy = [steer_sign, ramp_rate](const VehicleState&, double t, Rng&) {
    const double steer = steer_sign * std::min(ramp_rate * t, 0.40);
    return DriveInput{steer, 0.0};
  };
  return spec;
}

// Advance until the single agent DNFs or the duration runs out.
void run_until_event(Simulation& sim, double duration) {
  const auto steps =
      static_cast<std::uint64_t>(std::round(duration / sim.dt()));
  for (std::uint64_t i = 0; i < steps && sim.agent_running(0); ++i) {
    sim.advance();
  }
}

// The CoG sweep from the milestone's own acceptance line: heights whose
// static threshold sits above what the tyres can sustain never produce the
// event, heights below it always do, and where it fires the measured lateral
// acceleration agrees with the closed form.
TEST(RolloverEvent, FiresAcrossACoGSweepExactlyWhereTheStaticsSayItShould) {
  // With mu_y0 = 1.6 and load sensitivity k_mu = 0.15, the sustainable
  // lateral acceleration tops out near 1.45 g: full transfer doubles the
  // outer wheels' load and costs 2^-0.15 of their friction. Heights whose
  // threshold g t / 2h sits above that cannot be rolled by any steering
  // input; the marginal band between 14 and 12 m/s^2 (h from 0.08 to 0.10)
  // is deliberately left out of both lists.
  const double heights_that_cannot_roll[] = {0.06, 0.07};
  const double heights_that_roll[] = {0.12, 0.15, 0.18, 0.21};

  for (const double h : heights_that_cannot_roll) {
    Simulation sim;
    sim.add_agent(ramp_corner(h));
    run_until_event(sim, 5.0);
    EXPECT_TRUE(sim.agent_running(0))
        << "h_cog " << h << ": threshold "
        << slipx::static_rollover_threshold(sim.model(0).params())
        << " m/s^2 is past the friction limit, so the tyres must let go "
           "first and the car must slide, not roll";
  }

  std::uint64_t previous_step = 0;
  for (const double h : heights_that_roll) {
    Simulation sim;
    sim.add_agent(ramp_corner(h));
    run_until_event(sim, 5.0);
    ASSERT_FALSE(sim.agent_running(0)) << "h_cog " << h << " must roll";

    const DnfEvent& event = *sim.dnf(0);
    EXPECT_EQ(event.cause, DnfCause::kRolloverLeft)
        << "a left turn unloads the left wheels";
    EXPECT_DOUBLE_EQ(event.time,
                     static_cast<double>(event.step) * sim.dt());

    // The frozen diagnostics are the step that fired: both left wheels at
    // exactly zero, because the core's clamp writes a literal zero.
    EXPECT_EQ(sim.diagnostics(0).fz[slipx::kFrontLeft], 0.0);
    EXPECT_EQ(sim.diagnostics(0).fz[slipx::kRearLeft], 0.0);

    // The event should fire close to the closed-form threshold g t / 2h.
    // The band is one-sided-ish on purpose: the measured value sits a few
    // per cent BELOW the formula, because the loads are computed from the
    // first of L2's two passes and the coastdown decel shifts a little
    // load about, and neither effect should push it above.
    const double threshold =
        slipx::static_rollover_threshold(sim.model(0).params());
    EXPECT_GT(sim.diagnostics(0).ay, 0.85 * threshold);
    EXPECT_LT(sim.diagnostics(0).ay, 1.10 * threshold);

    // A taller car rolls no later on the same ramp: the threshold falls
    // monotonically with h, which is the one-line fact the tutorial series
    // insists on.
    if (previous_step != 0) EXPECT_LT(event.step, previous_step);
    previous_step = event.step;
  }
}

TEST(RolloverEvent, IsDeterministic) {
  Simulation a;
  a.add_agent(ramp_corner(0.18));
  Simulation b;
  b.add_agent(ramp_corner(0.18));

  a.run_for(2.0);
  b.run_for(2.0);

  ASSERT_FALSE(a.agent_running(0));
  ASSERT_FALSE(b.agent_running(0));
  EXPECT_EQ(a.dnf(0)->step, b.dnf(0)->step);
  EXPECT_EQ(a.dnf(0)->cause, b.dnf(0)->cause);
  EXPECT_EQ(a.trajectory_hash(), b.trajectory_hash());
}

// The cause names the unloaded side, and mirror symmetry holds through the
// event: a right turn is a left turn seen in a mirror, so the two must roll
// on the same step with mirrored causes.
TEST(RolloverEvent, TheCauseNamesTheUnloadedSide) {
  Simulation left;
  left.add_agent(ramp_corner(0.18, +1.0));
  Simulation right;
  right.add_agent(ramp_corner(0.18, -1.0));

  run_until_event(left, 3.0);
  run_until_event(right, 3.0);

  ASSERT_FALSE(left.agent_running(0));
  ASSERT_FALSE(right.agent_running(0));
  EXPECT_EQ(left.dnf(0)->cause, DnfCause::kRolloverLeft);
  EXPECT_EQ(right.dnf(0)->cause, DnfCause::kRolloverRight);
  EXPECT_EQ(left.dnf(0)->step, right.dnf(0)->step);
}

TEST(RolloverEvent, ADnfCarFreezesWhereItRolled) {
  auto policy_calls = std::make_shared<std::uint64_t>(0);
  AgentSpec spec = ramp_corner(0.18);
  const auto inner = spec.policy;
  spec.policy = [policy_calls, inner](const VehicleState& s, double t,
                                      Rng& rng) {
    ++*policy_calls;
    return inner(s, t, rng);
  };

  Simulation sim;
  sim.add_agent(std::move(spec));
  run_until_event(sim, 3.0);
  ASSERT_FALSE(sim.agent_running(0));

  const DnfEvent event = *sim.dnf(0);
  EXPECT_EQ(event.step, sim.step_count());
  // The policy ran once per step, up to and including the step that rolled,
  // and never again after it.
  EXPECT_EQ(*policy_calls, event.step);

  // The motion states are zeroed at the event: a stationary obstacle must
  // read as stationary, not as a car frozen mid-flight.
  EXPECT_EQ(sim.state(0).speed(), 0.0);
  EXPECT_EQ(sim.state(0).yaw_rate(), 0.0);
  EXPECT_EQ(sim.state(0).steer_rate, 0.0);
  for (const double w : sim.state(0).omega_w) EXPECT_EQ(w, 0.0);

  VehicleState at_event{};
  std::memcpy(&at_event, &sim.state(0), sizeof(VehicleState));

  sim.run(500);
  EXPECT_EQ(*policy_calls, event.step);
  EXPECT_EQ(std::memcmp(&at_event, &sim.state(0), sizeof(VehicleState)), 0)
      << "nothing about a frozen agent's state may evolve";
  EXPECT_EQ(sim.dnf(0)->step, event.step) << "the event does not re-fire";
}

// A frozen agent keeps its slot: the log stays rectangular and the other
// agents' trajectories are exactly what they would have been alone.
TEST(RolloverEvent, AFrozenAgentKeepsItsSlotAndDisturbsNobody) {
  SimulationConfig config;
  config.master_seed = 7;

  Simulation pair(config);
  pair.set_input_logging(true);
  pair.add_agent(ramp_corner(0.06));   // cannot roll
  pair.add_agent(ramp_corner(0.18));   // rolls
  pair.run_for(2.0);

  ASSERT_TRUE(pair.agent_running(0));
  ASSERT_FALSE(pair.agent_running(1));
  EXPECT_EQ(pair.input_log().size(), 2 * pair.step_count());

  Simulation solo(config);
  solo.add_agent(ramp_corner(0.06));
  solo.run_for(2.0);
  EXPECT_EQ(pair.agent_trajectory_hash(0), solo.agent_trajectory_hash(0));
}

TEST(RolloverEvent, SnapshotAndRestoreCarryTheEvent) {
  Simulation sim;
  sim.add_agent(ramp_corner(0.18));

  // Snapshot before the roll: the resumed run must reproduce the event and
  // the uninterrupted hash, bit for bit.
  sim.run_for(0.2);
  ASSERT_TRUE(sim.agent_running(0));
  const auto before = sim.snapshot();

  sim.run_for(2.0);
  ASSERT_FALSE(sim.agent_running(0));
  const DnfEvent event = *sim.dnf(0);
  const std::string hash = sim.trajectory_hash();

  sim.restore(before);
  EXPECT_TRUE(sim.agent_running(0)) << "restoring to before the event "
                                       "restores a running agent";
  sim.run_for(2.0);
  ASSERT_FALSE(sim.agent_running(0));
  EXPECT_EQ(sim.dnf(0)->step, event.step);
  EXPECT_EQ(sim.trajectory_hash(), hash);

  // Snapshot after the roll: restoring must not resurrect the car.
  const auto after = sim.snapshot();
  sim.reset();
  EXPECT_TRUE(sim.agent_running(0));
  sim.restore(after);
  ASSERT_FALSE(sim.agent_running(0));
  EXPECT_EQ(sim.dnf(0)->step, event.step);
  EXPECT_EQ(sim.dnf(0)->cause, event.cause);
}

TEST(RolloverEvent, ReplayReproducesTheRoll) {
  Simulation sim;
  sim.set_input_logging(true);
  sim.add_agent(ramp_corner(0.18));
  sim.run_for(2.0);
  ASSERT_FALSE(sim.agent_running(0));

  const DnfEvent event = *sim.dnf(0);
  const std::string hash = sim.trajectory_hash();
  const std::vector<DriveInput> log = sim.input_log();

  sim.replay(log);
  ASSERT_FALSE(sim.agent_running(0));
  EXPECT_EQ(sim.dnf(0)->step, event.step);
  EXPECT_EQ(sim.dnf(0)->cause, event.cause);
  EXPECT_EQ(sim.trajectory_hash(), hash);
}

TEST(RolloverEvent, ResetClearsTheEvent) {
  Simulation sim;
  sim.add_agent(ramp_corner(0.18));
  run_until_event(sim, 3.0);
  ASSERT_FALSE(sim.agent_running(0));

  sim.reset();
  EXPECT_TRUE(sim.agent_running(0));
  EXPECT_FALSE(sim.dnf(0).has_value());
  EXPECT_DOUBLE_EQ(sim.state(0).vel_body.x, 6.0);
}

// Below L2 the per-wheel loads are NaN: those tiers have no load transfer,
// so they have no rollover, however violent the manoeuvre. That is a stated
// limitation of the tiers, and the detection must not misread NaN as lifted.
TEST(RolloverEvent, TiersWithoutLoadTransferCannotRoll) {
  AgentSpec spec = ramp_corner(0.30);
  spec.tier = Tier::L1_Bicycle;

  Simulation sim;
  sim.add_agent(std::move(spec));
  sim.run_for(5.0);
  EXPECT_TRUE(sim.agent_running(0));
  EXPECT_FALSE(sim.dnf(0).has_value());
}

TEST(RolloverEvent, TheManifestSaysHowTheRunEnded) {
  Simulation sim;
  sim.add_agent(ramp_corner(0.06));
  sim.add_agent(ramp_corner(0.18));
  sim.run_for(2.0);
  ASSERT_FALSE(sim.agent_running(1));

  const auto manifest = sim.manifest();
  EXPECT_EQ(manifest.agents[0].status, "running");
  EXPECT_TRUE(manifest.agents[0].dnf_cause.empty());
  EXPECT_EQ(manifest.agents[1].status, "dnf");
  EXPECT_EQ(manifest.agents[1].dnf_cause, "rollover: left wheels unloaded");
  EXPECT_EQ(manifest.agents[1].dnf_step, sim.dnf(1)->step);

  const std::string json = manifest.to_json();
  EXPECT_NE(json.find("\"status\": \"running\""), std::string::npos);
  EXPECT_NE(json.find("\"status\": \"dnf\""), std::string::npos);
  EXPECT_NE(json.find("\"dnf_cause\": \"rollover: left wheels unloaded\""),
            std::string::npos);
  // And only there: a running agent's entry carries no dnf fields, rather
  // than empty ones a reader would have to interpret.
  EXPECT_EQ(json.find("\"dnf_cause\": \"\""), std::string::npos);

  // The digest answers "was this the same setup", and how the run ended is
  // an answer, not part of the question: two identically configured runs
  // stopped at different points must agree.
  Simulation fresh;
  fresh.add_agent(ramp_corner(0.06));
  fresh.add_agent(ramp_corner(0.18));
  EXPECT_EQ(fresh.manifest().configuration_digest(),
            manifest.configuration_digest());
}

}  // namespace
