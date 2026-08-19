// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Shared fixtures for the race-control scenario tests: the shipped track,
// and a centreline-following policy good enough to lap it. The policy is
// a test fixture, not a reference controller: it exists so the scenarios
// can put cars where the rules need them.

#ifndef SLIPX_RACE_TESTS_RACE_TEST_SUPPORT_HPP
#define SLIPX_RACE_TESTS_RACE_TEST_SUPPORT_HPP

#include <algorithm>
#include <cmath>
#include <string>

#include "slipx/race/starts.hpp"
#include "slipx/scene/projection.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/sim/simulation.hpp"

namespace race_test {

inline slipx::scene::Track shipped_track() {
  slipx::scene::Centreline geometry = slipx::scene::Centreline::from_file(
      std::string(SLIPX_EXAMPLE_TRACK_DIR) + "/centreline.csv");
  slipx::scene::TrackManifest manifest;
  manifest.name = "paddock_stadium";
  manifest.surface = "carpet";
  manifest.closed = true;
  manifest.geometry_source = "examples/tracks/make_tracks.py";
  manifest.geometry_licence = "Apache-2.0";
  manifest.provenance_label = "provisional";
  return slipx::scene::Track::build(geometry, manifest,
                                    {{"sponge", "carpet"}});
}

// Follow the centreline at a lateral offset and a target speed: project,
// aim at a lookahead point, steer proportionally. Captures the track by
// reference; the track must outlive the policy.
inline slipx::sim::Policy follow_centreline(const slipx::scene::Track& track,
                                            double speed,
                                            double lateral = 0.0,
                                            double lookahead = 0.8) {
  return [&track, speed, lateral, lookahead](
             const slipx::VehicleState& s, double, slipx::sense::Rng&) {
    const slipx::scene::Projection where =
        slipx::scene::project(track, s.pos.x, s.pos.y);
    const slipx::race::TrackPose target =
        slipx::race::pose_at(track, where.s + lookahead);
    const double nx = -std::sin(target.heading);
    const double ny = std::cos(target.heading);
    const double tx = target.x + lateral * nx;
    const double ty = target.y + lateral * ny;

    const double desired = std::atan2(ty - s.pos.y, tx - s.pos.x);
    const double error =
        std::atan2(std::sin(desired - s.yaw), std::cos(desired - s.yaw));

    slipx::DriveInput input;
    input.steer_cmd = std::max(-0.4, std::min(0.4, 1.5 * error));
    input.accel_cmd = 4.0 * (speed - s.vel_body.x);
    return input;
  };
}

// An L1 car with a footprint, ready for a race.
inline slipx::sim::AgentSpec race_car(slipx::sim::Policy policy) {
  slipx::sim::AgentSpec spec;
  spec.tier = slipx::Tier::L1_Bicycle;
  spec.policy = std::move(policy);
  spec.footprint_length = 0.50;
  spec.footprint_width = 0.30;
  return spec;
}

}  // namespace race_test

#endif  // SLIPX_RACE_TESTS_RACE_TEST_SUPPORT_HPP
