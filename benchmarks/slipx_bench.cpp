// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The performance benchmarks (M5.9).
//
// Hand-rolled rather than built on a benchmark library, for the same reason
// the manifest writer is hand-rolled: this has to run in CI on five platforms
// without a dependency anybody has to install, and what it measures is coarse
// enough that a harness with statistical machinery would be measuring its own
// cleverness. Each case runs a fixed amount of work and divides.
//
// Three numbers, and each is a claim somebody could act on:
//
//   step cost      what it costs to advance one L2 car by one step, which is
//                  what somebody embedding slipx_core cares about
//   one agent      how much faster than real time a single car with a LiDAR
//                  runs headless, which decides whether a training loop is
//                  worth starting
//   twenty agents  the same for a full grid, which decides whether a race
//                  can be run in CI
//
// The numbers are reported, not asserted. A CI machine is a shared virtual
// one with neighbours, and a threshold tight enough to be meaningful there
// would fail on a Tuesday for no reason; the ctest case around this checks
// that the benchmark runs and that nothing has regressed by an order of
// magnitude, which is the size of change that means somebody broke
// something rather than that the machine was busy.

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "slipx/scene/raycast.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/sense/lidar.hpp"
#include "slipx/sense/rng.hpp"
#include "slipx/sim/build_info.hpp"
#include "slipx/sim/simulation.hpp"
#include "slipx/vehicle_model.hpp"

namespace {

using slipx::DriveInput;
using slipx::Tier;
using slipx::VehicleModel;
using slipx::VehicleParams;
using slipx::VehicleState;
using slipx::scene::Centreline;
using slipx::scene::Track;
using slipx::scene::TrackManifest;
using slipx::scene::Walls;
using slipx::sense::Hit;
using slipx::sense::Lidar;
using slipx::sense::LidarSpec;
using slipx::sense::Pose;
using slipx::sense::Rng;

using Clock = std::chrono::steady_clock;

double seconds_since(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

Track shipped_track() {
  Centreline geometry = Centreline::from_file(
      std::string(SLIPX_EXAMPLE_TRACK_DIR) + "/centreline.csv");
  TrackManifest manifest;
  manifest.name = "paddock_stadium";
  manifest.surface = "carpet";
  manifest.closed = true;
  manifest.geometry_source = "examples/tracks/make_tracks.py";
  manifest.geometry_licence = "Apache-2.0";
  manifest.provenance_label = "provisional";
  return Track::build(geometry, manifest, {{"sponge", "carpet"}});
}

// One L2 step, in isolation. No sensors, no orchestrator: the number an
// embedder is buying.
double step_microseconds() {
  const VehicleParams params;
  const auto model = VehicleModel::create(Tier::L2_DoubleTrack, params);

  VehicleState state;
  state.vel_body.x = 5.0;
  for (auto& omega : state.omega_w) omega = state.vel_body.x / params.wheel_radius;
  const DriveInput input{0.05, 1.0};

  // A warm-up, so the first-touch page faults and the branch predictor's
  // education are not in the measurement.
  for (int i = 0; i < 20000; ++i) model->step(state, input, 1.0e-3, nullptr);

  const int steps = 400000;
  const auto start = Clock::now();
  for (int i = 0; i < steps; ++i) model->step(state, input, 1.0e-3, nullptr);
  const double elapsed = seconds_since(start);

  // Read the state afterwards so that nothing above can be optimised away.
  if (state.pos.x == 12345.6789) std::printf(" ");

  return elapsed / steps * 1.0e6;
}

// A grid of cars, each with a LiDAR looking at the track, run for a fixed
// amount of simulation time. The answer is the ratio of simulated time to
// wall time.
double real_time_factor(std::size_t agents, double simulated_seconds) {
  const Track track = shipped_track();
  const Walls walls(track);
  const VehicleParams params;

  LidarSpec spec;
  spec.rate_hz = 40.0;
  spec.rays = 1080;
  spec.range_max = 10.0;
  const Lidar lidar(spec);

  const auto world = [&walls, &spec](const Pose& origin, double bearing) {
    Hit hit;
    const auto found = walls.cast(origin.x, origin.y, bearing, spec.range_max);
    hit.hit = found.hit;
    hit.range = found.range;
    return hit;
  };

  slipx::sim::SimulationConfig config;
  config.master_seed = 1;
  slipx::sim::Simulation sim(config);

  for (std::size_t i = 0; i < agents; ++i) {
    slipx::sim::AgentSpec agent;
    agent.name = "car" + std::to_string(i);
    agent.tier = Tier::L2_DoubleTrack;
    agent.initial_state.pos.x = -4.0 + static_cast<double>(i) * 0.3;
    agent.initial_state.pos.y = -3.0;
    agent.initial_state.vel_body.x = 4.0;
    agent.policy = [](const VehicleState& s, double, Rng&) {
      return DriveInput{0.02, 2.0 * (4.0 - s.vel_body.x)};
    };
    sim.add_agent(agent);
  }

  const double dt = sim.dt();
  const auto steps = static_cast<std::uint64_t>(simulated_seconds / dt);
  const double scan_period = 1.0 / spec.rate_hz;

  std::vector<double> next_scan(agents, 0.0);
  Rng rng(7);

  const auto start = Clock::now();
  for (std::uint64_t step = 0; step < steps; ++step) {
    const double now = static_cast<double>(step) * dt;
    sim.advance();

    for (std::size_t i = 0; i < agents; ++i) {
      if (now < next_scan[i]) continue;
      const VehicleState frozen = sim.state(i);
      const auto pose_at = [frozen](double) {
        Pose pose;
        pose.x = frozen.pos.x;
        pose.y = frozen.pos.y;
        pose.yaw = frozen.yaw;
        return pose;
      };
      const auto scan = lidar.sample(now, pose_at, world, rng);
      if (scan.rays.empty()) std::printf(" ");  // keep the call
      next_scan[i] += scan_period;
    }
  }
  const double elapsed = seconds_since(start);

  return simulated_seconds / elapsed;
}

}  // namespace

int main() {
  std::printf("SlipX benchmarks\n");
  std::printf("  build      %s %s, %s\n", slipx::sim::kBuildCompilerId,
              slipx::sim::kBuildCompilerVersion, slipx::sim::kBuildType);
  std::printf("  platform   %s %s\n", slipx::sim::kBuildSystemName,
              slipx::sim::kBuildSystemProcessor);
  std::printf("\n");
  std::printf("  %-42s %10s %10s\n", "case", "measured", "target");

  const double step_us = step_microseconds();
  std::printf("  %-42s %8.3f us %8s\n", "L2 single-agent step", step_us,
              "< 5 us");

  const double one = real_time_factor(1, 5.0);
  std::printf("  %-42s %8.1f x  %8s\n",
              "1 agent, L2 + 2D LiDAR, headless", one, "> 100 x");

  const double twenty = real_time_factor(20, 2.0);
  std::printf("  %-42s %8.1f x  %8s\n",
              "20 agents, L2 + 2D LiDAR, headless", twenty, "> 10 x");

  std::printf("\n");
  std::printf("  Targets are the P1 goals. These numbers are from whatever\n");
  std::printf("  machine ran them, which for a CI job is a shared virtual\n");
  std::printf("  one; the published figures name their hardware.\n");
  return 0;
}
