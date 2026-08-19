// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The CI leaderboard harness:
//
//     slipx_leaderboard <output_dir> [master_seed]
//
// Runs a seeded round-robin between three canned entrants on the shipped
// track, writes one event stream per match, a batch manifest and
// leaderboard.json into <output_dir>, and prints the standings. The
// entrants are demonstration controllers whose steering carries a small
// seeded jitter, so the seed genuinely reaches the racing; a real
// evaluation swaps them for the stacks under test and keeps everything
// else. Reproducibility is the point: the same seed writes the same bytes,
// which the test suite asserts rather than promises.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "slipx/race/leaderboard.hpp"
#include "slipx/race/starts.hpp"
#include "slipx/scene/projection.hpp"

namespace {

using slipx::DriveInput;
using slipx::VehicleState;

slipx::scene::Track load_track(const std::string& directory) {
  slipx::scene::Centreline geometry =
      slipx::scene::Centreline::from_file(directory + "/centreline.csv");
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

// A centreline follower with a per-agent seeded steering jitter: the jitter
// is what lets a scenario's seed reach the outcome, exactly as a stack with
// stochastic planning would.
slipx::race::Entrant entrant(const std::string& name, double speed,
                             double lateral) {
  slipx::race::Entrant out;
  out.name = name;
  out.make_agent = [speed, lateral](const slipx::scene::Track& track) {
    slipx::sim::AgentSpec spec;
    spec.name = "entrant";
    spec.tier = slipx::Tier::L1_Bicycle;
    spec.footprint_length = 0.50;
    spec.footprint_width = 0.30;
    spec.policy = [&track, speed, lateral](const VehicleState& s, double,
                                           slipx::sense::Rng& rng) {
      const auto where = slipx::scene::project(track, s.pos.x, s.pos.y);
      const auto target = slipx::race::pose_at(track, where.s + 0.8);
      const double nx = -std::sin(target.heading);
      const double ny = std::cos(target.heading);
      const double tx = target.x + lateral * nx;
      const double ty = target.y + lateral * ny;
      const double desired = std::atan2(ty - s.pos.y, tx - s.pos.x);
      const double error =
          std::atan2(std::sin(desired - s.yaw), std::cos(desired - s.yaw));
      DriveInput input;
      const double steer = 1.5 * error + rng.normal(0.0, 0.01);
      input.steer_cmd = steer > 0.4 ? 0.4 : (steer < -0.4 ? -0.4 : steer);
      input.accel_cmd = 4.0 * (speed - s.vel_body.x);
      return input;
    };
    return spec;
  };
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                 "usage: slipx_leaderboard <output_dir> [master_seed]\n");
    return 2;
  }
  const std::string directory = argv[1];
  std::filesystem::create_directories(directory);

  slipx::race::BatchConfig batch;
  batch.master_seed =
      argc > 2 ? std::strtoull(argv[2], nullptr, 10) : 20260819u;
  batch.repetitions = 2;
  batch.race.laps_to_win = 2;
  batch.round_step_budget = 120000;

  const slipx::scene::Track track =
      load_track(std::string(SLIPX_EXAMPLE_TRACK_DIR));

  const std::vector<slipx::race::Entrant> entrants = {
      entrant("steady", 4.2, 0.25),
      entrant("brisk", 4.8, -0.25),
      entrant("reckless", 5.2, 0.0),
  };

  const slipx::race::BatchResult result =
      slipx::race::run_round_robin(track, entrants, batch, directory);

  std::printf("%s\n", slipx::race::ruleset_statement().c_str());
  std::printf("batch: master seed %llu, %zu matches -> %s\n",
              static_cast<unsigned long long>(batch.master_seed),
              result.stream_paths.size(), directory.c_str());
  std::printf("  %-12s %8s %8s %8s %10s\n", "entrant", "matches", "wins",
              "rounds", "abandoned");
  for (const auto& row : result.rows) {
    std::printf("  %-12s %8d %8d %8d %10d\n", row.name.c_str(), row.matches,
                row.match_wins, row.round_wins, row.abandoned);
  }
  return 0;
}
