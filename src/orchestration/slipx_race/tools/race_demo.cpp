// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// A canned, fully deterministic head-to-head match on the shipped track,
// written out as an event stream:
//
//     slipx_race_demo <output.mcap> [track_directory]
//
// It exists so that something outside this repository can hold the "one
// format" claim: the pytest suite runs this binary and reads its output
// with the reference `mcap` library, which is the same library that reads
// the run sinks. It is also the smallest working example of composing the
// sim, the track and race control.

#include <cmath>
#include <cstdio>
#include <string>

#include "slipx/race/event_stream.hpp"
#include "slipx/race/head_to_head.hpp"
#include "slipx/race/starts.hpp"
#include "slipx/scene/projection.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/sim/simulation.hpp"

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

slipx::sim::Policy follow(const slipx::scene::Track& track, double speed,
                          double lateral) {
  return [&track, speed, lateral](const VehicleState& s, double,
                                  slipx::sense::Rng&) {
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
    input.steer_cmd = error > 0.4 ? 0.4 : (error < -0.4 ? -0.4 : 1.5 * error);
    input.accel_cmd = 4.0 * (speed - s.vel_body.x);
    return input;
  };
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: slipx_race_demo <output.mcap> [track_dir]\n");
    return 2;
  }
  const std::string output = argv[1];
  const std::string track_dir =
      argc > 2 ? argv[2] : std::string(SLIPX_EXAMPLE_TRACK_DIR);

  const slipx::scene::Track track = load_track(track_dir);

  slipx::sim::SimulationConfig sim_config;
  sim_config.master_seed = 20260819;
  slipx::sim::Simulation sim(sim_config);

  const auto car = [&](double speed, double lateral) {
    slipx::sim::AgentSpec spec;
    spec.tier = slipx::Tier::L1_Bicycle;
    spec.policy = follow(track, speed, lateral);
    spec.footprint_length = 0.50;
    spec.footprint_width = 0.30;
    return spec;
  };
  sim.add_agent(car(5.0, 0.25));
  sim.add_agent(car(3.5, -0.25));

  slipx::race::RaceConfig config;
  config.laps_to_win = 2;   // a demonstration, not a twenty-lap final

  slipx::race::Match match(sim, track, 0, 1, 0.0, true,
                           sim_config.master_seed, config);
  match.run(120000);
  if (!match.finished()) {
    std::fprintf(stderr, "the demo match did not finish, which is a bug\n");
    return 1;
  }

  const bool written = slipx::race::write_event_stream(
      output, match.events(), config,
      {{"scenario", "race_demo"},
       {"master_seed", std::to_string(sim_config.master_seed)},
       {"configuration_digest", sim.manifest().configuration_digest()}});
  if (!written) {
    std::fprintf(stderr, "could not write %s\n", output.c_str());
    return 1;
  }

  std::printf("%s\n", slipx::race::ruleset_statement().c_str());
  std::printf("match: car %zu wins %d-%d over %d round(s); %zu events -> %s\n",
              match.winner(), match.round_wins(match.winner()),
              match.round_wins(1 - match.winner()), match.rounds_played(),
              match.events().size(), output.c_str());
  return 0;
}
