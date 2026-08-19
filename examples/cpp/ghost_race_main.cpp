// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Run the ghost race and write it down.
//
//   ./build/examples/cpp/slipx_ghost_race [output_dir] [track_dir]
//
// Prints a classification and writes three CSVs: the recorded states, the lap
// times, and the walls the track implies. It writes files and never opens a
// window (ADR-0024), and it draws nothing: rendering is
// examples/ghost_race_figure.py, which reads these and nothing else.
//
// The classification is a time trial ranking, because that is the only kind
// of result this simulator can honestly produce today. Nothing here decided
// an overtake, because nothing here can: see the header for what a ghost race
// is and is not.

#include <cstdio>
#include <string>
#include <vector>

#include "ghost_race.hpp"
#include "slipx/scene/centreline.hpp"
#include "slipx/scene/raycast.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/vehicle_model.hpp"

namespace {

using slipx::VehicleParams;
using slipx::examples::GhostRaceConfig;
using slipx::examples::GhostRaceRecording;
using slipx::examples::GhostResult;
using slipx::examples::run_ghost_race;
using slipx::scene::Centreline;
using slipx::scene::Track;
using slipx::scene::TrackManifest;
using slipx::scene::Walls;

Track load_track(const std::string& directory) {
  Centreline geometry = Centreline::from_file(directory + "/centreline.csv");

  // The manifest the shipped track carries. Stated here rather than parsed,
  // because parsing YAML lives above this layer in slipx_schema and an
  // example must not drag the Python half in to draw a picture.
  TrackManifest manifest;
  manifest.name = "paddock_stadium";
  manifest.surface = "carpet";
  manifest.closed = true;
  manifest.geometry_source = "examples/tracks/make_tracks.py";
  manifest.geometry_licence = "Apache-2.0";
  manifest.provenance_label = "provisional";

  return Track::build(geometry, manifest, {{"sponge", "carpet"}});
}

bool write_walls(const Walls& walls, const std::string& path) {
  std::FILE* file = std::fopen(path.c_str(), "w");
  if (file == nullptr) return false;
  std::fprintf(file, "wall,i,x,y\n");
  for (std::size_t i = 0; i < walls.left_x().size(); ++i) {
    std::fprintf(file, "left,%zu,%.6f,%.6f\n", i, walls.left_x()[i],
                 walls.left_y()[i]);
  }
  for (std::size_t i = 0; i < walls.right_x().size(); ++i) {
    std::fprintf(file, "right,%zu,%.6f,%.6f\n", i, walls.right_x()[i],
                 walls.right_y()[i]);
  }
  std::fclose(file);
  return true;
}

bool write_states(const GhostRaceRecording& recording,
                  const std::string& path) {
  std::FILE* file = std::fopen(path.c_str(), "w");
  if (file == nullptr) return false;
  std::fprintf(file, "t,agent,x,y,yaw,speed\n");
  for (std::size_t i = 0; i < recording.frames.size(); ++i) {
    for (const auto& frame : recording.frames[i]) {
      std::fprintf(file, "%.4f,%zu,%.6f,%.6f,%.6f,%.6f\n", frame.t, i, frame.x,
                   frame.y, frame.yaw, frame.speed);
    }
  }
  std::fclose(file);
  return true;
}

bool write_laps(const GhostRaceRecording& recording, const std::string& path) {
  std::FILE* file = std::fopen(path.c_str(), "w");
  if (file == nullptr) return false;
  std::fprintf(file, "agent,name,target_speed,lap,lap_time\n");
  for (std::size_t i = 0; i < recording.results.size(); ++i) {
    const GhostResult& result = recording.results[i];
    for (std::size_t lap = 0; lap < result.lap_times.size(); ++lap) {
      std::fprintf(file, "%zu,%s,%.4f,%zu,%.4f\n", i, result.name.c_str(),
                   result.target_speed, lap + 1, result.lap_times[lap]);
    }
  }
  std::fclose(file);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string out_dir = argc > 1 ? argv[1] : ".";
  const std::string track_dir = argc > 2 ? argv[2] : SLIPX_EXAMPLE_TRACK_DIR;

  const Track track = load_track(track_dir);
  const Walls walls(track);
  const VehicleParams params;

  GhostRaceConfig config;
  const GhostRaceRecording recording = run_ghost_race(track, params, config);

  std::printf("Ghost race: %zu cars, %d laps of %s (%.2f m)\n", config.agents,
              config.laps, track.name().c_str(), recording.lap_length);
  std::printf("  provenance %s, and no parameter set here has been measured "
              "against a car\n",
              track.manifest().provenance_label.c_str());
  std::printf("  no collision footprints, no race control: these are %zu "
              "time trials sharing a track\n\n", config.agents);

  std::printf("  %-6s %8s %9s %9s %9s %8s\n", "car", "target", "lap 1",
              "lap 2", "best", "margin");
  for (const GhostResult& result : recording.results) {
    std::printf("  %-6s %6.2f m/s", result.name.c_str(), result.target_speed);
    for (std::size_t lap = 0; lap < 2; ++lap) {
      if (lap < result.lap_times.size()) {
        std::printf(" %8.3f s", result.lap_times[lap]);
      } else {
        std::printf(" %10s", "-");
      }
    }
    std::printf(" %7.3f s %6.3f m%s\n", result.best_lap(),
                result.worst_margin,
                result.left_the_track ? "  LEFT THE TRACK" : "");
  }

  std::printf("\n  %.1f s of simulation\n", recording.duration);

  const std::string states = out_dir + "/ghost_race_states.csv";
  const std::string laps = out_dir + "/ghost_race_laps.csv";
  const std::string wall_file = out_dir + "/ghost_race_walls.csv";
  if (!write_states(recording, states) || !write_laps(recording, laps) ||
      !write_walls(walls, wall_file)) {
    std::fprintf(stderr, "could not write into %s\n", out_dir.c_str());
    return 1;
  }

  std::printf("  wrote %s, %s, %s\n", states.c_str(), laps.c_str(),
              wall_file.c_str());
  return 0;
}
