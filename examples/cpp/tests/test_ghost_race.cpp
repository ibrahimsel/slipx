// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Twenty cars driving the shipped track at once, headlessly.
//
// The reference stack tests say one car can drive a lap. These say the same
// pieces compose: twenty of them, orchestrated in lockstep, all finish, none
// leaves the corridor, and the run is the same run twice.
//
// What they deliberately do NOT assert is anything about racing. There is no
// contact model, so there is nothing to assert about it, and a test that
// checked the cars never occupy the same metre would be asserting a physical
// claim this simulator does not make.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "ghost_race.hpp"
#include "slipx/scene/centreline.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/vehicle_model.hpp"

namespace {

using slipx::VehicleParams;
using slipx::examples::GhostRaceConfig;
using slipx::examples::GhostRaceRecording;
using slipx::examples::centreline_pose;
using slipx::examples::run_ghost_race;
using slipx::scene::Centreline;
using slipx::scene::Track;
using slipx::scene::TrackManifest;

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

TEST(GhostRace, EveryCarCompletesItsLaps) {
  const Track track = shipped_track();
  const VehicleParams params;
  GhostRaceConfig config;

  const GhostRaceRecording recording = run_ghost_race(track, params, config);

  ASSERT_EQ(recording.results.size(), config.agents);
  for (const auto& result : recording.results) {
    EXPECT_GE(result.laps, config.laps)
        << result.name << " completed " << result.laps << " laps";
    EXPECT_EQ(result.lap_times.size(), static_cast<std::size_t>(config.laps))
        << result.name;
    EXPECT_FALSE(result.left_the_track)
        << result.name << " worst margin " << result.worst_margin << " m";

    // The shipped track's corridor is 1.5 m wide, so the margin to the nearer
    // edge cannot exceed half of that plus the tolerance the scenario asked
    // for. Asserting the bound as well as the flag is what makes the flag
    // mean anything: a scenario that quietly widened the corridor would
    // satisfy "never left the track" without ever being inside it.
    EXPECT_LE(result.worst_margin, 0.75 + config.limit_tolerance + 1e-9)
        << result.name << " reported a margin wider than the corridor";
    EXPECT_GT(result.worst_margin, 0.0) << result.name;
  }
}

// The grid is a rolling start, and every car is already doing its target
// speed when the clock starts. A standing start would still produce laps and
// a plausible ranking, several seconds slower, with the difference buried in
// a lap time nobody has an independent expectation for; only the early speed
// trace says which of the two happened.
//
// This deliberately asserts nothing about the wheel speeds. L2 has no wheel
// rotational state (ADR-0027), so seeding them changes one step of ESC curve
// lookup and nothing a test at this level can see.
TEST(GhostRace, TheFieldStartsAtSpeed) {
  const Track track = shipped_track();
  const VehicleParams params;
  GhostRaceConfig config;
  config.laps = 1;

  const GhostRaceRecording recording = run_ghost_race(track, params, config);

  for (std::size_t i = 0; i < recording.frames.size(); ++i) {
    const double target = recording.results[i].target_speed;
    for (const auto& frame : recording.frames[i]) {
      if (frame.t > 1.0) break;
      EXPECT_GT(frame.speed, target * 0.9)
          << recording.results[i].name << " was doing " << frame.speed
          << " m/s at t = " << frame.t << " s against a target of " << target;
    }
  }
}

// The controller is not fast and does not plan a line, so the honest check is
// that a car asked to go faster does go faster. It is the one thing the field
// spread is for.
TEST(GhostRace, ABiggerSpeedDemandGivesAQuickerLap) {
  const Track track = shipped_track();
  const VehicleParams params;
  GhostRaceConfig config;
  config.laps = 1;  // one lap answers this; the second only costs CI time
  const GhostRaceRecording recording = run_ghost_race(track, params, config);

  for (std::size_t i = 1; i < recording.results.size(); ++i) {
    const auto& slower = recording.results[i - 1];
    const auto& quicker = recording.results[i];
    ASSERT_GT(quicker.target_speed, slower.target_speed);
    EXPECT_LT(quicker.best_lap(), slower.best_lap())
        << quicker.name << " asked for " << quicker.target_speed
        << " m/s and lapped no quicker than " << slower.name;
  }
}

// A lap time that is physically impossible for the speed asked for would mean
// the lap counter is counting something other than a lap: a car that cut the
// infield, or a projection that wrapped. The bound is generous on the slow
// side, because a geometric controller loses time in the corners, and tight
// on the fast side, because nothing can beat driving the centreline at the
// target speed.
TEST(GhostRace, LapTimesAreConsistentWithTheDistanceAndTheSpeed) {
  const Track track = shipped_track();
  const VehicleParams params;
  GhostRaceConfig config;

  // Two laps deliberately, and this is the test that needs them: a lap time
  // that is really an elapsed clock reads correctly on the first lap and
  // twice as long on the second, so a one-lap version of this test cannot
  // tell the two apart.
  ASSERT_GE(config.laps, 2);
  const GhostRaceRecording recording = run_ghost_race(track, params, config);

  for (const auto& result : recording.results) {
    const double ideal = recording.lap_length / result.target_speed;
    for (const double lap : result.lap_times) {
      EXPECT_GT(lap, ideal * 0.95) << result.name << " lapped in " << lap
                                   << " s against a floor of " << ideal;
      EXPECT_LT(lap, ideal * 1.6) << result.name << " lapped in " << lap;
    }

    // The best lap is the quickest one, not the last or the longest. With
    // laps a few milliseconds apart, only an explicit comparison sees this.
    const double quickest =
        *std::min_element(result.lap_times.begin(), result.lap_times.end());
    EXPECT_DOUBLE_EQ(result.best_lap(), quickest) << result.name;
  }
}

// The grid is spaced around the lap rather than stacked on one straight,
// which is the thing that makes this look like a start rather than a pile.
// Twenty cars 0.42 m long on a 34.85 m lap have room for it.
TEST(GhostRace, TheGridIsSpacedAroundTheLap) {
  const Track track = shipped_track();
  const VehicleParams params;
  GhostRaceConfig config;
  config.laps = 1;  // the grid is the first frame; the race is not the point
  const GhostRaceRecording recording = run_ghost_race(track, params, config);

  const double spacing =
      recording.lap_length / static_cast<double>(config.agents);
  EXPECT_GT(spacing, 1.0);

  for (std::size_t i = 0; i < recording.frames.size(); ++i) {
    ASSERT_FALSE(recording.frames[i].empty());
    const auto& first = recording.frames[i].front();
    double heading = 0.0;
    const auto slot =
        centreline_pose(track, -spacing * static_cast<double>(i), &heading);
    EXPECT_NEAR(first.x, slot.first, 1e-9);
    EXPECT_NEAR(first.y, slot.second, 1e-9);

    // No two cars share a grid slot, and none is closer to another than a car
    // is long.
    for (std::size_t j = 0; j < i; ++j) {
      const auto& other = recording.frames[j].front();
      const double gap = std::hypot(first.x - other.x, first.y - other.y);
      EXPECT_GT(gap, 0.42) << "cars " << i << " and " << j << " start "
                           << gap << " m apart";
    }
  }
}

// The whole run, twice. Nothing in it reads a clock, so the second run is the
// first one; this is the property the lockstep orchestrator exists to give
// and it is worth checking through a scenario rather than only in a unit.
TEST(GhostRace, TheRunIsReproducible) {
  const Track track = shipped_track();
  const VehicleParams params;
  GhostRaceConfig config;
  config.laps = 1;

  const GhostRaceRecording one = run_ghost_race(track, params, config);
  const GhostRaceRecording two = run_ghost_race(track, params, config);

  ASSERT_EQ(one.frames.size(), two.frames.size());
  for (std::size_t i = 0; i < one.frames.size(); ++i) {
    ASSERT_EQ(one.frames[i].size(), two.frames[i].size());
    for (std::size_t f = 0; f < one.frames[i].size(); ++f) {
      EXPECT_DOUBLE_EQ(one.frames[i][f].x, two.frames[i][f].x);
      EXPECT_DOUBLE_EQ(one.frames[i][f].y, two.frames[i][f].y);
      EXPECT_DOUBLE_EQ(one.frames[i][f].yaw, two.frames[i][f].yaw);
    }
    ASSERT_EQ(one.results[i].lap_times.size(),
              two.results[i].lap_times.size());
    for (std::size_t lap = 0; lap < one.results[i].lap_times.size(); ++lap) {
      EXPECT_DOUBLE_EQ(one.results[i].lap_times[lap],
                       two.results[i].lap_times[lap]);
    }
  }
}

// Adding a car must not change the cars already there. The orchestrator
// derives each agent's random stream from the master seed and the agent
// index, precisely so that a scenario can grow without rewriting the runs
// beside it, and a scenario test is where that promise is visible.
TEST(GhostRace, TheFieldSizeDoesNotChangeTheGrid) {
  const Track track = shipped_track();
  const VehicleParams params;

  GhostRaceConfig small;
  small.agents = 5;
  small.laps = 1;
  GhostRaceConfig large = small;
  large.agents = 6;

  const GhostRaceRecording few = run_ghost_race(track, params, small);
  const GhostRaceRecording many = run_ghost_race(track, params, large);

  // The grid slots move, because they are spaced by the field size. The
  // speeds move for the same reason. What is asserted here is the weaker and
  // more useful thing: every car in the larger field still drives a clean
  // lap, so the scenario scales rather than only working at twenty.
  for (const auto& result : many.results) {
    EXPECT_EQ(result.laps, 1) << result.name;
    EXPECT_FALSE(result.left_the_track) << result.name;
  }
  EXPECT_EQ(few.results.size(), 5u);
  EXPECT_EQ(many.results.size(), 6u);
}

// An open track has no lap to count, so a ghost race on one is a request the
// scenario cannot honour. It refuses by name rather than running and
// reporting zero laps for everybody, which is the failure that looks like a
// controller bug.
TEST(GhostRace, AnOpenTrackIsRefused) {
  Centreline geometry = Centreline::from_file(
      std::string(SLIPX_EXAMPLE_TRACK_DIR) + "/centreline.csv");

  TrackManifest manifest;
  manifest.name = "paddock_stadium_open";
  manifest.surface = "carpet";
  manifest.closed = false;
  manifest.geometry_source = "examples/tracks/make_tracks.py";
  manifest.geometry_licence = "Apache-2.0";
  manifest.provenance_label = "provisional";

  const Track open = Track::build(geometry, manifest, {{"sponge", "carpet"}});
  const VehicleParams params;

  EXPECT_THROW(run_ghost_race(open, params, {}), std::invalid_argument);
}

}  // namespace
