// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The leaderboard harness, held to the milestone's own bar: a leaderboard
// run is reproducible from its manifest and seeds. The assertions are on
// bytes and files, not on in-memory objects, because the harness's design
// rule is that the files are the truth: the same batch writes the same
// stream bytes, the standings recompute from the streams alone, and the
// seeds demonstrably reach the racing rather than only the metadata.

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "slipx/race/event_stream.hpp"
#include "slipx/race/leaderboard.hpp"
#include "slipx/race/starts.hpp"
#include "slipx/scene/projection.hpp"
#include "slipx/sense/rng.hpp"
#include "race_test_support.hpp"

namespace {

using slipx::DriveInput;
using slipx::VehicleState;
using slipx::race::BatchConfig;
using slipx::race::BatchResult;
using slipx::race::Entrant;
using slipx::race::EventStreamContents;
using slipx::race::EventType;
using slipx::race::LeaderboardRow;
using slipx::race::RaceEvent;
using slipx::scene::Track;

std::string batch_dir(const char* name) {
  const std::string path = std::string(::testing::TempDir()) + name;
  std::filesystem::create_directories(path);
  return path;
}

std::string file_bytes(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
}

// A centreline follower with a seeded steering jitter, mirroring the tool's
// canned entrants: the jitter is what lets a scenario's seed reach the
// outcome, which several tests below rely on.
Entrant jittered(const std::string& name, double speed, double lateral) {
  Entrant out;
  out.name = name;
  out.make_agent = [speed, lateral](const Track& track) {
    slipx::sim::AgentSpec spec;
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

// An entrant that never moves: an empty policy coasts, and a car at rest
// stays at rest, so no round it enters can ever be decided.
Entrant coasting(const std::string& name) {
  Entrant out;
  out.name = name;
  out.make_agent = [](const Track&) {
    slipx::sim::AgentSpec spec;
    spec.tier = slipx::Tier::L1_Bicycle;
    spec.footprint_length = 0.50;
    spec.footprint_width = 0.30;
    return spec;
  };
  return out;
}

void expect_rows_equal(const std::vector<LeaderboardRow>& a,
                       const std::vector<LeaderboardRow>& b) {
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].name, b[i].name) << "row " << i;
    EXPECT_EQ(a[i].matches, b[i].matches) << "row " << i;
    EXPECT_EQ(a[i].match_wins, b[i].match_wins) << "row " << i;
    EXPECT_EQ(a[i].round_wins, b[i].round_wins) << "row " << i;
    EXPECT_EQ(a[i].abandoned, b[i].abandoned) << "row " << i;
  }
}

BatchConfig quick_batch(std::uint64_t master_seed, int repetitions) {
  BatchConfig batch;
  batch.master_seed = master_seed;
  batch.repetitions = repetitions;
  batch.race.laps_to_win = 1;   // the default 20 makes a slow test suite
  batch.round_step_budget = 30000;
  return batch;
}

// A synthetic parsed stream: standings() takes parsed streams by design, so
// the counting and ordering rules can be pinned without racing anybody.
EventStreamContents synthetic_stream(const std::string& entrant0,
                                     const std::string& entrant1,
                                     const std::vector<RaceEvent>& events) {
  EventStreamContents contents;
  contents.metadata = {{"entrant.0", entrant0}, {"entrant.1", entrant1}};
  contents.events = events;
  return contents;
}

RaceEvent event_of(EventType type, std::uint32_t agent) {
  RaceEvent event;
  event.type = type;
  event.agent = agent;
  return event;
}

TEST(Leaderboard, TheSameBatchTwiceWritesTheSameBytesAndTheSameRows) {
  const Track track = race_test::shipped_track();
  const std::vector<Entrant> entrants = {jittered("steady", 4.2, 0.25),
                                         jittered("brisk", 4.8, -0.25)};
  const BatchConfig batch = quick_batch(11, 2);

  const BatchResult first = slipx::race::run_round_robin(
      track, entrants, batch, batch_dir("lb_same_a"));
  const BatchResult second = slipx::race::run_round_robin(
      track, entrants, batch, batch_dir("lb_same_b"));

  ASSERT_EQ(first.stream_paths.size(), 2u) << "one pair, two repetitions";
  ASSERT_EQ(second.stream_paths.size(), 2u);
  for (std::size_t k = 0; k < first.stream_paths.size(); ++k) {
    const std::string bytes = file_bytes(first.stream_paths[k]);
    ASSERT_FALSE(bytes.empty());
    EXPECT_EQ(bytes, file_bytes(second.stream_paths[k])) << "stream " << k;
  }
  const std::string leaderboard = file_bytes(first.leaderboard_path);
  ASSERT_FALSE(leaderboard.empty());
  EXPECT_EQ(leaderboard, file_bytes(second.leaderboard_path));
  expect_rows_equal(first.rows, second.rows);
}

TEST(Leaderboard, StandingsRecomputedFromTheStreamPathsEqualTheResult) {
  const Track track = race_test::shipped_track();
  const std::vector<Entrant> entrants = {jittered("steady", 4.2, 0.25),
                                         jittered("brisk", 4.8, -0.25),
                                         jittered("keen", 5.2, 0.0)};

  const BatchResult result = slipx::race::run_round_robin(
      track, entrants, quick_batch(21, 1), batch_dir("lb_recompute"));

  // The third party the harness promises to agree with: a consumer holding
  // nothing but the files it names.
  ASSERT_EQ(result.stream_paths.size(), 3u);
  std::vector<EventStreamContents> streams(result.stream_paths.size());
  for (std::size_t k = 0; k < result.stream_paths.size(); ++k) {
    ASSERT_TRUE(
        slipx::race::read_event_stream(result.stream_paths[k], &streams[k]));
  }
  expect_rows_equal(slipx::race::standings(streams), result.rows);

  ASSERT_EQ(result.rows.size(), 3u);
  for (const LeaderboardRow& row : result.rows) {
    EXPECT_EQ(row.matches, 2) << row.name << " meets each other entrant once";
  }
}

TEST(Leaderboard, ADifferentMasterSeedChangesTheRacing) {
  const Track track = race_test::shipped_track();
  const std::vector<Entrant> entrants = {jittered("steady", 4.2, 0.25),
                                         jittered("brisk", 4.8, -0.25)};

  const BatchResult first = slipx::race::run_round_robin(
      track, entrants, quick_batch(3, 1), batch_dir("lb_seed_a"));
  const BatchResult second = slipx::race::run_round_robin(
      track, entrants, quick_batch(4, 1), batch_dir("lb_seed_b"));

  ASSERT_EQ(first.stream_paths.size(), 1u);
  ASSERT_EQ(second.stream_paths.size(), 1u);
  EXPECT_NE(file_bytes(first.stream_paths[0]),
            file_bytes(second.stream_paths[0]));

  // Bytes alone would differ through the recorded seed metadata even if the
  // seed never reached the simulation, so the claim is held against the
  // events themselves: the jittered steering must change the racing.
  EventStreamContents a, b;
  ASSERT_TRUE(slipx::race::read_event_stream(first.stream_paths[0], &a));
  ASSERT_TRUE(slipx::race::read_event_stream(second.stream_paths[0], &b));
  std::vector<std::tuple<EventType, std::uint64_t, double>> ea, eb;
  for (const RaceEvent& e : a.events) ea.emplace_back(e.type, e.step, e.value);
  for (const RaceEvent& e : b.events) eb.emplace_back(e.type, e.step, e.value);
  EXPECT_NE(ea, eb);
}

TEST(Leaderboard, AnUndecidedScenarioIsAbandonedAndNobodyWins) {
  const Track track = race_test::shipped_track();
  const std::vector<Entrant> entrants = {coasting("idle"),
                                         coasting("inert")};
  BatchConfig batch = quick_batch(5, 1);
  batch.round_step_budget = 500;   // nobody at rest decides anything

  const BatchResult result = slipx::race::run_round_robin(
      track, entrants, batch, batch_dir("lb_abandoned"));

  ASSERT_EQ(result.rows.size(), 2u);
  for (const LeaderboardRow& row : result.rows) {
    EXPECT_EQ(row.matches, 1) << row.name;
    EXPECT_EQ(row.match_wins, 0) << row.name;
    EXPECT_EQ(row.round_wins, 0) << row.name;
    EXPECT_EQ(row.abandoned, 1) << row.name;
  }

  // The stream still exists (an abandoned match is evidence too), and it
  // carries no winner for anyone to claim later.
  EventStreamContents contents;
  ASSERT_TRUE(
      slipx::race::read_event_stream(result.stream_paths[0], &contents));
  for (const RaceEvent& event : contents.events) {
    EXPECT_NE(event.type, EventType::kMatchWon);
  }
  EXPECT_NE(file_bytes(result.leaderboard_path).find("\"abandoned\": 1"),
            std::string::npos);
}

TEST(Leaderboard, StandingsMapWinsThroughTheStreamsOwnNames) {
  // The winner is agent 1 on purpose: an implementation reading the agent
  // index without mapping it through the stream's entrant names would hand
  // this match to the loser.
  const std::vector<EventStreamContents> streams = {
      synthetic_stream("paz", "quo",
                       {event_of(EventType::kRoundWon, 1),
                        event_of(EventType::kRoundWon, 1),
                        event_of(EventType::kMatchWon, 1)})};

  const std::vector<LeaderboardRow> rows = slipx::race::standings(streams);
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[0].name, "quo");
  EXPECT_EQ(rows[0].match_wins, 1);
  EXPECT_EQ(rows[0].round_wins, 2);
  EXPECT_EQ(rows[0].abandoned, 0);
  EXPECT_EQ(rows[1].name, "paz");
  EXPECT_EQ(rows[1].match_wins, 0);
  EXPECT_EQ(rows[1].round_wins, 0);
  EXPECT_EQ(rows[1].abandoned, 0) << "a decided match is not abandoned";
}

TEST(Leaderboard, StandingsOrderIsByMatchWinsThenRoundWinsThenName) {
  // Five entrants engineered to need every rung of the ordering: bryn leads
  // on match wins, cerys beats afon only on round wins, and dai and emrys
  // are separated by nothing but their names. The round-wins rung runs
  // AGAINST the alphabet on purpose (cerys must sort above afon), so a
  // comparator that quietly falls through to the name cannot pass; the
  // first mutation pass caught this test agreeing with the alphabet.
  const std::vector<EventStreamContents> streams = {
      synthetic_stream("bryn", "afon",
                       {event_of(EventType::kRoundWon, 0),
                        event_of(EventType::kRoundWon, 0),
                        event_of(EventType::kMatchWon, 0)}),
      synthetic_stream("bryn", "cerys",
                       {event_of(EventType::kRoundWon, 0),
                        event_of(EventType::kRoundWon, 0),
                        event_of(EventType::kMatchWon, 0)}),
      synthetic_stream("cerys", "dai",
                       {event_of(EventType::kRoundWon, 0),
                        event_of(EventType::kRoundWon, 0),
                        event_of(EventType::kMatchWon, 0)}),
      synthetic_stream("afon", "emrys",
                       {event_of(EventType::kRoundWon, 0),
                        event_of(EventType::kMatchWon, 0)}),
      synthetic_stream("dai", "emrys", {}),
      // A round was won but the match was not: the budget ran out in round
      // two. Only a match win decides a stream; this one is abandoned for
      // both cars, and bryn keeps the round.
      synthetic_stream("bryn", "afon",
                       {event_of(EventType::kRoundWon, 0)}),
  };

  const std::vector<LeaderboardRow> rows = slipx::race::standings(streams);
  ASSERT_EQ(rows.size(), 5u);
  EXPECT_EQ(rows[0].name, "bryn");
  EXPECT_EQ(rows[1].name, "cerys") << "more round wins beat the alphabet";
  EXPECT_EQ(rows[2].name, "afon");
  EXPECT_EQ(rows[3].name, "dai");
  EXPECT_EQ(rows[4].name, "emrys");

  EXPECT_EQ(rows[0].match_wins, 2);
  EXPECT_EQ(rows[0].round_wins, 5);
  EXPECT_EQ(rows[0].abandoned, 1);
  EXPECT_EQ(rows[1].match_wins, 1);
  EXPECT_EQ(rows[1].round_wins, 2);
  EXPECT_EQ(rows[2].match_wins, 1);
  EXPECT_EQ(rows[2].round_wins, 1);
  EXPECT_EQ(rows[2].abandoned, 1);
  EXPECT_EQ(rows[3].match_wins, 0);
  EXPECT_EQ(rows[3].abandoned, 1);
  EXPECT_EQ(rows[4].match_wins, 0);
  EXPECT_EQ(rows[4].abandoned, 1);
}

TEST(Leaderboard, TheManifestAndTheStreamsCarryTheDerivedSeeds) {
  const Track track = race_test::shipped_track();
  const std::vector<Entrant> entrants = {jittered("steady", 4.2, 0.25),
                                         jittered("brisk", 4.8, -0.25)};
  const std::uint64_t master = 99;

  const BatchResult result = slipx::race::run_round_robin(
      track, entrants, quick_batch(master, 3), batch_dir("lb_manifest"));

  // The seeds, computed independently of the code under test: reproducing a
  // batch from its manifest needs each scenario's seed to be the derivation
  // the header promises, not the master seed reused.
  ASSERT_EQ(result.stream_paths.size(), 3u);
  const std::string manifest = file_bytes(result.manifest_path);
  ASSERT_FALSE(manifest.empty());
  EXPECT_NE(manifest.find("\"master_seed\": 99"), std::string::npos);
  for (std::size_t k = 0; k < 3; ++k) {
    const std::uint64_t seed = slipx::sense::derive_seed(master, k);
    // One fragment per scenario line, tying the seed to the slot
    // alternation: repetitions alternate who takes the left slot.
    const std::string fragment =
        "\"seed\": " + std::to_string(seed) +
        ", \"a_on_left\": " + (k % 2 == 0 ? "true" : "false");
    EXPECT_NE(manifest.find(fragment), std::string::npos) << "scenario " << k;

    EventStreamContents contents;
    ASSERT_TRUE(
        slipx::race::read_event_stream(result.stream_paths[k], &contents));
    EXPECT_EQ(contents.metadata_value("scenario_seed"),
              std::to_string(seed));
    EXPECT_EQ(contents.metadata_value("master_seed"), "99");

    // And the alternation reaches the racing, not only the manifest: the
    // round-start event names the left car.
    bool saw_start = false;
    for (const RaceEvent& event : contents.events) {
      if (event.type == EventType::kRoundStart) {
        EXPECT_EQ(event.agent, k % 2 == 0 ? 0u : 1u) << "scenario " << k;
        saw_start = true;
        break;
      }
    }
    EXPECT_TRUE(saw_start) << "scenario " << k;
  }
}

TEST(Leaderboard, AnUnwritableDirectoryThrows) {
  const Track track = race_test::shipped_track();
  const std::vector<Entrant> entrants = {coasting("idle"),
                                         coasting("inert")};
  BatchConfig batch = quick_batch(7, 1);
  batch.round_step_budget = 100;

  const std::string missing =
      std::string(::testing::TempDir()) + "lb_missing/nested";
  std::filesystem::remove_all(std::string(::testing::TempDir()) +
                              "lb_missing");
  // The refusal must come from the write, not be laundered through a later
  // failure: a leaderboard that silently lost its evidence is the failure
  // mode the throw exists for.
  try {
    slipx::race::run_round_robin(track, entrants, batch, missing);
    FAIL() << "an unwritable directory must throw";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("could not write"),
              std::string::npos)
        << error.what();
  }
}

}  // namespace
