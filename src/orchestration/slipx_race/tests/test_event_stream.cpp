// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The event stream, held to the milestone's own bar: a full race replays
// from its stream alone. "Replays" means a consumer holding nothing but the
// file reconstructs the race, outcome for outcome, and agrees with the live
// object it never saw. Wire-level validity against the reference MCAP
// library is the pytest suite's job, which reads a file this code wrote.

#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <fstream>
#include <string>

#include "slipx/race/event_stream.hpp"
#include "slipx/race/head_to_head.hpp"
#include "race_test_support.hpp"

namespace {

using slipx::race::EventStreamContents;
using slipx::race::EventType;
using slipx::race::Match;
using slipx::race::RaceConfig;
using slipx::race::RaceEvent;

std::string temp_path(const char* name) {
  return std::string(::testing::TempDir()) + name;
}

// A crash-rich match, so the stream carries every event kind worth arguing
// about later: laps, crashes, warnings, restarts, a disqualification.
struct PlayedMatch {
  slipx::scene::Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  RaceConfig config;

  PlayedMatch() {
    sim.add_agent(race_test::race_car(
        race_test::follow_centreline(track, 2.0, 0.0)));
    sim.add_agent(race_test::race_car(
        race_test::follow_centreline(track, 5.0, 0.0)));
    config.laps_to_win = 10;
  }
};

TEST(EventStream, AFullRaceReplaysFromTheStreamAlone) {
  PlayedMatch fixture;
  Match match(fixture.sim, fixture.track, 0, 1, 0.0, true, 7,
              fixture.config);
  match.run(200000);
  ASSERT_TRUE(match.finished());

  const std::string path = temp_path("race_events.mcap");
  ASSERT_TRUE(slipx::race::write_event_stream(path, match.events(),
                                              fixture.config,
                                              {{"scenario", "test"}}));

  // The consumer: the file, and nothing else.
  EventStreamContents contents;
  ASSERT_TRUE(slipx::race::read_event_stream(path, &contents));

  // Field for field, bit for bit: seventeen significant digits round-trip
  // a double exactly, so equality here is EXPECT_EQ, not a tolerance.
  ASSERT_EQ(contents.events.size(), match.events().size());
  for (std::size_t i = 0; i < contents.events.size(); ++i) {
    const RaceEvent& read = contents.events[i];
    const RaceEvent& live = match.events()[i];
    EXPECT_EQ(read.type, live.type) << "event " << i;
    EXPECT_EQ(read.step, live.step);
    EXPECT_EQ(read.time, live.time);
    EXPECT_EQ(read.agent, live.agent);
    EXPECT_EQ(read.other, live.other);
    EXPECT_EQ(read.value, live.value);
    EXPECT_EQ(read.code, live.code);
  }

  // The race, reconstructed from the parsed events alone.
  std::uint32_t winner = slipx::race::kNoAgent;
  std::array<int, 2> warnings{0, 0};
  int crashes = 0;
  bool disqualified = false;
  for (const RaceEvent& event : contents.events) {
    switch (event.type) {
      case EventType::kMatchWon: winner = event.agent; break;
      case EventType::kWarning:
        warnings[event.agent] = event.code;
        break;
      case EventType::kCrash: ++crashes; break;
      case EventType::kDisqualified: disqualified = true; break;
      default: break;
    }
  }
  EXPECT_EQ(winner, 0u) << "the rammed leader wins by disqualification";
  EXPECT_TRUE(disqualified);
  EXPECT_EQ(crashes, fixture.config.warnings_to_disqualify);
  EXPECT_EQ(warnings[1], fixture.config.warnings_to_disqualify);

  // And what race it was: the pinned ruleset and the mechanised
  // configuration travel in the file, no repository required.
  EXPECT_EQ(contents.metadata_value("ruleset_revision"),
            slipx::race::kRulesetRevision);
  EXPECT_EQ(contents.metadata_value("config.warnings_to_disqualify"), "3");
  EXPECT_EQ(contents.metadata_value("scenario"), "test");
}

TEST(EventStream, TheSameRaceWritesTheSameBytes) {
  const auto play_and_write = [](const char* name) {
    PlayedMatch fixture;
    Match match(fixture.sim, fixture.track, 0, 1, 0.0, true, 7,
                fixture.config);
    match.run(200000);
    const std::string path = temp_path(name);
    EXPECT_TRUE(slipx::race::write_event_stream(path, match.events(),
                                                fixture.config));
    std::ifstream file(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(file)),
                       std::istreambuf_iterator<char>());
  };

  const std::string first = play_and_write("race_a.mcap");
  const std::string second = play_and_write("race_b.mcap");
  ASSERT_FALSE(first.empty());
  EXPECT_EQ(first, second);
}

TEST(EventStream, AFileThatIsNotAStreamIsRefused) {
  const std::string path = temp_path("not_mcap.bin");
  {
    std::ofstream file(path, std::ios::binary);
    file << "this is not an mcap file";
  }
  EventStreamContents contents;
  EXPECT_FALSE(slipx::race::read_event_stream(path, &contents));
  EXPECT_FALSE(slipx::race::read_event_stream(
      temp_path("does_not_exist.mcap"), &contents));
}

TEST(EventStream, AbsenceIsAbsentOnTheWire) {
  // The SlipX rule everywhere: what does not apply is left out, never
  // written as a sentinel a reader would have to be told about. A warning
  // has no second car, so its message has no "other" key at all.
  RaceEvent event;
  event.type = EventType::kWarning;
  event.step = 10;
  event.time = 0.01;
  event.agent = 2;
  event.other = slipx::race::kNoAgent;
  event.code = 1;

  const std::string path = temp_path("one_event.mcap");
  ASSERT_TRUE(
      slipx::race::write_event_stream(path, {event}, RaceConfig{}));

  // The schema mentions "other" as a property, so the scan looks for what
  // a sentinel would actually look like: the key with a NUMBER after it.
  std::ifstream file(path, std::ios::binary);
  const std::string bytes((std::istreambuf_iterator<char>(file)),
                          std::istreambuf_iterator<char>());
  EXPECT_NE(bytes.find("\"agent\":2"), std::string::npos);
  EXPECT_EQ(bytes.find("\"other\":4294967295"), std::string::npos);
  EXPECT_EQ(bytes.find("\"other\":-1"), std::string::npos);

  EventStreamContents contents;
  ASSERT_TRUE(slipx::race::read_event_stream(path, &contents));
  ASSERT_EQ(contents.events.size(), 1u);
  EXPECT_EQ(contents.events[0].other, slipx::race::kNoAgent);
}

TEST(EventStream, EveryEventTypeRoundTripsItsName) {
  for (int k = 0; k <= static_cast<int>(EventType::kHeatEnd); ++k) {
    const EventType type = static_cast<EventType>(k);
    EventType back = EventType::kRoundStart;
    ASSERT_TRUE(slipx::race::event_type_from_string(
        slipx::race::to_string(type), &back))
        << "type " << k;
    EXPECT_EQ(back, type);
  }
  EventType ignored;
  EXPECT_FALSE(slipx::race::event_type_from_string("unknown", &ignored));
}

}  // namespace
