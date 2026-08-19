// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Head-to-head racing (ruleset 2.5): the round, the match, the mechanised
// referee. Every scenario here is deterministic, and the assertions are on
// outcomes the rulebook names.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "slipx/race/head_to_head.hpp"
#include "slipx/race/starts.hpp"
#include "slipx/sim/mailbox.hpp"
#include "race_test_support.hpp"

namespace {

using slipx::race::EventType;
using slipx::race::HeadToHeadRound;
using slipx::race::Match;
using slipx::race::RaceConfig;
using slipx::race::RaceEvent;
using slipx::scene::Track;

std::vector<RaceEvent> events_of(const std::vector<RaceEvent>& events,
                                 EventType type) {
  std::vector<RaceEvent> out;
  for (const RaceEvent& e : events) {
    if (e.type == type) out.push_back(e);
  }
  return out;
}

RaceConfig short_race() {
  RaceConfig config;
  config.laps_to_win = 2;   // 20 laps (2.5.4.1) makes a slow test suite
  return config;
}

TEST(HeadToHead, ACleanRoundGoesToTheCarThatFinishesFirst) {
  const Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  // Different lanes on purpose: this scenario is about laps, not contact.
  sim.add_agent(
      race_test::race_car(race_test::follow_centreline(track, 5.0, 0.25)));
  sim.add_agent(
      race_test::race_car(race_test::follow_centreline(track, 3.5, -0.25)));

  HeadToHeadRound round(sim, track, 0, 1, 0.0, true, {0, 0}, short_race());
  round.run(60000);

  ASSERT_TRUE(round.finished());
  EXPECT_EQ(round.winner(), 0u) << "the faster car wins";
  EXPECT_EQ(round.laps(0), 2);
  EXPECT_FALSE(round.disqualified(0));
  EXPECT_FALSE(round.disqualified(1));

  const auto starts = events_of(round.events(), EventType::kRoundStart);
  ASSERT_EQ(starts.size(), 1u);
  EXPECT_EQ(starts[0].agent, 0u) << "car a took the left slot";
  const auto wins = events_of(round.events(), EventType::kRoundWon);
  ASSERT_EQ(wins.size(), 1u);
  EXPECT_EQ(wins[0].agent, 0u);
  EXPECT_EQ(wins[0].code, 0) << "won on the road, not by default";
  EXPECT_FALSE(events_of(round.events(), EventType::kLap).empty());
}

TEST(HeadToHead, ARearEndIsTheFollowersFaultAndThreeOfThemDisqualify) {
  const Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  // Both fight for the same racing line; the chaser is faster and will
  // rear-end the leader. Attribution (2.5.1.14.4, mechanised): the car
  // contributing more approach speed is at fault, which is the chaser.
  sim.add_agent(
      race_test::race_car(race_test::follow_centreline(track, 2.0, 0.0)));
  sim.add_agent(
      race_test::race_car(race_test::follow_centreline(track, 5.0, 0.0)));

  RaceConfig config = short_race();
  config.laps_to_win = 10;   // nobody finishes before the referee acts
  HeadToHeadRound round(sim, track, 0, 1, 0.0, true, {0, 0}, config);

  // To the first crash.
  for (int i = 0; i < 60000 && round.warnings(1) == 0 && !round.finished();
       ++i) {
    round.advance();
  }
  const auto crashes = events_of(round.events(), EventType::kCrash);
  ASSERT_FALSE(crashes.empty()) << "the chaser must catch the leader";
  EXPECT_EQ(crashes[0].agent, 1u) << "the follower is at fault";
  EXPECT_EQ(crashes[0].other, 0u);
  EXPECT_GT(crashes[0].value, config.light_contact_speed);
  EXPECT_EQ(round.warnings(1), 1);

  // The restart of 2.5.1.14.5 and .9: both at rest, the at-fault car the
  // gap plus the recovery bonus behind the victim along the track.
  EXPECT_DOUBLE_EQ(sim.state(0).speed(), 0.0);
  EXPECT_DOUBLE_EQ(sim.state(1).speed(), 0.0);
  const double s_victim =
      slipx::scene::project(track, sim.state(0).pos.x, sim.state(0).pos.y).s;
  const double s_fault =
      slipx::scene::project(track, sim.state(1).pos.x, sim.state(1).pos.y).s;
  double gap = s_victim - s_fault;
  if (gap < 0.0) gap += track.length();
  EXPECT_NEAR(gap, config.restart_gap + config.recovery_bonus, 0.15);

  // Run on: the same pair of policies produces the same crash again and
  // again, and the third warning disqualifies (2.5.1.14.7-8).
  round.run(120000);
  ASSERT_TRUE(round.finished());
  EXPECT_TRUE(round.disqualified(1));
  EXPECT_EQ(round.winner(), 0u);
  EXPECT_EQ(round.warnings(1), config.warnings_to_disqualify);
  EXPECT_FALSE(events_of(round.events(), EventType::kDisqualified).empty());
  const auto wins = events_of(round.events(), EventType::kRoundWon);
  ASSERT_EQ(wins.size(), 1u);
  EXPECT_EQ(wins[0].code, 1) << "won by the opponent's disqualification";
}

TEST(HeadToHead, ASlowNudgeIsRecordedAndNotPenalised) {
  const Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  // Two coasting cars, side by side and just clear, angled together by a
  // hair: they touch at a closing speed far below the threshold
  // (2.5.1.14.2).
  sim.add_agent(race_test::race_car({}));
  sim.add_agent(race_test::race_car({}));

  RaceConfig config = short_race();
  HeadToHeadRound round(sim, track, 0, 1, 0.0, true, {0, 0}, config);

  // Override the grid: place them converging gently.
  slipx::race::place_on_track(sim, 0, track, 1.0, 0.17, 2.0);
  slipx::race::place_on_track(sim, 1, track, 1.0, -0.17, 2.0);
  sim.state(0).yaw -= 0.02;   // drift right, towards the other car

  round.run(3000);

  const auto light = events_of(round.events(), EventType::kContactLight);
  EXPECT_FALSE(light.empty())
      << "they must actually touch, or this asserts nothing";
  EXPECT_LT(light.size(), 50u)
      << "one event per touch EPISODE: sustained rubbing recorded step by "
         "step would flood the stream";
  EXPECT_TRUE(events_of(round.events(), EventType::kCrash).empty());
  EXPECT_TRUE(events_of(round.events(), EventType::kRestart).empty());
  EXPECT_EQ(round.warnings(0), 0);
  EXPECT_EQ(round.warnings(1), 0);
}

TEST(HeadToHead, ADnfHandsTheRoundToTheSurvivor) {
  const Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  sim.add_agent(
      race_test::race_car(race_test::follow_centreline(track, 4.0, 0.25)));
  // The second car takes commands from a mailbox nobody posts to, with the
  // DNF timeout policy: hung from the first step (ADR-0044).
  slipx::sim::AgentSpec hung = race_test::race_car({});
  hung.mailbox = std::make_shared<slipx::sim::CommandMailbox>();
  hung.timeout_policy = slipx::sim::TimeoutPolicy::kDnf;
  sim.add_agent(std::move(hung));

  HeadToHeadRound round(sim, track, 0, 1, 0.0, true, {0, 0}, short_race());
  round.run(1000);

  ASSERT_TRUE(round.finished());
  EXPECT_EQ(round.winner(), 0u);
  EXPECT_FALSE(events_of(round.events(), EventType::kDnf).empty());
  const auto wins = events_of(round.events(), EventType::kRoundWon);
  ASSERT_EQ(wins.size(), 1u);
  EXPECT_EQ(wins[0].code, 1) << "won by the opponent's DNF";
}

TEST(HeadToHead, AMatchSwapsSidesAndTakesTwoRounds) {
  const Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  sim.add_agent(
      race_test::race_car(race_test::follow_centreline(track, 5.0, 0.25)));
  sim.add_agent(
      race_test::race_car(race_test::follow_centreline(track, 3.5, -0.25)));

  Match match(sim, track, 0, 1, 0.0, true, 42, short_race());
  match.run(60000);

  ASSERT_TRUE(match.finished());
  EXPECT_EQ(match.winner(), 0u);
  EXPECT_EQ(match.round_wins(0), 2);
  EXPECT_EQ(match.round_wins(1), 0);
  EXPECT_EQ(match.rounds_played(), 2);

  // Sides: car a chose the left in round one, and the sides swapped in
  // round two (2.5.1.9.2-3).
  const auto starts = events_of(match.events(), EventType::kRoundStart);
  ASSERT_EQ(starts.size(), 2u);
  EXPECT_EQ(starts[0].agent, 0u);
  EXPECT_EQ(starts[1].agent, 1u);

  const auto match_wins = events_of(match.events(), EventType::kMatchWon);
  ASSERT_EQ(match_wins.size(), 1u);
  EXPECT_EQ(match_wins[0].agent, 0u);
}

TEST(HeadToHead, ADisqualificationEndsTheMatchNotJustTheRound) {
  const Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  sim.add_agent(
      race_test::race_car(race_test::follow_centreline(track, 2.0, 0.0)));
  sim.add_agent(
      race_test::race_car(race_test::follow_centreline(track, 5.0, 0.0)));

  RaceConfig config = short_race();
  config.laps_to_win = 10;
  Match match(sim, track, 0, 1, 0.0, true, 7, config);
  match.run(200000);

  ASSERT_TRUE(match.finished());
  EXPECT_EQ(match.winner(), 0u);
  EXPECT_EQ(match.rounds_played(), 1) << "disqualification is from the "
                                         "race, not the round (2.5.1.14.8)";
}

TEST(HeadToHead, TheSameMatchTwiceIsTheSameMatch) {
  const auto play = [] {
    const Track track = race_test::shipped_track();
    slipx::sim::Simulation sim;
    sim.add_agent(
        race_test::race_car(race_test::follow_centreline(track, 5.0, 0.25)));
    sim.add_agent(
        race_test::race_car(race_test::follow_centreline(track, 3.5, -0.25)));
    Match match(sim, track, 0, 1, 0.0, true, 42, short_race());
    match.run(60000);
    return match.events().size();
  };
  EXPECT_EQ(play(), play());
}

}  // namespace
