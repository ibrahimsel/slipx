// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The time trial (ruleset 2.4): laps timed, the border enforced by rule,
// and the two-category scoring of 2.4.5.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "slipx/race/starts.hpp"
#include "slipx/race/time_trial.hpp"
#include "race_test_support.hpp"

namespace {

using slipx::race::EventType;
using slipx::race::RaceConfig;
using slipx::race::TimeTrial;
using slipx::race::TimeTrialResult;
using slipx::scene::Track;

bool has_event(const std::vector<slipx::race::RaceEvent>& events,
               EventType type) {
  return std::any_of(events.begin(), events.end(),
                     [type](const slipx::race::RaceEvent& e) {
                       return e.type == type;
                     });
}

std::vector<slipx::race::RaceEvent> events_of(
    const std::vector<slipx::race::RaceEvent>& events, EventType type) {
  std::vector<slipx::race::RaceEvent> out;
  for (const slipx::race::RaceEvent& e : events) {
    if (e.type == type) out.push_back(e);
  }
  return out;
}

TEST(TimeTrial, ACleanHeatTimesItsLapsAndKeepsItsStreak) {
  const Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  sim.add_agent(race_test::race_car(race_test::follow_centreline(track, 4.0)));
  slipx::race::place_on_track(sim, 0, track, 0.0, 0.0, 4.0);

  RaceConfig config;
  TimeTrial trial(sim, track, 0, config);
  trial.run_for(30.0);

  const TimeTrialResult& result = trial.result();
  ASSERT_GE(result.laps, 2) << "the fixture controller must actually lap";
  EXPECT_FALSE(result.dnf);

  // A steady 4 m/s lap of a known track length: the fastest lap is about
  // length over speed, and never faster than the track allows.
  const double nominal = track.length() / 4.0;
  EXPECT_GT(result.fastest_lap, 0.8 * nominal);
  EXPECT_LT(result.fastest_lap, 1.3 * nominal);

  // Every lap was clean, so the streak is the lap count.
  EXPECT_EQ(result.best_streak, result.laps);
  EXPECT_TRUE(has_event(trial.events(), EventType::kLap));
  EXPECT_TRUE(has_event(trial.events(), EventType::kHeatEnd));
  EXPECT_FALSE(has_event(trial.events(), EventType::kBorderCrash));
  EXPECT_FALSE(has_event(trial.events(), EventType::kWrongWay));
}

TEST(TimeTrial, AReversedHeatLapsTheOtherWayRound) {
  // Race control is handed the forward track and only the flag; the car
  // follows the raced direction. Laps must accumulate exactly as they do
  // forward, with nothing ruled wrong way (ADR-0056: reversal is the
  // reversed track, so no signed quantity needs a special case).
  const Track track = race_test::shipped_track();
  const Track raced = track.reversed();
  slipx::sim::Simulation sim;
  sim.add_agent(race_test::race_car(race_test::follow_centreline(raced, 4.0)));
  slipx::race::place_on_track(sim, 0, raced, 0.0, 0.0, 4.0);

  RaceConfig config;
  config.reversed = true;
  TimeTrial trial(sim, track, 0, config);
  trial.run_for(30.0);

  const TimeTrialResult& result = trial.result();
  ASSERT_GE(result.laps, 2) << "laps accumulate along the raced direction";
  EXPECT_FALSE(result.dnf);
  const double nominal = track.length() / 4.0;
  EXPECT_GT(result.fastest_lap, 0.8 * nominal);
  EXPECT_LT(result.fastest_lap, 1.3 * nominal);
  EXPECT_FALSE(has_event(trial.events(), EventType::kWrongWay));
  EXPECT_FALSE(has_event(trial.events(), EventType::kBorderCrash));
}

TEST(TimeTrial, DrivingAgainstTheRaceDirectionIsRuledOncePerExcursion) {
  // The car drives the track as declared while race control announced the
  // other way round: every metre is a metre lost.
  const Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  sim.add_agent(race_test::race_car(race_test::follow_centreline(track, 4.0)));
  slipx::race::place_on_track(sim, 0, track, 0.0, 0.0, 4.0);

  RaceConfig config;
  config.reversed = true;
  TimeTrial trial(sim, track, 0, config);
  trial.run_for(10.0);

  const auto rulings = events_of(trial.events(), EventType::kWrongWay);
  ASSERT_EQ(rulings.size(), 1u)
      << "one excursion, one ruling, however long it lasts";
  EXPECT_EQ(rulings[0].agent, 0u);
  EXPECT_GE(rulings[0].value, config.wrong_way_distance);
  EXPECT_LT(rulings[0].value, config.wrong_way_distance + 0.1);
  EXPECT_EQ(trial.result().laps, 0) << "backwards laps are not laps";
  EXPECT_FALSE(trial.result().dnf);
}

TEST(TimeTrial, LeavingTheCorridorIsABorderCrashThatResetsTheStreak) {
  const Track track = race_test::shipped_track();
  slipx::sim::Simulation sim;
  sim.add_agent(race_test::race_car(race_test::follow_centreline(track, 4.0)));
  slipx::race::place_on_track(sim, 0, track, 0.0, 0.0, 4.0);

  RaceConfig config;
  TimeTrial trial(sim, track, 0, config);

  // A clean lap first, so the streak has something to lose.
  trial.run_for(12.0);
  const int laps_before = trial.result().laps;
  ASSERT_GE(laps_before, 1);
  ASSERT_EQ(trial.result().best_streak, laps_before);

  // Shove the car 1.2 m to the left of the centreline, past the 0.75 m
  // corridor: the walls are rules here, not physics, and the corridor
  // check is what makes them real (2.5.3.1).
  slipx::VehicleState& state = sim.state(0);
  const auto where = slipx::scene::project(track, state.pos.x, state.pos.y);
  const auto pose = slipx::race::pose_at(track, where.s);
  state.pos.x = pose.x + 1.2 * -std::sin(pose.heading);
  state.pos.y = pose.y + 1.2 * std::cos(pose.heading);
  trial.advance();

  EXPECT_TRUE(has_event(trial.events(), EventType::kBorderCrash));
  EXPECT_TRUE(has_event(trial.events(), EventType::kRestart));
  // Placed back at rest inside the corridor (2.5.3.3).
  EXPECT_DOUBLE_EQ(sim.state(0).speed(), 0.0);

  // Run on: the streak restarted while the lap count kept going. The exact
  // arithmetic is assertable: the lap the crash interrupted is never clean,
  // so the best streak is the better of the pre-crash run and the
  // post-crash run, not their sum.
  trial.run_for(18.0);
  const TimeTrialResult& result = trial.result();
  EXPECT_GT(result.laps, laps_before);
  const int clean_after = result.laps - laps_before - 1;
  EXPECT_EQ(result.best_streak, std::max(laps_before, clean_after));
  EXPECT_LT(result.best_streak, result.laps)
      << "a heat with a border crash cannot be all-clean";
}

TEST(TimeTrialScoring, TwoCategoriesSumAndTiesGoToMoreLaps) {
  // Per 2.4.5: rank by fastest lap, rank by streak, points n..1 each,
  // final ties broken by lap count.
  std::vector<TimeTrialResult> results(3);
  results[0].fastest_lap = 10.0;
  results[0].best_streak = 3;
  results[0].laps = 5;
  results[1].fastest_lap = 9.0;
  results[1].best_streak = 1;
  results[1].laps = 4;
  results[2].fastest_lap = std::numeric_limits<double>::infinity();
  results[2].best_streak = 0;
  results[2].laps = 0;

  const std::vector<int> points = slipx::race::score_time_trial(results);
  // Fastest: 1 (3 pts), 0 (2), 2 (1). Streak: 0 (3), 1 (2), 2 (1).
  EXPECT_EQ(points[0], 5);
  EXPECT_EQ(points[1], 5);
  EXPECT_EQ(points[2], 2);

  const auto ranking = slipx::race::time_trial_ranking(results);
  // The 5-5 tie goes to team 0, which has more laps (2.4.5).
  EXPECT_EQ(ranking[0], 0u);
  EXPECT_EQ(ranking[1], 1u);
  EXPECT_EQ(ranking[2], 2u);
}

}  // namespace
