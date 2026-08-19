// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The obstacle avoidance test (ruleset 2.5.1.6): before a head-to-head, a
// car must get past an obstacle using its race code, without touching it
// and without coming to a complete stop (2.5.1.6.2-3). Stopping in front of
// it is explicitly not a pass: the test exists to force overtaking, not
// caution.

#ifndef SLIPX_RACE_OBSTACLE_TEST_HPP
#define SLIPX_RACE_OBSTACLE_TEST_HPP

#include <cstddef>
#include <vector>

#include "slipx/race/events.hpp"
#include "slipx/race/ruleset.hpp"
#include "slipx/scene/lap.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/sim/simulation.hpp"

namespace slipx {
namespace race {

enum class ObstacleOutcome { kPending, kPassed, kFailed };

// Failure codes carried in the event's `code` field.
inline constexpr int kObstacleFailStopped = 1;   // came to a complete stop
inline constexpr int kObstacleFailContact = 2;   // touched the obstacle
inline constexpr int kObstacleFailBorder = 3;    // crashed the border
inline constexpr int kObstacleFailDnf = 4;       // rolled or timed out

class ObstacleTest {
 public:
  // `obstacle` is another agent in the same simulation, footprinted and
  // typically at rest with no policy: the sim's contact model is what makes
  // touching it a detectable fact. The pass line is the obstacle's arc
  // position plus `pass_margin` metres.
  ObstacleTest(sim::Simulation& sim, const scene::Track& track,
               std::size_t car, std::size_t obstacle, RaceConfig config,
               double pass_margin = 1.0);

  void advance();
  void run(std::uint64_t max_steps);

  ObstacleOutcome outcome() const { return outcome_; }
  int failure_code() const { return failure_code_; }
  const std::vector<RaceEvent>& events() const { return events_; }

 private:
  void fail(int code);

  sim::Simulation& sim_;
  std::size_t car_;
  std::size_t obstacle_;
  RaceConfig config_;

  scene::LapCounter counter_;
  double distance_to_pass_ = 0.0;
  // The stop check arms once the car has actually got going, or a standing
  // start would fail at step one for being a standing start.
  bool moving_yet_ = false;

  ObstacleOutcome outcome_ = ObstacleOutcome::kPending;
  int failure_code_ = 0;
  std::vector<RaceEvent> events_;
};

}  // namespace race
}  // namespace slipx

#endif  // SLIPX_RACE_OBSTACLE_TEST_HPP
