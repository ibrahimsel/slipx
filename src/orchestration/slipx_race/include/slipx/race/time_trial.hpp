// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The time trial (ruleset section 2.4): one car, one heat, ranked on the
// fastest lap and on the longest run of consecutive clean laps.

#ifndef SLIPX_RACE_TIME_TRIAL_HPP
#define SLIPX_RACE_TIME_TRIAL_HPP

#include <cstddef>
#include <vector>

#include "slipx/race/events.hpp"
#include "slipx/race/ruleset.hpp"
#include "slipx/scene/lap.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/sim/simulation.hpp"

namespace slipx {
namespace race {

struct TimeTrialResult {
  int laps = 0;              // completed, clean or not
  // Fastest CLEAN lap. Infinity when no clean lap was completed, which
  // ranks it last, exactly where a heat with no flying lap belongs.
  double fastest_lap = 0.0;  // set to +inf by the constructor below   [s]
  int best_streak = 0;       // longest run of consecutive clean laps
  bool dnf = false;
};

class TimeTrial {
 public:
  // The car runs from wherever it stands; the caller places it (a heat
  // starts from the pit lane, not from a grid). The lap counter seeds at
  // the current position, so lap one begins here.
  TimeTrial(sim::Simulation& sim, const scene::Track& track,
            std::size_t agent, RaceConfig config);

  // One simulation step plus the rules: laps timed, a corridor excursion is
  // a border crash (2.5.3.1) that resets the streak and restarts the car at
  // rest where it left (2.5.3.3), a DNF ends the heat.
  void advance();

  // The whole heat (2.4: five-minute heats; the duration is the caller's).
  void run_for(double seconds);

  bool finished() const { return dnf_; }
  const TimeTrialResult& result() const { return result_; }
  const std::vector<RaceEvent>& events() const { return events_; }

 private:
  void emit(EventType type, double value, int code);

  sim::Simulation& sim_;
  const scene::Track& track_;
  std::size_t agent_;
  RaceConfig config_;

  scene::LapCounter counter_;
  bool was_inside_ = true;
  double lap_start_time_ = 0.0;
  bool lap_dirty_ = false;
  int streak_ = 0;
  bool dnf_ = false;

  TimeTrialResult result_;
  std::vector<RaceEvent> events_;
};

// The scoring of 2.4.5 across one heat's results: teams are ranked on
// fastest lap and, separately, on the consecutive-clean-laps streak; each
// ranking awards n points down to 1; the score is the sum. Deterministic
// tie-breaks inside each ranking go to the lower index, which stands in for
// the rulebook's silence on within-category ties.
std::vector<int> score_time_trial(const std::vector<TimeTrialResult>& results);

// The final ordering: by score, ties broken by more laps (2.4.5), then by
// index. Returns indices, best first.
std::vector<std::size_t> time_trial_ranking(
    const std::vector<TimeTrialResult>& results);

}  // namespace race
}  // namespace slipx

#endif  // SLIPX_RACE_TIME_TRIAL_HPP
