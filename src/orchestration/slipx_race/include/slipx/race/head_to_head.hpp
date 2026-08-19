// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Head-to-head racing (ruleset 2.5): a round is first-to-N laps from a
// side-by-side standing start, a match is best of three rounds with sides
// swapped, and contact is attributed and penalised by the mechanised rules
// of ADR-0046.

#ifndef SLIPX_RACE_HEAD_TO_HEAD_HPP
#define SLIPX_RACE_HEAD_TO_HEAD_HPP

#include <array>
#include <cstddef>
#include <vector>

#include "slipx/race/events.hpp"
#include "slipx/race/ruleset.hpp"
#include "slipx/scene/lap.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/sim/simulation.hpp"

namespace slipx {
namespace race {

// One round: grid start (2.5.1.9), first to laps_to_win laps (2.5.4.1),
// light contact recorded and not penalised (2.5.1.14.2), a crash attributed
// and restarted with the at-fault car set back (2.5.1.14.4-5, .9), every
// at-fault crash a warning and the third a disqualification (2.5.1.14.7-8),
// the border enforced by rule rather than by physics (2.5.3), and an
// opponent's DNF ending the round.
class HeadToHeadRound {
 public:
  // Places both cars on the grid at `line_s` (car_a on the left when
  // a_on_left) and seeds the lap counters there. `warnings` carries the
  // teams' warning counts into the round, because warnings accumulate over
  // the race, not the round.
  HeadToHeadRound(sim::Simulation& sim, const scene::Track& track,
                  std::size_t car_a, std::size_t car_b, double line_s,
                  bool a_on_left, std::array<int, 2> warnings,
                  RaceConfig config);

  void advance();
  void run(std::uint64_t max_steps);   // stops early once decided

  bool finished() const { return finished_; }
  // Meaningful only when finished(): 0 for car_a, 1 for car_b.
  std::size_t winner() const { return winner_; }
  bool disqualified(std::size_t which) const { return dq_[which]; }
  int warnings(std::size_t which) const { return warnings_[which]; }
  int laps(std::size_t which) const;
  const std::vector<RaceEvent>& events() const { return events_; }

 private:
  std::size_t sim_index(std::size_t which) const {
    return which == 0 ? car_a_ : car_b_;
  }
  void emit(EventType type, std::uint32_t agent, std::uint32_t other,
            double value, int code);
  void decide(std::size_t winner, int code);
  void handle_border(std::size_t which);
  void handle_contact();

  sim::Simulation& sim_;
  const scene::Track& track_;
  std::size_t car_a_;
  std::size_t car_b_;
  RaceConfig config_;

  std::array<scene::LapCounter, 2> counters_;
  std::array<bool, 2> was_inside_{true, true};
  std::array<double, 2> lap_start_time_{0.0, 0.0};
  std::array<int, 2> laps_done_{0, 0};
  std::array<int, 2> warnings_;
  std::array<bool, 2> dq_{false, false};
  std::array<bool, 2> dnf_noted_{false, false};
  bool in_contact_ = false;

  bool finished_ = false;
  std::size_t winner_ = 0;
  std::vector<RaceEvent> events_;
};

// Best of three (2.5.1.8): sides chosen by the higher qualifier in round
// one (the caller's a_on_left_first), swapped in round two (2.5.1.9.3), and
// decided by the rulebook's coin flip in round three (2.5.1.9.4), which a
// deterministic simulator mechanises as a seeded draw so a replayed match
// is the same match. The simulation is reset between rounds: that is the
// ten-minute repair window, compressed.
class Match {
 public:
  Match(sim::Simulation& sim, const scene::Track& track, std::size_t car_a,
        std::size_t car_b, double line_s, bool a_on_left_first,
        std::uint64_t seed, RaceConfig config);

  // Runs rounds until a team has rounds_to_win or is disqualified. A round
  // that exhausts its step budget undecided abandons the match with
  // finished() false: the rulebook has no rule for a race that never ends,
  // and inventing one here would be a quiet lie.
  void run(std::uint64_t max_steps_per_round);

  bool finished() const { return finished_; }
  std::size_t winner() const { return winner_; }
  int round_wins(std::size_t which) const { return round_wins_[which]; }
  int rounds_played() const { return rounds_played_; }
  const std::vector<RaceEvent>& events() const { return events_; }

 private:
  sim::Simulation& sim_;
  const scene::Track& track_;
  std::size_t car_a_;
  std::size_t car_b_;
  double line_s_;
  bool a_on_left_first_;
  std::uint64_t seed_;
  RaceConfig config_;

  std::array<int, 2> round_wins_{0, 0};
  std::array<int, 2> warnings_{0, 0};
  int rounds_played_ = 0;
  bool finished_ = false;
  std::size_t winner_ = 0;
  std::vector<RaceEvent> events_;
};

}  // namespace race
}  // namespace slipx

#endif  // SLIPX_RACE_HEAD_TO_HEAD_HPP
