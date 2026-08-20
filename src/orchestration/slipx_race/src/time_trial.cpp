// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/race/time_trial.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "slipx/race/starts.hpp"

namespace slipx {
namespace race {

TimeTrial::TimeTrial(sim::Simulation& sim, const scene::Track& track,
                     std::size_t agent, RaceConfig config)
    : sim_(sim),
      agent_(agent),
      config_(config),
      track_(config.reversed ? track.reversed() : track),
      counter_(track_, config.limit_tolerance),
      wrong_way_(config.wrong_way_distance) {
  result_.fastest_lap = std::numeric_limits<double>::infinity();
  const VehicleState& state = sim_.state(agent_);
  counter_.reset_to(state.pos.x, state.pos.y);
  lap_start_time_ = sim_.time();
}

void TimeTrial::emit(EventType type, double value, int code) {
  RaceEvent event;
  event.type = type;
  event.step = sim_.step_count();
  event.time = sim_.time();
  event.agent = static_cast<std::uint32_t>(agent_);
  event.value = value;
  event.code = code;
  events_.push_back(event);
}

void TimeTrial::advance() {
  if (dnf_) return;
  sim_.advance();

  if (!sim_.agent_running(agent_)) {
    dnf_ = true;
    result_.dnf = true;
    emit(EventType::kDnf, 0.0, 0);
    emit(EventType::kHeatEnd, 0.0, 0);
    return;
  }

  const VehicleState& state = sim_.state(agent_);
  counter_.update(state.pos.x, state.pos.y);

  // The border, enforced by rule rather than by physics (2.5.3.1): beyond
  // the tolerance the car has crashed the border and is placed at rest
  // where it left (2.5.3.3). The lap it was on is no longer a flying lap
  // and the streak restarts.
  const bool inside = counter_.limits().inside;
  if (was_inside_ && !inside) {
    emit(EventType::kBorderCrash, counter_.limits().margin, 0);
    const double s = counter_.where().s;
    place_on_track(sim_, agent_, track_, s, 0.0, 0.0);
    const VehicleState& placed = sim_.state(agent_);
    counter_.update(placed.pos.x, placed.pos.y);
    // A placement is a teleport, not driving: rebase rather than rule.
    wrong_way_.rebase(counter_.distance());
    emit(EventType::kRestart, s, 0);
    lap_dirty_ = true;
    streak_ = 0;
  }
  was_inside_ = counter_.limits().inside;

  if (wrong_way_.update(counter_.distance())) {
    emit(EventType::kWrongWay, wrong_way_.deficit(), 0);
  }

  if (counter_.laps() > result_.laps) {
    result_.laps = counter_.laps();
    const double lap_time = sim_.time() - lap_start_time_;
    emit(EventType::kLap, lap_time, result_.laps);
    if (!lap_dirty_) {
      result_.fastest_lap = std::min(result_.fastest_lap, lap_time);
      ++streak_;
      result_.best_streak = std::max(result_.best_streak, streak_);
    } else {
      // The crash already reset the running streak; the interrupted lap
      // itself simply never counts as clean.
      lap_dirty_ = false;
    }
    lap_start_time_ = sim_.time();
  }
}

void TimeTrial::run_for(double seconds) {
  const auto steps = static_cast<std::uint64_t>(seconds / sim_.dt());
  for (std::uint64_t i = 0; i < steps && !dnf_; ++i) advance();
  if (!dnf_) emit(EventType::kHeatEnd, 0.0, 0);
}

std::vector<int> score_time_trial(
    const std::vector<TimeTrialResult>& results) {
  const std::size_t n = results.size();
  std::vector<int> points(n, 0);

  // Rank by a criterion and award n points down to 1 (2.4.5). Ties inside a
  // category go to the lower index, deterministically, standing in for the
  // rulebook's silence.
  const auto award = [&](auto better) {
    std::vector<std::size_t> order(n);
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::sort(order.begin(), order.end(),
              [&](std::size_t a, std::size_t b) {
                if (better(a, b)) return true;
                if (better(b, a)) return false;
                return a < b;
              });
    for (std::size_t rank = 0; rank < n; ++rank) {
      points[order[rank]] += static_cast<int>(n - rank);
    }
  };

  award([&](std::size_t a, std::size_t b) {
    return results[a].fastest_lap < results[b].fastest_lap;
  });
  award([&](std::size_t a, std::size_t b) {
    return results[a].best_streak > results[b].best_streak;
  });
  return points;
}

std::vector<std::size_t> time_trial_ranking(
    const std::vector<TimeTrialResult>& results) {
  const std::vector<int> points = score_time_trial(results);
  std::vector<std::size_t> order(results.size());
  std::iota(order.begin(), order.end(), std::size_t{0});
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    if (points[a] != points[b]) return points[a] > points[b];
    // "In case of ties, the team with more laps is ranked higher" (2.4.5).
    if (results[a].laps != results[b].laps) {
      return results[a].laps > results[b].laps;
    }
    return a < b;
  });
  return order;
}

}  // namespace race
}  // namespace slipx
