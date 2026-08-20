// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/race/obstacle_test.hpp"

#include <cmath>

namespace slipx {
namespace race {

ObstacleTest::ObstacleTest(sim::Simulation& sim, const scene::Track& track,
                           std::size_t car, std::size_t obstacle,
                           RaceConfig config, double pass_margin)
    : sim_(sim),
      car_(car),
      obstacle_(obstacle),
      config_(config),
      track_(config.reversed ? track.reversed() : track),
      counter_(track_, config.limit_tolerance) {
  const VehicleState& state = sim_.state(car_);
  counter_.reset_to(state.pos.x, state.pos.y);

  // How far along the track the car has to travel to be past the obstacle:
  // the gap from its own arc position forward to the obstacle's, wrapped on
  // a closed track, plus the margin. Measured through the counter's
  // progress, so a car that wanders backwards first still has to cover it.
  const VehicleState& box = sim_.state(obstacle_);
  const double s_car = counter_.where().s;
  const double s_obstacle = scene::project(track_, box.pos.x, box.pos.y).s;
  double gap = s_obstacle - s_car;
  if (track_.is_closed()) {
    const double total = track_.length();
    gap = std::fmod(gap, total);
    if (gap < 0.0) gap += total;
  }
  distance_to_pass_ = gap + pass_margin;
}

void ObstacleTest::fail(int code) {
  outcome_ = ObstacleOutcome::kFailed;
  failure_code_ = code;
  RaceEvent event;
  event.type = EventType::kObstacleFailed;
  event.step = sim_.step_count();
  event.time = sim_.time();
  event.agent = static_cast<std::uint32_t>(car_);
  event.other = static_cast<std::uint32_t>(obstacle_);
  event.code = code;
  events_.push_back(event);
}

void ObstacleTest::advance() {
  if (outcome_ != ObstacleOutcome::kPending) return;
  sim_.advance();

  if (!sim_.agent_running(car_)) {
    fail(kObstacleFailDnf);
    return;
  }

  // Touching the obstacle at all is a failed pass (2.5.1.6: the point of
  // the test is a clean overtake).
  const std::uint32_t lo =
      static_cast<std::uint32_t>(car_ < obstacle_ ? car_ : obstacle_);
  const std::uint32_t hi =
      static_cast<std::uint32_t>(car_ < obstacle_ ? obstacle_ : car_);
  for (const sim::ContactEvent& contact : sim_.contacts()) {
    if (contact.a == lo && contact.b == hi) {
      fail(kObstacleFailContact);
      return;
    }
  }

  const VehicleState& state = sim_.state(car_);
  counter_.update(state.pos.x, state.pos.y);

  if (!counter_.limits().inside) {
    fail(kObstacleFailBorder);
    return;
  }

  // "The car must pass the obstacle without coming to a complete stop"
  // (2.5.1.6.3). The check arms once the car is moving, or a standing start
  // would fail for being a standing start.
  const double speed = state.speed();
  if (speed > config_.stop_speed) moving_yet_ = true;
  if (moving_yet_ && speed < config_.stop_speed) {
    fail(kObstacleFailStopped);
    return;
  }

  if (counter_.distance() >= distance_to_pass_) {
    outcome_ = ObstacleOutcome::kPassed;
    RaceEvent event;
    event.type = EventType::kObstaclePassed;
    event.step = sim_.step_count();
    event.time = sim_.time();
    event.agent = static_cast<std::uint32_t>(car_);
    event.other = static_cast<std::uint32_t>(obstacle_);
    events_.push_back(event);
  }
}

void ObstacleTest::run(std::uint64_t max_steps) {
  for (std::uint64_t i = 0;
       i < max_steps && outcome_ == ObstacleOutcome::kPending; ++i) {
    advance();
  }
}

}  // namespace race
}  // namespace slipx
