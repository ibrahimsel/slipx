// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/race/wrong_way.hpp"

#include <cmath>
#include <stdexcept>

namespace slipx {
namespace race {

WrongWayMonitor::WrongWayMonitor(double distance) : threshold_(distance) {
  if (!std::isfinite(distance) || distance <= 0.0) {
    throw std::invalid_argument(
        "wrong_way_distance must be a positive, finite number of metres: "
        "a zero threshold would rule on the first numerical wobble of a "
        "projection.");
  }
}

bool WrongWayMonitor::update(double progress) {
  if (!seeded_) {
    seeded_ = true;
    high_water_ = progress;
    progress_ = progress;
    return false;
  }

  progress_ = progress;
  if (progress >= high_water_) {
    // The lost ground is made back (or was never lost): a new furthest
    // point, and the next excursion is a new ruling.
    high_water_ = progress;
    ruled_ = false;
    return false;
  }
  if (!ruled_ && high_water_ - progress > threshold_) {
    ruled_ = true;
    return true;
  }
  return false;
}

void WrongWayMonitor::rebase(double progress) {
  seeded_ = true;
  high_water_ = progress;
  progress_ = progress;
  ruled_ = false;
}

double WrongWayMonitor::deficit() const {
  const double behind = high_water_ - progress_;
  return behind > 0.0 ? behind : 0.0;
}

}  // namespace race
}  // namespace slipx
