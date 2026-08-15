// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/scene/lap.hpp"

#include <cmath>
#include <stdexcept>

namespace slipx {
namespace scene {
namespace {

// How far the car moved along the centreline, resolving the wrap at the
// start line.
//
// On a closed track s jumps from just under the lap length to just over zero
// when the car crosses the line, and the raw difference is then a whole lap
// in the wrong direction. Taking the shorter of the two ways round fixes it,
// and the assumption that buys is that no single update moves the car more
// than half a lap. At a 1 kHz step and a 35 m lap that is 17 km/s, so the
// assumption is safe for a car and unsafe for a teleport, which is what
// reset_to exists for.
double progress(double from, double to, double lap_length, bool closed) {
  double delta = to - from;
  if (!closed) return delta;

  const double half = lap_length / 2.0;
  while (delta > half) delta -= lap_length;
  while (delta < -half) delta += lap_length;
  return delta;
}

}  // namespace

LapCounter::LapCounter(const Track& track, double limit_tolerance)
    : track_(&track), tolerance_(limit_tolerance) {
  if (!std::isfinite(limit_tolerance)) {
    throw std::invalid_argument(
        "track limit tolerance must be a finite number of metres. Zero is a "
        "rule, not an absence, so there is no default to fall back on.");
  }
}

void LapCounter::reset_to(double x, double y) {
  const bool first = !seeded_;

  where_ = project(*track_, x, y);
  limits_ = check_limits(*track_, where_, tolerance_);
  previous_s_ = where_.s;
  seeded_ = true;

  // The excursion history is not cleared. A restart is a new position, not a
  // new history: a car that put a wheel out before being recovered still put
  // a wheel out, and a counter that forgets it is one nobody can steward on.
  if (!limits_.inside) has_left_ = true;
  worst_margin_ = first ? limits_.margin
                        : std::fmin(worst_margin_, limits_.margin);
}

void LapCounter::update(double x, double y) {
  if (!seeded_) {
    reset_to(x, y);
    return;
  }

  where_ = project(*track_, x, y);
  limits_ = check_limits(*track_, where_, tolerance_);

  distance_ += progress(previous_s_, where_.s, track_->length(),
                        track_->is_closed());
  previous_s_ = where_.s;

  if (!limits_.inside) has_left_ = true;
  worst_margin_ = std::fmin(worst_margin_, limits_.margin);
}

int LapCounter::laps() const {
  if (!track_->is_closed()) return 0;

  // Floor rather than truncate, so that driving backwards past the start
  // gives -1 and not 0. Truncation would report the car as being on its
  // starting lap while it was behind the line, which is the one case the
  // count exists to distinguish.
  return static_cast<int>(std::floor(distance_ / track_->length()));
}

}  // namespace scene
}  // namespace slipx
