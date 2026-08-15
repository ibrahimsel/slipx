// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/sense/encoder.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace slipx {
namespace sense {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

void require(bool ok, const char* message) {
  if (!ok) throw std::invalid_argument(std::string("encoder: ") + message);
}

}  // namespace

WheelOdometry::WheelOdometry(const EncoderSpec& spec) : spec_(spec) {
  require(std::isfinite(spec.counts_per_revolution) &&
              spec.counts_per_revolution > 0.0,
          "counts_per_revolution must be positive [-]");
  require(std::isfinite(spec.wheel_radius) && spec.wheel_radius > 0.0,
          "wheel_radius must be positive [m]");

  bool any = false;
  for (bool used : spec.wheels_used) any = any || used;
  require(any,
          "wheels_used selects no wheel. An odometry averaging nothing is not "
          "an odometry, and a zero it reported would look like a stationary "
          "car.");
}

EncoderSample WheelOdometry::sample(double time, double dt,
                                    const VehicleState& state) {
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument(
        "encoder: dt must be a positive number of seconds.");
  }

  EncoderSample out;
  out.time = time;

  const double counts_per_radian = spec_.counts_per_revolution / kTwoPi;

  double sum_of_speeds = 0.0;
  double sum_of_distance = 0.0;
  int used = 0;

  for (std::size_t i = 0; i < kWheelCount; ++i) {
    // The wheel angle is accumulated first and quantised second. Doing it the
    // other way, by quantising each interval's rotation on its own, throws
    // away the remainder every sample: a wheel turning slowly enough to give
    // less than one count per interval would read exactly zero forever
    // instead of ticking over at the right average rate.
    angle_[i] += state.omega_w[i] * dt;

    // Truncation towards zero, not floor. The count is the number of edges
    // the wheel has crossed, signed, and a wheel that has turned backwards
    // through 6.37 counts has crossed six of them: floor would report seven,
    // an edge that never went past. The two agree for a wheel that only ever
    // goes forwards, which is why the error is easy to ship.
    const std::int64_t total =
        static_cast<std::int64_t>(angle_[i] * counts_per_radian);
    const std::int64_t delta = total - counts_[i];
    counts_[i] = total;
    out.counts[i] = total;

    const double turned = static_cast<double>(delta) / counts_per_radian;
    out.wheel_speed[i] = turned / dt;

    if (spec_.wheels_used[i]) {
      sum_of_speeds += out.wheel_speed[i];
      sum_of_distance += turned * spec_.wheel_radius;
      ++used;
    }
  }

  const double divisor = static_cast<double>(used);
  out.speed = sum_of_speeds / divisor * spec_.wheel_radius;

  // The whole model, in one line: distance is what the wheels turned times a
  // radius. Nothing here knows how fast the car was actually going, which is
  // exactly why this diverges under slip, and it diverges by the slip ratio
  // rather than by a number anybody chose.
  distance_ += sum_of_distance / divisor;
  out.distance = distance_;

  return out;
}

}  // namespace sense
}  // namespace slipx
