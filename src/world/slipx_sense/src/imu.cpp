// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/sense/imu.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace slipx {
namespace sense {
namespace {

void require(bool ok, const char* message) {
  if (!ok) throw std::invalid_argument(std::string("imu: ") + message);
}

}  // namespace

Imu::Imu(const ImuSpec& spec)
    : spec_(spec),
      accel_bias_x_(spec.accel_bias_x),
      accel_bias_y_(spec.accel_bias_y),
      gyro_bias_z_(spec.gyro_bias_z) {
  require(spec.accel_noise_density >= 0.0,
          "accel_noise_density must not be negative [m/s^2/sqrt(Hz)]");
  require(spec.gyro_noise_density >= 0.0,
          "gyro_noise_density must not be negative [rad/s/sqrt(Hz)]");
  require(spec.accel_bias_walk >= 0.0,
          "accel_bias_walk must not be negative [m/s^2/sqrt(s)]");
  require(spec.gyro_bias_walk >= 0.0,
          "gyro_bias_walk must not be negative [rad/s/sqrt(s)]");
  require(std::isfinite(spec.accel_scale_error),
          "accel_scale_error must be finite [-]");
  require(std::isfinite(spec.gyro_scale_error),
          "gyro_scale_error must be finite [-]");
  require(std::isfinite(spec.accel_bias_x) && std::isfinite(spec.accel_bias_y) &&
              std::isfinite(spec.gyro_bias_z),
          "the starting biases must be finite");
}

ImuSample Imu::sample(double time, double dt, double true_ax, double true_ay,
                      double true_yaw_rate, Rng& rng) {
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument(
        "imu: dt must be a positive number of seconds. It is the sensor's own "
        "sample interval, not the simulation step: the noise integrates over "
        "it, so passing the wrong one understates the noise by the square "
        "root of the ratio.");
  }

  const double root_dt = std::sqrt(dt);

  // The walk first, so that this sample sees the bias it walked to. Which
  // order is right is arguable; what matters is that it is fixed, because a
  // bias applied before or after its own update is a different trajectory.
  accel_bias_x_ += spec_.accel_bias_walk * root_dt * rng.normal();
  accel_bias_y_ += spec_.accel_bias_walk * root_dt * rng.normal();
  gyro_bias_z_ += spec_.gyro_bias_walk * root_dt * rng.normal();

  // Densities are per root hertz, so the per-sample sigma grows as the
  // interval shrinks. Sampling faster gives noisier samples and the same
  // answer after averaging, which is what a real unit does.
  const double accel_sigma =
      dt > 0.0 ? spec_.accel_noise_density / root_dt : 0.0;
  const double gyro_sigma = dt > 0.0 ? spec_.gyro_noise_density / root_dt : 0.0;

  ImuSample out;
  out.time = time;

  const double accel_scale = 1.0 + spec_.accel_scale_error;
  const double gyro_scale = 1.0 + spec_.gyro_scale_error;

  out.ax = true_ax * accel_scale + accel_bias_x_ + accel_sigma * rng.normal();
  out.ay = true_ay * accel_scale + accel_bias_y_ + accel_sigma * rng.normal();

  // A planar model has no vertical acceleration, so the proper acceleration
  // on this axis is gravity alone. It carries the scale error and its own
  // noise, and no bias: the two horizontal biases are the ones a planar run
  // can observe, and inventing a third would be a parameter nothing here
  // could ever identify.
  out.az = kStandardGravity * accel_scale + accel_sigma * rng.normal();

  out.yaw_rate =
      true_yaw_rate * gyro_scale + gyro_bias_z_ + gyro_sigma * rng.normal();

  return out;
}

}  // namespace sense
}  // namespace slipx
