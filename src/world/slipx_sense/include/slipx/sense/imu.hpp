// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// An inertial measurement unit.
//
// A MEMS IMU on a 1/10-scale car is wrong in three ways at once, and they are
// wrong on different timescales, which is why a single "noise" number does
// not describe one:
//
//   white noise      independent every sample, and the thing an average over
//                    a window removes
//   bias instability a slow random walk that averaging does NOT remove, and
//                    the reason dead reckoning on an IMU drifts rather than
//                    just being noisy
//   scale error      a fixed multiplier, so a rate gyro that reads 2 per cent
//                    low reads 2 per cent low forever and a 90 degree turn
//                    comes back as 88.2
//
// The scale error is a parameter here and not a draw, because it is a
// property of the unit rather than of the run: the same board has the same
// scale error tomorrow, and it is identifiable from a manoeuvre with a known
// total heading change. Making it random would put a number in the model that
// nobody could ever measure, which is the test every parameter in SlipX has
// to pass.
//
// The true accelerations come from StepDiagnostics::ax and ay, which are
// already specific forces at the CoG including the transport terms, and are
// therefore what an ideal accelerometer would read rather than the time
// derivatives of the body velocities.

#ifndef SLIPX_SENSE_IMU_HPP
#define SLIPX_SENSE_IMU_HPP

#include "slipx/sense/rng.hpp"

namespace slipx {
namespace sense {

// Standard gravity. The value, not a measurement: a planar model has no
// vertical dynamics, so a level car's z accelerometer reads exactly this.
constexpr double kStandardGravity = 9.80665;  // [m/s^2]

struct ImuSpec {
  // Continuous-time noise densities. Divided by the square root of the
  // sample interval to get a per-sample standard deviation, which is why they
  // are stated per root hertz: sampling faster gives noisier samples and the
  // same noise after averaging, which is the behaviour a real unit has and
  // the one a naive per-sample sigma gets wrong.
  double accel_noise_density = 0.002;  // [m/s^2 / sqrt(Hz)]
  double gyro_noise_density = 0.0003;  // [rad/s / sqrt(Hz)]

  // Bias random walk. Multiplied by the square root of the sample interval,
  // the other way round from the noise above, because a walk accumulates.
  double accel_bias_walk = 0.0001;  // [m/s^2 / sqrt(s)]
  double gyro_bias_walk = 0.00002;  // [rad/s / sqrt(s)]

  // Fixed multiplicative errors. 0.01 means the unit reads 1 per cent high.
  double accel_scale_error = 0.0;  // [-]
  double gyro_scale_error = 0.0;   // [-]

  // Starting biases. Zero is a legitimate value and an unusual one: a real
  // unit that has not been calibrated starts somewhere, and a stack that
  // estimates bias should be given something to find.
  double accel_bias_x = 0.0;  // [m/s^2]
  double accel_bias_y = 0.0;  // [m/s^2]
  double gyro_bias_z = 0.0;   // [rad/s]
};

struct ImuSample {
  double time = 0.0;  // [s]

  // Specific force in the body frame, x forward and y left.
  double ax = 0.0;  // [m/s^2]
  double ay = 0.0;  // [m/s^2]

  // A level car under a planar model has no vertical acceleration, so this
  // reads standard gravity plus the unit's own errors. It is reported rather
  // than omitted because a driver that expects three axes and gets two is a
  // driver that breaks, and because a stack estimating attitude from gravity
  // needs it.
  double az = 0.0;  // [m/s^2]

  double yaw_rate = 0.0;  // [rad/s]
};

// One IMU, with its own drifting biases. Per agent, and stateful: the bias
// walk is the whole reason this is an object rather than a function.
class Imu {
 public:
  // Throws std::invalid_argument, naming the field, for a negative noise
  // density or walk, or a non-finite scale error.
  explicit Imu(const ImuSpec& spec);

  const ImuSpec& spec() const { return spec_; }

  // Take a sample of the true motion over an interval of `dt` seconds.
  //
  // `dt` is the sensor's own sample interval, not the simulation step: an IMU
  // running at 200 Hz inside a 1 kHz simulation integrates its noise over
  // 5 ms, and passing the simulation step here would understate the noise by
  // the square root of the ratio.
  //
  // Throws std::invalid_argument if `dt` is not positive.
  ImuSample sample(double time, double dt, double true_ax, double true_ay,
                   double true_yaw_rate, Rng& rng);

  // The biases as they stand, which no consumer of the data can see. Exposed
  // so that a test can assert the walk walks, and so that a validation report
  // can plot the error a filter was trying to estimate.
  double accel_bias_x() const { return accel_bias_x_; }
  double accel_bias_y() const { return accel_bias_y_; }
  double gyro_bias_z() const { return gyro_bias_z_; }

 private:
  ImuSpec spec_;
  double accel_bias_x_ = 0.0;
  double accel_bias_y_ = 0.0;
  double gyro_bias_z_ = 0.0;
};

}  // namespace sense
}  // namespace slipx

#endif  // SLIPX_SENSE_IMU_HPP
