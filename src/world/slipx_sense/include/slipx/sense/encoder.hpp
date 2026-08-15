// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Wheel encoders, and the odometry a stack builds out of them.
//
// This is the sensor that lies, and it lies for a reason worth modelling
// rather than adding noise to. An encoder measures how far the WHEEL turned.
// Odometry multiplies that by a radius and reports how far the CAR went, and
// those are the same number only while the tyre is not slipping. Under
// acceleration the driven wheels turn faster than the ground goes past, under
// braking they turn slower, and a locked wheel says the car has stopped while
// it is still travelling at 8 m/s.
//
// So the divergence here is not an error term. It is the slip ratio the tyre
// model already computed, arriving in the odometry because odometry is
// defined in terms of wheel speed and the world is not. A stack that fuses
// encoders with an IMU is being tested against exactly the failure it exists
// to handle, and one that trusts encoders alone will drive into a wall in
// this simulator for the same reason it does on carpet.
//
// Counts are accumulated from wheel angle rather than computed per sample, so
// quantisation loses nothing over time: a wheel turning slowly enough to
// produce less than one count per sample still produces counts at the right
// average rate instead of reading zero forever.

#ifndef SLIPX_SENSE_ENCODER_HPP
#define SLIPX_SENSE_ENCODER_HPP

#include <array>
#include <cstdint>

#include "slipx/state.hpp"

namespace slipx {
namespace sense {

struct EncoderSpec {
  // Counts per full revolution of the wheel, after any gearing and after
  // quadrature decoding. A small hall-sensor motor encoder on a 1/10 car is
  // of the order of tens to a few hundred counts per wheel revolution, and
  // the quantisation is visible at low speed.
  double counts_per_revolution = 120.0;

  // The radius odometry multiplies by. Deliberately separate from the
  // vehicle's own wheel_radius: a stack uses the radius somebody typed into a
  // config file, and typing the wrong one is a common and instructive
  // failure, so the sensor is allowed to disagree with the car.
  double wheel_radius = 0.05;  // [m]

  // Which wheels the odometry averages. Front-left, front-rear ordering
  // follows the core's wheel indexing. A rear-wheel-drive car with a motor
  // encoder measures the driven wheels and therefore has the worst possible
  // odometry under acceleration, which is the default worth having visible.
  std::array<bool, kWheelCount> wheels_used{{true, true, true, true}};
};

struct EncoderSample {
  double time = 0.0;  // [s]

  // Cumulative counts per wheel, signed, as a real encoder's register would
  // read if it were wide enough not to wrap.
  std::array<std::int64_t, kWheelCount> counts{};

  // Wheel speeds as the counts imply over the last interval, which is what a
  // driver publishes and is not the same as the true wheel speed: it is
  // quantised.                                                     [rad/s]
  std::array<double, kWheelCount> wheel_speed{};

  // The odometry. Distance is cumulative since the encoder was created and
  // speed is the estimate over the last interval. Both are what the encoders
  // believe, not what happened.
  double distance = 0.0;  // [m]
  double speed = 0.0;     // [m/s]
};

class WheelOdometry {
 public:
  // Throws std::invalid_argument, naming the field, for a non-positive counts
  // per revolution or wheel radius, or a selection with no wheels in it: an
  // odometry averaging nothing is not an odometry.
  explicit WheelOdometry(const EncoderSpec& spec);

  const EncoderSpec& spec() const { return spec_; }

  // Advance by `dt` seconds of the wheel speeds in `state`, and report what
  // the encoders say.
  //
  // Noiseless on purpose. An encoder is a digital device: it does not have a
  // noise floor, it has a quantisation, and the interesting error in it is
  // the slip, which is already in the wheel speeds this is given. Adding a
  // gaussian here would bury the effect the sensor exists to demonstrate
  // under a number somebody chose.
  //
  // Throws std::invalid_argument if `dt` is not positive.
  EncoderSample sample(double time, double dt, const VehicleState& state);

  // Cumulative odometry distance.                                       [m]
  double distance() const { return distance_; }

 private:
  EncoderSpec spec_;
  std::array<double, kWheelCount> angle_{};          // [rad], accumulated
  std::array<std::int64_t, kWheelCount> counts_{};
  double distance_ = 0.0;
};

}  // namespace sense
}  // namespace slipx

#endif  // SLIPX_SENSE_ENCODER_HPP
