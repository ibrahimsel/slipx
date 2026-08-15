// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The IMU and the wheel encoders.
//
// The encoder cases run against a real L2 car rather than against invented
// wheel speeds. That matters: the claim being tested is that odometry
// diverges from the ground exactly when the tyre slips, and the only way to
// test it honestly is to let the tyre model decide when that is and then
// check the odometry against the slip ratios it reported.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "slipx/integrator.hpp"
#include "slipx/sense/encoder.hpp"
#include "slipx/sense/imu.hpp"
#include "slipx/sense/rng.hpp"
#include "slipx/vehicle_model.hpp"

namespace {

using slipx::DriveInput;
using slipx::StepDiagnostics;
using slipx::Tier;
using slipx::VehicleModel;
using slipx::VehicleParams;
using slipx::VehicleState;
using slipx::sense::EncoderSpec;
using slipx::sense::Imu;
using slipx::sense::ImuSample;
using slipx::sense::ImuSpec;
using slipx::sense::kStandardGravity;
using slipx::sense::Rng;
using slipx::sense::WheelOdometry;

ImuSpec silent_imu() {
  ImuSpec spec;
  spec.accel_noise_density = 0.0;
  spec.gyro_noise_density = 0.0;
  spec.accel_bias_walk = 0.0;
  spec.gyro_bias_walk = 0.0;
  return spec;
}

// ------------------------------------------------------------------ the IMU

TEST(Imu, APerfectUnitReportsTheTruthAndGravity) {
  Imu imu(silent_imu());
  Rng rng(1);

  const ImuSample sample = imu.sample(0.0, 0.005, 3.0, -1.5, 0.8, rng);

  EXPECT_DOUBLE_EQ(sample.ax, 3.0);
  EXPECT_DOUBLE_EQ(sample.ay, -1.5);
  EXPECT_DOUBLE_EQ(sample.yaw_rate, 0.8);
  // A planar model has no vertical acceleration, so the proper acceleration
  // on this axis is gravity alone.
  EXPECT_DOUBLE_EQ(sample.az, kStandardGravity);
}

// The error averaging does not remove. A 2 per cent scale error turns a
// 90 degree turn into 91.8, however long the gyro is integrated for and
// however many samples are averaged.
TEST(Imu, AScaleErrorIsNotRemovedByAveraging) {
  ImuSpec spec = silent_imu();
  spec.gyro_scale_error = 0.02;
  Imu imu(spec);
  Rng rng(1);

  const double dt = 0.005;
  const double true_rate = 1.0;  // rad/s
  double integrated = 0.0;
  for (int i = 0; i < 2000; ++i) {  // ten seconds
    integrated += imu.sample(i * dt, dt, 0.0, 0.0, true_rate, rng).yaw_rate * dt;
  }

  EXPECT_NEAR(integrated, 10.0 * 1.02, 1e-9);
}

TEST(Imu, TheAccelerometerCarriesItsOwnScaleError) {
  ImuSpec spec = silent_imu();
  spec.accel_scale_error = 0.02;
  Imu imu(spec);
  Rng rng(1);

  const ImuSample sample = imu.sample(0.0, 0.005, 5.0, -2.0, 0.0, rng);

  EXPECT_DOUBLE_EQ(sample.ax, 5.0 * 1.02);
  EXPECT_DOUBLE_EQ(sample.ay, -2.0 * 1.02);
  // Gravity is measured by the same imperfect axis, so a unit reading 2 per
  // cent high reads 2 per cent high on the one signal a stack might have
  // used to calibrate it.
  EXPECT_DOUBLE_EQ(sample.az, kStandardGravity * 1.02);
}

TEST(Imu, NoiseDensityScalesWithTheSampleInterval) {
  // The property that makes a density a density: sampling faster gives
  // noisier samples, and the same answer after averaging. A per-sample sigma
  // would give the same noise at every rate, which is wrong in the direction
  // that flatters a fast sensor.
  ImuSpec spec = silent_imu();
  spec.gyro_noise_density = 0.01;
  const int samples = 20000;

  const auto spread_at = [&](double dt) {
    Imu imu(spec);
    Rng rng(4);
    double sum_squares = 0.0;
    for (int i = 0; i < samples; ++i) {
      const double error = imu.sample(i * dt, dt, 0.0, 0.0, 0.0, rng).yaw_rate;
      sum_squares += error * error;
    }
    return std::sqrt(sum_squares / samples);
  };

  const double slow = spread_at(0.01);   // 100 Hz
  const double fast = spread_at(0.0025);  // 400 Hz

  EXPECT_NEAR(slow, 0.01 / std::sqrt(0.01), 0.01 * 0.1);
  EXPECT_NEAR(fast, 0.01 / std::sqrt(0.0025), 0.02 * 0.1);
  EXPECT_GT(fast, slow * 1.5) << "four times the rate, twice the sample noise";
}

// The reason dead reckoning drifts rather than merely being noisy. White
// noise averages away over a long run; a random walk does not, and grows as
// the square root of time.
TEST(Imu, TheBiasWalksAndDoesNotAverageAway) {
  ImuSpec spec = silent_imu();
  spec.gyro_bias_walk = 0.01;
  Imu imu(spec);
  Rng rng(2);

  const double dt = 0.005;
  std::vector<double> biases;
  for (int i = 0; i < 4000; ++i) {
    imu.sample(i * dt, dt, 0.0, 0.0, 0.0, rng);
    biases.push_back(imu.gyro_bias_z());
  }

  EXPECT_NE(biases.front(), biases.back());
  // Twenty seconds of a 0.01 rad/s per root second walk gives a standard
  // deviation of about 0.045 rad/s, so ending within a hair of zero would
  // mean the walk was not walking.
  EXPECT_GT(std::fabs(biases.back()), 0.005);

  // And the bias is in the output, not just in the object.
  const ImuSample sample = imu.sample(20.0, dt, 0.0, 0.0, 0.0, rng);
  EXPECT_NEAR(sample.yaw_rate, imu.gyro_bias_z(), 1e-12);
}

TEST(Imu, IsSeededAndReproducible) {
  ImuSpec spec;
  spec.gyro_noise_density = 0.01;
  spec.gyro_bias_walk = 0.001;

  Imu one(spec), two(spec), three(spec);
  Rng a(9), b(9), c(10);

  for (int i = 0; i < 100; ++i) {
    EXPECT_DOUBLE_EQ(one.sample(i * 0.01, 0.01, 1.0, 0.0, 0.5, a).yaw_rate,
                     two.sample(i * 0.01, 0.01, 1.0, 0.0, 0.5, b).yaw_rate);
  }
  const double different =
      three.sample(0.0, 0.01, 1.0, 0.0, 0.5, c).yaw_rate;
  EXPECT_NE(different, one.sample(0.0, 0.01, 1.0, 0.0, 0.5, a).yaw_rate);
}

TEST(Imu, RefusesASpecItCannotHonour) {
  EXPECT_THROW(
      {
        ImuSpec spec;
        spec.accel_noise_density = -1.0;
        Imu bad(spec);
      },
      std::invalid_argument);

  Imu imu{silent_imu()};
  Rng rng(1);
  EXPECT_THROW(imu.sample(0.0, 0.0, 0.0, 0.0, 0.0, rng), std::invalid_argument);
  EXPECT_THROW(imu.sample(0.0, -0.01, 0.0, 0.0, 0.0, rng),
               std::invalid_argument);
}

// -------------------------------------------------------------- the encoders

TEST(Encoder, CountsQuantiseAndLoseNothingOverTime) {
  EncoderSpec spec;
  spec.counts_per_revolution = 4.0;  // very coarse, so quantisation is visible
  WheelOdometry odometry(spec);

  VehicleState state;
  for (auto& omega : state.omega_w) omega = 1.0;  // rad/s

  // A quarter of a count per sample at this rate, so a per-sample
  // quantisation would report zero for ever.
  const double dt = 0.1;
  for (int i = 0; i < 100; ++i) odometry.sample(i * dt, dt, state);

  // Ten seconds at 1 rad/s is 10 rad, which is 10/(2 pi) * 4 = 6.37 counts.
  EXPECT_EQ(odometry.spec().counts_per_revolution, 4.0);
  const VehicleState& s = state;
  const auto final_sample = odometry.sample(10.0, dt, s);
  EXPECT_EQ(final_sample.counts[0], 6);
}

// Which wheels are measured changes the answer, and on a car whose wheels are
// turning at different speeds it changes it a lot. A rear-wheel-drive car
// with a motor encoder measures the driven wheels, which are the ones that
// slip, so it has the worst odometry available to it under acceleration.
// A count is the number of edges the wheel has crossed, and the sign of the
// rotation must not change how many that is. A wheel turned backwards through
// 6.37 counts has crossed six edges, not seven; flooring reports seven, an
// edge that never went past, and the two answers agree for a wheel that only
// ever goes forwards, which is how the error survives a test suite.
TEST(Encoder, CountsGoingBackwardsAreTheEdgesActuallyCrossed) {
  EncoderSpec spec;
  spec.counts_per_revolution = 4.0;
  WheelOdometry forwards(spec), backwards(spec);

  VehicleState going_forwards;
  for (auto& omega : going_forwards.omega_w) omega = 1.0;
  VehicleState going_backwards;
  for (auto& omega : going_backwards.omega_w) omega = -1.0;

  // Ten seconds either way is 10 rad, which is 6.37 counts.
  const double dt = 0.1;
  for (int i = 0; i < 100; ++i) {
    forwards.sample(i * dt, dt, going_forwards);
    backwards.sample(i * dt, dt, going_backwards);
  }

  const auto ahead = forwards.sample(10.0, 1e-9, going_forwards);
  const auto behind = backwards.sample(10.0, 1e-9, going_backwards);

  EXPECT_EQ(ahead.counts[0], 6);
  EXPECT_EQ(behind.counts[0], -6) << "the same six edges, the other way";
}

TEST(Encoder, OnlyTheSelectedWheelsCountTowardsOdometry) {
  VehicleState state;
  state.omega_w = {{10.0, 10.0, 30.0, 30.0}};  // rear wheels spinning up

  EncoderSpec all;
  all.counts_per_revolution = 100000.0;
  EncoderSpec fronts = all;
  fronts.wheels_used = {{true, true, false, false}};
  EncoderSpec rears = all;
  rears.wheels_used = {{false, false, true, true}};

  WheelOdometry every(all), front(fronts), rear(rears);
  every.sample(0.0, 1.0, state);
  front.sample(0.0, 1.0, state);
  rear.sample(0.0, 1.0, state);

  EXPECT_NEAR(front.distance(), 10.0 * all.wheel_radius, 1e-3);
  EXPECT_NEAR(rear.distance(), 30.0 * all.wheel_radius, 1e-3);
  EXPECT_NEAR(every.distance(), 20.0 * all.wheel_radius, 1e-3);
}

TEST(Encoder, RefusesASelectionWithNoWheelsInIt) {
  EncoderSpec spec;
  spec.wheels_used = {{false, false, false, false}};
  EXPECT_THROW(WheelOdometry{spec}, std::invalid_argument);

  spec.wheels_used = {{true, true, true, true}};
  spec.wheel_radius = 0.0;
  EXPECT_THROW(WheelOdometry{spec}, std::invalid_argument);
}

// ------------------------------------------- odometry against a real L2 car
//
// The case M5.5 exists for. The car is driven hard enough to slip and then
// allowed to settle, and the odometry is compared against the ground truth
// the model integrated. The assertion is not "odometry is wrong", it is
// "odometry is wrong exactly while the diagnostics report slip".

namespace {

struct Sampled {
  double true_distance = 0.0;
  double odometry_distance = 0.0;
  double peak_kappa = 0.0;
};

// Drive an L2 car at a fixed acceleration demand for a while, tracking the
// ground truth, the odometry and the largest slip ratio seen.
Sampled drive(double accel_cmd, double seconds) {
  const VehicleParams params;  // the struct defaults are a complete L2 car
  const auto model = VehicleModel::create(Tier::L2_DoubleTrack, params);

  EncoderSpec spec;
  spec.counts_per_revolution = 4000.0;  // fine, so quantisation is not the story
  spec.wheel_radius = params.wheel_radius;
  WheelOdometry odometry(spec);

  VehicleState state;
  state.vel_body.x = 2.0;  // rolling, so the slip ratio is well posed
  for (auto& omega : state.omega_w) omega = state.vel_body.x / params.wheel_radius;

  StepDiagnostics diagnostics;
  const double dt = 1.0e-3;
  const int steps = static_cast<int>(seconds / dt);

  Sampled out;
  for (int i = 0; i < steps; ++i) {
    const DriveInput input{0.0, accel_cmd};
    const double before = state.pos.x;
    model->step(state, input, dt, &diagnostics);
    out.true_distance += state.pos.x - before;

    odometry.sample(i * dt, dt, state);

    for (double kappa : diagnostics.kappa) {
      if (std::isfinite(kappa)) {
        out.peak_kappa = std::fmax(out.peak_kappa, std::fabs(kappa));
      }
    }
  }
  out.odometry_distance = odometry.distance();
  return out;
}

}  // namespace

TEST(Encoder, OdometryTracksTheGroundWhenTheTyreIsNotSlipping) {
  // A gentle demand: the tyre carries it with a slip ratio near zero, so the
  // wheels turn at very nearly the speed the ground goes past.
  const Sampled coasting = drive(0.05, 2.0);

  ASSERT_LT(coasting.peak_kappa, 0.01) << "this case is meant to be slip-free";

  const double error = std::fabs(coasting.odometry_distance -
                                 coasting.true_distance);
  EXPECT_LT(error / coasting.true_distance, 0.01)
      << "without slip, odometry is the truth to within quantisation";
}

TEST(Encoder, OdometryOverreadsExactlyWhenTheDrivenWheelsSlip) {
  // Full demand: the driven wheels spin up, the slip ratio goes positive, and
  // the encoders report a car that has travelled further than it has.
  const Sampled launching = drive(8.0, 2.0);

  ASSERT_GT(launching.peak_kappa, 0.02) << "this case is meant to slip";

  EXPECT_GT(launching.odometry_distance, launching.true_distance)
      << "a driven wheel that slips turns faster than the ground goes past";

  // And the divergence is of the order of the slip, not of the order of a
  // rounding error: the two claims together are what makes this the slip and
  // not a bug somewhere else.
  const double relative = (launching.odometry_distance -
                           launching.true_distance) /
                          launching.true_distance;
  EXPECT_GT(relative, 1e-4);
  EXPECT_LT(relative, launching.peak_kappa * 2.0)
      << "and no larger than the slip could account for";
}

TEST(Encoder, OdometryUnderreadsUnderBraking) {
  // The other sign, which is the one that matters: under braking the wheels
  // turn slower than the ground, so a stack trusting them thinks it has
  // stopped sooner than it has, and brakes later next time.
  const Sampled braking = drive(-8.0, 1.0);

  ASSERT_GT(braking.peak_kappa, 0.02);
  EXPECT_LT(braking.odometry_distance, braking.true_distance);
}

}  // namespace
