// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// CORE-17: the sign conventions documented in conventions.hpp, asserted.
//
// This file is the reason conventions.hpp can be called normative. Sign errors
// are the dominant bug class in vehicle dynamics code and they are invisible
// in a plot that looks plausible, so each documented claim gets an executable
// counterpart here. If a future change makes one of these fail, the change is
// wrong, or conventions.hpp needs rewriting and every consumer needs telling.

#include <gtest/gtest.h>

#include <cmath>

#include "slipx/vehicle_model.hpp"
#include "test_support.hpp"

namespace {

using namespace slipx_test;
using slipx::DriveInput;
using slipx::Integrator;
using slipx::StepDiagnostics;
using slipx::Tier;
using slipx::VehicleModel;
using slipx::VehicleState;

constexpr double kDt = kDefaultDt;

class Conventions : public ::testing::TestWithParam<Tier> {};

// ISO 8855: yaw is positive counter-clockwise, and positive steer is left.
// A left turn must therefore increase yaw, increase world y, and produce
// positive lateral acceleration. All three at once: getting one right by
// accident is easy, getting three right by accident is not.
TEST_P(Conventions, PositiveSteerTurnsLeft) {
  auto model = VehicleModel::create(GetParam(), reference_params());
  VehicleState s = travelling(4.0);

  const StepDiagnostics d = run_for(
      *model, s, 0.5, kDt, [](const VehicleState& st, double) {
        return DriveInput{0.10, hold_speed(st, 4.0)};
      });

  EXPECT_GT(s.yaw, 0.0) << "positive steer must increase yaw";
  EXPECT_GT(s.pos.y, 0.0) << "a left turn must move the car to world +y";
  EXPECT_GT(s.yaw_rate(), 0.0) << "yaw rate must be positive in a left turn";
  EXPECT_GT(d.ay, 0.0) << "lateral acceleration is positive to the left";
  EXPECT_GT(s.pos.x, 0.0) << "the car is still going forwards";
}

TEST_P(Conventions, NegativeSteerTurnsRight) {
  auto model = VehicleModel::create(GetParam(), reference_params());
  VehicleState s = travelling(4.0);

  const StepDiagnostics d = run_for(
      *model, s, 0.5, kDt, [](const VehicleState& st, double) {
        return DriveInput{-0.10, hold_speed(st, 4.0)};
      });

  EXPECT_LT(s.yaw, 0.0);
  EXPECT_LT(s.pos.y, 0.0);
  EXPECT_LT(s.yaw_rate(), 0.0);
  EXPECT_LT(d.ay, 0.0);
}

TEST_P(Conventions, BodyFrameXIsForward) {
  auto model = VehicleModel::create(GetParam(), reference_params());
  VehicleState s = at_rest();
  s.yaw = slipx::kPi / 2.0;  // pointing along world +y

  run_for(*model, s, 1.0, kDt,
          [](const VehicleState&, double) { return DriveInput{0.0, 2.0}; });

  EXPECT_GT(s.vel_body.x, 0.0) << "accelerating forward is +x in the body";
  EXPECT_GT(s.pos.y, 0.0) << "a car pointing along world +y goes that way";
  EXPECT_NEAR(s.pos.x, 0.0, 1e-9) << "and does not drift sideways in world x";
}

TEST_P(Conventions, SteerCommandIsClippedToTravelAndReported) {
  auto p = reference_params();
  p.steer_max = 0.30;
  auto model = VehicleModel::create(GetParam(), p);

  VehicleState s = travelling(3.0);
  StepDiagnostics d;
  model->step(s, DriveInput{0.9, 0.0}, kDt, &d);

  EXPECT_DOUBLE_EQ(s.steer, 0.30) << "achieved angle is the clipped one";
  EXPECT_TRUE(d.steer_saturated) << "and the caller is told it was clipped";

  model->step(s, DriveInput{0.10, 0.0}, kDt, &d);
  EXPECT_DOUBLE_EQ(s.steer, 0.10);
  EXPECT_FALSE(d.steer_saturated);
}

TEST_P(Conventions, DiagnosticsReportTheTierThatProducedThem) {
  auto model = VehicleModel::create(GetParam(), reference_params());
  VehicleState s = travelling(3.0);
  StepDiagnostics d;
  model->step(s, DriveInput{0.05, 0.0}, kDt, &d);
  EXPECT_EQ(d.tier, static_cast<int>(GetParam()));
}

INSTANTIATE_TEST_SUITE_P(AllTiers, Conventions,
                         ::testing::Values(Tier::L0_Kinematic,
                                           Tier::L1_Bicycle));

// ---------------------------------------------------------------- L1 only

// The ISO minus sign, in isolation: a car sliding to its left with the wheels
// straight ahead has a positive rear slip angle and a lateral force pointing
// right, which is what pulls it back.
TEST(ConventionsL1, PositiveSlipAngleGivesNegativeLateralForce) {
  auto model = VehicleModel::create(Tier::L1_Bicycle, reference_params());

  VehicleState s = travelling(5.0);
  s.vel_body.y = 0.5;  // sliding to the left

  slipx::StepDiagnostics d;
  model->step(s, DriveInput{0.0, 0.0}, kDt, &d);

  EXPECT_GT(d.alpha_rear, 0.0) << "velocity left of the wheel plane is +alpha";
  EXPECT_LT(d.fy_rear, 0.0) << "and the force opposes it: ISO, not SAE";
  EXPECT_GT(d.alpha_front, 0.0);
  EXPECT_LT(d.fy_front, 0.0);

  // Magnitude, not just sign: the linear region is Fy = -C_alpha * alpha.
  EXPECT_NEAR(d.fy_rear, -reference_params().c_alpha_r * d.alpha_rear, 1e-9);
}

TEST(ConventionsL1, SteerAngleEntersTheFrontSlipAngleWithAMinusSign) {
  auto model = VehicleModel::create(Tier::L1_Bicycle, reference_params());

  VehicleState s = travelling(5.0);
  slipx::StepDiagnostics d;
  model->step(s, DriveInput{0.08, 0.0}, kDt, &d);

  // alpha_f = atan2(vy + lf r, vx) - delta. Straight-line entry means the
  // first term is nearly zero after one millisecond, so alpha_f ~ -delta and
  // the resulting force is to the left, which starts the left turn.
  EXPECT_LT(d.alpha_front, 0.0);
  EXPECT_NEAR(d.alpha_front, -0.08, 5e-3);
  EXPECT_GT(d.fy_front, 0.0);
}

TEST(ConventionsL1, SideslipIsPositiveWhenVelocityIsToTheLeft) {
  VehicleState s = travelling(5.0);
  s.vel_body.y = 1.0;
  EXPECT_GT(s.sideslip(), 0.0);
  EXPECT_NEAR(s.sideslip(), std::atan2(1.0, 5.0), 1e-15);

  s.vel_body.y = -1.0;
  EXPECT_LT(s.sideslip(), 0.0);
}

// SI, not degrees: a car asked for 0.2 of something must turn 0.2 radians,
// and a test that only ever checks signs would not notice a factor of 57.
TEST(ConventionsL1, AnglesAreRadians) {
  auto model = VehicleModel::create(Tier::L1_Bicycle, reference_params());
  VehicleState s = travelling(5.0);
  model->step(s, DriveInput{0.2, 0.0}, kDt, nullptr);
  EXPECT_DOUBLE_EQ(s.steer, 0.2);
}

// A tier reports NaN for what it cannot represent, never zero (state.hpp).
TEST(ConventionsL0, UnrepresentableQuantitiesAreNaN) {
  auto model = VehicleModel::create(Tier::L0_Kinematic, reference_params());
  VehicleState s = travelling(4.0);
  slipx::StepDiagnostics d;
  model->step(s, DriveInput{0.1, 0.0}, kDt, &d);

  EXPECT_TRUE(std::isnan(d.alpha_front)) << "L0 has no tyres to slip";
  EXPECT_TRUE(std::isnan(d.alpha_rear));
  EXPECT_TRUE(std::isnan(d.fy_front));
  EXPECT_TRUE(std::isnan(d.fz_front));
  EXPECT_TRUE(std::isnan(d.load_transfer_long));
  EXPECT_TRUE(std::isnan(d.alpha[slipx::kFrontLeft]));

  // What it can represent is a number.
  EXPECT_FALSE(std::isnan(d.ax));
  EXPECT_FALSE(std::isnan(d.ay));
}

TEST(ConventionsL1, LoadTransferIsNaNBecauseL1CannotTransferLoad) {
  auto model = VehicleModel::create(Tier::L1_Bicycle, reference_params());
  VehicleState s = travelling(4.0);
  slipx::StepDiagnostics d;
  model->step(s, DriveInput{0.1, 3.0}, kDt, &d);

  EXPECT_TRUE(std::isnan(d.load_transfer_long)) << "CORE-05 arrives at L2";
  EXPECT_TRUE(std::isnan(d.load_transfer_lat));

  // Axle loads at L1 are the static ones and are reported as such.
  const auto p = reference_params();
  EXPECT_NEAR(d.fz_front, p.mass * slipx::kGravity * p.lr / p.wheelbase(),
              1e-12);
  EXPECT_NEAR(d.fz_rear, p.mass * slipx::kGravity * p.lf / p.wheelbase(),
              1e-12);
  EXPECT_NEAR(d.fz_front + d.fz_rear, p.mass * slipx::kGravity, 1e-12);
}

// Per-wheel float diagnostics stay NaN at a single-track tier rather than
// duplicating an axle value that was never computed per corner.
TEST(ConventionsL1, PerWheelSlipIsNaNAtASingleTrackTier) {
  auto model = VehicleModel::create(Tier::L1_Bicycle, reference_params());
  VehicleState s = travelling(4.0);
  slipx::StepDiagnostics d;
  model->step(s, DriveInput{0.1, 0.0}, kDt, &d);

  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    EXPECT_TRUE(std::isnan(d.alpha[i])) << "wheel " << i;
    EXPECT_TRUE(std::isnan(d.fy[i])) << "wheel " << i;
    EXPECT_TRUE(std::isnan(d.fz[i])) << "wheel " << i;
  }
  EXPECT_FALSE(std::isnan(d.alpha_front));
}

// Wheel ordering is fixed library-wide; an off-by-one here would silently swap
// left and right in every consumer.
TEST(Conventions, WheelIndexOrdering) {
  EXPECT_EQ(slipx::kFrontLeft, 0u);
  EXPECT_EQ(slipx::kFrontRight, 1u);
  EXPECT_EQ(slipx::kRearLeft, 2u);
  EXPECT_EQ(slipx::kRearRight, 3u);
  EXPECT_EQ(slipx::kWheelCount, 4u);
}

}  // namespace
