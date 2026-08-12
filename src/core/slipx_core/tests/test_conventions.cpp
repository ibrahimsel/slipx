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

#include "slipx/relaxation.hpp"
#include "slipx/tyre.hpp"
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
  const VehicleParams p = reference_params();
  auto model = VehicleModel::create(Tier::L1_Bicycle, p);

  VehicleState s = travelling(5.0);
  // Sliding to the left, gently. The magnitude assertion below is about the
  // linear law, so the operating point has to stay inside L1's mu_clip; the
  // sideslip that did so at the old, much softer cornering stiffness now
  // saturates the axle and would test the clip instead (ADR-0032).
  s.vel_body.y = 0.15;

  slipx::StepDiagnostics d;
  model->step(s, DriveInput{0.0, 0.0}, kDt, &d);

  EXPECT_GT(d.alpha_rear, 0.0) << "velocity left of the wheel plane is +alpha";
  EXPECT_LT(d.fy_rear, 0.0) << "and the force opposes it: ISO, not SAE";
  EXPECT_GT(d.alpha_front, 0.0);
  EXPECT_LT(d.fy_front, 0.0);

  // The clip is a different mechanism and this case is not about it. Asserted
  // rather than assumed, so a future parameter change cannot quietly turn the
  // magnitude check below into a tautology about the clip.
  ASSERT_LT(std::fabs(p.c_alpha_r * d.alpha_rear),
            p.mu_clip * 0.5 * p.mass * slipx::kGravity)
      << "the linear force must be inside mu_clip for this case to mean "
         "anything";

  // Magnitude, not just sign: the linear region is Fy = -C_alpha * alpha.
  EXPECT_NEAR(d.fy_rear, -p.c_alpha_r * d.alpha_rear, 1e-9);
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

// The ISO restoring sign, in MF-lite rather than in a model (CORE-06). This is
// the single most common place ISO and SAE get mixed up: under SAE the slip
// angle carries the opposite sign and the same tyre is written
// Fy = +C_alpha * alpha. Both describe a restoring force, and a paper that
// disagrees with this test is almost always an SAE paper.
TEST(ConventionsTyre, MfLiteRestoresAgainstTheSlipAngle) {
  slipx::TyreCoefficients c;
  const slipx::MfLite t = slipx::make_mf_lite(c, 61.0, 8.4);

  // Positive alpha: the wheel's velocity lies to the LEFT of the wheel plane,
  // and the force pushes it back to the right, so Fy is negative.
  EXPECT_LT(slipx::mf_lite_fy(t, 0.05, 8.4), 0.0);
  EXPECT_GT(slipx::mf_lite_fy(t, -0.05, 8.4), 0.0);
  EXPECT_EQ(slipx::mf_lite_fy(t, 0.0, 8.4), 0.0);

  // The same sign L1's linear tyre carries, and the same magnitude at small
  // slip, which is what makes the two tiers describe one car.
  EXPECT_NEAR(slipx::mf_lite_fy(t, 1e-5, 8.4), -61.0 * 1e-5, 1e-9);

  // Past the peak the force falls but never changes sign: the tyre stops
  // resisting harder, it does not start pushing the wrong way.
  for (const double alpha : {0.3, 0.8, 1.5, 3.0}) {
    EXPECT_LT(slipx::mf_lite_fy(t, alpha, 8.4), 0.0) << "at alpha " << alpha;
  }
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

// The transient does not change the sign convention. A positive slip angle
// produces a negative lateral force at steady state (asserted above), and it
// must do so at every instant while the tyre is still building up to it: the
// lagged angle stays on the same side of zero as the angle it is chasing, so
// the force never passes through the wrong sign on its way up.
//
// Worth asserting separately because a lag implemented with the sign of the
// error reversed still converges in magnitude and would pass a steady-state
// check while producing a force that pushed the wrong way for the first few
// milliseconds of every corner.
TEST(ConventionsTyre, TheRelaxationTransientNeverFlipsTheForceSign) {
  const slipx::TyreCoefficients coefficients;
  const slipx::MfLite tyre = slipx::make_mf_lite(coefficients, 61.0, 8.4);
  const double sigma = 0.08;
  const double vx = 8.0;

  for (const double alpha : {0.05, -0.05}) {
    double lagged = 0.0;
    for (int i = 0; i < 400; ++i) {
      lagged += 1e-4 * slipx::relaxation_rate(alpha, lagged, vx, sigma);

      // The lagged angle is on the same side of zero as the target.
      if (alpha > 0.0) {
        EXPECT_GE(lagged, 0.0);
      } else {
        EXPECT_LE(lagged, 0.0);
      }

      // And ISO 8855 holds throughout: positive slip, negative force.
      const double fy = slipx::mf_lite_fy(tyre, lagged, 8.4);
      if (lagged > 0.0) {
        EXPECT_LT(fy, 0.0);
      } else if (lagged < 0.0) {
        EXPECT_GT(fy, 0.0);
      }
    }
  }
}

// L2 represents everything L1 leaves NaN, so the same discipline runs the
// other way: a quantity this tier DOES compute must not come back NaN, or the
// contract that NaN means "unrepresentable" is worthless.
TEST(ConventionsL2, EverythingTheTierRepresentsIsANumber) {
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, reference_params());
  VehicleState s = travelling(6.0);
  StepDiagnostics d;
  for (int i = 0; i < 500; ++i) {
    model->step(s, DriveInput{0.08, 2.0}, kDefaultDt, &d);
  }

  EXPECT_EQ(d.tier, static_cast<int>(Tier::L2_DoubleTrack));
  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    EXPECT_FALSE(std::isnan(d.alpha[i])) << "wheel " << i;
    EXPECT_FALSE(std::isnan(d.kappa[i])) << "wheel " << i;
    EXPECT_FALSE(std::isnan(d.fx[i])) << "wheel " << i;
    EXPECT_FALSE(std::isnan(d.fy[i])) << "wheel " << i;
    EXPECT_FALSE(std::isnan(d.fz[i])) << "wheel " << i;
  }
  EXPECT_FALSE(std::isnan(d.load_transfer_long));
  EXPECT_FALSE(std::isnan(d.load_transfer_lat));
  EXPECT_FALSE(std::isnan(d.ax));
  EXPECT_FALSE(std::isnan(d.ay));
}

// The actuator and battery states are live at this tier now (ADR-0031), so
// the discipline is the reverse of what it was while they were CORE-08 to
// CORE-10 promises: they must move, plausibly, and the states the tier still
// does not represent must stay exactly where they were.
TEST(ConventionsL2, TheActuatorAndBatteryStatesAreAlive) {
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, reference_params());
  VehicleState s = travelling(6.0);
  const VehicleState before = s;
  StepDiagnostics d;

  // Early in the transient the servo is still travelling, so the rate state
  // is visibly nonzero.
  for (int i = 0; i < 20; ++i) {
    model->step(s, DriveInput{0.3, 4.0}, kDefaultDt, &d);
  }
  EXPECT_GT(s.steer_rate, 0.0) << "the servo is mid-swing towards +0.3";
  EXPECT_GT(s.steer, 0.0);
  EXPECT_LT(s.steer, 0.3) << "and has not arrived yet";

  for (int i = 0; i < 380; ++i) {
    model->step(s, DriveInput{0.3, 4.0}, kDefaultDt, &d);
  }

  // Settled: the achieved angle is at the command to servo accuracy, but it
  // got there through dynamics rather than assignment.
  EXPECT_NEAR(s.steer, 0.3, 1e-3);
  EXPECT_NEAR(s.steer_rate, 0.0, 0.05);

  // The battery has been paying for the drive the whole time. The terminal
  // voltage sits below the full-charge open-circuit 12.6 V (sag), and above
  // the fresh state's placeholder 11.1 V, which is nominal rather than an
  // open-circuit value and is overwritten on the first step.
  EXPECT_LT(s.soc, before.soc) << "driving must drain the pack (CORE-09)";
  EXPECT_LT(s.pack_v, 12.6) << "terminal voltage must sag below the OCV";
  EXPECT_GT(s.pack_v, 11.1);
  EXPECT_GT(s.soc, 0.99) << "but 0.4 s of driving is not a whole pack";

  EXPECT_EQ(s.roll, 0.0) << "no suspension at this tier";
  EXPECT_EQ(s.pitch, 0.0);
}

// The regen sign conventions, end to end (ADR-0031): braking while rolling
// forward is a negative wheel torque, a negative battery current (the pack is
// being charged), a rising state of charge, and a negative slip ratio on the
// driven wheels only, because the motor is the only brake the car has.
TEST(ConventionsL2, RegenBrakingCarriesTheChargeSigns) {
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, reference_params());
  VehicleState s = travelling(10.0);
  s.soc = 0.8;  // headroom to charge into
  StepDiagnostics d;

  // A moment of coasting so the wheel speeds match the velocity.
  for (int i = 0; i < 50; ++i) {
    model->step(s, DriveInput{0.0, 0.0}, kDefaultDt, &d);
  }
  const double soc_before = s.soc;

  for (int i = 0; i < 500; ++i) {
    model->step(s, DriveInput{0.0, -6.0}, kDefaultDt, &d);
  }

  EXPECT_LT(d.drive_torque, 0.0) << "braking is negative wheel torque";
  EXPECT_LT(d.pack_current, 0.0) << "regen charges: negative terminal current";
  EXPECT_GT(s.soc, soc_before) << "the charge went somewhere";
  EXPECT_TRUE(d.esc_saturated) << "-6 m/s^2 wants 1.05 N m against a 0.4 N m "
                                  "regen cap, so the limit must have engaged";

  EXPECT_LT(d.fx[slipx::kRearLeft], 0.0);
  EXPECT_LT(d.fx[slipx::kRearRight], 0.0);
  EXPECT_EQ(d.fx[slipx::kFrontLeft], 0.0) << "no friction brakes anywhere";
  EXPECT_EQ(d.fx[slipx::kFrontRight], 0.0);
  EXPECT_LT(d.kappa[slipx::kRearLeft], 0.0) << "braking slip is negative";
  EXPECT_EQ(d.kappa[slipx::kFrontLeft], 0.0) << "a freewheeling wheel has "
                                                "no slip at all";
}

// The lateral load transfer sign, through the assembled tier. Positive ay is a
// left turn and moves load to the RIGHT, which is the sign that catches
// people and the one load_transfer.hpp's own tests assert in isolation.
TEST(ConventionsL2, PositiveLateralAccelerationLoadsTheRightHandWheels) {
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, reference_params());
  VehicleState s = travelling(6.0);
  StepDiagnostics d;
  for (int i = 0; i < 2000; ++i) {
    model->step(s, DriveInput{0.07, hold_speed(s, 6.0)}, kDefaultDt, &d);
  }

  ASSERT_GT(d.ay, 0.0);
  EXPECT_GT(d.load_transfer_lat, 0.0);
  EXPECT_GT(d.fz[slipx::kFrontRight], d.fz[slipx::kFrontLeft]);
  EXPECT_GT(d.fz[slipx::kRearRight], d.fz[slipx::kRearLeft]);
}

}  // namespace
