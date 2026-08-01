// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// SRS 7, layer one: closed-form cases with known answers.
//
// These are the tests that catch sign errors and unit errors, which are the
// bugs vehicle dynamics code actually has. Each one compares the integrated
// model against a formula derived independently of the implementation, so
// agreement is evidence rather than a tautology.

#include <gtest/gtest.h>

#include <cmath>

#include "slipx/vehicle_model.hpp"
#include "test_support.hpp"

namespace {

using namespace slipx_test;
using slipx::DriveInput;
using slipx::StepDiagnostics;
using slipx::Tier;
using slipx::VehicleModel;
using slipx::VehicleParams;
using slipx::VehicleState;

constexpr double kDt = kDefaultDt;

// ------------------------------------------------------ steady-state corner

struct SteadyCorner {
  double delta = 0.0;      // road wheel angle held                  [rad]
  double speed = 0.0;      // magnitude of the CoG velocity          [m/s]
  double yaw_rate = 0.0;   //                                      [rad/s]
  double ay = 0.0;         // lateral specific force               [m/s^2]
  double radius = 0.0;     // path radius of the CoG                   [m]
};

// Hold a steer angle and a speed until the transient has died, then report the
// settled condition. Ten seconds at 1 kHz is many times the yaw settling time
// of a car this size.
SteadyCorner settle(const VehicleModel& model, double delta, double v_target) {
  VehicleState s = travelling(v_target);
  const StepDiagnostics d = run_for(
      model, s, 10.0, kDt, [&](const VehicleState& st, double) {
        return DriveInput{delta, hold_speed(st, v_target)};
      });

  SteadyCorner c;
  c.delta = delta;
  c.speed = s.speed();
  c.yaw_rate = s.yaw_rate();
  c.ay = d.ay;
  c.radius = c.speed / std::fabs(c.yaw_rate);
  return c;
}

// The textbook understeer gradient, in SI units of rad per m/s^2:
//
//   K = (W_f / C_f - W_r / C_r) / g       with W_f = m g l_r / L
//     = (m / L) * (l_r / C_f - l_f / C_r)
//
// Positive is understeer: more steer needed as lateral acceleration rises.
double textbook_understeer_gradient(const VehicleParams& p) {
  return (p.mass / p.wheelbase()) *
         (p.lr / p.c_alpha_f - p.lf / p.c_alpha_r);
}

// The measured gradient, from the steady-state cornering equation
// delta = L / R + K * ay.
double measured_understeer_gradient(const SteadyCorner& c, double wheelbase) {
  return (c.delta - wheelbase / c.radius) / c.ay;
}

TEST(Analytical, L1UndersteerGradientMatchesTheTextbookFormula) {
  const auto p = reference_params();
  auto model = VehicleModel::create(Tier::L1_Bicycle, p);

  const double expected = textbook_understeer_gradient(p);
  ASSERT_GT(expected, 0.0) << "the reference car is set up to understeer";

  const SteadyCorner c = settle(*model, 0.05, 5.0);
  ASSERT_FALSE(c.ay != c.ay);
  EXPECT_NEAR(measured_understeer_gradient(c, p.wheelbase()), expected,
              0.08 * expected)
      << "measured " << measured_understeer_gradient(c, p.wheelbase())
      << " vs textbook " << expected;
}

// The gradient is a property of the car, not of the test condition. Measuring
// it at two speeds and two steer angles and getting the same number is a much
// stronger statement than matching the formula once.
TEST(Analytical, L1UndersteerGradientIsIndependentOfSpeedAndSteerAngle) {
  const auto p = reference_params();
  auto model = VehicleModel::create(Tier::L1_Bicycle, p);

  const double a = measured_understeer_gradient(settle(*model, 0.05, 4.0),
                                                p.wheelbase());
  const double b = measured_understeer_gradient(settle(*model, 0.05, 6.0),
                                                p.wheelbase());
  const double c = measured_understeer_gradient(settle(*model, 0.08, 5.0),
                                                p.wheelbase());

  EXPECT_NEAR(a, b, 0.06 * std::fabs(a));
  EXPECT_NEAR(a, c, 0.10 * std::fabs(a));
}

// A neutral-steer car needs the same steer angle for a given radius at every
// speed. This is the cleanest available check that the front and rear axle
// terms enter the yaw balance with the right relative sign and magnitude: get
// either wrong and the speed independence disappears.
TEST(Analytical, L1NeutralSteerCarNeedsTheKinematicAngleAtEverySpeed) {
  auto p = reference_params();
  p.lf = p.lr = 0.16;
  p.c_alpha_f = p.c_alpha_r = 125.0;  // K = 0 by construction
  ASSERT_NEAR(textbook_understeer_gradient(p), 0.0, 1e-15);

  auto model = VehicleModel::create(Tier::L1_Bicycle, p);
  const double delta = 0.05;

  for (const double v : {3.0, 5.0, 7.0}) {
    const SteadyCorner c = settle(*model, delta, v);
    EXPECT_NEAR(c.delta, p.wheelbase() / c.radius, 0.02 * delta)
        << "at " << v << " m/s";
  }
}

TEST(Analytical, L1OversteerCarNeedsLessSteerAsSpeedRises) {
  auto p = reference_params();
  p.c_alpha_f = 160.0;
  p.c_alpha_r = 100.0;  // soft rear: oversteer
  ASSERT_LT(textbook_understeer_gradient(p), 0.0);

  auto model = VehicleModel::create(Tier::L1_Bicycle, p);
  const double slow = settle(*model, 0.05, 3.0).radius;
  const double fast = settle(*model, 0.05, 6.0).radius;

  EXPECT_LT(fast, slow) << "an oversteering car tightens its line with speed";

  auto q = reference_params();
  q.c_alpha_f = 100.0;
  q.c_alpha_r = 160.0;  // stiff rear: understeer
  auto understeerer = VehicleModel::create(Tier::L1_Bicycle, q);
  EXPECT_GT(settle(*understeerer, 0.05, 6.0).radius,
            settle(*understeerer, 0.05, 3.0).radius)
      << "and an understeering one runs wide";
}

// ------------------------------------------------------------- longitudinal

// Terminal velocity is where drive force balances drag plus rolling
// resistance:  m * a_cmd = c_d * v^2 + f_roll * m * g * tanh(v / v_eps).
// Solved here by bisection on the same balance the model integrates, which
// tests the force magnitudes rather than the integration.
TEST(Analytical, L1TerminalVelocityMatchesTheDragBalance) {
  auto p = reference_params();
  p.v_max = 50.0;  // lift the top-speed clip out of the way
  auto model = VehicleModel::create(Tier::L1_Bicycle, p);

  const double accel_cmd = 1.0;  // [m/s^2], low enough to settle below v_max
  const double drive = p.mass * accel_cmd;

  const auto residual = [&](double v) {
    return drive - p.drag_coeff * v * v -
           p.roll_resist * p.mass * slipx::kGravity * std::tanh(v / p.v_eps);
  };
  double lo = 0.0;
  double hi = 100.0;
  for (int i = 0; i < 200; ++i) {
    const double mid = 0.5 * (lo + hi);
    (residual(mid) > 0.0 ? lo : hi) = mid;
  }
  const double expected = 0.5 * (lo + hi);
  ASSERT_LT(expected, p.v_max);

  VehicleState s = at_rest();
  run_for(*model, s, 120.0, kDt, [&](const VehicleState&, double) {
    return DriveInput{0.0, accel_cmd};
  });

  EXPECT_NEAR(s.vel_body.x, expected, 1e-3);
}

// Coastdown is one of the identification manoeuvres (ID-02), and it is what
// separates the drag coefficient from the rolling resistance coefficient:
// rolling resistance is speed-independent, drag goes as v^2, so the
// deceleration at two speeds gives two equations.
TEST(Analytical, L1CoastdownDecelerationMatchesTheResistanceModel) {
  const auto p = reference_params();
  auto model = VehicleModel::create(Tier::L1_Bicycle, p);

  for (const double v0 : {4.0, 8.0}) {
    VehicleState s = travelling(v0);
    StepDiagnostics d;
    model->step(s, DriveInput{0.0, 0.0}, kDt, &d);

    const double expected =
        -(p.drag_coeff * v0 * v0 +
          p.roll_resist * p.mass * slipx::kGravity * std::tanh(v0 / p.v_eps)) /
        p.mass;
    EXPECT_NEAR(d.ax, expected, 1e-3) << "at " << v0 << " m/s";
  }
}

// L0 has no mass and no resistance by construction, so constant demand gives
// textbook constant acceleration. RK4 is exact on polynomials of this degree,
// so the tolerance is rounding, not truncation.
TEST(Analytical, L0StraightLineIsExactlyTheKinematicSolution) {
  auto model = VehicleModel::create(Tier::L0_Kinematic, reference_params());

  const double a = 2.0;
  const double t = 3.0;
  VehicleState s = at_rest();
  run_for(*model, s, t, kDt, [&](const VehicleState&, double) {
    return DriveInput{0.0, a};
  });

  EXPECT_NEAR(s.vel_body.x, a * t, 1e-12);
  EXPECT_NEAR(s.pos.x, 0.5 * a * t * t, 1e-9);
  EXPECT_NEAR(s.pos.y, 0.0, 1e-15);
  EXPECT_NEAR(s.yaw, 0.0, 1e-15);
}

// ---------------------------------------------------------------- geometry

// The kinematic bicycle's defining relations, checked against the geometry
// they come from rather than against another simulation:
//
//   beta      = atan(l_r tan(delta) / L)
//   yaw rate  = v cos(beta) tan(delta) / L
//   radius    = v / yaw rate
TEST(Analytical, L0YawRateAndRadiusFollowAckermannGeometry) {
  const auto p = reference_params();
  auto model = VehicleModel::create(Tier::L0_Kinematic, p);

  const double v = 4.0;
  const double delta = 0.20;
  const double beta = std::atan(p.lr * std::tan(delta) / p.wheelbase());

  // Four seconds is sixteen time constants of the proportional speed hold,
  // which is what it takes for the settled speed to be the speed the
  // relations below are checked against to a part in a million rather than a
  // part in a thousand.
  VehicleState s = travelling(v);
  run_for(*model, s, 4.0, kDt, [&](const VehicleState& st, double) {
    return DriveInput{delta, hold_speed(st, v)};
  });

  // The relations are checked against the achieved state, not the requested
  // speed. hold_speed holds the body x component, and the kinematic bicycle's
  // velocity vector sits at beta to the body axis, so the settled speed along
  // that vector is v / cos(beta). Two percent here, and comparing against the
  // requested number instead would have written that discrepancy into the
  // tolerance and hidden it.
  EXPECT_NEAR(s.sideslip(), beta, 1e-12);
  EXPECT_NEAR(s.speed(), v / std::cos(beta), 1e-6);

  const double expected_rate =
      s.speed() * std::cos(beta) * std::tan(delta) / p.wheelbase();
  EXPECT_NEAR(s.yaw_rate(), expected_rate, 1e-9);

  // Equivalently, and more usefully for a controller author: the yaw rate is
  // the body longitudinal speed times tan(delta) over the wheelbase, with no
  // beta term at all.
  EXPECT_NEAR(s.yaw_rate(), s.vel_body.x * std::tan(delta) / p.wheelbase(),
              1e-9);

  const double expected_radius = p.wheelbase() / (std::cos(beta) *
                                                  std::tan(delta));
  EXPECT_NEAR(s.speed() / s.yaw_rate(), expected_radius, 1e-6);
}

// A full circle must return the car to where it started. This catches
// integration drift and any inconsistency between the yaw rate the model
// reports and the path it actually traces, which a single-step check cannot.
TEST(Analytical, L0ClosesTheCircleItClaimsToBeDriving) {
  const auto p = reference_params();
  auto model = VehicleModel::create(Tier::L0_Kinematic, p);

  const double v = 4.0;
  const double delta = 0.20;

  // Settle onto the circle first, then measure the rate the car actually
  // holds and ask for exactly one revolution of it. Computing the period from
  // the requested speed instead would build the speed controller's steady
  // offset into the answer and make this a test of the controller.
  VehicleState s = travelling(v);
  const auto policy = [&](const VehicleState& st, double) {
    return DriveInput{delta, hold_speed(st, v)};
  };
  run_for(*model, s, 2.0, kDt, policy);

  const double rate = s.yaw_rate();
  const double period = 2.0 * slipx::kPi / rate;
  const double start_yaw = s.yaw;
  const auto start = s.pos;
  run_for(*model, s, period, kDt, policy);

  const double radius = s.speed() / rate;
  EXPECT_NEAR(s.pos.x, start.x, 1e-3 * radius);
  EXPECT_NEAR(s.pos.y, start.y, 1e-3 * radius);
  EXPECT_NEAR(slipx::wrap_to_pi(s.yaw - start_yaw), 0.0, 1e-3);
}

// --------------------------------------------------------------- saturation

// L1's stated limitation, made explicit: lateral force is clipped at mu * Fz,
// and the clip is reported. This is not a Magic Formula and does not pretend
// to be; what the test guarantees is that the tier cannot quietly produce a
// force no tyre could.
TEST(Analytical, L1LateralForceIsClippedAtTheFrictionLimitAndSaysSo) {
  const auto p = reference_params();
  auto model = VehicleModel::create(Tier::L1_Bicycle, p);

  VehicleState s = travelling(8.0);
  StepDiagnostics d;
  // Full lock at speed: far outside the linear region.
  model->step(s, DriveInput{p.steer_max, 0.0}, kDt, &d);

  const double fz_f = p.mass * slipx::kGravity * p.lr / p.wheelbase();
  EXPECT_LE(std::fabs(d.fy_front), p.mu_clip * fz_f + 1e-9);
  EXPECT_TRUE(d.tyre_saturated[slipx::kFrontLeft]);
  EXPECT_TRUE(d.tyre_saturated[slipx::kFrontRight]);

  // The unclipped linear force would have been far larger; without the clip
  // this car would corner at several g.
  EXPECT_GT(p.c_alpha_f * p.steer_max, 2.0 * p.mu_clip * fz_f);
}

TEST(Analytical, LateralAccelerationCannotExceedTheFrictionLimit) {
  const auto p = reference_params();
  auto model = VehicleModel::create(Tier::L1_Bicycle, p);

  VehicleState s = travelling(9.0);
  const StepDiagnostics d = run_for(
      *model, s, 2.0, kDt, [&](const VehicleState& st, double) {
        return DriveInput{p.steer_max, hold_speed(st, 9.0)};
      });

  EXPECT_LE(std::fabs(d.ay), p.mu_clip * slipx::kGravity + 1e-6);
}

}  // namespace
