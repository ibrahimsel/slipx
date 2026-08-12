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

#include "slipx/load_transfer.hpp"
#include "slipx/relaxation.hpp"
#include "slipx/tyre.hpp"
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

// ----------------------------------------------------------- load transfer
//
// CORE-05, the first piece of L2. Every case below compares
// slipx/load_transfer.hpp against the static equation written out here from
// the moment balance rather than copied from the implementation, which is the
// point of doing this before any tyre nonlinearity exists to hide a sign in.

using slipx::kFrontLeft;
using slipx::kFrontRight;
using slipx::kGravity;
using slipx::kRearLeft;
using slipx::kRearRight;
using slipx::WheelLoads;

// A car with distinct numbers in every position, so a transposed index or a
// front/rear swap cannot pass by symmetry.
VehicleParams asymmetric_params() {
  auto p = reference_params();
  p.mass = 4.1;
  p.lf = 0.13;
  p.lr = 0.19;          // rear-biased CoG: the front axle carries the more
  p.track_front = 0.22;
  p.track_rear = 0.25;
  p.h_cog = 0.07;
  return p;
}

double total(const WheelLoads& w) {
  return w.fz[kFrontLeft] + w.fz[kFrontRight] + w.fz[kRearLeft] +
         w.fz[kRearRight];
}

TEST(LoadTransfer, StaticAxleLoadsFollowTheMomentBalance) {
  const auto p = asymmetric_params();
  const WheelLoads w = slipx::static_loads(p);

  // Fz_front = m g l_r / L. The front carries l_r, not l_f.
  EXPECT_NEAR(w.fz_front, p.mass * kGravity * p.lr / p.wheelbase(), 1e-12);
  EXPECT_NEAR(w.fz_rear, p.mass * kGravity * p.lf / p.wheelbase(), 1e-12);
  EXPECT_NEAR(total(w), p.mass * kGravity, 1e-12);

  // The CoG is behind the mid-wheelbase point here, so the REAR axle carries
  // less. Stated as an inequality because that is the direction a reader can
  // check against the picture rather than against the algebra.
  EXPECT_GT(w.fz_front, w.fz_rear);

  // Nothing is transferred, and the corners of an axle share it equally.
  EXPECT_EQ(w.transfer_long, 0.0);
  EXPECT_EQ(w.transfer_lat, 0.0);
  EXPECT_EQ(w.fz[kFrontLeft], w.fz[kFrontRight]);
  EXPECT_EQ(w.fz[kRearLeft], w.fz[kRearRight]);
  EXPECT_FALSE(w.wheel_lifted);
}

// The static case must be the zero-acceleration case exactly, not merely to a
// tolerance. Two code paths that agree to eleven digits and not to sixteen are
// two models, and the cross-tier comparison in the low-acceleration limit
// would then be measuring the discrepancy between them.
TEST(LoadTransfer, ZeroAccelerationReproducesTheStaticCaseBitForBit) {
  const auto p = asymmetric_params();
  const WheelLoads s = slipx::static_loads(p);
  const WheelLoads q = slipx::quasi_static_loads(p, 0.0, 0.0);

  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    EXPECT_EQ(s.fz[i], q.fz[i]) << "wheel " << i;
  }
  EXPECT_EQ(s.fz_front, q.fz_front);
  EXPECT_EQ(s.fz_rear, q.fz_rear);
  EXPECT_EQ(q.transfer_long, 0.0);
  EXPECT_EQ(q.transfer_lat, 0.0);
}

// dFz_long = m ax h / L, and the sign: accelerating loads the rear.
TEST(LoadTransfer, LongitudinalTransferMatchesThePitchMomentBalance) {
  const auto p = asymmetric_params();
  const WheelLoads s = slipx::static_loads(p);

  for (const double ax : {-6.0, -2.0, 1.0, 5.0}) {
    const WheelLoads w = slipx::quasi_static_loads(p, ax, 0.0);
    const double expected = p.mass * ax * p.h_cog / p.wheelbase();

    EXPECT_NEAR(w.transfer_long, expected, 1e-12) << "at ax " << ax;
    EXPECT_NEAR(w.fz_front, s.fz_front - expected, 1e-12) << "at ax " << ax;
    EXPECT_NEAR(w.fz_rear, s.fz_rear + expected, 1e-12) << "at ax " << ax;
    EXPECT_NEAR(total(w), p.mass * kGravity, 1e-12) << "at ax " << ax;

    // No lateral acceleration, so no left/right difference at all.
    EXPECT_EQ(w.fz[kFrontLeft], w.fz[kFrontRight]);
    EXPECT_EQ(w.fz[kRearLeft], w.fz[kRearRight]);
    EXPECT_EQ(w.transfer_lat, 0.0);
  }

  EXPECT_GT(slipx::quasi_static_loads(p, 4.0, 0.0).fz_rear, s.fz_rear)
      << "accelerating must load the rear axle";
  EXPECT_GT(slipx::quasi_static_loads(p, -4.0, 0.0).fz_front, s.fz_front)
      << "braking must load the front axle";
}

// dFz_lat at an axle is m_axle ay h / t_axle, with the axle's share of the
// lateral force taken from the yaw moment balance: Fy_f = m ay l_r / L.
TEST(LoadTransfer, LateralTransferMatchesTheRollMomentBalancePerAxle) {
  const auto p = asymmetric_params();
  const WheelLoads s = slipx::static_loads(p);

  for (const double ay : {-4.0, -1.0, 2.0, 5.0}) {
    const WheelLoads w = slipx::quasi_static_loads(p, 0.0, ay);

    const double mass_front = p.mass * p.lr / p.wheelbase();
    const double mass_rear = p.mass * p.lf / p.wheelbase();
    const double d_front = mass_front * ay * p.h_cog / p.track_front;
    const double d_rear = mass_rear * ay * p.h_cog / p.track_rear;

    // Positive ay is a left turn, and it loads the RIGHT wheels.
    EXPECT_NEAR(w.fz[kFrontRight], 0.5 * s.fz_front + d_front, 1e-12);
    EXPECT_NEAR(w.fz[kFrontLeft], 0.5 * s.fz_front - d_front, 1e-12);
    EXPECT_NEAR(w.fz[kRearRight], 0.5 * s.fz_rear + d_rear, 1e-12);
    EXPECT_NEAR(w.fz[kRearLeft], 0.5 * s.fz_rear - d_rear, 1e-12);

    EXPECT_NEAR(w.transfer_lat, d_front + d_rear, 1e-12) << "at ay " << ay;
    EXPECT_NEAR(total(w), p.mass * kGravity, 1e-12) << "at ay " << ay;

    // Pure cornering moves nothing between the axles.
    EXPECT_NEAR(w.fz_front, s.fz_front, 1e-12);
    EXPECT_NEAR(w.fz_rear, s.fz_rear, 1e-12);
  }
}

// With equal tracks the two axle terms collapse to the textbook single
// expression, m ay h / t, independently of where the CoG sits along the
// wheelbase. That the weight distribution drops out is a real check on the
// axle split rather than a restatement of it.
TEST(LoadTransfer, EqualTracksGiveTheTextbookTotalLateralTransfer) {
  auto p = asymmetric_params();
  p.track_front = 0.24;
  p.track_rear = 0.24;

  for (const double lf : {0.10, 0.16, 0.22}) {
    p.lf = lf;
    p.lr = 0.32 - lf;
    const double ay = 3.5;
    const WheelLoads w = slipx::quasi_static_loads(p, 0.0, ay);
    EXPECT_NEAR(w.transfer_lat, p.mass * ay * p.h_cog / p.track_front, 1e-12)
        << "at lf " << lf;
  }
}

// The two transfers superpose, because both are linear in their acceleration
// and neither feeds the other. Worth asserting rather than assuming: the
// obvious implementation mistake is to apply the lateral split about the
// static axle load instead of the load that axle actually has after the
// longitudinal transfer, and that shows up here.
TEST(LoadTransfer, LongitudinalAndLateralTransfersSuperpose) {
  const auto p = asymmetric_params();
  const double ax = 3.0;
  const double ay = -2.5;

  const WheelLoads both = slipx::quasi_static_loads(p, ax, ay);
  const WheelLoads only_x = slipx::quasi_static_loads(p, ax, 0.0);
  const WheelLoads only_y = slipx::quasi_static_loads(p, 0.0, ay);
  const WheelLoads none = slipx::quasi_static_loads(p, 0.0, 0.0);

  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    EXPECT_NEAR(both.fz[i], only_x.fz[i] + only_y.fz[i] - none.fz[i], 1e-12)
        << "wheel " << i;
  }
  EXPECT_NEAR(total(both), p.mass * kGravity, 1e-12);
}

// Linear in each acceleration, so doubling the acceleration doubles the
// transfer. This is what makes the model identifiable: a skidpad at two
// lateral accelerations gives a straight line whose slope is m h / t, and h
// is the only unknown in it.
TEST(LoadTransfer, TransferIsLinearInAcceleration) {
  const auto p = asymmetric_params();
  const double a = slipx::quasi_static_loads(p, 0.0, 1.5).transfer_lat;
  const double b = slipx::quasi_static_loads(p, 0.0, 3.0).transfer_lat;
  EXPECT_NEAR(b, 2.0 * a, 1e-12);

  const double c = slipx::quasi_static_loads(p, 1.5, 0.0).transfer_long;
  const double d = slipx::quasi_static_loads(p, 3.0, 0.0).transfer_long;
  EXPECT_NEAR(d, 2.0 * c, 1e-12);
}

// The static rollover threshold, ay = g t / (2 h), checked by driving the
// model to exactly that acceleration and finding the inner wheels at zero.
// Independent of mass and of weight distribution, which is the whole reason
// the number is quotable for a chassis rather than for a chassis at a given
// ballast.
TEST(LoadTransfer, InnerWheelsLiftAtTheStaticRolloverThreshold) {
  auto p = asymmetric_params();
  p.track_front = 0.24;
  p.track_rear = 0.24;  // one threshold rather than two

  const double expected = kGravity * p.track_front / (2.0 * p.h_cog);
  EXPECT_NEAR(slipx::static_rollover_threshold(p), expected, 1e-12);

  const WheelLoads at = slipx::quasi_static_loads(p, 0.0, expected);
  EXPECT_NEAR(at.fz[kFrontLeft], 0.0, 1e-9);
  EXPECT_NEAR(at.fz[kRearLeft], 0.0, 1e-9);
  EXPECT_NEAR(at.fz[kFrontRight], at.fz_front, 1e-9);
  EXPECT_NEAR(at.fz[kRearRight], at.fz_rear, 1e-9);

  // Just below it every wheel is still loaded; just above it, one is not.
  EXPECT_FALSE(slipx::quasi_static_loads(p, 0.0, 0.99 * expected).wheel_lifted);
  EXPECT_TRUE(slipx::quasi_static_loads(p, 0.0, 1.01 * expected).wheel_lifted);

  // Mass and weight distribution cannot move it.
  auto heavy = p;
  heavy.mass = 3.0 * p.mass;
  heavy.lf = 0.10;
  heavy.lr = 0.22;
  EXPECT_NEAR(slipx::static_rollover_threshold(heavy), expected, 1e-12);
  EXPECT_TRUE(slipx::quasi_static_loads(heavy, 0.0, 1.01 * expected)
                  .wheel_lifted);
}

// Past the threshold, load is redistributed rather than deleted. A clamp that
// only floored the negative wheel at zero would lose weight from the car, and
// every tyre force computed from those loads would be wrong by that amount
// without anything looking obviously broken.
TEST(LoadTransfer, WeightIsConservedThroughWheelLift) {
  const auto p = asymmetric_params();
  for (const double ax : {-25.0, -8.0, 0.0, 8.0, 25.0}) {
    for (const double ay : {-30.0, -12.0, 0.0, 12.0, 30.0}) {
      const WheelLoads w = slipx::quasi_static_loads(p, ax, ay);
      EXPECT_NEAR(total(w), p.mass * kGravity, 1e-9)
          << "at ax " << ax << " ay " << ay;
      EXPECT_NEAR(w.fz_front + w.fz_rear, p.mass * kGravity, 1e-9)
          << "at ax " << ax << " ay " << ay;
      for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
        EXPECT_GE(w.fz[i], 0.0) << "wheel " << i << " at ax " << ax
                                << " ay " << ay;
      }
    }
  }
}

// A CoG on the ground has no lever to transfer load through, whatever the car
// is doing. This is the degenerate case the whole model reduces to, and it is
// the one that makes "CoG height is inert below L2" a statement about the
// tier rather than about the formula.
TEST(LoadTransfer, ZeroCoGHeightTransfersNothing) {
  auto p = asymmetric_params();
  p.h_cog = 1e-12;  // validate() forbids exactly zero; this is as close as it
                    // gets and the transfer is already below a micronewton
  const WheelLoads w = slipx::quasi_static_loads(p, 8.0, 8.0);
  const WheelLoads s = slipx::static_loads(p);
  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    EXPECT_NEAR(w.fz[i], s.fz[i], 1e-9) << "wheel " << i;
  }
}

// ----------------------------------------------------------------- MF-lite
//
// CORE-06, the second piece of L2. The Magic Formula has no closed-form
// inverse and no tidy algebraic identities, so what is checked here is the set
// of properties that make it a tyre rather than an arbitrary curve, each one
// derived from the formula on paper and none of them read back out of
// tyre.hpp: the slope at the origin, the height and existence of the peak, how
// both move with vertical load, and oddness.

using slipx::MfLite;
using slipx::TyreCoefficients;

// A tyre with no round numbers in it, so an omitted factor cannot cancel.
TyreCoefficients reference_tyre() {
  TyreCoefficients t;
  t.mu_y0 = 1.15;
  t.mu_x0 = 1.32;
  t.k_mu = 0.22;
  t.shape_c = 1.68;
  t.curvature_e = 0.42;
  return t;
}

constexpr double kCAlphaTyre = 61.0;  // per tyre                    [N/rad]
constexpr double kFzNom = 8.4;        // static load on that tyre        [N]

MfLite reference_mf_lite() {
  return slipx::make_mf_lite(reference_tyre(), kCAlphaTyre, kFzNom);
}

// Largest |Fy| over a fine sweep, and the slip angle it occurred at. Sampled
// rather than solved, because the peak has no closed form; the resolution is
// what sets the tolerance in the tests that use it.
struct Peak {
  double fy = 0.0;
  double alpha = 0.0;
};

Peak sweep_peak(const MfLite& t, double fz) {
  Peak best;
  // Out to 4 rad, which is far past anything a car reaches and is there so that
  // the pathological coefficient pairs below have their peak inside the sweep
  // rather than at its edge.
  for (int i = 0; i <= 400000; ++i) {
    const double alpha = 4.0 * static_cast<double>(i) / 400000.0;
    const double fy = std::fabs(slipx::mf_lite_fy(t, alpha, fz));
    if (fy > best.fy) {
      best.fy = fy;
      best.alpha = alpha;
    }
  }
  return best;
}

// The one property the whole B derivation exists to buy: at the nominal load
// and small slip, MF-lite IS L1's linear tyre, Fy = -C_alpha alpha. Not
// approximately and not to plotting accuracy. This is what makes the cross-tier
// convergence check a measurement of discretisation error rather than of a
// parameter mismatch, and it is the test that a B missing its C, its mu or its
// Fz fails immediately.
TEST(MfLite, SmallSlipReproducesTheLinearTyre) {
  const MfLite t = reference_mf_lite();

  for (const double alpha : {1e-7, 1e-6, 1e-5}) {
    const double fy = slipx::mf_lite_fy(t, alpha, kFzNom);
    const double linear = -kCAlphaTyre * alpha;
    // Relative, because the absolute force here is of order a micronewton.
    // The formula's leading correction is cubic in alpha, so the error at
    // 1e-5 rad is already below 1e-9 relative.
    EXPECT_NEAR(fy / linear, 1.0, 1e-8) << "at alpha " << alpha;
  }

  // Stated once more as the derivative, which is the quantity a skidpad
  // actually measures.
  EXPECT_NEAR(slipx::cornering_stiffness_at_load(t, kFzNom), kCAlphaTyre,
              1e-10);
}

// The peak is exactly mu Fz, reached at a finite slip angle, with a falling
// branch beyond it. The falling branch is the entire difference between this
// and L1's clip: it is the mechanism by which a car spins rather than slides
// and recovers.
TEST(MfLite, PeakIsMuTimesLoadAndTheCurveFallsBeyondIt) {
  const MfLite t = reference_mf_lite();
  const Peak peak = sweep_peak(t, kFzNom);

  EXPECT_NEAR(peak.fy, reference_tyre().mu_y0 * kFzNom, 1e-6);

  // At a finite slip angle, and at one the right size. The scale to measure it
  // against is not an absolute number of degrees, which would only be a
  // property of these provisional coefficients; it is the slip angle at which
  // the LINEAR tyre would have reached the same peak,
  //
  //   alpha_lin = mu_y Fz / C_alpha
  //
  // A real tyre peaks at somewhere between 1.5 and 3 times that, and the
  // multiple is set by C and E alone. See the note in tyre.hpp: a low C with a
  // high E is legal under tyre.schema.json and gives a multiple above 20,
  // which is a tyre whose peak no car reaches.
  const double alpha_lin = reference_tyre().mu_y0 * kFzNom / kCAlphaTyre;
  EXPECT_GT(peak.alpha, 1.5 * alpha_lin);
  EXPECT_LT(peak.alpha, 3.0 * alpha_lin);

  // And it falls afterwards. Checked at three points so a single flat spot
  // cannot pass, and the last is well past anything a car reaches.
  const double past_1 = std::fabs(slipx::mf_lite_fy(t, 2.0 * peak.alpha,
                                                    kFzNom));
  const double past_2 = std::fabs(slipx::mf_lite_fy(t, 4.0 * peak.alpha,
                                                    kFzNom));
  const double past_3 = std::fabs(slipx::mf_lite_fy(t, 8.0 * peak.alpha,
                                                    kFzNom));
  EXPECT_LT(past_1, peak.fy);
  EXPECT_LT(past_2, past_1);
  EXPECT_LT(past_3, past_2);

  // The size of the drop matters as much as its direction: a curve that gave
  // up 1% of its peak would not spin a car, because the moment that takes the
  // rear axle round needs the force to fall faster than the slip angle grows.
  EXPECT_LT(past_2, 0.9 * peak.fy);
}

// Where the peak sits, in the only units the question has an answer in.
// alpha_peak / alpha_lin depends on C and E and on nothing else: not on the
// cornering stiffness, not on the friction coefficient, and not on the load.
// That independence is why C and E are the two parameters a slip sweep is for,
// and it is what makes the trap in tyre.hpp a property of the pair rather than
// of a particular car.
TEST(MfLite, ThePeakSlipAngleScalesWithTheLinearTyreAndDependsOnlyOnCAndE) {
  const TyreCoefficients c = reference_tyre();

  double reference_ratio = 0.0;
  for (const double c_alpha : {30.0, 61.0, 150.0}) {
    for (const double fz_nom : {4.0, 8.4, 25.0}) {
      const MfLite t = slipx::make_mf_lite(c, c_alpha, fz_nom);
      const double alpha_lin = c.mu_y0 * fz_nom / c_alpha;
      const double ratio = sweep_peak(t, fz_nom).alpha / alpha_lin;

      if (reference_ratio == 0.0) reference_ratio = ratio;
      // Loose only because the peak is found by sampling; the underlying
      // quantity is exact.
      EXPECT_NEAR(ratio, reference_ratio, 1e-3)
          << "C_alpha " << c_alpha << " Fz_nom " << fz_nom;
    }
  }

  // The value for the reference coefficients, quoted so that a change to C or
  // E has to move a number in this file and be noticed.
  EXPECT_NEAR(reference_ratio, 2.692, 2e-3);

  // A larger C brings the peak in, which is the lever an identification run
  // uses when the fitted curve peaks too late.
  TyreCoefficients sharper = c;
  sharper.shape_c = 2.10;
  const MfLite s = slipx::make_mf_lite(sharper, kCAlphaTyre, kFzNom);
  const double alpha_lin = c.mu_y0 * kFzNom / kCAlphaTyre;
  EXPECT_LT(sweep_peak(s, kFzNom).alpha / alpha_lin, reference_ratio);

  // And the trap: legal coefficients at the corners of the schema's boxes give
  // a peak more than ten times further out than a tyre has one. Asserted so
  // that the claim in tyre.hpp is a measurement rather than an anecdote.
  TyreCoefficients flat = c;
  flat.shape_c = 1.05;
  flat.curvature_e = 0.87;
  const MfLite f = slipx::make_mf_lite(flat, kCAlphaTyre, kFzNom);
  EXPECT_GT(sweep_peak(f, kFzNom).alpha / alpha_lin, 10.0);
}

// mu = mu_0 (Fz / Fz_nom)^(-k_mu), and the sign of that exponent is the whole
// of load sensitivity: friction FALLS as the tyre is pushed harder. Get it
// backwards and load transfer starts producing grip.
TEST(MfLite, FrictionFallsWithLoadByThePowerLaw) {
  const TyreCoefficients c = reference_tyre();
  const MfLite t = reference_mf_lite();

  for (const double ratio : {0.4, 1.0, 1.8, 3.0}) {
    const double fz = ratio * kFzNom;
    const double expected_mu = c.mu_y0 * std::pow(ratio, -c.k_mu);

    EXPECT_NEAR(slipx::mu_at_load(c.mu_y0, c.k_mu, fz, kFzNom), expected_mu,
                1e-12)
        << "at Fz/Fz_nom " << ratio;
    EXPECT_NEAR(slipx::peak_lateral_force(t, fz), expected_mu * fz, 1e-12)
        << "at Fz/Fz_nom " << ratio;
    EXPECT_NEAR(slipx::peak_longitudinal_force(t, fz),
                c.mu_x0 * std::pow(ratio, -c.k_mu) * fz, 1e-12)
        << "at Fz/Fz_nom " << ratio;

    // And through the curve, not only through the coefficient.
    EXPECT_NEAR(sweep_peak(t, fz).fy, expected_mu * fz, 1e-5)
        << "at Fz/Fz_nom " << ratio;
  }

  // The consequence, in the form a student meets it: doubling the load on a
  // tyre gives less than twice the grip, so a car that has transferred its
  // weight onto two wheels has less grip than the same car flat.
  EXPECT_LT(slipx::peak_lateral_force(t, 2.0 * kFzNom),
            2.0 * slipx::peak_lateral_force(t, kFzNom));
}

// Cornering stiffness inherits the load sensitivity of mu, because B is fixed:
// C_alpha(Fz) = C_alpha(Fz_nom) (Fz / Fz_nom)^(1 - k_mu). Checked against a
// numerical derivative of the force law rather than against the closed form
// alone, so that the two are shown to describe one tyre.
TEST(MfLite, CorneringStiffnessGrowsSublinearlyWithLoad) {
  const TyreCoefficients c = reference_tyre();
  const MfLite t = reference_mf_lite();
  const double h = 1e-6;

  for (const double ratio : {0.4, 1.0, 1.8, 3.0}) {
    const double fz = ratio * kFzNom;
    const double expected = kCAlphaTyre * std::pow(ratio, 1.0 - c.k_mu);

    EXPECT_NEAR(slipx::cornering_stiffness_at_load(t, fz), expected, 1e-9)
        << "at Fz/Fz_nom " << ratio;

    const double slope = -(slipx::mf_lite_fy(t, h, fz) -
                           slipx::mf_lite_fy(t, -h, fz)) / (2.0 * h);
    EXPECT_NEAR(slope, expected, 1e-6 * expected)
        << "at Fz/Fz_nom " << ratio;
  }

  // Sub-linear, which is the same statement as the peak one above and is the
  // reason a heavily loaded outer tyre does not simply take over.
  EXPECT_LT(slipx::cornering_stiffness_at_load(t, 2.0 * kFzNom),
            2.0 * kCAlphaTyre);
  EXPECT_GT(slipx::cornering_stiffness_at_load(t, 2.0 * kFzNom), kCAlphaTyre);
}

// A tyre with no load produces no force, and in particular does not produce a
// NaN. mu(Fz) diverges at Fz = 0, so the naive mu(Fz) * Fz is inf * 0. This is
// not a hypothetical: quasi_static_loads clamps a lifted wheel to exactly zero,
// so any car that reaches its rollover threshold takes this path, and a single
// NaN entering the state poisons the whole trajectory and its hash.
TEST(MfLite, ALiftedWheelProducesZeroForceAndNotNaN) {
  for (const double k_mu : {0.0, 0.15, 0.5, 1.0}) {
    TyreCoefficients c = reference_tyre();
    c.k_mu = k_mu;
    const MfLite t = slipx::make_mf_lite(c, kCAlphaTyre, kFzNom);

    for (const double alpha : {-0.3, 0.0, 0.05, 1.2}) {
      const double fy = slipx::mf_lite_fy(t, alpha, 0.0);
      EXPECT_TRUE(std::isfinite(fy)) << "k_mu " << k_mu << " alpha " << alpha;
      EXPECT_EQ(fy, 0.0) << "k_mu " << k_mu << " alpha " << alpha;
    }
    EXPECT_EQ(slipx::peak_lateral_force(t, 0.0), 0.0);
    EXPECT_EQ(slipx::peak_longitudinal_force(t, 0.0), 0.0);
  }
}

// k_mu = 0 must switch load sensitivity off completely, leaving force
// proportional to load. It is the degenerate case a user reaches for when they
// have not identified k_mu, and it has to behave.
TEST(MfLite, ZeroLoadSensitivityMakesForceProportionalToLoad) {
  TyreCoefficients c = reference_tyre();
  c.k_mu = 0.0;
  const MfLite t = slipx::make_mf_lite(c, kCAlphaTyre, kFzNom);

  for (const double ratio : {0.5, 2.0, 3.0}) {
    EXPECT_NEAR(slipx::peak_lateral_force(t, ratio * kFzNom),
                ratio * c.mu_y0 * kFzNom, 1e-12);
    EXPECT_NEAR(slipx::mf_lite_fy(t, 0.08, ratio * kFzNom),
                ratio * slipx::mf_lite_fy(t, 0.08, kFzNom), 1e-12);
    EXPECT_NEAR(slipx::cornering_stiffness_at_load(t, ratio * kFzNom),
                ratio * kCAlphaTyre, 1e-10);
  }
}

// ------------------------------------------------- combined slip (CORE-06)

using slipx::CombinedForce;

// Inside the ellipse the tyre delivers exactly what was asked, bit for bit.
// Anything else would mean the combined-slip path quietly altering pure-slip
// results, and every analytical case above would then be testing a curve the
// model does not use.
TEST(CombinedSlip, ADemandInsideTheEllipseIsUntouched) {
  const double fx_max = 9.7;
  const double fy_max = 8.3;

  for (const double fx : {-4.0, 0.0, 2.5}) {
    for (const double fy : {-3.0, 0.0, 1.5}) {
      const CombinedForce r = slipx::friction_ellipse(fx, fy, fx_max, fy_max);
      EXPECT_EQ(r.fx, fx);
      EXPECT_EQ(r.fy, fy);
      EXPECT_FALSE(r.saturated);
    }
  }
}

// Outside it, the result lands ON the ellipse and keeps the direction of the
// demand. Both halves matter: landing on the ellipse is the friction budget,
// and keeping the direction is what makes this a projection rather than two
// independent clips. A circle of radius max(fx_max, fy_max) would pass the
// direction half and fail the first; an axis-wise clamp the reverse.
TEST(CombinedSlip, ADemandOutsideTheEllipseIsProjectedOntoIt) {
  const double fx_max = 9.7;
  const double fy_max = 8.3;

  for (const double fx : {-30.0, -8.0, 6.0, 25.0}) {
    for (const double fy : {-24.0, -7.0, 5.0, 19.0}) {
      const CombinedForce r = slipx::friction_ellipse(fx, fy, fx_max, fy_max);
      const double demand = std::hypot(fx / fx_max, fy / fy_max);
      if (demand <= 1.0) continue;

      EXPECT_TRUE(r.saturated) << "at fx " << fx << " fy " << fy;
      EXPECT_NEAR(std::hypot(r.fx / fx_max, r.fy / fy_max), 1.0, 1e-12)
          << "at fx " << fx << " fy " << fy;
      // Same direction: the ratio is preserved, so the cross product with the
      // demand vanishes.
      EXPECT_NEAR(r.fx * fy - r.fy * fx, 0.0, 1e-9)
          << "at fx " << fx << " fy " << fy;
      // And it is a reduction, never an increase.
      EXPECT_LT(std::hypot(r.fx, r.fy), std::hypot(fx, fy));
    }
  }
}

// The axis cases pin the ellipse's two semi-axes to the two peak forces, which
// is the statement that mu_x and mu_y mean what they say. Pure braking at the
// limit gets mu_x Fz and nothing else, and the same tyre asked for both at once
// gets neither in full.
TEST(CombinedSlip, PureAxisDemandsRecoverThePeakForces) {
  const MfLite t = reference_mf_lite();
  const double fz = kFzNom;
  const double fx_max = slipx::peak_longitudinal_force(t, fz);
  const double fy_max = slipx::peak_lateral_force(t, fz);

  const CombinedForce braking = slipx::friction_ellipse(-99.0, 0.0, fx_max,
                                                        fy_max);
  EXPECT_NEAR(braking.fx, -fx_max, 1e-12);
  EXPECT_EQ(braking.fy, 0.0);
  EXPECT_TRUE(braking.saturated);

  const CombinedForce cornering = slipx::friction_ellipse(0.0, 99.0, fx_max,
                                                          fy_max);
  EXPECT_EQ(cornering.fx, 0.0);
  EXPECT_NEAR(cornering.fy, fy_max, 1e-12);
  EXPECT_TRUE(cornering.saturated);

  // Braking at the limit leaves nothing to steer with. The number is the
  // teaching point: asking for full braking AND full cornering gives 71% of
  // each, not 100% of one and a bit of the other.
  //
  // "Full" has to be expressed as a fraction of each axis's own limit rather
  // than as two equal forces, because mu_x and mu_y differ and the budget is an
  // ellipse rather than a circle. Demanding equal newtons on a tyre with
  // unequal limits is a demand in some other direction, and it is projected
  // onto some other point of the ellipse.
  const CombinedForce both = slipx::friction_ellipse(-2.0 * fx_max,
                                                     2.0 * fy_max, fx_max,
                                                     fy_max);
  EXPECT_NEAR(both.fx / fx_max, -std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(both.fy / fy_max, std::sqrt(0.5), 1e-12);

  // A saturated tyre using half its braking budget still has 87% of its
  // cornering budget, not 50%. That is the shape of the ellipse and the reason
  // trail braking works at all. Asked for along the ray through that point,
  // because the projection follows the direction of the demand: this function
  // cannot be handed a longitudinal force to keep and asked what is left over,
  // and a caller wanting that priority has to express it as a direction.
  const CombinedForce trail = slipx::friction_ellipse(
      -1.0 * fx_max, 2.0 * std::sqrt(0.75) * fy_max, fx_max, fy_max);
  EXPECT_NEAR(trail.fx / fx_max, -0.5, 1e-12);
  EXPECT_NEAR(trail.fy / fy_max, std::sqrt(0.75), 1e-12);
}

// A wheel with no load has no friction budget at all, and the division that
// computes the ellipse is undefined there. It reports saturation, because a
// demand that cannot be met is exactly what that flag means.
TEST(CombinedSlip, AZeroBudgetDeliversNothingAndSaysSo) {
  const CombinedForce asked = slipx::friction_ellipse(5.0, -3.0, 0.0, 0.0);
  EXPECT_EQ(asked.fx, 0.0);
  EXPECT_EQ(asked.fy, 0.0);
  EXPECT_TRUE(asked.saturated);

  const CombinedForce idle = slipx::friction_ellipse(0.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(idle.fx, 0.0);
  EXPECT_EQ(idle.fy, 0.0);
  EXPECT_FALSE(idle.saturated);
}

// ------------------------------------------------------------- relaxation
//
// CORE-07, the third piece of L2 and the first one with memory. The lag has an
// exact solution, so unlike MF-lite this section can check the integrated
// answer against a closed form rather than against properties of a curve.
//
// The single fact every test here is built around: the lag is in DISTANCE
// rolled, not in time. A fixed time constant is the plausible wrong
// implementation, it fits at one speed, and the two are indistinguishable in
// any test conducted at one speed. Three of the cases below therefore run at
// two speeds on purpose.

constexpr double kSigma = 0.08;  // relaxation length                     [m]

// Forward Euler on relaxation_rate. Deliberately the crudest integrator: the
// closed form it is compared against is exact, so any error is the
// integrator's, and using RK4 here would hide a rate that was wrong by a
// factor the truncation error could absorb.
double euler_lag(double alpha, double alpha_lag, double vx, double sigma,
                 double dt, int steps) {
  for (int i = 0; i < steps; ++i) {
    alpha_lag += dt * slipx::relaxation_rate(alpha, alpha_lag, vx, sigma);
  }
  return alpha_lag;
}

// The closed form, derived on paper from sigma d(alpha')/ds + alpha' = alpha
// and written here without reference to relaxation.hpp so that agreement is
// evidence rather than a restatement.
double lag_reference(double alpha, double alpha_lag0, double vx, double sigma,
                     double t) {
  const double tau = sigma / std::fabs(vx);
  return alpha + (alpha_lag0 - alpha) * std::exp(-t / tau);
}

// The integrated rate reproduces the exponential it was derived from. This is
// the case that fails if the rate is missing its speed, its sigma or its sign.
TEST(Relaxation, TheIntegratedRateMatchesTheClosedFormExponential) {
  const double alpha = 0.05;   // the step the geometry asks for      [rad]
  const double vx = 6.0;       //                                     [m/s]
  const double dt = 1e-6;      // far inside the stability bound        [s]

  for (const double t : {0.002, 0.010, 0.040}) {
    const int steps = static_cast<int>(t / dt + 0.5);
    const double integrated = euler_lag(alpha, 0.0, vx, kSigma, dt, steps);
    const double exact = lag_reference(alpha, 0.0, vx, kSigma, t);
    // Forward Euler is first order, so at dt = 1 us over 40 ms the error is
    // of order dt/tau per step accumulated, which is well under 1e-4
    // relative.
    EXPECT_NEAR(integrated / exact, 1.0, 1e-4) << "at t " << t;
  }

  // And relaxation_exact is that same closed form, to the last bit the
  // library's exp can produce.
  EXPECT_NEAR(slipx::relaxation_exact(alpha, 0.0, vx, kSigma, 0.010),
              lag_reference(alpha, 0.0, vx, kSigma, 0.010), 1e-15);
}

// The lag is a distance, so the same distance rolled gives the same fraction
// of the step regardless of how fast it was rolled. Doubling the speed halves
// the time to any given fraction, exactly.
//
// This is the test a fixed time constant fails, and it is the reason
// relaxation_rate multiplies by speed rather than treating sigma as seconds.
TEST(Relaxation, TheLagIsInDistanceRolledAndNotInTime) {
  const double alpha = 0.04;
  const double distance = 0.05;  // one distance, two speeds             [m]

  double fraction_at_speed[2] = {0.0, 0.0};
  double integrated_at_speed[2] = {0.0, 0.0};
  int i = 0;
  for (const double vx : {3.0, 12.0}) {
    const double t = distance / vx;  // the time to roll it at this speed
    const double lagged = slipx::relaxation_exact(alpha, 0.0, vx, kSigma, t);
    fraction_at_speed[i] = lagged / alpha;

    // The same claim about relaxation_rate, which is what the tiers actually
    // integrate. Checked separately because relaxation_exact could carry the
    // speed correctly while the rate did not, and a rate that treated sigma as
    // a time constant would agree with itself at every speed.
    const double dt = t / 20000.0;
    integrated_at_speed[i] =
        euler_lag(alpha, 0.0, vx, kSigma, dt, 20000) / alpha;
    ++i;
  }

  // Same distance, same fraction, to the accuracy of exp itself.
  EXPECT_NEAR(fraction_at_speed[0], fraction_at_speed[1], 1e-15);
  EXPECT_NEAR(integrated_at_speed[0], integrated_at_speed[1], 1e-9);
  EXPECT_NEAR(integrated_at_speed[0], fraction_at_speed[0], 1e-4);

  // And that fraction is the one the exponential predicts for the distance,
  // computed here without a speed appearing at all.
  EXPECT_NEAR(fraction_at_speed[0], 1.0 - std::exp(-distance / kSigma), 1e-15);
}

// One time constant is one relaxation length rolled, and it delivers the
// textbook 63.2% of a step.
TEST(Relaxation, OneTimeConstantIsOneRelaxationLengthAndReaches63Percent) {
  for (const double vx : {2.0, 9.0, 18.0}) {
    const double tau = slipx::relaxation_time_constant(vx, kSigma);
    EXPECT_NEAR(tau, kSigma / vx, 1e-15) << "at vx " << vx;

    // The distance rolled in one time constant is exactly sigma.
    EXPECT_NEAR(vx * tau, kSigma, 1e-15) << "at vx " << vx;

    const double lagged = slipx::relaxation_exact(1.0, 0.0, vx, kSigma, tau);
    EXPECT_NEAR(lagged, 1.0 - std::exp(-1.0), 1e-15) << "at vx " << vx;
    EXPECT_NEAR(lagged, 0.6321205588285577, 1e-12) << "at vx " << vx;
  }

  // Halving the speed doubles the time constant. Stated separately because it
  // is the relationship an identification run measures by doing the same step
  // steer twice.
  EXPECT_NEAR(slipx::relaxation_time_constant(3.0, kSigma),
              2.0 * slipx::relaxation_time_constant(6.0, kSigma), 1e-15);
}

// A tyre relaxes over distance rolled, and a car reversing is rolling. The
// magnitude of vx is what appears in the rate, so a reversing car lags exactly
// as a forward one does.
//
// Without the magnitude the sign flips and the lag becomes a runaway: the
// error term grows exponentially instead of decaying, which is the failure
// this case exists to catch.
TEST(Relaxation, ReversingRelaxesRatherThanDiverging) {
  const double alpha = 0.03;
  const double vx = 5.0;

  EXPECT_NEAR(slipx::relaxation_rate(alpha, 0.0, -vx, kSigma),
              slipx::relaxation_rate(alpha, 0.0, vx, kSigma), 1e-15);

  // 0.02 s at 5 m/s is 0.1 m rolled, which is 1.25 relaxation lengths, so the
  // closed form puts the lag at 1 - exp(-1.25) of the step. Compared against
  // that rather than against a round fraction, because the point of the case
  // is that reversing follows the same exponential and not merely something
  // bounded.
  const double reversing = euler_lag(alpha, 0.0, -vx, kSigma, 1e-6, 20000);
  const double forward = euler_lag(alpha, 0.0, vx, kSigma, 1e-6, 20000);
  EXPECT_NEAR(reversing, forward, 1e-15);
  EXPECT_NEAR(reversing / alpha, 1.0 - std::exp(-1.25), 1e-4);

  // And given long enough it settles on the target rather than running away
  // from it, which is what the sign of the rate decides.
  const double settled = euler_lag(alpha, 0.0, -vx, kSigma, 1e-6, 120000);
  EXPECT_LT(settled, alpha);
  EXPECT_GT(settled, 0.99 * alpha);
}

// A stationary tyre cannot build carcass deflection by rolling, so its lagged
// slip angle holds whatever it had. Exactly zero rate, not a small one.
TEST(Relaxation, AStationaryTyreHoldsItsSlipAngle) {
  EXPECT_EQ(slipx::relaxation_rate(0.30, 0.05, 0.0, kSigma), 0.0);

  const double held = euler_lag(0.30, 0.05, 0.0, kSigma, 1e-3, 5000);
  EXPECT_EQ(held, 0.05);

  // The time constant is the other half of the same statement.
  EXPECT_TRUE(std::isinf(slipx::relaxation_time_constant(0.0, kSigma)));
  EXPECT_TRUE(std::isinf(slipx::relaxation_max_step(0.0, kSigma)));
}

// The documented stability bound, checked by integrating either side of it.
// Below 2 tau forward Euler converges; above it the error alternates in sign
// and grows, which is what an oscillating slip angle looks like before it
// becomes a NaN.
TEST(Relaxation, TheStabilityBoundIsTwoTimeConstants) {
  const double vx = 10.0;
  const double bound = slipx::relaxation_max_step(vx, kSigma);
  EXPECT_NEAR(bound, 2.0 * kSigma / vx, 1e-15);

  const double alpha = 0.02;

  const double stable = euler_lag(alpha, 0.0, vx, kSigma, 0.9 * bound, 60);
  EXPECT_NEAR(stable, alpha, 1e-6);

  const double unstable = euler_lag(alpha, 0.0, vx, kSigma, 1.1 * bound, 60);
  EXPECT_GT(std::fabs(unstable - alpha), 1e3 * alpha);
}

// The margin claimed in relaxation.hpp for the provisional parameter set, as
// an assertion rather than a comment. If a future default makes the reference
// car unstable at the default step, this fails and says so.
TEST(Relaxation, TheReferenceCarHasStepMarginAtItsTopSpeed) {
  const VehicleParams p = reference_params();
  const double bound = slipx::relaxation_max_step(p.v_max, p.tyre_front.relax_length);

  // 2 * 0.08 / 20 is 0.008 s, which is exactly eight default steps. Eight and
  // not more: the margin is comfortable rather than generous, and if a future
  // default erodes it this is where that shows up.
  EXPECT_GE(bound, 8.0 * kDt);
  EXPECT_NEAR(bound, 8.0 * kDt, 1e-15);
  EXPECT_NEAR(bound, 2.0 * p.tyre_front.relax_length / p.v_max, 1e-15);
}

// ------------------------------------------------------ L2, the assembled tier
//
// The pieces were each checked on their own above. What is checked here is
// that the assembly puts them together the right way round: that the loads the
// tyres see are the loads the load transfer computed, that the forces land in
// the body frame with the right signs, and that the reported wheel speeds
// describe the same car as the forces.

// Driving straight, the four loads are the static ones and they sum to the
// weight. The first thing to check about a double-track model, and the one
// that catches a track width or a wheelbase used the wrong way round.
TEST(L2, StraightRunningReproducesTheStaticLoads) {
  const VehicleParams p = reference_params();
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  VehicleState s = travelling(4.0);
  StepDiagnostics d;
  for (int i = 0; i < 500; ++i) {
    model->step(s, DriveInput{0.0, hold_speed(s, 4.0)}, kDt, &d);
  }

  const slipx::WheelLoads statics = slipx::static_loads(p);
  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    // A small longitudinal transfer survives, because holding a speed against
    // drag needs a non-zero ax. It is bounded by that ax, not by nothing.
    EXPECT_NEAR(d.fz[i], statics.fz[i], 0.05) << "wheel " << i;
  }
  EXPECT_NEAR(d.fz[kFrontLeft], d.fz[kFrontRight], 1e-12) << "no lateral "
      << "transfer in a straight line";
  EXPECT_NEAR(d.fz[0] + d.fz[1] + d.fz[2] + d.fz[3], p.mass * slipx::kGravity,
              1e-9);
}

// ISO 8855, end to end through the whole tier rather than through
// load_transfer.hpp on its own: a LEFT turn is positive steer, gives positive
// ay, and loads the RIGHT-hand wheels. This is the sign that catches people
// and the one the whole convention rests on.
TEST(L2, ALeftTurnLoadsTheRightHandWheels) {
  const VehicleParams p = reference_params();
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  VehicleState s = travelling(5.0);
  StepDiagnostics d;
  for (int i = 0; i < 3000; ++i) {
    model->step(s, DriveInput{0.06, hold_speed(s, 5.0)}, kDt, &d);
  }

  EXPECT_GT(d.ay, 0.0) << "a left turn is positive ay in ISO 8855";
  EXPECT_GT(s.yaw_rate(), 0.0);
  EXPECT_GT(d.fz[kFrontRight], d.fz[kFrontLeft]);
  EXPECT_GT(d.fz[kRearRight], d.fz[kRearLeft]);
  EXPECT_GT(d.load_transfer_lat, 0.0);

  // Weight is still conserved with the load moved across.
  EXPECT_NEAR(d.fz[0] + d.fz[1] + d.fz[2] + d.fz[3], p.mass * slipx::kGravity,
              1e-9);

  // And the lateral force points left, which is the sign a negative slip angle
  // produces (conventions.hpp).
  EXPECT_GT(d.fy_front, 0.0);
  EXPECT_GT(d.fy_rear, 0.0);
  EXPECT_LT(d.alpha_front, 0.0);
}

// The reported wheel speeds and the reported slip ratios describe one car.
// omega R = v (1 + kappa) by the definition of slip ratio, so this is a
// consistency check on the quasi-static inversion (ADR-0027) rather than on
// the physics: it is what stops the two diagnostics drifting apart.
TEST(L2, WheelSpeedsAgreeWithTheReportedSlipRatios) {
  const VehicleParams p = reference_params();
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  // Held at a modest speed with a gentle steer, so the car stays well inside
  // the friction limit. A constant acceleration demand here runs the car past
  // its limit within a second and the ordering assertions below stop meaning
  // anything, which is how this case was first written.
  VehicleState s = travelling(5.0);
  StepDiagnostics d;
  for (int i = 0; i < 4000; ++i) {
    model->step(s, DriveInput{0.03, hold_speed(s, 5.0) + 0.4}, kDt, &d);
  }
  ASSERT_LT(std::fabs(d.ay), 0.5 * slipx::kGravity) << "operating point is "
      << "outside the region this case is about";

  // The wheel centre longitudinal velocities, recomputed here from the state
  // rather than read out of the model.
  const double r = s.yaw_rate();
  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    const bool front = (i == kFrontLeft || i == kFrontRight);
    const bool left = (i == kFrontLeft || i == kRearLeft);
    const double track = front ? p.track_front : p.track_rear;
    const double yw = left ? 0.5 * track : -0.5 * track;
    const double vxw = s.vx() - r * yw;

    EXPECT_NEAR(s.omega_w[i] * p.wheel_radius, vxw * (1.0 + d.kappa[i]), 1e-9)
        << "wheel " << i;
  }

  // Driving through the rear axle (the default layout), so the rear slip
  // ratios are positive and the undriven front wheels freewheel at exactly
  // zero slip: no torque, no slip, by the quasi-static inversion.
  EXPECT_GT(d.kappa[kRearLeft], 0.0);
  EXPECT_GT(d.kappa[kRearRight], 0.0);
  EXPECT_EQ(d.kappa[kFrontLeft], 0.0);
  EXPECT_EQ(d.kappa[kFrontRight], 0.0);

  // The outer wheels of a left turn travel further and therefore faster.
  //
  // Not as obvious as it looks, and worth the comment. The open differential
  // delivers equal force to both rear wheels, and the outer wheel carries
  // more load and so has a higher slip stiffness, so it needs LESS slip
  // ratio for the same force. That works against the geometry. Inside the
  // linear region the geometric term wins by an order of magnitude; past the
  // limit it need not, which is the other reason this case pins its
  // operating point.
  EXPECT_GT(s.omega_w[kFrontRight], s.omega_w[kFrontLeft]);
  EXPECT_GT(s.omega_w[kRearRight], s.omega_w[kRearLeft]);
  EXPECT_LT(d.kappa[kRearRight], d.kappa[kRearLeft]);
}

// The drive split is equal, so on a symmetric car it produces no yaw moment.
// A load-proportional split does, by putting more thrust on the loaded outside
// wheels, and it is worth about 2% of steady-state radius at 0.36 g. This case
// exists because that was found by measurement and not by inspection.
TEST(L2, HardAccelerationInAStraightLineProducesNoYaw) {
  const VehicleParams p = reference_params();
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  // Half a second only. Run it for three seconds and the car reaches v_max,
  // the speed clamp zeroes the demand, drag takes over and the load transfer
  // reverses, which is correct behaviour and not what this case is about.
  VehicleState s = travelling(2.0);
  StepDiagnostics d;
  for (int i = 0; i < 500; ++i) {
    model->step(s, DriveInput{0.0, p.accel_max}, kDt, &d);
  }
  ASSERT_FALSE(d.speed_saturated);
  ASSERT_FALSE(d.accel_saturated);

  EXPECT_EQ(s.yaw_rate(), 0.0);
  EXPECT_EQ(s.pos.y, 0.0);
  EXPECT_EQ(d.fx[kFrontLeft], d.fx[kFrontRight]);
  EXPECT_EQ(d.fx[kRearLeft], d.fx[kRearRight]);

  // Accelerating moves load to the rear, which is the other half of the
  // straight-line case.
  EXPECT_GT(d.fz_rear, d.fz_front);
  EXPECT_GT(d.load_transfer_long, 0.0);
}

// Braking in a corner costs cornering force overall, and it does NOT cost it
// evenly. Two mechanisms act at once and they pull in opposite directions:
//
//   the friction ellipse takes lateral force away from every tyre that is
//   also being asked for longitudinal force, which at L2 means the DRIVEN
//   axle only, because a 1/10-scale car brakes through its motor (ADR-0031);
//
//   longitudinal load transfer moves vertical load onto the front axle, which
//   RAISES the front tyres' budget and lowers the rear's.
//
// On the rear-driven default both mechanisms punish the rear together: it is
// braking AND being unloaded, while the front is braking not at all and
// gaining load. That is why lifting off mid-corner rotates a rear-driven car
// in, and it is the motor-braking version of the trail-braking story. The
// regen limit is raised here so the braking is strong enough to engage the
// ellipse; the provisional default's 0.23 g would not.
TEST(L2, BrakingInACornerMovesGripForwardAndCostsTheRearItsLateral) {
  VehicleParams p = reference_params();
  p.torque_per_amp = 0.05;  // regen cap 2 N m: about 1.1 g of rear braking

  auto settle = [&](double accel_cmd) {
    auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);
    VehicleState s = travelling(7.0);
    // Settle into the corner first, so the servo has reached the steer angle
    // and both runs differ only in the braking demand.
    StepDiagnostics d;
    for (int i = 0; i < 1000; ++i) {
      model->step(s, DriveInput{0.24, 0.0}, kDt, &d);
    }
    for (int i = 0; i < 40; ++i) {
      model->step(s, DriveInput{0.24, accel_cmd}, kDt, &d);
    }
    return d;
  };

  // Forty milliseconds of braking only, so the two runs are at nearly the
  // same speed and the comparison is about the friction budget rather than
  // about one car having slowed down more.
  const StepDiagnostics coasting = settle(0.0);
  const StepDiagnostics braking = settle(-p.decel_max);

  // The headline: less lateral acceleration.
  EXPECT_LT(std::fabs(braking.ay), std::fabs(coasting.ay));

  // The rear wheels are braking and at least one is at its budget; the front
  // wheels carry NO braking force at all, because there is nothing up there
  // to brake with.
  EXPECT_LT(braking.fx[kRearLeft], 0.0);
  EXPECT_LT(braking.fx[kRearRight], 0.0);
  EXPECT_EQ(braking.fx[kFrontLeft], 0.0);
  EXPECT_EQ(braking.fx[kFrontRight], 0.0);
  EXPECT_TRUE(braking.tyre_saturated[kRearLeft] ||
              braking.tyre_saturated[kRearRight]);

  // Load moved forward.
  EXPECT_GT(braking.fz_front, coasting.fz_front);
  EXPECT_LT(braking.fz_rear, coasting.fz_rear);

  // And the grip followed it: the rear axle gives up most of its lateral
  // force, while the front, braking nothing and carrying more load, keeps
  // everything it had.
  EXPECT_LT(std::fabs(braking.fy_rear), 0.75 * std::fabs(coasting.fy_rear));
  EXPECT_GE(std::fabs(braking.fy_front), std::fabs(coasting.fy_front));
}

// No slip angle, no load and no arithmetic anywhere in the tier may produce a
// NaN over a run that includes a limit-exceeding input. The friction ellipse
// and the peak force laws both have zero-load guards; this is the case that
// exercises them through the assembled model rather than in isolation.
TEST(L2, AWheelLiftingRoundATightCornerProducesNoNaN) {
  VehicleParams p = reference_params();
  p.h_cog = 0.14;   // deliberately tall, so the inside wheels lift early
  p.steer_max = 0.6;

  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);
  VehicleState s = travelling(9.0);
  StepDiagnostics d;

  bool lifted = false;
  for (int i = 0; i < 4000; ++i) {
    model->step(s, DriveInput{0.6, hold_speed(s, 9.0)}, kDt, &d);
    for (unsigned w = 0; w < slipx::kWheelCount; ++w) {
      ASSERT_FALSE(std::isnan(d.fz[w])) << "step " << i << " wheel " << w;
      ASSERT_FALSE(std::isnan(d.fy[w])) << "step " << i << " wheel " << w;
      ASSERT_FALSE(std::isnan(d.fx[w])) << "step " << i << " wheel " << w;
      ASSERT_FALSE(std::isnan(s.alpha_lag[w])) << "step " << i;
      if (d.fz[w] == 0.0) lifted = true;
    }
    ASSERT_FALSE(std::isnan(s.vx()));
    ASSERT_FALSE(std::isnan(s.yaw_rate()));
  }
  EXPECT_TRUE(lifted) << "the case did not reach the condition it exists to "
                         "test; raise h_cog or the speed";
}

// The reported per-wheel forces must explain the car's motion, including the
// yaw moment the LONGITUDINAL forces make. That term, -y_w Fx, is what lets a
// double-track model yaw under asymmetric braking, and a bicycle model has no
// way to produce it at all.
//
// It is invisible in every symmetric case, and the open differential keeps it
// invisible even in a corner, because equal torque both sides is the whole
// point of an open diff. So this case runs a SPOOL, whose locked axle drives
// the inner wheel harder in a corner (ADR-0031), checks the asymmetry is
// really there, and then closes the moment balance against the yaw
// acceleration the model actually produced.
TEST(L2, ThePerWheelForcesExplainTheYawAccelerationIncludingTheirFxTerm) {
  VehicleParams p = reference_params();
  p.steer_max = 0.5;
  p.differential = slipx::Differential::kSpool;
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p,
                                    slipx::Integrator::kSemiImplicitEuler);

  VehicleState s = travelling(9.0);
  StepDiagnostics d;
  for (int i = 0; i < 300; ++i) {
    model->step(s, DriveInput{0.42, 4.0}, kDt, &d);
  }

  // The asymmetry this case depends on. Without it the fx term cancels and
  // the assertion below would pass whether or not the term exists.
  const double fx_asym = std::fabs(d.fx[kFrontLeft] - d.fx[kFrontRight]) +
                         std::fabs(d.fx[kRearLeft] - d.fx[kRearRight]);
  ASSERT_GT(fx_asym, 0.5) << "left and right must differ for this case to "
                             "test anything";

  // One more very small step, so the yaw acceleration over it is close to the
  // instantaneous value the reported forces describe.
  const double r_before = s.yaw_rate();
  const double tiny = 1e-7;
  model->step(s, DriveInput{0.42, 4.0}, tiny, &d);
  const double yaw_accel = (s.yaw_rate() - r_before) / tiny;

  // The moment, rebuilt from the diagnostics rather than read from the model,
  // at the ACHIEVED steer angle: the servo means the road wheels are not
  // where the command asked (ADR-0031), and the forces were resolved through
  // where they actually are.
  const double cos_d = std::cos(s.steer);
  const double sin_d = std::sin(s.steer);
  double mz = 0.0;
  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    const bool front = (i == kFrontLeft || i == kFrontRight);
    const bool left = (i == kFrontLeft || i == kRearLeft);
    const double xw = front ? p.lf : -p.lr;
    const double track = front ? p.track_front : p.track_rear;
    const double yw = left ? 0.5 * track : -0.5 * track;
    const double ci = front ? cos_d : 1.0;
    const double si = front ? sin_d : 0.0;
    const double fx_b = d.fx[i] * ci - d.fy[i] * si;
    const double fy_b = d.fx[i] * si + d.fy[i] * ci;
    mz += xw * fy_b - yw * fx_b;
  }

  // Relative, because yaw_accel is a forward difference over `tiny` and its
  // truncation error is proportional to the yaw jerk over that step, not to
  // anything about the identity being tested. Round-off in the quotient is
  // three orders of magnitude below this.
  EXPECT_NEAR(yaw_accel, mz / p.izz, 1e-5 * std::fabs(mz / p.izz) + 1e-6);

  // And the lateral equation closes too, which is the same statement for the
  // other degree of freedom.
  double fy_total = 0.0;
  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    const bool front = (i == kFrontLeft || i == kFrontRight);
    const double ci = front ? cos_d : 1.0;
    const double si = front ? sin_d : 0.0;
    fy_total += d.fx[i] * si + d.fy[i] * ci;
  }
  EXPECT_NEAR(d.ay, fy_total / p.mass, 1e-9);
}

// The tyre transient, through the assembled tier rather than in
// relaxation.hpp on its own. Every other L2 case here settles first, and in
// steady state the lagged slip angle equals the instantaneous one, so none of
// them can tell whether the tier uses the state it carries.
TEST(L2, TheForceFollowsTheLaggedSlipAngleAndNotTheInstantaneousOne) {
  const VehicleParams p = reference_params();
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  // Straight and settled, so every lagged angle starts at zero.
  VehicleState s = travelling(8.0);
  StepDiagnostics d;
  for (int i = 0; i < 500; ++i) {
    model->step(s, DriveInput{0.0, hold_speed(s, 8.0)}, kDt, &d);
  }
  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    ASSERT_NEAR(s.alpha_lag[i], 0.0, 1e-9);
  }

  // Now step the steering and take a handful of steps, far less than one
  // relaxation length of travel.
  for (int i = 0; i < 3; ++i) {
    model->step(s, DriveInput{0.12, hold_speed(s, 8.0)}, kDt, &d);
  }

  const slipx::WheelLoads statics = slipx::static_loads(p);
  const MfLite front = slipx::make_mf_lite(p.tyre_front, 0.5 * p.c_alpha_f,
                                           statics.fz[kFrontLeft]);

  for (const unsigned i : {kFrontLeft, kFrontRight}) {
    // The tyre has not caught up: it carries a fraction of the slip the
    // geometry is asking for.
    ASSERT_LT(std::fabs(s.alpha_lag[i]), 0.5 * std::fabs(d.alpha[i]))
        << "wheel " << i;

    // And the force it is making is the one the LAGGED angle produces, not the
    // one the instantaneous angle would. Compared against both, so the case
    // says which of the two it is rather than only that it is small.
    const double from_lagged = mf_lite_fy(front, s.alpha_lag[i], d.fz[i]);
    const double from_instant = mf_lite_fy(front, d.alpha[i], d.fz[i]);
    EXPECT_NEAR(d.fy[i], from_lagged, 1e-6 * std::fabs(from_lagged) + 1e-9)
        << "wheel " << i;
    EXPECT_GT(std::fabs(from_instant), 2.0 * std::fabs(d.fy[i]))
        << "wheel " << i;
  }
}

// A longer relaxation length is a slower car. Asserted at the vehicle level,
// where it is the thing a driver or a controller would notice.
//
// Note what is NOT asserted here. The tyre's lag is a distance, but the car's
// yaw response also carries its own lag from yaw inertia against tyre
// stiffness, and that one is a time constant. The two are in series, so
// vehicle yaw response is not distance-invariant and travelling the same
// distance at two speeds does not reach the same fraction of it: measured on
// the reference car, 0.2 m of travel gets to 78% of steady-state yaw at 4 m/s
// and 32% at 12 m/s. That mixture is exactly why identifying sigma from a step
// steer needs runs at two speeds to separate the two terms. Distance
// invariance holds at the tyre, and test_analytical.cpp's Relaxation section
// asserts it there.
TEST(L2, ALongerRelaxationLengthDelaysTheYawResponse) {
  auto yaw_fraction_after = [](double sigma, double v, double travel) {
    VehicleParams p = reference_params();
    p.tyre_front.relax_length = sigma;
    p.tyre_rear.relax_length = sigma;
    // A near-instant servo, so the response being measured is the tyre's lag
    // and not the actuator's. Separating the two is exactly what this case
    // is for; the servo has its own cases.
    p.steer_bandwidth = 400.0;
    p.steer_rate_max = 45.0;

    auto settled = [&] {
      auto m = VehicleModel::create(Tier::L2_DoubleTrack, p);
      VehicleState t = travelling(v);
      StepDiagnostics dd;
      for (int i = 0; i < 8000; ++i) {
        m->step(t, DriveInput{0.06, hold_speed(t, v)}, kDt, &dd);
      }
      return t.yaw_rate();
    }();

    auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);
    VehicleState s = travelling(v);
    StepDiagnostics d;
    for (int i = 0; i < 500; ++i) {
      model->step(s, DriveInput{0.0, hold_speed(s, v)}, kDt, &d);
    }
    const int steps = static_cast<int>(travel / (v * kDt) + 0.5);
    for (int i = 0; i < steps; ++i) {
      model->step(s, DriveInput{0.06, hold_speed(s, v)}, kDt, &d);
    }
    return s.yaw_rate() / settled;
  };

  // Same speed, three relaxation lengths, monotonically slower.
  const double quick = yaw_fraction_after(0.04, 8.0, 0.10);
  const double mid = yaw_fraction_after(0.10, 8.0, 0.10);
  const double slow = yaw_fraction_after(0.20, 8.0, 0.10);

  EXPECT_GT(quick, mid);
  EXPECT_GT(mid, slow);
  EXPECT_GT(quick - slow, 0.05) << "quick " << quick << " slow " << slow;
}

// --------------------------------------- L2 drivetrain and actuators
//
// The ADR-0031 slice: servo, ESC, battery and differential, each pinned
// against its closed form or its defining behaviour. The ideal-supply
// configuration (pack_v_full = pack_v_empty = pack_nominal_v, zero internal
// resistance) appears throughout because it makes the voltage scale exactly
// one, which turns "the curve" into an exact claim rather than a tolerance.

// The servo dynamics are decoupled from the vehicle states by construction,
// so the achieved angle must match the closed-form response of a linear
// second-order system exactly as long as neither the slew limit nor the
// travel stop engages. RK4 at 1 kHz integrates a 45 rad/s oscillator to well
// below 1e-6 absolute, so the tolerance here is integration error, not slack.
TEST(L2Servo, SmallStepResponseMatchesTheSecondOrderClosedForm) {
  const VehicleParams p = reference_params();
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  const double cmd = 0.10;  // peak unlimited rate about 2.6 rad/s, well
                            // inside the 10 rad/s slew limit
  VehicleState s = travelling(5.0);

  const double wn = p.steer_bandwidth;
  const double zeta = p.steer_damping;
  const double wd = wn * std::sqrt(1.0 - zeta * zeta);
  const auto closed_form = [&](double t) {
    return cmd * (1.0 - std::exp(-zeta * wn * t) *
                            (std::cos(wd * t) +
                             (zeta / std::sqrt(1.0 - zeta * zeta)) *
                                 std::sin(wd * t)));
  };

  int steps = 0;
  for (const double t_check : {0.02, 0.05, 0.10, 0.30}) {
    const int target = static_cast<int>(std::lround(t_check / kDt));
    for (; steps < target; ++steps) {
      model->step(s, DriveInput{cmd, 0.0}, kDt, nullptr);
    }
    EXPECT_NEAR(s.steer, closed_form(t_check), 1e-6) << "at t " << t_check;
  }
}

// A command too large for the servo's slew rate makes the angle ramp at the
// limit rather than follow the second-order shape: the defining signature of
// a rate-limited actuator, and the reason a fast chicane at 1/10 scale is
// steered slower than the controller asked.
//
// The servo here is deliberately stiffer than the provisional default. At
// 45 rad/s of bandwidth the damping term holds the rate under the slew limit
// for all but a few milliseconds of the largest legal step, so the default
// servo is bandwidth-limited, not slew-limited; a stiff one winds the rate
// state well past the limit and holds a clean plateau there.
TEST(L2Servo, ALargeStepIsSlewRateLimited) {
  VehicleParams p = reference_params();
  p.steer_bandwidth = 200.0;
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  VehicleState s = travelling(5.0);
  double previous = s.steer;
  double max_slope = 0.0;
  for (int i = 0; i < 36; ++i) {
    model->step(s, DriveInput{0.40, 0.0}, kDt, nullptr);
    const double slope = (s.steer - previous) / kDt;
    max_slope = std::fmax(max_slope, slope);
    // The limit is a limit: the angle never moves faster than max_rate, at
    // any step, whatever the internal rate state wound up to.
    ASSERT_LE(slope, p.steer_rate_max + 1e-9) << "step " << i;
    previous = s.steer;
  }

  // Not there yet after 36 ms: 0.4 rad at 10 rad/s takes 40 ms, although the
  // unlimited 200 rad/s response would long since have arrived. The ramp is
  // what a rate limit looks like.
  EXPECT_LT(s.steer, 0.37);

  // And the limit genuinely engaged rather than the command being too small
  // to reach it: the plateau runs at max_rate.
  EXPECT_GT(max_slope, 0.99 * p.steer_rate_max);
}

// The mechanical end stop is inelastic: the rack arrives and stops dead. The
// command is clipped to travel, so the only way to reach the stop is the
// servo's own overshoot, and an underdamped servo commanded to full lock
// overshoots by exp(-pi zeta / sqrt(1 - zeta^2)) and therefore does reach it.
//
// The discriminating quantity is the RATE, not the angle. Without the rate
// dying at the stop, the angle is pinned by the travel clamp anyway and looks
// identical; what persists is a rate state reporting a mechanism moving into
// a stop it is already resting against, which then has to be unwound before
// the servo can come off the lock.
TEST(L2Servo, TheTravelStopIsInelasticAndKillsTheRate) {
  const VehicleParams p = reference_params();
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  const double zeta = p.steer_damping;
  const double overshoot =
      std::exp(-slipx::kPi * zeta / std::sqrt(1.0 - zeta * zeta));
  ASSERT_GT(p.steer_max * (1.0 + overshoot), p.steer_max)
      << "an overdamped servo never reaches the stop and this case is void";

  VehicleState s = travelling(5.0);
  StepDiagnostics d;
  // Command beyond the travel; it is clipped to steer_max on the way in, so
  // the stop is reached by overshoot rather than by the command.
  for (int i = 0; i < 600; ++i) {
    model->step(s, DriveInput{2.0 * p.steer_max, 0.0}, kDt, &d);
  }

  EXPECT_EQ(s.steer, p.steer_max) << "the travel stop must hold the angle";
  EXPECT_EQ(s.steer_rate, 0.0)
      << "resting against an inelastic stop is not motion";

  // And it is genuinely at rest rather than momentarily crossing zero: it
  // stays there, step after step, with the command still pushing.
  for (int i = 0; i < 50; ++i) {
    model->step(s, DriveInput{2.0 * p.steer_max, 0.0}, kDt, &d);
    ASSERT_EQ(s.steer, p.steer_max) << "step " << i;
    ASSERT_EQ(s.steer_rate, 0.0) << "step " << i;
  }

  // The same on the other lock, because the stop is symmetric.
  for (int i = 0; i < 600; ++i) {
    model->step(s, DriveInput{-2.0 * p.steer_max, 0.0}, kDt, &d);
  }
  EXPECT_EQ(s.steer, -p.steer_max);
  EXPECT_EQ(s.steer_rate, 0.0);
}

// The ESC's torque-speed curve, exactly, through the assembled tier: settle
// at a speed, read the driven wheels' speeds out of the state, demand more
// than the curve can give, and the delivered torque must equal the curve at
// that speed capped by the current limit. Ideal supply, so the voltage scale
// is exactly one and EXPECT_EQ means equal.
TEST(L2Esc, DeliveredTorqueFollowsTheCurveAndTheCurrentCap) {
  VehicleParams p = reference_params();
  p.pack_v_full = p.pack_nominal_v;
  p.pack_v_empty = p.pack_nominal_v;
  p.pack_internal_resistance = 0.0;
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  for (const double v : {2.0, 6.0, 10.0, 14.0}) {
    VehicleState s = travelling(v);
    StepDiagnostics d;
    for (int i = 0; i < 2000; ++i) {
      model->step(s, DriveInput{0.0, hold_speed(s, v)}, kDt, &d);
    }

    // The budget the next step must grant, from the state it will read.
    const double omega_mean =
        0.5 * (s.omega_w[slipx::kRearLeft] + s.omega_w[slipx::kRearRight]);
    const double curve =
        p.torque_stall * (1.0 - omega_mean / p.omega_free);
    const double expected =
        std::fmin(curve, p.torque_per_amp * p.current_max);

    model->step(s, DriveInput{0.0, p.accel_max}, kDt, &d);
    EXPECT_EQ(d.drive_torque, expected) << "at " << v << " m/s";
    EXPECT_TRUE(d.esc_saturated) << "the demand of 1.4 N m exceeds the "
                                    "budget everywhere in this sweep";
    EXPECT_FALSE(d.accel_saturated) << "accel_max itself was not exceeded";
  }
}

// At launch the curve is worth its stall value and the current limit is what
// actually binds: 1.2 N m against a 2.0 N m stall on the provisional set.
// This is the observable difference between a torque-limited and a
// current-limited drivetrain, and it is why the limit is a schema field.
TEST(L2Esc, TheCurrentLimitBindsAtLaunch) {
  VehicleParams p = reference_params();
  p.pack_v_full = p.pack_nominal_v;
  p.pack_v_empty = p.pack_nominal_v;
  p.pack_internal_resistance = 0.0;
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  VehicleState s = at_rest();
  StepDiagnostics d;
  model->step(s, DriveInput{0.0, p.accel_max}, kDt, &d);

  EXPECT_EQ(d.drive_torque, p.torque_per_amp * p.current_max);
  EXPECT_TRUE(d.esc_saturated);
  EXPECT_LT(d.drive_torque, p.torque_stall) << "the curve did not bind";
}

// Braking is regen, regen has its own limit, and that limit is the only
// brake the car has (ADR-0031). decel_max remains a command bound; on the
// provisional numbers the regen cap of 0.4 N m is about 0.23 g, which is
// weak and honestly so.
TEST(L2Esc, BrakingIsCappedByTheRegenLimitNotDecelMax) {
  VehicleParams p = reference_params();
  p.pack_v_full = p.pack_nominal_v;
  p.pack_v_empty = p.pack_nominal_v;
  p.pack_internal_resistance = 0.0;
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  VehicleState s = travelling(8.0);
  StepDiagnostics d;
  model->step(s, DriveInput{0.0, -p.decel_max}, kDt, &d);

  EXPECT_EQ(d.drive_torque, -p.torque_per_amp * p.regen_current_max);
  EXPECT_TRUE(d.esc_saturated);
  EXPECT_FALSE(d.accel_saturated) << "-decel_max is a legal command; the "
                                     "ESC is what could not deliver it";

  // About 0.23 g, delivered through the rear axle only.
  const double decel = d.drive_torque / (p.wheel_radius * p.mass);
  EXPECT_NEAR(decel, -2.29, 0.05);
}

// The battery, isolated: driving sags the terminal voltage below the
// open-circuit value and drains the state of charge monotonically; coasting
// holds the voltage at open circuit exactly, because no power is flowing.
TEST(L2Battery, SagUnderLoadAndMonotoneSocDecay) {
  const VehicleParams p = reference_params();
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  const auto ocv = [&](double soc) {
    return p.pack_v_empty + soc * (p.pack_v_full - p.pack_v_empty);
  };

  VehicleState s = travelling(4.0);
  StepDiagnostics d;
  double soc_prev = s.soc;
  int sagging_steps = 0;
  for (int i = 0; i < 3000; ++i) {
    model->step(s, DriveInput{0.0, p.accel_max}, kDt, &d);
    // Sag needs current, and the battery current is the electrical power
    // over the voltage, so a stalled wheel draws nothing: the very first
    // step, taken at zero wheel speed, sags nothing and that is the model
    // rather than a bug (ADR-0031).
    if (d.pack_current > 0.0) {
      ASSERT_LT(s.pack_v, ocv(s.soc)) << "step " << i;
      ++sagging_steps;
    }
    ASSERT_LE(s.soc, soc_prev) << "step " << i;
    soc_prev = s.soc;
  }
  EXPECT_GT(sagging_steps, 2900) << "the run must actually have been under "
                                    "load for the assertions to have bitten";
  EXPECT_LT(s.soc, 1.0 - 1e-5) << "three seconds of full throttle must "
                                  "visibly cost charge";

  // Coasting: no demand, no current, no sag.
  model->step(s, DriveInput{0.0, 0.0}, kDt, &d);
  EXPECT_EQ(d.drive_torque, 0.0);
  EXPECT_EQ(d.pack_current, 0.0);
  EXPECT_EQ(s.pack_v, ocv(s.soc));
}

// A drained pack is a slower car: the open-circuit voltage falls with state
// of charge, the voltage scale drops below one, and the whole curve comes
// down with it. This is the mechanism that makes the last lap slower than
// the first, and it must be visible in the delivered torque.
TEST(L2Battery, ALowerStateOfChargeDeliversLessTorque) {
  const VehicleParams p = reference_params();
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  // 14 m/s, high enough on the curve that the current limit is not the
  // binding cap at any state of charge here; at low speed it would hide the
  // voltage effect entirely, which is itself asserted in the launch case.
  const auto torque_at = [&](double soc) {
    VehicleState s = travelling(14.0);
    s.soc = soc;
    // One settling step so the wheel speeds match the velocity, then read
    // the delivered torque under full demand.
    StepDiagnostics d;
    model->step(s, DriveInput{0.0, 0.0}, kDt, &d);
    model->step(s, DriveInput{0.0, p.accel_max}, kDt, &d);
    return d.drive_torque;
  };

  const double full = torque_at(1.0);
  const double half = torque_at(0.5);
  const double low = torque_at(0.1);
  EXPECT_GT(full, half);
  EXPECT_GT(half, low);
  EXPECT_GT(low, 0.0) << "a low pack is weak, not absent; there is no "
                         "low-voltage cutoff in the model (ADR-0031)";
}

// The no-battery fixture: an ideal supply must reproduce the bare curve
// EXACTLY, not within a woolly tolerance. This is the case that proves the
// battery model composes with the ESC rather than leaking into it.
TEST(L2Battery, AnIdealSupplyReproducesTheBareCurveExactly) {
  VehicleParams p = reference_params();
  p.pack_v_full = p.pack_nominal_v;
  p.pack_v_empty = p.pack_nominal_v;
  p.pack_internal_resistance = 0.0;
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  VehicleState s = travelling(9.0);
  StepDiagnostics d;
  for (int i = 0; i < 400; ++i) {
    model->step(s, DriveInput{0.0, hold_speed(s, 9.0)}, kDt, &d);
  }
  const double omega_mean =
      0.5 * (s.omega_w[slipx::kRearLeft] + s.omega_w[slipx::kRearRight]);
  model->step(s, DriveInput{0.0, p.accel_max}, kDt, &d);

  const double curve = p.torque_stall * (1.0 - omega_mean / p.omega_free);
  EXPECT_EQ(d.drive_torque,
            std::fmin(curve, p.torque_per_amp * p.current_max));
  EXPECT_EQ(s.pack_v, p.pack_nominal_v) << "no sag without resistance";
}

// Sag is not merely reported, it is FED BACK: the second pass re-evaluates the
// torque budget at the sagged terminal voltage, so what the ESC delivers is
// the curve at pack_v and not the curve at the open-circuit voltage. Skipping
// the second pass leaves the two agreeing on the voltage and disagreeing on
// the torque, which is why this case pins the torque against the reported
// voltage rather than against a tolerance.
//
// A large internal resistance is used deliberately. At the provisional 20
// mohm the two passes differ by about 10%, which a tolerance could swallow;
// this makes the effect unmissable and the equality still exact.
TEST(L2Battery, TheSaggedVoltageRescalesTheTorqueBudget) {
  VehicleParams p = reference_params();
  p.pack_internal_resistance = 0.08;
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  // High enough on the curve that the curve binds rather than the current
  // limit: below the knee the cap hides the voltage entirely.
  const double v = 14.0;
  VehicleState s = travelling(v);
  StepDiagnostics d;
  for (int i = 0; i < 2000; ++i) {
    model->step(s, DriveInput{0.0, hold_speed(s, v)}, kDt, &d);
  }

  const double omega_mean =
      0.5 * (s.omega_w[slipx::kRearLeft] + s.omega_w[slipx::kRearRight]);
  const double ocv =
      p.pack_v_empty + s.soc * (p.pack_v_full - p.pack_v_empty);

  model->step(s, DriveInput{0.0, p.accel_max}, kDt, &d);

  const auto curve_at = [&](double v_pack) {
    const double scale = v_pack / p.pack_nominal_v;
    return p.torque_stall * scale *
           (1.0 - omega_mean / (p.omega_free * scale));
  };

  ASSERT_LT(s.pack_v, ocv) << "the case must actually have sagged";
  ASSERT_LT(curve_at(ocv), p.torque_per_amp * p.current_max)
      << "the curve must be the binding cap, not the current limit";

  // The delivered torque is the curve at the SAGGED voltage, exactly.
  EXPECT_EQ(d.drive_torque, curve_at(s.pack_v));
  // And that is a materially different number from the curve at open circuit,
  // so the equality above is not vacuous.
  EXPECT_LT(d.drive_torque, 0.9 * curve_at(ocv));
}

// Drivetrain efficiency is a loss in BOTH directions: driving, the pack must
// supply more than the wheels receive; regenerating, the pack recovers less
// than the wheels give up. An efficiency that only divides is half a model,
// and on an ideal supply the two cases pin exactly, because halving the
// efficiency leaves the torque budget untouched and scales the pack current
// by exactly two one way and one half the other.
TEST(L2Battery, EfficiencyLosesPowerInBothDirections) {
  const auto current_at = [](double efficiency, double accel_cmd) {
    VehicleParams p = reference_params();
    p.pack_v_full = p.pack_nominal_v;
    p.pack_v_empty = p.pack_nominal_v;
    p.pack_internal_resistance = 0.0;
    p.drive_efficiency = efficiency;
    auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

    VehicleState s = travelling(8.0);
    StepDiagnostics d;
    for (int i = 0; i < 2000; ++i) {
      model->step(s, DriveInput{0.0, hold_speed(s, 8.0)}, kDt, &d);
    }
    model->step(s, DriveInput{0.0, accel_cmd}, kDt, &d);
    return std::make_pair(d.pack_current, d.drive_torque);
  };

  const VehicleParams ref = reference_params();

  // Driving: the same wheel torque, drawn through half the efficiency, is
  // exactly twice the pack current.
  const auto lossless_drive = current_at(1.0, ref.accel_max);
  const auto lossy_drive = current_at(0.5, ref.accel_max);
  ASSERT_GT(lossless_drive.first, 0.0);
  EXPECT_EQ(lossy_drive.second, lossless_drive.second)
      << "an ideal supply's budget must not depend on efficiency";
  EXPECT_EQ(lossy_drive.first, 2.0 * lossless_drive.first);

  // Regenerating: the same wheel torque, recovered through half the
  // efficiency, returns exactly half the current to the pack.
  const auto lossless_regen = current_at(1.0, -ref.decel_max);
  const auto lossy_regen = current_at(0.5, -ref.decel_max);
  ASSERT_LT(lossless_regen.first, 0.0) << "regen must charge the pack";
  EXPECT_EQ(lossy_regen.second, lossless_regen.second);
  EXPECT_EQ(lossy_regen.first, 0.5 * lossless_regen.first);
}

// The open differential: equal torque both sides of the driven axle, in a
// corner where the loads are anything but equal. That equality is the no-yaw
// -moment property the measured equal split of ADR-0027 demanded, now
// produced by an actual differential model.
TEST(L2Differential, AnOpenDiffDeliversEqualForceInACorner) {
  const VehicleParams p = reference_params();  // open, rear drive
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  VehicleState s = travelling(6.0);
  StepDiagnostics d;
  for (int i = 0; i < 3000; ++i) {
    model->step(s, DriveInput{0.06, hold_speed(s, 6.0) + 0.3}, kDt, &d);
  }
  ASSERT_GT(d.fz[kRearRight], d.fz[kRearLeft]) << "the corner must load the "
                                                  "outside for this to test "
                                                  "anything";
  EXPECT_GT(d.fx[kRearLeft], 0.0);
  EXPECT_EQ(d.fx[kRearLeft], d.fx[kRearRight]);
  EXPECT_EQ(d.fx[kFrontLeft], 0.0) << "nothing drives the front axle";
  EXPECT_EQ(d.fx[kFrontRight], 0.0);
}

// And the open diff's defining failure: lift the inside wheel and the whole
// axle delivers nothing, because the unloaded side can support no torque and
// equal torque means the loaded side gets the same nothing. The teaching
// artefact, asserted.
TEST(L2Differential, AnOpenDiffIsHelplessWithAWheelInTheAir) {
  VehicleParams p = reference_params();
  // A deliberately top-heavy fixture, not a plausible car: with a 0.24 m
  // track this puts the static rollover threshold at 0.46 g, well under what
  // the tyres deliver, which is the only way a 1/10-scale car lifts anything
  // without a kerb. Half a radian of steer is NOT the way to get there: the
  // car understeers and scrubs instead of loading the outside, so the steer
  // is the smaller angle that actually reaches the condition.
  p.h_cog = 0.26;
  p.steer_max = 0.6;
  p.steer_bandwidth = 2000.0;  // instant steer; the servo is not the point
  p.steer_rate_max = 500.0;
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  VehicleState s = travelling(5.0);
  StepDiagnostics d;
  bool lifted_under_demand = false;
  for (int i = 0; i < 3000; ++i) {
    model->step(s, DriveInput{0.35, 4.0}, kDt, &d);
    if (d.fz[kRearLeft] == 0.0 && d.drive_torque > 0.0) {
      lifted_under_demand = true;
      EXPECT_EQ(d.fx[kRearLeft], 0.0) << "step " << i;
      EXPECT_EQ(d.fx[kRearRight], 0.0)
          << "the loaded wheel must be held to the lifted wheel's nothing";
    }
  }
  ASSERT_TRUE(lifted_under_demand) << "the case never reached the condition "
                                      "it exists to test";
}

// The spool: one axle speed, so the slower inner wheel runs the higher slip
// ratio and drives harder. The force asymmetry is inboard, its yaw moment
// fights the corner, and the sum still meets the demand.
TEST(L2Differential, ASpoolDrivesTheInnerWheelHarder) {
  VehicleParams p = reference_params();
  p.differential = slipx::Differential::kSpool;
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);

  VehicleState s = travelling(6.0);
  StepDiagnostics d;
  for (int i = 0; i < 3000; ++i) {
    model->step(s, DriveInput{0.06, hold_speed(s, 6.0) + 0.3}, kDt, &d);
  }
  ASSERT_FALSE(d.tyre_saturated[kRearLeft] || d.tyre_saturated[kRearRight])
      << "this case is about the constraint, not the friction cap";

  // Left turn: the left wheels are inner and slower.
  EXPECT_GT(d.fx[kRearLeft], d.fx[kRearRight]);
  EXPECT_GT(d.kappa[kRearLeft], d.kappa[kRearRight]);

  // The split still sums to what the ESC granted.
  EXPECT_NEAR(d.fx[kRearLeft] + d.fx[kRearRight],
              d.drive_torque / p.wheel_radius, 1e-9);
}

// Coasting, a spool scrubs: with no demand at all the locked axle drags the
// outer wheel and drives the inner one, and the resulting yaw moment pushes
// the car wide. An open diff does neither. This is the turn-in difference a
// driver feels between the two, expressed as path radius.
TEST(L2Differential, ASpoolScrubsAndUndersteersWhereAnOpenDiffRollsFree) {
  const auto settled = [](slipx::Differential diff) {
    VehicleParams p = reference_params();
    p.differential = diff;
    auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);
    VehicleState s = travelling(5.0);
    StepDiagnostics d;
    for (int i = 0; i < 8000; ++i) {
      model->step(s, DriveInput{0.08, hold_speed(s, 5.0)}, kDt, &d);
    }
    struct Out {
      double radius;
      double fx_inner;
      double fx_outer;
    };
    return Out{s.speed() / std::fabs(s.yaw_rate()), d.fx[kRearLeft],
               d.fx[kRearRight]};
  };

  const auto open = settled(slipx::Differential::kOpen);
  const auto spool = settled(slipx::Differential::kSpool);

  // The scrub: inner wheel driving, outer wheel dragging, with only the
  // speed-hold's trickle of demand.
  EXPECT_GT(spool.fx_inner, spool.fx_outer);
  EXPECT_LT(spool.fx_outer, 0.0) << "the outer wheel is dragged";

  // And the car runs wider for the same steer: understeer push.
  EXPECT_GT(spool.radius, 1.01 * open.radius)
      << "open " << open.radius << " spool " << spool.radius;
}

// The preloaded LSD is a spool until the axle torque difference exceeds the
// preload, and past that it transfers exactly the preload toward the slower
// wheel. Both regimes pinned; the huge-preload case must be bit-identical to
// the spool, because it takes the same code path with the same numbers.
TEST(L2Differential, AnLsdIsASpoolInsideThePreloadAndTransfersItPastIt) {
  const auto run = [](slipx::Differential diff, double preload) {
    VehicleParams p = reference_params();
    p.differential = diff;
    p.lsd_preload = preload;
    auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);
    VehicleState s = travelling(6.0);
    StepDiagnostics d;
    for (int i = 0; i < 4000; ++i) {
      model->step(s, DriveInput{0.10, hold_speed(s, 6.0)}, kDt, &d);
    }
    return std::make_pair(s, d);
  };

  // Locked regime: a preload no corner can exceed.
  const auto spool = run(slipx::Differential::kSpool, 0.0);
  const auto locked = run(slipx::Differential::kLsd, 10.0);
  EXPECT_EQ(spool.first.pos.x, locked.first.pos.x);
  EXPECT_EQ(spool.first.pos.y, locked.first.pos.y);
  EXPECT_EQ(spool.first.yaw, locked.first.yaw);
  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    EXPECT_EQ(spool.second.fx[i], locked.second.fx[i]) << "wheel " << i;
  }

  // Slipping regime: a small preload, exceeded in this corner, so the axle
  // holds exactly the preload's worth of force difference, slow side up.
  const double preload = 0.02;
  const auto slipping = run(slipx::Differential::kLsd, preload);
  ASSERT_FALSE(slipping.second.tyre_saturated[kRearLeft] ||
               slipping.second.tyre_saturated[kRearRight]);
  const double diff_force =
      slipping.second.fx[kRearLeft] - slipping.second.fx[kRearRight];
  EXPECT_NEAR(diff_force * reference_params().wheel_radius, preload, 1e-12)
      << "the clutch transfers the preload toward the slower inner wheel";
}

// The layouts: who gets driven, who freewheels. The 4WD centre is locked
// 50/50 (ADR-0031), so under straight-line drive the two axles' forces
// match.
TEST(L2Differential, TheLayoutsDriveTheAxlesTheyName) {
  const auto forces = [](slipx::DriveLayout layout) {
    VehicleParams p = reference_params();
    p.layout = layout;
    auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);
    VehicleState s = travelling(4.0);
    StepDiagnostics d;
    for (int i = 0; i < 300; ++i) {
      model->step(s, DriveInput{0.0, 4.0}, kDt, &d);
    }
    return d;
  };

  const StepDiagnostics rwd = forces(slipx::DriveLayout::kRearWheelDrive);
  EXPECT_GT(rwd.fx[kRearLeft], 0.0);
  EXPECT_EQ(rwd.fx[kFrontLeft], 0.0);

  const StepDiagnostics fwd = forces(slipx::DriveLayout::kFrontWheelDrive);
  EXPECT_GT(fwd.fx[kFrontLeft], 0.0);
  EXPECT_EQ(fwd.fx[kRearLeft], 0.0);

  const StepDiagnostics awd = forces(slipx::DriveLayout::kAllWheelDrive);
  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    EXPECT_GT(awd.fx[i], 0.0) << "wheel " << i;
  }
  EXPECT_NEAR(awd.fx[kFrontLeft] + awd.fx[kFrontRight],
              awd.fx[kRearLeft] + awd.fx[kRearRight], 1e-9)
      << "a locked centre splits 50/50";
}

}  // namespace
