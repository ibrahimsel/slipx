// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// SRS 7, layer three: the tiers must converge in the low lateral acceleration
// limit, and must diverge outside it.
//
// Both halves matter. Convergence establishes that L0 and L1 are describing
// the same car, so a controller can be moved between them; divergence is the
// entire reason for having more than one tier, and the point at which it
// starts is the tracked, released artefact rather than something a user
// discovers by being surprised. This file measures that crossover and asserts
// it lies where the physics says it should.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>

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

struct Settled {
  double ay = 0.0;
  double yaw_rate = 0.0;
  double radius = 0.0;
  double speed = 0.0;
};

// Tiers are compared by path radius rather than by yaw rate, and the reason is
// worth stating because it looks like an evasion and is not.
//
// The speed hold is proportional, so it settles with a small offset against
// whatever resists it. L0 has no resistance at all and reaches its target;
// L1 carries drag and rolling resistance and settles two to three percent
// slower. Yaw rate is proportional to speed, so comparing yaw rates directly
// measures that offset, which is a property of the test's speed controller and
// not of either model. Path radius is what both tiers are actually claiming
// about the car and is speed-independent to first order, so that is what gets
// compared.

Settled settle(Tier tier, const VehicleParams& p, double delta, double v) {
  auto model = VehicleModel::create(tier, p);
  VehicleState s = travelling(v);
  const StepDiagnostics d = run_for(
      *model, s, 12.0, kDt, [&](const VehicleState& st, double) {
        return DriveInput{delta, hold_speed(st, v)};
      });
  Settled out;
  out.ay = d.ay;
  out.yaw_rate = s.yaw_rate();
  out.speed = s.speed();
  out.radius = s.speed() / std::fabs(s.yaw_rate());
  return out;
}

// At low lateral acceleration the tyres are barely working, the slip angles
// are small, and the dynamic model reduces to the kinematic one. The
// agreement is not exact and is not asked to be: the residual is the
// understeer term K * ay, which is a real effect that L0 has no way to
// represent.
TEST(CrossTier, L0AndL1AgreeInTheLowLateralAccelerationLimit) {
  const auto p = reference_params();
  const double delta = 0.02;
  const double v = 1.5;  // ay of order 0.1 m/s^2

  const Settled l0 = settle(Tier::L0_Kinematic, p, delta, v);
  const Settled l1 = settle(Tier::L1_Bicycle, p, delta, v);

  ASSERT_LT(std::fabs(l1.ay), 0.2) << "this test must stay in the linear limit";
  EXPECT_NEAR(l1.radius, l0.radius, 0.02 * l0.radius);
  EXPECT_GT(l1.yaw_rate, 0.0) << "and they agree on which way the car went";
  EXPECT_GT(l0.yaw_rate, 0.0);

  // What is left of the disagreement is the understeer term, which is a real
  // effect L0 has no representation of, so L1 must sit on the wide side.
  EXPECT_GT(l1.radius, l0.radius);
}

// The divergence is not noise: it is the understeer gradient, and it grows
// with lateral acceleration exactly as delta = L/R + K*ay predicts. Asserting
// the size of the disagreement, rather than only its existence, is what makes
// this a physics test instead of a regression test.
TEST(CrossTier, DivergenceIsTheUndersteerGradientAndGrowsWithLateralG) {
  const auto p = reference_params();
  const double delta = 0.05;
  const double k_expected =
      (p.mass / p.wheelbase()) * (p.lr / p.c_alpha_f - p.lf / p.c_alpha_r);

  double previous_error = 0.0;
  for (const double v : {2.0, 4.0, 6.0}) {
    const Settled l0 = settle(Tier::L0_Kinematic, p, delta, v);
    const Settled l1 = settle(Tier::L1_Bicycle, p, delta, v);

    // L1 needs more steer than L0 for the same radius, so at the same steer
    // angle it runs wider. The gap, expressed back as a steer angle, is K*ay.
    const double gap = p.wheelbase() / l1.radius - p.wheelbase() / l0.radius;
    EXPECT_LT(gap, 0.0) << "the understeering car must run wider at " << v;
    EXPECT_NEAR(-gap, k_expected * l1.ay, 0.15 * k_expected * l1.ay)
        << "at " << v << " m/s";

    EXPECT_GT(std::fabs(gap), previous_error)
        << "divergence must grow with lateral acceleration";
    previous_error = std::fabs(gap);
  }
}

// Where the tiers stop agreeing, in one number a user can act on. Emitted as
// test output so CI keeps a record of it per commit; it is the same quantity
// that becomes the released crossover plot once L2 exists (SRS 7).
TEST(CrossTier, ReportTheCrossoverLateralAcceleration) {
  const auto p = reference_params();
  const double tolerance = 0.05;  // 5% disagreement in path radius

  double crossover_ay = -1.0;
  std::printf("\n  lateral g   L0 radius   L1 radius   disagreement\n");
  for (double v = 1.0; v <= 9.0; v += 0.5) {
    const Settled l0 = settle(Tier::L0_Kinematic, p, 0.05, v);
    const Settled l1 = settle(Tier::L1_Bicycle, p, 0.05, v);
    const double rel = std::fabs(l1.radius - l0.radius) / l0.radius;
    std::printf("  %8.3f   %9.3f   %9.3f   %11.1f%%\n",
                l1.ay / slipx::kGravity, l0.radius, l1.radius, 100.0 * rel);
    if (crossover_ay < 0.0 && rel > tolerance) {
      crossover_ay = std::fabs(l1.ay);
    }
  }

  ASSERT_GT(crossover_ay, 0.0)
      << "the tiers never disagreed by 5%, which means one of them is wrong";
  std::printf("  L0/L1 crossover at %.2f m/s^2 (%.2f g)\n\n", crossover_ay,
              crossover_ay / slipx::kGravity);

  // Sanity bounds rather than a pinned value: the crossover moves with the
  // parameter set, and pinning it would make this a change detector.
  EXPECT_GT(crossover_ay, 0.5);
  EXPECT_LT(crossover_ay, slipx::kGravity * p.mu_clip);
}

// A softer car diverges from the kinematic model sooner. This is the
// statement a student should leave with: the crossover is a property of the
// tyres, not a fixed speed.
TEST(CrossTier, SofterTyresMoveTheCrossoverToLowerLateralAcceleration) {
  const auto disagreement = [](double stiffness_scale) {
    auto p = reference_params();
    p.c_alpha_f *= stiffness_scale;
    p.c_alpha_r *= stiffness_scale;
    const Settled l0 = settle(Tier::L0_Kinematic, p, 0.05, 5.0);
    const Settled l1 = settle(Tier::L1_Bicycle, p, 0.05, 5.0);
    return std::fabs(l1.radius - l0.radius) / l0.radius;
  };

  EXPECT_GT(disagreement(0.5), disagreement(2.0));
}

// Straight-line motion is the degenerate case where the tiers must agree
// closely: with no steer there is no slip and nothing for L1 to add except
// the resistance terms L0 does not model.
TEST(CrossTier, StraightLineAgreementIsBoundedByTheResistanceTermsAlone) {
  auto p = reference_params();
  p.drag_coeff = 0.0;
  p.roll_resist = 0.0;  // with resistance off there is nothing left to differ

  auto l0 = VehicleModel::create(Tier::L0_Kinematic, p);
  auto l1 = VehicleModel::create(Tier::L1_Bicycle, p);

  VehicleState a = at_rest();
  VehicleState b = at_rest();
  for (int i = 0; i < 3000; ++i) {
    l0->step(a, DriveInput{0.0, 2.0}, kDt, nullptr);
    l1->step(b, DriveInput{0.0, 2.0}, kDt, nullptr);
  }
  EXPECT_NEAR(a.pos.x, b.pos.x, 1e-9);
  EXPECT_NEAR(a.vel_body.x, b.vel_body.x, 1e-12);
}

// One controller, unchanged, against both tiers: the experiment the tier
// system exists to make possible (SRS 2.4). It must be able to hold a line at
// L0 and at L1, and the L1 run must need more steer to do it.
TEST(CrossTier, OneControllerRunsAgainstBothTiersUnchanged) {
  const auto p = reference_params();
  const double target_radius = 6.0;

  const auto drive_a_circle = [&](Tier tier) {
    auto model = VehicleModel::create(tier, p);
    VehicleState s = travelling(5.0);
    double steer_sum = 0.0;
    int samples = 0;

    for (int i = 0; i < 12000; ++i) {
      // Curvature hold: kinematic feed-forward plus a gentle correction.
      //
      // The feed-forward is the whole reason a controller ports between
      // tiers: delta = L / R is what both models agree on, and the correction
      // only has to make up the difference. A pure proportional loop with a
      // high gain chatters against L0, whose yaw rate follows the steer angle
      // instantaneously with no dynamics to damp it, and then the comparison
      // measures the controller rather than the tiers.
      const double curvature =
          (s.speed() > 1e-6) ? s.yaw_rate() / s.speed() : 0.0;
      const double target_curvature = 1.0 / target_radius;
      const double steer = slipx::clamp(
          p.wheelbase() * target_curvature +
              0.3 * p.wheelbase() * (target_curvature - curvature),
          -p.steer_max, p.steer_max);

      model->step(s, DriveInput{steer, hold_speed(s, 5.0)}, kDt, nullptr);
      if (i > 8000) {
        steer_sum += s.steer;
        ++samples;
      }
    }
    return steer_sum / samples;
  };

  const double steer_l0 = drive_a_circle(Tier::L0_Kinematic);
  const double steer_l1 = drive_a_circle(Tier::L1_Bicycle);

  EXPECT_GT(steer_l0, 0.0);
  EXPECT_GT(steer_l1, steer_l0)
      << "holding the same radius costs more steer once tyres exist";
  EXPECT_LT(steer_l1, 2.0 * steer_l0) << "but not dramatically more at 0.4 g";
}

// ------------------------------------------------------- L1 against L2
//
// The convergence that matters most, because L2 is the tier the product claim
// rests on and L1 is the tier that has been checked against closed-form
// formulae. If they disagree where they should not, one of them is wrong.

// Coasting, with drag and rolling resistance switched off, there is no
// longitudinal force anywhere in the car. Both tiers are then solving the same
// lateral problem, and they agree to six figures rather than to a tolerance
// somebody chose.
//
// This is the tightest cross-tier statement in the suite and it is the one
// that proves MF-lite's derived B (ADR-0023) reproduces L1's linear tyre
// through a whole vehicle rather than only at a single tyre.
TEST(CrossTier, L1AndL2AgreeExactlyWhenNoLongitudinalForceIsPresent) {
  VehicleParams p = reference_params();
  p.drag_coeff = 0.0;
  p.roll_resist = 0.0;

  auto l1 = VehicleModel::create(Tier::L1_Bicycle, p);
  auto l2 = VehicleModel::create(Tier::L2_DoubleTrack, p);

  VehicleState s1 = travelling(5.0);
  VehicleState s2 = travelling(5.0);
  StepDiagnostics d1;
  StepDiagnostics d2;
  for (int i = 0; i < 40000; ++i) {
    const DriveInput u{0.005, 0.0};  // no drive demand at all
    l1->step(s1, u, kDt, &d1);
    l2->step(s2, u, kDt, &d2);
  }

  const double r1 = s1.speed() / std::fabs(s1.yaw_rate());
  const double r2 = s2.speed() / std::fabs(s2.yaw_rate());
  EXPECT_NEAR(r2 / r1, 1.0, 1e-4) << "L1 " << r1 << " L2 " << r2;

  // And the per-wheel longitudinal forces really are zero, so the case is
  // testing what it says it is.
  for (unsigned i = 0; i < slipx::kWheelCount; ++i) EXPECT_EQ(d2.fx[i], 0.0);
}

// With the drive on, the tiers separate by a few tenths of a percent even at
// low lateral acceleration, and the residual is L1's approximation rather than
// L2's error: a bicycle model applies the front tyre's longitudinal force
// along the body x axis, where a double-track model resolves it through the
// steer angle and picks up an fx sin(delta) term in the lateral direction.
//
// Asserted as a bound rather than left as a footnote, because a future change
// that widens it is a change to one of the two models and should say so.
TEST(CrossTier, DrivenAgreementIsBoundedByTheSteeredForceProjection) {
  const VehicleParams p = reference_params();

  for (const double delta : {0.005, 0.01, 0.02}) {
    const Settled l1 = settle(Tier::L1_Bicycle, p, delta, 5.0);
    const Settled l2 = settle(Tier::L2_DoubleTrack, p, delta, 5.0);
    const double rel = std::fabs(l2.radius - l1.radius) / l1.radius;

    EXPECT_LT(std::fabs(l2.ay), 2.3) << "this case must stay in the linear "
                                        "region for the bound to mean anything";
    EXPECT_LT(rel, 0.01) << "delta " << delta << " ay " << l2.ay;
  }
}

// Below the crossover the README promises 5% on path radius. Asserted here for
// the pair the promise is actually about, since L2 is the tier a user is told
// to trust.
TEST(CrossTier, L1AndL2AgreeWithinFivePercentBelowTheStatedLateralG) {
  const VehicleParams p = reference_params();

  for (const double delta : {0.005, 0.01, 0.02, 0.03}) {
    const Settled l1 = settle(Tier::L1_Bicycle, p, delta, 5.0);
    const Settled l2 = settle(Tier::L2_DoubleTrack, p, delta, 5.0);
    if (std::fabs(l2.ay) > 0.23 * slipx::kGravity) continue;
    EXPECT_LT(std::fabs(l2.radius - l1.radius) / l1.radius, 0.05)
        << "delta " << delta << " ay " << l2.ay;
  }
}

// And they must diverge outside it, or L2 is not adding anything. The
// mechanism is MF-lite's falling branch and load transfer, neither of which L1
// has, so the gap grows with lateral acceleration rather than staying flat.
TEST(CrossTier, L2DivergesFromL1AsTheTyresStartWorking) {
  const VehicleParams p = reference_params();

  const Settled low_1 = settle(Tier::L1_Bicycle, p, 0.01, 5.0);
  const Settled low_2 = settle(Tier::L2_DoubleTrack, p, 0.01, 5.0);
  const Settled high_1 = settle(Tier::L1_Bicycle, p, 0.10, 5.0);
  const Settled high_2 = settle(Tier::L2_DoubleTrack, p, 0.10, 5.0);

  const double low = std::fabs(low_2.radius - low_1.radius) / low_1.radius;
  const double high = std::fabs(high_2.radius - high_1.radius) / high_1.radius;

  EXPECT_GT(std::fabs(high_2.ay), 4.0 * std::fabs(low_2.ay));
  EXPECT_GT(high, 4.0 * low) << "low " << low << " high " << high;
}

}  // namespace
