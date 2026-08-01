// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// SRS 7, layer two: invariants and properties, sampled over parameter space
// rather than checked at one operating point.
//
// The determinism tests here are the core-level half of NFR-02. slipx_sim
// tests the other half (whole-run trajectory hashes); this file establishes
// that the model underneath is deterministic and stateless in the first place,
// because a hash over a nondeterministic model is just a hash of one run.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <type_traits>
#include <vector>

#include "slipx/vehicle_model.hpp"
#include "test_support.hpp"

namespace {

using namespace slipx_test;
using slipx::DriveInput;
using slipx::Integrator;
using slipx::StepDiagnostics;
using slipx::Tier;
using slipx::VehicleModel;
using slipx::VehicleParams;
using slipx::VehicleState;

constexpr double kDt = kDefaultDt;

// A deterministic sweep over valid parameter space. Not std::random_device and
// not std::shuffle: a property test that cannot be replayed from its seed is
// a test that reports failures nobody can reproduce (CORE-04).
std::vector<VehicleParams> parameter_sweep() {
  std::vector<VehicleParams> out;
  for (const double mass : {2.8, 3.5, 5.2}) {
    for (const double lf : {0.13, 0.16, 0.19}) {
      for (const double cf : {90.0, 120.0, 170.0}) {
        for (const double cr : {90.0, 130.0, 170.0}) {
          VehicleParams p = reference_params();
          p.mass = mass;
          p.lf = lf;
          p.lr = 0.32 - lf;
          p.izz = 0.05 * mass / 3.5;
          p.c_alpha_f = cf;
          p.c_alpha_r = cr;
          out.push_back(p);
        }
      }
    }
  }
  return out;
}

// A steer input with a bit of everything: a step, a hold, a reversal and a
// return to centre. Deterministic and closed form, so both halves of a
// symmetry comparison see exactly the same numbers.
DriveInput manoeuvre(const VehicleState& s, double t, double sign) {
  double steer = 0.0;
  if (t > 0.5 && t <= 1.5) steer = 0.12;
  else if (t > 1.5 && t <= 2.5) steer = -0.08;
  else if (t > 2.5 && t <= 3.0) steer = 0.04;
  return DriveInput{sign * steer, hold_speed(s, 5.0)};
}

class Invariants : public ::testing::TestWithParam<Tier> {};

// A left turn must mirror a right turn exactly. This is asserted bit for bit,
// not within a tolerance: every operation in the lateral path is odd in the
// mirrored quantities, and libm's sin, cos, atan2 and tanh are all exactly
// symmetric about zero. A tolerance here would hide the asymmetry that a
// misplaced sign or a one-sided clamp introduces, which is precisely the bug
// this test exists to catch.
TEST_P(Invariants, LeftTurnMirrorsRightTurnBitForBit) {
  auto model = VehicleModel::create(GetParam(), reference_params());

  VehicleState left = travelling(5.0);
  VehicleState right = travelling(5.0);
  const int n = static_cast<int>(std::lround(3.5 / kDt));
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) * kDt;
    model->step(left, manoeuvre(left, t, 1.0), kDt, nullptr);
    model->step(right, manoeuvre(right, t, -1.0), kDt, nullptr);
  }

  EXPECT_EQ(left.pos.x, right.pos.x);
  EXPECT_EQ(left.pos.y, -right.pos.y);
  EXPECT_EQ(left.yaw, -right.yaw);
  EXPECT_EQ(left.vel_body.x, right.vel_body.x);
  EXPECT_EQ(left.vel_body.y, -right.vel_body.y);
  EXPECT_EQ(left.yaw_rate(), -right.yaw_rate());
  EXPECT_EQ(left.steer, -right.steer);
}

// CORE-03: step is const and reads no hidden state, so the same model object
// driven twice from the same state must produce the same trajectory. If a
// model ever cached anything between steps, this is what would catch it.
TEST_P(Invariants, TheSameModelObjectReplaysBitIdentically) {
  auto model = VehicleModel::create(GetParam(), reference_params());

  const auto run = [&] {
    VehicleState s = travelling(5.0);
    const int n = static_cast<int>(std::lround(3.5 / kDt));
    for (int i = 0; i < n; ++i) {
      model->step(s, manoeuvre(s, static_cast<double>(i) * kDt, 1.0), kDt,
                  nullptr);
    }
    return s;
  };

  const VehicleState a = run();
  const VehicleState b = run();  // same object, second time
  EXPECT_EQ(std::memcmp(&a, &b, sizeof(VehicleState)), 0);

  // And a freshly constructed model must agree with the used one.
  auto fresh = VehicleModel::create(GetParam(), reference_params());
  VehicleState c = travelling(5.0);
  const int n = static_cast<int>(std::lround(3.5 / kDt));
  for (int i = 0; i < n; ++i) {
    fresh->step(c, manoeuvre(c, static_cast<double>(i) * kDt, 1.0), kDt,
                nullptr);
  }
  EXPECT_EQ(std::memcmp(&a, &c, sizeof(VehicleState)), 0);
}

// Snapshot and restore is a memcpy (CORE-03, SIM-08). Asserted as a property
// of the type at compile time and as a property of the model at run time:
// copy the state out, run on, put it back, and the continuation must be
// identical to the one that never diverged.
TEST_P(Invariants, SnapshotAndRestoreIsAMemcpy) {
  static_assert(std::is_trivially_copyable<VehicleState>::value,
                "VehicleState must stay a memcpy-able POD");
  static_assert(std::is_trivially_copyable<StepDiagnostics>::value, "");

  auto model = VehicleModel::create(GetParam(), reference_params());
  VehicleState s = travelling(5.0);
  for (int i = 0; i < 500; ++i) {
    model->step(s, manoeuvre(s, static_cast<double>(i) * kDt, 1.0), kDt,
                nullptr);
  }

  VehicleState snapshot;
  std::memcpy(&snapshot, &s, sizeof(VehicleState));

  // Diverge.
  for (int i = 0; i < 500; ++i) {
    model->step(s, DriveInput{0.3, -2.0}, kDt, nullptr);
  }

  // Restore and take the branch that was never taken.
  std::memcpy(&s, &snapshot, sizeof(VehicleState));
  VehicleState control;
  std::memcpy(&control, &snapshot, sizeof(VehicleState));
  for (int i = 500; i < 1000; ++i) {
    const double t = static_cast<double>(i) * kDt;
    model->step(s, manoeuvre(s, t, 1.0), kDt, nullptr);
    model->step(control, manoeuvre(control, t, 1.0), kDt, nullptr);
  }
  EXPECT_EQ(std::memcmp(&s, &control, sizeof(VehicleState)), 0);
}

// Two models sharing nothing must not interfere, which is the property that
// makes N instances trivially parallel (CORE-03). Interleaving their steps
// must give the same answer as running them one after the other.
TEST_P(Invariants, InterleavedInstancesDoNotInterfere) {
  auto a = VehicleModel::create(GetParam(), reference_params());
  auto b_params = reference_params();
  b_params.mass = 5.0;
  auto b = VehicleModel::create(GetParam(), b_params);

  VehicleState sa_i = travelling(5.0);
  VehicleState sb_i = travelling(3.0);
  const int n = 2000;
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) * kDt;
    a->step(sa_i, manoeuvre(sa_i, t, 1.0), kDt, nullptr);
    b->step(sb_i, manoeuvre(sb_i, t, -1.0), kDt, nullptr);
  }

  VehicleState sa_s = travelling(5.0);
  for (int i = 0; i < n; ++i) {
    a->step(sa_s, manoeuvre(sa_s, static_cast<double>(i) * kDt, 1.0), kDt,
            nullptr);
  }
  VehicleState sb_s = travelling(3.0);
  for (int i = 0; i < n; ++i) {
    b->step(sb_s, manoeuvre(sb_s, static_cast<double>(i) * kDt, -1.0), kDt,
            nullptr);
  }

  EXPECT_EQ(std::memcmp(&sa_i, &sa_s, sizeof(VehicleState)), 0);
  EXPECT_EQ(std::memcmp(&sb_i, &sb_s, sizeof(VehicleState)), 0);
}

// Asking for diagnostics must not change the trajectory. It is an easy
// mistake to make and a miserable one to debug, because it makes the
// instrumented run disagree with the fast one.
TEST_P(Invariants, DiagnosticsDoNotPerturbTheTrajectory) {
  auto model = VehicleModel::create(GetParam(), reference_params());

  VehicleState quiet = travelling(5.0);
  VehicleState loud = travelling(5.0);
  StepDiagnostics d;
  for (int i = 0; i < 3500; ++i) {
    const double t = static_cast<double>(i) * kDt;
    model->step(quiet, manoeuvre(quiet, t, 1.0), kDt, nullptr);
    model->step(loud, manoeuvre(loud, t, 1.0), kDt, &d);
  }
  EXPECT_EQ(std::memcmp(&quiet, &loud, sizeof(VehicleState)), 0);
}

// Yaw must stay wrapped however many laps are driven, or every consumer that
// compares headings has to know to wrap first, and one of them will forget.
TEST_P(Invariants, YawStaysWrapped) {
  auto model = VehicleModel::create(GetParam(), reference_params());
  VehicleState s = travelling(5.0);
  for (int i = 0; i < 30000; ++i) {  // many revolutions
    model->step(s, DriveInput{0.25, hold_speed(s, 5.0)}, kDt, nullptr);
    ASSERT_LE(s.yaw, slipx::kPi);
    ASSERT_GT(s.yaw, -slipx::kPi);
  }
}

INSTANTIATE_TEST_SUITE_P(AllTiers, Invariants,
                         ::testing::Values(Tier::L0_Kinematic,
                                           Tier::L1_Bicycle));

// -------------------------------------------------------- L0-specific facts

// The teaching artefact, asserted (SRS 2.4). At L0 the trajectory is a
// function of geometry and command alone: mass, yaw inertia, CoG height,
// cornering stiffness and friction have no representation, so changing them
// must change nothing. A student who sees this and expects otherwise has
// learned what the tier is. A version of L0 that responded to mass would be
// teaching them something false, so the test demands bit-identity.
TEST(InvariantsL0, MassCoGAndTyresHaveNoEffectOnTheTrajectory) {
  const auto run = [](const VehicleParams& p) {
    auto model = VehicleModel::create(Tier::L0_Kinematic, p);
    VehicleState s = travelling(5.0);
    for (int i = 0; i < 3500; ++i) {
      model->step(s, manoeuvre(s, static_cast<double>(i) * kDt, 1.0), kDt,
                  nullptr);
    }
    return s;
  };

  const VehicleState base = run(reference_params());

  auto heavy = reference_params();
  heavy.mass = 12.0;
  heavy.izz = 0.4;
  heavy.h_cog = 0.15;
  heavy.c_alpha_f = 400.0;
  heavy.c_alpha_r = 40.0;
  heavy.mu_clip = 0.2;
  heavy.drag_coeff = 5.0;
  heavy.roll_resist = 0.5;
  const VehicleState changed = run(heavy);

  EXPECT_EQ(std::memcmp(&base, &changed, sizeof(VehicleState)), 0)
      << "L0 must be insensitive to everything it does not represent";
}

// The other half of the same statement: L0 does respond to what it does
// represent, so the test above is establishing insensitivity rather than
// establishing that the model ignores its inputs.
TEST(InvariantsL0, GeometryDoesChangeTheTrajectory) {
  const auto radius = [](double lf, double lr) {
    auto p = reference_params();
    p.lf = lf;
    p.lr = lr;
    auto model = VehicleModel::create(Tier::L0_Kinematic, p);
    VehicleState s = travelling(5.0);
    run_for(*model, s, 0.5, kDt, [](const VehicleState& st, double) {
      return DriveInput{0.15, hold_speed(st, 5.0)};
    });
    return s.speed() / s.yaw_rate();
  };

  EXPECT_GT(radius(0.25, 0.25), radius(0.16, 0.16))
      << "a longer wheelbase turns more slowly for the same steer angle";
}

// ------------------------------------------------------- L1-specific facts

// With the drivetrain idle and resistance switched off, the only remaining
// force is the tyres', and a slipping tyre can only dissipate. Kinetic energy
// must therefore never increase. This is the invariant that catches a sign
// error in the yaw moment, which otherwise pumps energy in and shows up as a
// car that mysteriously speeds up in a corner.
TEST(InvariantsL1, TyreForcesOnlyDissipateEnergy) {
  auto p = reference_params();
  p.drag_coeff = 0.0;
  p.roll_resist = 0.0;
  auto model = VehicleModel::create(Tier::L1_Bicycle, p);

  VehicleState s = travelling(6.0);
  s.rates.z = 1.5;      // yawing
  s.vel_body.y = 0.8;   // and sliding, so the tyres are working

  const auto energy = [&](const VehicleState& st) {
    return 0.5 * p.mass * st.vel_body.xy().squared_norm() +
           0.5 * p.izz * st.rates.z * st.rates.z;
  };

  const double initial = energy(s);
  double previous = initial;
  for (int i = 0; i < 5000; ++i) {
    model->step(s, DriveInput{0.0, 0.0}, kDt, nullptr);
    const double now = energy(s);
    ASSERT_LE(now, previous + 1e-12) << "energy increased at step " << i;
    previous = now;
  }
  EXPECT_LT(previous, 0.995 * initial)
      << "and it must actually have dissipated some, not merely not gained";
}

// Monotonicity over the parameter sweep: a stiffer front axle makes a car
// turn more sharply for the same steer angle, at every point in the sampled
// space. Property-based rather than single-point, so a sign that is right for
// the reference car and wrong for a rear-heavy one does not survive.
TEST(InvariantsL1, StifferFrontAxleAlwaysTightensTheLine) {
  for (const VehicleParams& p : parameter_sweep()) {
    VehicleParams stiffer = p;
    stiffer.c_alpha_f = p.c_alpha_f * 1.5;

    const auto yaw_rate = [](const VehicleParams& q) {
      auto model = VehicleModel::create(Tier::L1_Bicycle, q);
      VehicleState s = travelling(4.0);
      run_for(*model, s, 4.0, kDt, [](const VehicleState& st, double) {
        return DriveInput{0.04, hold_speed(st, 4.0)};
      });
      return s.yaw_rate();
    };

    EXPECT_GT(yaw_rate(stiffer), yaw_rate(p))
        << "mass " << p.mass << " lf " << p.lf << " cf " << p.c_alpha_f
        << " cr " << p.c_alpha_r;
  }
}

// Every parameter set in the sweep must produce a finite trajectory. NaN in a
// physics core is not a numerical curiosity: it propagates into a controller,
// which then does something arbitrary.
TEST(InvariantsL1, NoParameterSetInTheSweepProducesNaN) {
  for (const VehicleParams& p : parameter_sweep()) {
    ASSERT_EQ(slipx::validate(p), nullptr);
    auto model = VehicleModel::create(Tier::L1_Bicycle, p);

    VehicleState s = at_rest();  // including from standstill
    StepDiagnostics d;
    for (int i = 0; i < 6000; ++i) {
      model->step(s, manoeuvre(s, static_cast<double>(i) * kDt, 1.0), kDt, &d);
    }
    ASSERT_TRUE(std::isfinite(s.pos.x));
    ASSERT_TRUE(std::isfinite(s.pos.y));
    ASSERT_TRUE(std::isfinite(s.yaw));
    ASSERT_TRUE(std::isfinite(s.vel_body.x));
    ASSERT_TRUE(std::isfinite(s.vel_body.y));
    ASSERT_TRUE(std::isfinite(s.rates.z));
    ASSERT_TRUE(std::isfinite(d.ay));
  }
}

// Both integrators must agree on where the car ends up, or the choice between
// them is a choice between two different cars rather than between two costs.
// They will not agree to machine precision, and are not asked to.
TEST(InvariantsL1, BothIntegratorsAgreeToTheirTruncationError) {
  auto rk4 = VehicleModel::create(Tier::L1_Bicycle, reference_params(),
                                  Integrator::kRK4);
  auto sie = VehicleModel::create(Tier::L1_Bicycle, reference_params(),
                                  Integrator::kSemiImplicitEuler);

  VehicleState a = travelling(5.0);
  VehicleState b = travelling(5.0);
  for (int i = 0; i < 3500; ++i) {
    const double t = static_cast<double>(i) * kDt;
    rk4->step(a, manoeuvre(a, t, 1.0), kDt, nullptr);
    sie->step(b, manoeuvre(b, t, 1.0), kDt, nullptr);
  }

  const double travelled = a.pos.xy().norm();
  EXPECT_NEAR(a.pos.x, b.pos.x, 0.01 * travelled);
  EXPECT_NEAR(a.pos.y, b.pos.y, 0.01 * travelled);
  EXPECT_NEAR(a.yaw, b.yaw, 0.02);
}

}  // namespace
