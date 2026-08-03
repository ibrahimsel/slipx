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

#include "slipx/load_transfer.hpp"
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

// -------------------------------------------------- load transfer (CORE-05)
//
// The closed-form checks are in test_analytical.cpp. What is here is the half
// that a single operating point cannot establish: exact symmetry, and
// monotonicity in the parameters over the sampled space rather than at one
// convenient car.

using slipx::kFrontLeft;
using slipx::kFrontRight;
using slipx::kRearLeft;
using slipx::kRearRight;
using slipx::WheelLoads;

// Cars, accelerations and CoG heights spanning the plausible 1/10 range plus
// enough beyond it to reach wheel lift. Deterministic and closed form, as
// with the parameter sweep above (CORE-04).
std::vector<VehicleParams> chassis_sweep() {
  std::vector<VehicleParams> out;
  for (const double mass : {2.8, 4.4}) {
    for (const double lf : {0.11, 0.16, 0.21}) {
      for (const double h : {0.03, 0.06, 0.11}) {
        for (const double track : {0.20, 0.24}) {
          VehicleParams p = reference_params();
          p.mass = mass;
          p.lf = lf;
          p.lr = 0.32 - lf;
          p.h_cog = h;
          p.track_front = track;
          p.track_rear = track + 0.02;
          out.push_back(p);
        }
      }
    }
  }
  return out;
}

// A left turn must mirror a right turn exactly, in the loads as much as in
// the trajectory. Asserted bit for bit rather than within a tolerance: every
// operation in the lateral path is odd in ay, so any inexactness here is an
// asymmetry that has been introduced rather than a rounding difference that
// had to be tolerated. This is the invariant that catches a one-sided clamp,
// which is exactly the shape of the wheel-lift code.
TEST(InvariantsLoadTransfer, MirroringLateralAccelerationSwapsLeftAndRight) {
  for (const VehicleParams& p : chassis_sweep()) {
    for (const double ax : {-4.0, 0.0, 4.0}) {
      for (const double ay : {0.5, 4.0, 20.0}) {  // 20 is past wheel lift
        const WheelLoads left = slipx::quasi_static_loads(p, ax, ay);
        const WheelLoads right = slipx::quasi_static_loads(p, ax, -ay);

        EXPECT_EQ(left.fz[kFrontLeft], right.fz[kFrontRight]);
        EXPECT_EQ(left.fz[kFrontRight], right.fz[kFrontLeft]);
        EXPECT_EQ(left.fz[kRearLeft], right.fz[kRearRight]);
        EXPECT_EQ(left.fz[kRearRight], right.fz[kRearLeft]);
        EXPECT_EQ(left.fz_front, right.fz_front);
        EXPECT_EQ(left.fz_rear, right.fz_rear);
        EXPECT_EQ(left.transfer_long, right.transfer_long);
        EXPECT_EQ(left.transfer_lat, -right.transfer_lat);
        EXPECT_EQ(left.wheel_lifted, right.wheel_lifted);
      }
    }
  }
}

// Raising the CoG must lower the rollover threshold, at every car in the
// sweep. This is the monotonicity SRS 7 names, and it is the statement a
// student is meant to leave with: the battery position is a handling
// parameter, and below L2 the model had no way to say so.
TEST(InvariantsLoadTransfer, RaisingTheCoGLowersTheRolloverThreshold) {
  for (const VehicleParams& p : chassis_sweep()) {
    VehicleParams higher = p;
    higher.h_cog = p.h_cog * 1.5;

    EXPECT_LT(slipx::static_rollover_threshold(higher),
              slipx::static_rollover_threshold(p))
        << "h_cog " << p.h_cog << " track " << p.track_front;

    // And the same fact stated through the loads rather than the formula: at
    // a lateral acceleration the lower car survives, the higher one lifts a
    // wheel.
    const double ay = 0.5 * (slipx::static_rollover_threshold(higher) +
                             slipx::static_rollover_threshold(p));
    EXPECT_FALSE(slipx::quasi_static_loads(p, 0.0, ay).wheel_lifted);
    EXPECT_TRUE(slipx::quasi_static_loads(higher, 0.0, ay).wheel_lifted);
  }
}

// The other two levers, in the directions a chassis builder would predict:
// a wider track raises the threshold, and neither mass nor ballast position
// moves it at all.
TEST(InvariantsLoadTransfer, WiderTrackRaisesTheThresholdAndMassDoesNothing) {
  for (const VehicleParams& p : chassis_sweep()) {
    VehicleParams wider = p;
    wider.track_front = p.track_front * 1.25;
    wider.track_rear = p.track_rear * 1.25;
    EXPECT_GT(slipx::static_rollover_threshold(wider),
              slipx::static_rollover_threshold(p));

    VehicleParams heavy = p;
    heavy.mass = p.mass * 2.0;
    EXPECT_EQ(slipx::static_rollover_threshold(heavy),
              slipx::static_rollover_threshold(p));
  }
}

// Monotone in the accelerations too: more lateral acceleration always moves
// more load, up to the point where there is no more to move.
TEST(InvariantsLoadTransfer, MoreAccelerationAlwaysMovesMoreLoad) {
  for (const VehicleParams& p : chassis_sweep()) {
    double previous_lat = -1.0;
    for (const double ay : {0.0, 1.0, 2.0, 3.0, 4.0}) {
      const double moved = slipx::quasi_static_loads(p, 0.0, ay).transfer_lat;
      ASSERT_GT(moved, previous_lat) << "at ay " << ay;
      previous_lat = moved;
    }

    double previous_long = -1e9;
    for (const double ax : {-4.0, -2.0, 0.0, 2.0, 4.0}) {
      const double moved = slipx::quasi_static_loads(p, ax, 0.0).transfer_long;
      ASSERT_GT(moved, previous_long) << "at ax " << ax;
      previous_long = moved;
    }
  }
}

// No parameter set and no acceleration in the sweep may produce a NaN, a
// negative load, or a set of loads that does not add up to the weight of the
// car. The last one is the invariant everything downstream rests on: a tyre
// model is only as good as the vertical load it is handed.
TEST(InvariantsLoadTransfer, LoadsStayFiniteNonNegativeAndSumToTheWeight) {
  for (const VehicleParams& p : chassis_sweep()) {
    const double weight = p.mass * slipx::kGravity;
    for (const double ax : {-40.0, -9.0, 0.0, 9.0, 40.0}) {
      for (const double ay : {-40.0, -9.0, 0.0, 9.0, 40.0}) {
        const WheelLoads w = slipx::quasi_static_loads(p, ax, ay);
        double sum = 0.0;
        for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
          ASSERT_TRUE(std::isfinite(w.fz[i]))
              << "wheel " << i << " at ax " << ax << " ay " << ay;
          ASSERT_GE(w.fz[i], 0.0)
              << "wheel " << i << " at ax " << ax << " ay " << ay;
          sum += w.fz[i];
        }
        ASSERT_NEAR(sum, weight, 1e-9) << "at ax " << ax << " ay " << ay;
        ASSERT_TRUE(std::isfinite(w.transfer_long));
        ASSERT_TRUE(std::isfinite(w.transfer_lat));
      }
    }
  }
}

// --------------------------------------------------------- MF-lite (CORE-06)
//
// The closed-form properties are in test_analytical.cpp. What is here is what
// no single operating point can establish: exact oddness, and the bounds and
// monotonicity holding over a swept space of tyres rather than at one
// convenient set of coefficients.

using slipx::CombinedForce;
using slipx::MfLite;
using slipx::TyreCoefficients;

// Tyres spanning the ranges tyre.schema.json admits, at the extremes as well as
// the middle: a hard low-grip compound through a soft sticky one, no load
// sensitivity through the maximum, and shape factors at both ends of the band
// that still describes a tyre. Deterministic and closed form (CORE-04).
std::vector<TyreCoefficients> tyre_sweep() {
  std::vector<TyreCoefficients> out;
  for (const double mu_y : {0.55, 1.15, 2.20}) {
    for (const double k_mu : {0.0, 0.22, 0.60}) {
      for (const double shape_c : {1.05, 1.43, 1.90}) {
        for (const double curvature_e : {-2.0, 0.0, 0.87}) {
          TyreCoefficients t;
          t.mu_y0 = mu_y;
          t.mu_x0 = mu_y * 1.15;
          t.k_mu = k_mu;
          t.shape_c = shape_c;
          t.curvature_e = curvature_e;
          out.push_back(t);
        }
      }
    }
  }
  return out;
}

constexpr double kSweepCAlpha = 61.0;  // per tyre                    [N/rad]
constexpr double kSweepFzNom = 8.4;    //                                 [N]

// A tyre must behave identically in both directions. Asserted bit for bit
// rather than within a tolerance: mf_shape is composed of odd functions, so any
// asymmetry here has been introduced rather than tolerated. This is the
// invariant that catches a one-sided clamp or a fabs where a sign belonged,
// which is the shape of the zero-load guard.
TEST(InvariantsMfLite, LateralForceIsExactlyOddInSlipAngle) {
  for (const TyreCoefficients& c : tyre_sweep()) {
    const MfLite t = slipx::make_mf_lite(c, kSweepCAlpha, kSweepFzNom);
    for (const double fz : {0.0, 2.0, 8.4, 30.0}) {
      for (const double alpha : {1e-4, 0.03, 0.15, 0.8, 3.0}) {
        ASSERT_EQ(slipx::mf_lite_fy(t, -alpha, fz),
                  -slipx::mf_lite_fy(t, alpha, fz))
            << "mu_y0 " << c.mu_y0 << " C " << c.shape_c << " E "
            << c.curvature_e << " Fz " << fz << " alpha " << alpha;
      }
      ASSERT_EQ(slipx::mf_lite_fy(t, 0.0, fz), 0.0);
    }
  }
}

// No tyre in the sweep, at any slip angle or load, may exceed its friction
// budget or produce a NaN. This is the bound everything downstream rests on:
// the friction ellipse, the saturation flag and the claim that L2 has a limit
// at all are only as good as |Fy| <= mu Fz holding everywhere.
TEST(InvariantsMfLite, ForceNeverExceedsTheFrictionBudget) {
  for (const TyreCoefficients& c : tyre_sweep()) {
    const MfLite t = slipx::make_mf_lite(c, kSweepCAlpha, kSweepFzNom);
    for (const double fz : {0.0, 0.4, 8.4, 30.0}) {
      const double budget = slipx::peak_lateral_force(t, fz);
      ASSERT_TRUE(std::isfinite(budget));
      ASSERT_GE(budget, 0.0);

      for (int i = -400; i <= 400; ++i) {
        const double alpha = 0.01 * static_cast<double>(i);
        const double fy = slipx::mf_lite_fy(t, alpha, fz);
        ASSERT_TRUE(std::isfinite(fy))
            << "mu_y0 " << c.mu_y0 << " Fz " << fz << " alpha " << alpha;
        ASSERT_LE(std::fabs(fy), budget * (1.0 + 1e-12))
            << "mu_y0 " << c.mu_y0 << " Fz " << fz << " alpha " << alpha;
      }
    }
  }
}

// Below the peak, more slip angle means more force, at every tyre in the
// sweep. A curve that was not monotone up to its peak would have two slip
// angles delivering the same force on the rising side, which is not a tyre and
// is what the schema's bound of E <= 1 exists to prevent.
TEST(InvariantsMfLite, ForceRisesMonotonicallyUpToThePeak) {
  for (const TyreCoefficients& c : tyre_sweep()) {
    const MfLite t = slipx::make_mf_lite(c, kSweepCAlpha, kSweepFzNom);
    const double budget = slipx::peak_lateral_force(t, kSweepFzNom);

    double previous = -1.0;
    for (int i = 0; i <= 2000; ++i) {
      const double alpha = 0.001 * static_cast<double>(i);
      const double fy = std::fabs(slipx::mf_lite_fy(t, alpha, kSweepFzNom));
      // Stop at the peak; the falling branch beyond it is checked in
      // test_analytical.cpp and is not monotone by design.
      if (fy < previous) break;
      ASSERT_GE(fy, previous) << "C " << c.shape_c << " E " << c.curvature_e
                              << " alpha " << alpha;
      ASSERT_LE(fy, budget * (1.0 + 1e-12));
      previous = fy;
    }
    // The rise has to get somewhere near the budget rather than stalling.
    ASSERT_GT(previous, 0.5 * budget) << "C " << c.shape_c;
  }
}

// More load is more grip, always, even though the coefficient falls. The two
// pull in opposite directions and the product must still increase, which is
// what k_mu < 1 means physically; a k_mu above 1 would describe a tyre that
// grips less the harder you push it, and the schema bounds it at 1 for that
// reason.
TEST(InvariantsMfLite, MoreLoadIsAlwaysMoreForceButLessCoefficient) {
  for (const TyreCoefficients& c : tyre_sweep()) {
    const MfLite t = slipx::make_mf_lite(c, kSweepCAlpha, kSweepFzNom);

    double previous_force = -1.0;
    double previous_mu = 1e9;
    for (const double fz : {0.5, 2.0, 8.4, 20.0, 40.0}) {
      const double force = slipx::peak_lateral_force(t, fz);
      const double mu = slipx::mu_at_load(c.mu_y0, c.k_mu, fz, kSweepFzNom);
      ASSERT_GT(force, previous_force) << "mu_y0 " << c.mu_y0 << " Fz " << fz;
      if (c.k_mu > 0.0) {
        ASSERT_LT(mu, previous_mu) << "mu_y0 " << c.mu_y0 << " Fz " << fz;
      } else {
        ASSERT_EQ(mu, c.mu_y0);
      }
      previous_force = force;
      previous_mu = mu;
    }
  }
}

// The ellipse never returns more than the budget, never reverses a component,
// and never produces a NaN, over a grid of demands and budgets that includes
// the degenerate ones.
TEST(InvariantsCombinedSlip, TheResultAlwaysLiesInsideTheEllipse) {
  for (const double fx_max : {0.0, 0.7, 9.7}) {
    for (const double fy_max : {0.0, 1.3, 8.3}) {
      for (const double fx : {-40.0, -9.7, -0.2, 0.0, 0.2, 9.7, 40.0}) {
        for (const double fy : {-40.0, -8.3, -0.2, 0.0, 0.2, 8.3, 40.0}) {
          const CombinedForce r =
              slipx::friction_ellipse(fx, fy, fx_max, fy_max);

          ASSERT_TRUE(std::isfinite(r.fx)) << fx << " " << fy;
          ASSERT_TRUE(std::isfinite(r.fy)) << fx << " " << fy;
          ASSERT_LE(std::fabs(r.fx), std::fabs(fx));
          ASSERT_LE(std::fabs(r.fy), std::fabs(fy));
          ASSERT_GE(r.fx * fx, 0.0) << "the ellipse reversed fx";
          ASSERT_GE(r.fy * fy, 0.0) << "the ellipse reversed fy";

          if (fx_max > 0.0 && fy_max > 0.0) {
            ASSERT_LE(std::hypot(r.fx / fx_max, r.fy / fy_max), 1.0 + 1e-12)
                << "fx " << fx << " fy " << fy;
          } else {
            ASSERT_EQ(r.fx, 0.0);
            ASSERT_EQ(r.fy, 0.0);
          }
        }
      }
    }
  }
}

// Mirroring a demand mirrors the result exactly, in both axes independently.
// The ellipse is symmetric about both, and a tyre that delivered slightly more
// braking than acceleration would bias every trajectory in one direction.
TEST(InvariantsCombinedSlip, MirroringADemandMirrorsTheResult) {
  const double fx_max = 9.7;
  const double fy_max = 8.3;
  for (const double fx : {0.2, 6.0, 40.0}) {
    for (const double fy : {0.3, 5.0, 30.0}) {
      const CombinedForce a = slipx::friction_ellipse(fx, fy, fx_max, fy_max);
      const CombinedForce bx = slipx::friction_ellipse(-fx, fy, fx_max,
                                                       fy_max);
      const CombinedForce by = slipx::friction_ellipse(fx, -fy, fx_max,
                                                       fy_max);

      ASSERT_EQ(bx.fx, -a.fx);
      ASSERT_EQ(bx.fy, a.fy);
      ASSERT_EQ(by.fx, a.fx);
      ASSERT_EQ(by.fy, -a.fy);
      ASSERT_EQ(bx.saturated, a.saturated);
      ASSERT_EQ(by.saturated, a.saturated);
    }
  }
}

// ------------------------------------------------- relaxation invariants
//
// Sampled over speed, relaxation length and both slip angles, because the
// properties below have to hold everywhere rather than at the operating point
// the analytical cases pick.

constexpr double kRelaxSigma = 0.08;  // relaxation length                [m]

// The lag approaches its target and never passes it. Overshoot would mean the
// tyre briefly producing more slip than the geometry asked for, which no
// first-order lag does and which is what an integrator run past its stability
// bound looks like.
TEST(InvariantsRelaxation, TheLagApproachesTheTargetAndNeverOvershoots) {
  for (const double sigma : {0.02, 0.08, 0.30}) {
    for (const double vx : {0.5, 4.0, 20.0}) {
      for (const double alpha : {-0.20, -0.01, 0.03, 0.15}) {
        double lagged = 0.0;
        const double dt = 1e-4;
        double previous_gap = std::fabs(alpha - lagged);

        for (int i = 0; i < 4000; ++i) {
          lagged += dt * slipx::relaxation_rate(alpha, lagged, vx, sigma);

          // Never past the target, in either direction.
          if (alpha >= 0.0) {
            EXPECT_GE(lagged, -1e-15);
            EXPECT_LE(lagged, alpha + 1e-15);
          } else {
            EXPECT_LE(lagged, 1e-15);
            EXPECT_GE(lagged, alpha - 1e-15);
          }

          // And the gap closes monotonically.
          const double gap = std::fabs(alpha - lagged);
          EXPECT_LE(gap, previous_gap + 1e-15);
          previous_gap = gap;
        }
      }
    }
  }
}

// The rate is exactly zero at steady state, for every speed and every angle.
// Not nearly zero: a settled tyre that keeps nudging its own slip angle would
// put a drift into a state the trajectory hash covers.
TEST(InvariantsRelaxation, SteadyStateIsExactlyStationary) {
  for (const double sigma : {0.02, 0.08, 0.30}) {
    for (const double vx : {0.0, 1.0, 7.5, 25.0}) {
      for (const double alpha : {-0.3, 0.0, 0.11}) {
        EXPECT_EQ(slipx::relaxation_rate(alpha, alpha, vx, sigma), 0.0);
        EXPECT_EQ(slipx::relaxation_exact(alpha, alpha, vx, sigma, 0.01),
                  alpha);
      }
    }
  }
}

// A left turn mirrors a right turn, bit for bit, through the transient as well
// as at steady state. The same claim the load transfer and combined slip
// suites make, extended to the one part of L2 that has memory.
TEST(InvariantsRelaxation, MirroringTheSlipAngleMirrorsTheLag) {
  for (const double vx : {1.0, 6.0, 17.0}) {
    for (const double alpha : {0.02, 0.09, 0.25}) {
      for (const double lagged : {0.0, 0.01, 0.30}) {
        EXPECT_EQ(slipx::relaxation_rate(-alpha, -lagged, vx, kRelaxSigma),
                  -slipx::relaxation_rate(alpha, lagged, vx, kRelaxSigma));
        EXPECT_EQ(slipx::relaxation_exact(-alpha, -lagged, vx, kRelaxSigma, 0.01),
                  -slipx::relaxation_exact(alpha, lagged, vx, kRelaxSigma, 0.01));
      }
    }
  }
}

// A shorter relaxation length is a quicker tyre, at every speed and every
// elapsed time. Monotonicity in the parameter, which is the property that
// makes sigma identifiable from a step steer rise time at all: if two sigmas
// gave the same rise time the fit would have no unique answer.
TEST(InvariantsRelaxation, AShorterRelaxationLengthRespondsSooner) {
  const double alpha = 0.05;
  for (const double vx : {2.0, 8.0, 19.0}) {
    for (const double t : {0.001, 0.005, 0.020}) {
      double previous = 2.0;  // above any achievable lagged angle
      for (const double sigma : {0.30, 0.15, 0.08, 0.04, 0.01}) {
        const double lagged =
            slipx::relaxation_exact(alpha, 0.0, vx, sigma, t);
        EXPECT_GT(lagged, previous == 2.0 ? -1.0 : previous)
            << "sigma " << sigma << " vx " << vx << " t " << t;
        previous = lagged;
      }
    }
  }
}

// The lagged angle fed through MF-lite is always a force the tyre could
// actually produce at its CURRENT load. This is the property that chose a
// lagged slip angle over a lagged force (ADR-0026), so it is asserted rather
// than argued: the load is collapsed to zero mid-transient, and the force
// follows it down in the same step instead of trailing behind.
TEST(InvariantsRelaxation, ForceStaysInsideTheBudgetWhenTheWheelUnloads) {
  const MfLite tyre =
      slipx::make_mf_lite(TyreCoefficients{}, kSweepCAlpha, kSweepFzNom);
  const double vx = 10.0;
  const double sigma = 0.08;
  const double alpha = 0.06;

  // Build the transient up under load.
  double lagged = 0.0;
  for (int i = 0; i < 200; ++i) {
    lagged += 1e-4 * slipx::relaxation_rate(alpha, lagged, vx, sigma);
  }
  EXPECT_GT(lagged, 0.0);
  EXPECT_GT(std::fabs(slipx::mf_lite_fy(tyre, lagged, kSweepFzNom)), 1.0);

  // Now lift the wheel. The slip angle keeps its history, which is correct:
  // the carcass is still deflected. The force does not, because it is
  // evaluated from the load that exists now.
  EXPECT_EQ(slipx::mf_lite_fy(tyre, lagged, 0.0), 0.0);

  // And at every load on the way down, the lagged-angle force is inside the
  // budget for that load. A lagged FORCE would exceed it here.
  for (const double fz : {kSweepFzNom, 4.0, 1.0, 0.25, 0.0}) {
    const double fy = std::fabs(slipx::mf_lite_fy(tyre, lagged, fz));
    EXPECT_LE(fy, slipx::peak_lateral_force(tyre, fz) + 1e-12)
        << "at load " << fz;
  }
}

// --------------------------------------------------------- L2 invariants
//
// The properties the assembled tier must have whatever it is driven through,
// as opposed to the operating points test_analytical.cpp pins down.

// A right turn is a left turn mirrored, bit for bit, through ten states and
// four contact patches. Not within a tolerance: every operation in the tier is
// either odd or even in the lateral direction, so any asymmetry here has been
// introduced rather than tolerated.
//
// This is the single strongest test of the whole tier. A sign error anywhere
// in the wheel offsets, the load transfer split, the yaw moment or the slip
// angles breaks it, and almost nothing else does.
TEST(InvariantsL2, MirroringTheSteeringMirrorsTheEntireCar) {
  const VehicleParams p = reference_params();
  auto left = VehicleModel::create(Tier::L2_DoubleTrack, p);
  auto right = VehicleModel::create(Tier::L2_DoubleTrack, p);

  VehicleState sl = travelling(6.0);
  VehicleState sr = travelling(6.0);
  StepDiagnostics dl;
  StepDiagnostics dr;

  for (int i = 0; i < 3000; ++i) {
    const double steer = 0.12 * std::sin(0.004 * i);
    const double accel = 2.0;
    left->step(sl, DriveInput{steer, accel}, kDt, &dl);
    right->step(sr, DriveInput{-steer, accel}, kDt, &dr);

    ASSERT_EQ(sl.vel_body.x, sr.vel_body.x) << "step " << i;
    ASSERT_EQ(sl.vel_body.y, -sr.vel_body.y) << "step " << i;
    ASSERT_EQ(sl.yaw_rate(), -sr.yaw_rate()) << "step " << i;
    ASSERT_EQ(sl.pos.x, sr.pos.x) << "step " << i;
    ASSERT_EQ(sl.pos.y, -sr.pos.y) << "step " << i;
    ASSERT_EQ(sl.yaw, -sr.yaw) << "step " << i;

    // Left and right swap across the mirror, front and rear do not.
    ASSERT_EQ(sl.alpha_lag[kFrontLeft], -sr.alpha_lag[kFrontRight])
        << "step " << i;
    ASSERT_EQ(sl.alpha_lag[kRearLeft], -sr.alpha_lag[kRearRight])
        << "step " << i;
    ASSERT_EQ(dl.fz[kFrontLeft], dr.fz[kFrontRight]) << "step " << i;
    ASSERT_EQ(dl.fz[kRearLeft], dr.fz[kRearRight]) << "step " << i;
    ASSERT_EQ(dl.fy[kFrontLeft], -dr.fy[kFrontRight]) << "step " << i;
    ASSERT_EQ(dl.fx[kFrontLeft], dr.fx[kFrontRight]) << "step " << i;
    ASSERT_EQ(dl.ay, -dr.ay) << "step " << i;
    ASSERT_EQ(dl.ax, dr.ax) << "step " << i;
  }
}

// The four wheel loads sum to the vehicle weight at every step of a violent
// manoeuvre, including through wheel lift. Load is moved between corners,
// never created or destroyed, and the assembled tier must not lose that
// property that load_transfer.hpp has on its own.
TEST(InvariantsL2, WeightIsConservedThroughAnythingTheCarIsDriventhrough) {
  VehicleParams p = reference_params();
  p.h_cog = 0.12;
  p.steer_max = 0.6;

  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);
  VehicleState s = travelling(8.0);
  StepDiagnostics d;

  const double weight = p.mass * slipx::kGravity;
  for (int i = 0; i < 6000; ++i) {
    const double steer = 0.55 * std::sin(0.01 * i);
    const double accel = 8.0 * std::cos(0.003 * i);
    model->step(s, DriveInput{steer, accel}, kDt, &d);

    const double total = d.fz[0] + d.fz[1] + d.fz[2] + d.fz[3];
    ASSERT_NEAR(total, weight, 1e-9) << "step " << i;
    for (unsigned w = 0; w < slipx::kWheelCount; ++w) {
      ASSERT_GE(d.fz[w], 0.0) << "step " << i << " wheel " << w;
    }
  }
}

// No tyre may exceed its own friction budget at any point of any manoeuvre.
// The bound the whole tier rests on: the friction ellipse is only meaningful
// if it actually bounds what comes out of the assembled model.
TEST(InvariantsL2, NoTyreEverExceedsItsFrictionBudget) {
  VehicleParams p = reference_params();
  p.h_cog = 0.10;
  p.steer_max = 0.5;

  auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);
  VehicleState s = travelling(9.0);
  StepDiagnostics d;

  const slipx::WheelLoads statics = slipx::static_loads(p);
  const MfLite front = slipx::make_mf_lite(p.tyre_front, 0.5 * p.c_alpha_f,
                                           statics.fz[kFrontLeft]);
  const MfLite rear = slipx::make_mf_lite(p.tyre_rear, 0.5 * p.c_alpha_r,
                                          statics.fz[kRearLeft]);

  for (int i = 0; i < 5000; ++i) {
    model->step(s, DriveInput{0.45 * std::sin(0.012 * i),
                              10.0 * std::sin(0.005 * i)}, kDt, &d);
    for (unsigned w = 0; w < slipx::kWheelCount; ++w) {
      const bool is_front = (w == kFrontLeft || w == kFrontRight);
      const MfLite& t = is_front ? front : rear;
      const double fx_max = slipx::peak_longitudinal_force(t, d.fz[w]);
      const double fy_max = slipx::peak_lateral_force(t, d.fz[w]);
      if (fx_max <= 0.0 || fy_max <= 0.0) {
        ASSERT_EQ(d.fx[w], 0.0) << "step " << i << " wheel " << w;
        ASSERT_EQ(d.fy[w], 0.0) << "step " << i << " wheel " << w;
        continue;
      }
      const double demand = std::hypot(d.fx[w] / fx_max, d.fy[w] / fy_max);
      ASSERT_LE(demand, 1.0 + 1e-9) << "step " << i << " wheel " << w;
    }
  }
}

// Raising the CoG must make the car lift a wheel sooner, at the assembled tier
// and not only in load_transfer.hpp. Monotonic in the parameter, which is what
// makes CoG height something a user can reason about rather than tune.
// Note the lower bound on the sweep. A car only lifts a wheel if its static
// rollover threshold, g t / (2 h), is BELOW what its tyres can deliver, so
// with a 0.24 m track and mu near 1.1 nothing lifts below about 0.11 m of CoG
// height: it slides first, which is the behaviour a 1/10-scale car should
// have. Sweeping from 0.05 finds no lift at all and says nothing about
// monotonicity, which is how this case was first written.
TEST(InvariantsL2, ARaisedCoGLiftsAWheelAtALowerSpeed) {
  double previous_speed = 1e9;
  for (const double h : {0.12, 0.14, 0.17, 0.21}) {
    VehicleParams p = reference_params();
    p.h_cog = h;
    p.steer_max = 0.6;

    // A tall car at half a radian of steer lifts at around 2 m/s, so the
    // sweep starts below that and steps finely: starting at 3 m/s puts every
    // height at the first sample and the monotonicity means nothing.
    double lift_speed = 0.0;
    for (int step = 0; step < 250 && lift_speed == 0.0; ++step) {
      const double v = 1.0 + 0.01 * step;
      auto model = VehicleModel::create(Tier::L2_DoubleTrack, p);
      VehicleState s = travelling(v);
      StepDiagnostics d;
      for (int i = 0; i < 400; ++i) {
        model->step(s, DriveInput{0.5, hold_speed(s, v)}, kDt, &d);
      }
      for (unsigned w = 0; w < slipx::kWheelCount; ++w) {
        if (d.fz[w] == 0.0) lift_speed = v;
      }
    }
    ASSERT_GT(lift_speed, 0.0) << "no lift found at h_cog " << h;
    EXPECT_LT(lift_speed, previous_speed) << "h_cog " << h;
    previous_speed = lift_speed;
  }

  // The other side of the same statement: a low car slides rather than
  // lifting, at any speed it can reach.
  VehicleParams low = reference_params();
  low.h_cog = 0.05;
  low.steer_max = 0.6;
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, low);
  VehicleState s = travelling(low.v_max);
  StepDiagnostics d;
  for (int i = 0; i < 3000; ++i) {
    model->step(s, DriveInput{0.5, hold_speed(s, low.v_max)}, kDt, &d);
    for (unsigned w = 0; w < slipx::kWheelCount; ++w) {
      ASSERT_GT(d.fz[w], 0.0) << "a 0.05 m CoG should slide, not lift";
    }
  }
}

// CoG height, track width and tyre compound do nothing below L2 and something
// at L2. That is the teaching artefact the whole tier system exists for, and
// it is asserted rather than described.
TEST(InvariantsL2, ParametersInertBelowL2AreLiveAtL2) {
  auto radius = [](Tier tier, const VehicleParams& p) {
    auto model = VehicleModel::create(tier, p);
    VehicleState s = travelling(7.0);
    StepDiagnostics d;
    for (int i = 0; i < 4000; ++i) {
      model->step(s, DriveInput{0.10, hold_speed(s, 7.0)}, kDt, &d);
    }
    return s.speed() / std::fabs(s.yaw_rate());
  };

  VehicleParams low = reference_params();
  VehicleParams high = reference_params();
  high.h_cog = 3.0 * low.h_cog;

  // L1 cannot see it at all: same trajectory, bit for bit.
  EXPECT_EQ(radius(Tier::L1_Bicycle, low), radius(Tier::L1_Bicycle, high));

  // L2 can.
  EXPECT_NE(radius(Tier::L2_DoubleTrack, low),
            radius(Tier::L2_DoubleTrack, high));

  // Same for track width, which L1 does not carry either.
  VehicleParams narrow = reference_params();
  VehicleParams wide = reference_params();
  wide.track_front = 2.0 * narrow.track_front;
  wide.track_rear = 2.0 * narrow.track_rear;
  EXPECT_EQ(radius(Tier::L1_Bicycle, narrow), radius(Tier::L1_Bicycle, wide));
  EXPECT_NE(radius(Tier::L2_DoubleTrack, narrow),
            radius(Tier::L2_DoubleTrack, wide));
}

}  // namespace
