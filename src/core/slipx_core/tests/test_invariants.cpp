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

}  // namespace
