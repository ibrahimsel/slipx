// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// CORE-13: the two fixed-step integrators.
//
// Tested against problems with closed-form solutions rather than against the
// vehicle model, so that an integrator bug and a physics bug cannot hide
// behind each other. The convergence-order tests are the ones that matter: an
// integrator that is merely "close" is indistinguishable from one that has
// silently degraded to first order until somebody halves the step and gets a
// different answer.

#include <gtest/gtest.h>

#include <cmath>

#include "slipx/integrator.hpp"

namespace {

using slipx::Integrator;
using slipx::StateVec;

// Measures the observed order of convergence: halving the step should divide
// the error by 2^order, so the log ratio recovers the order.
template <class Stepper, class Exact>
double observed_order(const Stepper& advance, const Exact& exact, double t_end,
                      double dt_coarse) {
  const double e_coarse = std::fabs(advance(dt_coarse) - exact(t_end));
  const double e_fine = std::fabs(advance(0.5 * dt_coarse) - exact(t_end));
  return std::log2(e_coarse / e_fine);
}

// y' = -y, y(0) = 1, y(t) = exp(-t). One state, no velocity/position split,
// so n_vel is irrelevant to RK4 here.
TEST(RK4, IsFourthOrderOnAScalarDecay) {
  const auto advance = [](double dt) {
    StateVec<1> y{1.0};
    const int n = static_cast<int>(std::lround(1.0 / dt));
    for (int i = 0; i < n; ++i) {
      slipx::integrate_rk4<1>(y, dt, [](const StateVec<1>& q) {
        return StateVec<1>{-q[0]};
      });
    }
    return y[0];
  };
  const auto exact = [](double t) { return std::exp(-t); };

  EXPECT_NEAR(advance(0.01), exact(1.0), 1e-9);
  EXPECT_NEAR(observed_order(advance, exact, 1.0, 0.05), 4.0, 0.15);
}

// Harmonic oscillator: [v, x] with v' = -x, x' = v. Layout matches the
// library convention, velocity-like index first.
struct Oscillator {
  StateVec<2> operator()(const StateVec<2>& q) const {
    return StateVec<2>{-q[1], q[0]};
  }
};

TEST(RK4, IsFourthOrderOnAnOscillator) {
  const auto advance = [](double dt) {
    StateVec<2> y{0.0, 1.0};  // v = 0, x = 1
    const int n = static_cast<int>(std::lround(2.0 / dt));
    for (int i = 0; i < n; ++i) slipx::integrate_rk4<2>(y, dt, Oscillator{});
    return y[1];
  };
  const auto exact = [](double t) { return std::cos(t); };

  EXPECT_NEAR(advance(0.001), exact(2.0), 1e-12);
  EXPECT_NEAR(observed_order(advance, exact, 2.0, 0.05), 4.0, 0.2);
}

TEST(SemiImplicitEuler, IsFirstOrder) {
  const auto advance = [](double dt) {
    StateVec<2> y{0.0, 1.0};
    const int n = static_cast<int>(std::lround(2.0 / dt));
    for (int i = 0; i < n; ++i) {
      slipx::integrate_semi_implicit<2>(y, dt, 1, Oscillator{});
    }
    return y[1];
  };
  const auto exact = [](double t) { return std::cos(t); };

  EXPECT_NEAR(observed_order(advance, exact, 2.0, 0.02), 1.0, 0.15);
}

// The reason explicit Euler is not offered at all. Over many cycles the
// symplectic scheme keeps the oscillator's energy bounded, while explicit
// Euler grows it without limit; the yaw modes of a bicycle model are
// oscillatory, and an integrator that inflates them turns a marginal
// controller into a diverging one for reasons that have nothing to do with
// the physics.
TEST(SemiImplicitEuler, KeepsOscillatorEnergyBounded) {
  StateVec<2> y{0.0, 1.0};
  const double dt = 0.01;
  const double e0 = y[0] * y[0] + y[1] * y[1];

  double e_max = e0;
  for (int i = 0; i < 100000; ++i) {  // ~160 cycles
    slipx::integrate_semi_implicit<2>(y, dt, 1, Oscillator{});
    e_max = std::fmax(e_max, y[0] * y[0] + y[1] * y[1]);
  }
  // Bounded by a small step-dependent margin, not drifting upward.
  EXPECT_LT(e_max / e0, 1.0 + 2.0 * dt);
}

// The "semi" in semi-implicit: positions must advance on the UPDATED
// velocities. Reusing the first derivative evaluation would silently make this
// explicit Euler, which passes the order test above and fails the energy test.
// This asserts the mechanism directly, on one step of a case where the two
// schemes differ by a known amount.
TEST(SemiImplicitEuler, PositionsUseTheUpdatedVelocities) {
  StateVec<2> y{0.0, 1.0};  // v = 0, x = 1
  const double dt = 0.1;
  slipx::integrate_semi_implicit<2>(y, dt, 1, Oscillator{});

  // v_new = v + dt * (-x)  = -0.1
  // x_new = x + dt * v_new = 1 - 0.01 = 0.99   (explicit Euler would give 1.0)
  EXPECT_DOUBLE_EQ(y[0], -0.1);
  EXPECT_DOUBLE_EQ(y[1], 0.99);
}

TEST(Integrate, DispatchesToTheRequestedScheme) {
  const auto run = [](Integrator scheme) {
    StateVec<2> y{0.0, 1.0};
    slipx::integrate<2>(scheme, y, 0.1, 1, Oscillator{});
    return y;
  };

  const auto rk4 = run(Integrator::kRK4);
  const auto sie = run(Integrator::kSemiImplicitEuler);
  EXPECT_DOUBLE_EQ(sie[1], 0.99);
  EXPECT_NE(rk4[1], sie[1]) << "the choice must actually change the answer";
  EXPECT_NEAR(rk4[1], std::cos(0.1), 1e-6);
}

// NFR-02: the same call twice, bit for bit. Not approximately.
TEST(Integrate, IsBitIdenticalAcrossRepeatedRuns) {
  const auto run = [](Integrator scheme) {
    StateVec<2> y{0.3, 1.7};
    for (int i = 0; i < 5000; ++i) {
      slipx::integrate<2>(scheme, y, 1.0e-3, 1, Oscillator{});
    }
    return y;
  };

  for (const Integrator scheme :
       {Integrator::kRK4, Integrator::kSemiImplicitEuler}) {
    const auto a = run(scheme);
    const auto b = run(scheme);
    EXPECT_EQ(a[0], b[0]);
    EXPECT_EQ(a[1], b[1]);
  }
}

TEST(Integrator, HasStableNamesForTheManifest) {
  // SIM-06 hashes these strings into the run manifest. Renaming one silently
  // invalidates every stored manifest, so the names are pinned here.
  EXPECT_STREQ(slipx::to_string(Integrator::kRK4), "rk4");
  EXPECT_STREQ(slipx::to_string(Integrator::kSemiImplicitEuler),
               "semi_implicit_euler");
}

}  // namespace
