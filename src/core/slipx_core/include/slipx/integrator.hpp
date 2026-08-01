// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Fixed-step integrators (CORE-13).
//
// Both are header-only templates over a compile-time state size and a
// callable, so they inline into each tier's step and allocate nothing
// (CORE-01). Neither reads a clock, a global, or anything but its arguments
// (CORE-03, CORE-04).
//
// Summation order is written out explicitly and must not be "tidied". The RK4
// combination below is the one whose rounding the reference trajectory hashes
// were produced with; regrouping it is a behaviour change even though it is an
// identity in exact arithmetic (NFR-02).
//
// Which integrator was used is part of the run manifest (SIM-06), because two
// runs of the same car with the same seed and different integrators are not
// the same run and a leaderboard that treats them as one is wrong.

#ifndef SLIPX_INTEGRATOR_HPP
#define SLIPX_INTEGRATOR_HPP

#include <array>
#include <cstddef>

namespace slipx {

enum class Integrator {
  // Fourth-order Runge-Kutta. Four derivative evaluations per step. The
  // default: at 1 kHz it is far more accurate than the model it is
  // integrating, so the error budget is spent on physics rather than on
  // integration.
  kRK4,
  // Semi-implicit (symplectic) Euler. One derivative evaluation per step:
  // velocities first, then positions using the NEW velocities. Roughly four
  // times cheaper and markedly better behaved than explicit Euler for the
  // oscillatory yaw modes, which is why explicit Euler is not offered at all.
  // Offered for mass RL rollouts where step cost dominates.
  kSemiImplicitEuler
};

inline const char* to_string(Integrator i) {
  switch (i) {
    case Integrator::kRK4: return "rk4";
    case Integrator::kSemiImplicitEuler: return "semi_implicit_euler";
  }
  return "unknown";
}

// The state vector convention shared by both integrators and by every tier:
// indices [0, n_vel) are velocity-like (their derivatives are accelerations),
// indices [n_vel, N) are position-like (their derivatives are the velocities).
// Semi-implicit Euler needs that split; RK4 does not care but the layout is
// kept uniform so a tier does not have to know which integrator it is under.
template <std::size_t N>
using StateVec = std::array<double, N>;

// y' = f(y). f must be pure: same input, same output, no captured mutable
// state. Every tier's derivative satisfies this by construction because
// VehicleModel::step is const (CORE-03).
template <std::size_t N, class Deriv>
inline void integrate_rk4(StateVec<N>& y, double dt, const Deriv& f) {
  const StateVec<N> k1 = f(y);

  StateVec<N> tmp;
  for (std::size_t i = 0; i < N; ++i) tmp[i] = y[i] + 0.5 * dt * k1[i];
  const StateVec<N> k2 = f(tmp);

  for (std::size_t i = 0; i < N; ++i) tmp[i] = y[i] + 0.5 * dt * k2[i];
  const StateVec<N> k3 = f(tmp);

  for (std::size_t i = 0; i < N; ++i) tmp[i] = y[i] + dt * k3[i];
  const StateVec<N> k4 = f(tmp);

  // Fixed grouping. Do not refactor into a loop over an array of k vectors
  // with an accumulate: the addition order is part of the result.
  for (std::size_t i = 0; i < N; ++i) {
    y[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }
}

// Velocities advance on the old derivative; positions then advance on the
// updated velocities, which is what makes this symplectic rather than merely
// explicit. n_vel is the split point described above.
template <std::size_t N, class Deriv>
inline void integrate_semi_implicit(StateVec<N>& y, double dt,
                                    std::size_t n_vel, const Deriv& f) {
  const StateVec<N> k = f(y);
  for (std::size_t i = 0; i < n_vel; ++i) y[i] += dt * k[i];

  // Second evaluation sees the updated velocities. This is the "semi" part;
  // reusing k here would silently degrade the scheme to explicit Euler.
  const StateVec<N> k2 = f(y);
  for (std::size_t i = n_vel; i < N; ++i) y[i] += dt * k2[i];
}

// Dispatch shared by every tier.
template <std::size_t N, class Deriv>
inline void integrate(Integrator scheme, StateVec<N>& y, double dt,
                      std::size_t n_vel, const Deriv& f) {
  if (scheme == Integrator::kSemiImplicitEuler) {
    integrate_semi_implicit<N>(y, dt, n_vel, f);
  } else {
    integrate_rk4<N>(y, dt, f);
  }
}

}  // namespace slipx

#endif  // SLIPX_INTEGRATOR_HPP
