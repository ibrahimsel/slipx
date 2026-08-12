// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Shared fixtures for the core test suite.
//
// The parameter set below is PROVISIONAL (NFR-08): it is plausible for a
// 1/10-scale car and has not been measured against one. It exists so the tests
// have something concrete to integrate, and no claim about a real vehicle
// rests on it. When the registry has an identified set, the analytical tests
// should be run against that too, but their assertions are closed-form and do
// not depend on which set is used.

#ifndef SLIPX_TEST_SUPPORT_HPP
#define SLIPX_TEST_SUPPORT_HPP

#include <cmath>
#include <vector>

#include "slipx/vehicle_model.hpp"

namespace slipx_test {

using slipx::DriveInput;
using slipx::StepDiagnostics;
using slipx::VehicleParams;
using slipx::VehicleState;

inline VehicleParams reference_params() {
  VehicleParams p;              // the struct defaults, which are provisional
  p.mass = 3.5;                 //                                      [kg]
  p.izz = 0.05;                 //                                  [kg m^2]
  p.lf = 0.16;                  //                                       [m]
  p.lr = 0.16;                  //                                       [m]
  p.h_cog = 0.06;               //                                       [m]
  p.c_alpha_f = 420.0;          //                                   [N/rad]
  p.c_alpha_r = 455.0;          //                                   [N/rad]
  p.mu_clip = 1.1;              //                                       [-]
  return p;
}

// A state at rest at the origin, pointing along world +x.
inline VehicleState at_rest() { return VehicleState{}; }

inline VehicleState travelling(double vx) {
  VehicleState s;
  s.vel_body.x = vx;
  return s;
}

// Proportional speed hold. Every manoeuvre below needs one and none of them
// needs a good one: the point is to hold a speed while the lateral dynamics
// are what is under test.
inline double hold_speed(const VehicleState& s, double target) {
  return 4.0 * (target - s.vel_body.x);
}

// Advance a model for a fixed duration under a caller-supplied policy.
// Returns the final diagnostics so a test can read the settled condition.
template <class Policy>
StepDiagnostics run_for(const slipx::VehicleModel& model, VehicleState& s,
                        double duration, double dt, const Policy& policy) {
  StepDiagnostics diag;
  const int n = static_cast<int>(std::lround(duration / dt));
  for (int i = 0; i < n; ++i) {
    const DriveInput u = policy(s, static_cast<double>(i) * dt);
    model.step(s, u, dt, &diag);
  }
  return diag;
}

// Sampled trajectory, for tests that compare two runs point by point.
struct Sample {
  double t = 0.0;
  VehicleState state;
};

template <class Policy>
std::vector<Sample> trajectory(const slipx::VehicleModel& model,
                               VehicleState s, double duration, double dt,
                               const Policy& policy, int every = 1) {
  std::vector<Sample> out;
  const int n = static_cast<int>(std::lround(duration / dt));
  out.reserve(static_cast<std::size_t>(n / every + 1));
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) * dt;
    const DriveInput u = policy(s, t);
    model.step(s, u, dt, nullptr);
    if ((i + 1) % every == 0) out.push_back(Sample{t + dt, s});
  }
  return out;
}

inline constexpr double kDefaultDt = 1.0e-3;  // SIM-01 default rate, 1 kHz

}  // namespace slipx_test

#endif  // SLIPX_TEST_SUPPORT_HPP
