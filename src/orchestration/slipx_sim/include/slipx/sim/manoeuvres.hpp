// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Standard manoeuvres as policies, and the canonical P0 conformance run.
//
// The identification manoeuvre library proper is slipx_id and arrives in P2
// (ID-02). What lives here is the small subset the simulator itself needs: a
// step steer for the conformance run, and the speed hold every manoeuvre is
// built on.
//
// The step steer is the P0 exit gate. A third party pip-installs the package,
// loads a car, integrates a step steer and gets the same trajectory hash as
// CI; that sentence only means something if "a step steer" denotes exactly one
// sequence of numbers, so it is defined here once and used by the C++
// conformance binary, the Python bindings and CI alike.

#ifndef SLIPX_SIM_MANOEUVRES_HPP
#define SLIPX_SIM_MANOEUVRES_HPP

#include "slipx/sim/simulation.hpp"

namespace slipx {
namespace sim {

// Proportional speed hold. Deliberately simple and deliberately not tuned:
// it exists to keep the speed roughly constant while the lateral dynamics are
// what is being observed, and a cleverer one would put its own dynamics into
// every manoeuvre that used it.
inline double hold_speed(const VehicleState& s, double target,
                         double gain = 4.0) {
  return gain * (target - s.vel_body.x);
}

struct StepSteerSpec {
  double target_speed = 5.0;   // held throughout                    [m/s]
  double steer = 0.10;         // step amplitude, positive is left   [rad]
  double t_step = 1.0;         // when the step is applied             [s]
  double speed_gain = 4.0;     // proportional gain on the speed hold
};

// Straight-line run-up, then a single instantaneous steer step held to the
// end. The transient after the step is where the yaw dynamics, the understeer
// gradient and (from L2) the tyre relaxation length all show themselves,
// which is why this is the manoeuvre both the conformance hash and the
// identification of sigma are built on.
inline Policy step_steer(const StepSteerSpec& spec = {}) {
  return [spec](const VehicleState& s, double t, Rng&) {
    const double steer = (t >= spec.t_step) ? spec.steer : 0.0;
    return DriveInput{steer,
                      hold_speed(s, spec.target_speed, spec.speed_gain)};
  };
}

// ---------------------------------------------------------- conformance run
//
// The canonical scenario behind the published reference hashes.
//
// Every number in it is pinned, including the ones that look arbitrary. It
// uses the VehicleParams struct defaults rather than a parameter file, so that
// the run has no dependency on slipx_schema and can be executed by a consumer
// who has only the core and the orchestrator (CORE-01). Those defaults are
// PROVISIONAL and describe no measured car (NFR-08); the run is a determinism
// check, not a physics claim.
//
// Changing anything here changes the reference hashes, and that is a release
// event: it invalidates comparisons against every previously published result.

struct ConformanceSpec {
  Tier tier = Tier::L1_Bicycle;
  Integrator integrator = Integrator::kRK4;
  double dt = 1.0e-3;          // 1 kHz (SIM-01 default)
  double duration = 5.0;       //                                      [s]
  double initial_speed = 5.0;  //                                    [m/s]
  std::uint64_t seed = 20260801;
};

inline Simulation make_conformance_run(const ConformanceSpec& spec = {}) {
  SimulationConfig config;
  config.dt = spec.dt;
  config.integrator = spec.integrator;
  config.master_seed = spec.seed;

  Simulation sim(config);

  AgentSpec agent;
  agent.name = "conformance";
  agent.tier = spec.tier;
  agent.params = VehicleParams{};  // provisional defaults, pinned by version
  agent.initial_state.vel_body.x = spec.initial_speed;
  agent.policy = step_steer(StepSteerSpec{});
  sim.add_agent(std::move(agent));

  return sim;
}

}  // namespace sim
}  // namespace slipx

#endif  // SLIPX_SIM_MANOEUVRES_HPP
