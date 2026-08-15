// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The orchestrator: N agents, one fixed step, in lockstep (SIM-01, SIM-02,
// SIM-03, SIM-07, SIM-09).
//
// Everything here is single-process and single-threaded. The ROS 2 barrier
// with acknowledgements and a timeout policy is SIM-05 and arrives in P3; this
// is the in-process mode it will be checked against, and the one a course
// assignment or an RL rollout actually wants.
//
// Three properties are worth stating because they are what the determinism
// claim is built from:
//
//   Commands are collected before any agent moves. Every policy sees the world
//   as it was at the start of the step, so a result cannot depend on the order
//   the agents happen to be stored in. This is the lockstep barrier, minus the
//   networking.
//
//   Time is steps * dt, never an accumulated sum. Adding 0.001 to a double ten
//   thousand times does not give 10.0, and a simulation whose clock drifts
//   against its own step count cannot be replayed against a recorded input
//   sequence.
//
//   Randomness is per agent and derived from one master seed. Adding an agent
//   to a scenario must not change the numbers every other agent draws.
//
// A note on allocation: slipx_core promises not to allocate inside step, and
// that promise is tested. slipx_sim makes no such promise, because a policy is
// a std::function the caller supplied and what it does is their business. What
// slipx_sim does guarantee is that its own per-step bookkeeping allocates
// nothing after the agents have been added.

#ifndef SLIPX_SIM_SIMULATION_HPP
#define SLIPX_SIM_SIMULATION_HPP

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "slipx/sense/rng.hpp"
#include "slipx/sim/hash.hpp"
#include "slipx/sim/manifest.hpp"
#include "slipx/vehicle_model.hpp"

namespace slipx {
namespace sim {

// The seeded generator lives in slipx_sense, the lowest C++ layer above the
// core that needs randomness (ADR-0037). It is named here as well, because a
// run's per-agent random stream is an orchestration concept even though the
// type that produces it is not.
using sense::Rng;
using sense::derive_seed;

// An in-process agent policy (SIM-02): a callable, not a topic, not a socket.
// Receives the agent's state at the start of the step, the simulation time,
// and the agent's own random stream. It must be a pure function of those three
// things if the run is to be reproducible; a policy that reads a wall clock is
// outside anything this library can promise, and is the first thing to check
// when a replay fails.
using Policy = std::function<DriveInput(const VehicleState& state, double time,
                                        Rng& rng)>;

struct AgentSpec {
  std::string name = "car";
  Tier tier = Tier::L1_Bicycle;
  VehicleParams params{};
  VehicleState initial_state{};
  Policy policy{};   // empty means coast: zero steer, zero demand
};

struct SimulationConfig {
  // SIM-01: fixed step, default 1 kHz, decoupled from any sensor rate.
  double dt = 1.0e-3;
  Integrator integrator = Integrator::kRK4;
  std::uint64_t master_seed = 0;

  // How often states are folded into the trajectory hash. One means every
  // step, which is the strongest check and the default. A larger stride is
  // for long runs where hashing dominates; it weakens the check, because
  // divergence that appears and decays between samples goes unseen.
  std::uint64_t hash_stride = 1;

  // Schema version the parameters were parsed with, for the manifest. Empty
  // when parameters were built in code.
  std::string schema_version;
};

class Simulation {
 public:
  explicit Simulation(SimulationConfig config = {});

  // Movable, not copyable. A Simulation owns its models through unique_ptr, so
  // a copy would have to decide whether the clone shares them or rebuilds
  // them, and both answers are wrong: sharing breaks the one-model-per-agent
  // assumption that makes instances independent, and rebuilding silently
  // produces a similar run rather than the same one. Callers who want a
  // second run should build a second Simulation from the same spec, or call
  // reset(), which is exact.
  Simulation(const Simulation&) = delete;
  Simulation& operator=(const Simulation&) = delete;
  Simulation(Simulation&&) = default;
  Simulation& operator=(Simulation&&) = default;

  // Returns the agent's index. Throws std::invalid_argument if the tier or
  // parameters are unusable, with the message from the core's own validator.
  //
  // There is no upper bound on the agent count, by construction rather than by
  // a large constant (SIM-09).
  std::size_t add_agent(AgentSpec spec);

  // One fixed step for every agent: collect all commands, then move everybody.
  void advance();

  void run(std::uint64_t steps);
  // Rounds to the nearest whole number of steps, because a partial step would
  // be a second step size and would make the run unreproducible from the
  // manifest.
  void run_for(double duration);

  // Returns all agents to their initial states and rewinds the clock, the
  // hashes and the random streams. What it does not do is rebuild the models,
  // so a reset run is the same run, not a similar one.
  void reset();

  std::size_t agent_count() const { return agents_.size(); }
  double time() const;
  std::uint64_t step_count() const { return steps_; }
  double dt() const { return config_.dt; }

  const VehicleState& state(std::size_t i) const;
  // Mutable access is for scenario setup and for tests. Writing to a state
  // mid-run is legal and is how a scenario teleports a car to a grid slot, but
  // it is not recorded in the input log, so a run that does it cannot be
  // replayed from the log alone.
  VehicleState& state(std::size_t i);
  const StepDiagnostics& diagnostics(std::size_t i) const;
  const VehicleModel& model(std::size_t i) const;
  Rng& rng(std::size_t i);

  std::string trajectory_hash() const;
  std::string agent_trajectory_hash(std::size_t i) const;

  // SIM-06. Includes the hashes accumulated so far, so calling it mid-run
  // gives a manifest for the run up to now.
  RunManifest manifest() const;

  // ------------------------------------------------------------- replay
  //
  // SIM-07: given the same manifest and the same input sequence, the replay is
  // bit-identical. Recording is off by default because the log grows by
  // (agents * 16 bytes) per step and most runs do not need it.
  void set_input_logging(bool enabled) { logging_inputs_ = enabled; }
  bool input_logging() const { return logging_inputs_; }

  // Flat, step-major: entry (step * agent_count + agent).
  const std::vector<DriveInput>& input_log() const { return input_log_; }

  // Resets and re-runs from a recorded log, ignoring the agents' policies
  // entirely. This is the operation a leaderboard appeal performs: the
  // policies may be gone, may be proprietary, or may have been the thing under
  // dispute, and none of that should stand between an adjudicator and the
  // trajectory.
  void replay(const std::vector<DriveInput>& log);

 private:
  struct Agent {
    std::string name;
    std::unique_ptr<VehicleModel> model;
    VehicleState state;
    VehicleState initial_state;
    StepDiagnostics diagnostics;
    Policy policy;
    Rng rng;
    std::uint64_t seed = 0;
    TrajectoryHash hash;
  };

  void hash_states();
  void check_index(std::size_t i) const;

  SimulationConfig config_;
  std::vector<Agent> agents_;
  // Preallocated in add_agent so that advance() does no allocation of its own.
  std::vector<DriveInput> pending_inputs_;
  std::vector<DriveInput> input_log_;
  std::uint64_t steps_ = 0;
  bool logging_inputs_ = false;
};

}  // namespace sim
}  // namespace slipx

#endif  // SLIPX_SIM_SIMULATION_HPP
