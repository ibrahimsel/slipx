// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/sim/simulation.hpp"

#include <cmath>
#include <stdexcept>

#include "slipx/sim/build_info.hpp"

namespace slipx {
namespace sim {
namespace {

// Digest of a parameter set, for the manifest. Fixed field order, and it
// covers every field: two runs whose cars differ in any parameter must be
// distinguishable, including in the ones a given tier ignores, because the
// tier can change without the file changing.
std::string params_digest(const VehicleParams& p) {
  TrajectoryHash h;
  h.update(p.mass);
  h.update(p.izz);
  h.update(p.ixx);
  h.update(p.iyy);
  h.update(p.lf);
  h.update(p.lr);
  h.update(p.track_front);
  h.update(p.track_rear);
  h.update(p.h_cog);
  h.update(p.wheel_radius);
  h.update(p.c_alpha_f);
  h.update(p.c_alpha_r);
  h.update(p.mu_clip);
  h.update(p.accel_max);
  h.update(p.decel_max);
  h.update(p.v_max);
  h.update(p.steer_max);
  h.update(p.drag_coeff);
  h.update(p.roll_resist);
  h.update(p.v_eps);
  h.update_u64(static_cast<std::uint64_t>(p.provenance));
  return h.hex();
}

}  // namespace

Simulation::Simulation(SimulationConfig config) : config_(config) {
  if (!(config_.dt > 0.0)) {
    throw std::invalid_argument("slipx_sim: dt must be positive [s]");
  }
  if (config_.hash_stride == 0) {
    throw std::invalid_argument("slipx_sim: hash_stride must be at least 1");
  }
}

std::size_t Simulation::add_agent(AgentSpec spec) {
  const std::size_t index = agents_.size();

  Agent agent;
  agent.name = std::move(spec.name);
  // create() throws with the core's own message if the parameters are
  // impossible or the tier is not implemented. Not caught and rephrased here:
  // the core's message names the offending field, and wrapping it would only
  // bury that.
  agent.model = VehicleModel::create(spec.tier, spec.params,
                                     config_.integrator);
  agent.state = spec.initial_state;
  agent.initial_state = spec.initial_state;
  agent.policy = std::move(spec.policy);
  agent.seed = derive_seed(config_.master_seed, index);
  agent.rng = Rng(agent.seed);

  agents_.push_back(std::move(agent));
  // Sized once, here, so that advance() never allocates.
  pending_inputs_.resize(agents_.size());
  return index;
}

void Simulation::check_index(std::size_t i) const {
  if (i >= agents_.size()) {
    throw std::out_of_range("slipx_sim: agent index out of range");
  }
}

double Simulation::time() const {
  // Not an accumulated sum. See the header.
  return static_cast<double>(steps_) * config_.dt;
}

void Simulation::hash_states() {
  if (steps_ % config_.hash_stride != 0) return;
  for (Agent& a : agents_) a.hash.update(a.state);
}

void Simulation::advance() {
  const double t = time();

  // Phase one: collect every command against the world as it is now. No agent
  // has moved yet, so no policy can see another agent's future.
  for (std::size_t i = 0; i < agents_.size(); ++i) {
    Agent& a = agents_[i];
    pending_inputs_[i] = a.policy ? a.policy(a.state, t, a.rng) : DriveInput{};
  }

  if (logging_inputs_) {
    input_log_.insert(input_log_.end(), pending_inputs_.begin(),
                      pending_inputs_.end());
  }

  // Phase two: everybody moves.
  for (std::size_t i = 0; i < agents_.size(); ++i) {
    Agent& a = agents_[i];
    a.model->step(a.state, pending_inputs_[i], config_.dt, &a.diagnostics);
  }

  ++steps_;
  hash_states();
}

void Simulation::run(std::uint64_t steps) {
  for (std::uint64_t i = 0; i < steps; ++i) advance();
}

void Simulation::run_for(double duration) {
  const double n = std::round(duration / config_.dt);
  if (n < 0.0) {
    throw std::invalid_argument("slipx_sim: duration must not be negative");
  }
  run(static_cast<std::uint64_t>(n));
}

void Simulation::reset() {
  for (Agent& a : agents_) {
    a.state = a.initial_state;
    a.diagnostics = StepDiagnostics{};
    a.rng = Rng(a.seed);
    a.hash = TrajectoryHash{};
  }
  steps_ = 0;
  input_log_.clear();
}

void Simulation::replay(const std::vector<DriveInput>& log) {
  if (agents_.empty()) return;
  if (log.size() % agents_.size() != 0) {
    throw std::invalid_argument(
        "slipx_sim: input log length is not a multiple of the agent count; "
        "it was recorded from a different scenario");
  }

  reset();
  const std::size_t n_steps = log.size() / agents_.size();
  for (std::size_t step = 0; step < n_steps; ++step) {
    for (std::size_t i = 0; i < agents_.size(); ++i) {
      Agent& a = agents_[i];
      a.model->step(a.state, log[step * agents_.size() + i], config_.dt,
                    &a.diagnostics);
    }
    ++steps_;
    hash_states();
  }
}

const VehicleState& Simulation::state(std::size_t i) const {
  check_index(i);
  return agents_[i].state;
}

VehicleState& Simulation::state(std::size_t i) {
  check_index(i);
  return agents_[i].state;
}

const StepDiagnostics& Simulation::diagnostics(std::size_t i) const {
  check_index(i);
  return agents_[i].diagnostics;
}

const VehicleModel& Simulation::model(std::size_t i) const {
  check_index(i);
  return *agents_[i].model;
}

Rng& Simulation::rng(std::size_t i) {
  check_index(i);
  return agents_[i].rng;
}

std::string Simulation::agent_trajectory_hash(std::size_t i) const {
  check_index(i);
  return agents_[i].hash.hex();
}

std::string Simulation::trajectory_hash() const {
  // Folds the per-agent hashes together in agent order. Order matters and is
  // fixed by insertion: a race is not the same race with the cars renumbered.
  TrajectoryHash h;
  for (const Agent& a : agents_) h.update_u64(a.hash.value());
  return h.hex();
}

RunManifest Simulation::manifest() const {
  RunManifest m;
  m.capture_build_info();
  m.schema_version = config_.schema_version;
  m.dt = config_.dt;
  m.steps = steps_;
  m.integrator = to_string(config_.integrator);
  m.master_seed = config_.master_seed;

  m.agents.reserve(agents_.size());
  m.agent_trajectory_hashes.reserve(agents_.size());
  for (const Agent& a : agents_) {
    AgentManifest am;
    am.name = a.name;
    am.tier = to_string(a.model->tier());
    am.params_digest = params_digest(a.model->params());
    am.seed = a.seed;
    m.agents.push_back(std::move(am));
    m.agent_trajectory_hashes.push_back(a.hash.hex());
  }
  m.trajectory_hash = trajectory_hash();
  return m;
}

}  // namespace sim
}  // namespace slipx
