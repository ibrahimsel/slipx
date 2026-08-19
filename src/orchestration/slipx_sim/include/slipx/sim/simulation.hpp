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
#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "slipx/contact.hpp"
#include "slipx/sense/rng.hpp"
#include "slipx/sim/hash.hpp"
#include "slipx/sim/mailbox.hpp"
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

// What the barrier does about a step whose command never arrived
// (ADR-0044). Only meaningful for a mailbox-driven agent: a policy callable
// is synchronous and cannot miss.
enum class TimeoutPolicy {
  // Block until the command arrives. Strict lockstep: one hung agent hangs
  // the race, and in exchange the trajectory cannot depend on timing.
  kWait,
  // Do not step the agent this step; it resumes exactly where it paused
  // when commands return. A pause for debugging and lenient practice, not
  // physics: the car holds mid-corner speed in suspension while the world
  // moves on.
  kFreeze,
  // Step with the neutral input, the same coasting an agent with no policy
  // gets.
  kCoast,
  // The agent is out (ADR-0042's machinery, DnfCause::kTimeout): frozen in
  // place, velocities zeroed, a stationary obstacle.
  kDnf,
};

inline const char* to_string(TimeoutPolicy policy) {
  switch (policy) {
    case TimeoutPolicy::kWait: return "wait";
    case TimeoutPolicy::kFreeze: return "freeze";
    case TimeoutPolicy::kCoast: return "coast";
    case TimeoutPolicy::kDnf: return "dnf";
  }
  return "unknown";
}

struct AgentSpec {
  std::string name = "car";
  Tier tier = Tier::L1_Bicycle;
  VehicleParams params{};
  VehicleState initial_state{};
  Policy policy{};   // empty means coast: zero steer, zero demand

  // The asynchronous alternative to a policy (ADR-0044): commands arrive
  // step-tagged through this mailbox, posted by another thread or, later, a
  // transport. Setting both a policy and a mailbox is refused: an agent has
  // one command source. When the entry for a step is missing, timeout_policy
  // answers; it is meaningless without a mailbox and ignored there.
  std::shared_ptr<CommandMailbox> mailbox{};
  TimeoutPolicy timeout_policy = TimeoutPolicy::kWait;

  // Collision footprint (ADR-0043): an oriented rectangle, overall length by
  // overall width, centred on the wheelbase midpoint. Both zero, the
  // default, means the agent declares no footprint and touches nothing, so
  // a scenario without footprints reproduces bit for bit what it produced
  // before contact existed. The car schema carries these as
  // geometry.length and geometry.width. Setting one without the other is
  // refused: a rectangle with one dimension is not a smaller footprint, it
  // is a mistake.
  double footprint_length = 0.0;   //                                    [m]
  double footprint_width = 0.0;    //                                    [m]
};

// Why an agent stopped racing (ADR-0042, ADR-0044); race control will
// extend this.
//
// The two rollover values name the UNLOADED side: kRolloverLeft means both
// left wheels reached zero vertical load, which is what happens in a hard
// left turn (positive ay loads the right wheels), and the car is pivoting
// about its right-hand contact patches. kTimeout is the kDnf answer to a
// barrier miss: the agent's command did not arrive and its timeout policy
// said that ends its run.
enum class DnfCause {
  kRolloverLeft,
  kRolloverRight,
  kTimeout,
};

inline const char* to_string(DnfCause cause) {
  switch (cause) {
    case DnfCause::kRolloverLeft: return "rollover: left wheels unloaded";
    case DnfCause::kRolloverRight: return "rollover: right wheels unloaded";
    case DnfCause::kTimeout: return "barrier timeout: no command arrived";
  }
  return "unknown";
}

// The discrete event that ended an agent's run. Carries its cause, because a
// DNF whose reason has to be reconstructed from the trajectory is a DNF that
// will be argued about.
struct DnfEvent {
  // The value step_count() returned after the advance that produced the
  // event, so the event names the first step at which the state was frozen.
  std::uint64_t step = 0;
  double time = 0.0;   // step * dt                                       [s]
  DnfCause cause = DnfCause::kRolloverLeft;
};

// How a run treats the wall clock.
//
// The two modes answer different questions and a run cannot answer both. In
// deterministic mode there is no clock at all: the simulation advances as
// fast as it can and the trajectory is a function of the inputs, which is
// what makes a hash worth comparing. In validation mode the run is paced
// against real time, so a stack under test experiences a scan arriving late
// the way it would on a car, and the trajectory then depends on how loaded
// the machine was.
//
// The manifest records which mode produced it, and says plainly that a
// validation run promises nothing about reproducibility. A manifest that
// claimed bit-identity for a run paced by a laptop's scheduler would be the
// most damaging single line in this library.
enum class RunMode {
  kDeterministic,
  kValidation,
};

inline const char* to_string(RunMode mode) {
  return mode == RunMode::kValidation ? "validation" : "deterministic";
}

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

  // Deterministic unless asked otherwise, and asking is deliberate: a run
  // that quietly became unreproducible because a default changed is the
  // failure this whole library is arranged to prevent.
  RunMode mode = RunMode::kDeterministic;

  // Validation mode only. How much faster than real time to run: 1.0 is real
  // time, 0.5 is half speed for a stack that cannot keep up, and larger
  // values compress a long run. Ignored in deterministic mode, where there is
  // no clock to be fast or slow against.
  double real_time_factor = 1.0;

  // Agent-to-agent contact (ADR-0043): one impulse per touching pair per
  // step, restitution and Coulomb friction, applied between steps to agents
  // that declare a footprint. The defaults are plausible for foam bumpers on
  // plastic shells and are fitted to nothing; see contact.hpp, which keeps
  // saying so. Recorded in the manifest and folded into the configuration
  // digest, because two runs that disagree here were different races.
  ContactParams contact{};

  // How long the barrier waits, in wall-clock seconds, for a mailbox entry
  // that has not arrived before ruling it a miss and applying the agent's
  // timeout policy (ADR-0044). Zero means a poll. Ignored for kWait agents,
  // whose barrier waits forever by definition. Wall clock, so a live run
  // with non-wait mailbox agents is reproducible from its input log rather
  // than by re-running; the manifest says which promise applies.
  double barrier_timeout = 0.0;
};

// One resolved contact, reported so a layer above (race control, the event
// stream) can interpret it; the sim resolves the physics and holds no
// opinion about fault (ADR-0046). The list for the most recent step is
// available through Simulation::contacts().
struct ContactEvent {
  std::uint64_t step = 0;    // the step count after the advance that hit
  std::uint32_t a = 0;       // agent indices, a < b
  std::uint32_t b = 0;
  Vec2 point;                // contact point, world                     [m]
  Vec2 normal;               // unit, from a toward b
  double jn = 0.0;           // normal impulse delivered               [N s]
  // Each car's contribution to the closing speed, positive toward the
  // other car; the sum is the closing speed (see contact.hpp).
  double approach_a = 0.0;   //                                        [m/s]
  double approach_b = 0.0;   //                                        [m/s]
};

// Everything a running simulation is, at one instant (SIM-08).
//
// The vehicle state is trivially copyable by design, so the expensive part of
// this is nothing. What makes it more than a memcpy of the states is the
// bookkeeping around them: each agent's random stream and each agent's
// running hash have to come back too, or the resumed run is a different run
// that happens to start from the same position.
struct AgentSnapshot {
  VehicleState state;
  StepDiagnostics diagnostics;
  Rng::State rng;
  std::uint64_t hash_state = 0;
  // Whether, and how, the agent's run had already ended. Without this a
  // restore could resurrect a rolled car, and a snapshot taken after an
  // event would restore into a run that disagrees with its own manifest.
  std::optional<DnfEvent> dnf;
};

struct SimulationSnapshot {
  std::uint64_t steps = 0;
  std::vector<AgentSnapshot> agents;

  // How much of the input log had been written. Restoring truncates the log
  // to this, so that resuming and running on produces the log the
  // uninterrupted run would have, rather than one with the discarded steps
  // still in it.
  std::size_t input_log_entries = 0;
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

  // ------------------------------------------------------------- events
  //
  // Rollover ends an agent's run (ADR-0042): after any step whose
  // diagnostics show both wheels of one side at zero vertical load, the
  // agent is DNF. Its policy is never called again, its model never steps
  // again, its pose freezes where the event found it and its velocities are
  // zeroed, so it remains in the world as a stationary obstacle. Below L2
  // the per-wheel loads are NaN and no agent can roll, which is a stated
  // limitation of those tiers rather than of the detection.
  bool agent_running(std::size_t i) const;

  // The event that ended the agent's run, or an empty optional while it is
  // still running.
  const std::optional<DnfEvent>& dnf(std::size_t i) const;

  // The collision footprint the agent declared (ADR-0043), read-only, for
  // whoever composes queries over the same boxes the contact pass collides
  // (the track world does). Zeroes, and has_footprint false, when none was
  // declared: such an agent touches nothing and is invisible to sensors,
  // one rule rather than two.
  bool has_footprint(std::size_t i) const;
  double footprint_half_length(std::size_t i) const;   //                 [m]
  double footprint_half_width(std::size_t i) const;    //                 [m]
  // How far the box centre (the wheelbase midpoint) sits ahead of the CoG.
  double footprint_centre_offset(std::size_t i) const; //                 [m]

  // Every contact the most recent step resolved, in pair order. Cleared and
  // refilled by each advance (and by each replayed step), so a consumer
  // polls it between steps; keeping history is the consumer's job.
  const std::vector<ContactEvent>& contacts() const { return contacts_; }

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

  // --------------------------------------------------------- snapshots
  //
  // SIM-08. Take a snapshot mid-run, carry on, and restore: the simulation is
  // then exactly where it was, and running on from there reproduces the run
  // that was never interrupted, bit for bit, in deterministic mode.
  SimulationSnapshot snapshot() const;

  // Throws std::invalid_argument if the snapshot describes a different number
  // of agents. It is not a general loader: a snapshot belongs to the
  // simulation that produced it, and restoring one into a differently
  // configured run would produce a plausible trajectory from a car that was
  // never there.
  void restore(const SimulationSnapshot& snapshot);

  RunMode mode() const { return config_.mode; }

 private:
  struct Agent {
    std::string name;
    std::unique_ptr<VehicleModel> model;
    VehicleState state;
    VehicleState initial_state;
    StepDiagnostics diagnostics;
    Policy policy;
    std::shared_ptr<CommandMailbox> mailbox;
    TimeoutPolicy timeout_policy = TimeoutPolicy::kWait;
    Rng rng;
    std::uint64_t seed = 0;
    TrajectoryHash hash;
    std::optional<DnfEvent> dnf;

    // Cached from the spec at add_agent (ADR-0043). Zero half-extents mean
    // no footprint: the agent is skipped by the contact pass entirely.
    double half_length = 0.0;
    double half_width = 0.0;
    double centre_offset = 0.0;   // wheelbase midpoint ahead of the CoG [m]
    bool has_footprint() const { return half_length > 0.0; }
  };

  void hash_states();
  void check_index(std::size_t i) const;
  // One agent's share of a step: skip it if it is DNF'd or the input is the
  // missed-step marker (NaN, ADR-0044), step it otherwise, then check its
  // diagnostics for an event. Shared by advance() and replay() so a
  // replayed roll rolls at the recorded step and a replayed pause pauses.
  void step_agent(Agent& a, const DriveInput& input);
  // Resolve one agent's command for the step about to be computed: the
  // policy callable, or the mailbox with its timeout policy (ADR-0044). May
  // set the agent DNF (kDnf on a miss); returns the input to log and apply,
  // NaN-tagged for a frozen step.
  DriveInput resolve_command(Agent& a, double t);
  // A timeout-DNF, shared by the live path and replay so both freeze the
  // agent at the same step with the same cause.
  void dnf_timeout(Agent& a);
  // The contact pass (ADR-0043), after every agent has moved: one impulse
  // per touching footprinted pair, pairs in ascending index order, one
  // pass. Shared by advance() and replay() for the same reason step_agent
  // is. With no footprints declared this touches nothing, which is what
  // keeps every pre-contact trajectory bit-identical.
  void resolve_contacts();
  // Validation mode only; reads no clock in deterministic mode.
  void pace();

  SimulationConfig config_;
  std::vector<Agent> agents_;
  // Preallocated in add_agent so that advance() does no allocation of its own.
  std::vector<DriveInput> pending_inputs_;
  // Refilled by every contact pass. Grows on the first step whose contact
  // count exceeds anything seen before and keeps its capacity after, so the
  // steady state allocates nothing.
  std::vector<ContactEvent> contacts_;
  std::vector<DriveInput> input_log_;
  std::uint64_t steps_ = 0;
  bool logging_inputs_ = false;

  // Validation mode only: when the run started on the wall clock, and how
  // much simulation time had elapsed then. Unused, and untouched, in
  // deterministic mode.
  bool pacing_started_ = false;
  std::chrono::steady_clock::time_point pacing_origin_{};
  double pacing_origin_time_ = 0.0;
};

}  // namespace sim
}  // namespace slipx

#endif  // SLIPX_SIM_SIMULATION_HPP
