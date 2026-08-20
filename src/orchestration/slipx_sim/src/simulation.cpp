// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/sim/simulation.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include "slipx/sim/build_info.hpp"

namespace slipx {
namespace sim {
namespace {

// Digest of a parameter set, for the manifest. Fixed field order, and it
// covers every field: two runs whose cars differ in any parameter must be
// distinguishable, including in the ones a given tier ignores, because the
// tier can change without the file changing. When VehicleParams grows a
// field, it is added here in the same change; the tyre blocks and c_kappa
// were once missed by exactly that omission.
void update_tyre(TrajectoryHash& h, const TyreCoefficients& t) {
  h.update(t.mu_y0);
  h.update(t.mu_x0);
  h.update(t.k_mu);
  h.update(t.relax_length);
  h.update(t.shape_c);
  h.update(t.curvature_e);
}

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
  update_tyre(h, p.tyre_front);
  update_tyre(h, p.tyre_rear);
  h.update(p.c_kappa);
  h.update(p.accel_max);
  h.update(p.decel_max);
  h.update(p.v_max);
  h.update_u64(static_cast<std::uint64_t>(p.layout));
  h.update_u64(static_cast<std::uint64_t>(p.differential));
  h.update(p.lsd_preload);
  h.update(p.torque_stall);
  h.update(p.omega_free);
  h.update(p.torque_per_amp);
  h.update(p.drive_efficiency);
  h.update(p.current_max);
  h.update(p.regen_current_max);
  h.update(p.pack_nominal_v);
  h.update(p.pack_v_full);
  h.update(p.pack_v_empty);
  h.update(p.pack_capacity_ah);
  h.update(p.pack_internal_resistance);
  h.update(p.steer_max);
  h.update(p.steer_rate_max);
  h.update(p.steer_bandwidth);
  h.update(p.steer_damping);
  h.update(p.drag_coeff);
  h.update(p.roll_resist);
  h.update(p.v_eps);
  h.update_u64(static_cast<std::uint64_t>(p.provenance));
  return h.hex();
}

// The rollover signal (ADR-0042): both wheels of one side at zero vertical
// load in the step's diagnostics. The loads come from the core's clamped
// load pass, which writes a literal zero for a lifted wheel, so the
// comparison is exact; <= only guards against a representation that cannot
// occur. Below L2 the entries are NaN, every comparison is false, and tiers
// without load transfer never roll.
//
// A single wheel at zero is deliberately not the signal: that is
// three-wheeling, which is routine near the limit at whichever axle lifts
// first. A car carried entirely on one side's contact patches is the static
// rollover condition proper.
std::optional<DnfCause> rollover_signal(const StepDiagnostics& d) {
  if (d.fz[kFrontLeft] <= 0.0 && d.fz[kRearLeft] <= 0.0) {
    return DnfCause::kRolloverLeft;
  }
  if (d.fz[kFrontRight] <= 0.0 && d.fz[kRearRight] <= 0.0) {
    return DnfCause::kRolloverRight;
  }
  return std::nullopt;
}

// A DNF'd car is a stationary obstacle, and its recorded state has to say
// so: position and yaw keep the value the event found them at, and the
// motion states are zeroed, because a constant position alongside a nonzero
// velocity is a recording that contradicts itself. The kinetic energy the
// car had leaves the record here; that is the roll and slide this model
// does not simulate (ADR-0042).
void freeze(VehicleState& s) {
  s.vel_body = Vec3{};
  s.rates = Vec3{};
  for (double& w : s.omega_w) w = 0.0;
  s.steer_rate = 0.0;
}

// The missed-step marker the input log carries for a frozen or timeout-DNF'd
// step (ADR-0044). In-band NaN, and sound only because every command door
// refuses NaN: the mailbox at post, the policy path below. Replay answers a
// marked slot by applying that agent's own timeout policy.
DriveInput miss_marker() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  return DriveInput{nan, nan};
}

bool is_miss(const DriveInput& input) {
  return std::isnan(input.steer_cmd) || std::isnan(input.accel_cmd);
}

}  // namespace

Simulation::Simulation(SimulationConfig config) : config_(config) {
  if (!(config_.dt > 0.0)) {
    throw std::invalid_argument("slipx_sim: dt must be positive [s]");
  }
  if (config_.hash_stride == 0) {
    throw std::invalid_argument("slipx_sim: hash_stride must be at least 1");
  }
  // Written as negations so a NaN, which compares false against everything,
  // is refused rather than smuggled into every collision of the run.
  if (!(config_.contact.restitution >= 0.0 &&
        config_.contact.restitution <= 1.0)) {
    throw std::invalid_argument(
        "slipx_sim: contact.restitution must be in [0, 1]");
  }
  if (!(config_.contact.friction >= 0.0)) {
    throw std::invalid_argument(
        "slipx_sim: contact.friction must not be negative");
  }
  if (!(config_.contact.restitution_min_speed >= 0.0)) {
    throw std::invalid_argument(
        "slipx_sim: contact.restitution_min_speed must not be negative "
        "[m/s]");
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
  if (spec.policy && spec.mailbox) {
    throw std::invalid_argument(
        "slipx_sim: an agent has one command source; set a policy or a "
        "mailbox, not both");
  }
  agent.policy = std::move(spec.policy);
  agent.mailbox = std::move(spec.mailbox);
  agent.timeout_policy = spec.timeout_policy;
  agent.seed = derive_seed(config_.master_seed, index);
  agent.rng = Rng(agent.seed);

  // The collision footprint (ADR-0043). Both dimensions or neither: one
  // without the other is not a smaller footprint, it is a mistake, and it
  // is named rather than defaulted (the same rule the loader lives by).
  if (!(spec.footprint_length >= 0.0) || !(spec.footprint_width >= 0.0)) {
    throw std::invalid_argument(
        "slipx_sim: footprint_length and footprint_width must not be "
        "negative [m]");
  }
  if ((spec.footprint_length > 0.0) != (spec.footprint_width > 0.0)) {
    throw std::invalid_argument(
        "slipx_sim: a collision footprint needs both footprint_length and "
        "footprint_width; set both [m] or neither (no footprint)");
  }
  agent.half_length = 0.5 * spec.footprint_length;
  agent.half_width = 0.5 * spec.footprint_width;
  // The footprint is the car's body, and a car's body is centred between
  // its axles, not on its CoG.
  agent.centre_offset = 0.5 * (spec.params.lf - spec.params.lr);
  agent.bounding_radius = std::sqrt(agent.half_length * agent.half_length +
                                    agent.half_width * agent.half_width);

  agents_.push_back(std::move(agent));
  // Sized once, here, so that advance() never allocates.
  pending_inputs_.resize(agents_.size());
  return index;
}

void Simulation::add_wall(const std::vector<Vec2>& points, bool closed) {
  // Walls are scenery, latched before the green flag (ADR-0055): a wall
  // that appears mid-run would make the manifest describe a race that was
  // two different races.
  if (steps_ != 0) {
    throw std::invalid_argument(
        "slipx_sim: walls are latched before the first advance; add them "
        "while the step count is zero (reset() keeps them)");
  }
  if (points.size() < 2) {
    throw std::invalid_argument(
        "slipx_sim: a wall polyline needs at least two points");
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y)) {
      throw std::invalid_argument(
          "slipx_sim: wall point " + std::to_string(i) +
          " is not finite [m]");
    }
  }
  if (closed && points.front().x == points.back().x &&
      points.front().y == points.back().y) {
    throw std::invalid_argument(
        "slipx_sim: a closed wall repeats its first point, which would add "
        "a zero-length closing segment; drop the duplicate");
  }

  const std::size_t spans = closed ? points.size() : points.size() - 1;
  for (std::size_t i = 0; i < spans; ++i) {
    const Vec2& a = points[i];
    const Vec2& b = points[(i + 1) % points.size()];
    if (a.x == b.x && a.y == b.y) {
      throw std::invalid_argument(
          "slipx_sim: wall points " + std::to_string(i) + " and " +
          std::to_string(i + 1) +
          " coincide, a zero-length segment; drop the duplicate");
    }
    WallSegment segment;
    segment.a = a;
    segment.b = b;
    segment.min_x = a.x < b.x ? a.x : b.x;
    segment.max_x = a.x < b.x ? b.x : a.x;
    segment.min_y = a.y < b.y ? a.y : b.y;
    segment.max_y = a.y < b.y ? b.y : a.y;
    wall_segments_.push_back(segment);
  }
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
  // has moved yet, so no policy can see another agent's future. A DNF'd
  // agent's source is never asked again; a neutral input holds its slot so
  // the log stays rectangular and a replay lines up agent for agent.
  for (std::size_t i = 0; i < agents_.size(); ++i) {
    pending_inputs_[i] = resolve_command(agents_[i], t);
  }

  if (logging_inputs_) {
    input_log_.insert(input_log_.end(), pending_inputs_.begin(),
                      pending_inputs_.end());
  }

  // Phase two: everybody still racing moves.
  for (std::size_t i = 0; i < agents_.size(); ++i) {
    step_agent(agents_[i], pending_inputs_[i]);
  }

  // Phase three: contact, between the steps rather than inside them.
  resolve_contacts();

  ++steps_;
  hash_states();

  pace();
}

// The contact pass (ADR-0043). One impulse per touching pair per step,
// pairs visited in ascending index order, one pass and no convergence loop:
// an impulse applied to pair (0,1) is visible to pair (0,2) in the same
// step, which is order-dependent but deterministic, and the order is the
// agent numbering the manifest records.
void Simulation::resolve_contacts() {
  contacts_.clear();
  wall_contacts_.clear();
  const auto body_of = [](const Agent& a) {
    ContactBody b;
    b.cog = a.state.pos.xy();
    b.yaw = a.state.yaw;
    const double c = std::cos(a.state.yaw);
    const double s = std::sin(a.state.yaw);
    b.velocity = Vec2{c * a.state.vel_body.x - s * a.state.vel_body.y,
                      s * a.state.vel_body.x + c * a.state.vel_body.y};
    b.yaw_rate = a.state.rates.z;
    // A DNF'd car is an immovable obstacle: zero inverse mass is the
    // standard spelling of "no impulse moves this" (ADR-0042, ADR-0043).
    if (!a.dnf) {
      b.inv_mass = 1.0 / a.model->params().mass;
      b.inv_izz = 1.0 / a.model->params().izz;
    }
    b.centre_offset = a.centre_offset;
    b.half_length = a.half_length;
    b.half_width = a.half_width;
    return b;
  };

  // Applies the world-frame deltas to the state. Only called for a touched,
  // running agent: a frozen agent's state must stay byte-identical, and
  // round-tripping its velocity through the world frame would not.
  const auto apply = [](Agent& a, const Vec2& dv, double dw, const Vec2& dp) {
    const double c = std::cos(a.state.yaw);
    const double s = std::sin(a.state.yaw);
    const double vx_w = c * a.state.vel_body.x - s * a.state.vel_body.y + dv.x;
    const double vy_w = s * a.state.vel_body.x + c * a.state.vel_body.y + dv.y;
    a.state.vel_body.x = c * vx_w + s * vy_w;
    a.state.vel_body.y = -s * vx_w + c * vy_w;
    a.state.rates.z += dw;
    a.state.pos.x += dp.x;
    a.state.pos.y += dp.y;
  };

  for (std::size_t i = 0; i < agents_.size(); ++i) {
    if (!agents_[i].has_footprint()) continue;
    for (std::size_t j = i + 1; j < agents_.size(); ++j) {
      if (!agents_[j].has_footprint()) continue;
      Agent& a = agents_[i];
      Agent& b = agents_[j];
      if (a.dnf && b.dnf) continue;   // two frozen cars have nothing to say

      const ContactBody body_a = body_of(a);
      const ContactBody body_b = body_of(b);
      const ContactGeometry geometry = rectangle_contact(body_a, body_b);
      if (!geometry.touching) continue;

      const ContactImpulse impulse =
          resolve_contact(body_a, body_b, geometry, config_.contact);
      if (!a.dnf) {
        apply(a, impulse.delta_velocity_a, impulse.delta_yaw_rate_a,
              impulse.delta_position_a);
      }
      if (!b.dnf) {
        apply(b, impulse.delta_velocity_b, impulse.delta_yaw_rate_b,
              impulse.delta_position_b);
      }

      ContactEvent event;
      event.step = steps_ + 1;
      event.a = static_cast<std::uint32_t>(i);
      event.b = static_cast<std::uint32_t>(j);
      event.point = geometry.point;
      event.normal = geometry.normal;
      event.jn = impulse.jn;
      event.approach_a = impulse.approach_a;
      event.approach_b = impulse.approach_b;
      contacts_.push_back(event);
    }
  }

  // The wall pass (ADR-0055), after the pair pass so a car shoved into a
  // wall by a pair impulse is pushed back out in the same step. Ascending
  // (agent, segment) order, sequential like the pairs: within one agent a
  // later segment sees the state an earlier one left. A DNF'd car is
  // skipped outright: an immovable car against an immovable wall has
  // nothing to exchange, and its frozen state must stay byte-identical.
  if (wall_segments_.empty()) return;
  for (std::size_t i = 0; i < agents_.size(); ++i) {
    Agent& a = agents_[i];
    if (!a.has_footprint() || a.dnf) continue;

    // The reject circle: the footprint's bounding radius, doubled, because
    // a resolution against one segment can move the centre by up to one
    // depth (itself bounded by the radius) before a later segment of the
    // same agent is tested against the centre computed here.
    const double c_yaw = std::cos(a.state.yaw);
    const double s_yaw = std::sin(a.state.yaw);
    const double cx = a.state.pos.x + c_yaw * a.centre_offset;
    const double cy = a.state.pos.y + s_yaw * a.centre_offset;
    const double reach = 2.0 * a.bounding_radius;

    for (std::size_t s = 0; s < wall_segments_.size(); ++s) {
      const WallSegment& segment = wall_segments_[s];
      if (cx + reach < segment.min_x || cx - reach > segment.max_x ||
          cy + reach < segment.min_y || cy - reach > segment.max_y) {
        continue;
      }

      const ContactBody body = body_of(a);
      const ContactGeometry geometry =
          segment_contact(segment.a, segment.b, body);
      if (!geometry.touching) continue;

      // The wall as a contact body: immovable, at rest. Its CoG is only an
      // impulse arm, and a zero inverse inertia makes the arm irrelevant;
      // the midpoint is recorded for the event's sake.
      ContactBody wall;
      wall.cog = (segment.a + segment.b) * 0.5;
      const ContactImpulse impulse =
          resolve_contact(wall, body, geometry, config_.contact);
      apply(a, impulse.delta_velocity_b, impulse.delta_yaw_rate_b,
            impulse.delta_position_b);

      WallContactEvent event;
      event.step = steps_ + 1;
      event.agent = static_cast<std::uint32_t>(i);
      event.segment = static_cast<std::uint32_t>(s);
      event.point = geometry.point;
      event.normal = geometry.normal;
      event.jn = impulse.jn;
      event.approach = impulse.approach_b;
      wall_contacts_.push_back(event);
    }
  }
}

// Resolve one agent's command for the step about to be computed: the
// synchronous policy, or the mailbox with its miss answers (ADR-0044).
DriveInput Simulation::resolve_command(Agent& a, double t) {
  if (a.dnf) return DriveInput{};

  if (a.mailbox) {
    const bool wait_forever = a.timeout_policy == TimeoutPolicy::kWait;
    const std::optional<DriveInput> command =
        a.mailbox->take(steps_, wait_forever, config_.barrier_timeout);
    if (command) return *command;
    // A miss. kWait cannot reach here: its take waits until it can answer.
    switch (a.timeout_policy) {
      case TimeoutPolicy::kFreeze:
        return miss_marker();
      case TimeoutPolicy::kCoast:
        return DriveInput{};
      case TimeoutPolicy::kDnf:
        dnf_timeout(a);
        return miss_marker();   // the onset marker replay re-reads
      case TimeoutPolicy::kWait:
        break;
    }
    return DriveInput{};
  }

  if (a.policy) {
    const DriveInput input = a.policy(a.state, t, a.rng);
    if (is_miss(input)) {
      // Refused loudly rather than integrated: a NaN command would poison
      // every hash downstream, and NaN in the log is reserved as the
      // missed-step marker (ADR-0044).
      throw std::invalid_argument(
          "slipx_sim: a policy returned a NaN command");
    }
    return input;
  }
  return DriveInput{};
}

void Simulation::dnf_timeout(Agent& a) {
  DnfEvent event;
  event.step = steps_ + 1;
  event.time = static_cast<double>(event.step) * config_.dt;
  event.cause = DnfCause::kTimeout;
  a.dnf = event;
  freeze(a.state);
}

void Simulation::step_agent(Agent& a, const DriveInput& input) {
  if (a.dnf) return;        // frozen: the state no longer evolves
  if (is_miss(input)) return;   // a missed step: paused, not stepped
  a.model->step(a.state, input, config_.dt, &a.diagnostics);

  if (const std::optional<DnfCause> cause = rollover_signal(a.diagnostics)) {
    DnfEvent event;
    // steps_ has not been incremented for this advance yet; the event names
    // the step count as the caller will see it, i.e. the first step at which
    // the state is frozen.
    event.step = steps_ + 1;
    event.time = static_cast<double>(event.step) * config_.dt;
    event.cause = *cause;
    a.dnf = event;
    freeze(a.state);
  }
}

// Soft real time, and only in validation mode.
//
// Soft, and the word is doing work: this sleeps when the simulation is ahead
// of the clock and does not try to catch up when it is behind, because
// catching up means taking steps faster than real time, which is the one
// thing a latency test must not do. A run that cannot keep up is a result,
// not an error, and the honest response is to fall behind visibly rather than
// to compress the timeline and report a stack meeting deadlines it missed.
//
// Nothing here runs in deterministic mode. Not "runs and does nothing": the
// clock is not read at all, so there is no path by which a scheduling
// decision can reach a trajectory.
void Simulation::pace() {
  if (config_.mode != RunMode::kValidation) return;

  const double factor = config_.real_time_factor > 0.0
                            ? config_.real_time_factor
                            : 1.0;

  if (!pacing_started_) {
    pacing_started_ = true;
    pacing_origin_ = std::chrono::steady_clock::now();
    pacing_origin_time_ = time();
    return;
  }

  const double elapsed_sim = (time() - pacing_origin_time_) / factor;
  const auto target = pacing_origin_ +
                      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double>(elapsed_sim));
  const auto now = std::chrono::steady_clock::now();
  if (now < target) std::this_thread::sleep_until(target);
}

SimulationSnapshot Simulation::snapshot() const {
  SimulationSnapshot out;
  out.steps = steps_;
  out.input_log_entries = input_log_.size();
  out.agents.reserve(agents_.size());

  for (const Agent& a : agents_) {
    AgentSnapshot one;
    one.state = a.state;
    one.diagnostics = a.diagnostics;
    one.rng = a.rng.save();
    one.hash_state = a.hash.value();
    one.dnf = a.dnf;
    out.agents.push_back(one);
  }
  return out;
}

void Simulation::restore(const SimulationSnapshot& snapshot) {
  if (snapshot.agents.size() != agents_.size()) {
    throw std::invalid_argument(
        "slipx_sim: this snapshot holds " +
        std::to_string(snapshot.agents.size()) +
        " agents and the simulation has " + std::to_string(agents_.size()) +
        ". A snapshot belongs to the simulation that produced it; restoring "
        "one into a differently configured run would produce a plausible "
        "trajectory from a car that was never there.");
  }

  for (std::size_t i = 0; i < agents_.size(); ++i) {
    Agent& a = agents_[i];
    const AgentSnapshot& one = snapshot.agents[i];
    a.state = one.state;
    a.diagnostics = one.diagnostics;
    a.rng.restore(one.rng);
    a.hash.restore(one.hash_state);
    a.dnf = one.dnf;
  }

  steps_ = snapshot.steps;

  // The log is truncated rather than left alone, so that resuming and running
  // on produces the log the uninterrupted run would have written.
  if (snapshot.input_log_entries <= input_log_.size()) {
    input_log_.resize(snapshot.input_log_entries);
  }

  // Pacing restarts from here. Restoring a snapshot in validation mode and
  // expecting the clock to carry on from where it was would make the
  // simulation sprint to catch up with a wall time it never spent running.
  pacing_started_ = false;
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
    a.dnf.reset();
  }
  steps_ = 0;
  input_log_.clear();
  contacts_.clear();
  wall_contacts_.clear();
  pacing_started_ = false;
}

void Simulation::replay(const std::vector<DriveInput>& log) {
  // replay(sim.input_log()) is the natural call and hands this function a
  // reference to the member that reset() below is about to clear; without
  // this copy the run would quietly replay an empty log.
  if (&log == &input_log_) {
    const std::vector<DriveInput> copy = log;
    replay(copy);
    return;
  }
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
      const DriveInput& input = log[step * agents_.size() + i];
      // A NaN-tagged slot is the missed-step marker (ADR-0044): replay
      // answers it with the agent's own timeout policy, which reproduces
      // the recorded pause or the timeout-DNF at the recorded step. It can
      // only appear for an agent configured to write it, and a log that
      // says otherwise was recorded from a different scenario.
      if (is_miss(input)) {
        if (!a.mailbox || (a.timeout_policy != TimeoutPolicy::kFreeze &&
                           a.timeout_policy != TimeoutPolicy::kDnf)) {
          throw std::invalid_argument(
              "slipx_sim: the input log marks a missed step for an agent "
              "whose command source cannot miss; the log was recorded from "
              "a different scenario");
        }
        if (a.timeout_policy == TimeoutPolicy::kDnf && !a.dnf) {
          dnf_timeout(a);
        }
      }
      // The same per-agent path advance() takes, so a replayed roll rolls at
      // the recorded step and the frozen tail of the log (neutral inputs) is
      // skipped exactly as it was skipped when recorded.
      step_agent(a, input);
    }
    resolve_contacts();
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

bool Simulation::agent_running(std::size_t i) const {
  check_index(i);
  return !agents_[i].dnf.has_value();
}

const std::optional<DnfEvent>& Simulation::dnf(std::size_t i) const {
  check_index(i);
  return agents_[i].dnf;
}

bool Simulation::has_footprint(std::size_t i) const {
  check_index(i);
  return agents_[i].has_footprint();
}

double Simulation::footprint_half_length(std::size_t i) const {
  check_index(i);
  return agents_[i].half_length;
}

double Simulation::footprint_half_width(std::size_t i) const {
  check_index(i);
  return agents_[i].half_width;
}

double Simulation::footprint_centre_offset(std::size_t i) const {
  check_index(i);
  return agents_[i].centre_offset;
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
  m.run_mode = to_string(config_.mode);
  m.contact_restitution = config_.contact.restitution;
  m.contact_friction = config_.contact.friction;
  m.contact_restitution_min_speed = config_.contact.restitution_min_speed;
  m.wall_segments = wall_segments_.size();
  if (!wall_segments_.empty()) {
    // Coordinates in segment order (ADR-0055): two runs against different
    // walls were different races, and this is how they are told apart.
    TrajectoryHash walls;
    for (const WallSegment& segment : wall_segments_) {
      walls.update(segment.a.x);
      walls.update(segment.a.y);
      walls.update(segment.b.x);
      walls.update(segment.b.y);
    }
    m.walls_digest = walls.hex();
  }

  m.agents.reserve(agents_.size());
  m.agent_trajectory_hashes.reserve(agents_.size());
  for (const Agent& a : agents_) {
    AgentManifest am;
    am.name = a.name;
    am.tier = to_string(a.model->tier());
    am.params_digest = params_digest(a.model->params());
    am.seed = a.seed;
    am.footprint_length = 2.0 * a.half_length;
    am.footprint_width = 2.0 * a.half_width;
    if (a.mailbox) {
      am.command_source = "mailbox";
      am.timeout_policy = to_string(a.timeout_policy);
      if (a.timeout_policy != TimeoutPolicy::kWait) {
        m.timing_dependent_commands = true;
      }
    } else {
      am.command_source = a.policy ? "policy" : "coast";
    }
    if (a.dnf) {
      am.status = "dnf";
      am.dnf_cause = to_string(a.dnf->cause);
      am.dnf_step = a.dnf->step;
    }
    m.agents.push_back(std::move(am));
    m.agent_trajectory_hashes.push_back(a.hash.hex());
  }
  m.trajectory_hash = trajectory_hash();
  return m;
}

}  // namespace sim
}  // namespace slipx
