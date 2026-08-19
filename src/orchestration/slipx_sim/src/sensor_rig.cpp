// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/sim/sensor_rig.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace slipx {
namespace sim {
namespace {

constexpr double kTau = 6.28318530717958647692;  // 2 pi

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument("slipx_sim sensor rig: " + message);
  }
}

// The shared schedule and transport checks. The sensor models validate
// their own specs; these are the fields the rig owns.
void check_schedule(const std::string& name, double rate_hz, double phase,
                    double latency_s, double latency_jitter_s) {
  require(!name.empty(), "a sensor needs a name; it becomes the topic name");
  require(std::isfinite(rate_hz) && rate_hz > 0.0,
          name + ": rate_hz must be positive [Hz]");
  require(std::isfinite(phase) && phase >= 0.0 && phase < 1.0,
          name + ": phase must be in [0, 1) [-]");
  require(std::isfinite(latency_s) && latency_s >= 0.0,
          name + ": latency_s must not be negative [s]");
  require(std::isfinite(latency_jitter_s) && latency_jitter_s >= 0.0,
          name + ": latency_jitter_s must not be negative [s]");
  // The LiDAR model enforces the same bound on its own latency fields, for
  // the same reason: jitter is symmetric about the constant, so a wider
  // jitter would stamp a message before the instant it measured.
  require(latency_jitter_s <= latency_s,
          name + ": latency_jitter_s must not exceed latency_s, or a sample "
                 "could be stamped before it was measured [s]");
}

// Uniform transport jitter, drawn once per message (the LiDAR's semantics,
// applied by the rig to the sensors whose models do not own transport).
double jitter_draw(double width, sense::Rng& rng) {
  return width > 0.0 ? rng.uniform(-width, width) : 0.0;
}

}  // namespace

SensorRig::SensorRig(const Simulation& sim, WorldFunction world,
                     std::uint64_t seed)
    : sim_(&sim),
      world_(std::move(world)),
      seed_(seed),
      agents_(sim.agent_count()),
      last_time_(sim.time()) {}

void SensorRig::check_agent(std::size_t index) const {
  if (index >= agents_.size()) {
    throw std::invalid_argument(
        "slipx_sim sensor rig: agent index " + std::to_string(index) +
        " is out of range for " + std::to_string(agents_.size()) + " agents");
  }
}

void SensorRig::attach(std::size_t agent, const AgentSensors& sensors) {
  check_agent(agent);
  if (collected_) {
    throw std::logic_error(
        "slipx_sim sensor rig: attach after the first collect(); a sensor "
        "appearing mid-run would need a start time no configuration carries");
  }

  AgentRig& rig = agents_[agent];

  // Names are checked against everything already attached as well as within
  // this call: they become topic names, and two sensors publishing to one
  // topic is a mistake, not a feature.
  std::vector<std::string> names;
  for (const LidarInstance& lidar : rig.lidars) names.push_back(lidar.name);
  for (const ImuInstance& imu : rig.imus) names.push_back(imu.name);
  for (const EncoderInstance& enc : rig.encoders) names.push_back(enc.name);
  const auto claim = [&names](const std::string& name) {
    if (std::find(names.begin(), names.end(), name) != names.end()) {
      throw std::invalid_argument(
          "slipx_sim sensor rig: duplicate sensor name '" + name +
          "' on one agent; names become topic names and must be unique");
    }
    names.push_back(name);
  };

  // Instance ordinals continue across calls, so each sensor's stream is
  // derive_seed(derive_seed(seed, agent), ordinal) in declaration order and
  // one agent's sensors cannot change another agent's draws.
  std::uint64_t ordinal = static_cast<std::uint64_t>(
      rig.lidars.size() + rig.imus.size() + rig.encoders.size());
  const std::uint64_t agent_seed =
      sense::derive_seed(seed_, static_cast<std::uint64_t>(agent));
  const auto stream = [&agent_seed, &ordinal]() {
    return sense::Rng(sense::derive_seed(agent_seed, ordinal++));
  };

  // Built aside and committed only once every sensor has passed: an attach
  // that refuses one sensor must leave the rig exactly as it found it, or a
  // caller who catches the refusal is left with half an agent.
  std::vector<LidarInstance> new_lidars;
  std::vector<ImuInstance> new_imus;
  std::vector<EncoderInstance> new_encoders;
  double history_span = rig.history_span;

  for (const LidarSensor& sensor : sensors.lidars) {
    require(static_cast<bool>(world_),
            sensor.name + ": a LiDAR needs a world; construct the rig with a "
                          "WorldFunction that answers rays");
    check_schedule(sensor.name, sensor.spec.rate_hz, sensor.phase,
                   sensor.spec.latency_s, sensor.spec.latency_jitter_s);
    claim(sensor.name);
    LidarInstance instance{sensor.name, sense::Lidar(sensor.spec),
                           sensor.phase, 0, stream(), {}, {}, {}};
    new_lidars.push_back(std::move(instance));
    history_span = std::max(history_span, 1.0 / sensor.spec.rate_hz);
  }
  for (const ImuSensor& sensor : sensors.imus) {
    check_schedule(sensor.name, sensor.rate_hz, sensor.phase,
                   sensor.latency_s, sensor.latency_jitter_s);
    claim(sensor.name);
    ImuInstance instance{sensor.name,       sense::Imu(sensor.spec),
                         sensor.rate_hz,    sensor.phase,
                         sensor.latency_s,  sensor.latency_jitter_s,
                         0,                 stream(),
                         {},                {},
                         {}};
    new_imus.push_back(std::move(instance));
  }
  for (const EncoderSensor& sensor : sensors.encoders) {
    check_schedule(sensor.name, sensor.rate_hz, sensor.phase,
                   sensor.latency_s, sensor.latency_jitter_s);
    // Below L2 the model never writes wheel speeds, so the state keeps its
    // initial zeros and an encoder would report a moving car as stationary
    // forever. That is the silent zero this project refuses everywhere
    // (ADR-0006), arriving through a sensor instead of a sink, so it is
    // refused by name here rather than measured as a lie.
    require(static_cast<int>(sim_->model(agent).tier()) >=
                static_cast<int>(Tier::L2_DoubleTrack),
            sensor.name + ": wheel encoders need L2 or above; below L2 the "
                          "model does not compute wheel speeds, and an "
                          "encoder would report the car as stationary");
    claim(sensor.name);
    EncoderInstance instance{sensor.name,       sense::WheelOdometry(sensor.spec),
                             sensor.rate_hz,    sensor.phase,
                             sensor.latency_s,  sensor.latency_jitter_s,
                             0,                 stream(),
                             {},                {},
                             {}};
    new_encoders.push_back(std::move(instance));
  }

  for (LidarInstance& instance : new_lidars) {
    rig.lidars.push_back(std::move(instance));
  }
  for (ImuInstance& instance : new_imus) {
    rig.imus.push_back(std::move(instance));
  }
  for (EncoderInstance& instance : new_encoders) {
    rig.encoders.push_back(std::move(instance));
  }
  rig.history_span = history_span;

  // Seed the pose history with where the agent stands now, so a scan whose
  // window opens at attach time has a pose to interpolate from.
  if (rig.poses.empty()) record_pose(agent);
}

void SensorRig::record_pose(std::size_t index) {
  AgentRig& rig = agents_[index];
  const VehicleState& state = sim_->state(index);
  const double now = sim_->time();
  if (!rig.poses.empty() && rig.poses.back().time == now) return;
  rig.poses.push_back(PoseSample{now, state.pos.x, state.pos.y, state.yaw});

  // Trim what no scan can reach any more, keeping one entry beyond the span
  // so the oldest reachable instant still has a bracket.
  const double cutoff = now - rig.history_span;
  while (rig.poses.size() > 2 && rig.poses[1].time <= cutoff) {
    rig.poses.erase(rig.poses.begin());
  }
}

sense::Pose SensorRig::pose_at(const AgentRig& rig, double time) const {
  sense::Pose pose;
  const std::vector<PoseSample>& poses = rig.poses;
  if (poses.empty()) return pose;

  if (time <= poses.front().time) {
    const PoseSample& p = poses.front();
    return sense::Pose{p.x, p.y, p.yaw};
  }
  if (time >= poses.back().time) {
    const PoseSample& p = poses.back();
    return sense::Pose{p.x, p.y, p.yaw};
  }

  const auto after = std::lower_bound(
      poses.begin(), poses.end(), time,
      [](const PoseSample& p, double t) { return p.time < t; });
  const PoseSample& b = *after;
  const PoseSample& a = *(after - 1);
  const double span = b.time - a.time;
  const double u = span > 0.0 ? (time - a.time) / span : 0.0;

  pose.x = a.x + u * (b.x - a.x);
  pose.y = a.y + u * (b.y - a.y);
  // Shortest arc, because a yaw that wraps from just below pi to just above
  // minus pi has turned a few degrees, not nearly a full circle.
  pose.yaw = a.yaw + u * std::remainder(b.yaw - a.yaw, kTau);
  return pose;
}

void SensorRig::collect() {
  const double now = sim_->time();
  if (now < last_time_) {
    throw std::logic_error(
        "slipx_sim sensor rig: simulation time moved backwards; after a "
        "reset or restore, build a new rig and replay from the start");
  }
  last_time_ = now;
  collected_ = true;

  for (std::size_t i = 0; i < agents_.size(); ++i) {
    if (!agents_[i].poses.empty()) record_pose(i);
    collect_agent(i, now);
  }
}

void SensorRig::collect_agent(std::size_t index, double now) {
  AgentRig& rig = agents_[index];
  const bool running = sim_->agent_running(index);

  // New samples first, deliveries second, so a sample due now with zero
  // latency is visible from the same collect that took it.
  if (running) {
    for (LidarInstance& lidar : rig.lidars) {
      const double period = lidar.model.period();
      // A scan starting at t is cast once the whole revolution it spans has
      // been simulated, because its last ray needs a pose that exists.
      double start = (static_cast<double>(lidar.next_scan) + lidar.phase) *
                     period;
      while (start + period <= now) {
        const auto pose_fn = [this, &rig](double t) {
          return pose_at(rig, t);
        };
        const auto range_fn = [this, index](const sense::Pose& origin,
                                            double bearing) {
          return world_(index, origin, bearing);
        };
        lidar.pending.push_back(
            lidar.model.sample(start, pose_fn, range_fn, lidar.rng));
        ++lidar.next_scan;
        start = (static_cast<double>(lidar.next_scan) + lidar.phase) * period;
      }
    }

    for (ImuInstance& imu : rig.imus) {
      const double period = 1.0 / imu.rate_hz;
      double due = (static_cast<double>(imu.next_sample) + imu.phase) * period;
      while (due <= now) {
        // Truth at step resolution: the first step at or after the instant,
        // which is the step whose collect this is. The sample keeps its
        // exact scheduled time, so the schedule never drifts (ADR-0047).
        const StepDiagnostics& diagnostics = sim_->diagnostics(index);
        const VehicleState& state = sim_->state(index);
        ImuReading reading;
        reading.sample = imu.model.sample(due, period, diagnostics.ax,
                                          diagnostics.ay, state.yaw_rate(),
                                          imu.rng);
        reading.stamp_time =
            due + imu.latency_s + jitter_draw(imu.latency_jitter_s, imu.rng);
        imu.pending.push_back(reading);
        ++imu.next_sample;
        due = (static_cast<double>(imu.next_sample) + imu.phase) * period;
      }
    }

    for (EncoderInstance& encoder : rig.encoders) {
      const double period = 1.0 / encoder.rate_hz;
      double due =
          (static_cast<double>(encoder.next_sample) + encoder.phase) * period;
      while (due <= now) {
        OdometryReading reading;
        reading.sample = encoder.model.sample(due, period, sim_->state(index));
        reading.stamp_time =
            due + encoder.latency_s +
            jitter_draw(encoder.latency_jitter_s, encoder.rng);
        encoder.pending.push_back(reading);
        ++encoder.next_sample;
        due = (static_cast<double>(encoder.next_sample) + encoder.phase) *
              period;
      }
    }
  }

  // Deliveries: everything stamped at or before now becomes visible, in
  // stamp order. A DNF'd agent takes no new samples, but what its sensors
  // already measured still arrives; the transport does not know the car is
  // out (ADR-0047).
  for (LidarInstance& lidar : rig.lidars) {
    std::stable_sort(lidar.pending.begin(), lidar.pending.end(),
                     [](const sense::Scan& a, const sense::Scan& b) {
                       return a.stamp_time < b.stamp_time;
                     });
    auto first_late = lidar.pending.begin();
    while (first_late != lidar.pending.end() &&
           first_late->stamp_time <= now) {
      if (!lidar.latest || first_late->stamp_time >= lidar.latest->stamp_time) {
        lidar.latest = *first_late;
      }
      lidar.delivered.push_back(std::move(*first_late));
      ++first_late;
    }
    lidar.pending.erase(lidar.pending.begin(), first_late);
  }
  for (ImuInstance& imu : rig.imus) {
    std::stable_sort(imu.pending.begin(), imu.pending.end(),
                     [](const ImuReading& a, const ImuReading& b) {
                       return a.stamp_time < b.stamp_time;
                     });
    auto first_late = imu.pending.begin();
    while (first_late != imu.pending.end() && first_late->stamp_time <= now) {
      if (!imu.latest || first_late->stamp_time >= imu.latest->stamp_time) {
        imu.latest = *first_late;
      }
      imu.delivered.push_back(*first_late);
      ++first_late;
    }
    imu.pending.erase(imu.pending.begin(), first_late);
  }
  for (EncoderInstance& encoder : rig.encoders) {
    std::stable_sort(encoder.pending.begin(), encoder.pending.end(),
                     [](const OdometryReading& a, const OdometryReading& b) {
                       return a.stamp_time < b.stamp_time;
                     });
    auto first_late = encoder.pending.begin();
    while (first_late != encoder.pending.end() &&
           first_late->stamp_time <= now) {
      if (!encoder.latest ||
          first_late->stamp_time >= encoder.latest->stamp_time) {
        encoder.latest = *first_late;
      }
      encoder.delivered.push_back(*first_late);
      ++first_late;
    }
    encoder.pending.erase(encoder.pending.begin(), first_late);
  }
}

namespace {

[[noreturn]] void unknown_sensor(std::size_t agent, const std::string& name) {
  throw std::invalid_argument("slipx_sim sensor rig: agent " +
                              std::to_string(agent) + " has no sensor named '" +
                              name + "'");
}

}  // namespace

std::vector<sense::Scan> SensorRig::take_scans(std::size_t agent,
                                               const std::string& name) {
  check_agent(agent);
  for (LidarInstance& lidar : agents_[agent].lidars) {
    if (lidar.name == name) {
      std::vector<sense::Scan> out;
      out.swap(lidar.delivered);
      return out;
    }
  }
  unknown_sensor(agent, name);
}

std::vector<ImuReading> SensorRig::take_imu(std::size_t agent,
                                            const std::string& name) {
  check_agent(agent);
  for (ImuInstance& imu : agents_[agent].imus) {
    if (imu.name == name) {
      std::vector<ImuReading> out;
      out.swap(imu.delivered);
      return out;
    }
  }
  unknown_sensor(agent, name);
}

std::vector<OdometryReading> SensorRig::take_odometry(std::size_t agent,
                                                      const std::string& name) {
  check_agent(agent);
  for (EncoderInstance& encoder : agents_[agent].encoders) {
    if (encoder.name == name) {
      std::vector<OdometryReading> out;
      out.swap(encoder.delivered);
      return out;
    }
  }
  unknown_sensor(agent, name);
}

const std::optional<sense::Scan>& SensorRig::latest_scan(
    std::size_t agent, const std::string& name) const {
  check_agent(agent);
  for (const LidarInstance& lidar : agents_[agent].lidars) {
    if (lidar.name == name) return lidar.latest;
  }
  unknown_sensor(agent, name);
}

const std::optional<ImuReading>& SensorRig::latest_imu(
    std::size_t agent, const std::string& name) const {
  check_agent(agent);
  for (const ImuInstance& imu : agents_[agent].imus) {
    if (imu.name == name) return imu.latest;
  }
  unknown_sensor(agent, name);
}

const std::optional<OdometryReading>& SensorRig::latest_odometry(
    std::size_t agent, const std::string& name) const {
  check_agent(agent);
  for (const EncoderInstance& encoder : agents_[agent].encoders) {
    if (encoder.name == name) return encoder.latest;
  }
  unknown_sensor(agent, name);
}

}  // namespace sim
}  // namespace slipx
