// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The sensor rig: per-agent sensors running against a simulation they can
// only observe (ADR-0047).
//
// The rig holds a const reference to the simulation, so an agent with a
// full sensor suite drives bit for bit the trajectory of its bare twin;
// that is a property of the type system here, not of anybody's care.
// Sensors therefore stay out of the manifest and the configuration digest,
// which name what changes a trajectory. A policy that steers on scans makes
// the trajectory depend on them the way it depends on any policy internals,
// and is replayed the same way: from the input log, with no rig.
//
// The world arrives as a function from an agent index, a pose and a bearing
// to a hit. The index is there because a car must not see itself: only the
// caller knows which box in its overlay belongs to which agent, so only the
// caller can apply the self-skip. The rig includes sensor headers and never
// scene headers (ADR-0037); what answers the ray is the caller's business.
//
// Timing, in one paragraph. Each sensor samples at (k + phase) / rate,
// computed from k so the schedule cannot drift. An instant between step
// boundaries is served by the first step at or after it and stamped with
// its exact scheduled instant, so the truth is at most one step late and
// says so here rather than pretending to interpolate diagnostics. The
// LiDAR's emitter pose IS interpolated, per ray, from the pose history the
// rig records at each collect(), which is what makes motion distortion
// emerge; other agents are seen at step resolution because their poses
// between steps are not defined by the simulation. A sample becomes visible
// only at its instant plus a constant latency plus a uniform jitter drawn
// once per message, the LiDAR's own semantics applied to the IMU and the
// encoder, whose models deliberately do not model transport.

#ifndef SLIPX_SIM_SENSOR_RIG_HPP
#define SLIPX_SIM_SENSOR_RIG_HPP

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "slipx/sense/encoder.hpp"
#include "slipx/sense/imu.hpp"
#include "slipx/sense/lidar.hpp"
#include "slipx/sense/rng.hpp"
#include "slipx/sim/simulation.hpp"

namespace slipx {
namespace sim {

// The world as the rig's sensors can see it. `agent` is the asking car.
using WorldFunction = std::function<sense::Hit(
    std::size_t agent, const sense::Pose& origin, double bearing)>;

// One LiDAR instance. Rate, range, noise, dropouts and latency all live in
// the spec (the model owns them); the rig adds only the phase, the fraction
// of the period before the first scan starts.
struct LidarSensor {
  std::string name;
  sense::LidarSpec spec;
  double phase = 0.0;  //                                                 [-]
};

// One IMU instance. The model owns the error behaviour; the schedule and
// the transport are the rig's, because a real unit's driver owns them too.
struct ImuSensor {
  std::string name;
  sense::ImuSpec spec;
  double rate_hz = 200.0;         //                                     [Hz]
  double phase = 0.0;             // fraction of the period               [-]
  double latency_s = 0.0;         //                                      [s]
  double latency_jitter_s = 0.0;  // uniform, at most latency_s           [s]
};

// One wheel-encoder odometry instance, scheduled and delivered like the IMU.
struct EncoderSensor {
  std::string name;
  sense::EncoderSpec spec;
  double rate_hz = 100.0;         //                                     [Hz]
  double phase = 0.0;             //                                      [-]
  double latency_s = 0.0;         //                                      [s]
  double latency_jitter_s = 0.0;  //                                      [s]
};

// Everything one agent carries. Empty vectors are the cheap opponent.
struct AgentSensors {
  std::vector<LidarSensor> lidars;
  std::vector<ImuSensor> imus;
  std::vector<EncoderSensor> encoders;
};

// A delivered IMU or odometry message: the sample, which carries the instant
// it measured, plus the instant it became visible. A scan carries both times
// itself (sense::Scan::start_time and stamp_time), so it needs no wrapper.
struct ImuReading {
  sense::ImuSample sample;
  double stamp_time = 0.0;  //                                            [s]
};

struct OdometryReading {
  sense::EncoderSample sample;
  double stamp_time = 0.0;  //                                            [s]
};

class SensorRig {
 public:
  // `seed` is the rig's own master seed; every sensor instance draws from
  // derive_seed(derive_seed(seed, agent), instance) with instances numbered
  // in declaration order, so one agent's sensors cannot change another's
  // observations. It is deliberately a separate seed from the simulation's:
  // the rig never touches the simulation's streams.
  SensorRig(const Simulation& sim, WorldFunction world, std::uint64_t seed);

  // Attach an agent's sensors. Before the first collect() only, because a
  // sensor appearing mid-run would need a schedule with a start time, and
  // no file format carries one. Throws std::invalid_argument for a bad
  // agent index, a duplicate name on the same agent, a rate that is not
  // positive, a phase outside [0, 1), a negative latency, or a jitter
  // exceeding the latency (nothing may be stamped before it was measured);
  // the sensor models validate their own specs and their exceptions pass
  // through. Throws std::logic_error after the first collect().
  void attach(std::size_t agent, const AgentSensors& sensors);

  // Read everything that became due since the last call, up to the
  // simulation's current time, and record the current poses for the scan
  // interpolation. Call after every advance(): the rig serves missed
  // intervals from the poses it saw, so collecting rarely degrades the
  // interpolation rather than failing, and that cost is stated here.
  // Throws std::logic_error if simulation time moved backwards (a reset or
  // restore happened under the rig; build a new rig and replay instead).
  void collect();

  // Deliveries since the last take, oldest first, drained by the call.
  // Throws std::invalid_argument for an index or a name no sensor has.
  std::vector<sense::Scan> take_scans(std::size_t agent,
                                      const std::string& name);
  std::vector<ImuReading> take_imu(std::size_t agent, const std::string& name);
  std::vector<OdometryReading> take_odometry(std::size_t agent,
                                             const std::string& name);

  // The newest delivered message, kept across takes; empty until the first
  // delivery. This is the "what does the car know right now" accessor a
  // policy wants, where take_* is the "every message once" accessor a
  // bridge wants.
  const std::optional<sense::Scan>& latest_scan(std::size_t agent,
                                                const std::string& name) const;
  const std::optional<ImuReading>& latest_imu(std::size_t agent,
                                              const std::string& name) const;
  const std::optional<OdometryReading>& latest_odometry(
      std::size_t agent, const std::string& name) const;

 private:
  struct PoseSample {
    double time = 0.0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  struct LidarInstance {
    std::string name;
    sense::Lidar model;
    double phase = 0.0;
    std::uint64_t next_scan = 0;  // index k of the next scan to cast
    sense::Rng rng;
    std::vector<sense::Scan> pending;    // cast, not yet visible
    std::vector<sense::Scan> delivered;  // visible, not yet taken
    std::optional<sense::Scan> latest;
  };

  struct ImuInstance {
    std::string name;
    sense::Imu model;
    double rate_hz = 0.0;
    double phase = 0.0;
    double latency_s = 0.0;
    double latency_jitter_s = 0.0;
    std::uint64_t next_sample = 0;
    sense::Rng rng;
    std::vector<ImuReading> pending;
    std::vector<ImuReading> delivered;
    std::optional<ImuReading> latest;
  };

  struct EncoderInstance {
    std::string name;
    sense::WheelOdometry model;
    double rate_hz = 0.0;
    double phase = 0.0;
    double latency_s = 0.0;
    double latency_jitter_s = 0.0;
    std::uint64_t next_sample = 0;
    sense::Rng rng;
    std::vector<OdometryReading> pending;
    std::vector<OdometryReading> delivered;
    std::optional<OdometryReading> latest;
  };

  struct AgentRig {
    std::vector<LidarInstance> lidars;
    std::vector<ImuInstance> imus;
    std::vector<EncoderInstance> encoders;
    // Pose history for the scan interpolation, oldest first, trimmed to the
    // longest attached revolution. Seeded with the pose at attach time.
    std::vector<PoseSample> poses;
    double history_span = 0.0;  // how far back the LiDARs need           [s]
  };

  void check_agent(std::size_t index) const;
  void record_pose(std::size_t index);
  sense::Pose pose_at(const AgentRig& rig, double time) const;
  void collect_agent(std::size_t index, double now);

  const Simulation* sim_;
  WorldFunction world_;
  std::uint64_t seed_ = 0;
  std::vector<AgentRig> agents_;
  double last_time_ = 0.0;
  bool collected_ = false;
};

}  // namespace sim
}  // namespace slipx

#endif  // SLIPX_SIM_SENSOR_RIG_HPP
