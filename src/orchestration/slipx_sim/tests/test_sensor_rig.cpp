// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The sensor rig (ADR-0047), held to its own claims: observation never
// perturbs a trajectory, schedules are exact and phases hold, latency is a
// delivery time, the seed reaches every stream independently, motion
// distortion emerges from the pose history, and a DNF'd agent stops
// sensing. The world here is a synthetic wall, because the rig's contract
// is with a function, not with a track (ADR-0037).

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "slipx/sim/mailbox.hpp"
#include "slipx/sim/sensor_rig.hpp"
#include "slipx/sim/simulation.hpp"

namespace {

using slipx::DriveInput;
using slipx::Tier;
using slipx::VehicleState;
using slipx::sense::Hit;
using slipx::sense::Pose;
using slipx::sense::Scan;
using slipx::sim::AgentSensors;
using slipx::sim::EncoderSensor;
using slipx::sim::ImuReading;
using slipx::sim::ImuSensor;
using slipx::sim::LidarSensor;
using slipx::sim::OdometryReading;
using slipx::sim::SensorRig;
using slipx::sim::Simulation;
using slipx::sim::SimulationConfig;
using slipx::sim::WorldFunction;

// A wall at x = wall_x, infinite in y: the simplest world whose answer
// depends on both the emitter's position and its bearing, which is what the
// distortion test needs.
WorldFunction wall_world(double wall_x) {
  return [wall_x](std::size_t, const Pose& origin, double bearing) {
    Hit hit;
    const double along = std::cos(bearing);
    if (along > 1e-9 && origin.x < wall_x) {
      hit.hit = true;
      hit.range = (wall_x - origin.x) / along;
    }
    return hit;
  };
}

// An L1 car pointed at positive x. No footprint: contact is not this file's
// subject.
slipx::sim::AgentSpec car(double vx) {
  slipx::sim::AgentSpec spec;
  spec.tier = Tier::L1_Bicycle;
  spec.initial_state.vel_body.x = vx;
  spec.policy = [](const VehicleState&, double, slipx::sense::Rng&) {
    return DriveInput{};  // coast
  };
  return spec;
}

// An L2 car, for the encoder tests: below L2 the model does not compute
// wheel speeds and the rig refuses the sensor.
slipx::sim::AgentSpec l2_car(double vx) {
  slipx::sim::AgentSpec spec = car(vx);
  spec.tier = Tier::L2_DoubleTrack;
  return spec;
}

// A kinematic car holding a constant steer: its speed and yaw rate are
// exactly constant, so a ray's expected measurement is analytic and the
// interpolation tests can assert equality rather than plausibility.
slipx::sim::AgentSpec l0_car(double vx, double steer) {
  slipx::sim::AgentSpec spec;
  spec.tier = Tier::L0_Kinematic;
  spec.initial_state.vel_body.x = vx;
  spec.policy = [steer](const VehicleState&, double, slipx::sense::Rng&) {
    DriveInput input;
    input.steer_cmd = steer;
    return input;
  };
  return spec;
}

// A world whose answer encodes the bearing itself, so a measured range
// reveals the yaw the rig interpolated for that ray.
WorldFunction compass_world() {
  return [](std::size_t, const Pose&, double bearing) {
    Hit hit;
    hit.hit = true;
    hit.range = 10.0 + std::sin(bearing);
    return hit;
  };
}

// A noise-free LiDAR, so a ray's range is pure geometry and two scans can
// be compared exactly.
LidarSensor clean_lidar(const std::string& name, double rate_hz,
                        std::size_t rays) {
  LidarSensor sensor;
  sensor.name = name;
  sensor.spec.rate_hz = rate_hz;
  sensor.spec.rays = rays;
  sensor.spec.range_max = 30.0;
  sensor.spec.latency_s = 0.0;
  sensor.spec.latency_jitter_s = 0.0;
  sensor.spec.noise_base_m = 0.0;
  sensor.spec.noise_per_metre = 0.0;
  sensor.spec.dropout_probability = 0.0;
  return sensor;
}

// A noise-free IMU: a sample equals the truth it was fed, which is what the
// schedule tests need to see which step served the instant.
ImuSensor clean_imu(const std::string& name, double rate_hz, double phase) {
  ImuSensor sensor;
  sensor.name = name;
  sensor.rate_hz = rate_hz;
  sensor.phase = phase;
  sensor.spec.accel_noise_density = 0.0;
  sensor.spec.gyro_noise_density = 0.0;
  sensor.spec.accel_bias_walk = 0.0;
  sensor.spec.gyro_bias_walk = 0.0;
  return sensor;
}

TEST(SensorRig, ObservationNeverPerturbsTheTrajectory) {
  const auto run = [](bool sensed) {
    Simulation sim;
    sim.add_agent(l2_car(3.0));
    SensorRig rig(sim, wall_world(20.0), 7);
    if (sensed) {
      AgentSensors sensors;
      sensors.lidars.push_back(clean_lidar("scan", 40.0, 90));
      sensors.imus.push_back(clean_imu("imu", 200.0, 0.25));
      EncoderSensor odometry;
      odometry.name = "wheel_encoder";
      rig.attach(0, sensors);
      AgentSensors more;
      more.encoders.push_back(odometry);
      rig.attach(0, more);
    }
    for (int i = 0; i < 200; ++i) {
      sim.advance();
      rig.collect();
    }
    return sim.state(0);
  };

  const VehicleState bare = run(false);
  const VehicleState sensed = run(true);
  EXPECT_EQ(bare.pos.x, sensed.pos.x);
  EXPECT_EQ(bare.pos.y, sensed.pos.y);
  EXPECT_EQ(bare.yaw, sensed.yaw);
  EXPECT_EQ(bare.vel_body.x, sensed.vel_body.x);
}

TEST(SensorRig, SchedulesAreExactAndPhasesHold) {
  Simulation sim;
  sim.add_agent(l2_car(2.0));
  SensorRig rig(sim, {}, 1);

  AgentSensors sensors;
  sensors.imus.push_back(clean_imu("imu", 200.0, 0.25));
  EncoderSensor odometry;
  odometry.name = "wheel_encoder";
  odometry.rate_hz = 100.0;
  odometry.phase = 0.5;
  sensors.encoders.push_back(odometry);
  EncoderSensor step_rate;
  step_rate.name = "tick";
  step_rate.rate_hz = 1000.0;
  step_rate.phase = 0.0;
  sensors.encoders.push_back(step_rate);
  // An encoder whose instants fall BETWEEN step boundaries, because the
  // first mutation pass showed that boundary-aligned schedules cannot tell
  // a scheduled instant from the step that served it.
  EncoderSensor off_beat;
  off_beat.name = "off_beat";
  off_beat.rate_hz = 400.0;
  off_beat.phase = 0.25;
  sensors.encoders.push_back(off_beat);
  rig.attach(0, sensors);

  for (int i = 0; i < 100; ++i) {
    sim.advance();
    rig.collect();
  }

  // (k + 0.25) / 200 within 0.1 s: k = 0..19. The times are the scheduled
  // instants themselves, not the step boundaries that served them.
  const std::vector<ImuReading> imu = rig.take_imu(0, "imu");
  ASSERT_EQ(imu.size(), 20u);
  for (std::size_t k = 0; k < imu.size(); ++k) {
    EXPECT_EQ(imu[k].sample.time,
              (static_cast<double>(k) + 0.25) * (1.0 / 200.0))
        << "sample " << k;
    EXPECT_EQ(imu[k].stamp_time, imu[k].sample.time)
        << "zero latency delivers at the instant itself";
  }

  const std::vector<OdometryReading> odo = rig.take_odometry(0, "wheel_encoder");
  ASSERT_EQ(odo.size(), 10u);
  for (std::size_t k = 0; k < odo.size(); ++k) {
    EXPECT_EQ(odo[k].sample.time,
              (static_cast<double>(k) + 0.5) * (1.0 / 100.0))
        << "sample " << k;
  }

  // The boundary case of "at or after": a sensor at the physics rate with
  // zero phase has every instant exactly on a step boundary, including
  // t = 0 and t = now, and both belong to it. 0.1 s at 1 kHz is instants
  // 0 through 100 inclusive: one hundred and one, not one hundred.
  const std::vector<OdometryReading> tick = rig.take_odometry(0, "tick");
  EXPECT_EQ(tick.size(), 101u);

  // And the off-boundary case: the times are the scheduled instants, which
  // no step boundary equals, so a sample stamped with its serving step
  // would fail here where the aligned sensors cannot see it.
  const std::vector<OdometryReading> off = rig.take_odometry(0, "off_beat");
  ASSERT_EQ(off.size(), 40u);
  for (std::size_t k = 0; k < off.size(); ++k) {
    EXPECT_EQ(off[k].sample.time,
              (static_cast<double>(k) + 0.25) * (1.0 / 400.0))
        << "sample " << k;
  }

  // The odometry integrated real time: a car near 2 m/s for the 95 ms the
  // last sample covers has rolled about 19 cm, and an encoder advanced by
  // the wrong interval (the simulation step instead of its own period)
  // would be an order of magnitude out.
  EXPECT_NEAR(odo.back().sample.distance, 2.0 * 0.095, 0.05);
}

TEST(SensorRig, AnInstantBetweenStepsIsServedByTheFirstStepAtOrAfterIt) {
  Simulation sim;
  // Accelerating and turning, so every step's ax, ay and yaw rate all
  // differ and the serving step is identifiable from each field.
  slipx::sim::AgentSpec spec = car(1.0);
  spec.policy = [](const VehicleState&, double, slipx::sense::Rng&) {
    DriveInput input;
    input.accel_cmd = 2.0;
    input.steer_cmd = 0.2;
    return input;
  };
  sim.add_agent(spec);

  SensorRig rig(sim, {}, 1);
  AgentSensors sensors;
  // One instance between boundaries and one exactly on them, because "at or
  // after" has to hold at the boundary too, where an off-by-one serves the
  // next step's truth instead.
  sensors.imus.push_back(clean_imu("between", 200.0, 0.25));
  sensors.imus.push_back(clean_imu("aligned", 200.0, 0.0));
  rig.attach(0, sensors);

  // Record what the truth was at each step boundary, with the same time
  // arithmetic the rig sees.
  struct Truth {
    double time, ax, ay, yaw_rate;
  };
  std::vector<Truth> truth;
  for (int i = 0; i < 60; ++i) {
    sim.advance();
    rig.collect();
    truth.push_back(Truth{sim.time(), sim.diagnostics(0).ax,
                          sim.diagnostics(0).ay, sim.state(0).yaw_rate()});
  }

  for (const char* name : {"between", "aligned"}) {
    const std::vector<ImuReading> imu = rig.take_imu(0, name);
    ASSERT_FALSE(imu.empty()) << name;
    for (const ImuReading& reading : imu) {
      // The first recorded boundary at or after the scheduled instant is
      // the step that served it (ADR-0047).
      const Truth* expected = nullptr;
      for (const Truth& entry : truth) {
        if (entry.time >= reading.sample.time) {
          expected = &entry;
          break;
        }
      }
      ASSERT_NE(expected, nullptr) << "nothing served " << reading.sample.time;
      EXPECT_EQ(reading.sample.ax, expected->ax)
          << name << " at " << reading.sample.time;
      EXPECT_EQ(reading.sample.ay, expected->ay);
      EXPECT_EQ(reading.sample.yaw_rate, expected->yaw_rate);
    }
  }
}

TEST(SensorRig, LatencyDelaysDeliveryAndJitterStaysBounded) {
  Simulation sim;
  sim.add_agent(l2_car(2.0));
  SensorRig rig(sim, wall_world(10.0), 3);

  AgentSensors sensors;
  ImuSensor imu = clean_imu("imu", 100.0, 0.0);
  imu.latency_s = 0.003;
  imu.latency_jitter_s = 0.001;
  sensors.imus.push_back(imu);
  // A slow-transport LiDAR and a laggy encoder as well, because the
  // delivery gate is per sensor kind and each copy of it has to hold.
  LidarSensor lidar = clean_lidar("scan", 40.0, 36);
  lidar.spec.latency_s = 0.03;
  sensors.lidars.push_back(lidar);
  EncoderSensor odometry;
  odometry.name = "odom";
  odometry.latency_s = 0.003;
  odometry.latency_jitter_s = 0.001;
  sensors.encoders.push_back(odometry);
  rig.attach(0, sensors);

  std::vector<ImuReading> all;
  for (int i = 0; i < 120; ++i) {
    sim.advance();
    const double now = sim.time();
    rig.collect();
    for (const ImuReading& reading : rig.take_imu(0, "imu")) {
      // Nothing arrives from the future.
      EXPECT_LE(reading.stamp_time, now);
      all.push_back(reading);
    }
    for (const Scan& scan : rig.take_scans(0, "scan")) {
      EXPECT_LE(scan.stamp_time, now);
      EXPECT_GE(scan.stamp_time, scan.start_time + 0.03);
    }
    for (const OdometryReading& reading : rig.take_odometry(0, "odom")) {
      EXPECT_LE(reading.stamp_time, now);
      EXPECT_GE(reading.stamp_time, reading.sample.time + 0.002);
    }
  }

  ASSERT_FALSE(all.empty());
  int below_constant = 0;
  int above_constant = 0;
  for (const ImuReading& reading : all) {
    EXPECT_GE(reading.stamp_time, reading.sample.time + 0.002);
    EXPECT_LE(reading.stamp_time, reading.sample.time + 0.004);
    (reading.stamp_time - reading.sample.time < 0.003 ? below_constant
                                                      : above_constant)++;
  }
  // Jitter is a draw, and it is symmetric about the constant: over a
  // hundred samples both sides of it appear, so a jitter that only ever
  // delayed would be visible here.
  EXPECT_GT(below_constant, 0);
  EXPECT_GT(above_constant, 0);

  const auto& latest = rig.latest_imu(0, "imu");
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->stamp_time, all.back().stamp_time);
}

TEST(SensorRig, TheSeedReachesEveryStreamIndependently) {
  // A noisy IMU, so the stream actually shows in the samples.
  const auto noisy_imu = [](const std::string& name) {
    ImuSensor sensor;
    sensor.name = name;
    sensor.rate_hz = 100.0;
    return sensor;
  };

  const auto run = [&](std::uint64_t rig_seed, bool second_agent_sensed) {
    Simulation sim;
    sim.add_agent(car(2.0));
    sim.add_agent(car(2.0));
    SensorRig rig(sim, {}, rig_seed);
    AgentSensors sensors;
    sensors.imus.push_back(noisy_imu("imu"));
    rig.attach(0, sensors);
    if (second_agent_sensed) rig.attach(1, sensors);
    for (int i = 0; i < 50; ++i) {
      sim.advance();
      rig.collect();
    }
    return rig.take_imu(0, "imu");
  };

  const std::vector<ImuReading> one = run(11, false);
  const std::vector<ImuReading> two = run(11, false);
  ASSERT_EQ(one.size(), two.size());
  for (std::size_t k = 0; k < one.size(); ++k) {
    EXPECT_EQ(one[k].sample.ax, two[k].sample.ax) << "sample " << k;
    EXPECT_EQ(one[k].sample.yaw_rate, two[k].sample.yaw_rate);
    EXPECT_EQ(one[k].stamp_time, two[k].stamp_time);
  }

  // Sensors on another agent draw from another stream: agent 0 sees the
  // same numbers whether or not agent 1 is sensed.
  const std::vector<ImuReading> with_neighbour = run(11, true);
  ASSERT_EQ(one.size(), with_neighbour.size());
  for (std::size_t k = 0; k < one.size(); ++k) {
    EXPECT_EQ(one[k].sample.ax, with_neighbour[k].sample.ax) << "sample " << k;
  }

  // And the seed matters: a different rig seed is a different unit.
  const std::vector<ImuReading> reseeded = run(12, false);
  ASSERT_EQ(one.size(), reseeded.size());
  bool differs = false;
  for (std::size_t k = 0; k < one.size(); ++k) {
    if (one[k].sample.ax != reseeded[k].sample.ax) differs = true;
  }
  EXPECT_TRUE(differs);

  // Two agents with the same sensor are two units, not one unit heard
  // twice: their streams derive per agent, so their noise differs even
  // though their trajectories are identical.
  {
    Simulation sim;
    sim.add_agent(car(2.0));
    sim.add_agent(car(2.0));
    SensorRig rig(sim, {}, 11);
    AgentSensors sensors;
    sensors.imus.push_back(noisy_imu("imu"));
    rig.attach(0, sensors);
    rig.attach(1, sensors);
    for (int i = 0; i < 50; ++i) {
      sim.advance();
      rig.collect();
    }
    const std::vector<ImuReading> first = rig.take_imu(0, "imu");
    const std::vector<ImuReading> second = rig.take_imu(1, "imu");
    ASSERT_EQ(first.size(), second.size());
    bool agents_differ = false;
    for (std::size_t k = 0; k < first.size(); ++k) {
      if (first[k].sample.ax != second[k].sample.ax) agents_differ = true;
    }
    EXPECT_TRUE(agents_differ);
  }

  // And two identical sensors on one agent are two units as well: the
  // stream derives per instance within the agent.
  {
    Simulation sim;
    sim.add_agent(car(2.0));
    SensorRig rig(sim, {}, 11);
    AgentSensors sensors;
    sensors.imus.push_back(noisy_imu("imu_a"));
    sensors.imus.push_back(noisy_imu("imu_b"));
    rig.attach(0, sensors);
    for (int i = 0; i < 50; ++i) {
      sim.advance();
      rig.collect();
    }
    const std::vector<ImuReading> a = rig.take_imu(0, "imu_a");
    const std::vector<ImuReading> b = rig.take_imu(0, "imu_b");
    ASSERT_EQ(a.size(), b.size());
    bool instances_differ = false;
    for (std::size_t k = 0; k < a.size(); ++k) {
      if (a[k].sample.ax != b[k].sample.ax) instances_differ = true;
    }
    EXPECT_TRUE(instances_differ);
  }
}

TEST(SensorRig, AStationaryCarProducesTheUndistortedScan) {
  Simulation sim;
  sim.add_agent(car(0.0));
  SensorRig rig(sim, wall_world(10.0), 5);

  AgentSensors sensors;
  sensors.lidars.push_back(clean_lidar("scan", 40.0, 72));
  rig.attach(0, sensors);

  // One full revolution has to be simulated before the scan exists.
  for (int i = 0; i < 30; ++i) {
    sim.advance();
    rig.collect();
  }
  const std::vector<Scan> scans = rig.take_scans(0, "scan");
  ASSERT_FALSE(scans.empty());
  const Scan& observed = scans.front();

  // The same scan cast by hand from the constant pose. Noise-free, so the
  // rays are pure geometry and the comparison is exact.
  const slipx::sense::Lidar manual(clean_lidar("scan", 40.0, 72).spec);
  slipx::sense::Rng rng(99);  // irrelevant: nothing is drawn into the rays
  const Pose origin;          // the car started at the world origin
  const WorldFunction world = wall_world(10.0);
  const Scan expected = manual.sample(
      0.0, [&origin](double) { return origin; },
      [&world](const Pose& o, double b) { return world(0, o, b); }, rng);

  ASSERT_EQ(observed.rays.size(), expected.rays.size());
  EXPECT_EQ(observed.start_time, expected.start_time);
  EXPECT_EQ(observed.stamp_time, expected.stamp_time);
  for (std::size_t k = 0; k < observed.rays.size(); ++k) {
    EXPECT_EQ(observed.rays[k].valid, expected.rays[k].valid) << "ray " << k;
    if (observed.rays[k].valid && expected.rays[k].valid) {
      EXPECT_EQ(observed.rays[k].range, expected.rays[k].range)
          << "ray " << k;
    }
  }
}

TEST(SensorRig, MotionDistortionEmergesFromThePoseHistory) {
  Simulation sim;
  sim.add_agent(car(5.0));  // closing on the wall at 5 m/s
  SensorRig rig(sim, wall_world(10.0), 5);

  AgentSensors sensors;
  sensors.lidars.push_back(clean_lidar("scan", 40.0, 72));
  rig.attach(0, sensors);

  for (int i = 0; i < 30; ++i) {
    sim.advance();
    rig.collect();
  }
  const std::vector<Scan> scans = rig.take_scans(0, "scan");
  ASSERT_FALSE(scans.empty());
  const Scan& observed = scans.front();

  // The frozen-pose scan the reference stack test settles for: everything
  // cast from where the car was when the revolution started.
  const slipx::sense::Lidar manual(clean_lidar("scan", 40.0, 72).spec);
  slipx::sense::Rng rng(99);
  const Pose origin;
  const WorldFunction world = wall_world(10.0);
  const Scan frozen = manual.sample(
      0.0, [&origin](double) { return origin; },
      [&world](const Pose& o, double b) { return world(0, o, b); }, rng);

  // At 5 m/s over a 25 ms revolution the car moves 12.5 cm, so late rays
  // must measure a visibly shorter wall than the frozen scan claims.
  double largest_difference = 0.0;
  ASSERT_EQ(observed.rays.size(), frozen.rays.size());
  for (std::size_t k = 0; k < observed.rays.size(); ++k) {
    if (observed.rays[k].valid && frozen.rays[k].valid) {
      largest_difference =
          std::max(largest_difference, std::fabs(observed.rays[k].range -
                                                 frozen.rays[k].range));
    }
  }
  EXPECT_GT(largest_difference, 0.05);
}

TEST(SensorRig, RayPosesAreInterpolatedExactly) {
  // A kinematic car coasting straight holds its speed exactly, so the
  // emitter's position at any ray time is analytic and the assertion can be
  // equality to rounding, not plausibility. A rig that served rays from
  // step boundaries instead of interpolating would be off by up to one
  // step of motion, five millimetres here, against a nanometre tolerance.
  Simulation sim;
  sim.add_agent(l0_car(5.0, 0.0));
  SensorRig rig(sim, wall_world(10.0), 5);

  const LidarSensor lidar = clean_lidar("scan", 40.0, 72);
  AgentSensors sensors;
  sensors.lidars.push_back(lidar);
  rig.attach(0, sensors);

  for (int i = 0; i < 30; ++i) {
    sim.advance();
    rig.collect();
  }
  const std::vector<Scan> scans = rig.take_scans(0, "scan");
  ASSERT_FALSE(scans.empty());
  const Scan& scan = scans.front();

  const double revolution = 1.0 / lidar.spec.rate_hz;
  const double span = lidar.spec.angle_max - lidar.spec.angle_min;
  const double count = static_cast<double>(lidar.spec.rays);
  int checked = 0;
  for (std::size_t i = 0; i < scan.rays.size(); ++i) {
    if (!scan.rays[i].valid) continue;
    const double fraction = static_cast<double>(i) / count;
    const double t = fraction * revolution;
    const double angle = lidar.spec.angle_min + fraction * span;
    const double expected = (10.0 - 5.0 * t) / std::cos(angle);
    EXPECT_NEAR(scan.rays[i].range, expected, 1.0e-9) << "ray " << i;
    ++checked;
  }
  EXPECT_GT(checked, 20) << "the wall should be visible to forward rays";
}

TEST(SensorRig, YawInterpolationTakesTheShortestArcThroughTheSeam) {
  // The state's yaw is wrapped to (-pi, pi] every step, so a car turning
  // left through the seam hands the rig two poses whose plain difference is
  // nearly a full circle. The compass world encodes the bearing in the
  // range, so a ray's measurement reveals the yaw the rig interpolated;
  // anything but the shortest arc misses by a number nobody could excuse.
  Simulation sim;
  slipx::sim::AgentSpec spec = l0_car(2.0, 0.3);
  spec.initial_state.yaw = 3.14159265358979323846 - 0.02;
  sim.add_agent(spec);
  SensorRig rig(sim, compass_world(), 5);

  const LidarSensor lidar = clean_lidar("scan", 40.0, 72);
  AgentSensors sensors;
  sensors.lidars.push_back(lidar);
  rig.attach(0, sensors);

  // The independent record: unwrap the per-step yaw in the test itself.
  std::vector<std::pair<double, double>> unwrapped;  // (time, yaw)
  unwrapped.emplace_back(0.0, spec.initial_state.yaw);
  bool crossed = false;
  for (int i = 0; i < 30; ++i) {
    sim.advance();
    rig.collect();
    const double previous = unwrapped.back().second;
    const double next =
        previous +
        std::remainder(sim.state(0).yaw - previous, 6.28318530717958647692);
    if (sim.state(0).yaw < 0.0) crossed = true;
    unwrapped.emplace_back(sim.time(), next);
  }
  ASSERT_TRUE(crossed) << "the scenario must actually cross the seam";

  const std::vector<Scan> scans = rig.take_scans(0, "scan");
  ASSERT_FALSE(scans.empty());
  const Scan& scan = scans.front();

  const double revolution = 1.0 / lidar.spec.rate_hz;
  const double span = lidar.spec.angle_max - lidar.spec.angle_min;
  const double count = static_cast<double>(lidar.spec.rays);
  for (std::size_t i = 0; i < scan.rays.size(); ++i) {
    ASSERT_TRUE(scan.rays[i].valid) << "the compass always answers";
    const double fraction = static_cast<double>(i) / count;
    const double t = fraction * revolution;
    const double angle = lidar.spec.angle_min + fraction * span;

    // Linear interpolation of the unwrapped record, which is what the
    // shortest arc through the wrapped record must agree with.
    double yaw = unwrapped.back().second;
    for (std::size_t s = 1; s < unwrapped.size(); ++s) {
      if (unwrapped[s].first >= t) {
        const auto& a = unwrapped[s - 1];
        const auto& b = unwrapped[s];
        const double u = (t - a.first) / (b.first - a.first);
        yaw = a.second + u * (b.second - a.second);
        break;
      }
    }
    const double expected = 10.0 + std::sin(yaw + angle);
    EXPECT_NEAR(scan.rays[i].range, expected, 1.0e-9) << "ray " << i;
  }
}

TEST(SensorRig, NoiseIntegratesOverTheSensorsOwnInterval) {
  // A noise density is stated per root hertz, so a 100 Hz IMU integrates
  // over 10 ms and its per-sample sigma is density times root rate. A rig
  // that fed the model the physics step instead of the sensor's own period
  // would overstate the noise by root ten, which two hundred samples can
  // tell from the truth with room to spare.
  Simulation sim;
  sim.add_agent(car(0.0));  // at rest: the samples are the noise alone
  SensorRig rig(sim, {}, 21);

  AgentSensors sensors;
  ImuSensor imu = clean_imu("imu", 100.0, 0.0);
  imu.spec.accel_noise_density = 0.002;  // sigma = 0.002 * sqrt(100) = 0.02
  sensors.imus.push_back(imu);
  rig.attach(0, sensors);

  for (int i = 0; i < 2000; ++i) {
    sim.advance();
    rig.collect();
  }
  const std::vector<ImuReading> readings = rig.take_imu(0, "imu");
  ASSERT_GE(readings.size(), 200u);

  double mean = 0.0;
  for (const ImuReading& reading : readings) mean += reading.sample.ax;
  mean /= static_cast<double>(readings.size());
  double variance = 0.0;
  for (const ImuReading& reading : readings) {
    variance += (reading.sample.ax - mean) * (reading.sample.ax - mean);
  }
  variance /= static_cast<double>(readings.size() - 1);
  const double stddev = std::sqrt(variance);

  EXPECT_GT(stddev, 0.013);
  EXPECT_LT(stddev, 0.030);
}

TEST(SensorRig, TheWorldKnowsWhoIsAsking) {
  // The world function receives the asking agent, because a car must not
  // see itself in an overlay only the caller understands. A world whose
  // answer encodes the asker makes a dropped index visible.
  Simulation sim;
  sim.add_agent(car(0.0));
  sim.add_agent(car(0.0));
  const WorldFunction who = [](std::size_t agent, const Pose&, double) {
    Hit hit;
    hit.hit = true;
    hit.range = 5.0 + static_cast<double>(agent);
    return hit;
  };
  SensorRig rig(sim, who, 4);

  AgentSensors sensors;
  sensors.lidars.push_back(clean_lidar("scan", 40.0, 36));
  rig.attach(0, sensors);
  rig.attach(1, sensors);

  for (int i = 0; i < 30; ++i) {
    sim.advance();
    rig.collect();
  }
  for (std::size_t agent = 0; agent < 2; ++agent) {
    const std::vector<Scan> scans = rig.take_scans(agent, "scan");
    ASSERT_FALSE(scans.empty());
    for (const auto& ray : scans.front().rays) {
      ASSERT_TRUE(ray.valid);
      EXPECT_EQ(ray.range, 5.0 + static_cast<double>(agent));
    }
  }
}

TEST(SensorRig, ACheapOpponentCarriesNothing) {
  Simulation sim;
  sim.add_agent(car(2.0));
  sim.add_agent(car(2.0));
  SensorRig rig(sim, wall_world(10.0), 5);

  AgentSensors sensors;
  sensors.lidars.push_back(clean_lidar("scan", 40.0, 36));
  rig.attach(0, sensors);
  rig.attach(1, AgentSensors{});  // explicitly nothing: the cheap opponent

  for (int i = 0; i < 60; ++i) {
    sim.advance();
    rig.collect();
  }

  EXPECT_FALSE(rig.take_scans(0, "scan").empty());
  EXPECT_THROW(rig.take_scans(1, "scan"), std::invalid_argument);
  EXPECT_THROW(rig.latest_scan(1, "scan"), std::invalid_argument);
}

TEST(SensorRig, ADnfdAgentStopsSensingButPendingStillArrives) {
  SimulationConfig config;
  Simulation sim(config);
  slipx::sim::AgentSpec spec;
  spec.tier = Tier::L1_Bicycle;
  spec.initial_state.vel_body.x = 2.0;
  spec.mailbox = std::make_shared<slipx::sim::CommandMailbox>();
  spec.timeout_policy = slipx::sim::TimeoutPolicy::kDnf;
  const std::size_t agent = sim.add_agent(spec);

  SensorRig rig(sim, {}, 9);
  AgentSensors sensors;
  ImuSensor imu = clean_imu("imu", 100.0, 0.0);
  imu.latency_s = 0.005;  // long enough to outlive the car
  sensors.imus.push_back(imu);
  rig.attach(agent, sensors);

  // Twenty commanded steps, then silence: the next advance rules the miss
  // and the timeout policy ends the run (ADR-0044).
  for (std::uint64_t step = 0; step < 20; ++step) {
    spec.mailbox->post(step, DriveInput{});
    sim.advance();
    rig.collect();
  }
  ASSERT_TRUE(sim.agent_running(agent));
  sim.advance();
  ASSERT_FALSE(sim.agent_running(agent));
  const double dnf_time = sim.dnf(agent)->time;

  std::vector<ImuReading> all;
  for (int i = 0; i < 30; ++i) {
    rig.collect();
    for (const ImuReading& reading : rig.take_imu(agent, "imu")) {
      all.push_back(reading);
    }
    sim.advance();
  }

  ASSERT_FALSE(all.empty());
  for (const ImuReading& reading : all) {
    // Nothing was measured after the car's run ended; what was already
    // measured still arrived, stamps and all.
    EXPECT_LT(reading.sample.time, dnf_time);
  }
  // Samples measured just before the DNF deliver just after it.
  EXPECT_GT(all.back().stamp_time, all.back().sample.time);
}

TEST(SensorRig, TakeDrainsAndLatestPersists) {
  Simulation sim;
  sim.add_agent(car(2.0));
  SensorRig rig(sim, {}, 2);
  AgentSensors sensors;
  sensors.imus.push_back(clean_imu("imu", 100.0, 0.0));
  rig.attach(0, sensors);

  for (int i = 0; i < 50; ++i) {
    sim.advance();
    rig.collect();
  }
  const std::vector<ImuReading> first = rig.take_imu(0, "imu");
  ASSERT_FALSE(first.empty());
  EXPECT_TRUE(rig.take_imu(0, "imu").empty()) << "take drains";
  ASSERT_TRUE(rig.latest_imu(0, "imu").has_value()) << "latest persists";
  EXPECT_EQ(rig.latest_imu(0, "imu")->stamp_time, first.back().stamp_time);
}

TEST(SensorRig, RefusalsNameTheirReasons) {
  Simulation sim;
  sim.add_agent(car(2.0));

  // A LiDAR needs a world.
  SensorRig blind(sim, {}, 1);
  AgentSensors with_lidar;
  with_lidar.lidars.push_back(clean_lidar("scan", 40.0, 36));
  EXPECT_THROW(blind.attach(0, with_lidar), std::invalid_argument);

  SensorRig rig(sim, wall_world(10.0), 1);
  EXPECT_THROW(rig.attach(3, AgentSensors{}), std::invalid_argument);

  AgentSensors bad_phase;
  bad_phase.imus.push_back(clean_imu("imu", 100.0, 1.0));
  EXPECT_THROW(rig.attach(0, bad_phase), std::invalid_argument);

  AgentSensors bad_jitter;
  ImuSensor imu = clean_imu("imu", 100.0, 0.0);
  imu.latency_s = 0.001;
  imu.latency_jitter_s = 0.002;  // could stamp before the measurement
  bad_jitter.imus.push_back(imu);
  EXPECT_THROW(rig.attach(0, bad_jitter), std::invalid_argument);

  AgentSensors duplicate;
  duplicate.imus.push_back(clean_imu("imu", 100.0, 0.0));
  duplicate.imus.push_back(clean_imu("imu", 100.0, 0.0));
  EXPECT_THROW(rig.attach(0, duplicate), std::invalid_argument);

  // The agent is L1, which never computes wheel speeds: an encoder on it
  // would report a moving car as stationary, so the rig refuses it.
  AgentSensors encoder_below_l2;
  encoder_below_l2.encoders.push_back(EncoderSensor{});
  encoder_below_l2.encoders.back().name = "wheel_encoder";
  EXPECT_THROW(rig.attach(0, encoder_below_l2), std::invalid_argument);

  // Attaching after the first collect has no start time to offer.
  AgentSensors fine;
  fine.imus.push_back(clean_imu("imu", 100.0, 0.0));
  rig.attach(0, fine);
  sim.advance();
  rig.collect();
  EXPECT_THROW(rig.attach(0, AgentSensors{}), std::logic_error);

  // A reset moves time backwards under the rig, and the rig says so rather
  // than serving stale history (ADR-0047).
  sim.reset();
  EXPECT_THROW(rig.collect(), std::logic_error);
}

}  // namespace
