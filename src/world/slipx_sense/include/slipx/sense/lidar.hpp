// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// A 2D scanning LiDAR.
//
// The thing this model exists to get right is that a scan is not a snapshot.
// A spinning LiDAR emits its rays one at a time over the duration of a
// revolution, and a car moving while it spins drags the scan into a shape the
// world does not have. That distortion is not a defect to be modelled as an
// error term: it is what the sensor genuinely measures, and a stack that
// deskews scans is a stack that has to be tested against scans that need
// deskewing.
//
// So every ray carries its own timestamp, and every ray is cast from the pose
// the emitter had at that timestamp. Distortion then emerges from the same
// motion the vehicle model produced, rather than being applied afterwards as
// a function of speed, and a stationary car produces a scan with none of it
// without anything having to special-case standing still.
//
// What the ray hits is not this component's question. The world arrives as a
// function from a ray to a distance (ADR-0037), so this file never learns
// what a track is, and a test can point the sensor at a circle whose exact
// answer is known.
//
// Three sources of imperfection, each seeded and reproducible:
//
//   latency    a constant plus jitter, configured independently of the rate,
//              because the two are independent on real hardware and a driver
//              that assumes otherwise is the bug this exists to expose
//   noise      range-dependent standard deviation, since a return from 8 m is
//              not as good as one from 0.5 m
//   dropouts   a base probability, multiplied by a per-material factor the
//              world function reports, because a black foam wall and a
//              retroreflective post are not the same target

#ifndef SLIPX_SENSE_LIDAR_HPP
#define SLIPX_SENSE_LIDAR_HPP

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "slipx/sense/rng.hpp"

namespace slipx {
namespace sense {

// Where the emitter is, in the world frame. ISO 8855: x forward, y left, yaw
// positive anticlockwise.
struct Pose {
  double x = 0.0;    // [m]
  double y = 0.0;    // [m]
  double yaw = 0.0;  // [rad]
};

// What a ray found. `material_dropout` scales the configured dropout
// probability for this target: 1 is an ordinary surface, 0 never drops, and
// larger values are for surfaces a LiDAR struggles with. It is a parameter
// rather than a table of materials because the table would be a list of
// guesses, and this way a world that knows nothing about materials returns 1
// and is honest.
struct Hit {
  bool hit = false;
  double range = 0.0;             // [m]
  double material_dropout = 1.0;  // [-]
};

// The world, as the sensor can see it: cast a ray from `origin` along
// `direction` (a world-frame bearing in radians) and say what it found.
using RangeFunction = std::function<Hit(const Pose& origin, double bearing)>;

// Where the emitter was at a given time. Called once per ray, with that ray's
// own timestamp, which is the whole mechanism behind motion distortion.
using PoseFunction = std::function<Pose(double time)>;

struct LidarSpec {
  // Full revolutions per second. Rays are spread evenly across one
  // revolution in time, which is what makes the scan take as long as it
  // takes.                                                             [Hz]
  double rate_hz = 10.0;

  std::size_t rays = 1080;

  // The angular sector, in the sensor frame, relative to the car's heading.
  // A full-circle unit has -pi to pi; most competition units have a blind
  // arc behind the mast.                                              [rad]
  double angle_min = -2.35619449019234;   // -135 degrees
  double angle_max = 2.35619449019234;    // +135 degrees

  double range_min = 0.02;  // below this the unit reports nothing      [m]
  double range_max = 30.0;  // beyond this the unit reports nothing     [m]

  // Constant transport delay from measurement to the data being available,
  // and the width of the uniform jitter added to it. Configured
  // independently of the rate: on real hardware they are set by different
  // things, and a stack that assumes a scan arrives one period after it
  // started is a stack with a bug this makes visible.                   [s]
  double latency_s = 0.0;
  double latency_jitter_s = 0.0;

  // Range noise, standard deviation = noise_base_m + noise_per_metre * range.
  // Two terms because a real unit has a floor it cannot do better than and a
  // component that grows with distance.                            [m], [-]
  double noise_base_m = 0.01;
  double noise_per_metre = 0.002;

  // Probability that an ordinary return is lost, before the material factor.
  //                                                                    [-]
  double dropout_probability = 0.0;
};

// One ray. A dropped or out-of-range ray is `valid == false` and its range is
// NaN, never zero: zero is a wall against the mast, and reporting one is how
// a dropout becomes an emergency stop (ADR-0006).
struct Ray {
  double time = 0.0;   // when this ray was emitted, simulation time     [s]
  double angle = 0.0;  // sensor-frame bearing                         [rad]
  double range = 0.0;  // measured range, NaN when not valid             [m]
  bool valid = false;
};

struct Scan {
  // When the first ray was emitted, and when the scan became available to a
  // consumer. The gap is the revolution plus the latency, and the two are
  // separate fields because a stack that confuses them is a stack whose
  // timestamps are wrong by a scan period.
  double start_time = 0.0;   // [s]
  double stamp_time = 0.0;   // [s]
  std::vector<Ray> rays;
};

class Lidar {
 public:
  // Throws std::invalid_argument, naming the field, for a rate that is not
  // positive, fewer than two rays, an inverted or empty angular sector, an
  // inverted range window, negative latency or jitter, negative noise, or a
  // dropout probability outside [0, 1].
  explicit Lidar(const LidarSpec& spec);

  const LidarSpec& spec() const { return spec_; }

  // The interval between the starts of two scans.                       [s]
  double period() const { return 1.0 / spec_.rate_hz; }

  // Produce one scan beginning at `start_time`.
  //
  // `pose_at` is called once per ray with that ray's emission time, so the
  // caller decides how motion between poses is interpolated; feeding it a
  // constant pose is how a stationary car is expressed, and it produces an
  // undistorted scan with nothing here checking for it.
  //
  // `rng` is the agent's own stream. It is advanced by a fixed number of
  // draws per ray whatever happens, so that a dropped ray does not shift
  // every later ray's noise onto a different draw and make the scan depend
  // on the world in a way nobody intended.
  Scan sample(double start_time, const PoseFunction& pose_at,
              const RangeFunction& world, Rng& rng) const;

 private:
  LidarSpec spec_;
};

}  // namespace sense
}  // namespace slipx

#endif  // SLIPX_SENSE_LIDAR_HPP
