// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/sense/lidar.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace slipx {
namespace sense {
namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

void require(bool ok, const char* message) {
  if (!ok) throw std::invalid_argument(std::string("lidar: ") + message);
}

}  // namespace

Lidar::Lidar(const LidarSpec& spec) : spec_(spec) {
  require(std::isfinite(spec.rate_hz) && spec.rate_hz > 0.0,
          "rate_hz must be positive [Hz]");
  require(spec.rays >= 2, "rays must be at least 2; one ray is not a scan");
  require(std::isfinite(spec.angle_min) && std::isfinite(spec.angle_max),
          "angle_min and angle_max must be finite [rad]");
  require(spec.angle_max > spec.angle_min,
          "angle_max must exceed angle_min [rad]");
  require(spec.range_min >= 0.0, "range_min must not be negative [m]");
  require(spec.range_max > spec.range_min,
          "range_max must exceed range_min [m]");
  require(std::isfinite(spec.latency_s) && spec.latency_s >= 0.0,
          "latency_s must not be negative [s]");
  require(std::isfinite(spec.latency_jitter_s) && spec.latency_jitter_s >= 0.0,
          "latency_jitter_s must not be negative [s]");
  require(spec.noise_base_m >= 0.0, "noise_base_m must not be negative [m]");
  require(spec.noise_per_metre >= 0.0,
          "noise_per_metre must not be negative [-]");
  require(spec.dropout_probability >= 0.0 && spec.dropout_probability <= 1.0,
          "dropout_probability must be between 0 and 1 [-]");
  // Jitter is symmetric about the constant delay, so a jitter wider than the
  // delay would let a message arrive before the ray that produced it. That is
  // not a configuration anybody means, and letting it through would put a
  // scan in the past on somebody else's timeline.
  require(spec.latency_jitter_s <= spec.latency_s,
          "latency_jitter_s must not exceed latency_s, or a scan could be "
          "stamped before the ray that produced it [s]");
}

Scan Lidar::sample(double start_time, const PoseFunction& pose_at,
                   const RangeFunction& world, Rng& rng) const {
  Scan scan;
  scan.start_time = start_time;
  scan.rays.reserve(spec_.rays);

  const double revolution = period();
  const double span = spec_.angle_max - spec_.angle_min;

  // Rays are spread over the revolution in time as well as in angle, and the
  // last ray lands one gap short of a full period rather than on it, so that
  // consecutive scans do not emit two rays at the same instant.
  const double count = static_cast<double>(spec_.rays);

  for (std::size_t i = 0; i < spec_.rays; ++i) {
    const double fraction = static_cast<double>(i) / count;

    Ray ray;
    ray.time = start_time + fraction * revolution;
    ray.angle = spec_.angle_min + fraction * span;

    // The pose at this ray's own time. This one line is the whole of motion
    // distortion: nothing below multiplies anything by a speed.
    const Pose emitter = pose_at(ray.time);
    const Hit found = world(emitter, emitter.yaw + ray.angle);

    // Two draws per ray, always, whether or not they are used. A generator
    // advanced a different number of times depending on what the ray hit
    // would make every later ray's noise depend on the world, so two runs
    // that differ by one distant wall would differ everywhere.
    const double dropout_draw = rng.uniform();
    const double noise_draw = rng.normal();

    const bool in_window = found.hit && found.range >= spec_.range_min &&
                           found.range <= spec_.range_max;

    const double dropout_probability =
        spec_.dropout_probability * found.material_dropout;
    const bool dropped = dropout_draw < dropout_probability;

    if (!in_window || dropped) {
      // NaN, never zero. A zero range is a wall against the mast, and a
      // dropout reported as one is how a missing return becomes an emergency
      // stop in somebody's controller (ADR-0006).
      ray.range = kNaN;
      ray.valid = false;
    } else {
      const double sigma =
          spec_.noise_base_m + spec_.noise_per_metre * found.range;
      double measured = found.range + sigma * noise_draw;

      // Noise can push a return outside the unit's window, and a real unit
      // reports nothing rather than reporting an impossible number.
      if (measured < spec_.range_min || measured > spec_.range_max) {
        ray.range = kNaN;
        ray.valid = false;
      } else {
        ray.range = measured;
        ray.valid = true;
      }
    }

    scan.rays.push_back(ray);
  }

  // Latency is drawn once per scan, not once per ray: the delay is the
  // transport of the finished message, and rays within one scan do not
  // overtake each other. Uniform jitter, because a bus delay is bounded and a
  // normal one is not.
  const double jitter =
      spec_.latency_jitter_s > 0.0
          ? rng.uniform(-spec_.latency_jitter_s, spec_.latency_jitter_s)
          : 0.0;
  const double last_ray_time = start_time + (count - 1.0) / count * revolution;
  scan.stamp_time = last_ray_time + spec_.latency_s + jitter;

  return scan;
}

}  // namespace sense
}  // namespace slipx
