// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The 2D LiDAR.
//
// The world used here is a circle of known radius centred on the origin,
// because the range from any point inside it along any bearing has a closed
// form and the tests can therefore assert a number rather than a shape. That
// is the payoff of the sensor not knowing what it is looking at (ADR-0037):
// a track would have made every one of these assertions approximate.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "slipx/sense/lidar.hpp"
#include "slipx/sense/rng.hpp"

namespace {

using slipx::sense::Hit;
using slipx::sense::Lidar;
using slipx::sense::LidarSpec;
using slipx::sense::Pose;
using slipx::sense::Rng;
using slipx::sense::Scan;

constexpr double kPi = 3.14159265358979323846;
constexpr double kWallRadius = 10.0;

// Distance from a point inside a circle to the circle, along a bearing.
// Solving |o + t d| = R for the positive root.
double range_to_circle(const Pose& from, double bearing, double radius) {
  const double dx = std::cos(bearing);
  const double dy = std::sin(bearing);
  const double b = from.x * dx + from.y * dy;
  const double c = from.x * from.x + from.y * from.y - radius * radius;
  const double discriminant = b * b - c;
  if (discriminant < 0.0) return -1.0;
  return -b + std::sqrt(discriminant);
}

Hit circular_wall(const Pose& origin, double bearing) {
  Hit hit;
  const double range = range_to_circle(origin, bearing, kWallRadius);
  if (range < 0.0) return hit;
  hit.hit = true;
  hit.range = range;
  return hit;
}

LidarSpec quiet_spec() {
  LidarSpec spec;
  spec.rate_hz = 10.0;
  spec.rays = 360;
  spec.angle_min = -kPi;
  spec.angle_max = kPi;
  spec.range_min = 0.02;
  spec.range_max = 30.0;
  spec.noise_base_m = 0.0;
  spec.noise_per_metre = 0.0;
  spec.dropout_probability = 0.0;
  return spec;
}

// A car sitting still at the centre of the circle.
Pose stationary(double) { return Pose{}; }

// ------------------------------------------------------------- the geometry

TEST(Lidar, AStationaryScanFromTheCentreIsTheWallRadiusEverywhere) {
  const Lidar lidar(quiet_spec());
  Rng rng(1);

  const Scan scan = lidar.sample(0.0, stationary, circular_wall, rng);

  ASSERT_EQ(scan.rays.size(), 360u);
  for (const auto& ray : scan.rays) {
    ASSERT_TRUE(ray.valid);
    EXPECT_NEAR(ray.range, kWallRadius, 1e-9);
  }
}

TEST(Lidar, RaysAreSpreadOverTheRevolutionInTimeAndInAngle) {
  const Lidar lidar(quiet_spec());
  Rng rng(1);

  const Scan scan = lidar.sample(5.0, stationary, circular_wall, rng);

  EXPECT_DOUBLE_EQ(scan.start_time, 5.0);
  EXPECT_DOUBLE_EQ(scan.rays.front().time, 5.0);
  EXPECT_DOUBLE_EQ(scan.rays.front().angle, -kPi);

  // The last ray lands one gap short of a full revolution, so that two
  // consecutive scans do not emit two rays at the same instant.
  const double gap = lidar.period() / 360.0;
  EXPECT_NEAR(scan.rays.back().time, 5.0 + lidar.period() - gap, 1e-12);
  EXPECT_LT(scan.rays.back().time, 5.0 + lidar.period());

  for (std::size_t i = 1; i < scan.rays.size(); ++i) {
    EXPECT_GT(scan.rays[i].time, scan.rays[i - 1].time);
    EXPECT_GT(scan.rays[i].angle, scan.rays[i - 1].angle);
  }
}

// --------------------------------------------------------- motion distortion
//
// The case M5.4 exists for. A car crossing the circle while the head spins
// measures ranges from a different place for every ray, and the scan is not
// the shape of the world. Nothing in the sensor multiplies anything by a
// speed; the distortion is entirely a consequence of asking for the pose at
// each ray's own timestamp.

TEST(Lidar, AMovingCarProducesADistortedScanAndAStationaryOneDoesNot) {
  const Lidar lidar(quiet_spec());
  Rng rng(1);

  // Still, at the centre: every range is the radius, so the spread is zero.
  const Scan still = lidar.sample(0.0, stationary, circular_wall, rng);

  double still_min = kWallRadius, still_max = kWallRadius;
  for (const auto& ray : still.rays) {
    still_min = std::fmin(still_min, ray.range);
    still_max = std::fmax(still_max, ray.range);
  }
  EXPECT_NEAR(still_max - still_min, 0.0, 1e-9);

  // Now the same scan while the car drives across the circle at 8 m/s. In
  // the 0.1 s the head takes to turn, it moves 0.8 m, so rays emitted early
  // and late in the revolution are cast from points 0.8 m apart.
  const auto driving = [](double time) {
    Pose pose;
    pose.x = 8.0 * time;
    return pose;
  };

  const Scan moving = lidar.sample(0.0, driving, circular_wall, rng);

  // A ray straight ahead early in the scan and one straight ahead late in it
  // would have measured the same range from a stationary car. Here the
  // forward ray is emitted once, so compare against the undistorted answer
  // from the pose at the START of the scan, which is what a consumer that
  // treats a scan as a snapshot assumes.
  double worst = 0.0;
  const Pose at_start = driving(0.0);
  for (const auto& ray : moving.rays) {
    ASSERT_TRUE(ray.valid);
    const double if_it_were_a_snapshot =
        range_to_circle(at_start, ray.angle, kWallRadius);
    worst = std::fmax(worst, std::fabs(ray.range - if_it_were_a_snapshot));
  }

  // Measurably distorted: the error against the snapshot assumption is of the
  // order of the distance travelled, not of the order of the noise floor.
  EXPECT_GT(worst, 0.2) << "a scan taken while moving is not a snapshot";
  EXPECT_LT(worst, 1.0) << "and not more distorted than the car moved";
}

TEST(Lidar, DistortionScalesWithHowFarTheCarMovedDuringTheScan) {
  const Lidar lidar(quiet_spec());
  Rng rng(1);

  const auto spread_at = [&](double speed) {
    const auto motion = [speed](double time) {
      Pose pose;
      pose.x = speed * time;
      return pose;
    };
    const Scan scan = lidar.sample(0.0, motion, circular_wall, rng);
    const Pose at_start = motion(0.0);
    double worst = 0.0;
    for (const auto& ray : scan.rays) {
      worst = std::fmax(worst, std::fabs(ray.range - range_to_circle(
                                                        at_start, ray.angle,
                                                        kWallRadius)));
    }
    return worst;
  };

  const double slow = spread_at(1.0);
  const double fast = spread_at(8.0);

  EXPECT_GT(fast, slow * 4.0) << "eight times the speed, and not a constant";
  EXPECT_NEAR(spread_at(0.0), 0.0, 1e-9);
}

// ----------------------------------------------------------------- the noise

TEST(Lidar, NoiseIsSeededAndReproducible) {
  LidarSpec spec = quiet_spec();
  spec.noise_base_m = 0.02;
  const Lidar lidar(spec);

  Rng one(42), two(42), three(43);
  const Scan a = lidar.sample(0.0, stationary, circular_wall, one);
  const Scan b = lidar.sample(0.0, stationary, circular_wall, two);
  const Scan c = lidar.sample(0.0, stationary, circular_wall, three);

  for (std::size_t i = 0; i < a.rays.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.rays[i].range, b.rays[i].range) << "ray " << i;
  }

  bool any_difference = false;
  for (std::size_t i = 0; i < a.rays.size(); ++i) {
    if (a.rays[i].range != c.rays[i].range) any_difference = true;
  }
  EXPECT_TRUE(any_difference) << "a different seed is a different scan";
}

TEST(Lidar, NoiseGrowsWithRange) {
  // A wall ten times further away, with a purely range-proportional noise, so
  // the measured spread should grow with it.
  LidarSpec spec = quiet_spec();
  spec.noise_base_m = 0.0;
  spec.noise_per_metre = 0.01;
  spec.rays = 2000;
  const Lidar lidar(spec);

  const auto spread_against = [&](double radius) {
    const auto wall = [radius](const Pose& origin, double bearing) {
      Hit hit;
      const double range = range_to_circle(origin, bearing, radius);
      if (range < 0.0) return hit;
      hit.hit = true;
      hit.range = range;
      return hit;
    };
    Rng rng(7);
    const Scan scan = lidar.sample(0.0, stationary, wall, rng);
    double sum_squares = 0.0;
    std::size_t count = 0;
    for (const auto& ray : scan.rays) {
      if (!ray.valid) continue;
      const double error = ray.range - radius;
      sum_squares += error * error;
      ++count;
    }
    return std::sqrt(sum_squares / static_cast<double>(count));
  };

  const double near_wall = spread_against(1.0);
  const double far_wall = spread_against(10.0);

  EXPECT_NEAR(near_wall, 0.01, 0.002);
  EXPECT_NEAR(far_wall, 0.10, 0.02);
}

// ------------------------------------------------------------- the dropouts

TEST(Lidar, DropoutsArriveAsNaNAndNeverAsZero) {
  LidarSpec spec = quiet_spec();
  spec.dropout_probability = 0.5;
  const Lidar lidar(spec);
  Rng rng(3);

  const Scan scan = lidar.sample(0.0, stationary, circular_wall, rng);

  std::size_t dropped = 0;
  for (const auto& ray : scan.rays) {
    if (ray.valid) continue;
    ++dropped;
    // The rule this shares with every other part of SlipX: a quantity that
    // was not measured is absent, and a dropout reported as a range of zero
    // is a wall against the mast.
    EXPECT_TRUE(std::isnan(ray.range));
  }

  EXPECT_GT(dropped, 100u);
  EXPECT_LT(dropped, 260u);
}

TEST(Lidar, AMaterialFactorScalesTheDropoutProbability) {
  LidarSpec spec = quiet_spec();
  spec.dropout_probability = 0.5;
  const Lidar lidar(spec);

  const auto wall_with_material = [](double factor) {
    return [factor](const Pose& origin, double bearing) {
      Hit hit = circular_wall(origin, bearing);
      hit.material_dropout = factor;
      return hit;
    };
  };

  const auto dropped_with = [&](double factor) {
    Rng rng(3);
    const Scan scan =
        lidar.sample(0.0, stationary, wall_with_material(factor), rng);
    std::size_t dropped = 0;
    for (const auto& ray : scan.rays) {
      if (!ray.valid) ++dropped;
    }
    return dropped;
  };

  // A surface a LiDAR never misses, an ordinary one, and black foam.
  EXPECT_EQ(dropped_with(0.0), 0u);
  EXPECT_GT(dropped_with(2.0), dropped_with(1.0));
}

// The reason the generator is advanced the same number of times per ray
// whatever the ray did. Without it, one extra dropout early in a scan shifts
// every later ray onto a different draw, and two runs that differ by one
// distant wall differ everywhere.
TEST(Lidar, TheDrawsPerRayDoNotDependOnWhatTheRayFound) {
  LidarSpec spec = quiet_spec();
  spec.noise_base_m = 0.01;
  const Lidar lidar(spec);

  // A world where one ray misses entirely, and the same world where it does
  // not. Every other ray must be unchanged.
  const auto with_a_gap = [](const Pose& origin, double bearing) {
    Hit hit = circular_wall(origin, bearing);
    if (bearing > -kPi + 0.02 && bearing < -kPi + 0.04) hit.hit = false;
    return hit;
  };

  Rng one(11), two(11);
  const Scan whole = lidar.sample(0.0, stationary, circular_wall, one);
  const Scan gapped = lidar.sample(0.0, stationary, with_a_gap, two);

  std::size_t differing = 0;
  for (std::size_t i = 0; i < whole.rays.size(); ++i) {
    if (whole.rays[i].valid != gapped.rays[i].valid) {
      ++differing;
    } else if (whole.rays[i].valid) {
      EXPECT_DOUBLE_EQ(whole.rays[i].range, gapped.rays[i].range)
          << "ray " << i << " moved because a different ray missed";
    }
  }
  EXPECT_GE(differing, 1u);
  EXPECT_LE(differing, 3u);
}

// ------------------------------------------------------------- the timestamps

TEST(Lidar, LatencyStampsTheScanAfterItsLastRay) {
  LidarSpec spec = quiet_spec();
  spec.latency_s = 0.005;
  const Lidar lidar(spec);
  Rng rng(1);

  const Scan scan = lidar.sample(2.0, stationary, circular_wall, rng);

  EXPECT_GT(scan.stamp_time, scan.rays.back().time);
  EXPECT_NEAR(scan.stamp_time - scan.rays.back().time, 0.005, 1e-12);
}

TEST(Lidar, LatencyJitterIsSeededAndBounded) {
  LidarSpec spec = quiet_spec();
  spec.latency_s = 0.010;
  spec.latency_jitter_s = 0.004;
  const Lidar lidar(spec);

  Rng one(5), two(5);
  std::vector<double> delays;
  for (int i = 0; i < 20; ++i) {
    const Scan a = lidar.sample(i * 0.1, stationary, circular_wall, one);
    const Scan b = lidar.sample(i * 0.1, stationary, circular_wall, two);
    EXPECT_DOUBLE_EQ(a.stamp_time, b.stamp_time) << "scan " << i;

    const double delay = a.stamp_time - a.rays.back().time;
    EXPECT_GE(delay, 0.006 - 1e-12);
    EXPECT_LE(delay, 0.014 + 1e-12);
    delays.push_back(delay);
  }

  bool varies = false;
  for (double delay : delays) {
    if (std::fabs(delay - delays.front()) > 1e-9) varies = true;
  }
  EXPECT_TRUE(varies) << "jitter that never jitters is a constant";
}

// Latency and rate are set by different things on real hardware, and a stack
// that assumes a scan arrives one period after it started has a bug this is
// meant to expose rather than hide.
TEST(Lidar, LatencyIsIndependentOfTheRate) {
  LidarSpec slow = quiet_spec();
  slow.rate_hz = 5.0;
  slow.latency_s = 0.02;

  LidarSpec fast = quiet_spec();
  fast.rate_hz = 40.0;
  fast.latency_s = 0.02;

  Rng one(1), two(1);
  const Scan a = Lidar(slow).sample(0.0, stationary, circular_wall, one);
  const Scan b = Lidar(fast).sample(0.0, stationary, circular_wall, two);

  EXPECT_NEAR(a.stamp_time - a.rays.back().time, 0.02, 1e-12);
  EXPECT_NEAR(b.stamp_time - b.rays.back().time, 0.02, 1e-12);
  EXPECT_GT(a.rays.back().time, b.rays.back().time) << "but the scans differ";
}

// ------------------------------------------------------------- the refusals

TEST(Lidar, RefusesASpecItCannotHonour) {
  const auto build = [](void (*edit)(LidarSpec&)) {
    LidarSpec spec = quiet_spec();
    edit(spec);
    return Lidar(spec);
  };

  EXPECT_THROW(build([](LidarSpec& s) { s.rate_hz = 0.0; }),
               std::invalid_argument);
  EXPECT_THROW(build([](LidarSpec& s) { s.rays = 1; }), std::invalid_argument);
  EXPECT_THROW(build([](LidarSpec& s) { s.angle_max = s.angle_min; }),
               std::invalid_argument);
  EXPECT_THROW(build([](LidarSpec& s) { s.range_max = s.range_min; }),
               std::invalid_argument);
  EXPECT_THROW(build([](LidarSpec& s) { s.latency_s = -1.0; }),
               std::invalid_argument);
  EXPECT_THROW(build([](LidarSpec& s) { s.dropout_probability = 1.5; }),
               std::invalid_argument);

  // Jitter wider than the delay would let a message be stamped before the
  // ray that produced it.
  EXPECT_THROW(build([](LidarSpec& s) {
                 s.latency_s = 0.001;
                 s.latency_jitter_s = 0.002;
               }),
               std::invalid_argument);
}

// Noise can push a good return past the end of the unit's window, and a real
// unit then reports nothing rather than a number it cannot have measured. The
// wall here sits just inside the maximum with noise wide enough to cross it.
TEST(Lidar, AMeasurementNoisePushesOutsideTheWindowIsDropped) {
  LidarSpec spec = quiet_spec();
  spec.range_max = 10.2;
  spec.noise_base_m = 0.5;
  spec.rays = 2000;
  const Lidar lidar(spec);
  Rng rng(9);

  const Scan scan = lidar.sample(0.0, stationary, circular_wall, rng);

  std::size_t dropped = 0;
  for (const auto& ray : scan.rays) {
    if (ray.valid) {
      // Whatever else happens, a valid range is inside the window. Reporting
      // 10.7 m from a unit that cannot see past 10.2 m is worse than
      // reporting nothing.
      EXPECT_GE(ray.range, spec.range_min);
      EXPECT_LE(ray.range, spec.range_max);
    } else {
      ++dropped;
      EXPECT_TRUE(std::isnan(ray.range));
    }
  }

  EXPECT_GT(dropped, 100u) << "with this much noise some returns must be lost";
}

// The other end of the same window. A unit has a minimum range because the
// receiver cannot resolve a return from inside it, and a wall closer than
// that reads as nothing rather than as a very short range.
// The noise matters here and is the whole point of the case. Without it the
// measurement stays at 10 cm and the check on the noisy value would catch it
// anyway; with it, a blind-zone return that nothing rejected up front could
// be pushed out past the minimum and reported as a real wall half a metre
// away. A return from inside the blind zone is not a measurement to be
// perturbed, it is a measurement the unit never made.
TEST(Lidar, AReturnInsideTheMinimumRangeIsAbsent) {
  LidarSpec spec = quiet_spec();
  spec.range_min = 0.5;
  spec.noise_base_m = 0.5;
  spec.rays = 2000;
  const Lidar lidar(spec);
  Rng rng(1);

  // A wall at 10 cm, well inside the 50 cm minimum.
  const auto close_wall = [](const Pose& origin, double bearing) {
    Hit hit;
    hit.hit = true;
    hit.range = range_to_circle(origin, bearing, 0.1);
    return hit;
  };

  const Scan scan = lidar.sample(0.0, stationary, close_wall, rng);

  for (const auto& ray : scan.rays) {
    EXPECT_FALSE(ray.valid);
    EXPECT_TRUE(std::isnan(ray.range));
  }
}

TEST(Lidar, ARangeOutsideTheUnitsWindowIsAbsentNotClamped) {
  LidarSpec spec = quiet_spec();
  spec.range_max = 5.0;  // the wall is at 10 m
  const Lidar lidar(spec);
  Rng rng(1);

  const Scan scan = lidar.sample(0.0, stationary, circular_wall, rng);

  for (const auto& ray : scan.rays) {
    EXPECT_FALSE(ray.valid);
    EXPECT_TRUE(std::isnan(ray.range)) << "clamping would invent a wall at 5 m";
  }
}

}  // namespace
