// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The hand-rolled linear algebra that replaced Eigen (D-02). Small enough to
// be obviously correct and therefore worth testing exhaustively, because
// everything above it inherits its mistakes.

#include <gtest/gtest.h>

#include <type_traits>

#include "slipx/math.hpp"

namespace {

using slipx::Mat3;
using slipx::Vec2;
using slipx::Vec3;

// The whole snapshot/restore-is-a-memcpy argument (CORE-03, SIM-08) rests on
// this, so it is asserted at compile time rather than assumed.
static_assert(std::is_trivially_copyable<Vec2>::value, "");
static_assert(std::is_trivially_copyable<Vec3>::value, "");
static_assert(std::is_trivially_copyable<Mat3>::value, "");
static_assert(std::is_standard_layout<Vec3>::value, "");

// Default construction must zero, not leave indeterminate values: an
// uninitialised velocity is the kind of bug that reproduces once a week.
static_assert(Vec3{}.x == 0.0 && Vec3{}.y == 0.0 && Vec3{}.z == 0.0, "");

TEST(Vec3, ArithmeticIsElementwise) {
  const Vec3 a{1.0, 2.0, 3.0};
  const Vec3 b{0.5, -1.0, 4.0};

  EXPECT_EQ((a + b).x, 1.5);
  EXPECT_EQ((a + b).y, 1.0);
  EXPECT_EQ((a + b).z, 7.0);
  EXPECT_EQ((a - b).y, 3.0);
  EXPECT_EQ((a * 2.0).z, 6.0);
  EXPECT_EQ((2.0 * a).z, 6.0);
  EXPECT_EQ((-a).x, -1.0);
}

TEST(Vec3, DotAndCross) {
  const Vec3 x{1.0, 0.0, 0.0};
  const Vec3 y{0.0, 1.0, 0.0};
  const Vec3 z{0.0, 0.0, 1.0};

  EXPECT_EQ(x.dot(y), 0.0);
  EXPECT_EQ(x.dot(x), 1.0);

  // Right-handed, which is what makes the ISO 8855 frame in conventions.hpp
  // mean what it says.
  const Vec3 c = x.cross(y);
  EXPECT_EQ(c.x, z.x);
  EXPECT_EQ(c.y, z.y);
  EXPECT_EQ(c.z, z.z);
}

TEST(Vec3, Norms) {
  const Vec3 v{3.0, 4.0, 12.0};
  EXPECT_DOUBLE_EQ(v.squared_norm(), 169.0);
  EXPECT_DOUBLE_EQ(v.norm(), 13.0);
  EXPECT_DOUBLE_EQ(v.xy().norm(), 5.0);
}

TEST(Vec2, CrossIsTheScalarZComponent) {
  const Vec2 a{1.0, 0.0};
  const Vec2 b{0.0, 1.0};
  EXPECT_DOUBLE_EQ(a.cross(b), 1.0);
  EXPECT_DOUBLE_EQ(b.cross(a), -1.0);
  EXPECT_DOUBLE_EQ(a.cross(a), 0.0);
}

TEST(Mat3, IdentityAndDiagonal) {
  const Vec3 v{1.0, 2.0, 3.0};
  const Vec3 r = Mat3::identity() * v;
  EXPECT_DOUBLE_EQ(r.x, 1.0);
  EXPECT_DOUBLE_EQ(r.y, 2.0);
  EXPECT_DOUBLE_EQ(r.z, 3.0);

  const Vec3 d = Mat3::diagonal(2.0, 3.0, 4.0) * v;
  EXPECT_DOUBLE_EQ(d.x, 2.0);
  EXPECT_DOUBLE_EQ(d.y, 6.0);
  EXPECT_DOUBLE_EQ(d.z, 12.0);

  EXPECT_DOUBLE_EQ(Mat3::identity().determinant(), 1.0);
  EXPECT_DOUBLE_EQ(Mat3::diagonal(2.0, 3.0, 4.0).determinant(), 24.0);
}

TEST(Mat3, TransposeIsAnInvolution) {
  Mat3 m{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}}};
  const Mat3 t = m.transpose();
  EXPECT_DOUBLE_EQ(t.m[0][1], 4.0);
  EXPECT_DOUBLE_EQ(t.m[1][0], 2.0);
  const Mat3 tt = t.transpose();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) EXPECT_DOUBLE_EQ(tt.m[i][j], m.m[i][j]);
  }
}

TEST(RotationZ, PositiveYawTurnsBodyForwardTowardsWorldLeft) {
  // A quarter turn counter-clockwise takes body +x to world +y (CORE-17).
  const Mat3 r = slipx::rotation_z(slipx::kPi / 2.0);
  const Vec3 world = r * Vec3{1.0, 0.0, 0.0};
  EXPECT_NEAR(world.x, 0.0, 1e-15);
  EXPECT_NEAR(world.y, 1.0, 1e-15);
  EXPECT_DOUBLE_EQ(world.z, 0.0);
}

TEST(RotationZ, IsOrthonormal) {
  const Mat3 r = slipx::rotation_z(0.7);
  EXPECT_NEAR(r.determinant(), 1.0, 1e-15);
  const Vec3 v{1.3, -0.4, 2.0};
  EXPECT_NEAR((r * v).norm(), v.norm(), 1e-15);
}

TEST(WrapToPi, MapsIntoTheHalfOpenInterval) {
  EXPECT_NEAR(slipx::wrap_to_pi(0.3), 0.3, 1e-15);
  EXPECT_NEAR(slipx::wrap_to_pi(2.0 * slipx::kPi + 0.3), 0.3, 1e-14);
  EXPECT_NEAR(slipx::wrap_to_pi(-2.0 * slipx::kPi - 0.3), -0.3, 1e-14);

  // Many laps of accumulated yaw must not drift out of range, because a lap
  // counter that trusts the range is downstream of this.
  for (int k = -20; k <= 20; ++k) {
    const double a = 0.9 + 2.0 * slipx::kPi * static_cast<double>(k);
    const double w = slipx::wrap_to_pi(a);
    EXPECT_LE(w, slipx::kPi);
    EXPECT_GT(w, -slipx::kPi);
    EXPECT_NEAR(std::sin(w), std::sin(a), 1e-12);
    EXPECT_NEAR(std::cos(w), std::cos(a), 1e-12);
  }
}

TEST(ScalarUtils, ClampSignSquare) {
  EXPECT_DOUBLE_EQ(slipx::clamp(5.0, -1.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(slipx::clamp(-5.0, -1.0, 1.0), -1.0);
  EXPECT_DOUBLE_EQ(slipx::clamp(0.25, -1.0, 1.0), 0.25);

  EXPECT_DOUBLE_EQ(slipx::sign(-3.0), -1.0);
  EXPECT_DOUBLE_EQ(slipx::sign(3.0), 1.0);
  EXPECT_DOUBLE_EQ(slipx::sign(0.0), 0.0);

  EXPECT_DOUBLE_EQ(slipx::square(-3.0), 9.0);
}

TEST(Gravity, IsStandardGravityNotAnApproximation) {
  // Pinned so the analytical load-transfer and terminal-velocity tests are
  // exact rather than approximately exact.
  EXPECT_DOUBLE_EQ(slipx::kGravity, 9.80665);
}

}  // namespace
