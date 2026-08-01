// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Fixed-size linear algebra for slipx_core.
//
// This header exists because of D-02, resolved in favour of hand-rolled types.
// slipx_core's whole selling point is that embedding it costs nothing (NFR-01,
// CORE-01), and a find_package(Eigen3) in every consumer's CMakeLists is not
// nothing. The core never needs a solver, a decomposition or a dynamically
// sized matrix: L0 through L3 are small explicit expressions in two and three
// dimensions. What Eigen would contribute here is expression templates we do
// not use and an extra variable in the determinism argument (NFR-02), since
// its vectorisation and alignment behaviour is another thing that would have
// to be pinned per platform.
//
// Dependencies: <cmath>, <cstddef>. That is the complete list.

#ifndef SLIPX_MATH_HPP
#define SLIPX_MATH_HPP

#include <cmath>
#include <cstddef>

namespace slipx {

// Everything below is constexpr and trivially copyable, so a VehicleState made
// of these is a memcpy for snapshot/restore purposes (CORE-03).

struct Vec2 {
  double x{0.0};
  double y{0.0};

  constexpr Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  constexpr Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  constexpr Vec2 operator*(double s) const { return {x * s, y * s}; }
  constexpr Vec2 operator-() const { return {-x, -y}; }

  constexpr Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
  constexpr Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
  constexpr Vec2& operator*=(double s) { x *= s; y *= s; return *this; }

  constexpr double dot(const Vec2& o) const { return x * o.x + y * o.y; }
  // Scalar cross product (the z component of the 3-D cross product).
  constexpr double cross(const Vec2& o) const { return x * o.y - y * o.x; }
  double norm() const { return std::hypot(x, y); }
  constexpr double squared_norm() const { return x * x + y * y; }
};

constexpr Vec2 operator*(double s, const Vec2& v) { return v * s; }

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  constexpr Vec3 operator+(const Vec3& o) const {
    return {x + o.x, y + o.y, z + o.z};
  }
  constexpr Vec3 operator-(const Vec3& o) const {
    return {x - o.x, y - o.y, z - o.z};
  }
  constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  constexpr Vec3 operator-() const { return {-x, -y, -z}; }

  constexpr Vec3& operator+=(const Vec3& o) {
    x += o.x; y += o.y; z += o.z; return *this;
  }
  constexpr Vec3& operator-=(const Vec3& o) {
    x -= o.x; y -= o.y; z -= o.z; return *this;
  }
  constexpr Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }

  // Fixed summation order, here and everywhere else in the core: reordering a
  // floating-point reduction changes its result, and NFR-02 does not allow the
  // result to depend on which order a future edit happened to pick.
  constexpr double dot(const Vec3& o) const {
    return x * o.x + y * o.y + z * o.z;
  }
  constexpr Vec3 cross(const Vec3& o) const {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
  double norm() const { return std::sqrt(x * x + y * y + z * z); }
  constexpr double squared_norm() const { return x * x + y * y + z * z; }

  constexpr Vec2 xy() const { return {x, y}; }
};

constexpr Vec3 operator*(double s, const Vec3& v) { return v * s; }

// Row-major 3x3. Used for inertia tensors and for the body-to-world rotation.
struct Mat3 {
  // m[row][col]
  double m[3][3]{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

  static constexpr Mat3 identity() {
    return Mat3{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
  }

  static constexpr Mat3 diagonal(double a, double b, double c) {
    return Mat3{{{a, 0.0, 0.0}, {0.0, b, 0.0}, {0.0, 0.0, c}}};
  }

  constexpr Vec3 operator*(const Vec3& v) const {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
  }

  constexpr Mat3 transpose() const {
    return Mat3{{{m[0][0], m[1][0], m[2][0]},
                 {m[0][1], m[1][1], m[2][1]},
                 {m[0][2], m[1][2], m[2][2]}}};
  }

  constexpr double determinant() const {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
         - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
         + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  }
};

// Rotation about the body z axis. Yaw is positive counter-clockwise viewed
// from above, per ISO 8855 (CORE-17); see conventions.hpp.
inline Mat3 rotation_z(double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Mat3{{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

// --------------------------------------------------------------- scalar utils

constexpr double clamp(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

constexpr double sign(double v) {
  return v > 0.0 ? 1.0 : (v < 0.0 ? -1.0 : 0.0);
}

constexpr double square(double v) { return v * v; }

// Wrap to (-pi, pi]. Uses atan2 rather than fmod so the result is continuous
// and does not depend on the sign convention fmod happens to use for
// negatives.
inline double wrap_to_pi(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline constexpr double kPi = 3.14159265358979323846;

// Standard gravity [m/s^2]. Fixed, not measured: a parameter that varies by
// 0.3% with latitude is not worth making configurable, and pinning it keeps
// the analytical tests exact.
inline constexpr double kGravity = 9.80665;

}  // namespace slipx

#endif  // SLIPX_MATH_HPP
