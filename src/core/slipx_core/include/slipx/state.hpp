// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// VehicleState, DriveInput and StepDiagnostics.
//
// VehicleState is deliberately one struct for every tier rather than a variant
// or a tier-specific type. Two reasons, both load-bearing:
//
//   1. It is trivially copyable and fixed size, so snapshot and restore is a
//      memcpy (CORE-03, SIM-08) and a replay buffer is an array.
//   2. A controller written against L1 can be pointed at L2 without changing
//      a line, which is the cross-tier teaching experience (SRS 2.4). If the
//      state type changed with the tier, that experiment would need a code
//      change and nobody would run it.
//
// The cost is that a lower tier leaves fields it cannot represent untouched.
// That is documented per field below rather than hidden.

#ifndef SLIPX_STATE_HPP
#define SLIPX_STATE_HPP

#include <array>

#include "slipx/conventions.hpp"
#include "slipx/math.hpp"

namespace slipx {

// ISO 8855 conventions, SI units. See conventions.hpp, which is normative.
struct VehicleState {
  Vec3 pos{};                // world position of the CoG               [m]
  double yaw = 0.0;          // heading, positive counter-clockwise   [rad]
  double pitch = 0.0;        // positive nose-up. L3 only.            [rad]
  double roll = 0.0;         // positive right-side-down. L3 only.    [rad]

  Vec3 vel_body{};           // velocity in the body frame:
                             // x forward, y left, z up              [m/s]
  Vec3 rates{};              // body roll, pitch and yaw rate. Only z
                             // is a real degree of freedom below L3.
                             //                                     [rad/s]

  std::array<double, kWheelCount> omega_w{};  // wheel speeds, positive
                             // forward. L2 onward; L0 and L1 have no
                             // wheel states and leave these at zero.
                             //                                     [rad/s]

  double steer = 0.0;        // ACHIEVED road wheel angle, positive
                             // left. Equals the command at L0 and L1,
                             // which have no servo model.            [rad]
  double steer_rate = 0.0;   // L2 onward (CORE-10).                [rad/s]

  double soc = 1.0;          // battery state of charge, fraction in
                             // [0, 1]. L2 onward (CORE-09).            [-]
  double pack_v = 11.1;      // pack terminal voltage. L2 onward.       [V]

  std::array<double, kWheelCount> Fz{};  // vertical tyre loads,
                             // diagnostic. L2 onward; below that
                             // there is no load transfer to report.    [N]

  // Lagged slip angle per wheel: the slip the tyre carcass has actually built
  // up, which trails the slip the geometry asks for by the relaxation length
  // (CORE-07, ADR-0026). L2 onward; L0 and L1 have no tyre transient and leave
  // these at zero.
  //
  // Zero rather than NaN, unlike the diagnostics. ADR-0006's rule is about
  // reported quantities, where a plausible zero would be believed. This is
  // hashed state, and hash.hpp treats a NaN in a trajectory as evidence the run
  // is already broken, so a tier that parks NaN here would poison every hash it
  // produced.
  std::array<double, kWheelCount> alpha_lag{};  //                    [rad]

  // Convenience accessors. Named rather than commented so that a reader of
  // calling code does not have to remember that rates.z is the yaw rate.
  double yaw_rate() const { return rates.z; }
  double& yaw_rate() { return rates.z; }
  double vx() const { return vel_body.x; }
  double vy() const { return vel_body.y; }
  double speed() const { return vel_body.xy().norm(); }
  // Body slip angle: the wedge between where the car points and where it is
  // actually going. Positive when the velocity vector is to the left of the
  // vehicle x axis.                                                  [rad]
  double sideslip() const { return std::atan2(vel_body.y, vel_body.x); }
};

// Commanded, pre-actuator. What the driver or controller asked for, not what
// the car achieved; the difference is the actuator model's output and, from
// L2, is visible as VehicleState::steer versus DriveInput::steer_cmd.
struct DriveInput {
  double steer_cmd = 0.0;    // road wheel angle, positive left       [rad]
  double accel_cmd = 0.0;    // longitudinal acceleration demand,
                             // positive forward. From L2 this becomes
                             // a torque demand through the ESC model
                             // (CORE-08); at L0 and L1 it is applied
                             // directly, subject to accel_max and
                             // decel_max.                          [m/s^2]
};

// Optional per-step diagnostics (CORE-12). Passing nullptr costs nothing, so
// the hot path stays cheap; passing a pointer gives a student the numbers they
// need to plot exactly why the car spun.
//
// Quantities a tier cannot represent are set to NaN, not to zero. Zero is a
// plausible slip angle and would be silently believed; NaN is loud. A plot of
// L0 slip angles is empty, which is the correct answer to the question.
struct StepDiagnostics {
  std::array<double, kWheelCount> alpha{};  // slip angle, ISO sign  [rad]
  std::array<double, kWheelCount> kappa{};  // slip ratio              [-]
  std::array<double, kWheelCount> fx{};     // longitudinal tyre force [N]
  std::array<double, kWheelCount> fy{};     // lateral tyre force      [N]
  std::array<double, kWheelCount> fz{};     // vertical tyre load      [N]
  // Per-wheel saturation. A bool has no NaN, so at the single-track tiers,
  // where there is one tyre per axle, both wheels of an axle carry that
  // axle's flag. The per-wheel float arrays above stay NaN there rather than
  // duplicating a value the tier did not compute.
  std::array<bool, kWheelCount> tyre_saturated{};

  // Axle-resolved values, which is what a single-track tier can actually
  // report. At L2 these are the sums over each axle.
  double alpha_front = 0.0;  //                                      [rad]
  double alpha_rear = 0.0;   //                                      [rad]
  double fy_front = 0.0;     //                                        [N]
  double fy_rear = 0.0;      //                                        [N]
  double fz_front = 0.0;     // front axle vertical load                [N]
  double fz_rear = 0.0;      // rear axle vertical load                 [N]

  // Specific forces at the CoG, in the body frame. These are what an ideal
  // accelerometer at the CoG would read, which is the quantity a validation
  // report compares against a real IMU trace, so they include the vy*r and
  // vx*r transport terms rather than being the raw time derivatives of the
  // body velocities.
  double ax = 0.0;           //                                     [m/s^2]
  double ay = 0.0;           //                                     [m/s^2]

  // Load transfer components, reported separately so a student can see which
  // one dominates. NaN below L2, which has no mechanism to transfer load
  // (CORE-05, P1). NaN rather than zero: zero is a number somebody would
  // plot and believe.
  double load_transfer_long = 0.0;   // front-to-rear                   [N]
  double load_transfer_lat = 0.0;    // left-to-right                   [N]

  bool steer_saturated = false;   // command clipped to steer_max
  bool accel_saturated = false;   // demand clipped to accel/decel_max
  bool speed_saturated = false;   // demand clipped by v_max

  // Which tier produced these numbers. Present so a plot cannot be mislabelled
  // after the fact.
  int tier = 0;
};

}  // namespace slipx

#endif  // SLIPX_STATE_HPP
