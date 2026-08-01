// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Sign conventions and units (CORE-17, NFR-07).
//
// Sign errors and unit errors are the dominant bug class in vehicle dynamics
// code, which is why this file is normative rather than explanatory and why
// every claim in it has a corresponding assertion in
// tests/test_conventions.cpp. If the tests and this file ever disagree, the
// tests are right and one of them is a bug.
//
// ============================================================================
// Axes: ISO 8855
// ============================================================================
//
// Vehicle body frame, origin at the centre of gravity:
//
//   x  forward, along the vehicle's longitudinal axis      [m]
//   y  to the LEFT                                         [m]
//   z  up                                                  [m]
//
// This is a right-handed frame. It is not the SAE frame, which has y to the
// right and z down; a sign that looks wrong against a textbook is usually a
// textbook using SAE.
//
// World frame: x, y in the ground plane, z up. Yaw is measured from world x,
// positive counter-clockwise viewed from above, so a left turn increases yaw.
// Roll is positive right-side-down, pitch is positive nose-up, both following
// the right-hand rule about the body x and y axes respectively.
//
// Angular rates in VehicleState::rates are body-frame roll, pitch and yaw
// rate [rad/s], positive about the corresponding body axis by the right-hand
// rule.
//
// ============================================================================
// Steering
// ============================================================================
//
// The road wheel angle delta is positive for a LEFT turn (counter-clockwise
// about z). Positive delta therefore produces positive yaw rate and positive
// lateral acceleration in steady state. DriveInput::steer_cmd is the commanded
// road wheel angle, not a normalised stick position and not a servo pulse
// width; converting from either is the caller's job.
//
// ============================================================================
// Tyre slip: ISO 8855
// ============================================================================
//
// Slip angle, for a wheel whose velocity in the WHEEL-CARRIER frame has
// longitudinal component v_lon and lateral component v_lat (positive left):
//
//   alpha = atan2(v_lat, v_lon) - delta_wheel                          [rad]
//
// so alpha is positive when the wheel's velocity vector lies to the LEFT of
// the wheel plane. The resulting lateral force opposes that, which in the ISO
// frame makes it negative:
//
//   Fy = -C_alpha * alpha       (linear region, C_alpha > 0)           [N]
//
// This minus sign is the single most common place to get ISO and SAE mixed up.
// Under SAE the slip angle carries the opposite sign and the formula is
// written Fy = +C_alpha * alpha. Both describe a restoring force. SlipX is
// ISO throughout.
//
// Slip ratio, with omega the wheel speed and R_e the effective rolling radius:
//
//   kappa = (omega * R_e - v_lon) / max(|v_lon|, v_eps)                [-]
//
// positive under drive, negative under braking. Used from L2 onward; L0 and L1
// have no wheel states to compute it from.
//
// ============================================================================
// Units
// ============================================================================
//
// SI everywhere, without exception and without prefixes: metres, kilograms,
// seconds, radians, newtons, newton-metres, kilogram square metres, volts,
// amperes. Not degrees, not millimetres, not km/h. Schema files may present
// friendlier units to humans, but conversion happens in slipx_schema and never
// crosses into the core (NFR-07 requires every public parameter to name its
// unit in its doc comment; the doc coverage gate enforces it).
//
// State of charge is the one dimensionless quantity that could be mistaken for
// a percentage: it is a fraction in [0, 1].

#ifndef SLIPX_CONVENTIONS_HPP
#define SLIPX_CONVENTIONS_HPP

namespace slipx {

// Wheel ordering, fixed for every fixed-size per-wheel array in the library.
// Front/rear is by axle, left/right is by the ISO y axis, so FL is the wheel a
// passenger facing forward would call front-left.
enum WheelIndex : unsigned {
  kFrontLeft = 0,
  kFrontRight = 1,
  kRearLeft = 2,
  kRearRight = 3,
  kWheelCount = 4
};

}  // namespace slipx

#endif  // SLIPX_CONVENTIONS_HPP
