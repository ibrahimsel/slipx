// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Physical sanity checks on VehicleParams.
//
// The bar here is "this parameter set describes no object that could exist",
// not "this parameter set describes a legal RoboRacer car". The second
// question is slipx_schema's (SCH-03, SCH-04) and the core is not allowed to
// know that slipx_schema exists (CORE-01). The division matters: somebody
// embedding the core in their own simulator gets these checks and none of the
// competition rules, which is what they want.
//
// Every message is a string literal, so this allocates nothing and compiles
// under -fno-exceptions.

#include <cmath>

#include "slipx/params.hpp"

namespace slipx {

namespace {

// One tyre's coefficients. The message does not name the axle, because a
// string literal cannot be built at run time and this file allocates nothing;
// a caller who needs to know which axle has two fields to look at.
const char* validate_tyre(const TyreCoefficients& t) {
  if (!(t.mu_y0 > 0.0)) return "tyre mu_y0 must be positive [-]";
  if (!(t.mu_x0 > 0.0)) return "tyre mu_x0 must be positive [-]";
  if (!(t.k_mu >= 0.0)) return "tyre k_mu must not be negative [-]";
  // Zero relaxation length is a division by zero in relaxation_rate, not an
  // instantaneous tyre. A caller who wants no transient wants a tier below L2.
  if (!(t.relax_length > 0.0)) return "tyre relax_length must be positive [m]";
  // Below 1 the Magic Formula has no peak, which is not a tyre; above 1 the
  // curvature factor folds the curve back on itself.
  if (!(t.shape_c > 1.0)) return "tyre shape_c must exceed 1 [-]";
  if (!(t.curvature_e <= 1.0)) return "tyre curvature_e must not exceed 1 [-]";
  return nullptr;
}

}  // namespace

const char* validate(const VehicleParams& p) {
  if (!(p.mass > 0.0)) return "mass must be positive [kg]";
  if (!(p.izz > 0.0)) return "izz must be positive [kg m^2]";
  if (!(p.ixx > 0.0)) return "ixx must be positive [kg m^2]";
  if (!(p.iyy > 0.0)) return "iyy must be positive [kg m^2]";

  // lf and lr are distances from the CoG to each axle, so both are positive
  // and a zero wheelbase divides by zero in every steering equation.
  if (!(p.lf > 0.0)) return "lf must be positive [m]";
  if (!(p.lr > 0.0)) return "lr must be positive [m]";
  if (!(p.track_front > 0.0)) return "track_front must be positive [m]";
  if (!(p.track_rear > 0.0)) return "track_rear must be positive [m]";
  if (!(p.h_cog > 0.0)) return "h_cog must be positive [m]";
  if (!(p.wheel_radius > 0.0)) return "wheel_radius must be positive [m]";

  // A CoG below the wheel centres is possible on a real car but not on this
  // one: at 1/10 scale the battery and the electronics sit on the deck, above
  // the axle line. Rejected because it is far more often a units error
  // (millimetres entered as metres) than a real chassis.
  if (!(p.h_cog < 1.0)) return "h_cog implausibly high; SI units are metres";

  if (!(p.c_alpha_f > 0.0)) return "c_alpha_f must be positive [N/rad]";
  if (!(p.c_alpha_r > 0.0)) return "c_alpha_r must be positive [N/rad]";
  if (!(p.mu_clip > 0.0)) return "mu_clip must be positive [-]";

  // The MF-lite block, used from L2. Checked here rather than only in the
  // schema because the core is reachable without the schema (CORE-01), and
  // every one of these divides or exponentiates something.
  if (!(p.c_kappa > 0.0)) return "c_kappa must be positive [N per unit slip]";
  if (const char* why = validate_tyre(p.tyre_front)) return why;
  if (const char* why = validate_tyre(p.tyre_rear)) return why;

  if (!(p.accel_max > 0.0)) return "accel_max must be positive [m/s^2]";
  if (!(p.decel_max > 0.0))
    return "decel_max must be positive; it is a magnitude [m/s^2]";
  if (!(p.v_max > 0.0)) return "v_max must be positive [m/s]";

  if (!(p.lsd_preload >= 0.0)) return "lsd_preload must not be negative [N m]";

  // The ESC and battery block, used from L2. Every one of these divides
  // something or scales a torque, so a zero or a sign error here is a NaN or
  // a backwards car three frames later.
  if (!(p.torque_stall > 0.0)) return "torque_stall must be positive [N m]";
  if (!(p.omega_free > 0.0)) return "omega_free must be positive [rad/s]";
  if (!(p.torque_per_amp > 0.0))
    return "torque_per_amp must be positive [N m/A]";
  if (!(p.drive_efficiency > 0.0 && p.drive_efficiency <= 1.0))
    return "drive_efficiency must be in (0, 1] [-]";
  if (!(p.current_max > 0.0)) return "current_max must be positive [A]";
  if (!(p.regen_current_max >= 0.0))
    return "regen_current_max must not be negative [A]";

  if (!(p.pack_nominal_v > 0.0)) return "pack_nominal_v must be positive [V]";
  if (!(p.pack_capacity_ah > 0.0))
    return "pack_capacity_ah must be positive [A h]";
  if (!(p.pack_internal_resistance >= 0.0))
    return "pack_internal_resistance must not be negative [ohm]";
  // Non-strict on purpose: pack_v_full = pack_v_empty = pack_nominal_v with
  // zero internal resistance is the ideal-supply configuration, which is both
  // a legitimate model and the no-battery test fixture (ADR-0031).
  if (!(p.pack_v_empty > 0.0)) return "pack_v_empty must be positive [V]";
  if (!(p.pack_v_empty <= p.pack_nominal_v && p.pack_nominal_v <= p.pack_v_full))
    return "pack voltages must be ordered: v_empty <= v_nominal <= v_full [V]";

  if (!(p.steer_max > 0.0))
    return "steer_max must be positive; it is a symmetric magnitude [rad]";
  if (!(p.steer_max < 0.5 * kPi))
    return "steer_max must be below pi/2 [rad]; degrees are not SI";
  if (!(p.steer_rate_max > 0.0))
    return "steer_rate_max must be positive [rad/s]";
  if (!(p.steer_bandwidth > 0.0))
    return "steer_bandwidth must be positive [rad/s]";
  if (!(p.steer_damping > 0.0))
    return "steer_damping must be positive [-]";

  if (!(p.drag_coeff >= 0.0)) return "drag_coeff must not be negative [kg/m]";
  if (!(p.roll_resist >= 0.0)) return "roll_resist must not be negative [-]";

  // v_eps is the slip-angle denominator floor (see l1_bicycle.cpp). Zero
  // reintroduces the standstill singularity it exists to prevent.
  if (!(p.v_eps > 0.0)) return "v_eps must be positive [m/s]";

  return nullptr;
}

}  // namespace slipx
