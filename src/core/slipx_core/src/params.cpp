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

  if (!(p.accel_max > 0.0)) return "accel_max must be positive [m/s^2]";
  if (!(p.decel_max > 0.0))
    return "decel_max must be positive; it is a magnitude [m/s^2]";
  if (!(p.v_max > 0.0)) return "v_max must be positive [m/s]";

  if (!(p.steer_max > 0.0))
    return "steer_max must be positive; it is a symmetric magnitude [rad]";
  if (!(p.steer_max < 0.5 * kPi))
    return "steer_max must be below pi/2 [rad]; degrees are not SI";

  if (!(p.drag_coeff >= 0.0)) return "drag_coeff must not be negative [kg/m]";
  if (!(p.roll_resist >= 0.0)) return "roll_resist must not be negative [-]";

  // v_eps is the slip-angle denominator floor (see l1_bicycle.cpp). Zero
  // reintroduces the standstill singularity it exists to prevent.
  if (!(p.v_eps > 0.0)) return "v_eps must be positive [m/s]";

  return nullptr;
}

}  // namespace slipx
