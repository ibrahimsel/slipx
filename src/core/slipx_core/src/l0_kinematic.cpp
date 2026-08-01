// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// L0: kinematic bicycle. Four states: x, y, yaw, v.
//
// What is present: Ackermann geometry, and a velocity vector aligned with
// where the wheels point.
//
// What is absent: everything else. There are no tyres, so there is no slip,
// no saturation and no spin. Mass, yaw inertia, CoG height, cornering
// stiffness, differential and tyre compound have NO EFFECT on the trajectory
// this tier produces, and that is the specified behaviour rather than an
// omission (SRS 2.4). A student who changes the mass here and sees an
// identical trajectory has learned what the tier is; a version of L0 that
// responded to mass would be teaching them something false.
//
// The one consequence worth stating for users: this model cannot leave the
// friction circle because it has no friction circle. It will corner at 3 g
// without complaint. Use it for MPC internal prediction and mass RL rollouts,
// where that is a fair trade for four states and no singularity at standstill,
// and not for anything that claims to say what a real car would do.

#include <cmath>
#include <limits>

#include "models_internal.hpp"
#include "slipx/integrator.hpp"
#include "slipx/math.hpp"

namespace slipx {
namespace internal {
namespace {

// State vector layout. Velocity-like indices first (see integrator.hpp).
enum : std::size_t { kV = 0, kX = 1, kY = 2, kYaw = 3, kN = 4, kNVel = 1 };

class KinematicL0 final : public VehicleModel {
 public:
  KinematicL0(const VehicleParams& p, Integrator integ)
      : params_(p), integrator_(integ) {}

  Tier tier() const override { return Tier::L0_Kinematic; }
  const VehicleParams& params() const override { return params_; }
  Integrator integrator() const override { return integrator_; }
  std::size_t state_dimension() const override { return kN; }

  void step(VehicleState& s, const DriveInput& u, double dt,
            StepDiagnostics* out) const override {
    const double delta = clamp(u.steer_cmd, -params_.steer_max,
                               params_.steer_max);
    const bool steer_sat = (delta != u.steer_cmd);

    // Signed speed along the velocity vector. Reconstructed from the body
    // velocity rather than carried as a separate field, so that a caller can
    // hand this model a state that an L1 or L2 model produced and have it
    // mean the same thing (SRS 2.4: one state type across tiers).
    const double v_in = std::copysign(s.vel_body.xy().norm(), s.vel_body.x);

    double accel = clamp(u.accel_cmd, -params_.decel_max, params_.accel_max);
    const bool accel_sat = (accel != u.accel_cmd);

    // Top speed as a hard clip on the demand rather than as a drag term:
    // a drag term would make the trajectory depend on mass, which is exactly
    // what this tier is defined not to do.
    bool speed_sat = false;
    if (v_in >= params_.v_max && accel > 0.0) {
      accel = 0.0;
      speed_sat = true;
    } else if (v_in <= -params_.v_max && accel < 0.0) {
      accel = 0.0;
      speed_sat = true;
    }

    // Sideslip is geometric here, fixed by the steer angle alone.
    const double beta =
        std::atan2(params_.lr * std::tan(delta), params_.wheelbase());
    const double cos_beta = std::cos(beta);
    const double tan_delta = std::tan(delta);
    const double inv_wheelbase = 1.0 / params_.wheelbase();

    StateVec<kN> y{v_in, s.pos.x, s.pos.y, s.yaw};

    const auto deriv = [&](const StateVec<kN>& q) {
      StateVec<kN> d{};
      d[kV] = accel;
      d[kX] = q[kV] * std::cos(q[kYaw] + beta);
      d[kY] = q[kV] * std::sin(q[kYaw] + beta);
      d[kYaw] = q[kV] * cos_beta * tan_delta * inv_wheelbase;
      return d;
    };

    integrate<kN>(integrator_, y, dt, kNVel, deriv);

    const double v = y[kV];
    const double yaw_rate = v * cos_beta * tan_delta * inv_wheelbase;

    s.pos.x = y[kX];
    s.pos.y = y[kY];
    s.yaw = wrap_to_pi(y[kYaw]);
    s.vel_body.x = v * cos_beta;
    s.vel_body.y = v * std::sin(beta);
    s.vel_body.z = 0.0;
    s.rates = Vec3{0.0, 0.0, yaw_rate};
    s.steer = delta;
    s.steer_rate = 0.0;

    if (out != nullptr) {
      reset_diagnostics(*out, Tier::L0_Kinematic);
      // The two specific forces are real: a body accelerating along a curved
      // path does experience these, and an IMU mounted on a car following
      // this trajectory would read them. Everything downstream of a tyre
      // stays NaN, because there is no tyre.
      out->ax = accel;
      out->ay = v * yaw_rate;
      out->steer_saturated = steer_sat;
      out->accel_saturated = accel_sat;
      out->speed_saturated = speed_sat;
    }
  }

 private:
  VehicleParams params_;
  Integrator integrator_;
};

}  // namespace

std::unique_ptr<VehicleModel> make_l0(const VehicleParams& p, Integrator i) {
  return std::unique_ptr<VehicleModel>(new KinematicL0(p, i));
}

}  // namespace internal
}  // namespace slipx
