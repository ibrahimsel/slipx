// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// L1: dynamic bicycle with linear tyres. Six states: vx, vy, r, x, y, yaw.
//
// What is present: sideslip, yaw dynamics, a real understeer gradient, the
// step-steer transient, and lateral force proportional to slip angle,
// Fy = -C_alpha * alpha (ISO sign; see conventions.hpp).
//
// What is absent, and why the tier below L2 cannot carry the product claim:
//
//   No saturation shape. Lateral force is linear in slip angle and is then
//   CLIPPED at mu * Fz. A clip is not a Magic Formula: there is no peak, no
//   falling branch beyond the peak, and therefore no mechanism by which the
//   car spins. It slides at the limit and recovers as soon as the slip angle
//   comes back. Real cars do not do this.
//
//   No load transfer. Axle vertical loads are the static ones, so CoG height
//   is inert (CORE-05 is a P1/L2 requirement).
//
//   No wheel states, so no slip ratio, no differential and no combined slip.
//   Longitudinal and lateral force do not compete for the same friction.
//
// The clip is deliberately visible rather than smoothed: StepDiagnostics
// reports tyre_saturated the moment it engages, so the point at which L1 stops
// being believable is a number the user can plot rather than a feeling they
// develop. That crossover against L2 is the tracked cross-tier artefact
// (SRS 7).
//
// Validity: forward motion. The slip-angle definition degenerates in reverse
// and near standstill, where the velocity direction is not well determined by
// a velocity of nearly zero. The mitigation is params.v_eps, a floor on the
// speed appearing in the slip-angle denominators; it is a parameter rather
// than a hidden constant because it changes the model and therefore belongs
// in the manifest hash. Below roughly v_eps, use L0.

#include <cmath>

#include "models_internal.hpp"
#include "slipx/integrator.hpp"
#include "slipx/math.hpp"

namespace slipx {
namespace internal {
namespace {

// Velocity-like indices first (see integrator.hpp).
enum : std::size_t {
  kVx = 0, kVy = 1, kR = 2, kX = 3, kY = 4, kYaw = 5, kN = 6, kNVel = 3
};

// Everything the derivative needs that does not change within a step.
struct StepConstants {
  double delta = 0.0;
  double cos_delta = 1.0;
  double sin_delta = 0.0;
  double fx_drive = 0.0;   // longitudinal force from the drivetrain    [N]
  double fz_front = 0.0;   // static axle load                          [N]
  double fz_rear = 0.0;    // static axle load                          [N]
  double fy_front_max = 0.0;  // friction clip magnitude                [N]
  double fy_rear_max = 0.0;   //                                        [N]
};

// Outputs of the force model that the diagnostics block wants but the
// integrator does not.
struct Forces {
  double alpha_f = 0.0;
  double alpha_r = 0.0;
  double fy_f = 0.0;
  double fy_r = 0.0;
  double f_resist = 0.0;   // drag + rolling, opposing travel           [N]
  bool front_saturated = false;
  bool rear_saturated = false;
};

class BicycleL1 final : public VehicleModel {
 public:
  BicycleL1(const VehicleParams& p, Integrator integ)
      : params_(p), integrator_(integ) {}

  Tier tier() const override { return Tier::L1_Bicycle; }
  const VehicleParams& params() const override { return params_; }
  Integrator integrator() const override { return integrator_; }
  std::size_t state_dimension() const override { return kN; }

  void step(VehicleState& s, const DriveInput& u, double dt,
            StepDiagnostics* out) const override {
    const double delta = clamp(u.steer_cmd, -params_.steer_max,
                               params_.steer_max);
    const bool steer_sat = (delta != u.steer_cmd);

    double accel = clamp(u.accel_cmd, -params_.decel_max, params_.accel_max);
    const bool accel_sat = (accel != u.accel_cmd);

    bool speed_sat = false;
    if (s.vel_body.x >= params_.v_max && accel > 0.0) {
      accel = 0.0;
      speed_sat = true;
    }

    StepConstants c;
    c.delta = delta;
    c.cos_delta = std::cos(delta);
    c.sin_delta = std::sin(delta);
    c.fx_drive = params_.mass * accel;

    const double w = params_.wheelbase();
    c.fz_front = params_.mass * kGravity * params_.lr / w;
    c.fz_rear = params_.mass * kGravity * params_.lf / w;
    c.fy_front_max = params_.mu_clip * c.fz_front;
    c.fy_rear_max = params_.mu_clip * c.fz_rear;

    StateVec<kN> y{s.vel_body.x, s.vel_body.y, s.rates.z,
                   s.pos.x, s.pos.y, s.yaw};

    const auto deriv = [&](const StateVec<kN>& q) {
      Forces f;
      return derivative(q, c, &f);
    };

    integrate<kN>(integrator_, y, dt, kNVel, deriv);

    s.vel_body.x = y[kVx];
    s.vel_body.y = y[kVy];
    s.vel_body.z = 0.0;
    s.rates = Vec3{0.0, 0.0, y[kR]};
    s.pos.x = y[kX];
    s.pos.y = y[kY];
    s.yaw = wrap_to_pi(y[kYaw]);
    s.steer = delta;
    s.steer_rate = 0.0;

    if (out != nullptr) {
      // Reported at the end of the step, so the numbers describe the state
      // the caller now holds rather than the one it handed in. This is the
      // convention across every tier: diagnostics and state agree.
      Forces f;
      const StateVec<kN> d = derivative(y, c, &f);

      reset_diagnostics(*out, Tier::L1_Bicycle);
      out->alpha_front = f.alpha_f;
      out->alpha_rear = f.alpha_r;
      out->fy_front = f.fy_f;
      out->fy_rear = f.fy_r;
      out->fz_front = c.fz_front;
      out->fz_rear = c.fz_rear;
      // Specific forces, including the transport terms: what an IMU reads.
      out->ax = d[kVx] - y[kVy] * y[kR];
      out->ay = d[kVy] + y[kVx] * y[kR];
      out->tyre_saturated[kFrontLeft] = f.front_saturated;
      out->tyre_saturated[kFrontRight] = f.front_saturated;
      out->tyre_saturated[kRearLeft] = f.rear_saturated;
      out->tyre_saturated[kRearRight] = f.rear_saturated;
      out->steer_saturated = steer_sat;
      out->accel_saturated = accel_sat;
      out->speed_saturated = speed_sat;
    }
  }

 private:
  // The whole model, in one pure function of state and step constants.
  StateVec<kN> derivative(const StateVec<kN>& q, const StepConstants& c,
                          Forces* f) const {
    const double vx = q[kVx];
    const double vy = q[kVy];
    const double r = q[kR];

    // Speed floor in the slip-angle denominator only. It never enters the
    // momentum equations, so it cannot manufacture or destroy energy; it only
    // stops the velocity direction becoming arbitrary as the speed goes to
    // zero.
    const double vx_safe = (vx >= 0.0) ? std::fmax(vx, params_.v_eps)
                                       : std::fmin(vx, -params_.v_eps);

    // ISO 8855: alpha is positive when the wheel's velocity lies to the left
    // of the wheel plane, and the force opposes it, hence the minus sign
    // below. See conventions.hpp; this is the sign that separates ISO from
    // SAE and it is asserted in test_conventions.cpp.
    const double alpha_f = std::atan2(vy + params_.lf * r, vx_safe) - c.delta;
    const double alpha_r = std::atan2(vy - params_.lr * r, vx_safe);

    const double fy_f_linear = -params_.c_alpha_f * alpha_f;
    const double fy_r_linear = -params_.c_alpha_r * alpha_r;
    const double fy_f = clamp(fy_f_linear, -c.fy_front_max, c.fy_front_max);
    const double fy_r = clamp(fy_r_linear, -c.fy_rear_max, c.fy_rear_max);

    // Aerodynamic drag and rolling resistance, both opposing travel. The
    // rolling term is smoothed through tanh over v_eps rather than switched on
    // sign(vx): a discontinuity at zero speed makes RK4's four evaluations
    // disagree about which side of it they are on, and the resulting step
    // depends on the step size in a way that is not a discretisation error.
    const double f_drag = params_.drag_coeff * vx * std::fabs(vx);
    const double f_roll = params_.roll_resist * params_.mass * kGravity *
                          std::tanh(vx / params_.v_eps);
    const double f_resist = f_drag + f_roll;

    StateVec<kN> d{};
    d[kVx] = (c.fx_drive - fy_f * c.sin_delta - f_resist) / params_.mass
             + vy * r;
    d[kVy] = (fy_f * c.cos_delta + fy_r) / params_.mass - vx * r;
    d[kR] = (params_.lf * fy_f * c.cos_delta - params_.lr * fy_r) / params_.izz;

    const double cos_yaw = std::cos(q[kYaw]);
    const double sin_yaw = std::sin(q[kYaw]);
    d[kX] = vx * cos_yaw - vy * sin_yaw;
    d[kY] = vx * sin_yaw + vy * cos_yaw;
    d[kYaw] = r;

    if (f != nullptr) {
      f->alpha_f = alpha_f;
      f->alpha_r = alpha_r;
      f->fy_f = fy_f;
      f->fy_r = fy_r;
      f->f_resist = f_resist;
      f->front_saturated = (fy_f != fy_f_linear);
      f->rear_saturated = (fy_r != fy_r_linear);
    }
    return d;
  }

  VehicleParams params_;
  Integrator integrator_;
};

}  // namespace

std::unique_ptr<VehicleModel> make_l1(const VehicleParams& p, Integrator i) {
  return std::unique_ptr<VehicleModel>(new BicycleL1(p, i));
}

}  // namespace internal
}  // namespace slipx
