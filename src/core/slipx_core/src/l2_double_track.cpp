// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// L2: double-track with load transfer, MF-lite tyres and relaxation.
//
// Ten states: vx, vy, r, four lagged slip angles, x, y, yaw. This is the first
// tier at which CoG height, track width and tyre compound do anything, and it
// is the tier the system identification in P2 fits against.
//
// The three pieces it assembles were each built and tested on their own, and
// each is a pure function that this file calls rather than reimplements:
// load_transfer.hpp (CORE-05, ADR-0022), tyre.hpp (CORE-06, ADR-0023) and
// relaxation.hpp (CORE-07, ADR-0026).
//
// ============================================================================
// What is present that L1 does not have
// ============================================================================
//
//   Four contact patches, each with its own vertical load, slip angle and
//   friction budget. A left turn now loads the right-hand wheels, and the
//   inside wheels can lift.
//
//   A real saturation shape. MF-lite has a peak and a falling branch beyond
//   it, so the car can spin rather than sliding and recovering the moment the
//   slip angle comes back, which is what L1's clip does.
//
//   Combined slip. Longitudinal and lateral force compete for one friction
//   budget per tyre, so braking into a corner costs cornering force.
//
//   A tyre transient. Force follows a change in slip angle over a relaxation
//   length rather than instantly.
//
// ============================================================================
// What is absent, and is absent deliberately rather than forgotten
// ============================================================================
//
// This is the minimal double-track. The following are P1 requirements that have
// not landed, and every one of them is a thing the model does NOT represent:
//
//   CORE-08, the ESC. accel_cmd is converted to a longitudinal force demand
//   and clipped, exactly as at L1. There is no torque-speed curve, no current
//   limit and no regen limit distinct from the mechanical one.
//
//   CORE-09, the battery. VehicleState::soc and ::pack_v are not written by
//   this tier and there is no voltage sag.
//
//   CORE-10, the steering actuator. The road wheel angle follows the command
//   in the same step, so VehicleState::steer_rate stays zero. Commanded and
//   achieved differ only by the travel clip.
//
//   CORE-11, the differential and the drive layout. There is no diff and no
//   2WD/4WD distinction. The longitudinal demand is split between the four
//   wheels in proportion to vertical load, which is what an idealised torque
//   distribution does: it is the split that brings every wheel to its friction
//   limit at the same moment. It is not a drivetrain, and a car whose handling
//   depends on its diff is not represented here.
//
// Two further simplifications worth stating because they are invisible:
//
//   Parallel steer. Both front wheels take the same road wheel angle. Real
//   steering geometry turns the inner wheel further (Ackermann), and the
//   difference matters at large angles and low speed. There is no Ackermann
//   parameter in the schema to represent it with, and inventing one would be
//   the guess ADR-0009 refuses.
//
//   No suspension. Load transfer is quasi-static, so it arrives in the same
//   instant as the acceleration causing it (ADR-0022). Roll and pitch stay
//   zero and are L3's.
//
// ============================================================================
// Two algebraic loops, and how each is closed
// ============================================================================
//
// Vertical load depends on acceleration, acceleration depends on tyre force,
// and tyre force depends on vertical load. That loop is closed by a FIXED two
// pass evaluation: static loads, forces, resulting accelerations, then loads
// again and the final forces. Fixed rather than iterated to a tolerance,
// because a convergence loop is a place for nondeterminism to hide and the
// iteration count would become part of the trajectory (NFR-02, ADR-0027).
//
// The second loop is the longitudinal one. Slip ratio depends on wheel speed,
// wheel speed depends on the torque balance at the wheel, and that balance
// depends on the longitudinal force the slip ratio produced. It is closed by
// not having a wheel rotational state at all: the force demand is met up to the
// friction budget, and the slip ratio consistent with the force that was
// actually delivered is reported. ADR-0027 gives the reasoning and, more
// importantly, what that costs: this tier cannot represent a locked or a
// spinning wheel as a dynamic event.
//
// Validity: forward motion, as at L1, with params.v_eps flooring the speed in
// every slip denominator. Below roughly v_eps, use L0.

#include <array>
#include <cmath>

#include "models_internal.hpp"
#include "slipx/integrator.hpp"
#include "slipx/load_transfer.hpp"
#include "slipx/math.hpp"
#include "slipx/relaxation.hpp"
#include "slipx/tyre.hpp"

namespace slipx {
namespace internal {
namespace {

// Velocity-like indices first (see integrator.hpp). The lagged slip angles are
// grouped with the velocities rather than with the positions: they are forces
// in waiting, and semi-implicit Euler should have them updated before the
// position integration that consumes their effect.
enum : std::size_t {
  kVx = 0, kVy = 1, kR = 2,
  kLag0 = 3,
  kX = 7, kY = 8, kYaw = 9,
  kN = 10, kNVel = 7
};

// Everything the derivative needs that does not change within a step.
struct StepConstants {
  double delta = 0.0;
  double cos_delta = 1.0;
  double sin_delta = 0.0;
  double fx_demand = 0.0;  // total longitudinal force demanded         [N]

  MfLite tyre_front;       // one FRONT tyre, B derived at its static load
  MfLite tyre_rear;        // one REAR tyre

  // Longitudinal slip stiffness per tyre at that tyre's static load, used to
  // report the slip ratio consistent with the delivered force. Positive. The
  // two are equal today and are kept separate so an axle split, if one is ever
  // identifiable, does not have to reshape this struct.
  double c_kappa_front = 0.0;                                  //  [N per -]
  double c_kappa_rear = 0.0;                                   //  [N per -]

  double fz_nom_front = 0.0;  // static load on one front tyre           [N]
  double fz_nom_rear = 0.0;   // static load on one rear tyre            [N]
};

// Everything the diagnostics block wants and the integrator does not.
struct Forces {
  std::array<double, kWheelCount> alpha{};    // geometric slip angle  [rad]
  std::array<double, kWheelCount> kappa{};    // reported slip ratio     [-]
  std::array<double, kWheelCount> fx{};       // delivered, wheel frame  [N]
  std::array<double, kWheelCount> fy{};       //                         [N]
  std::array<double, kWheelCount> fz{};       //                         [N]
  std::array<double, kWheelCount> vx_wheel{}; // wheel centre, body x  [m/s]
  std::array<bool, kWheelCount> saturated{};
  double ax = 0.0;                            //                    [m/s^2]
  double ay = 0.0;                            //                    [m/s^2]
  double mz = 0.0;                            // yaw moment          [N m]
  double transfer_long = 0.0;                 //                        [N]
  double transfer_lat = 0.0;                  //                        [N]
};

class DoubleTrackL2 final : public VehicleModel {
 public:
  DoubleTrackL2(const VehicleParams& p, Integrator integ)
      : params_(p), integrator_(integ) {}

  Tier tier() const override { return Tier::L2_DoubleTrack; }
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
    c.fx_demand = params_.mass * accel;

    // The tyres are built once per step rather than once per derivative
    // evaluation: B is a function of the parameters and the static load only,
    // so it does not change within a step (CORE-03).
    const WheelLoads statics = static_loads(params_);
    c.fz_nom_front = statics.fz[kFrontLeft];
    c.fz_nom_rear = statics.fz[kRearLeft];

    // VehicleParams carries cornering stiffness per AXLE, because that is what
    // a single-track tier needs; MF-lite is per tyre, so each axle value is
    // halved here. Doing it in one place keeps the two conventions from
    // meeting anywhere else.
    c.tyre_front = make_mf_lite(params_.tyre_front, 0.5 * params_.c_alpha_f,
                                c.fz_nom_front);
    c.tyre_rear = make_mf_lite(params_.tyre_rear, 0.5 * params_.c_alpha_r,
                               c.fz_nom_rear);
    // One slip stiffness for all four tyres: the run that identifies it cannot
    // separate the axles. See the field's note in params.hpp.
    c.c_kappa_front = params_.c_kappa;
    c.c_kappa_rear = params_.c_kappa;

    StateVec<kN> y{};
    y[kVx] = s.vel_body.x;
    y[kVy] = s.vel_body.y;
    y[kR] = s.rates.z;
    for (unsigned i = 0; i < kWheelCount; ++i) y[kLag0 + i] = s.alpha_lag[i];
    y[kX] = s.pos.x;
    y[kY] = s.pos.y;
    y[kYaw] = s.yaw;

    const auto deriv = [&](const StateVec<kN>& q) {
      Forces f;
      return derivative(q, c, &f);
    };

    integrate<kN>(integrator_, y, dt, kNVel, deriv);

    s.vel_body.x = y[kVx];
    s.vel_body.y = y[kVy];
    s.vel_body.z = 0.0;
    s.rates = Vec3{0.0, 0.0, y[kR]};
    for (unsigned i = 0; i < kWheelCount; ++i) s.alpha_lag[i] = y[kLag0 + i];
    s.pos.x = y[kX];
    s.pos.y = y[kY];
    s.yaw = wrap_to_pi(y[kYaw]);
    s.steer = delta;
    s.steer_rate = 0.0;  // no servo model at this tier; CORE-10

    // Reported at the end of the step, so the numbers describe the state the
    // caller now holds. Same convention as L0 and L1.
    Forces f;
    const StateVec<kN> d = derivative(y, c, &f);

    // Wheel speeds follow from the reported slip ratio rather than from a
    // rotational state (ADR-0027). kappa = (omega R - v) / v, so
    // omega = v (1 + kappa) / R. These are consistent with the delivered
    // force by construction and are NOT an independent degree of freedom.
    for (unsigned i = 0; i < kWheelCount; ++i) {
      s.omega_w[i] = f.vx_wheel[i] * (1.0 + f.kappa[i]) / params_.wheel_radius;
    }
    for (unsigned i = 0; i < kWheelCount; ++i) s.Fz[i] = f.fz[i];

    if (out != nullptr) {
      reset_diagnostics(*out, Tier::L2_DoubleTrack);
      for (unsigned i = 0; i < kWheelCount; ++i) {
        out->alpha[i] = f.alpha[i];
        out->kappa[i] = f.kappa[i];
        out->fx[i] = f.fx[i];
        out->fy[i] = f.fy[i];
        out->fz[i] = f.fz[i];
        out->tyre_saturated[i] = f.saturated[i];
      }

      // Axle-resolved values, so a controller or a plot written against L1
      // keeps working. Slip angle is the mean over the axle and force is the
      // sum, which are the two that reduce to L1's single tyre correctly.
      out->alpha_front = 0.5 * (f.alpha[kFrontLeft] + f.alpha[kFrontRight]);
      out->alpha_rear = 0.5 * (f.alpha[kRearLeft] + f.alpha[kRearRight]);
      out->fy_front = f.fy[kFrontLeft] + f.fy[kFrontRight];
      out->fy_rear = f.fy[kRearLeft] + f.fy[kRearRight];
      out->fz_front = f.fz[kFrontLeft] + f.fz[kFrontRight];
      out->fz_rear = f.fz[kRearLeft] + f.fz[kRearRight];

      out->ax = d[kVx] - y[kVy] * y[kR];
      out->ay = d[kVy] + y[kVx] * y[kR];
      out->load_transfer_long = f.transfer_long;
      out->load_transfer_lat = f.transfer_lat;

      out->steer_saturated = steer_sat;
      out->accel_saturated = accel_sat;
      out->speed_saturated = speed_sat;
    }
  }

 private:
  // Wheel position in the body frame. x forward, y to the LEFT, so the left
  // wheels carry a positive y (conventions.hpp).
  void wheel_offset(unsigned i, double* x, double* y) const {
    const bool front = (i == kFrontLeft || i == kFrontRight);
    const bool left = (i == kFrontLeft || i == kRearLeft);
    *x = front ? params_.lf : -params_.lr;
    const double track = front ? params_.track_front : params_.track_rear;
    *y = left ? 0.5 * track : -0.5 * track;
  }

  // One evaluation of the tyre forces at a given set of vertical loads. Called
  // twice per derivative: see the note on the algebraic loop above.
  void evaluate_forces(const StateVec<kN>& q, const StepConstants& c,
                       const WheelLoads& loads, Forces* f) const {
    const double vx = q[kVx];
    const double vy = q[kVy];
    const double r = q[kR];

    // Per-wheel contributions, reduced below in a fixed, mirror-symmetric
    // order. Accumulating them one at a time in wheel order looks equivalent
    // and is not: floating-point addition is commutative but not associative,
    // so ((FL + FR) + RL) + RR and its left/right mirror differ in the last
    // bits, and the mirror-symmetry invariant fails by about 1e-5 relative.
    // Pairing each axle first makes the reduction exactly symmetric, because
    // each pair sum is a two-term sum and those do commute exactly (CORE-04,
    // ADR-0004: fixed reduction order in a numerical path).
    std::array<double, kWheelCount> fx_b{};
    std::array<double, kWheelCount> fy_b{};
    std::array<double, kWheelCount> mz_w{};

    for (unsigned i = 0; i < kWheelCount; ++i) {
      double xw = 0.0;
      double yw = 0.0;
      wheel_offset(i, &xw, &yw);
      const bool front = (i == kFrontLeft || i == kFrontRight);

      // Velocity of the wheel centre in the body frame. The yaw rate makes the
      // outside wheels travel faster than the inside ones, which is the whole
      // reason a double-track model differs from a bicycle.
      const double vxw = vx - r * yw;
      const double vyw = vy + r * xw;

      const double vx_safe = (vxw >= 0.0) ? std::fmax(vxw, params_.v_eps)
                                          : std::fmin(vxw, -params_.v_eps);

      // ISO 8855, and the same construction as L1: the slip angle is measured
      // between the wheel's travel and its plane, and the steered wheels have
      // their plane rotated by delta.
      const double steer_i = front ? c.delta : 0.0;
      const double alpha = std::atan2(vyw, vx_safe) - steer_i;

      const MfLite& tyre = front ? c.tyre_front : c.tyre_rear;
      const double fz = loads.fz[i];

      // The force uses the LAGGED slip angle, which is the state, while the
      // load is the instantaneous one. That asymmetry is the point of
      // ADR-0026: the force is inside the current friction budget however much
      // history the slip angle carries.
      const double fy_pure = mf_lite_fy(tyre, q[kLag0 + i], fz);

      // Longitudinal demand, split EQUALLY between the four wheels.
      //
      // Equally rather than in proportion to vertical load, and the reason is
      // worth the comment because load-proportional looks more sophisticated.
      // In a corner the outer wheels carry more load, so a load-proportional
      // split puts more thrust on the outside, and that asymmetry is a yaw
      // moment: the car turns into the corner under power. Measured on the
      // reference car it is worth about 2% of steady-state radius at 0.36 g,
      // which is larger than several effects this tier exists to represent.
      // No differential does that. An equal split produces no drive-induced
      // yaw moment on a symmetric car, which is the correct behaviour for a
      // tier that does not model a differential at all (CORE-11).
      //
      // A lifted wheel is still asked for its quarter and delivers nothing,
      // because its friction budget is zero. The demand is then simply not
      // met, which is what actually happens.
      const double fx_pure = 0.25 * c.fx_demand;

      const CombinedForce delivered =
          friction_ellipse(fx_pure, fy_pure, peak_longitudinal_force(tyre, fz),
                           peak_lateral_force(tyre, fz));

      // Wheel frame to body frame. Only the front wheels are rotated.
      const double cos_i = front ? c.cos_delta : 1.0;
      const double sin_i = front ? c.sin_delta : 0.0;
      fx_b[i] = delivered.fx * cos_i - delivered.fy * sin_i;
      fy_b[i] = delivered.fx * sin_i + delivered.fy * cos_i;

      // Yaw moment about the CoG. The second term is why a double-track model
      // yaws under asymmetric braking and a bicycle model cannot.
      mz_w[i] = xw * fy_b[i] - yw * fx_b[i];

      f->alpha[i] = alpha;
      f->fx[i] = delivered.fx;
      f->fy[i] = delivered.fy;
      f->fz[i] = fz;
      f->vx_wheel[i] = vxw;
      f->saturated[i] = delivered.saturated;

      // The slip ratio consistent with the force that was actually delivered,
      // through the linear slip stiffness at this load. Reported, not
      // integrated (ADR-0027). Load sensitivity enters the same way it does
      // for cornering stiffness: the stiffness scales as the peak force does.
      const double c_kappa_nom = front ? c.c_kappa_front : c.c_kappa_rear;
      const double fz_nom = front ? c.fz_nom_front : c.fz_nom_rear;
      const double c_kappa =
          c_kappa_nom * std::pow(fz / fz_nom, 1.0 - tyre.k_mu);
      f->kappa[i] = (c_kappa > 0.0) ? (delivered.fx / c_kappa) : 0.0;
    }

    // Axle first, then the two axles. See the note above the arrays.
    double fx_body = (fx_b[kFrontLeft] + fx_b[kFrontRight]) +
                     (fx_b[kRearLeft] + fx_b[kRearRight]);
    const double fy_body = (fy_b[kFrontLeft] + fy_b[kFrontRight]) +
                           (fy_b[kRearLeft] + fy_b[kRearRight]);
    const double mz = (mz_w[kFrontLeft] + mz_w[kFrontRight]) +
                      (mz_w[kRearLeft] + mz_w[kRearRight]);

    // Aerodynamic drag and rolling resistance, both opposing travel, and both
    // acting at the CoG rather than per wheel. Same smoothing as L1: a
    // discontinuity at zero speed makes RK4's four evaluations disagree about
    // which side of it they are on.
    const double f_drag = params_.drag_coeff * vx * std::fabs(vx);
    const double f_roll = params_.roll_resist * params_.mass * kGravity *
                          std::tanh(vx / params_.v_eps);
    fx_body -= (f_drag + f_roll);

    f->ax = fx_body / params_.mass;
    f->ay = fy_body / params_.mass;
    f->mz = mz;
    f->transfer_long = loads.transfer_long;
    f->transfer_lat = loads.transfer_lat;
  }

  StateVec<kN> derivative(const StateVec<kN>& q, const StepConstants& c,
                          Forces* f) const {
    // Pass one: static loads, to get an acceleration to transfer load with.
    Forces first;
    evaluate_forces(q, c, static_loads(params_), &first);

    // Pass two: the loads those accelerations imply, and the forces they give.
    // Two passes, always, whatever the residual (ADR-0027).
    const WheelLoads loads = quasi_static_loads(params_, first.ax, first.ay);
    evaluate_forces(q, c, loads, f);

    StateVec<kN> d{};
    d[kVx] = f->ax + q[kVy] * q[kR];
    d[kVy] = f->ay - q[kVx] * q[kR];
    d[kR] = f->mz / params_.izz;

    for (unsigned i = 0; i < kWheelCount; ++i) {
      const bool front = (i == kFrontLeft || i == kFrontRight);
      const double sigma = front ? params_.tyre_front.relax_length
                                 : params_.tyre_rear.relax_length;
      d[kLag0 + i] = relaxation_rate(f->alpha[i], q[kLag0 + i],
                                     f->vx_wheel[i], sigma);
    }

    const double cos_yaw = std::cos(q[kYaw]);
    const double sin_yaw = std::sin(q[kYaw]);
    d[kX] = q[kVx] * cos_yaw - q[kVy] * sin_yaw;
    d[kY] = q[kVx] * sin_yaw + q[kVy] * cos_yaw;
    d[kYaw] = q[kR];

    return d;
  }

  VehicleParams params_;
  Integrator integrator_;
};

}  // namespace

std::unique_ptr<VehicleModel> make_l2(const VehicleParams& p, Integrator i) {
  return std::unique_ptr<VehicleModel>(new DoubleTrackL2(p, i));
}

}  // namespace internal
}  // namespace slipx
