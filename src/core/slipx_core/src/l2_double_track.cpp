// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// L2: double-track with load transfer, MF-lite tyres, relaxation, and the
// drivetrain: differential, ESC, battery and steering servo (ADR-0031).
//
// Thirteen states: vx, vy, r, four lagged slip angles, steer_rate, soc, x, y,
// yaw, steer. This is the first tier at which CoG height, track width, tyre
// compound, differential type and battery state do anything, and it is the
// tier the system identification in P2 fits against.
//
// The pieces it assembles were each built and tested on their own, and each is
// a pure function this file calls rather than reimplements: load_transfer.hpp
// (ADR-0022), tyre.hpp (ADR-0023) and relaxation.hpp (ADR-0026). The
// drivetrain and actuators are this file's own, and ADR-0031 is their record.
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
//   A drivetrain. The drive layout selects the driven axle (or both), the
//   differential splits torque within it (open, spool or preloaded LSD), and
//   braking goes through the driven axle only, because a 1/10-scale car
//   brakes through its motor and has no friction brakes.
//
//   An ESC and a battery. Available torque follows a torque-speed curve
//   scaled by actual pack voltage, capped by the current limit; braking
//   torque is capped by the regen limit, which is usually the operative
//   brake limit and is weak. Pack voltage sags under load through the
//   internal resistance, and state of charge decays with the current drawn.
//
//   A steering servo. The road wheel angle follows the command through a
//   slew-limited second-order lag, so commanded and achieved steer differ
//   and both are visible (DriveInput::steer_cmd against VehicleState::steer).
//
// ============================================================================
// What is absent, and is absent deliberately rather than forgotten
// ============================================================================
//
//   No wheel rotational state (ADR-0027). Wheel speeds and slip ratios are
//   reported quasi-statically from the delivered forces, so a locked or a
//   spinning wheel is not a dynamic event this tier can represent. When the
//   friction ellipse caps one wheel of a spool axle, the reported wheel
//   speeds break the shared-speed constraint in favour of force consistency;
//   the force is the trajectory-authoritative quantity.
//
//   No low-voltage cutoff. soc clamps at zero and the curve keeps evaluating
//   at pack_v_empty. A real ESC cuts drive earlier; the cutoff is firmware
//   configuration, not something identifiable from driving, so it is absent
//   rather than guessed.
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
// The algebraic loops, and how each is closed
// ============================================================================
//
// Vertical load depends on acceleration, acceleration depends on tyre force,
// and tyre force depends on vertical load. That loop is closed by a FIXED two
// pass evaluation: static loads, forces, resulting accelerations, then loads
// again and the final forces. Fixed rather than iterated to a tolerance,
// because a convergence loop is a place for nondeterminism to hide and the
// iteration count would become part of the trajectory (ADR-0027).
//
// The longitudinal loop is closed by not having a wheel rotational state at
// all: the force demand is met up to the friction budget, and the slip ratio
// consistent with the force that was actually delivered is reported.
//
// The electrical loop (torque draws current, current sags the pack, the
// sagged pack rescales the torque curve) is closed by a fixed two passes of
// its own, once per step at the entry state: pass one takes the open-circuit
// voltage and computes the electrical power, pass two solves the terminal
// voltage in closed form and re-evaluates the curve there (ADR-0031). The
// torque budget is therefore a per-step constant, like the clipped command,
// and the derivative stays a pure function.
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

// Velocity-like indices first (see integrator.hpp). The lagged slip angles
// are grouped with the velocities rather than with the positions: they are
// forces in waiting, and semi-implicit Euler should have them updated before
// the position integration that consumes their effect. steer_rate is
// velocity-like and steer position-like, so the servo pair integrates in the
// same order as vx and x; soc's derivative is a per-step constant and sits
// with the velocities so the position block stays contiguous.
enum : std::size_t {
  kVx = 0, kVy = 1, kR = 2,
  kLag0 = 3,
  kSteerRate = 7, kSoc = 8,
  kX = 9, kY = 10, kYaw = 11, kSteer = 12,
  kN = 13, kNVel = 9
};

// Everything the derivative needs that does not change within a step.
struct StepConstants {
  double delta_cmd = 0.0;  // travel-clipped steering command         [rad]
  double fx_demand = 0.0;  // longitudinal force demand after the ESC
                           // and battery limits (ADR-0031)             [N]
  double front_share = 0.0;  // fraction of fx_demand to each axle,
  double rear_share = 0.0;   // set by the drive layout                 [-]
  double soc_rate = 0.0;   // d(soc)/dt, a per-step constant          [1/s]

  MfLite tyre_front;       // one FRONT tyre, B derived at its static load
  MfLite tyre_rear;        // one REAR tyre

  // Longitudinal slip stiffness per tyre at that tyre's static load, used to
  // report the slip ratio consistent with the delivered force and to close
  // the spool's constrained-speed form. Positive. The two are equal today and
  // are kept separate so an axle split, if one is ever identifiable, does not
  // have to reshape this struct.
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

// What one force pass knows about a wheel before the differential splits the
// longitudinal demand.
struct WheelPre {
  double fy_pure = 0.0;   // lateral demand from the lagged slip     [N]
  double fx_max = 0.0;    // friction budget axes at this load       [N]
  double fy_max = 0.0;    //                                         [N]
  double c_kappa = 0.0;   // slip stiffness at this load       [N per -]
  double vx_safe = 0.0;   // floored wheel-centre speed            [m/s]
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
    const double delta_cmd = clamp(u.steer_cmd, -params_.steer_max,
                                   params_.steer_max);
    const bool steer_sat = (delta_cmd != u.steer_cmd);

    double accel = clamp(u.accel_cmd, -params_.decel_max, params_.accel_max);
    const bool accel_sat = (accel != u.accel_cmd);

    bool speed_sat = false;
    if (s.vel_body.x >= params_.v_max && accel > 0.0) {
      accel = 0.0;
      speed_sat = true;
    }

    StepConstants c;
    c.delta_cmd = delta_cmd;
    switch (params_.layout) {
      case DriveLayout::kFrontWheelDrive:
        c.front_share = 1.0;
        c.rear_share = 0.0;
        break;
      case DriveLayout::kRearWheelDrive:
        c.front_share = 0.0;
        c.rear_share = 1.0;
        break;
      case DriveLayout::kAllWheelDrive:
        // Locked centre, 50/50: a typical 1/10-scale 4WD is a belt or shaft
        // with no centre differential, and the schema deliberately has no
        // field for one (ADR-0031).
        c.front_share = 0.5;
        c.rear_share = 0.5;
        break;
    }

    // ------------------------------------------------------ ESC and battery
    // Evaluated once per step at the entry state, quasi-static over one step
    // (ADR-0031): the electrical time constants are far below the 1 kHz step
    // and pack state moves on the scale of seconds. The result is a per-step
    // torque budget, which keeps the derivative pure.
    //
    // The curve speed is the mean of the driven wheels' entry speeds. A state
    // constructed by hand at speed with omega_w still zero gets one step of
    // stall-region torque before the report catches up; that is noted in
    // ADR-0031 and is not a bug.
    double omega_mean = 0.0;
    switch (params_.layout) {
      case DriveLayout::kFrontWheelDrive:
        omega_mean = 0.5 * (s.omega_w[kFrontLeft] + s.omega_w[kFrontRight]);
        break;
      case DriveLayout::kRearWheelDrive:
        omega_mean = 0.5 * (s.omega_w[kRearLeft] + s.omega_w[kRearRight]);
        break;
      case DriveLayout::kAllWheelDrive:
        omega_mean =
            0.5 * (0.5 * (s.omega_w[kFrontLeft] + s.omega_w[kFrontRight]) +
                   0.5 * (s.omega_w[kRearLeft] + s.omega_w[kRearRight]));
        break;
    }

    const double t_cmd = params_.mass * accel * params_.wheel_radius;
    const double soc_entry = clamp(s.soc, 0.0, 1.0);
    const double ocv =
        params_.pack_v_empty +
        soc_entry * (params_.pack_v_full - params_.pack_v_empty);

    // The torque the ESC will pass at a given pack voltage: the voltage-scaled
    // curve for drive, the amp caps both ways.
    const auto esc_budget = [&](double v_pack) {
      const double scale = v_pack / params_.pack_nominal_v;
      double curve = params_.torque_stall * scale *
                     (1.0 - omega_mean / (params_.omega_free * scale));
      if (curve < 0.0) curve = 0.0;
      const double drive_cap =
          std::fmin(curve, params_.torque_per_amp * params_.current_max);
      const double regen_cap =
          params_.torque_per_amp * params_.regen_current_max;
      return clamp(t_cmd, -regen_cap, drive_cap);
    };
    // Battery-terminal electrical power for a wheel torque. Efficiency loses
    // power in both directions: it divides on discharge and multiplies on
    // regen (ADR-0031).
    const auto electrical_power = [&](double torque) {
      const double p_mech = torque * omega_mean;
      return (p_mech >= 0.0) ? p_mech / params_.drive_efficiency
                             : p_mech * params_.drive_efficiency;
    };

    // Pass one at open-circuit voltage; pass two at the sagged terminal
    // voltage, solved in closed form from V^2 - ocv V + R P = 0 (upper root).
    // A negative discriminant means the demand exceeds the pack's maximum
    // deliverable power, and V clamps to the peak-power point ocv/2.
    const double p1 = electrical_power(esc_budget(ocv));
    double v_pack = ocv;
    const double rp = params_.pack_internal_resistance * p1;
    if (rp != 0.0) {
      const double disc = ocv * ocv - 4.0 * rp;
      v_pack = (disc >= 0.0) ? 0.5 * (ocv + std::sqrt(disc)) : 0.5 * ocv;
    }
    const double t_budget = esc_budget(v_pack);
    const double p2 = electrical_power(t_budget);
    const double pack_current = p2 / v_pack;
    const bool esc_sat = (t_budget != t_cmd);

    c.fx_demand = t_budget / params_.wheel_radius;
    c.soc_rate = -pack_current / (3600.0 * params_.pack_capacity_ah);

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
    y[kSteerRate] = s.steer_rate;
    y[kSoc] = soc_entry;
    y[kX] = s.pos.x;
    y[kY] = s.pos.y;
    y[kYaw] = s.yaw;
    y[kSteer] = s.steer;

    const auto deriv = [&](const StateVec<kN>& q) {
      Forces f;
      return derivative(q, c, &f);
    };

    integrate<kN>(integrator_, y, dt, kNVel, deriv);

    // The mechanical end stop, inelastic: travel clamps and the rate dies at
    // the stop (ADR-0031). The command is already inside the travel, so this
    // only engages on overshoot.
    if (y[kSteer] > params_.steer_max) {
      y[kSteer] = params_.steer_max;
      y[kSteerRate] = 0.0;
    } else if (y[kSteer] < -params_.steer_max) {
      y[kSteer] = -params_.steer_max;
      y[kSteerRate] = 0.0;
    }
    y[kSoc] = clamp(y[kSoc], 0.0, 1.0);

    s.vel_body.x = y[kVx];
    s.vel_body.y = y[kVy];
    s.vel_body.z = 0.0;
    s.rates = Vec3{0.0, 0.0, y[kR]};
    for (unsigned i = 0; i < kWheelCount; ++i) s.alpha_lag[i] = y[kLag0 + i];
    s.pos.x = y[kX];
    s.pos.y = y[kY];
    s.yaw = wrap_to_pi(y[kYaw]);
    s.steer = y[kSteer];
    s.steer_rate = y[kSteerRate];
    s.soc = y[kSoc];
    // Algebraic, like the per-wheel Fz: a reported consequence of this step's
    // load, not a degree of freedom (ADR-0031).
    s.pack_v = v_pack;

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

      out->drive_torque = t_budget;
      out->pack_current = pack_current;

      out->steer_saturated = steer_sat;
      out->accel_saturated = accel_sat;
      out->speed_saturated = speed_sat;
      out->esc_saturated = esc_sat;
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

  // The differential: one axle's longitudinal demand becomes two delivered
  // force pairs (ADR-0031). Closed forms only; the fixed evaluation counts
  // are part of the trajectory, exactly as the load passes are.
  //
  // Mirror symmetry is load-bearing here: every rule below commutes with
  // swapping left and right, including its tie cases, and the invariant test
  // holds the whole tier to it bit for bit.
  void split_axle(unsigned l, unsigned r, double f_axle,
                  const std::array<WheelPre, kWheelCount>& pre,
                  std::array<CombinedForce, kWheelCount>* delivered) const {
    const auto ellipse = [&](unsigned i, double fx) {
      return friction_ellipse(fx, pre[i].fy_pure, pre[i].fx_max, pre[i].fy_max);
    };

    // The spool's constrained-speed solution, shared by kSpool and kLsd.
    // Both wheels turn at one speed, so with the linear slip stiffness c_i
    // the wheel force at a common contact speed omega R is
    // F_i = (c_i / v_i) omega R - c_i, and force balance closes it:
    // omega R = (F_axle + c_l + c_r) / (c_l / v_l + c_r / v_r). The slower
    // (inner) wheel carries the larger slip ratio and drives harder, which
    // is the spool's understeer push arriving from the constraint alone.
    const auto spool_forces = [&](double* fl, double* fr) {
      const double al = pre[l].c_kappa / pre[l].vx_safe;
      const double ar = pre[r].c_kappa / pre[r].vx_safe;
      const double denom = al + ar;
      if (denom <= 0.0) {
        // Both wheels unloaded: nothing to react the torque against.
        *fl = 0.0;
        *fr = 0.0;
        return;
      }
      // The pair sum is bracketed first: a three-term sum associated left to
      // right computes (f_axle + c_l) + c_r, whose rounding differs from its
      // left/right mirror, and the mirror-symmetry invariant catches exactly
      // that (ADR-0004: fixed reduction order in a numerical path).
      const double omega_r =
          (f_axle + (pre[l].c_kappa + pre[r].c_kappa)) / denom;
      *fl = al * omega_r - pre[l].c_kappa;
      *fr = ar * omega_r - pre[r].c_kappa;
    };

    switch (params_.differential) {
      case Differential::kOpen: {
        // Equal torque both sides, and the weaker side caps both: an open
        // differential cannot support a torque difference across the axle,
        // so the side that delivers less sets what the other receives. The
        // exact-tie case keeps both wheels' own ellipse results; re-running
        // one side on a tie would break mirror symmetry.
        const double half = 0.5 * f_axle;
        CombinedForce dl = ellipse(l, half);
        CombinedForce dr = ellipse(r, half);
        const double mag_l = std::fabs(dl.fx);
        const double mag_r = std::fabs(dr.fx);
        if (mag_l < mag_r) {
          dr = ellipse(r, dl.fx);
        } else if (mag_r < mag_l) {
          dl = ellipse(l, dr.fx);
        }
        (*delivered)[l] = dl;
        (*delivered)[r] = dr;
        return;
      }
      case Differential::kSpool: {
        double fl = 0.0;
        double fr = 0.0;
        spool_forces(&fl, &fr);
        (*delivered)[l] = ellipse(l, fl);
        (*delivered)[r] = ellipse(r, fr);
        return;
      }
      case Differential::kLsd: {
        // Locked while the implied torque difference is within the preload;
        // past it the clutch slips and the split falls back to an open one
        // biased by the preload toward the slower wheel, which is the wheel
        // the clutch is feeding torque to. With no speed difference there is
        // no slip direction to bias along and the halves stay equal.
        double fl = 0.0;
        double fr = 0.0;
        spool_forces(&fl, &fr);
        if (std::fabs(fl - fr) * params_.wheel_radius <= params_.lsd_preload) {
          (*delivered)[l] = ellipse(l, fl);
          (*delivered)[r] = ellipse(r, fr);
          return;
        }
        const double half = 0.5 * f_axle;
        const double bias = 0.5 * params_.lsd_preload / params_.wheel_radius;
        if (pre[l].vx_safe < pre[r].vx_safe) {
          fl = half + bias;
          fr = half - bias;
        } else if (pre[r].vx_safe < pre[l].vx_safe) {
          fl = half - bias;
          fr = half + bias;
        } else {
          fl = half;
          fr = half;
        }
        (*delivered)[l] = ellipse(l, fl);
        (*delivered)[r] = ellipse(r, fr);
        return;
      }
    }
  }

  // One evaluation of the tyre forces at a given set of vertical loads. Called
  // twice per derivative: see the note on the algebraic loop above.
  void evaluate_forces(const StateVec<kN>& q, const StepConstants& c,
                       const WheelLoads& loads, Forces* f) const {
    const double vx = q[kVx];
    const double vy = q[kVy];
    const double r = q[kR];

    // The ACHIEVED road wheel angle is state now (ADR-0031), so the steer
    // trigonometry is per evaluation rather than per step.
    const double delta = q[kSteer];
    const double cos_delta = std::cos(delta);
    const double sin_delta = std::sin(delta);

    // Phase one: everything about a wheel that does not depend on how the
    // differential splits the longitudinal demand.
    std::array<WheelPre, kWheelCount> pre{};
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
      const double steer_i = front ? delta : 0.0;
      const double alpha = std::atan2(vyw, vx_safe) - steer_i;

      const MfLite& tyre = front ? c.tyre_front : c.tyre_rear;
      const double fz = loads.fz[i];

      // The force uses the LAGGED slip angle, which is the state, while the
      // load is the instantaneous one. That asymmetry is the point of
      // ADR-0026: the force is inside the current friction budget however much
      // history the slip angle carries.
      pre[i].fy_pure = mf_lite_fy(tyre, q[kLag0 + i], fz);
      pre[i].fx_max = peak_longitudinal_force(tyre, fz);
      pre[i].fy_max = peak_lateral_force(tyre, fz);
      pre[i].vx_safe = vx_safe;

      // Slip stiffness at this load, entering the same way it does for
      // cornering stiffness: the stiffness scales as the peak force does.
      const double c_kappa_nom = front ? c.c_kappa_front : c.c_kappa_rear;
      const double fz_nom = front ? c.fz_nom_front : c.fz_nom_rear;
      pre[i].c_kappa = c_kappa_nom * std::pow(fz / fz_nom, 1.0 - tyre.k_mu);

      f->alpha[i] = alpha;
      f->fz[i] = fz;
      f->vx_wheel[i] = vxw;
    }

    // Phase two: the drivetrain. Each driven axle splits its share of the
    // demand through the differential; an undriven axle freewheels, which is
    // a zero longitudinal demand through the same ellipse.
    std::array<CombinedForce, kWheelCount> delivered{};
    if (c.front_share != 0.0) {
      split_axle(kFrontLeft, kFrontRight, c.front_share * c.fx_demand, pre,
                 &delivered);
    } else {
      delivered[kFrontLeft] = friction_ellipse(
          0.0, pre[kFrontLeft].fy_pure, pre[kFrontLeft].fx_max,
          pre[kFrontLeft].fy_max);
      delivered[kFrontRight] = friction_ellipse(
          0.0, pre[kFrontRight].fy_pure, pre[kFrontRight].fx_max,
          pre[kFrontRight].fy_max);
    }
    if (c.rear_share != 0.0) {
      split_axle(kRearLeft, kRearRight, c.rear_share * c.fx_demand, pre,
                 &delivered);
    } else {
      delivered[kRearLeft] = friction_ellipse(
          0.0, pre[kRearLeft].fy_pure, pre[kRearLeft].fx_max,
          pre[kRearLeft].fy_max);
      delivered[kRearRight] = friction_ellipse(
          0.0, pre[kRearRight].fy_pure, pre[kRearRight].fx_max,
          pre[kRearRight].fy_max);
    }

    // Phase three: body frame, moments and the slip-ratio report.
    //
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

      // Wheel frame to body frame. Only the front wheels are rotated.
      const double cos_i = front ? cos_delta : 1.0;
      const double sin_i = front ? sin_delta : 0.0;
      fx_b[i] = delivered[i].fx * cos_i - delivered[i].fy * sin_i;
      fy_b[i] = delivered[i].fx * sin_i + delivered[i].fy * cos_i;

      // Yaw moment about the CoG. The second term is why a double-track model
      // yaws under asymmetric braking and a bicycle model cannot.
      mz_w[i] = xw * fy_b[i] - yw * fx_b[i];

      f->fx[i] = delivered[i].fx;
      f->fy[i] = delivered[i].fy;
      f->saturated[i] = delivered[i].saturated;

      // The slip ratio consistent with the force that was actually delivered,
      // through the linear slip stiffness at this load. Reported, not
      // integrated (ADR-0027). An undriven wheel delivers no longitudinal
      // force and freewheels at exactly zero slip.
      f->kappa[i] = (pre[i].c_kappa > 0.0)
                        ? (delivered[i].fx / pre[i].c_kappa)
                        : 0.0;
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

    // The servo: a slew-limited second-order lag on the achieved road wheel
    // angle (ADR-0031). The rate state can wind past the slew limit; the
    // angle only ever moves at the clamped rate.
    const double wn = params_.steer_bandwidth;
    d[kSteer] = clamp(q[kSteerRate], -params_.steer_rate_max,
                      params_.steer_rate_max);
    d[kSteerRate] = wn * wn * (c.delta_cmd - q[kSteer]) -
                    2.0 * params_.steer_damping * wn * q[kSteerRate];

    // Per-step constant: the electrical loop closed once at the entry state.
    d[kSoc] = c.soc_rate;

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
