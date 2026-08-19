// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Quasi-static load transfer (CORE-05).
//
// This is the first piece of L2 and it is deliberately separable from the
// rest of it. Load transfer is the mechanism through which CoG height,
// weight distribution and track width finally act on a trajectory, and it has
// a closed-form answer, so it is worth having as a pure function that can be
// checked against the static equations before any tyre nonlinearity is in the
// way. The tests in test_analytical.cpp compare it against formulae derived
// independently of this file.
//
// Public rather than internal, for the same reason the rest of the core is:
// somebody embedding SlipX to drive their own tyre model wants per-corner
// vertical loads, and this is the whole of what they need. It is header-only,
// allocates nothing and reads nothing outside its arguments (CORE-01,
// CORE-03).
//
// ============================================================================
// What "quasi-static" means here, and what it does not
// ============================================================================
//
// The loads are an instantaneous algebraic function of the accelerations. The
// car is treated as a rigid body on rigid suspension: there is no roll or
// pitch degree of freedom, no spring or damper, and therefore no load transfer
// transient. Applying a step of lateral acceleration moves the load in the
// same instant.
//
// A real car does not do that. Load transfer through a suspension lags the
// lateral acceleration by roughly the roll mode period, which is tens of
// milliseconds. That lag is a suspension state and belongs to L3, which is
// what SRS 2.4 lists it under. At L2 the alternative to quasi-static is not a
// better model, it is two spring rates and two damping rates that nobody with
// a car park and an IMU can identify, which ADR-0009 rules out for tyres and
// which is no more defensible here.
//
// ============================================================================
// Sign conventions: ISO 8855 (see conventions.hpp)
// ============================================================================
//
// ax is positive forward, ay is positive to the LEFT. So:
//
//   accelerating (ax > 0)    moves load rearward
//   braking      (ax < 0)    moves load forward
//   turning left (ay > 0)    moves load to the RIGHT, onto the outer wheels
//
// The last one catches people, and it is the one the mirror-symmetry test
// exists to hold: in the ISO frame a left turn produces positive ay, and the
// wheels that get loaded are the right-hand ones.

#ifndef SLIPX_LOAD_TRANSFER_HPP
#define SLIPX_LOAD_TRANSFER_HPP

#include <array>

#include "slipx/conventions.hpp"
#include "slipx/math.hpp"
#include "slipx/params.hpp"

namespace slipx {

// Per-corner vertical loads and the transfer terms that produced them.
//
// The invariant this type carries, and which the tests assert in every case
// including wheel lift: the four wheel loads sum to the vehicle weight,
// exactly. Load is moved between corners, never created or destroyed.
struct WheelLoads {
  // Vertical load per wheel, in the fixed wheel order of conventions.hpp.
  // Never negative: a tyre can push the road away and cannot pull it up.
  std::array<double, kWheelCount> fz{};                          //       [N]

  double fz_front = 0.0;   // front axle total, fz[FL] + fz[FR]         [N]
  double fz_rear = 0.0;    // rear axle total                            [N]

  // Load moved from the front axle to the rear. Positive under acceleration,
  // negative under braking. Reported after any wheel-lift clamping, so it is
  // always the transfer that actually happened rather than the one the
  // unclamped formula asked for.
  double transfer_long = 0.0;                                    //       [N]

  // Load moved from the left wheels to the right ones, summed over both
  // axles. Positive in a left turn (positive ay), by the sign note above.
  double transfer_lat = 0.0;                                     //       [N]

  // At least one wheel reached zero load, so the unclamped transfer would
  // have asked a tyre to pull down on the road. This is the static rollover
  // condition and it is reported rather than silently clamped, because it is
  // the point past which the quasi-static model has stopped describing a car
  // that still has four wheels on the ground. Detecting rollover as an event
  // and halting the agent is the orchestrator's job (ADR-0042); what the
  // core owes that decision is this flag and the clamped per-wheel loads.
  bool wheel_lifted = false;
};

// Static loads: the case ax = ay = 0, which is what L1 uses and what every
// other result here reduces to.
//
//   Fz_front = m g l_r / L        Fz_rear = m g l_f / L
//
// Note that the FRONT load carries l_r. A CoG close to the front axle means a
// small l_f and a large l_r, and the front axle carries most of the weight;
// getting this the wrong way round is the classic sign error in this formula
// and it is asserted in test_analytical.cpp rather than trusted.
inline WheelLoads static_loads(const VehicleParams& p) {
  const double weight = p.mass * kGravity;
  const double inv_wheelbase = 1.0 / p.wheelbase();

  WheelLoads out;
  out.fz_front = weight * p.lr * inv_wheelbase;
  out.fz_rear = weight * p.lf * inv_wheelbase;
  out.fz[kFrontLeft] = 0.5 * out.fz_front;
  out.fz[kFrontRight] = 0.5 * out.fz_front;
  out.fz[kRearLeft] = 0.5 * out.fz_rear;
  out.fz[kRearRight] = 0.5 * out.fz_rear;
  out.transfer_long = 0.0;
  out.transfer_lat = 0.0;
  return out;
}

// Quasi-static loads under a body-frame acceleration (CORE-05).
//
//   ax  longitudinal specific force at the CoG, positive forward   [m/s^2]
//   ay  lateral specific force at the CoG, positive left           [m/s^2]
//
// These are the specific forces an ideal accelerometer at the CoG would read,
// which is the same quantity StepDiagnostics::ax and ::ay report, so the loads
// and the diagnostics describe one car and not two.
//
// Longitudinal, from the pitch moment about the contact patches:
//
//   dFz_long = m ax h / L
//
// Lateral, per axle, from the roll moment about that axle:
//
//   dFz_lat_axle = m_axle ay h / t_axle
//
// where m_axle is that axle's share of the lateral force rather than its share
// of the weight. In steady state those are the same number, and it is worth
// saying why rather than leaving it looking like an assumption. Yaw moment
// balance gives l_f Fy_f = l_r Fy_r, and the two lateral forces sum to m ay,
// so Fy_f = m ay l_r / L and Fy_r = m ay l_f / L. That is the static weight
// split exactly. See ADR-0022 for why the split is not a roll stiffness
// distribution: a roll stiffness distribution is not identifiable in a car
// park, and this one follows from a moment balance and introduces no
// parameter at all.
inline WheelLoads quasi_static_loads(const VehicleParams& p, double ax,
                                     double ay) {
  const double weight = p.mass * kGravity;
  const double inv_wheelbase = 1.0 / p.wheelbase();

  // ---------------------------------------------------------- longitudinal
  const double static_front = weight * p.lr * inv_wheelbase;
  const double static_rear = weight * p.lf * inv_wheelbase;
  const double dfz_long = p.mass * ax * p.h_cog * inv_wheelbase;

  double fz_front = static_front - dfz_long;
  double fz_rear = static_rear + dfz_long;

  WheelLoads out;

  // An axle cannot pull the car down. Clamping one axle to zero and giving
  // the whole weight to the other preserves the sum, which is the invariant
  // the caller is entitled to rely on; clamping without redistributing would
  // quietly delete weight from the car under hard braking.
  if (fz_front < 0.0) {
    fz_front = 0.0;
    fz_rear = weight;
    out.wheel_lifted = true;
  } else if (fz_rear < 0.0) {
    fz_rear = 0.0;
    fz_front = weight;
    out.wheel_lifted = true;
  }

  // --------------------------------------------------------------- lateral
  // Axle shares of the lateral force, from the yaw moment balance above.
  const double mass_front = p.mass * p.lr * inv_wheelbase;
  const double mass_rear = p.mass * p.lf * inv_wheelbase;
  const double dfz_lat_front = mass_front * ay * p.h_cog / p.track_front;
  const double dfz_lat_rear = mass_rear * ay * p.h_cog / p.track_rear;

  // Positive ay is a left turn and loads the RIGHT wheels.
  double fz_fl = 0.5 * fz_front - dfz_lat_front;
  double fz_fr = 0.5 * fz_front + dfz_lat_front;
  double fz_rl = 0.5 * fz_rear - dfz_lat_rear;
  double fz_rr = 0.5 * fz_rear + dfz_lat_rear;

  // Same redistribution rule per axle: the inner wheel lifts and the outer
  // one takes the whole axle load.
  if (fz_fl < 0.0) {
    fz_fl = 0.0;
    fz_fr = fz_front;
    out.wheel_lifted = true;
  } else if (fz_fr < 0.0) {
    fz_fr = 0.0;
    fz_fl = fz_front;
    out.wheel_lifted = true;
  }
  if (fz_rl < 0.0) {
    fz_rl = 0.0;
    fz_rr = fz_rear;
    out.wheel_lifted = true;
  } else if (fz_rr < 0.0) {
    fz_rr = 0.0;
    fz_rl = fz_rear;
    out.wheel_lifted = true;
  }

  out.fz[kFrontLeft] = fz_fl;
  out.fz[kFrontRight] = fz_fr;
  out.fz[kRearLeft] = fz_rl;
  out.fz[kRearRight] = fz_rr;
  out.fz_front = fz_front;
  out.fz_rear = fz_rear;

  // Reported from the loads that came out, not from the formulae that went
  // in, so that a clamped case reports the transfer that happened.
  out.transfer_long = static_front - fz_front;
  out.transfer_lat = 0.5 * ((fz_fr - fz_fl) + (fz_rr - fz_rl));
  return out;
}

// Lateral acceleration at which the inner wheels of an axle first reach zero
// load, with no longitudinal acceleration applied. Positive magnitude.
//
//   ay_lift = g t / (2 h)
//
// Independent of mass and of weight distribution, which is why it is the
// number a chassis is judged on: it is the static stability factor times g,
// and the only two things a builder can change are the track and the height of
// the battery. The vehicle threshold is the lower of the two axles', since the
// first wheel to lift is the one that matters.
//
// This is the static threshold and not a rollover event. It says nothing about
// whether the car goes over, which depends on how long the lateral
// acceleration is held and on the roll inertia; detecting rollover and halting
// the agent is the orchestrator's job (ADR-0042). Nor does it mean the car
// reaches this value: on a low-friction surface the tyres let go first, at
// ay = mu g, which is the usual and much safer outcome at 1/10 scale.
inline double static_rollover_threshold(const VehicleParams& p) {
  const double front = kGravity * p.track_front / (2.0 * p.h_cog);
  const double rear = kGravity * p.track_rear / (2.0 * p.h_cog);
  return front < rear ? front : rear;
}

}  // namespace slipx

#endif  // SLIPX_LOAD_TRANSFER_HPP
