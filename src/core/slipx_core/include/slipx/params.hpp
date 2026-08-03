// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// VehicleParams: the plain struct through which every parameter enters the
// core (CORE-01).
//
// There is deliberately no parsing here, no file path, no YAML, no version
// negotiation and no defaulting logic. slipx_core must build and pass its full
// test suite with slipx_schema absent, so the boundary is a struct a caller
// can fill in by hand in six lines of C++. Everything about where the numbers
// came from is somebody else's problem, which is exactly what makes the core
// embeddable.
//
// Every field carries its unit and, where it has one, its sign convention
// (NFR-07). Fields that a given tier cannot represent are named as such: at L0
// and L1 a change to CoG height or track width has no effect on the
// trajectory, and that is correct behaviour rather than a bug (SRS 2.4).

#ifndef SLIPX_PARAMS_HPP
#define SLIPX_PARAMS_HPP

#include "slipx/conventions.hpp"
#include "slipx/math.hpp"

namespace slipx {

// Provenance of a parameter set, carried into the core so that tooling can
// print it rather than only documenting it (NFR-08). The core does nothing
// with this beyond passing it through; it exists so that a value cannot travel
// from a registry entry to a plot without its label travelling too.
enum class Provenance {
  kProvisional,  // literature- or plausibility-derived. Not measured.
  kIdentified,   // fitted from vehicle data by slipx_id, with residuals.
  kMeasured      // directly measured on a rig or scale.
};

struct VehicleParams {
  // ------------------------------------------------------------------ mass
  double mass = 3.5;         // total sprung + unsprung mass            [kg]
  double izz = 0.05;         // yaw inertia about the CoG z axis   [kg m^2]
  double ixx = 0.02;         // roll inertia. L3 only.             [kg m^2]
  double iyy = 0.06;         // pitch inertia. L3 only.            [kg m^2]

  // -------------------------------------------------------------- geometry
  double lf = 0.16;          // CoG to front axle, positive forward     [m]
  double lr = 0.16;          // CoG to rear axle, positive rearward     [m]
  double track_front = 0.24; // front track width                       [m]
  double track_rear = 0.24;  // rear track width                        [m]
  double h_cog = 0.06;       // CoG height above ground. No effect
                             // below L2: nothing there can transfer
                             // load.                                   [m]
  double wheel_radius = 0.05;  // effective rolling radius              [m]

  double wheelbase() const { return lf + lr; }

  // ---------------------------------------------------------------- tyres
  // L1 uses cornering stiffness per AXLE (both tyres of that axle summed),
  // because a single-track model has one tyre per axle and splitting it would
  // be a fiction. L2 replaces these with MF-lite per corner (CORE-06, P1);
  // these fields stay, because L1 remains a supported tier.
  //
  // Sign: positive. The restoring sign lives in the force law, Fy = -C * alpha
  // (see conventions.hpp), not in the parameter.
  double c_alpha_f = 120.0;  // front axle cornering stiffness     [N/rad]
  double c_alpha_r = 130.0;  // rear axle cornering stiffness      [N/rad]

  // Peak friction, used at L1 only to clip the linear tyre so a step steer
  // does not produce an unbounded lateral force. It is a clip, not a Magic
  // Formula: L1's stated limitation is that it has no saturation shape, and
  // clipping keeps that visible rather than pretending otherwise.
  double mu_clip = 1.1;      // peak friction coefficient              [-]

  // Relaxation length: the distance the tyre must roll before its lateral
  // force reaches the steady-state value for the slip it is being given
  // (CORE-07). The lag is in distance, so the time constant is
  // relax_length / speed and falls as the car speeds up; relaxation.hpp
  // carries the derivation. No effect below L2, which has no tyre transient.
  //
  // Identifiable in a car park from the rise time of yaw rate in a step steer
  // at two speeds, which is why it is a parameter at all (ADR-0009). The value
  // is provisional: it is roughly 1.5 wheel radii, which is where a full-size
  // tyre sits, and it has been measured on nothing.
  double relax_length = 0.08;  //                                       [m]

  // ----------------------------------------------------------- drivetrain
  // At L0 and L1 the drivetrain is a commanded longitudinal acceleration with
  // limits. ESC torque-speed curve, current limit, regen and battery sag are
  // CORE-08 and CORE-09, which arrive at L2 in P1. Until then a caller asking
  // for 40 m/s^2 gets the limit, not the request, and StepDiagnostics says the
  // actuator saturated.
  double accel_max = 8.0;    // peak forward acceleration          [m/s^2]
  double decel_max = 12.0;   // peak braking deceleration, positive
                             // magnitude                          [m/s^2]
  double v_max = 20.0;       // top speed                            [m/s]

  // ------------------------------------------------------------- steering
  // Servo rate limit and second-order lag are CORE-10, L2, P1. At L0 and L1
  // the road wheel angle follows the command instantaneously, clipped to
  // travel.
  double steer_max = 0.40;   // road wheel travel, symmetric, positive
                             // magnitude; positive command is left  [rad]

  // ------------------------------------------------------------ resistance
  // Both act against the direction of travel. At 1/10 scale aerodynamic drag
  // is close to negligible below about 15 m/s and is out of scope as a
  // modelled aero map (SRS 1.5), but a drag term still belongs here because
  // coastdown is one of the identification manoeuvres (ID-02) and coastdown
  // without drag fits the rolling resistance wrong.
  double drag_coeff = 0.015; // 0.5 * rho * Cd * A                  [kg/m]
  double roll_resist = 0.015;  // rolling resistance coefficient       [-]

  // ------------------------------------------------------------ provenance
  // Defaults to provisional because the defaults in this struct are
  // provisional: they are plausible for a 1/10-scale car and have not been
  // measured against one. Nothing in SlipX may present them as anything else.
  Provenance provenance = Provenance::kProvisional;

  // Numerical floor for the longitudinal speed appearing in slip-angle
  // denominators. Not a physical parameter: it is the documented mitigation
  // for the single-track model's singularity at standstill. Exposed rather
  // than hidden so that it appears in the manifest hash and so that a caller
  // who changes it knows they have changed the model.
  double v_eps = 0.5;        // [m/s]
};

// Returns nullptr if the parameter set is usable, or a static message naming
// the offending field.
//
// This is a sanity check on physical impossibility (negative mass, zero
// wheelbase), NOT schema validation. Range checking, RoboRacer dimensional
// legality (SCH-03) and inertia plausibility (SCH-04) live in slipx_schema,
// which the core is not allowed to know about. The string is a literal, so
// this allocates nothing and works with -fno-exceptions.
const char* validate(const VehicleParams& p);

}  // namespace slipx

#endif  // SLIPX_PARAMS_HPP
