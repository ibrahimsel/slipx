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
#include "slipx/tyre.hpp"

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

// Which axles the motor drives. There is no centre-differential option because
// a typical 1/10-scale 4WD is a locked belt or shaft; kAllWheelDrive splits
// torque 50/50 between the axles (ADR-0031).
enum class DriveLayout {
  kRearWheelDrive,   // schema "2WD_rear"; the common competition layout
  kFrontWheelDrive,  // schema "2WD_front"
  kAllWheelDrive     // schema "4WD", locked centre
};

// How the driven axle splits torque between its two wheels. Modelled from L2
// (ADR-0031); below L2 it cannot affect the trajectory.
enum class Differential {
  kOpen,   // equal torque; the weaker wheel caps both
  kSpool,  // locked axle: one shared wheel speed
  kLsd     // preloaded limited-slip; needs lsd_preload
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
  //
  // Normalised by the static per-tyre load these are about 24 and 26 per
  // radian, which is where a small soft tyre plausibly sits; a full-size
  // passenger tyre is 14 to 21 and a racing slick is higher. Together with
  // mu_y0 and the MF-lite shape factors they put the tyre's peak near 7
  // degrees of slip angle, which is where a peak belongs. They were 120 and
  // 130 until ADR-0032, which describes what was wrong with that and why
  // fixing it moved every published hash.
  double c_alpha_f = 420.0;  // front axle cornering stiffness     [N/rad]
  double c_alpha_r = 455.0;  // rear axle cornering stiffness      [N/rad]

  // Peak friction, used at L1 only to clip the linear tyre so a step steer
  // does not produce an unbounded lateral force. It is a clip, not a Magic
  // Formula: L1's stated limitation is that it has no saturation shape, and
  // clipping keeps that visible rather than pretending otherwise.
  double mu_clip = 1.1;      // peak friction coefficient              [-]

  // The MF-lite coefficients, used from L2 (CORE-06). Below L2 the tyre is
  // linear and clipped, so none of these has any effect.
  //
  // Per axle, matching c_alpha_f and c_alpha_r above and matching the schema,
  // where a car names a front and a rear tyre as separate (compound, surface)
  // references (ADR-0010). They are usually the same tyre and are allowed not
  // to be.
  //
  // The stiffness factor B is NOT here: it is derived at construction from the
  // cornering stiffness above and the static load, because B on its own is not
  // measurable (ADR-0023). The nominal load is likewise not here: the static
  // per-tyre load of this car is the reference point.
  TyreCoefficients tyre_front{};
  TyreCoefficients tyre_rear{};

  // Longitudinal slip stiffness per TYRE at that tyre's static load, positive:
  // the initial slope of longitudinal force against slip ratio. Used from L2
  // to report the slip ratio consistent with the delivered force, and
  // therefore to set the wheel speeds (ADR-0027).
  //
  // One value for all four tyres rather than a front and a rear, unlike
  // cornering stiffness. That is not an oversight: the manoeuvre that
  // identifies it is a straight-line acceleration run, which measures the
  // whole car's longitudinal response and cannot separate the axles. A
  // parameter split finer than the measurement that produces it is the error
  // ADR-0009 exists to prevent.
  //
  // Identifiable in a car park from encoder slip ratio against IMU
  // longitudinal acceleration, so it meets ADR-0009's bar. It is NOT in
  // tyre.schema.json at schema 0.1.0, which is why the loader refuses to build
  // L2 from a 0.1.0 tyre file rather than defaulting it; schema 0.2.0 adds the
  // field. See ADR-0025.
  //
  // Provisional. It was set at twice the per-tyre cornering stiffness above,
  // which is the usual ratio between the two slopes, and ADR-0032 then raised
  // the cornering stiffness without following it here: the two are identified
  // by different manoeuvres (a skidpad and a straight-line acceleration run),
  // so moving one because the other moved would couple two parameters the
  // measurement keeps separate (ADR-0009). The ratio was a plausibility
  // argument rather than a law, and it no longer holds; this number stands on
  // its own until something measures it.
  double c_kappa = 120.0;    //                            [N per unit slip]


  // ----------------------------------------------------------- drivetrain
  // At L0 and L1 the drivetrain is a commanded longitudinal acceleration with
  // limits, and these three are the whole of it. From L2 the ESC block below
  // supersedes accel_max as the mechanism; accel_max and decel_max remain
  // command bounds, and the ESC decides what is actually delivered.
  double accel_max = 8.0;    // peak forward acceleration          [m/s^2]
  double decel_max = 12.0;   // peak braking deceleration, positive
                             // magnitude                          [m/s^2]
  double v_max = 20.0;       // top speed                            [m/s]

  // Drive layout and differential, used from L2 (ADR-0031). The default is an
  // open differential on the rear axle: the open diff produces no
  // drive-induced yaw moment on a symmetric car, which keeps the struct
  // defaults agreeing with the single-track tiers at low lateral
  // acceleration. The reference car FILE says spool, because most 1/10
  // competition cars run a locked rear axle; the file describes a real class
  // of car, the default describes the neutral baseline.
  //
  // Braking goes through the driven axle only: a 1/10-scale car brakes
  // through its motor and has no friction brakes, so there is no brake bias
  // parameter anywhere.
  DriveLayout layout = DriveLayout::kRearWheelDrive;
  Differential differential = Differential::kOpen;
  double lsd_preload = 0.0;  // LSD locking preload torque across the
                             // axle; consumed only when differential
                             // is kLsd                             [N m]

  // ------------------------------------------------------------------ ESC
  // The motor and ESC torque-speed curve, stated at the WHEELS and at
  // pack_nominal_v so that no gear ratio or motor constant is a parameter
  // (ADR-0030 records why the electrical parameterisation was rejected).
  // Used from L2; below L2 the accel limits above are the whole model.
  //
  //   T_avail(omega) = torque_stall * s * (1 - omega / (omega_free * s))
  //   s = pack_v / pack_nominal_v
  //
  // then capped by torque_per_amp * current_max. Negative (braking) torque is
  // capped by torque_per_amp * regen_current_max, and that regen cap is the
  // only brake the model has.
  double torque_stall = 2.0;      // total wheel torque at zero wheel
                                  // speed, full throttle, before the
                                  // current limit                    [N m]
  double omega_free = 480.0;      // wheel speed at which drive torque
                                  // reaches zero                   [rad/s]
  double torque_per_amp = 0.01;   // wheel torque per ampere of motor
                                  // current                        [N m/A]
  double drive_efficiency = 0.85; // wheel power over battery-terminal
                                  // power, in (0, 1]. Losses apply in
                                  // both directions                    [-]
  double current_max = 120.0;     // ESC drive current limit            [A]
  double regen_current_max = 40.0;  // regen current limit, its own
                                  // number and usually well below
                                  // drive                             [A]

  // -------------------------------------------------------------- battery
  // Open-circuit voltage linear in state of charge between pack_v_empty and
  // pack_v_full; internal resistance produces sag under load (ADR-0031).
  // Used from L2. Setting pack_v_full = pack_v_empty = pack_nominal_v with
  // zero internal resistance is the ideal-supply configuration and is valid.
  double pack_nominal_v = 11.1;   // the voltage the ESC curve is
                                  // stated at; 3S LiPo nominal        [V]
  double pack_v_full = 12.6;      // open-circuit voltage at soc 1     [V]
  double pack_v_empty = 9.9;      // open-circuit voltage at soc 0     [V]
  double pack_capacity_ah = 5.2;  //                                 [A h]
  double pack_internal_resistance = 0.020;  //                       [ohm]

  // ------------------------------------------------------------- steering
  // The road wheel angle follows the command instantaneously at L0 and L1,
  // clipped to travel. From L2 the servo is a slew-limited second-order lag
  // (ADR-0031) and steer/steer_rate become integrated state.
  double steer_max = 0.40;   // road wheel travel, symmetric, positive
                             // magnitude; positive command is left  [rad]
  double steer_rate_max = 10.0;   // servo slew limit              [rad/s]
  double steer_bandwidth = 45.0;  // second-order natural frequency
                                  //                               [rad/s]
  double steer_damping = 0.7;     // damping ratio; below 1 the servo
                                  // overshoots, which is physics       [-]

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
