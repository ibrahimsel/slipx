// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// pybind11 bindings for slipx_core and slipx_sim.
//
// Bindings are written alongside the core rather than bolted on afterwards
// (SRS 1.3), because the audience that decides whether this library gets
// adopted is largely a Python audience: RL researchers, course instructors and
// students. A binding layer added late is a binding layer shaped by whatever
// the C++ API happened to become.
//
// Two rules this file follows:
//
//   The Python API is the C++ API. Same names, same units, same conventions,
//   same tiers. A tutorial written in one translates line by line into the
//   other, and there is no second set of semantics to keep in step.
//
//   Nothing here decides anything. Parsing lives in slipx_schema, physics in
//   slipx_core, orchestration in slipx_sim. This file moves values across the
//   boundary and does not add behaviour of its own; the moment it starts
//   computing something, that thing exists only for Python users.
//
// On the GIL: it is held throughout, including inside run(). A policy is a
// Python callable and calling one without the GIL crashes the interpreter,
// and pybind11's std::function wrapper gives no way to ask whether a given
// policy came from Python. Releasing it conditionally is worth doing and is
// not done here, because the version that guesses wrong is a segfault rather
// than a slow test.
//
// What that costs is parallelism inside one process. What it does not cost is
// parallelism across processes: the physics is a const, allocation-free
// function of its arguments (CORE-01, CORE-03), so N interpreters running N
// simulations share nothing and scale linearly, which is how a batched RL
// rollout should be arranged in Python regardless.

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>
#include <string>

#include "slipx/sim/hash.hpp"
#include "slipx/sim/manifest.hpp"
#include "slipx/sim/manoeuvres.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/sense/rng.hpp"
#include "slipx/sim/sensor_rig.hpp"
#include "slipx/sim/simulation.hpp"
#include "slipx/sim/track_world.hpp"
#include "slipx/tyre.hpp"
#include "slipx/vehicle_model.hpp"
#include "slipx/version.hpp"

namespace py = pybind11;
using namespace slipx;
using namespace slipx::sim;

namespace {

std::string repr_state(const VehicleState& s) {
  std::ostringstream o;
  o.precision(6);
  o << "VehicleState(pos=(" << s.pos.x << ", " << s.pos.y << "), yaw=" << s.yaw
    << ", vel_body=(" << s.vel_body.x << ", " << s.vel_body.y
    << "), yaw_rate=" << s.rates.z << ", steer=" << s.steer << ")";
  return o.str();
}

}  // namespace

PYBIND11_MODULE(_slipx, m) {
  m.doc() =
      "Bindings for slipx_core and slipx_sim.\n\n"
      "Import the `slipx` package rather than this module directly; this is "
      "the extension it is built on.";

  m.attr("__version__") = kVersion;
  m.attr("core_version") = kVersion;

  // ------------------------------------------------------------------ math
  py::class_<Vec3>(m, "Vec3", "Three doubles. ISO 8855: x forward, y left, z up.")
      .def(py::init<>())
      .def(py::init([](double x, double y, double z) { return Vec3{x, y, z}; }),
           py::arg("x"), py::arg("y"), py::arg("z") = 0.0)
      .def_readwrite("x", &Vec3::x)
      .def_readwrite("y", &Vec3::y)
      .def_readwrite("z", &Vec3::z)
      .def("norm", &Vec3::norm)
      .def("__len__", [](const Vec3&) { return 3; })
      .def("__iter__",
           [](const Vec3& v) {
             return py::iter(py::make_tuple(v.x, v.y, v.z));
           })
      .def("__repr__", [](const Vec3& v) {
        std::ostringstream o;
        o << "Vec3(" << v.x << ", " << v.y << ", " << v.z << ")";
        return o.str();
      });

  // ------------------------------------------------------------------ enums
  py::enum_<Tier>(m, "Tier",
                  "Fidelity level, selected at construction. Not the roadmap "
                  "phases P0-P5.")
      .value("L0_Kinematic", Tier::L0_Kinematic,
             "4 states. Ackermann geometry only. Mass, CoG, tyres and "
             "drivetrain have no effect on the trajectory, by design.")
      .value("L1_Bicycle", Tier::L1_Bicycle,
             "6 states. Sideslip, yaw dynamics, linear tyres clipped at the "
             "friction limit. No saturation shape, so no spin.")
      .value("L2_DoubleTrack", Tier::L2_DoubleTrack,
             "13 states. Double-track: four contact patches with per-corner "
             "vertical loads, MF-lite tyres with a real peak and falling "
             "branch, combined slip, tyre relaxation, quasi-static load "
             "transfer, and the drivetrain: open/spool/LSD differential with "
             "2WD or 4WD, ESC torque-speed curve with current and regen "
             "limits, battery sag and state of charge, and a slew-limited "
             "second-order steering servo. Braking is motor braking through "
             "the driven axle only. Still absent, deliberately: wheel "
             "rotational dynamics (no lockup or wheelspin events), a "
             "low-voltage cutoff, Ackermann geometry (parallel steer), and "
             "suspension.")
      .value("L3_Extended", Tier::L3_Extended,
             "Not implemented. Requesting it raises rather than silently "
             "substituting a lower tier.");

  py::enum_<Integrator>(m, "Integrator")
      .value("RK4", Integrator::kRK4, "Fourth order, four evaluations. Default.")
      .value("SemiImplicitEuler", Integrator::kSemiImplicitEuler,
             "Symplectic, one evaluation. Cheaper; first order.");

  py::enum_<Provenance>(m, "Provenance",
                        "How a parameter set was obtained (NFR-08).")
      .value("Provisional", Provenance::kProvisional,
             "Literature- or plausibility-derived. Not measured.")
      .value("Identified", Provenance::kIdentified,
             "Fitted from vehicle data, with residuals.")
      .value("Measured", Provenance::kMeasured, "Directly measured.");

  py::enum_<DriveLayout>(m, "DriveLayout",
                         "Which axles the motor drives. Modelled from L2; a "
                         "4WD centre is locked 50/50, because a typical "
                         "1/10-scale 4WD is a belt with no centre diff.")
      .value("RearWheelDrive", DriveLayout::kRearWheelDrive,
             "schema \"2WD_rear\"; the common competition layout")
      .value("FrontWheelDrive", DriveLayout::kFrontWheelDrive,
             "schema \"2WD_front\"")
      .value("AllWheelDrive", DriveLayout::kAllWheelDrive,
             "schema \"4WD\", locked centre, 50/50");

  py::enum_<Differential>(m, "Differential",
                          "How the driven axle splits torque between its "
                          "wheels. Modelled from L2.")
      .value("Open", Differential::kOpen,
             "equal torque; the weaker wheel caps both")
      .value("Spool", Differential::kSpool,
             "locked axle: one shared wheel speed")
      .value("Lsd", Differential::kLsd,
             "preloaded limited-slip; set lsd_preload");

  // ----------------------------------------------------------------- params
  // MF-lite coefficients (CORE-06). Exposed so a caller can build an L2 car in
  // Python without a tyre file, the same way VehicleParams itself is.
  py::class_<slipx::TyreCoefficients>(m, "TyreCoefficients")
      .def(py::init<>())
      .def_readwrite("mu_y0", &slipx::TyreCoefficients::mu_y0,
                     "peak lateral friction at the nominal load [-]")
      .def_readwrite("mu_x0", &slipx::TyreCoefficients::mu_x0,
                     "peak longitudinal friction at the nominal load [-]")
      .def_readwrite("k_mu", &slipx::TyreCoefficients::k_mu,
                     "load sensitivity exponent, positive [-]")
      .def_readwrite("relax_length", &slipx::TyreCoefficients::relax_length,
                     "relaxation length [m]")
      .def_readwrite("shape_c", &slipx::TyreCoefficients::shape_c,
                     "Magic Formula shape factor C, above 1 [-]")
      .def_readwrite("curvature_e", &slipx::TyreCoefficients::curvature_e,
                     "Magic Formula curvature factor E, at most 1 [-]");

  // The tyre model itself, evaluable pointwise. A vehicle model answers "what
  // did this car do"; these answer "what does this tyre do", which is the
  // question a tyre plot, a fitted parameter check and a hand calculation all
  // ask. Without them the only way to see a tyre curve is to drive a car
  // across it and read the diagnostics back, and every figure of a tyre curve
  // ends up drawn from a second model that agrees with this one by hand.
  py::class_<slipx::MfLite>(
      m, "MfLite",
      "A built MF-lite tyre: the coefficients with the stiffness factor B "
      "derived and the reference load fixed.\n\n"
      "Build one with slipx.make_mf_lite(); B is never taken from a file.")
      .def(py::init<>())
      .def_readwrite("b", &slipx::MfLite::b,
                     "stiffness factor, derived from cornering stiffness [-]")
      .def_readwrite("c", &slipx::MfLite::c, "shape factor [-]")
      .def_readwrite("e", &slipx::MfLite::e, "curvature factor [-]")
      .def_readwrite("mu_y0", &slipx::MfLite::mu_y0,
                     "peak lateral friction at fz_nom [-]")
      .def_readwrite("mu_x0", &slipx::MfLite::mu_x0,
                     "peak longitudinal friction at fz_nom [-]")
      .def_readwrite("k_mu", &slipx::MfLite::k_mu,
                     "load sensitivity exponent [-]")
      .def_readwrite("fz_nom", &slipx::MfLite::fz_nom,
                     "the static per-tyre load the coefficients are stated "
                     "at [N]");

  py::class_<slipx::CombinedForce>(
      m, "CombinedForce",
      "A tyre force pair after the friction budget has been spent.")
      .def_readonly("fx", &slipx::CombinedForce::fx,
                    "longitudinal, positive forward [N]")
      .def_readonly("fy", &slipx::CombinedForce::fy,
                    "lateral, positive left, ISO 8855 [N]")
      .def_readonly("saturated", &slipx::CombinedForce::saturated,
                    "the demand exceeded the ellipse and was scaled back");

  m.def("make_mf_lite", &slipx::make_mf_lite, py::arg("coefficients"),
        py::arg("c_alpha"), py::arg("fz_nom"),
        "Build a tyre. c_alpha is the cornering stiffness of ONE tyre at "
        "fz_nom [N/rad]; a VehicleParams axle value is twice this. fz_nom is "
        "the static vertical load on that one tyre [N].\n\n"
        "The stiffness factor B is derived here and is never read from a "
        "parameter set, so MF-lite reproduces the linear tyre exactly at "
        "small slip.");

  m.def("mf_lite_fy", &slipx::mf_lite_fy, py::arg("tyre"), py::arg("alpha"),
        py::arg("fz"),
        "Pure-slip lateral force [N] at slip angle alpha [rad] and vertical "
        "load fz [N]. ISO 8855: a positive slip angle gives a negative "
        "lateral force.");

  m.def("peak_lateral_force", &slipx::peak_lateral_force, py::arg("tyre"),
        py::arg("fz"),
        "Peak lateral force magnitude [N] at a vertical load [N]. Falls "
        "short of mu_y0 * fz above the nominal load, which is load "
        "sensitivity.");

  m.def("peak_longitudinal_force", &slipx::peak_longitudinal_force,
        py::arg("tyre"), py::arg("fz"),
        "Peak longitudinal force magnitude [N] at a vertical load [N].");

  m.def("cornering_stiffness_at_load", &slipx::cornering_stiffness_at_load,
        py::arg("tyre"), py::arg("fz"),
        "Cornering stiffness [N/rad] at a vertical load [N], positive: the "
        "slope of -Fy against alpha at the origin. The quantity a skidpad "
        "measures.");

  m.def("friction_ellipse", &slipx::friction_ellipse, py::arg("fx"),
        py::arg("fy"), py::arg("fx_max"), py::arg("fy_max"),
        "Spend one contact patch's budget on two demands. Returns the pair "
        "scaled back onto the ellipse when the demand exceeds it, and "
        "unchanged when it does not.");

  py::class_<VehicleParams>(
      m, "VehicleParams",
      "The plain struct through which every parameter enters the core.\n\n"
      "SI units, ISO 8855 signs. Fields a given tier cannot represent have no "
      "effect on it, which is the specified behaviour and not an omission: at "
      "L0 and L1, changing h_cog changes nothing.\n\n"
      "Use slipx.load_car() to build one from a car directory.")
      .def(py::init<>())
      .def_readwrite("mass", &VehicleParams::mass, "total mass [kg]")
      .def_readwrite("izz", &VehicleParams::izz, "yaw inertia [kg m^2]")
      .def_readwrite("ixx", &VehicleParams::ixx, "roll inertia, L3 [kg m^2]")
      .def_readwrite("iyy", &VehicleParams::iyy, "pitch inertia, L3 [kg m^2]")
      .def_readwrite("lf", &VehicleParams::lf, "CoG to front axle [m]")
      .def_readwrite("lr", &VehicleParams::lr, "CoG to rear axle [m]")
      .def_readwrite("track_front", &VehicleParams::track_front, "[m]")
      .def_readwrite("track_rear", &VehicleParams::track_rear, "[m]")
      .def_readwrite("h_cog", &VehicleParams::h_cog,
                     "CoG height [m]. No effect below L2.")
      .def_readwrite("wheel_radius", &VehicleParams::wheel_radius, "[m]")
      .def_readwrite("c_alpha_f", &VehicleParams::c_alpha_f,
                     "front AXLE cornering stiffness, positive [N/rad]")
      .def_readwrite("c_alpha_r", &VehicleParams::c_alpha_r,
                     "rear AXLE cornering stiffness, positive [N/rad]")
      .def_readwrite("mu_clip", &VehicleParams::mu_clip,
                     "peak friction used to clip L1's linear tyre [-]")
      .def_readwrite("tyre_front", &VehicleParams::tyre_front,
                     "front MF-lite coefficients. No effect below L2.")
      .def_readwrite("tyre_rear", &VehicleParams::tyre_rear,
                     "rear MF-lite coefficients. No effect below L2.")
      .def_readwrite("c_kappa", &VehicleParams::c_kappa,
                     "longitudinal slip stiffness per tyre, positive "
                     "[N per unit slip]. No effect below L2.")
      .def_readwrite("accel_max", &VehicleParams::accel_max,
                     "command bound [m/s^2]. From L2 the ESC curve decides "
                     "what is delivered.")
      .def_readwrite("decel_max", &VehicleParams::decel_max,
                     "command bound, positive magnitude [m/s^2]. From L2 the "
                     "regen limit is usually the operative brake limit.")
      .def_readwrite("v_max", &VehicleParams::v_max, "[m/s]")
      .def_readwrite("layout", &VehicleParams::layout,
                     "driven axles. No effect below L2.")
      .def_readwrite("differential", &VehicleParams::differential,
                     "axle torque split. No effect below L2.")
      .def_readwrite("lsd_preload", &VehicleParams::lsd_preload,
                     "LSD locking preload torque across the axle [N m]. "
                     "Consumed only when differential is Lsd.")
      .def_readwrite("torque_stall", &VehicleParams::torque_stall,
                     "total wheel torque at zero wheel speed, full throttle, "
                     "at pack_nominal_v, before the current limit [N m]. "
                     "No effect below L2.")
      .def_readwrite("omega_free", &VehicleParams::omega_free,
                     "wheel speed at which drive torque reaches zero, at "
                     "pack_nominal_v [rad/s]. No effect below L2.")
      .def_readwrite("torque_per_amp", &VehicleParams::torque_per_amp,
                     "wheel torque per ampere of motor current [N m/A]; what "
                     "turns the current limits into torque caps. No effect "
                     "below L2.")
      .def_readwrite("drive_efficiency", &VehicleParams::drive_efficiency,
                     "wheel power over battery-terminal power, in (0, 1] [-]. "
                     "Loses power in both directions. No effect below L2.")
      .def_readwrite("current_max", &VehicleParams::current_max,
                     "ESC drive current limit [A]. No effect below L2.")
      .def_readwrite("regen_current_max", &VehicleParams::regen_current_max,
                     "regen current limit [A]; the only brake the model has. "
                     "No effect below L2.")
      .def_readwrite("pack_nominal_v", &VehicleParams::pack_nominal_v,
                     "the voltage the ESC curve is stated at [V]. No effect "
                     "below L2.")
      .def_readwrite("pack_v_full", &VehicleParams::pack_v_full,
                     "open-circuit voltage at soc 1 [V]. No effect below L2.")
      .def_readwrite("pack_v_empty", &VehicleParams::pack_v_empty,
                     "open-circuit voltage at soc 0 [V]. No effect below L2.")
      .def_readwrite("pack_capacity_ah", &VehicleParams::pack_capacity_ah,
                     "[A h]. No effect below L2.")
      .def_readwrite("pack_internal_resistance",
                     &VehicleParams::pack_internal_resistance,
                     "[ohm]; produces sag under load. No effect below L2.")
      .def_readwrite("steer_max", &VehicleParams::steer_max,
                     "road wheel travel, symmetric magnitude [rad]")
      .def_readwrite("steer_rate_max", &VehicleParams::steer_rate_max,
                     "servo slew limit [rad/s]. No effect below L2.")
      .def_readwrite("steer_bandwidth", &VehicleParams::steer_bandwidth,
                     "servo second-order natural frequency [rad/s]. No effect "
                     "below L2.")
      .def_readwrite("steer_damping", &VehicleParams::steer_damping,
                     "servo damping ratio [-]; below 1 the servo overshoots. "
                     "No effect below L2.")
      .def_readwrite("drag_coeff", &VehicleParams::drag_coeff,
                     "0.5 rho Cd A [kg/m]")
      .def_readwrite("roll_resist", &VehicleParams::roll_resist, "[-]")
      .def_readwrite("v_eps", &VehicleParams::v_eps,
                     "slip-angle speed floor [m/s]. Numerical, not physical.")
      .def_readwrite("provenance", &VehicleParams::provenance)
      .def_property_readonly("wheelbase", &VehicleParams::wheelbase, "[m]")
      .def("validate",
           [](const VehicleParams& p) -> py::object {
             const char* why = slipx::validate(p);
             if (why == nullptr) return py::none();
             return py::str(why);
           },
           "None if the parameters describe a possible object, otherwise the "
           "reason. This is a physical sanity check, not schema validation: "
           "competition legality is slipx_schema's job.")
      .def("copy", [](const VehicleParams& p) { return VehicleParams(p); },
           "A value copy, tyre blocks included. What a fitter perturbs and "
           "what a scenario edits without reaching the original.")
      .def("__repr__", [](const VehicleParams& p) {
        std::ostringstream o;
        o << "VehicleParams(mass=" << p.mass << ", wheelbase=" << p.wheelbase()
          << ", izz=" << p.izz << ")";
        return o.str();
      });

  // ------------------------------------------------------------------ state
  py::class_<VehicleState>(m, "VehicleState",
                           "One state for every tier, so a controller moves "
                           "between tiers unchanged. Trivially copyable.")
      .def(py::init<>())
      .def_readwrite("pos", &VehicleState::pos, "world position of the CoG [m]")
      .def_readwrite("yaw", &VehicleState::yaw,
                     "heading, positive counter-clockwise [rad]")
      .def_readwrite("pitch", &VehicleState::pitch, "L3 only [rad]")
      .def_readwrite("roll", &VehicleState::roll, "L3 only [rad]")
      .def_readwrite("vel_body", &VehicleState::vel_body,
                     "body-frame velocity: x forward, y left [m/s]")
      .def_readwrite("rates", &VehicleState::rates,
                     "body roll, pitch and yaw rate [rad/s]")
      .def_readwrite("steer", &VehicleState::steer,
                     "ACHIEVED road wheel angle [rad]")
      .def_readwrite("steer_rate", &VehicleState::steer_rate, "L2 on [rad/s]")
      .def_readwrite("soc", &VehicleState::soc, "fraction in [0, 1]")
      .def_readwrite("pack_v", &VehicleState::pack_v, "[V]")
      .def_readwrite("omega_w", &VehicleState::omega_w, "wheel speeds, L2 on")
      .def_readwrite("Fz", &VehicleState::Fz, "vertical tyre loads, L2 on [N]")
      .def_readwrite("alpha_lag", &VehicleState::alpha_lag,
                     "lagged slip angle per wheel, L2 on [rad]")
      .def_property_readonly(
          "yaw_rate", [](const VehicleState& s) { return s.rates.z; },
          "[rad/s]")
      .def_property_readonly(
          "vx", [](const VehicleState& s) { return s.vel_body.x; }, "[m/s]")
      .def_property_readonly(
          "vy", [](const VehicleState& s) { return s.vel_body.y; }, "[m/s]")
      .def("speed", &VehicleState::speed, "[m/s]")
      .def("sideslip", &VehicleState::sideslip,
           "body slip angle, positive when the velocity is to the left [rad]")
      .def("copy", [](const VehicleState& s) { return VehicleState(s); },
           "A snapshot. The type is trivially copyable, so this is a memcpy.")
      .def("__copy__", [](const VehicleState& s) { return VehicleState(s); })
      .def("__repr__", &repr_state);

  py::class_<DriveInput>(m, "DriveInput",
                         "Commanded, pre-actuator. What was asked for, not "
                         "what the car achieved.")
      .def(py::init<>())
      .def(py::init([](double steer_cmd, double accel_cmd) {
             return DriveInput{steer_cmd, accel_cmd};
           }),
           py::arg("steer_cmd") = 0.0, py::arg("accel_cmd") = 0.0)
      .def_readwrite("steer_cmd", &DriveInput::steer_cmd,
                     "road wheel angle, positive left [rad]")
      .def_readwrite("accel_cmd", &DriveInput::accel_cmd,
                     "longitudinal acceleration demand [m/s^2]")
      .def("__repr__", [](const DriveInput& u) {
        std::ostringstream o;
        o << "DriveInput(steer_cmd=" << u.steer_cmd
          << ", accel_cmd=" << u.accel_cmd << ")";
        return o.str();
      });

  py::class_<StepDiagnostics>(
      m, "StepDiagnostics",
      "Optional per-step diagnostics (CORE-12): why the car did what it did.\n\n"
      "Quantities a tier cannot represent are NaN, never zero. A plot of L0 "
      "slip angles is empty, which is the correct answer to the question.")
      .def(py::init<>())
      .def_readonly("alpha", &StepDiagnostics::alpha, "per-wheel slip angle [rad]")
      .def_readonly("kappa", &StepDiagnostics::kappa, "per-wheel slip ratio [-]")
      .def_readonly("fx", &StepDiagnostics::fx, "[N]")
      .def_readonly("fy", &StepDiagnostics::fy, "[N]")
      .def_readonly("fz", &StepDiagnostics::fz, "[N]")
      .def_readonly("tyre_saturated", &StepDiagnostics::tyre_saturated)
      .def_readonly("alpha_front", &StepDiagnostics::alpha_front, "[rad]")
      .def_readonly("alpha_rear", &StepDiagnostics::alpha_rear, "[rad]")
      .def_readonly("fy_front", &StepDiagnostics::fy_front, "[N]")
      .def_readonly("fy_rear", &StepDiagnostics::fy_rear, "[N]")
      .def_readonly("fz_front", &StepDiagnostics::fz_front, "[N]")
      .def_readonly("fz_rear", &StepDiagnostics::fz_rear, "[N]")
      .def_readonly("ax", &StepDiagnostics::ax, "specific force [m/s^2]")
      .def_readonly("ay", &StepDiagnostics::ay, "specific force [m/s^2]")
      .def_readonly("load_transfer_long", &StepDiagnostics::load_transfer_long,
                    "[N]. NaN below L2, which cannot transfer load.")
      .def_readonly("load_transfer_lat", &StepDiagnostics::load_transfer_lat,
                    "[N]. NaN below L2.")
      .def_readonly("drive_torque", &StepDiagnostics::drive_torque,
                    "total wheel torque the ESC delivered, after the curve, "
                    "current and regen limits [N m]. Negative when braking. "
                    "NaN below L2, which has no ESC.")
      .def_readonly("pack_current", &StepDiagnostics::pack_current,
                    "battery terminal current [A], positive discharging, "
                    "negative charging under regen. NaN below L2.")
      .def_readonly("steer_saturated", &StepDiagnostics::steer_saturated)
      .def_readonly("accel_saturated", &StepDiagnostics::accel_saturated)
      .def_readonly("speed_saturated", &StepDiagnostics::speed_saturated)
      .def_readonly("esc_saturated", &StepDiagnostics::esc_saturated,
                    "torque demand clipped by the ESC curve, current limit "
                    "or regen limit. Always False below L2.")
      .def_property_readonly(
          "tier", [](const StepDiagnostics& d) { return static_cast<Tier>(d.tier); },
          "which tier produced these numbers, so a plot cannot be mislabelled");

  // ------------------------------------------------------------ the model
  py::class_<VehicleModel>(m, "VehicleModel",
                           "One interface for every tier. step is const and "
                           "allocates nothing.")
      .def_static(
          "create",
          [](Tier tier, const VehicleParams& params, Integrator integrator) {
            return VehicleModel::create(tier, params, integrator);
          },
          py::arg("tier"), py::arg("params"),
          py::arg("integrator") = Integrator::kRK4,
          "Raises ValueError if the tier is not implemented or the parameters "
          "are physically impossible. An unimplemented tier is never silently "
          "substituted.")
      .def(
          "step",
          [](const VehicleModel& self, VehicleState& state,
             const DriveInput& input, double dt, StepDiagnostics* out) {
            self.step(state, input, dt, out);
          },
          py::arg("state"), py::arg("input"), py::arg("dt"),
          py::arg("diagnostics") = nullptr,
          "Advance state in place by dt. Passing diagnostics=None is the "
          "cheap path and does not change the trajectory.")
      .def_property_readonly("tier", &VehicleModel::tier)
      .def_property_readonly("integrator", &VehicleModel::integrator)
      .def_property_readonly("params", &VehicleModel::params,
                             py::return_value_policy::reference_internal)
      .def_property_readonly("state_dimension", &VehicleModel::state_dimension)
      .def("__repr__", [](const VehicleModel& v) {
        return std::string("VehicleModel(tier=") + to_string(v.tier()) +
               ", integrator=" + to_string(v.integrator()) + ")";
      });

  // ------------------------------------------------------------------- rng
  py::class_<Rng>(m, "Rng",
                  "SplitMix64. Written out rather than taken from <random> "
                  "because the standard library's distributions are not "
                  "specified to agree across implementations, which would "
                  "break the determinism claim.")
      .def(py::init<std::uint64_t>(), py::arg("seed") = 0)
      .def("next_u64", &Rng::next_u64)
      .def("uniform", py::overload_cast<>(&Rng::uniform), "[0, 1)")
      .def("uniform", py::overload_cast<double, double>(&Rng::uniform),
           py::arg("lo"), py::arg("hi"))
      .def("normal", py::overload_cast<>(&Rng::normal))
      .def("normal", py::overload_cast<double, double>(&Rng::normal),
           py::arg("mean"), py::arg("stddev"));

  m.def("derive_seed", &derive_seed, py::arg("master_seed"),
        py::arg("stream_index"),
        "One agent's stream seed. Mixed rather than added, so adjacent agents "
        "do not get correlated streams.");

  // -------------------------------------------------------------- manifest
  py::class_<AgentManifest>(m, "AgentManifest")
      .def_readonly("name", &AgentManifest::name)
      .def_readonly("tier", &AgentManifest::tier)
      .def_readonly("params_digest", &AgentManifest::params_digest)
      .def_readonly("seed", &AgentManifest::seed)
      .def_readonly("footprint_length", &AgentManifest::footprint_length,
                    "declared collision footprint, zero when none [m]")
      .def_readonly("footprint_width", &AgentManifest::footprint_width)
      .def_readonly("command_source", &AgentManifest::command_source,
                    "'policy', 'coast' or 'mailbox'")
      .def_readonly("timeout_policy", &AgentManifest::timeout_policy,
                    "for mailbox agents; empty otherwise")
      .def_readonly("status", &AgentManifest::status,
                    "'running', or 'dnf' with the cause and step below. A "
                    "result, not configuration: excluded from the "
                    "configuration digest.")
      .def_readonly("dnf_cause", &AgentManifest::dnf_cause,
                    "empty while running")
      .def_readonly("dnf_step", &AgentManifest::dnf_step,
                    "meaningless while running");

  py::class_<RunManifest>(
      m, "RunManifest",
      "SIM-06. What was simulated and what it was simulated with. Its hash is "
      "what two parties compare when they disagree about a result.")
      .def_readonly("slipx_core_version", &RunManifest::slipx_core_version)
      .def_readonly("schema_version", &RunManifest::schema_version)
      .def_readonly("dt", &RunManifest::dt)
      .def_readonly("steps", &RunManifest::steps)
      .def_readonly("integrator", &RunManifest::integrator)
      .def_readonly("master_seed", &RunManifest::master_seed)
      .def_readonly("contact_restitution", &RunManifest::contact_restitution,
                    "the contact constants are configuration, like dt: two "
                    "runs that disagree here were different races")
      .def_readonly("contact_friction", &RunManifest::contact_friction)
      .def_readonly("contact_restitution_min_speed",
                    &RunManifest::contact_restitution_min_speed)
      .def_readonly("timing_dependent_commands",
                    &RunManifest::timing_dependent_commands,
                    "True when barrier misses were decided by a wall clock: "
                    "such a run is bit-identical only when replayed from "
                    "its input log, and the determinism block says so.")
      .def_readonly("agents", &RunManifest::agents)
      .def_readonly("compiler_id", &RunManifest::compiler_id)
      .def_readonly("compiler_version", &RunManifest::compiler_version)
      .def_readonly("cxx_flags", &RunManifest::cxx_flags)
      .def_readonly("build_type", &RunManifest::build_type)
      .def_readonly("system_name", &RunManifest::system_name)
      .def_readonly("system_processor", &RunManifest::system_processor)
      .def_readonly("git_sha", &RunManifest::git_sha)
      .def_readonly("libc_id", &RunManifest::libc_id,
                    "The C library the run executed against, read at run "
                    "time. The trajectory hash tracks libm, so a run that "
                    "differs here was never entitled to agree.")
      .def_readonly("libc_version", &RunManifest::libc_version,
                    "Version as the library reports it, e.g. '2.39'. Empty on "
                    "platforms that offer no runtime version.")
      .def_readonly("trajectory_hash", &RunManifest::trajectory_hash)
      .def_readonly("agent_trajectory_hashes",
                    &RunManifest::agent_trajectory_hashes)
      .def("configuration_digest", &RunManifest::configuration_digest,
           "Digest of the setup, excluding the result. Two runs whose digests "
           "agree should produce the same trajectory hash.")
      .def("to_json", &RunManifest::to_json)
      .def("write", &RunManifest::write, py::arg("path"),
           "False if the file could not be written; does not raise, so a run "
           "that finished still reports its result.");

  // ------------------------------------------------------------ simulation
  py::class_<ContactParams>(
      m, "ContactParams",
      "The collision constants: plausible for foam bumpers on plastic "
      "shells, identified from nothing, and every document that touches "
      "contact keeps saying so. What IS promised is determinism, momentum "
      "conservation, the friction cone and mirror symmetry.")
      .def(py::init<>())
      .def_readwrite("restitution", &ContactParams::restitution,
                     "0 is a dead stop, 1 an elastic bounce [-]")
      .def_readwrite("friction", &ContactParams::friction,
                     "Coulomb friction at the contact [-]")
      .def_readwrite("restitution_min_speed",
                     &ContactParams::restitution_min_speed,
                     "below this closing speed nothing bounces; an "
                     "anti-jitter device, not physics [m/s]");

  py::class_<SimulationConfig>(m, "SimulationConfig")
      .def(py::init<>())
      .def_readwrite("dt", &SimulationConfig::dt, "fixed step [s], default 1 kHz")
      .def_readwrite("integrator", &SimulationConfig::integrator)
      .def_readwrite("master_seed", &SimulationConfig::master_seed)
      .def_readwrite("hash_stride", &SimulationConfig::hash_stride)
      .def_readwrite("schema_version", &SimulationConfig::schema_version)
      .def_readwrite("contact", &SimulationConfig::contact,
                     "agent-to-agent contact constants (ADR-0043); they "
                     "apply between agents that declare a footprint")
      .def_readwrite("barrier_timeout", &SimulationConfig::barrier_timeout,
                     "wall-clock seconds the barrier waits for a missing "
                     "mailbox entry before ruling a miss; zero polls; "
                     "ignored for Wait agents, whose barrier waits forever");

  py::class_<AgentSpec>(m, "AgentSpec")
      .def(py::init<>())
      .def_readwrite("name", &AgentSpec::name)
      .def_readwrite("tier", &AgentSpec::tier)
      .def_readwrite("params", &AgentSpec::params)
      .def_readwrite("initial_state", &AgentSpec::initial_state)
      .def_readwrite("policy", &AgentSpec::policy,
                     "callable(state, time, rng) -> DriveInput. Must be a pure "
                     "function of those three if the run is to replay.")
      .def_readwrite(
          "footprint_length", &AgentSpec::footprint_length,
          "collision footprint length, overall [m]. Both zero (the default) "
          "means no footprint: the agent touches nothing and nothing touches "
          "it. The car schema carries this as geometry.length.")
      .def_readwrite("footprint_width", &AgentSpec::footprint_width,
                     "collision footprint width, overall [m]; see "
                     "footprint_length")
      .def_readwrite("mailbox", &AgentSpec::mailbox,
                     "asynchronous command source; an agent has one source, "
                     "so a policy and a mailbox together are refused")
      .def_readwrite("timeout_policy", &AgentSpec::timeout_policy,
                     "what a missed step does; meaningless without a "
                     "mailbox");

  // ---------------------------------------------------------------- events
  py::enum_<DnfCause>(
      m, "DnfCause",
      "Why an agent stopped racing. The rollover values name the UNLOADED "
      "side: RolloverLeft means both left wheels reached zero vertical load, "
      "which is what a hard left turn does (positive ay loads the right "
      "wheels). Timeout is a barrier miss whose policy said the run ends.")
      .value("RolloverLeft", DnfCause::kRolloverLeft)
      .value("RolloverRight", DnfCause::kRolloverRight)
      .value("Timeout", DnfCause::kTimeout);

  // --------------------------------------------------------------- barrier
  py::enum_<TimeoutPolicy>(
      m, "TimeoutPolicy",
      "What the barrier does about a step whose command never arrived. Only "
      "meaningful for a mailbox-driven agent; a policy callable cannot "
      "miss.")
      .value("Wait", TimeoutPolicy::kWait,
             "block until it arrives: strict lockstep, timing cannot change "
             "the trajectory, and one hung agent hangs the race")
      .value("Freeze", TimeoutPolicy::kFreeze,
             "do not step the agent this step; it resumes where it paused")
      .value("Coast", TimeoutPolicy::kCoast,
             "step with the neutral input, like an agent with no policy")
      .value("Dnf", TimeoutPolicy::kDnf,
             "the agent is out: frozen in place, a stationary obstacle");

  py::class_<CommandMailbox, std::shared_ptr<CommandMailbox>>(
      m, "CommandMailbox",
      "The barrier's doorway: a thread-safe queue of step-tagged commands "
      "into an otherwise single-threaded simulation. The tag is the "
      "acknowledgement: post(step, input) delivers a command for that step, "
      "ack(step) says 'alive, hold my last one'. Tags strictly increase; "
      "the simulation takes the entry tagged with exactly the step it is "
      "about to compute, and anything else is a miss answered by the "
      "agent's TimeoutPolicy.")
      .def(py::init<>())
      .def("post", &CommandMailbox::post, py::arg("step"), py::arg("input"),
           py::call_guard<py::gil_scoped_release>(),
           "Refuses NaN (reserved as the log's missed-step marker) and "
           "non-increasing tags, by name.")
      .def("ack", &CommandMailbox::ack, py::arg("step"),
           py::call_guard<py::gil_scoped_release>(),
           "Acknowledge a step with no new command: hold the last one. "
           "Before anything was posted, the held command is a coast.");

  py::class_<DnfEvent>(
      m, "DnfEvent",
      "The discrete event that ended an agent's run. After it, the agent's "
      "policy is never called again, its pose is frozen where the event "
      "found it, and its velocities read zero: a stationary obstacle, not a "
      "car frozen mid-flight.")
      .def_readonly("step", &DnfEvent::step,
                    "the first step at which the state was frozen")
      .def_readonly("time", &DnfEvent::time, "step * dt [s]")
      .def_readonly("cause", &DnfEvent::cause)
      .def("__repr__", [](const DnfEvent& e) {
        std::ostringstream o;
        o << "DnfEvent(step=" << e.step << ", time=" << e.time << ", cause='"
          << to_string(e.cause) << "')";
        return o.str();
      });

  py::class_<Simulation>(
      m, "Simulation",
      "N agents, one fixed step, in lockstep. Every policy sees the world as "
      "it was at the start of the step, so a result cannot depend on the "
      "order agents were added in.")
      .def(py::init<SimulationConfig>(), py::arg("config") = SimulationConfig{})
      .def("add_agent", &Simulation::add_agent, py::arg("spec"),
           "Returns the agent's index. No upper bound on the count.")
      // The GIL is released while stepping, so a Python thread can post
      // into a mailbox while a Wait barrier blocks; pybind reacquires it
      // whenever a Python policy is actually invoked.
      .def("advance", &Simulation::advance,
           py::call_guard<py::gil_scoped_release>(),
           "One step for every agent.")
      .def("run", &Simulation::run, py::arg("steps"),
           py::call_guard<py::gil_scoped_release>())
      .def("run_for", &Simulation::run_for, py::arg("duration"),
           py::call_guard<py::gil_scoped_release>(),
           "Rounds to whole steps: a partial step would be a second step size.")
      .def("reset", &Simulation::reset,
           "Back to the initial states, clock, hashes and random streams. Does "
           "not rebuild the models, so a reset run is the same run.")
      .def("replay", &Simulation::replay, py::arg("log"),
           py::call_guard<py::gil_scoped_release>(),
           "Re-run from a recorded input sequence, ignoring the policies. What "
           "a leaderboard appeal does when the policies are gone.")
      .def_property_readonly("agent_count", &Simulation::agent_count)
      .def_property_readonly("time", &Simulation::time,
                             "steps * dt, never an accumulated sum [s]")
      .def_property_readonly("step_count", &Simulation::step_count)
      .def_property_readonly("dt", &Simulation::dt)
      .def(
          "state",
          [](Simulation& self, std::size_t i) -> VehicleState& {
            return self.state(i);
          },
          py::arg("index"), py::return_value_policy::reference_internal)
      .def("diagnostics", &Simulation::diagnostics, py::arg("index"),
           py::return_value_policy::reference_internal)
      .def("model", &Simulation::model, py::arg("index"),
           py::return_value_policy::reference_internal)
      .def("rng", &Simulation::rng, py::arg("index"),
           py::return_value_policy::reference_internal)
      .def("agent_running", &Simulation::agent_running, py::arg("index"),
           "False once the agent has DNF'd. A DNF'd agent freezes in place "
           "and stays in the world as a stationary obstacle.")
      .def("dnf", &Simulation::dnf, py::arg("index"),
           "The DnfEvent that ended the agent's run, or None while it is "
           "still running.")
      .def("trajectory_hash", &Simulation::trajectory_hash)
      .def("agent_trajectory_hash", &Simulation::agent_trajectory_hash,
           py::arg("index"))
      .def("manifest", &Simulation::manifest)
      .def("set_input_logging", &Simulation::set_input_logging,
           py::arg("enabled"))
      .def_property_readonly("input_logging", &Simulation::input_logging)
      .def("input_log", &Simulation::input_log,
           "Flat, step-major: entry (step * agent_count + agent).");

  // ----------------------------------------------------- sensors (ADR-0047)
  //
  // The rig observes a simulation it cannot write to, so sensing is free by
  // construction and stays out of the manifest and the digest. The world is
  // a callable(agent, pose, bearing) -> Hit; a Python callable works and is
  // called once per ray, so a native world is where the speed is when it
  // matters. collect() deliberately keeps the GIL: the world may be Python.

  py::class_<sense::Pose>(m, "Pose",
                          "An emitter pose in the world frame (ISO 8855: x "
                          "forward, y left, yaw positive anticlockwise).")
      .def(py::init<>())
      .def_readwrite("x", &sense::Pose::x, "[m]")
      .def_readwrite("y", &sense::Pose::y, "[m]")
      .def_readwrite("yaw", &sense::Pose::yaw, "[rad]");

  py::class_<sense::Hit>(
      m, "Hit",
      "What a ray found. material_dropout scales the configured dropout "
      "probability for this target: 1 is an ordinary surface, 0 never "
      "drops. A world that knows nothing about materials returns 1 and is "
      "honest.")
      .def(py::init<>())
      .def_readwrite("hit", &sense::Hit::hit)
      .def_readwrite("range", &sense::Hit::range, "[m]")
      .def_readwrite("material_dropout", &sense::Hit::material_dropout,
                     "[-]");

  py::class_<sense::LidarSpec>(
      m, "LidarSpec",
      "The 2D scanning LiDAR. Rays are spread over one revolution in time, "
      "so a scan takes as long as it takes and motion distortion emerges "
      "from the pose at each ray's own timestamp.")
      .def(py::init<>())
      .def_readwrite("rate_hz", &sense::LidarSpec::rate_hz,
                     "revolutions per second; one scan each [Hz]")
      .def_readwrite("rays", &sense::LidarSpec::rays)
      .def_readwrite("angle_min", &sense::LidarSpec::angle_min, "[rad]")
      .def_readwrite("angle_max", &sense::LidarSpec::angle_max, "[rad]")
      .def_readwrite("range_min", &sense::LidarSpec::range_min, "[m]")
      .def_readwrite("range_max", &sense::LidarSpec::range_max, "[m]")
      .def_readwrite("latency_s", &sense::LidarSpec::latency_s, "[s]")
      .def_readwrite("latency_jitter_s", &sense::LidarSpec::latency_jitter_s,
                     "uniform half-width, at most latency_s [s]")
      .def_readwrite("noise_base_m", &sense::LidarSpec::noise_base_m, "[m]")
      .def_readwrite("noise_per_metre", &sense::LidarSpec::noise_per_metre,
                     "[-]")
      .def_readwrite("dropout_probability",
                     &sense::LidarSpec::dropout_probability, "[-]");

  py::class_<sense::Ray>(
      m, "Ray",
      "One ray. A dropped or out-of-range ray is valid == False and its "
      "range is NaN, never zero: zero is a wall against the mast.")
      .def_readonly("time", &sense::Ray::time,
                    "when this ray was emitted, simulation time [s]")
      .def_readonly("angle", &sense::Ray::angle, "sensor-frame bearing [rad]")
      .def_readonly("range", &sense::Ray::range, "NaN when not valid [m]")
      .def_readonly("valid", &sense::Ray::valid);

  py::class_<sense::Scan>(
      m, "Scan",
      "One scan. start_time is when the first ray was emitted, stamp_time "
      "when the scan became available to a consumer; the gap is the "
      "revolution plus the latency, and confusing them puts a stack's "
      "timestamps out by a scan period.")
      .def_readonly("start_time", &sense::Scan::start_time, "[s]")
      .def_readonly("stamp_time", &sense::Scan::stamp_time, "[s]")
      .def_readonly("rays", &sense::Scan::rays);

  py::class_<sense::ImuSpec>(
      m, "ImuSpec",
      "The IMU's three error mechanisms on their three timescales: white "
      "noise as a density, a bias random walk averaging does not remove, "
      "and fixed scale errors that are properties of the unit.")
      .def(py::init<>())
      .def_readwrite("accel_noise_density", &sense::ImuSpec::accel_noise_density,
                     "[m/s^2 / sqrt(Hz)]")
      .def_readwrite("gyro_noise_density", &sense::ImuSpec::gyro_noise_density,
                     "[rad/s / sqrt(Hz)]")
      .def_readwrite("accel_bias_walk", &sense::ImuSpec::accel_bias_walk,
                     "[m/s^2 / sqrt(s)]")
      .def_readwrite("gyro_bias_walk", &sense::ImuSpec::gyro_bias_walk,
                     "[rad/s / sqrt(s)]")
      .def_readwrite("accel_scale_error", &sense::ImuSpec::accel_scale_error,
                     "0.01 reads 1 per cent high [-]")
      .def_readwrite("gyro_scale_error", &sense::ImuSpec::gyro_scale_error,
                     "[-]")
      .def_readwrite("accel_bias_x", &sense::ImuSpec::accel_bias_x, "[m/s^2]")
      .def_readwrite("accel_bias_y", &sense::ImuSpec::accel_bias_y, "[m/s^2]")
      .def_readwrite("gyro_bias_z", &sense::ImuSpec::gyro_bias_z, "[rad/s]");

  py::class_<sense::ImuSample>(
      m, "ImuSample",
      "One IMU sample: specific force in the body frame (a level car's az "
      "reads standard gravity plus the unit's errors) and the yaw rate.")
      .def_readonly("time", &sense::ImuSample::time, "[s]")
      .def_readonly("ax", &sense::ImuSample::ax, "[m/s^2]")
      .def_readonly("ay", &sense::ImuSample::ay, "[m/s^2]")
      .def_readonly("az", &sense::ImuSample::az, "[m/s^2]")
      .def_readonly("yaw_rate", &sense::ImuSample::yaw_rate, "[rad/s]");

  py::class_<sense::EncoderSpec>(
      m, "EncoderSpec",
      "Wheel encoders and the odometry built from them. Noiseless on "
      "purpose: an encoder has a quantisation, not a noise floor, and its "
      "interesting error is the slip, which is already in the wheel "
      "speeds. The radius is deliberately allowed to disagree with the "
      "car's own.")
      .def(py::init<>())
      .def_readwrite("counts_per_revolution",
                     &sense::EncoderSpec::counts_per_revolution,
                     "after gearing and quadrature decoding [-]")
      .def_readwrite("wheel_radius", &sense::EncoderSpec::wheel_radius, "[m]")
      .def_readwrite("wheels_used", &sense::EncoderSpec::wheels_used,
                     "which wheels the odometry averages: FL, FR, RL, RR");

  py::class_<sense::EncoderSample>(
      m, "EncoderSample",
      "What the encoders say, which is not what happened: distance and "
      "speed are what the wheels turned times a radius, and they diverge "
      "from ground truth by exactly the slip.")
      .def_readonly("time", &sense::EncoderSample::time, "[s]")
      .def_readonly("counts", &sense::EncoderSample::counts,
                    "cumulative, signed, per wheel")
      .def_readonly("wheel_speed", &sense::EncoderSample::wheel_speed,
                    "quantised, over the last interval [rad/s]")
      .def_readonly("distance", &sense::EncoderSample::distance,
                    "cumulative, what the encoders believe [m]")
      .def_readonly("speed", &sense::EncoderSample::speed, "[m/s]");

  py::class_<LidarSensor>(m, "LidarSensor",
                          "One LiDAR instance on one agent: the model's spec "
                          "plus the rig's phase.")
      .def(py::init<>())
      .def_readwrite("name", &LidarSensor::name)
      .def_readwrite("spec", &LidarSensor::spec)
      .def_readwrite("phase", &LidarSensor::phase,
                     "fraction of the period before the first scan [-]");

  py::class_<ImuSensor>(m, "ImuSensor",
                        "One IMU instance: the model's spec plus the "
                        "schedule and transport the rig owns.")
      .def(py::init<>())
      .def_readwrite("name", &ImuSensor::name)
      .def_readwrite("spec", &ImuSensor::spec)
      .def_readwrite("rate_hz", &ImuSensor::rate_hz, "[Hz]")
      .def_readwrite("phase", &ImuSensor::phase, "[-]")
      .def_readwrite("latency_s", &ImuSensor::latency_s, "[s]")
      .def_readwrite("latency_jitter_s", &ImuSensor::latency_jitter_s,
                     "uniform half-width, at most latency_s [s]");

  py::class_<EncoderSensor>(m, "EncoderSensor",
                            "One wheel-encoder odometry instance, scheduled "
                            "and delivered like the IMU. Needs L2 or above; "
                            "below L2 the model never writes wheel speeds "
                            "and the rig refuses it by name.")
      .def(py::init<>())
      .def_readwrite("name", &EncoderSensor::name)
      .def_readwrite("spec", &EncoderSensor::spec)
      .def_readwrite("rate_hz", &EncoderSensor::rate_hz, "[Hz]")
      .def_readwrite("phase", &EncoderSensor::phase, "[-]")
      .def_readwrite("latency_s", &EncoderSensor::latency_s, "[s]")
      .def_readwrite("latency_jitter_s", &EncoderSensor::latency_jitter_s,
                     "[s]");

  py::class_<AgentSensors>(
      m, "AgentSensors",
      "Everything one agent carries. Empty lists are the cheap opponent. "
      "The list attributes copy on read, so assign whole lists rather than "
      "appending to what you read back.")
      .def(py::init<>())
      .def_readwrite("lidars", &AgentSensors::lidars)
      .def_readwrite("imus", &AgentSensors::imus)
      .def_readwrite("encoders", &AgentSensors::encoders);

  py::class_<ImuReading>(m, "ImuReading",
                         "A delivered IMU message: the sample, which carries "
                         "the instant it measured, plus when it became "
                         "visible.")
      .def_readonly("sample", &ImuReading::sample)
      .def_readonly("stamp_time", &ImuReading::stamp_time, "[s]");

  py::class_<OdometryReading>(m, "OdometryReading",
                              "A delivered odometry message; see ImuReading.")
      .def_readonly("sample", &OdometryReading::sample)
      .def_readonly("stamp_time", &OdometryReading::stamp_time, "[s]");

  // The scene's track and the composed racing world (ADR-0049), bound just
  // far enough to build both from a directory the Python loader has
  // already validated; slipx.load_scene_track is the friendly entrance.
  py::class_<scene::Track>(
      m, "SceneTrack",
      "The scene's track: geometry plus a declared surface. Build one with "
      "slipx.load_scene_track, which validates the directory through "
      "slipx_schema first; this static build is the raw seam it uses.")
      .def_static(
          "build",
          [](const std::string& centreline_csv, const std::string& name,
             const std::string& surface, bool closed,
             const std::string& geometry_source,
             const std::string& geometry_licence,
             const std::string& provenance_label,
             const std::vector<std::pair<std::string, std::string>>& tyres) {
            scene::Centreline geometry =
                scene::Centreline::from_file(centreline_csv);
            scene::TrackManifest manifest;
            manifest.name = name;
            manifest.surface = surface;
            manifest.closed = closed;
            manifest.geometry_source = geometry_source;
            manifest.geometry_licence = geometry_licence;
            manifest.provenance_label = provenance_label;
            std::vector<scene::TyrePair> pairs;
            pairs.reserve(tyres.size());
            for (const auto& tyre : tyres) {
              pairs.push_back(scene::TyrePair{tyre.first, tyre.second});
            }
            return scene::Track::build(geometry, manifest, pairs);
          },
          py::arg("centreline_csv"), py::arg("name"), py::arg("surface"),
          py::arg("closed"), py::arg("geometry_source"),
          py::arg("geometry_licence"), py::arg("provenance_label"),
          py::arg("tyre_pairs"),
          "tyre_pairs is the list of (compound, surface) pairs the cars "
          "bring; the build refuses a surface no tyre was identified on.")
      .def_property_readonly("length", &scene::Track::length, "[m]")
      .def_property_readonly("closed", &scene::Track::is_closed);

  py::class_<TrackWorld>(
      m, "TrackWorld",
      "The world a racing sensor rig sees (ADR-0049): the nearer of the "
      "track's walls and the simulation's other cars, the asker skipped, "
      "the boxes exactly the footprints the contact pass collides, refit "
      "once per step. Hand it to SensorRig and rays never cross into "
      "Python. max_range bounds every cast: set it to the longest range "
      "any sensor will ask for.")
      .def(py::init<const scene::Track&, const Simulation&, double>(),
           py::arg("track"), py::arg("sim"), py::arg("max_range"),
           py::keep_alive<1, 2>(), py::keep_alive<1, 3>(),
           "Build after the last add_agent: a world missing a car refuses "
           "to answer rather than hiding an obstacle.")
      .def("__call__", &TrackWorld::operator(), py::arg("agent"),
           py::arg("origin"), py::arg("bearing"),
           "One ray, answered the way the rig would see it.");

  py::class_<SensorRig>(
      m, "SensorRig",
      "Per-agent sensors running against a simulation they can only observe "
      "(ADR-0047). An agent with a full suite drives bit for bit the "
      "trajectory of its bare twin; sensors stay out of the manifest and "
      "the configuration digest because they cannot change a trajectory. "
      "Call collect() after every advance(); a sample becomes visible at "
      "its instant plus latency plus jitter, and take_* drains while "
      "latest_* keeps the newest.")
      // The native-world overload first, so a TrackWorld binds by type
      // rather than being wrapped as a Python callable and paying an
      // interpreter round trip per ray.
      .def(py::init([](const Simulation& sim, const TrackWorld& world,
                       std::uint64_t seed) {
             return new SensorRig(sim, world.function(), seed);
           }),
           py::arg("sim"), py::arg("world"), py::arg("seed") = 0,
           py::keep_alive<1, 2>(), py::keep_alive<1, 3>(),
           "Native world (ADR-0049): rays never enter Python. The rig keeps "
           "both the simulation and the world alive.")
      .def(py::init<const Simulation&, WorldFunction, std::uint64_t>(),
           py::arg("sim"), py::arg("world") = WorldFunction{},
           py::arg("seed") = 0,
           // The rig holds a reference to the simulation for its lifetime.
           py::keep_alive<1, 2>(),
           "world is callable(agent, Pose, bearing) -> Hit, called once per "
           "ray with the asking agent's index so the caller can apply the "
           "self-skip; None is legal for a rig with no LiDARs. seed is the "
           "rig's own: sensor streams derive per agent and per instance "
           "from it, and the simulation's streams are never touched.")
      .def("attach", &SensorRig::attach, py::arg("agent"), py::arg("sensors"),
           "Before the first collect() only. Refuses by name a bad index, a "
           "duplicate name, a bad schedule, a jitter exceeding its latency, "
           "a LiDAR without a world, or an encoder below L2; a refused "
           "attach leaves the rig unchanged.")
      .def("collect", &SensorRig::collect,
           "Read everything due up to the simulation's current time. Keeps "
           "the GIL: the world may be a Python callable.")
      .def("take_scans", &SensorRig::take_scans, py::arg("agent"),
           py::arg("name"), "Deliveries since the last take, oldest first.")
      .def("take_imu", &SensorRig::take_imu, py::arg("agent"), py::arg("name"))
      .def("take_odometry", &SensorRig::take_odometry, py::arg("agent"),
           py::arg("name"))
      .def("latest_scan", &SensorRig::latest_scan, py::arg("agent"),
           py::arg("name"), "The newest delivered scan, or None.")
      .def("latest_imu", &SensorRig::latest_imu, py::arg("agent"),
           py::arg("name"))
      .def("latest_odometry", &SensorRig::latest_odometry, py::arg("agent"),
           py::arg("name"));

  // ------------------------------------------------------------ manoeuvres
  py::class_<StepSteerSpec>(m, "StepSteerSpec")
      .def(py::init<>())
      .def_readwrite("target_speed", &StepSteerSpec::target_speed, "[m/s]")
      .def_readwrite("steer", &StepSteerSpec::steer, "[rad]")
      .def_readwrite("t_step", &StepSteerSpec::t_step, "[s]")
      .def_readwrite("speed_gain", &StepSteerSpec::speed_gain);

  m.def("step_steer", &step_steer, py::arg("spec") = StepSteerSpec{},
        "The P0 exit-gate manoeuvre: straight run-up, then one steer step "
        "held to the end.");

  m.def("hold_speed", &hold_speed, py::arg("state"), py::arg("target"),
        py::arg("gain") = 4.0,
        "Proportional speed hold on the body longitudinal component.");

  py::class_<ConformanceSpec>(m, "ConformanceSpec")
      .def(py::init<>())
      .def_readwrite("tier", &ConformanceSpec::tier)
      .def_readwrite("integrator", &ConformanceSpec::integrator)
      .def_readwrite("dt", &ConformanceSpec::dt)
      .def_readwrite("duration", &ConformanceSpec::duration)
      .def_readwrite("initial_speed", &ConformanceSpec::initial_speed)
      .def_readwrite("seed", &ConformanceSpec::seed);

  m.def("make_conformance_run", &make_conformance_run,
        py::arg("spec") = ConformanceSpec{},
        "The canonical scenario behind the published reference hashes. Uses "
        "the VehicleParams defaults, which are PROVISIONAL and describe no "
        "measured car: it is a determinism check, not a physics claim.");

  // ------------------------------------------------------------------ hash
  py::class_<TrajectoryHash>(
      m, "TrajectoryHash",
      "FNV-1a 64 over the bit patterns of the state doubles. Chosen for being "
      "trivial to reimplement correctly elsewhere, since every binding must "
      "agree with it digit for digit. Not cryptographic and not used as such: "
      "it detects divergence, it does not resist forgery.")
      .def(py::init<>())
      .def("update", py::overload_cast<double>(&TrajectoryHash::update),
           py::arg("value"))
      .def("update_state",
           py::overload_cast<const VehicleState&>(&TrajectoryHash::update),
           py::arg("state"))
      .def("update_text",
           [](TrajectoryHash& h, const std::string& text) { h.update(text); },
           py::arg("text"))
      .def("value", &TrajectoryHash::value)
      .def("hex", &TrajectoryHash::hex);

  m.def("hash_text", &hash_text, py::arg("text"));

  // Translated so that Python users get ValueError rather than an opaque
  // RuntimeError from a failed model construction.
  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    } catch (const std::invalid_argument& e) {
      PyErr_SetString(PyExc_ValueError, e.what());
    } catch (const std::out_of_range& e) {
      PyErr_SetString(PyExc_IndexError, e.what());
    }
  });
}
