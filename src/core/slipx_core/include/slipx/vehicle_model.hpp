// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The one interface every fidelity tier is exposed through (CORE-02).
//
// The properties that matter, and that the tests assert:
//
//   step is const           no hidden state, so N instances are trivially
//                           parallel and snapshot/restore is a memcpy
//                           (CORE-03)
//   no allocation in step   verified by an operator new counter in
//                           tests/test_no_alloc.cpp (CORE-01)
//   no clock, no RNG        determinism is a property of the code, not of a
//                           configuration flag (CORE-04)
//   one state type          a controller moves between tiers unchanged, which
//                           is the point of having tiers (SRS 2.4)

#ifndef SLIPX_VEHICLE_MODEL_HPP
#define SLIPX_VEHICLE_MODEL_HPP

#include <memory>

#include "slipx/integrator.hpp"
#include "slipx/params.hpp"
#include "slipx/state.hpp"

namespace slipx {

// Fidelity levels, selected at construction. Not to be confused with the
// roadmap phases P0-P5.
enum class Tier {
  // 4 states: x, y, yaw, v. Ackermann geometry, velocity aligned with the
  // wheel heading. No tyre behaviour at all: mass, CoG, tyres and drivetrain
  // do not affect the trajectory. For mass RL rollouts and as an MPC internal
  // model.
  L0_Kinematic,
  // 6 states: adds lateral velocity and yaw rate. Sideslip, yaw dynamics,
  // understeer gradient, step-steer transient, linear tyres. Lateral force is
  // clipped rather than saturated, so there is no spin. CoG height is inert.
  L1_Bicycle,
  // 13 states. Load transfer, MF-lite, combined slip, differential and drive
  // layout, ESC torque curve with battery sag, and servo dynamics. The
  // default tier and the one system ID fits against.
  L2_DoubleTrack,
  // L2 plus thermal and suspension states. P4.
  L3_Extended
};

inline const char* to_string(Tier t) {
  switch (t) {
    case Tier::L0_Kinematic: return "L0_Kinematic";
    case Tier::L1_Bicycle: return "L1_Bicycle";
    case Tier::L2_DoubleTrack: return "L2_DoubleTrack";
    case Tier::L3_Extended: return "L3_Extended";
  }
  return "unknown";
}

class VehicleModel {
 public:
  virtual ~VehicleModel() = default;

  // Advance state by dt. const, and reads nothing outside its arguments.
  //
  // out may be nullptr, which is the cheap path. dt is a caller-chosen fixed
  // step, not a measured elapsed time; the core has no clock to measure one
  // with and would not use it if it did (CORE-04).
  virtual void step(VehicleState& state, const DriveInput& input, double dt,
                    StepDiagnostics* out = nullptr) const = 0;

  virtual Tier tier() const = 0;

  // The parameters this model was constructed with. Returned by reference and
  // never mutated: a model is immutable after construction, which is what
  // makes sharing one across threads safe without a lock.
  virtual const VehicleParams& params() const = 0;

  virtual Integrator integrator() const = 0;

  // Number of states the tier actually integrates. Diagnostic, and used by
  // the manifest so a replay cannot silently be compared against a different
  // model order.
  virtual std::size_t state_dimension() const = 0;

  // Throws std::invalid_argument if the tier is not implemented yet or the
  // parameters are physically impossible (see validate() in params.hpp).
  // Construction is not the hot path, so throwing here costs nothing that
  // matters; step itself is noexcept in practice and never allocates.
  //
  // Callers built with -fno-exceptions should use try_create.
  static std::unique_ptr<VehicleModel> create(
      Tier tier, const VehicleParams& params,
      Integrator integrator = Integrator::kRK4);

  // Non-throwing counterpart. Returns nullptr and, if reason is non-null, sets
  // it to a static message explaining why.
  static std::unique_ptr<VehicleModel> try_create(
      Tier tier, const VehicleParams& params,
      Integrator integrator = Integrator::kRK4,
      const char** reason = nullptr) noexcept;
};

}  // namespace slipx

#endif  // SLIPX_VEHICLE_MODEL_HPP
