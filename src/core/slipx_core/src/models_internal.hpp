// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Internal factory declarations. Not installed: the tier classes are reachable
// only through VehicleModel::create, so adding, renaming or re-deriving a tier
// is not an ABI change for anybody embedding the core.

#ifndef SLIPX_MODELS_INTERNAL_HPP
#define SLIPX_MODELS_INTERNAL_HPP

#include <memory>

#include "slipx/vehicle_model.hpp"

namespace slipx {
namespace internal {

std::unique_ptr<VehicleModel> make_l0(const VehicleParams& p, Integrator i);
std::unique_ptr<VehicleModel> make_l1(const VehicleParams& p, Integrator i);

// Shared by both tiers: fills every field of a diagnostics block with NaN, so
// that a tier only has to write the quantities it actually represents and
// anything it does not is loud rather than plausibly zero.
void reset_diagnostics(StepDiagnostics& d, Tier tier);

}  // namespace internal
}  // namespace slipx

#endif  // SLIPX_MODELS_INTERNAL_HPP
