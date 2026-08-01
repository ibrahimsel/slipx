// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Tier factory and the shared diagnostics reset.

#include <cstdio>
#include <cstdlib>
#include <limits>

#if defined(__cpp_exceptions) && __cpp_exceptions
#include <stdexcept>
#endif

#include "models_internal.hpp"

namespace slipx {
namespace internal {

void reset_diagnostics(StepDiagnostics& d, Tier tier) {
  // NaN, not zero. A tier that cannot represent a quantity leaves it NaN, so
  // that a plot of it is empty rather than flat at zero and quietly believed
  // (see state.hpp).
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (unsigned i = 0; i < kWheelCount; ++i) {
    d.alpha[i] = nan;
    d.kappa[i] = nan;
    d.fx[i] = nan;
    d.fy[i] = nan;
    d.fz[i] = nan;
    d.tyre_saturated[i] = false;
  }
  d.alpha_front = nan;
  d.alpha_rear = nan;
  d.fy_front = nan;
  d.fy_rear = nan;
  d.fz_front = nan;
  d.fz_rear = nan;
  d.ax = nan;
  d.ay = nan;
  d.load_transfer_long = nan;
  d.load_transfer_lat = nan;
  d.steer_saturated = false;
  d.accel_saturated = false;
  d.speed_saturated = false;
  d.tier = static_cast<int>(tier);
}

namespace {

const char* tier_availability(Tier tier) {
  switch (tier) {
    case Tier::L0_Kinematic:
    case Tier::L1_Bicycle:
      return nullptr;
    case Tier::L2_DoubleTrack:
      return "Tier L2_DoubleTrack is not implemented yet. CORE-02 places it "
             "in P1; until it lands, asking for it returns an error rather "
             "than silently substituting L1, because a trajectory labelled "
             "L2 that is actually L1 is worse than no trajectory.";
    case Tier::L3_Extended:
      return "Tier L3_Extended is not implemented yet (CORE-16, P4).";
  }
  return "Unknown tier.";
}

}  // namespace
}  // namespace internal

std::unique_ptr<VehicleModel> VehicleModel::try_create(
    Tier tier, const VehicleParams& params, Integrator integrator,
    const char** reason) noexcept {
  if (reason != nullptr) *reason = nullptr;

  if (const char* why = internal::tier_availability(tier)) {
    if (reason != nullptr) *reason = why;
    return nullptr;
  }
  if (const char* why = validate(params)) {
    if (reason != nullptr) *reason = why;
    return nullptr;
  }

  switch (tier) {
    case Tier::L0_Kinematic:
      return internal::make_l0(params, integrator);
    case Tier::L1_Bicycle:
      return internal::make_l1(params, integrator);
    default:
      return nullptr;  // unreachable: tier_availability rejected it above
  }
}

std::unique_ptr<VehicleModel> VehicleModel::create(Tier tier,
                                                   const VehicleParams& params,
                                                   Integrator integrator) {
  const char* reason = nullptr;
  auto model = try_create(tier, params, integrator, &reason);
  if (model) return model;

#if defined(__cpp_exceptions) && __cpp_exceptions
  throw std::invalid_argument(reason != nullptr ? reason
                                                : "slipx: cannot create model");
#else
  // Built with -fno-exceptions. Callers in that configuration are expected to
  // use try_create; reaching here means a contract violation that cannot be
  // reported, and continuing would mean returning a null model into a
  // simulation loop.
  std::fprintf(stderr, "slipx: %s\n",
               reason != nullptr ? reason : "cannot create model");
  std::abort();
#endif
}

}  // namespace slipx
