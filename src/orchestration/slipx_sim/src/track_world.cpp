// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/sim/track_world.hpp"

#include <cmath>
#include <stdexcept>

namespace slipx {
namespace sim {

TrackWorld::TrackWorld(const scene::Track& track, const Simulation& sim,
                       double max_range)
    : sim_(&sim), walls_(track), max_range_(max_range) {
  if (!(max_range > 0.0) || !std::isfinite(max_range)) {
    throw std::invalid_argument(
        "slipx_sim track world: max_range must be positive and finite; set "
        "it to the longest range any sensor will ask for [m]");
  }

  // Gathered once: a footprint cannot change after add_agent, and reading
  // it per ray would buy nothing but cache misses.
  footprints_.resize(sim.agent_count());
  for (std::size_t i = 0; i < footprints_.size(); ++i) {
    Footprint& footprint = footprints_[i];
    footprint.present = sim.has_footprint(i);
    if (footprint.present) {
      footprint.half_length = sim.footprint_half_length(i);
      footprint.half_width = sim.footprint_half_width(i);
      footprint.centre_offset = sim.footprint_centre_offset(i);
    }
  }
  overlay_.resize(footprints_.size());
}

void TrackWorld::refit() const {
  if (sim_->agent_count() != footprints_.size()) {
    throw std::logic_error(
        "slipx_sim track world: agents were added after this world was "
        "built, and a world missing a car is an invisible obstacle. Build "
        "the world after the last add_agent.");
  }
  const std::uint64_t step = sim_->step_count();
  if (refit_once_ && step == refit_step_) return;

  for (std::size_t i = 0; i < footprints_.size(); ++i) {
    const Footprint& footprint = footprints_[i];
    if (!footprint.present) {
      overlay_.set(i, 0.0, 0.0, 0.0, 0.0, 0.0, false);
      continue;
    }
    // The same box the contact pass collides: centred between the axles,
    // not on the CoG (ADR-0043). A DNF'd car keeps its box; a wreck is an
    // obstacle to sensors exactly as it is to bumpers (ADR-0042).
    const VehicleState& state = sim_->state(i);
    const double heading_x = std::cos(state.yaw);
    const double heading_y = std::sin(state.yaw);
    overlay_.set(i, state.pos.x + heading_x * footprint.centre_offset,
                 state.pos.y + heading_y * footprint.centre_offset, state.yaw,
                 footprint.half_length, footprint.half_width, true);
  }
  refit_step_ = step;
  refit_once_ = true;
}

sense::Hit TrackWorld::operator()(std::size_t agent, const sense::Pose& origin,
                                  double bearing) const {
  refit();

  const scene::RayHit wall =
      walls_.cast(origin.x, origin.y, bearing, max_range_);
  const scene::OverlayHit car =
      overlay_.cast(origin.x, origin.y, bearing, max_range_, agent);

  sense::Hit hit;
  if (wall.hit && (!car.hit || wall.range <= car.range)) {
    hit.hit = true;
    hit.range = wall.range;
  } else if (car.hit) {
    hit.hit = true;
    hit.range = car.range;
  }
  return hit;
}

WorldFunction TrackWorld::function() const {
  return [this](std::size_t agent, const sense::Pose& origin, double bearing) {
    return (*this)(agent, origin, bearing);
  };
}

}  // namespace sim
}  // namespace slipx
