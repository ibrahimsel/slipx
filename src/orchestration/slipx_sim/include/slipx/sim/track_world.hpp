// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The world a racing sensor rig sees: the track's walls and the
// simulation's other cars, composed once here instead of once per caller
// (ADR-0049).
//
// This is the seam ADR-0037 promised: the scene owns geometry, the sensors
// own timing and noise, and they meet in slipx_sim, which is allowed to
// know about both. A ray is answered with the nearer of the wall hit (the
// grid-accelerated cast the benchmarks measure) and the car hit (the agent
// overlay of ADR-0045, with the asking car skipped, because an emitter does
// not see its own body). The boxes are the same footprints the contact pass
// collides, centred between the axles rather than on the CoG (ADR-0043), so
// what a LiDAR sees touching is what the physics says is touching; a DNF'd
// car keeps its box, because a wreck is an obstacle to sensors exactly as
// it is to bumpers (ADR-0042).
//
// The overlay is refit lazily, once per simulation step, on the first ray
// that arrives after an advance: rays within one step see one consistent
// world, and other agents appear at step resolution, which is the stated
// approximation of ADR-0047 (their poses between steps are not defined by
// the simulation).
//
// Not thread-safe: the lazy refit writes scratch state under a const call,
// the same trade the wall grid documents. One rig, one thread, which is the
// simulation's own arrangement.

#ifndef SLIPX_SIM_TRACK_WORLD_HPP
#define SLIPX_SIM_TRACK_WORLD_HPP

#include <cstddef>
#include <cstdint>

#include "slipx/scene/broadphase.hpp"
#include "slipx/scene/raycast.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/sim/sensor_rig.hpp"

namespace slipx {
namespace sim {

class TrackWorld {
 public:
  // `max_range` bounds every cast: walls and cars beyond it do not exist
  // to this world, so set it to the longest range any sensor will ask for.
  // Throws std::invalid_argument if it is not positive and finite. The
  // track and the simulation are referenced, not copied, and must outlive
  // this object.
  TrackWorld(const scene::Track& track, const Simulation& sim,
             double max_range);

  // The rig's world signature: the nearer of wall and car, the asker
  // skipped. Callable directly, or through function() for the rig.
  sense::Hit operator()(std::size_t agent, const sense::Pose& origin,
                        double bearing) const;

  // A WorldFunction referring to *this; the TrackWorld must outlive it.
  WorldFunction function() const;

 private:
  void refit() const;

  const Simulation* sim_;
  scene::Walls walls_;
  double max_range_ = 0.0;

  // Cached footprints, gathered once: a spec cannot change after add_agent.
  struct Footprint {
    double half_length = 0.0;
    double half_width = 0.0;
    double centre_offset = 0.0;
    bool present = false;
  };
  std::vector<Footprint> footprints_;

  mutable scene::AgentOverlay overlay_;
  // The step the overlay was last refit at, so rays within one step share
  // one consistent world. The sentinel forces the first refit, including
  // for rays cast before the first advance.
  mutable std::uint64_t refit_step_ = static_cast<std::uint64_t>(-1);
  mutable bool refit_once_ = false;
};

}  // namespace sim
}  // namespace slipx

#endif  // SLIPX_SIM_TRACK_WORLD_HPP
