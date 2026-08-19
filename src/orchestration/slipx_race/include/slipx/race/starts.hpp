// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Placing cars on a track: the pose at an arc length, and the start
// procedures built from it.

#ifndef SLIPX_RACE_STARTS_HPP
#define SLIPX_RACE_STARTS_HPP

#include <cstddef>

#include "slipx/scene/track.hpp"
#include "slipx/sim/simulation.hpp"

namespace slipx {
namespace race {

// The centreline pose at arc length s: position and the direction of
// travel. On a closed track s wraps; on an open one it is clamped to the
// ends, because a restart position past the end of the world is a caller
// error better absorbed than amplified.
struct TrackPose {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;   // [rad]
};

TrackPose pose_at(const scene::Track& track, double s);

// Put one car on the track: at arc length s, `lateral` metres to the left
// of the centreline, pointing along it, moving at `speed` along its own
// heading, with the transient states (yaw rate, lateral velocity, steer,
// tyre lag, wheel speeds) reset to match. This is the teleport the mutable
// state accessor exists for; it does not touch the input log, so a run
// that places cars is replayed WITH its race controller, not from the log
// alone.
void place_on_track(sim::Simulation& sim, std::size_t agent,
                    const scene::Track& track, double s, double lateral,
                    double speed);

// The ruleset's standing start (2.5.1.9.1-2): side by side at the line,
// separated by one car width, at rest. `left_car` takes the left slot,
// which is the choice rule 2.5.1.9.2 gives the higher qualifier.
void grid_start(sim::Simulation& sim, const scene::Track& track,
                std::size_t left_car, std::size_t right_car, double line_s,
                double gap);

// The same geometry at speed. NOT a ruleset procedure: the pinned revision
// defines only the standing start, and this exists because scenario authors
// ask for it; a race that uses it is not racing under the pinned rules and
// should say so.
void rolling_start(sim::Simulation& sim, const scene::Track& track,
                   std::size_t left_car, std::size_t right_car, double line_s,
                   double gap, double speed);

}  // namespace race
}  // namespace slipx

#endif  // SLIPX_RACE_STARTS_HPP
