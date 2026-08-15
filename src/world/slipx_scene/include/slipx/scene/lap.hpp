// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Counting laps, per agent.
//
// The obvious implementation watches for the car crossing the start line and
// adds one. It is also the one that miscounts, in three ways that all show up
// in a real race: a car that crosses the line sideways crosses it twice, a
// car that spins and reverses over it gains a lap it did not drive, and a car
// nudged over the line while stationary can gain several.
//
// What is counted here instead is progress. Each update measures how far
// along the centreline the car moved since the last one, signed, and adds it
// to a running total; the lap count is that total divided by the lap length.
// Crossing the line backwards subtracts, because the car really did go
// backwards, and a car wobbling on the line accumulates nothing. Direction
// falls out rather than being special-cased, which is the whole reason for
// doing it this way.
//
// One counter per agent. It holds the agent's own history and nothing about
// the track, which it borrows.

#ifndef SLIPX_SCENE_LAP_HPP
#define SLIPX_SCENE_LAP_HPP

#include "slipx/scene/projection.hpp"
#include "slipx/scene/track.hpp"

namespace slipx {
namespace scene {

class LapCounter {
 public:
  // `limit_tolerance` widens the track limits corridor on both sides, in
  // metres, and is required rather than defaulted: zero is a rule, not an
  // absence. It is per agent because a race may hold a class of car to a
  // different standard from the one beside it.
  //
  // The track is borrowed and must outlive the counter. It is not copied
  // because a track is a few hundred points per agent otherwise, and it is
  // never modified.
  //
  // Throws std::invalid_argument if `limit_tolerance` is not finite.
  LapCounter(const Track& track, double limit_tolerance);

  // Take a new position. The first call seeds the counter and counts no
  // progress, because progress needs two positions.
  void update(double x, double y);

  // Seed the counter at a position without counting the move to it. This is
  // what a teleport or a race restart needs: without it, a car placed back on
  // the grid would count the whole distance from wherever it was.
  void reset_to(double x, double y);

  // Completed laps, on a closed track. Negative if the car has driven
  // backwards past the start. Always zero on an open track, which has no lap
  // to complete; use distance() there.
  int laps() const;

  // Signed distance travelled along the centreline since the counter was
  // seeded, laps included. This is the number lap counting is derived from
  // and it is the more useful one for a controller.                      [m]
  double distance() const { return distance_; }

  // The most recent projection, and what it implied about the limits.
  const Projection& where() const { return where_; }
  const LimitStatus& limits() const { return limits_; }

  // Whether the car has EVER been outside the limits since the counter was
  // seeded. A momentary excursion is the thing a race stewards on, and a flag
  // that clears itself the moment the car comes back is a flag nobody sees.
  bool has_left_the_track() const { return has_left_; }

  // The worst margin seen since seeding: the deepest the car went outside, or
  // the closest it came to the edge if it never left.                    [m]
  double worst_margin() const { return worst_margin_; }

 private:
  const Track* track_;
  double tolerance_;

  bool seeded_ = false;
  double previous_s_ = 0.0;
  double distance_ = 0.0;

  Projection where_;
  LimitStatus limits_;
  bool has_left_ = false;
  double worst_margin_ = 0.0;
};

}  // namespace scene
}  // namespace slipx

#endif  // SLIPX_SCENE_LAP_HPP
