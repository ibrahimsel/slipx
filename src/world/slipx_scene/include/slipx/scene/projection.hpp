// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Where a car is, expressed in the track's terms rather than the world's.
//
// Everything that asks a question about progress or position on a track is
// asking about the same two numbers: how far along the centreline the car is,
// and how far to one side. Lap counting needs the first, track limits need
// the second, and a controller wanting a lookahead point needs both, so they
// are computed once here rather than three times in three places.
//
// The projection is a full search over every segment. That is O(n) per call,
// n is a few hundred for a track sampled at 0.1 m, and it is deliberately not
// optimised yet. The obvious improvement is to start from the segment the
// same agent used last time, which turns the search into a local one, and it
// also makes the answer depend on where the car has been rather than only on
// where it is. That is a trade worth making against a measurement, not
// against a guess, so it waits for the benchmarks.

#ifndef SLIPX_SCENE_PROJECTION_HPP
#define SLIPX_SCENE_PROJECTION_HPP

#include <cstddef>

#include "slipx/scene/track.hpp"

namespace slipx {
namespace scene {

// A world position expressed against the centreline.
struct Projection {
  // Arc length of the closest point on the centreline, from the start.  [m]
  double s = 0.0;

  // Signed distance from the centreline, positive to the LEFT of the
  // direction of travel. Left is positive because y is left in ISO 8855 and
  // a second convention for the same idea is how sign errors get in.      [m]
  double lateral = 0.0;

  // The segment the closest point fell on, as an index into the centreline's
  // points: the segment runs from `segment` to `segment + 1`, or from the
  // last point back to the first when the track is closed and `segment` is
  // the last index.
  std::size_t segment = 0;

  // How far along that segment the closest point sits, 0 at its start and 1
  // at its end. Widths are interpolated with it.                          [-]
  double t = 0.0;
};

// The closest point on the centreline to (x, y).
//
// Closest by perpendicular distance to a segment, not by distance to a
// sample: on a track sampled at 0.1 m the difference is up to 5 cm, which is
// a third of a 1/10-scale car's width and would make a track-limits call
// wrong at exactly the moment it matters.
//
// When two segments are equidistant, the earlier one wins. That is arbitrary
// and it is fixed rather than left to whichever comparison happens to run
// first, because a tie broken differently on two machines is a different
// trajectory.
Projection project(const Track& track, double x, double y);

// The drivable width at a projection, to the left and to the right,
// interpolated along the segment the projection landed on.
struct Widths {
  double left = 0.0;   // [m]
  double right = 0.0;  // [m]
};

Widths widths_at(const Track& track, const Projection& where);

// Whether a projected position is inside the track, and by how much.
struct LimitStatus {
  bool inside = true;

  // Distance to the nearer edge, positive inside and negative outside, with
  // the tolerance already applied. A number rather than a flag because "how
  // close was it" is the question anybody adjudicating a run actually has.
  double margin = 0.0;  // [m]
};

// `tolerance` widens the corridor on both sides: the value a competition
// would set to decide how much of a wheel over a line is a violation. It is
// required rather than defaulted, because zero is a rule and not an absence.
// A negative tolerance narrows the corridor, which is a legitimate way to ask
// for a margin, and is allowed.
//
// Throws std::invalid_argument if `tolerance` is not finite.
LimitStatus check_limits(const Track& track, const Projection& where,
                         double tolerance);

}  // namespace scene
}  // namespace slipx

#endif  // SLIPX_SCENE_PROJECTION_HPP
