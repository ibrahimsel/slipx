// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Watching for a car driving against the race direction.
//
// The judgment is the lap counter's signed progress compared with its own
// high-water mark: a car a threshold's worth of metres behind the furthest
// it has been is driving the wrong way. One ruling stands per excursion,
// re-armed only when the car has made the lost ground back, so a car that
// keeps reversing is ruled once rather than flooding the stream, and a car
// that wobbles on the spot is never ruled at all. Teleports (restarts,
// set-backs) move a car without it driving anywhere, so the procedures
// rebase the monitor after every placement rather than ruling on them.

#ifndef SLIPX_RACE_WRONG_WAY_HPP
#define SLIPX_RACE_WRONG_WAY_HPP

namespace slipx {
namespace race {

class WrongWayMonitor {
 public:
  // `distance` is RaceConfig::wrong_way_distance. Throws
  // std::invalid_argument unless it is a positive, finite number of
  // metres: a zero threshold would rule on the first numerical wobble of
  // a projection, and there is no default to fall back on.
  explicit WrongWayMonitor(double distance);

  // Feed the current signed progress along the race direction. The first
  // call seeds the baseline and never rules. Returns true exactly when a
  // new ruling fires: the deficit has just crossed the threshold.
  bool update(double progress);

  // Accept the current progress as the new baseline without judging the
  // move, and re-arm: what a teleport needs.
  void rebase(double progress);

  // Metres behind the furthest point so far; what a ruling reports.  [m]
  double deficit() const;

 private:
  double threshold_;
  bool seeded_ = false;
  double high_water_ = 0.0;
  double progress_ = 0.0;
  bool ruled_ = false;
};

}  // namespace race
}  // namespace slipx

#endif  // SLIPX_RACE_WRONG_WAY_HPP
