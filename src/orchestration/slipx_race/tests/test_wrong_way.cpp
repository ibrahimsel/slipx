// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The wrong-way monitor, judged on bare numbers: the race procedures feed
// it lap-counter progress, but the hysteresis is its own contract and is
// pinned here where every branch can be reached in a dozen lines.

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "slipx/race/wrong_way.hpp"

namespace {

using slipx::race::WrongWayMonitor;

TEST(WrongWay, RulesOncePastTheThresholdAndOncePerExcursion) {
  WrongWayMonitor monitor(1.0);
  EXPECT_FALSE(monitor.update(52.0));   // the seed judges nothing
  EXPECT_FALSE(monitor.update(51.4));   // deficit 0.6: below the threshold
  EXPECT_FALSE(monitor.update(51.0));   // deficit 1.0: not yet PAST it
  EXPECT_TRUE(monitor.update(50.9));    // deficit 1.1: ruled, once
  EXPECT_NEAR(monitor.deficit(), 1.1, 1e-12);
  EXPECT_FALSE(monitor.update(47.0));   // still the same excursion
  EXPECT_FALSE(monitor.update(50.0));   // recovering is not a new offence
}

TEST(WrongWay, ReArmsOnlyWhenTheGroundIsMadeBack) {
  WrongWayMonitor monitor(1.0);
  monitor.update(52.0);
  EXPECT_TRUE(monitor.update(50.5));
  EXPECT_FALSE(monitor.update(51.8));   // short of the high water: armed? no
  EXPECT_FALSE(monitor.update(50.5));   // ...so this is still excursion one
  EXPECT_FALSE(monitor.update(52.0));   // the ground is made back
  EXPECT_TRUE(monitor.update(50.9));    // a new excursion, a new ruling
}

TEST(WrongWay, WobblingRulesNothing) {
  WrongWayMonitor monitor(1.0);
  monitor.update(10.0);
  for (int i = 0; i < 100; ++i) {
    EXPECT_FALSE(monitor.update(10.0 + ((i % 2 == 0) ? -0.3 : 0.3)));
  }
}

TEST(WrongWay, RebaseForgivesATeleportAndReArms) {
  WrongWayMonitor monitor(1.0);
  monitor.update(52.0);
  EXPECT_TRUE(monitor.update(50.5));
  // The referee sets the car back three metres: not driving.
  monitor.rebase(47.5);
  EXPECT_FALSE(monitor.update(47.4));   // a fresh baseline, deficit 0.1
  EXPECT_TRUE(monitor.update(46.4));    // and a fresh excursion can rule
}

TEST(WrongWay, RefusesAThresholdThatWouldRuleOnNoise) {
  EXPECT_THROW(WrongWayMonitor(0.0), std::invalid_argument);
  EXPECT_THROW(WrongWayMonitor(-1.0), std::invalid_argument);
  EXPECT_THROW(
      WrongWayMonitor(std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
}

}  // namespace
