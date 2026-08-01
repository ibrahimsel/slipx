// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// SIM-03: the seeded random stream.
//
// The pinned values in the first test are the point of this file. SplitMix64
// is a published algorithm with published outputs, and the Python bindings,
// any future C ABI consumer and the cross-architecture conformance job all
// have to agree with these exact numbers. Testing statistical properties
// without pinning the sequence would let a subtly different implementation
// pass while producing different noise on every platform.

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <vector>

#include "slipx/sim/rng.hpp"

namespace {

using slipx::sim::derive_seed;
using slipx::sim::Rng;

// SplitMix64 from seed 0, as published. If this test fails, the engine has
// been changed, and every reference hash in the repository is now wrong.
TEST(Rng, ReproducesThePublishedSplitMix64Sequence) {
  Rng rng(0);
  EXPECT_EQ(rng.next_u64(), 0xE220A8397B1DCDAFULL);
  EXPECT_EQ(rng.next_u64(), 0x6E789E6AA1B965F4ULL);
  EXPECT_EQ(rng.next_u64(), 0x06C45D188009454FULL);
  EXPECT_EQ(rng.next_u64(), 0xF88BB8A8724C81ECULL);
}

TEST(Rng, SameSeedGivesTheSameStream) {
  Rng a(12345);
  Rng b(12345);
  for (int i = 0; i < 1000; ++i) EXPECT_EQ(a.next_u64(), b.next_u64());
}

TEST(Rng, DifferentSeedsGiveDifferentStreams) {
  Rng a(1);
  Rng b(2);
  int agreements = 0;
  for (int i = 0; i < 1000; ++i) {
    if (a.next_u64() == b.next_u64()) ++agreements;
  }
  EXPECT_EQ(agreements, 0);
}

TEST(Rng, UniformStaysInRangeAndCoversIt) {
  Rng rng(7);
  double lo = 2.0;
  double hi = -1.0;
  for (int i = 0; i < 100000; ++i) {
    const double u = rng.uniform();
    ASSERT_GE(u, 0.0);
    ASSERT_LT(u, 1.0) << "the interval is half-open";
    lo = std::fmin(lo, u);
    hi = std::fmax(hi, u);
  }
  EXPECT_LT(lo, 0.001);
  EXPECT_GT(hi, 0.999);
}

TEST(Rng, UniformOverAnIntervalIsAffine) {
  Rng rng(11);
  for (int i = 0; i < 10000; ++i) {
    const double u = rng.uniform(-3.0, 5.0);
    ASSERT_GE(u, -3.0);
    ASSERT_LT(u, 5.0);
  }
}

// Enough to catch a scale error or a missing square root, which is what
// actually goes wrong in a Box-Muller implementation. Not a test of
// normality.
TEST(Rng, NormalHasTheRightFirstTwoMoments) {
  Rng rng(99);
  const int n = 200000;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int i = 0; i < n; ++i) {
    const double x = rng.normal();
    sum += x;
    sum_sq += x * x;
  }
  const double mean = sum / n;
  const double variance = sum_sq / n - mean * mean;
  EXPECT_NEAR(mean, 0.0, 0.01);
  EXPECT_NEAR(variance, 1.0, 0.02);

  Rng scaled(99);
  double s_sum = 0.0;
  for (int i = 0; i < n; ++i) s_sum += scaled.normal(4.0, 0.5);
  EXPECT_NEAR(s_sum / n, 4.0, 0.01);
}

TEST(Rng, NormalIsReproducible) {
  Rng a(2026);
  Rng b(2026);
  for (int i = 0; i < 1000; ++i) EXPECT_EQ(a.normal(), b.normal());
}

// The spare value from the polar method must not leak between two generators
// that happen to have been seeded identically at different points in a run.
TEST(Rng, NormalSpareIsPerInstance) {
  Rng a(5);
  const double first = a.normal();
  const double second = a.normal();

  Rng b(5);
  EXPECT_EQ(b.normal(), first);
  EXPECT_EQ(b.normal(), second);
}

// Adjacent agents must not get correlated streams. Adding the index to the
// master seed would give exactly that, which is why derive_seed mixes.
TEST(DeriveSeed, AdjacentAgentsGetUncorrelatedStreams) {
  std::set<std::uint64_t> firsts;
  for (std::uint64_t agent = 0; agent < 64; ++agent) {
    Rng rng(derive_seed(42, agent));
    firsts.insert(rng.next_u64());
  }
  EXPECT_EQ(firsts.size(), 64u) << "no two agents may start on the same value";

  // And the derived seeds themselves are not a run of consecutive integers.
  const std::uint64_t s0 = derive_seed(42, 0);
  const std::uint64_t s1 = derive_seed(42, 1);
  EXPECT_NE(s1 - s0, 1ULL);
}

TEST(DeriveSeed, IsAFunctionOfBothMasterSeedAndIndex) {
  EXPECT_NE(derive_seed(1, 0), derive_seed(2, 0));
  EXPECT_NE(derive_seed(1, 0), derive_seed(1, 1));
  EXPECT_EQ(derive_seed(1, 0), derive_seed(1, 0));

  // The obvious collision: agent i of one run must not match agent j of
  // another just because the seeds and indices sum the same way.
  EXPECT_NE(derive_seed(10, 5), derive_seed(5, 10));
}

}  // namespace
