// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The trajectory hash (NFR-02, SIM-07).
//
// As with the RNG, the pinned values matter: the Python bindings compute the
// same hash and must agree digit for digit, so FNV-1a's published test vectors
// are asserted directly rather than trusted.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "slipx/sim/hash.hpp"

namespace {

using slipx::sim::hash_text;
using slipx::sim::TrajectoryHash;
using slipx::VehicleState;

// Published FNV-1a 64 test vectors.
TEST(TrajectoryHash, MatchesThePublishedFnv1aVectors) {
  TrajectoryHash empty;
  EXPECT_EQ(empty.value(), 0xCBF29CE484222325ULL);

  TrajectoryHash a;
  a.update("a");
  EXPECT_EQ(a.value(), 0xAF63DC4C8601EC8CULL);

  TrajectoryHash foobar;
  foobar.update("foobar");
  EXPECT_EQ(foobar.value(), 0x85944171F73967E8ULL);
}

TEST(TrajectoryHash, HexIsSixteenLowercaseDigits) {
  TrajectoryHash h;
  h.update("foobar");
  EXPECT_EQ(h.hex(), "85944171f73967e8");
  EXPECT_EQ(h.hex().size(), 16u);

  TrajectoryHash empty;
  EXPECT_EQ(empty.hex(), "cbf29ce484222325");
}

// -0.0 and 0.0 are the same state and must hash the same, or a car that
// arrived at rest by braking would differ from one that arrived by
// accelerating (see hash.hpp).
TEST(TrajectoryHash, NegativeZeroHashesAsPositiveZero) {
  TrajectoryHash pos;
  pos.update(0.0);
  TrajectoryHash neg;
  neg.update(-0.0);
  EXPECT_EQ(pos.value(), neg.value());
}

TEST(TrajectoryHash, DistinguishesValuesOneUlpApart) {
  const double v = 1.0;
  const double next = std::nextafter(v, 2.0);
  ASSERT_NE(v, next);

  TrajectoryHash a;
  a.update(v);
  TrajectoryHash b;
  b.update(next);
  EXPECT_NE(a.value(), b.value())
      << "a hash that cannot see one ulp cannot police bit-identity";
}

TEST(TrajectoryHash, OrderMatters) {
  TrajectoryHash ab;
  ab.update(1.0);
  ab.update(2.0);

  TrajectoryHash ba;
  ba.update(2.0);
  ba.update(1.0);

  EXPECT_NE(ab.value(), ba.value())
      << "two trajectories through the same points in a different order are "
         "different trajectories";
}

TEST(TrajectoryHash, HashesEveryFieldOfTheState) {
  const auto with = [](void (*mutate)(VehicleState&)) {
    VehicleState s;
    mutate(s);
    TrajectoryHash h;
    h.update(s);
    return h.value();
  };

  const std::uint64_t base = with([](VehicleState&) {});

  EXPECT_NE(base, with([](VehicleState& s) { s.pos.x = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.pos.y = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.pos.z = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.yaw = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.pitch = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.roll = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.vel_body.x = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.vel_body.y = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.vel_body.z = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.rates.x = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.rates.y = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.rates.z = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.steer = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.steer_rate = 1.0; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.soc = 0.5; }));
  EXPECT_NE(base, with([](VehicleState& s) { s.pack_v = 12.0; }));

  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    VehicleState s;
    s.omega_w[i] = 1.0;
    TrajectoryHash h;
    h.update(s);
    EXPECT_NE(base, h.value()) << "omega_w[" << i << "] is not hashed";

    VehicleState t;
    t.Fz[i] = 1.0;
    TrajectoryHash g;
    g.update(t);
    EXPECT_NE(base, g.value()) << "Fz[" << i << "] is not hashed";
  }
}

// A wheel swap must change the hash. It would not if the per-wheel arrays
// were folded in with a commutative operation, which is an easy thing to
// reach for and would make left-right errors invisible to the whole
// determinism apparatus.
TEST(TrajectoryHash, PerWheelOrderMatters) {
  VehicleState a;
  a.omega_w[slipx::kFrontLeft] = 10.0;
  a.omega_w[slipx::kFrontRight] = 20.0;

  VehicleState b;
  b.omega_w[slipx::kFrontLeft] = 20.0;
  b.omega_w[slipx::kFrontRight] = 10.0;

  TrajectoryHash ha;
  ha.update(a);
  TrajectoryHash hb;
  hb.update(b);
  EXPECT_NE(ha.value(), hb.value());
}

TEST(TrajectoryHash, NaNIsHashedRatherThanCanonicalised) {
  VehicleState s;
  s.pos.x = std::numeric_limits<double>::quiet_NaN();
  TrajectoryHash h;
  h.update(s);
  TrajectoryHash clean;
  clean.update(VehicleState{});
  EXPECT_NE(h.value(), clean.value())
      << "a broken run must not hash as a clean one";
}

TEST(HashText, IsAStandaloneHelperWithTheSameValues) {
  EXPECT_EQ(hash_text("foobar"), "85944171f73967e8");
  EXPECT_EQ(hash_text(""), "cbf29ce484222325");
}

}  // namespace
