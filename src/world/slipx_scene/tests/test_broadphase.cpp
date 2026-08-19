// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The racing broadphase (ADR-0045). Everything here is held to the same
// standard the grid index was held to: an acceleration structure is worth
// exactly its agreement with the brute-force definition, demanded bit for
// bit over sweeps rather than to a tolerance, because a broadphase that
// occasionally misses the true nearest thing returns a wall or a car that
// is not there.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "slipx/scene/broadphase.hpp"
#include "slipx/scene/track.hpp"

namespace {

using slipx::scene::AgentOverlay;
using slipx::scene::Centreline;
using slipx::scene::OverlayHit;
using slipx::scene::RayHit;
using slipx::scene::SceneBvh;
using slipx::scene::Track;
using slipx::scene::TrackManifest;
using slipx::scene::Walls;

constexpr double kPi = 3.14159265358979323846;

TrackManifest manifest_for(const std::string& name, bool closed) {
  TrackManifest manifest;
  manifest.name = name;
  manifest.surface = "carpet";
  manifest.closed = closed;
  manifest.geometry_source = "a test";
  manifest.geometry_licence = "Apache-2.0";
  manifest.provenance_label = "provisional";
  return manifest;
}

Track square(bool closed = true) {
  Centreline geometry = Centreline::from_csv(
      "0.0,0.0,1.0,1.0\n"
      "10.0,0.0,1.0,1.0\n"
      "10.0,10.0,1.0,1.0\n"
      "0.0,10.0,1.0,1.0\n",
      "square.csv");
  return Track::build(geometry, manifest_for("square", closed),
                      {{"sponge", "carpet"}});
}

// ------------------------------------------------------------------ the BVH

TEST(SceneBvh, AgreesWithBruteForceOnTheSquare) {
  const Walls walls(square());
  const SceneBvh bvh(walls);
  EXPECT_GT(bvh.node_count(), 1u) << "or this asserts brute force against "
                                     "itself";

  std::size_t hits = 0;
  for (int gx = -3; gx <= 13; ++gx) {
    for (int gy = -3; gy <= 13; ++gy) {
      for (int a = 0; a < 24; ++a) {
        const double bearing = -kPi + 2.0 * kPi * a / 24.0;
        const RayHit fast = bvh.cast(gx, gy, bearing, 30.0);
        const RayHit slow = walls.cast_brute_force(gx, gy, bearing, 30.0);

        ASSERT_EQ(fast.hit, slow.hit)
            << "at (" << gx << ", " << gy << ") bearing " << bearing;
        if (fast.hit) {
          ++hits;
          EXPECT_DOUBLE_EQ(fast.range, slow.range)
              << "at (" << gx << ", " << gy << ") bearing " << bearing;
          EXPECT_EQ(fast.left_wall, slow.left_wall);
        }
      }
    }
  }
  EXPECT_GT(hits, 1000u) << "a sweep that hits nothing proves nothing";
}

TEST(SceneBvh, AgreesWithBruteForceOnTheShippedStadium) {
  Centreline geometry = Centreline::from_file(
      std::string(SLIPX_EXAMPLE_TRACK_DIR) + "/centreline.csv");
  const Track track = Track::build(
      geometry, manifest_for("paddock_stadium", true), {{"sponge", "carpet"}});
  const Walls walls(track);
  const SceneBvh bvh(walls);

  std::size_t hits = 0;
  for (int gx = -9; gx <= 9; ++gx) {
    for (int gy = -5; gy <= 5; ++gy) {
      for (int a = 0; a < 36; ++a) {
        const double bearing = -kPi + 2.0 * kPi * a / 36.0;
        const RayHit fast = bvh.cast(gx, gy, bearing, 12.0);
        const RayHit slow = walls.cast_brute_force(gx, gy, bearing, 12.0);

        ASSERT_EQ(fast.hit, slow.hit)
            << "at (" << gx << ", " << gy << ") bearing " << bearing;
        if (fast.hit) {
          ++hits;
          EXPECT_DOUBLE_EQ(fast.range, slow.range)
              << "at (" << gx << ", " << gy << ") bearing " << bearing;
          EXPECT_EQ(fast.left_wall, slow.left_wall);
        }
      }
    }
  }
  EXPECT_GT(hits, 1000u);
}

// The diagonal-through-a-corner family that once caught the grid stepping
// around a cell (see test_raycast.cpp). A BVH has no cells to step around,
// but a ray running exactly along a node boundary exercises the slab test's
// parallel branch, which is the analogous way for this structure to lie.
TEST(SceneBvh, ARayThroughWallCornersAgreesWithBruteForce) {
  const Walls walls(square());
  const SceneBvh bvh(walls);
  constexpr double kQuarter = kPi / 4.0;

  for (int k = 1; k <= 5; ++k) {
    const double starts[][2] = {
        {-1.0 - k, -1.0 + k},
        {11.0 + k, -1.0 + k},
        {-1.0 - k, 11.0 - k},
        {11.0 + k, 11.0 - k},
    };
    const double towards[] = {-kQuarter, -3.0 * kQuarter, kQuarter,
                              3.0 * kQuarter};
    for (int c = 0; c < 4; ++c) {
      const RayHit fast =
          bvh.cast(starts[c][0], starts[c][1], towards[c], 30.0);
      const RayHit slow = walls.cast_brute_force(starts[c][0], starts[c][1],
                                                 towards[c], 30.0);
      ASSERT_EQ(fast.hit, slow.hit) << "corner " << c << " offset " << k;
      if (slow.hit) {
        EXPECT_DOUBLE_EQ(fast.range, slow.range)
            << "corner " << c << " offset " << k;
      }
    }
  }

  // Axis-parallel rays exactly along the walls themselves: the parallel
  // branch of the slab test, at the boundary values.
  for (int a = 0; a < 4; ++a) {
    const double bearing = a * kPi / 2.0;
    for (const double y : {-1.0, 0.0, 1.0, 9.0, 10.0, 11.0}) {
      const RayHit fast = bvh.cast(-3.0, y, bearing, 30.0);
      const RayHit slow = walls.cast_brute_force(-3.0, y, bearing, 30.0);
      ASSERT_EQ(fast.hit, slow.hit) << "y " << y << " bearing " << bearing;
      if (slow.hit) EXPECT_DOUBLE_EQ(fast.range, slow.range);
    }
  }
}

TEST(SceneBvh, AnOpenTrackMissesLikeTheDefinition) {
  Centreline geometry = Centreline::from_csv(
      "0.0,0.0,1.0,1.0\n"
      "10.0,0.0,1.0,1.0\n",
      "straight.csv");
  const Track track = Track::build(geometry, manifest_for("straight", false),
                                   {{"sponge", "carpet"}});
  const Walls walls(track);
  const SceneBvh bvh(walls);

  const RayHit away = bvh.cast(5.0, 0.0, 0.0, 30.0);
  EXPECT_FALSE(away.hit) << "out of the open end: a genuine miss";
  EXPECT_DOUBLE_EQ(away.range, 0.0);

  const RayHit across = bvh.cast(5.0, 0.0, kPi / 2.0, 30.0);
  ASSERT_TRUE(across.hit);
  EXPECT_NEAR(across.range, 1.0, 1e-9);
}

// -------------------------------------------------------------- the overlay

// Seven boxes in poses chosen to exercise every branch: rotated, axis
// aligned, overlapping, distant, inactive.
AgentOverlay constellation() {
  AgentOverlay overlay;
  overlay.resize(7);
  overlay.set(0, 0.0, 0.0, 0.0, 0.275, 0.15);
  overlay.set(1, 1.0, 0.1, 0.4, 0.275, 0.15);
  overlay.set(2, 1.2, -0.05, -0.7, 0.275, 0.15);
  overlay.set(3, 4.0, 4.0, 2.2, 0.275, 0.15);
  overlay.set(4, -2.0, 0.5, kPi / 2.0, 0.275, 0.15);
  overlay.set(5, 0.4, 0.05, 0.1, 0.275, 0.15, false);   // inactive
  overlay.set(6, 1.05, 0.2, 1.9, 0.275, 0.15);
  return overlay;
}

TEST(AgentOverlay, CastAgreesWithBruteForceAcrossASweep) {
  const AgentOverlay overlay = constellation();

  std::size_t hits = 0;
  for (int gx = -4; gx <= 6; ++gx) {
    for (int gy = -3; gy <= 6; ++gy) {
      for (int a = 0; a < 36; ++a) {
        const double bearing = -kPi + 2.0 * kPi * a / 36.0;
        const double x = 0.5 * gx + 0.13;
        const double y = 0.5 * gy - 0.07;
        const OverlayHit fast = overlay.cast(x, y, bearing, 15.0);
        const OverlayHit slow = overlay.cast_brute_force(x, y, bearing, 15.0);

        ASSERT_EQ(fast.hit, slow.hit)
            << "at (" << x << ", " << y << ") bearing " << bearing;
        if (fast.hit) {
          ++hits;
          EXPECT_EQ(fast.range, slow.range);
          EXPECT_EQ(fast.index, slow.index);
        }
      }
    }
  }
  EXPECT_GT(hits, 500u);
}

TEST(AgentOverlay, TheAnalyticalCases) {
  AgentOverlay overlay;
  overlay.resize(2);
  overlay.set(0, 5.0, 0.0, 0.0, 0.5, 0.25);
  overlay.set(1, 9.0, 0.0, kPi / 2.0, 0.5, 0.25);   // rotated a quarter turn

  // Head-on into box 0's near face at x = 4.5.
  const OverlayHit head_on = overlay.cast(0.0, 0.0, 0.0, 20.0);
  ASSERT_TRUE(head_on.hit);
  EXPECT_NEAR(head_on.range, 4.5, 1e-12);
  EXPECT_EQ(head_on.index, 0u);

  // Skip the near box: the rotated one presents its WIDTH along x, so its
  // near face is at 9 - 0.25.
  const OverlayHit skipped = overlay.cast(0.0, 0.0, 0.0, 20.0, 0);
  ASSERT_TRUE(skipped.hit);
  EXPECT_NEAR(skipped.range, 8.75, 1e-12);
  EXPECT_EQ(skipped.index, 1u);

  // From inside a box: touching, at range zero, not invisible.
  const OverlayHit inside = overlay.cast(5.0, 0.0, 1.0, 20.0);
  ASSERT_TRUE(inside.hit);
  EXPECT_DOUBLE_EQ(inside.range, 0.0);

  // A miss is a miss.
  EXPECT_FALSE(overlay.cast(0.0, 5.0, 0.0, 20.0).hit);

  // Deactivating a box removes it from every query.
  overlay.set(0, 5.0, 0.0, 0.0, 0.5, 0.25, false);
  const OverlayHit past = overlay.cast(0.0, 0.0, 0.0, 20.0);
  ASSERT_TRUE(past.hit);
  EXPECT_EQ(past.index, 1u);
}

TEST(AgentOverlay, PairsAgreeWithBruteForce) {
  const AgentOverlay overlay = constellation();

  std::vector<std::pair<std::uint32_t, std::uint32_t>> fast;
  std::vector<std::pair<std::uint32_t, std::uint32_t>> slow;
  overlay.overlapping_pairs(fast);
  overlay.overlapping_pairs_brute_force(slow);
  EXPECT_EQ(fast, slow);
  EXPECT_FALSE(fast.empty()) << "a constellation with no overlaps asserts "
                                "nothing";

  // The inactive box appears in no pair.
  for (const auto& pair : fast) {
    EXPECT_NE(pair.first, 5u);
    EXPECT_NE(pair.second, 5u);
  }

  // And a dense line of touching boxes, where the sweep's break condition
  // earns its keep: bounds that exactly kiss COUNT, because the broadphase
  // is conservative and the narrowphase decides. The sixth box shares x
  // with the row and is far away in y, which is exactly the pair a sweep
  // that forgot its y check would invent.
  AgentOverlay row;
  row.resize(6);
  for (std::size_t i = 0; i < 5; ++i) {
    row.set(i, static_cast<double>(i), 0.0, 0.0, 0.5, 0.2);
  }
  row.set(5, 2.0, 5.0, 0.0, 0.5, 0.2);
  row.overlapping_pairs(fast);
  row.overlapping_pairs_brute_force(slow);
  EXPECT_EQ(fast, slow);
  EXPECT_EQ(fast.size(), 4u) << "each box kisses only its neighbours, and "
                                "the box above the row touches nothing";
}

TEST(AgentOverlay, RefitMovesTheBoxesWithoutRebuilding) {
  AgentOverlay overlay;
  overlay.resize(2);
  overlay.set(0, 0.0, 0.0, 0.0, 0.5, 0.25);
  overlay.set(1, 10.0, 0.0, 0.0, 0.5, 0.25);

  std::vector<std::pair<std::uint32_t, std::uint32_t>> pairs;
  overlay.overlapping_pairs(pairs);
  EXPECT_TRUE(pairs.empty());

  // The refit path: move box 1 next to box 0, as a step of a race would.
  overlay.set(1, 0.6, 0.1, 0.3, 0.5, 0.25);
  overlay.overlapping_pairs(pairs);
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0], (std::pair<std::uint32_t, std::uint32_t>{0, 1}));

  const OverlayHit hit = overlay.cast(-2.0, 0.0, 0.0, 20.0);
  ASSERT_TRUE(hit.hit);
  EXPECT_EQ(hit.index, 0u);
}

}  // namespace
