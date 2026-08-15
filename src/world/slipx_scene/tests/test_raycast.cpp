// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The walls of a track, and rays against them.
//
// A track's walls are not stored: the centreline carries a width to each
// side, so both walls are implied and this builds them. The tests are on a
// square, where every wall position is a number that can be written down, and
// on the shipped stadium, where the interesting question is whether a ray
// from the racing line finds the wall at the distance the widths promise.

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "slipx/scene/raycast.hpp"
#include "slipx/scene/track.hpp"

namespace {

using slipx::scene::Centreline;
using slipx::scene::RayHit;
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

// A 10 by 10 square, anticlockwise from the origin, 1 m each side.
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

// Travelling anticlockwise, left is towards the inside of the loop, so the
// left wall is the smaller square and the right wall the larger one. That is
// worth pinning: it is the sense in which "left" is used everywhere else, and
// getting it backwards mirrors every scan.
TEST(Walls, LeftIsTheInsideOfAnAnticlockwiseLoop) {
  const Walls walls(square());

  EXPECT_DOUBLE_EQ(walls.left_x()[0], 1.0);
  EXPECT_DOUBLE_EQ(walls.left_y()[0], 1.0);
  EXPECT_DOUBLE_EQ(walls.right_x()[0], -1.0);
  EXPECT_DOUBLE_EQ(walls.right_y()[0], -1.0);
}

// The mitre. Offsetting a corner along the bisector by the width alone would
// put it at width*cos(45 degrees) from each edge, 29 per cent short, and the
// wall would visibly cut the corner. Every corner of the inner square must be
// exactly 1 m in from both edges it joins.
TEST(Walls, ACornerIsOffsetByTheFullWidthFromBothEdges) {
  const Walls walls(square());

  for (std::size_t i = 0; i < 4; ++i) {
    const double x = walls.left_x()[i];
    const double y = walls.left_y()[i];
    // The inner square runs from (1, 1) to (9, 9).
    EXPECT_TRUE((std::fabs(x - 1.0) < 1e-9 || std::fabs(x - 9.0) < 1e-9))
        << "corner " << i << " x = " << x;
    EXPECT_TRUE((std::fabs(y - 1.0) < 1e-9 || std::fabs(y - 9.0) < 1e-9))
        << "corner " << i << " y = " << y;
  }
}

TEST(Walls, ARayFromTheCentrelineFindsTheWallAtTheDeclaredWidth) {
  const Walls walls(square());

  // Halfway along the bottom edge, travelling +x. Left is +y, and the left
  // wall is 1 m away; right is -y, and the right wall is 1 m away too.
  const RayHit left = walls.cast(5.0, 0.0, kPi / 2.0, 30.0);
  ASSERT_TRUE(left.hit);
  EXPECT_NEAR(left.range, 1.0, 1e-9);
  EXPECT_TRUE(left.left_wall);

  const RayHit right = walls.cast(5.0, 0.0, -kPi / 2.0, 30.0);
  ASSERT_TRUE(right.hit);
  EXPECT_NEAR(right.range, 1.0, 1e-9);
  EXPECT_FALSE(right.left_wall);
}

TEST(Walls, ARayFindsTheNearestWallAndNotTheFirstOneItTests) {
  const Walls walls(square());

  // Standing on the bottom edge looking straight up the track: the far side
  // of the loop is 10 m away, and the inner wall is 1 m away. The near one
  // is the answer.
  const RayHit hit = walls.cast(5.0, 0.0, kPi / 2.0, 30.0);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.range, 1.0, 1e-9);
}

TEST(Walls, ARayBehindTheEmitterIsNotAHit) {
  const Walls walls(square());

  // Looking along the track rather than across it. On a square the ray runs
  // down the corridor and meets the far corner's walls, not something behind.
  const RayHit ahead = walls.cast(5.0, 0.0, 0.0, 30.0);
  ASSERT_TRUE(ahead.hit);
  EXPECT_GT(ahead.range, 4.0) << "the wall ahead, not the one behind";
}

TEST(Walls, AMissIsAMissAndNotARangeOfZero) {
  // An open track: two points, so the walls are two finite segments and a ray
  // pointed away from them leaves the world.
  Centreline geometry = Centreline::from_csv(
      "0.0,0.0,1.0,1.0\n"
      "10.0,0.0,1.0,1.0\n",
      "straight.csv");
  const Track track = Track::build(geometry, manifest_for("straight", false),
                                   {{"sponge", "carpet"}});
  const Walls walls(track);

  // Backwards off the end of the walls.
  const RayHit miss = walls.cast(-1.0, 0.0, kPi, 30.0);
  EXPECT_FALSE(miss.hit) << "reporting zero here would be a wall on the mast";
}

TEST(Walls, MaxRangeBoundsTheSearch) {
  const Walls walls(square());

  EXPECT_TRUE(walls.cast(5.0, 0.0, kPi / 2.0, 2.0).hit);
  EXPECT_FALSE(walls.cast(5.0, 0.0, kPi / 2.0, 0.5).hit);
}

// An open track's walls end. The case has to be built on three points, not
// two: with two points the segment from the last back to the first is the
// same segment reversed, so a closing wall that should not exist is
// geometrically invisible and the test would pass either way.
TEST(Walls, AnOpenTrackHasNoClosingWall) {
  Centreline geometry = Centreline::from_csv(
      "0.0,0.0,1.0,1.0\n"
      "10.0,0.0,1.0,1.0\n"
      "10.0,10.0,1.0,1.0\n",
      "corner.csv");

  const Track open = Track::build(geometry, manifest_for("corner", false),
                                  {{"sponge", "carpet"}});
  const Track closed = Track::build(geometry, manifest_for("corner", true),
                                    {{"sponge", "carpet"}});

  EXPECT_FALSE(Walls(open).closed());
  EXPECT_TRUE(Walls(closed).closed());

  // A ray aimed across the diagonal that a closing wall would occupy, from a
  // point outside the corridor. Closed, the diagonal is there to be hit;
  // open, the ray leaves the world.
  const double north_west = 3.0 * kPi / 4.0;

  const RayHit on_open = Walls(open).cast(7.0, 3.0, north_west, 30.0);
  EXPECT_FALSE(on_open.hit) << "an open track has no wall across its ends";

  const RayHit on_closed = Walls(closed).cast(7.0, 3.0, north_west, 30.0);
  EXPECT_TRUE(on_closed.hit) << "and a closed one does";
}

// ---------------------------------------------------------- the shipped track

TEST(Walls, TheShippedStadiumIsTheWidthItClaims) {
  Centreline geometry = Centreline::from_file(
      std::string(SLIPX_EXAMPLE_TRACK_DIR) + "/centreline.csv");
  const Track track = Track::build(
      geometry, manifest_for("paddock_stadium", true), {{"sponge", "carpet"}});
  const Walls walls(track);

  // On the bottom straight at the start, travelling +x. The manifest says
  // 0.75 m each side, and a ray across the track should find exactly that.
  const RayHit left = walls.cast(-4.0 + 2.0, -3.0, kPi / 2.0, 30.0);
  ASSERT_TRUE(left.hit);
  EXPECT_NEAR(left.range, 0.75, 1e-6);

  const RayHit right = walls.cast(-4.0 + 2.0, -3.0, -kPi / 2.0, 30.0);
  ASSERT_TRUE(right.hit);
  EXPECT_NEAR(right.range, 0.75, 1e-6);

  // And in the middle of an end, where the walls are curved. Travelling
  // anticlockwise at the far right of the stadium, the car heads +y, so left
  // is -x (towards the inside) and right is +x.
  const RayHit inner = walls.cast(4.0 + 3.0, 0.0, kPi, 30.0);
  ASSERT_TRUE(inner.hit);
  EXPECT_NEAR(inner.range, 0.75, 1e-3)
      << "a polyline wall on an arc is not exact, but it is close";
}

// ------------------------------------------------------- the spatial index
//
// The index exists because a measurement asked for it, and it is worth
// exactly as much as its agreement with the definition. So the definition is
// kept, as cast_brute_force, and these cases sweep thousands of rays across
// both tracks and demand the two answers are identical rather than close: a
// grid that occasionally misses the true nearest segment and returns the
// second one would pass a tolerance-based test and be wrong in the way that
// matters, which is a wall that is not there.

TEST(Walls, TheIndexAgreesWithBruteForceOnTheSquare) {
  const Walls walls(square());

  std::size_t hits = 0;
  for (int gx = -3; gx <= 13; ++gx) {
    for (int gy = -3; gy <= 13; ++gy) {
      for (int a = 0; a < 24; ++a) {
        const double bearing = -kPi + 2.0 * kPi * a / 24.0;
        const double x = gx;
        const double y = gy;

        const RayHit fast = walls.cast(x, y, bearing, 30.0);
        const RayHit slow = walls.cast_brute_force(x, y, bearing, 30.0);

        ASSERT_EQ(fast.hit, slow.hit)
            << "at (" << x << ", " << y << ") bearing " << bearing;
        if (fast.hit) {
          ++hits;
          EXPECT_DOUBLE_EQ(fast.range, slow.range)
              << "at (" << x << ", " << y << ") bearing " << bearing;
          EXPECT_EQ(fast.left_wall, slow.left_wall);
        }
      }
    }
  }
  EXPECT_GT(hits, 1000u) << "a sweep that hits nothing proves nothing";
}

TEST(Walls, TheIndexAgreesWithBruteForceOnTheShippedStadium) {
  Centreline geometry = Centreline::from_file(
      std::string(SLIPX_EXAMPLE_TRACK_DIR) + "/centreline.csv");
  const Track track = Track::build(
      geometry, manifest_for("paddock_stadium", true), {{"sponge", "carpet"}});
  const Walls walls(track);

  // The index has to be doing something, or this test is asserting that
  // brute force agrees with itself.
  EXPECT_GT(walls.cell_count(), 1u);

  std::size_t hits = 0;
  for (int gx = -9; gx <= 9; ++gx) {
    for (int gy = -5; gy <= 5; ++gy) {
      for (int a = 0; a < 36; ++a) {
        const double bearing = -kPi + 2.0 * kPi * a / 36.0;
        const double x = gx;
        const double y = gy;

        const RayHit fast = walls.cast(x, y, bearing, 12.0);
        const RayHit slow = walls.cast_brute_force(x, y, bearing, 12.0);

        ASSERT_EQ(fast.hit, slow.hit)
            << "at (" << x << ", " << y << ") bearing " << bearing;
        if (fast.hit) {
          ++hits;
          EXPECT_DOUBLE_EQ(fast.range, slow.range)
              << "at (" << x << ", " << y << ") bearing " << bearing;
        }
      }
    }
  }
  EXPECT_GT(hits, 2000u);
}

// A ray that starts a long way outside the track's bounding box, crosses it
// and leaves. The traversal must not give up when it starts outside the grid,
// which is the easy way to write it and is wrong for exactly this ray.
TEST(Walls, TheIndexFindsAWallFromOutsideTheGrid) {
  const Walls walls(square());

  const RayHit fast = walls.cast(-25.0, 5.0, 0.0, 60.0);
  const RayHit slow = walls.cast_brute_force(-25.0, 5.0, 0.0, 60.0);

  EXPECT_TRUE(slow.hit);
  EXPECT_EQ(fast.hit, slow.hit);
  EXPECT_DOUBLE_EQ(fast.range, slow.range);
}

}  // namespace
