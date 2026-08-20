// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Projection, track limits and lap counting.
//
// The analytical cases run on a square, where every closest point and every
// arc length can be written down. The property cases run on the shipped
// stadium, where the questions are about direction and repetition rather
// than about exact numbers.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

#include "slipx/scene/lap.hpp"
#include "slipx/scene/projection.hpp"
#include "slipx/scene/track.hpp"

namespace {

using slipx::scene::Centreline;
using slipx::scene::LapCounter;
using slipx::scene::Projection;
using slipx::scene::Track;
using slipx::scene::TrackManifest;
using slipx::scene::check_limits;
using slipx::scene::project;
using slipx::scene::widths_at;

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

// A 10 by 10 square, anticlockwise from the origin, 1 m of width each side.
// Corners at (0,0), (10,0), (10,10), (0,10); the lap is 40 m.
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

// The shipped stadium, built from the same file the examples use.
Track stadium() {
  Centreline geometry = Centreline::from_file(
      std::string(SLIPX_EXAMPLE_TRACK_DIR) + "/centreline.csv");
  return Track::build(geometry, manifest_for("paddock_stadium", true),
                      {{"sponge", "carpet"}});
}

// ------------------------------------------------------------- projection

TEST(Projection, FindsThePointOnASegmentNotTheNearestSample) {
  const Track track = square();

  // Halfway along the bottom edge and 0.5 m to the left of it. The nearest
  // stored sample is 5 m away at a corner; the nearest point on the segment
  // is 0.5 m away. On a track sampled at 0.1 m the difference between those
  // two answers is up to 5 cm, which is a third of a car's width.
  const Projection where = project(track, 5.0, 0.5);

  EXPECT_DOUBLE_EQ(where.s, 5.0);
  EXPECT_DOUBLE_EQ(where.lateral, 0.5);
  EXPECT_EQ(where.segment, 0u);
}

TEST(Projection, SignsLateralOffsetPositiveToTheLeft) {
  const Track track = square();

  // Travelling along +x on the bottom edge, so +y is left and -y is right.
  EXPECT_GT(project(track, 5.0, 0.5).lateral, 0.0);
  EXPECT_LT(project(track, 5.0, -0.5).lateral, 0.0);

  // And on the top edge the car is travelling along -x, so the signs of the
  // world axes flip while left stays left.
  EXPECT_GT(project(track, 5.0, 9.5).lateral, 0.0);
  EXPECT_LT(project(track, 5.0, 10.5).lateral, 0.0);
}

TEST(Projection, MeasuresArcLengthAroundTheWholeLap) {
  const Track track = square();

  EXPECT_DOUBLE_EQ(project(track, 0.0, 0.0).s, 0.0);
  EXPECT_DOUBLE_EQ(project(track, 10.0, 0.0).s, 10.0);
  EXPECT_DOUBLE_EQ(project(track, 10.0, 10.0).s, 20.0);
  EXPECT_DOUBLE_EQ(project(track, 0.0, 10.0).s, 30.0);

  // Halfway down the closing segment, which runs from the last point back to
  // the first and whose length is the closing chord rather than a difference
  // of two stored arc lengths.
  EXPECT_DOUBLE_EQ(project(track, 0.0, 5.0).s, 35.0);
  EXPECT_DOUBLE_EQ(track.length(), 40.0);
}

TEST(Projection, AnOpenTrackHasNoClosingSegment) {
  const Track track = square(false);

  // The same query as above. With the closing segment gone, the point off the
  // left-hand edge projects onto the far end of the polyline instead.
  EXPECT_DOUBLE_EQ(track.length(), 30.0);
  EXPECT_LE(project(track, 0.0, 5.0).s, 30.0);
}

TEST(Projection, ProjectsBeyondAnEndOntoTheEndItself) {
  const Track track = square(false);

  // Well past the finish of an open track: the closest point is the last
  // sample, not a point on the line the last segment lies on.
  const Projection where = project(track, -5.0, 10.0);
  EXPECT_DOUBLE_EQ(where.s, 30.0);
  EXPECT_DOUBLE_EQ(std::fabs(where.lateral), 5.0);
}

// --------------------------------------------------------------- the limits

TEST(Limits, InsideAndOutsideWithTheMarginToProve) {
  const Track track = square();

  // 1 m of width each side, no tolerance.
  const auto middle = check_limits(track, project(track, 5.0, 0.0), 0.0);
  EXPECT_TRUE(middle.inside);
  EXPECT_DOUBLE_EQ(middle.margin, 1.0);

  const auto edge = check_limits(track, project(track, 5.0, 1.0), 0.0);
  EXPECT_TRUE(edge.inside) << "exactly on the line is inside";
  EXPECT_DOUBLE_EQ(edge.margin, 0.0);

  const auto out = check_limits(track, project(track, 5.0, 1.25), 0.0);
  EXPECT_FALSE(out.inside);
  EXPECT_DOUBLE_EQ(out.margin, -0.25);
}

TEST(Limits, ToleranceWidensTheCorridorAndANegativeOneNarrowsIt) {
  const Track track = square();
  const Projection where = project(track, 5.0, 1.25);

  EXPECT_FALSE(check_limits(track, where, 0.0).inside);
  EXPECT_TRUE(check_limits(track, where, 0.5).inside);

  // A negative tolerance is a legitimate way to ask for a margin, so it is
  // allowed rather than treated as a mistake.
  EXPECT_FALSE(check_limits(track, project(track, 5.0, 0.9), -0.2).inside);
}

TEST(Limits, AreMeasuredAgainstTheSideTheCarIsOn) {
  // An asymmetric track, so a left width standing in for a right one shows.
  Centreline geometry = Centreline::from_csv(
      "0.0,0.0,0.5,2.0\n"
      "10.0,0.0,0.5,2.0\n"
      "10.0,10.0,0.5,2.0\n"
      "0.0,10.0,0.5,2.0\n",
      "asymmetric.csv");
  const Track track =
      Track::build(geometry, manifest_for("asymmetric", true),
                   {{"sponge", "carpet"}});

  // 1.5 m to the left is inside a 2 m left width; 1.5 m to the right is well
  // outside a 0.5 m right width.
  EXPECT_TRUE(check_limits(track, project(track, 5.0, 1.5), 0.0).inside);
  EXPECT_FALSE(check_limits(track, project(track, 5.0, -1.5), 0.0).inside);
}

TEST(Limits, RefuseANonFiniteTolerance) {
  const Track track = square();
  const Projection where = project(track, 5.0, 0.0);

  EXPECT_THROW(check_limits(track, where, std::nan("")), std::invalid_argument);
  EXPECT_THROW(LapCounter(track, std::nan("")), std::invalid_argument);
}

TEST(Limits, WidthsInterpolateAlongASegment) {
  Centreline geometry = Centreline::from_csv(
      "0.0,0.0,1.0,1.0\n"
      "10.0,0.0,3.0,3.0\n",
      "widening.csv");
  const Track track = Track::build(geometry, manifest_for("widening", false),
                                   {{"sponge", "carpet"}});

  EXPECT_DOUBLE_EQ(widths_at(track, project(track, 5.0, 0.0)).left, 2.0);
  EXPECT_DOUBLE_EQ(widths_at(track, project(track, 2.5, 0.0)).left, 1.5);
}

// --------------------------------------------------------------- lap counting

TEST(LapCounter, CountsAFullLapAndNoMore) {
  const Track track = square();
  LapCounter counter(track, 0.0);

  // Round the square in 1 m steps, anticlockwise, ending back at the start.
  counter.update(0.0, 0.0);
  for (int i = 1; i <= 10; ++i) counter.update(static_cast<double>(i), 0.0);
  for (int i = 1; i <= 10; ++i) counter.update(10.0, static_cast<double>(i));
  for (int i = 1; i <= 10; ++i) counter.update(10.0 - i, 10.0);
  for (int i = 1; i <= 10; ++i) counter.update(0.0, 10.0 - i);

  EXPECT_EQ(counter.laps(), 1);
  EXPECT_NEAR(counter.distance(), 40.0, 1e-9);
}

TEST(LapCounter, CountsSeveralLaps) {
  const Track track = square();
  LapCounter counter(track, 0.0);

  counter.update(0.0, 0.0);
  for (int lap = 0; lap < 3; ++lap) {
    for (int i = 1; i <= 10; ++i) counter.update(static_cast<double>(i), 0.0);
    for (int i = 1; i <= 10; ++i) counter.update(10.0, static_cast<double>(i));
    for (int i = 1; i <= 10; ++i) counter.update(10.0 - i, 10.0);
    for (int i = 1; i <= 10; ++i) counter.update(0.0, 10.0 - i);
  }

  EXPECT_EQ(counter.laps(), 3);
  EXPECT_NEAR(counter.distance(), 120.0, 1e-9);
}

// The reason the counter measures progress rather than watching for a line
// crossing. A car that reverses over the start line has not completed a lap,
// and an implementation that adds one on every crossing says it has.
TEST(LapCounter, DrivingBackwardsOverTheLineSubtracts) {
  const Track track = square();
  LapCounter counter(track, 0.0);

  counter.update(0.0, 0.0);
  counter.update(1.0, 0.0);
  counter.update(0.0, 0.0);

  EXPECT_NEAR(counter.distance(), 0.0, 1e-9);
  EXPECT_EQ(counter.laps(), 0);

  // Carry on backwards, over the line and down the closing segment.
  counter.update(0.0, 9.0);
  EXPECT_LT(counter.distance(), 0.0);
  EXPECT_EQ(counter.laps(), -1) << "behind the line is not the starting lap";
}

// A car nudged back and forth on the line crosses it repeatedly and completes
// nothing, which is the second way a crossing counter miscounts.
TEST(LapCounter, WobblingOnTheLineAccumulatesNothing) {
  const Track track = square();
  LapCounter counter(track, 0.0);

  counter.update(0.5, 0.0);
  for (int i = 0; i < 20; ++i) {
    counter.update(0.0, 9.5);  // just behind the line
    counter.update(0.5, 0.0);  // just past it
  }

  EXPECT_EQ(counter.laps(), 0);
  EXPECT_NEAR(counter.distance(), 0.0, 1e-9);
}

TEST(LapCounter, TheFirstUpdateCountsNoProgress) {
  const Track track = square();
  LapCounter counter(track, 0.0);

  counter.update(5.0, 0.0);

  EXPECT_DOUBLE_EQ(counter.distance(), 0.0);
  EXPECT_DOUBLE_EQ(counter.where().s, 5.0);
}

// Without this, a car placed back on the grid counts the distance from
// wherever it was to wherever it now is as though it drove there.
TEST(LapCounter, ResettingDoesNotCountTheMove) {
  const Track track = square();
  LapCounter counter(track, 0.0);

  counter.update(0.0, 0.0);
  counter.update(5.0, 0.0);
  EXPECT_NEAR(counter.distance(), 5.0, 1e-9);

  counter.reset_to(0.0, 10.0);          // teleported to the far corner
  EXPECT_NEAR(counter.distance(), 5.0, 1e-9);

  counter.update(1.0, 10.0);            // and drove one metre from there
  EXPECT_NEAR(counter.distance(), 4.0, 1e-9) << "that metre runs backwards";
}

TEST(LapCounter, RemembersAnExcursionAfterTheCarComesBack) {
  const Track track = square();
  LapCounter counter(track, 0.0);

  counter.update(1.0, 0.0);
  EXPECT_FALSE(counter.has_left_the_track());

  counter.update(2.0, 1.5);             // 0.5 m outside a 1 m half-width
  EXPECT_TRUE(counter.limits().inside == false);
  EXPECT_TRUE(counter.has_left_the_track());
  EXPECT_NEAR(counter.worst_margin(), -0.5, 1e-9);

  counter.update(3.0, 0.0);             // back on the racing line
  EXPECT_TRUE(counter.limits().inside);
  EXPECT_TRUE(counter.has_left_the_track()) << "a flag that clears itself is "
                                               "a flag nobody sees";
  EXPECT_NEAR(counter.worst_margin(), -0.5, 1e-9);
}

TEST(LapCounter, AnOpenTrackCountsDistanceAndNoLaps) {
  const Track track = square(false);
  LapCounter counter(track, 0.0);

  counter.update(0.0, 0.0);
  counter.update(10.0, 0.0);
  counter.update(10.0, 10.0);

  EXPECT_EQ(counter.laps(), 0);
  EXPECT_NEAR(counter.distance(), 20.0, 1e-9);

  // Driving the whole thing end to end is the case that discriminates. The
  // distance then equals the track's length exactly, and a lap count that is
  // only a division would call that one lap. An open track has no lap: the
  // car reached the far end of a straight, which is not the same event.
  counter.update(0.0, 10.0);
  EXPECT_NEAR(counter.distance(), 30.0, 1e-9);
  EXPECT_DOUBLE_EQ(counter.distance(), track.length());
  EXPECT_EQ(counter.laps(), 0);

  // And the same going back, where the division would say minus one.
  counter.update(10.0, 10.0);
  counter.update(10.0, 0.0);
  counter.update(0.0, 0.0);
  EXPECT_NEAR(counter.distance(), 0.0, 1e-9);
  EXPECT_EQ(counter.laps(), 0);
}

// --------------------------------------------------------- the shipped track

TEST(LapCounter, DrivesALapOfTheShippedStadium) {
  const Track track = stadium();
  LapCounter counter(track, 0.0);

  // Drive the centreline itself, at 5 cm steps, using the same construction
  // the generator used. Half a lap should read as half a lap.
  const double lap = 2.0 * 8.0 + 2.0 * kPi * 3.0;
  const double half = 8.0 / 2.0;
  const auto point_at = [&](double s) {
    const double end = kPi * 3.0;
    if (s < 8.0) return std::make_pair(-half + s, -3.0);
    s -= 8.0;
    if (s < end) {
      const double theta = -kPi / 2.0 + s / 3.0;
      return std::make_pair(half + 3.0 * std::cos(theta), 3.0 * std::sin(theta));
    }
    s -= end;
    if (s < 8.0) return std::make_pair(half - s, 3.0);
    s -= 8.0;
    const double theta = kPi / 2.0 + s / 3.0;
    return std::make_pair(-half + 3.0 * std::cos(theta), 3.0 * std::sin(theta));
  };

  // Just over a lap, not exactly one. Stopping on the line is a boundary
  // case in both directions at once: the polyline the car is projected onto
  // is a hair shorter than the true centreline, because a chord cuts the
  // corner off an arc, so a car that has driven exactly one true lap has
  // covered slightly more than one polyline lap, and a car that stops
  // precisely on the line is a few ulps either side of the count. A real car
  // crosses the line.
  const int steps = 700;
  const double driven = 1.05 * lap;
  for (int i = 0; i <= steps; ++i) {
    const auto p = point_at(std::fmod(driven * i / steps, lap));
    counter.update(p.first, p.second);
  }

  EXPECT_EQ(counter.laps(), 1);
  EXPECT_NEAR(counter.distance(), driven, 0.05);
  EXPECT_FALSE(counter.has_left_the_track())
      << "the centreline is not a track limits violation";
}

TEST(LapCounter, TheReversedTrackCountsTheSamePathTheOtherWay) {
  // One physical drive, judged twice: the stadium's own samples walked
  // backwards read as negative progress against the track as declared and
  // as positive progress against its reversal. This is the whole mechanism
  // by which a race runs the other way round: reverse the track, and every
  // signed quantity follows.
  const Track track = stadium();
  const Track raced = track.reversed();
  LapCounter forward(track, 0.0);
  LapCounter reversed(raced, 0.0);

  const auto& points = track.centreline().points();
  const std::size_t n = points.size();
  // Just over a lap, not exactly one, for the boundary reason the test
  // above spells out.
  const std::size_t steps = n + n / 20;
  for (std::size_t i = 0; i <= steps; ++i) {
    const auto& p = points[(n - (i % n)) % n];
    forward.update(p.x, p.y);
    reversed.update(p.x, p.y);
  }

  EXPECT_GT(reversed.distance(), track.length());
  EXPECT_NEAR(reversed.distance(), -forward.distance(), 1.0e-9)
      << "the two directions must disagree only in sign";
  EXPECT_EQ(reversed.laps(), 1);
  EXPECT_LE(forward.laps(), -1);
  EXPECT_FALSE(reversed.has_left_the_track())
      << "the corridor is the same corridor both ways";
}

// The boundary the test above steps around, pinned deliberately: short of the
// line is the lap the car is on, not the one it has finished.
TEST(LapCounter, JustShortOfTheLineIsStillTheFirstLap) {
  const Track track = square();
  LapCounter counter(track, 0.0);

  counter.update(0.0, 0.0);
  for (int i = 1; i <= 10; ++i) counter.update(static_cast<double>(i), 0.0);
  for (int i = 1; i <= 10; ++i) counter.update(10.0, static_cast<double>(i));
  for (int i = 1; i <= 10; ++i) counter.update(10.0 - i, 10.0);
  for (int i = 1; i <= 9; ++i) counter.update(0.0, 10.0 - i);

  EXPECT_NEAR(counter.distance(), 39.0, 1e-9);
  EXPECT_EQ(counter.laps(), 0);

  counter.update(0.0, 0.0);
  EXPECT_EQ(counter.laps(), 1);
}

}  // namespace
