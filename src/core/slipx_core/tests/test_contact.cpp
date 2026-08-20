// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The contact impulse (ADR-0043), against what it actually promises: not
// fidelity to any measured collision, but momentum conservation, the Coulomb
// cone, mirror symmetry bit for bit, and closed-form agreement in the cases
// simple enough to work by hand.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "slipx/contact.hpp"

namespace {

using slipx::ContactBody;
using slipx::ContactGeometry;
using slipx::ContactImpulse;
using slipx::ContactParams;
using slipx::Vec2;
using slipx::rectangle_contact;
using slipx::resolve_contact;
using slipx::segment_contact;

// A movable 1 kg, 1 kg m^2 box, half a metre by 0.3, centred on its CoG.
ContactBody box(double x, double y, double yaw = 0.0) {
  ContactBody b;
  b.cog = Vec2{x, y};
  b.yaw = yaw;
  b.inv_mass = 1.0;
  b.inv_izz = 1.0;
  b.half_length = 0.25;
  b.half_width = 0.15;
  return b;
}

ContactBody mirrored(const ContactBody& b) {
  ContactBody m = b;
  m.cog.y = -m.cog.y;
  m.yaw = -m.yaw;
  m.velocity.y = -m.velocity.y;
  m.yaw_rate = -m.yaw_rate;
  return m;
}

// ------------------------------------------------------------------ geometry

TEST(ContactGeometry, SeparatedBoxesDoNotTouch) {
  const ContactBody a = box(0.0, 0.0);
  EXPECT_FALSE(rectangle_contact(a, box(1.0, 0.0)).touching);
  EXPECT_FALSE(rectangle_contact(a, box(0.0, 0.5)).touching);
  // Diagonal: separated on both face axes' perpendiculars only jointly.
  EXPECT_FALSE(rectangle_contact(a, box(0.55, 0.35)).touching);
}

TEST(ContactGeometry, ExactlyTouchingIsNotTouching) {
  // Kissing faces have zero overlap, and zero depth would produce a zero
  // impulse and a zero correction anyway; excluding it keeps depth strictly
  // positive whenever touching is true.
  const ContactBody a = box(0.0, 0.0);
  EXPECT_FALSE(rectangle_contact(a, box(0.5, 0.0)).touching);
}

TEST(ContactGeometry, HeadOnOverlapFindsTheCentralContact) {
  const ContactBody a = box(-0.2, 0.0);
  const ContactBody b = box(0.2, 0.0);
  const ContactGeometry g = rectangle_contact(a, b);

  ASSERT_TRUE(g.touching);
  EXPECT_DOUBLE_EQ(g.normal.x, 1.0);
  EXPECT_DOUBLE_EQ(g.normal.y, 0.0);
  EXPECT_NEAR(g.depth, 0.1, 1e-12);
  // Faces span the same y interval, so the clipped midpoint sits on the
  // centreline: the symmetric case produces no yaw moment.
  EXPECT_NEAR(g.point.y, 0.0, 1e-12);

  // Argument order decides which way the normal points: from the first
  // body toward the second, whichever side of each other they sit. This is
  // the branch that flips the axis, and a wrong sign here is an impulse
  // that pulls the cars together.
  const ContactGeometry swapped = rectangle_contact(b, a);
  ASSERT_TRUE(swapped.touching);
  EXPECT_DOUBLE_EQ(swapped.normal.x, -1.0);
  EXPECT_DOUBLE_EQ(swapped.depth, g.depth);
}

TEST(ContactGeometry, TheFootprintSitsOnItsOffsetCentre) {
  // A car's body is centred between its axles, not on its CoG. A body with
  // its rectangle pushed 0.2 m ahead of the CoG touches where the
  // rectangle is, and does not touch where the CoG is.
  ContactBody a = box(0.0, 0.0);
  a.centre_offset = 0.2;

  // A probe just ahead: the offset rectangle spans x in [-0.05, 0.45], so
  // a probe centred at 0.6 (spanning [0.35, 0.85]) overlaps it, and would
  // not overlap an unshifted rectangle spanning [-0.25, 0.25].
  EXPECT_TRUE(rectangle_contact(a, box(0.6, 0.0)).touching);
  // A probe just behind the CoG: the unshifted rectangle would overlap it,
  // the offset one does not.
  EXPECT_FALSE(rectangle_contact(a, box(-0.4, 0.0)).touching);
}

TEST(ContactGeometry, SideOverlapPointsSideways) {
  const ContactBody a = box(0.0, 0.0);
  const ContactBody b = box(0.0, 0.25);
  const ContactGeometry g = rectangle_contact(a, b);

  ASSERT_TRUE(g.touching);
  EXPECT_DOUBLE_EQ(g.normal.x, 0.0);
  EXPECT_DOUBLE_EQ(g.normal.y, 1.0);
  EXPECT_NEAR(g.depth, 0.05, 1e-12);
}

TEST(ContactGeometry, ARotatedBoxStillResolves) {
  const ContactBody a = box(0.0, 0.0);
  const ContactBody b = box(0.45, 0.0, 0.7);
  const ContactGeometry g = rectangle_contact(a, b);

  ASSERT_TRUE(g.touching);
  EXPECT_GT(g.depth, 0.0);
  const double norm = std::hypot(g.normal.x, g.normal.y);
  EXPECT_NEAR(norm, 1.0, 1e-12);
  EXPECT_TRUE(std::isfinite(g.point.x));
  EXPECT_TRUE(std::isfinite(g.point.y));
}

TEST(ContactGeometry, CornerOnCornerGrazeStaysFinite) {
  // Two boxes overlapping only at a corner: the incident edge clips away
  // entirely and the fallback keeps the nearer endpoint rather than
  // averaging nothing.
  const ContactBody a = box(0.0, 0.0);
  const ContactBody b = box(0.49, 0.29);
  const ContactGeometry g = rectangle_contact(a, b);

  ASSERT_TRUE(g.touching);
  EXPECT_TRUE(std::isfinite(g.point.x));
  EXPECT_TRUE(std::isfinite(g.point.y));
  EXPECT_GT(g.depth, 0.0);
}

// ------------------------------------------------------------------- impulse

TEST(ContactImpulseLaw, HeadOnElasticCollisionSwapsTheVelocities) {
  ContactBody a = box(-0.2, 0.0);
  ContactBody b = box(0.2, 0.0);
  a.velocity = Vec2{2.0, 0.0};
  b.velocity = Vec2{-2.0, 0.0};

  ContactParams p;
  p.restitution = 1.0;
  const ContactGeometry g = rectangle_contact(a, b);
  const ContactImpulse r = resolve_contact(a, b, g, p);

  EXPECT_NEAR(a.velocity.x + r.delta_velocity_a.x, -2.0, 1e-12);
  EXPECT_NEAR(b.velocity.x + r.delta_velocity_b.x, 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(r.delta_yaw_rate_a, 0.0);
  EXPECT_DOUBLE_EQ(r.delta_yaw_rate_b, 0.0);
  EXPECT_DOUBLE_EQ(r.jt, 0.0);
}

TEST(ContactImpulseLaw, ZeroRestitutionKillsTheClosingSpeed) {
  ContactBody a = box(-0.2, 0.0);
  ContactBody b = box(0.2, 0.0);
  a.velocity = Vec2{2.0, 0.0};
  b.velocity = Vec2{-2.0, 0.0};

  ContactParams p;
  p.restitution = 0.0;
  const ContactImpulse r = resolve_contact(a, b, rectangle_contact(a, b), p);

  EXPECT_NEAR(a.velocity.x + r.delta_velocity_a.x, 0.0, 1e-12);
  EXPECT_NEAR(b.velocity.x + r.delta_velocity_b.x, 0.0, 1e-12);
}

TEST(ContactImpulseLaw, ASlowPushDoesNotBounce) {
  // Below restitution_min_speed the restitution is treated as zero: two
  // cars rubbing side by side push apart, they do not chatter. This is an
  // anti-jitter device and the test pins it as behaviour, not physics.
  ContactBody a = box(-0.2, 0.0);
  ContactBody b = box(0.2, 0.0);
  a.velocity = Vec2{0.03, 0.0};   // closing at 0.03 m/s, threshold 0.1

  ContactParams p;
  p.restitution = 1.0;
  const ContactImpulse r = resolve_contact(a, b, rectangle_contact(a, b), p);

  const double vn_after = (b.velocity.x + r.delta_velocity_b.x) -
                          (a.velocity.x + r.delta_velocity_a.x);
  EXPECT_NEAR(vn_after, 0.0, 1e-12) << "killed, not reversed";
}

TEST(ContactImpulseLaw, SeparatingBodiesGetNoImpulseButStillSeparate) {
  ContactBody a = box(-0.2, 0.0);
  ContactBody b = box(0.2, 0.0);
  a.velocity = Vec2{-1.0, 0.0};   // penetrating but already separating
  b.velocity = Vec2{1.0, 0.0};

  const ContactImpulse r =
      resolve_contact(a, b, rectangle_contact(a, b), ContactParams{});

  EXPECT_DOUBLE_EQ(r.jn, 0.0);
  EXPECT_DOUBLE_EQ(r.delta_velocity_a.x, 0.0);
  // The positional projection still runs: penetration is a geometry fact,
  // not a velocity fact.
  EXPECT_LT(r.delta_position_a.x, 0.0);
  EXPECT_GT(r.delta_position_b.x, 0.0);
}

TEST(ContactImpulseLaw, TheProjectionSeparatesTheBoxesAndNoFurther) {
  ContactBody a = box(-0.2, 0.0);
  ContactBody b = box(0.2, 0.0);
  a.velocity = Vec2{1.0, 0.0};

  const ContactGeometry g = rectangle_contact(a, b);
  const ContactImpulse r = resolve_contact(a, b, g, ContactParams{});

  ContactBody a2 = a;
  ContactBody b2 = b;
  a2.cog += r.delta_position_a;
  b2.cog += r.delta_position_b;
  EXPECT_FALSE(rectangle_contact(a2, b2).touching)
      << "the full projection leaves no overlap behind";

  // And it is minimal: nudge them back together by a hair and they touch
  // again. A projection that overshoots is a teleport, not a separation.
  b2.cog.x -= 1e-6;
  EXPECT_TRUE(rectangle_contact(a2, b2).touching);
}


TEST(ContactImpulseLaw, AnImmovableBodyDoesNotMove) {
  ContactBody a = box(-0.2, 0.0);
  a.velocity = Vec2{2.0, 0.0};
  ContactBody wall = box(0.2, 0.0);
  wall.inv_mass = 0.0;
  wall.inv_izz = 0.0;

  ContactParams p;
  p.restitution = 1.0;
  const ContactImpulse r =
      resolve_contact(a, wall, rectangle_contact(a, wall), p);

  EXPECT_DOUBLE_EQ(r.delta_velocity_b.x, 0.0);
  EXPECT_DOUBLE_EQ(r.delta_velocity_b.y, 0.0);
  EXPECT_DOUBLE_EQ(r.delta_yaw_rate_b, 0.0);
  EXPECT_DOUBLE_EQ(r.delta_position_b.x, 0.0);
  // The moving body takes the whole correction and reflects elastically.
  EXPECT_NEAR(a.velocity.x + r.delta_velocity_a.x, -2.0, 1e-12);
  EXPECT_LT(r.delta_position_a.x, 0.0);

  // Two immovable bodies produce exact zeros everywhere, not a division by
  // the zero their inverse masses sum to. Unreachable through the
  // orchestrator, which skips frozen pairs, but this is a public function
  // and somebody's embedding will find the case.
  ContactBody wall2 = wall;
  wall2.cog = Vec2{0.1, 0.0};
  const ContactImpulse rw =
      resolve_contact(wall, wall2, rectangle_contact(wall, wall2), p);
  EXPECT_EQ(rw.jn, 0.0);
  EXPECT_EQ(rw.delta_position_a.x, 0.0);
  EXPECT_EQ(rw.delta_position_b.x, 0.0);
  EXPECT_TRUE(std::isfinite(rw.delta_velocity_a.x));
}

TEST(ContactImpulseLaw, AGlancingHitYawsTheStruckCarTheRightWay) {
  // Struck on a point LEFT of its CoG by an impulse pushing forward (+x):
  // the torque r x j is negative, so the car yaws clockwise (rightward,
  // negative in ISO 8855). Geometry built by hand because the sign is the
  // whole assertion.
  ContactBody a = box(-0.3, 0.1);
  a.velocity = Vec2{2.0, 0.0};
  ContactBody b = box(0.2, 0.0);

  ContactGeometry g;
  g.touching = true;
  g.normal = Vec2{1.0, 0.0};
  g.depth = 0.01;
  g.point = Vec2{-0.05, 0.1};   // 0.1 m left of b's CoG

  ContactParams p;
  p.friction = 0.0;
  const ContactImpulse r = resolve_contact(a, b, g, p);
  EXPECT_GT(r.jn, 0.0);
  EXPECT_LT(r.delta_yaw_rate_b, 0.0);
}

// ---------------------------------------------------------------- invariants

// The configurations the conservation and symmetry laws are held against:
// varied masses, angles, offsets, spins and friction, chosen to make every
// term of the impulse nonzero rather than to look like any particular crash.
struct Scenario {
  ContactBody a;
  ContactBody b;
  ContactParams p;
};

std::vector<Scenario> scenarios() {
  std::vector<Scenario> out;
  {
    Scenario s;
    s.a = box(-0.2, 0.02);
    s.a.velocity = Vec2{3.0, 0.2};
    s.a.yaw_rate = 0.5;
    s.b = box(0.2, -0.03, 0.15);
    s.b.velocity = Vec2{-1.0, 0.1};
    s.p.friction = 0.5;
    out.push_back(s);
  }
  {
    Scenario s;
    s.a = box(-0.15, 0.1, 0.3);
    s.a.velocity = Vec2{1.5, -0.5};
    s.a.inv_mass = 1.0 / 3.5;
    s.a.inv_izz = 1.0 / 0.05;
    s.b = box(0.2, 0.0, -0.4);
    s.b.velocity = Vec2{-0.5, 0.3};
    s.b.yaw_rate = -1.0;
    s.b.inv_mass = 1.0 / 2.0;
    s.b.inv_izz = 1.0 / 0.03;
    s.p.restitution = 0.8;
    s.p.friction = 0.9;
    out.push_back(s);
  }
  {
    // A side swipe: contact on the lateral axis, heavy friction.
    Scenario s;
    s.a = box(0.0, 0.0);
    s.a.velocity = Vec2{4.0, 0.1};
    s.b = box(0.1, 0.27, 0.05);
    s.b.velocity = Vec2{2.0, -0.2};
    s.b.yaw_rate = 0.8;
    s.p.friction = 1.2;
    out.push_back(s);
  }
  return out;
}

TEST(ContactInvariants, MomentumIsConservedLinearAndAngular) {
  for (const Scenario& s : scenarios()) {
    const ContactGeometry g = rectangle_contact(s.a, s.b);
    ASSERT_TRUE(g.touching);
    const ContactImpulse r = resolve_contact(s.a, s.b, g, s.p);
    ASSERT_GT(r.jn, 0.0) << "a conservation case that never collides "
                            "asserts nothing";

    const double ma = 1.0 / s.a.inv_mass;
    const double mb = 1.0 / s.b.inv_mass;
    const double ia = 1.0 / s.a.inv_izz;
    const double ib = 1.0 / s.b.inv_izz;

    // Linear: the two bodies' momentum changes cancel.
    EXPECT_NEAR(ma * r.delta_velocity_a.x + mb * r.delta_velocity_b.x, 0.0,
                1e-11);
    EXPECT_NEAR(ma * r.delta_velocity_a.y + mb * r.delta_velocity_b.y, 0.0,
                1e-11);

    // Angular, about the world origin: L = m (cog x v) + I w per body.
    const double dL =
        ma * s.a.cog.cross(r.delta_velocity_a) + ia * r.delta_yaw_rate_a +
        mb * s.b.cog.cross(r.delta_velocity_b) + ib * r.delta_yaw_rate_b;
    EXPECT_NEAR(dL, 0.0, 1e-11);

    // The friction cone: the tangential impulse never exceeds its budget.
    EXPECT_LE(std::abs(r.jt), s.p.friction * r.jn + 1e-15);
  }
}

// Newton's kinematic definition of restitution, held for an eccentric
// frictionless impact: the normal relative velocity AT THE CONTACT POINT,
// yaw contributions included, reverses scaled by e. This is the assertion
// that pins the effective-mass denominator (the r x n terms): break those
// and momentum still balances, but the bounce comes out wrong.
TEST(ContactInvariants, RestitutionMeansTheNormalVelocityReverses) {
  for (Scenario s : scenarios()) {
    s.p.friction = 0.0;   // the sequential friction pass couples weakly
                          // into the normal direction; the law is exact
                          // only without it
    s.p.restitution = 0.6;
    s.p.restitution_min_speed = 0.0;
    const ContactGeometry g = rectangle_contact(s.a, s.b);
    ASSERT_TRUE(g.touching);
    const ContactImpulse r = resolve_contact(s.a, s.b, g, s.p);
    ASSERT_GT(r.jn, 0.0);

    const auto point_velocity = [&g](const ContactBody& body, const Vec2& dv,
                                     double dw) {
      const Vec2 arm = g.point - body.cog;
      const double w = body.yaw_rate + dw;
      return Vec2{body.velocity.x + dv.x - w * arm.y,
                  body.velocity.y + dv.y + w * arm.x};
    };

    const Vec2 va0 = point_velocity(s.a, Vec2{}, 0.0);
    const Vec2 vb0 = point_velocity(s.b, Vec2{}, 0.0);
    const double vn_before = (vb0 - va0).dot(g.normal);

    const Vec2 va1 =
        point_velocity(s.a, r.delta_velocity_a, r.delta_yaw_rate_a);
    const Vec2 vb1 =
        point_velocity(s.b, r.delta_velocity_b, r.delta_yaw_rate_b);
    const double vn_after = (vb1 - va1).dot(g.normal);

    EXPECT_NEAR(vn_after, -s.p.restitution * vn_before, 1e-11);
  }
}

TEST(ContactInvariants, MirrorSymmetryHoldsBitForBit) {
  for (const Scenario& s : scenarios()) {
    const ContactGeometry g = rectangle_contact(s.a, s.b);
    const ContactImpulse r = resolve_contact(s.a, s.b, g, s.p);

    const ContactBody am = mirrored(s.a);
    const ContactBody bm = mirrored(s.b);
    const ContactGeometry gm = rectangle_contact(am, bm);
    const ContactImpulse rm = resolve_contact(am, bm, gm, s.p);

    ASSERT_EQ(g.touching, gm.touching);
    // EXPECT_EQ on doubles, deliberately: the promise is bit-identity, not
    // closeness, and it is what makes a mirrored race the same race.
    EXPECT_EQ(g.depth, gm.depth);
    EXPECT_EQ(g.normal.x, gm.normal.x);
    EXPECT_EQ(g.normal.y, -gm.normal.y);
    EXPECT_EQ(g.point.x, gm.point.x);
    EXPECT_EQ(g.point.y, -gm.point.y);

    EXPECT_EQ(r.jn, rm.jn);
    EXPECT_EQ(r.jt, -rm.jt);
    EXPECT_EQ(r.delta_velocity_a.x, rm.delta_velocity_a.x);
    EXPECT_EQ(r.delta_velocity_a.y, -rm.delta_velocity_a.y);
    EXPECT_EQ(r.delta_velocity_b.x, rm.delta_velocity_b.x);
    EXPECT_EQ(r.delta_velocity_b.y, -rm.delta_velocity_b.y);
    EXPECT_EQ(r.delta_yaw_rate_a, -rm.delta_yaw_rate_a);
    EXPECT_EQ(r.delta_yaw_rate_b, -rm.delta_yaw_rate_b);
    EXPECT_EQ(r.delta_position_a.x, rm.delta_position_a.x);
    EXPECT_EQ(r.delta_position_a.y, -rm.delta_position_a.y);
  }
}

TEST(ContactInvariants, TheSameInputsGiveTheSameOutputs) {
  for (const Scenario& s : scenarios()) {
    const ContactGeometry g1 = rectangle_contact(s.a, s.b);
    const ContactGeometry g2 = rectangle_contact(s.a, s.b);
    const ContactImpulse r1 = resolve_contact(s.a, s.b, g1, s.p);
    const ContactImpulse r2 = resolve_contact(s.a, s.b, g2, s.p);
    EXPECT_EQ(r1.jn, r2.jn);
    EXPECT_EQ(r1.jt, r2.jt);
    EXPECT_EQ(r1.delta_velocity_a.x, r2.delta_velocity_a.x);
    EXPECT_EQ(r1.delta_yaw_rate_b, r2.delta_yaw_rate_b);
  }
}

// ------------------------------------------------------------------ segments
//
// The wall contact (ADR-0055): a zero-thickness segment against a
// footprint, resolved to the side the body's centre is on.

ContactBody wall_of(const Vec2& p, const Vec2& q) {
  ContactBody wall;   // both reciprocals zero: immovable, at rest
  wall.cog = (p + q) * 0.5;
  return wall;
}

TEST(SegmentContact, MissGrazeAndDegenerateAreNotTouching) {
  const ContactBody b = box(0.0, 0.0);
  // Beyond the front face.
  EXPECT_FALSE(
      segment_contact(Vec2{0.3, -1.0}, Vec2{0.3, 1.0}, b).touching);
  // Exactly on the front face: kissing, excluded as for rectangles.
  EXPECT_FALSE(
      segment_contact(Vec2{0.25, -1.0}, Vec2{0.25, 1.0}, b).touching);
  // Inside the front slab but wholly beyond the side planes.
  EXPECT_FALSE(
      segment_contact(Vec2{0.2, 0.2}, Vec2{0.2, 1.0}, b).touching);
  // A zero-length segment touches nothing, whatever it overlaps.
  EXPECT_FALSE(
      segment_contact(Vec2{0.1, 0.0}, Vec2{0.1, 0.0}, b).touching);
}

TEST(SegmentContact, HeadOnDepthNormalAndPoint) {
  // A long vertical wall crossing the front face: the centre sits 0.2 to
  // the wall's left, so the normal points at the centre and the depth is
  // the front half-length past the line.
  const ContactBody b = box(0.0, 0.0);
  const ContactGeometry g =
      segment_contact(Vec2{0.2, -1.0}, Vec2{0.2, 1.0}, b);
  ASSERT_TRUE(g.touching);
  EXPECT_DOUBLE_EQ(g.normal.x, -1.0);
  EXPECT_DOUBLE_EQ(g.normal.y, 0.0);
  EXPECT_NEAR(g.depth, 0.05, 1e-15);
  // The clipped span is the box's width slab, so its midpoint is central:
  // the symmetric case produces no yaw moment, as for rectangles.
  EXPECT_NEAR(g.point.x, 0.2, 1e-15);
  EXPECT_NEAR(g.point.y, 0.0, 1e-15);
}

TEST(SegmentContact, TheNormalFollowsTheCentreNotTheWinding) {
  // The same wall with its endpoints swapped reverses the raw winding
  // normal; the orientation branch must land both on the centre's side.
  const ContactBody b = box(0.0, 0.0);
  const ContactGeometry g =
      segment_contact(Vec2{0.2, -1.0}, Vec2{0.2, 1.0}, b);
  const ContactGeometry swapped =
      segment_contact(Vec2{0.2, 1.0}, Vec2{0.2, -1.0}, b);
  ASSERT_TRUE(g.touching);
  ASSERT_TRUE(swapped.touching);
  EXPECT_DOUBLE_EQ(swapped.normal.x, g.normal.x);
  EXPECT_DOUBLE_EQ(swapped.normal.y, g.normal.y);
  EXPECT_DOUBLE_EQ(swapped.depth, g.depth);
}

TEST(SegmentContact, AnEndInsideTheBoxClipsToTheBoundary) {
  // A wall whose end stops inside the box, running out through the front
  // face. The normal is still the line normal toward the centre, and the
  // contact point is the midpoint of the inside portion.
  const ContactBody b = box(0.0, -0.05);
  const ContactGeometry g =
      segment_contact(Vec2{0.2, 0.0}, Vec2{5.0, 0.0}, b);
  ASSERT_TRUE(g.touching);
  EXPECT_DOUBLE_EQ(g.normal.x, 0.0);
  EXPECT_DOUBLE_EQ(g.normal.y, -1.0);
  EXPECT_NEAR(g.depth, 0.10, 1e-15);
  EXPECT_NEAR(g.point.x, 0.225, 1e-12);
  EXPECT_NEAR(g.point.y, 0.0, 1e-15);
}

TEST(SegmentContact, ACentreExactlyOnTheLineKeepsTheWindingNormal) {
  // Measure-zero tie, resolved deterministically: the raw normal (p to q
  // rotated a quarter turn counter-clockwise) is kept, exactly as
  // rectangle_contact keeps the axis orientation at zero centre distance.
  const ContactBody b = box(0.0, 0.0);
  const ContactGeometry g =
      segment_contact(Vec2{-1.0, 0.0}, Vec2{1.0, 0.0}, b);
  ASSERT_TRUE(g.touching);
  EXPECT_EQ(g.normal.x, 0.0);
  EXPECT_EQ(g.normal.y, 1.0);
  EXPECT_DOUBLE_EQ(g.depth, 0.15);
}

TEST(SegmentContact, ImmovableWallReflectsTheBody) {
  ContactBody b = box(0.2, 0.0);
  b.velocity = Vec2{2.0, 0.0};
  const Vec2 p{0.4, -1.0};
  const Vec2 q{0.4, 1.0};
  const ContactGeometry g = segment_contact(p, q, b);
  ASSERT_TRUE(g.touching);

  ContactParams params;
  params.restitution = 0.6;
  params.restitution_min_speed = 0.0;
  const ContactImpulse r = resolve_contact(wall_of(p, q), b, g, params);

  // The wall's deltas are exactly zero: immovability is not approximate.
  EXPECT_EQ(r.delta_velocity_a.x, 0.0);
  EXPECT_EQ(r.delta_velocity_a.y, 0.0);
  EXPECT_EQ(r.delta_yaw_rate_a, 0.0);
  EXPECT_EQ(r.delta_position_a.x, 0.0);
  EXPECT_EQ(r.delta_position_a.y, 0.0);

  // A central hit on a unit mass: the body leaves at -e times its
  // approach, and takes the whole positional correction.
  EXPECT_NEAR(b.velocity.x + r.delta_velocity_b.x, -0.6 * 2.0, 1e-12);
  EXPECT_NEAR(r.delta_position_b.x, -g.depth, 1e-15);
  EXPECT_EQ(r.delta_yaw_rate_b, 0.0);
}

TEST(SegmentContact, ApproachIsTheBodysAloneAndSeparationGetsNoImpulse) {
  const Vec2 p{0.4, -1.0};
  const Vec2 q{0.4, 1.0};
  ContactBody b = box(0.2, 0.0);
  b.velocity = Vec2{2.0, 0.0};
  const ContactGeometry g = segment_contact(p, q, b);
  ASSERT_TRUE(g.touching);
  const ContactParams params;
  const ContactImpulse closing = resolve_contact(wall_of(p, q), b, g, params);
  EXPECT_EQ(closing.approach_a, 0.0);
  EXPECT_NEAR(closing.approach_b, 2.0, 1e-12);
  EXPECT_GT(closing.jn, 0.0);

  // Penetrating but already separating: the projection still applies, the
  // impulse does not (pushing a leaving body would add energy).
  b.velocity = Vec2{-2.0, 0.0};
  const ContactImpulse leaving = resolve_contact(wall_of(p, q), b, g, params);
  EXPECT_EQ(leaving.jn, 0.0);
  EXPECT_EQ(leaving.delta_velocity_b.x, 0.0);
  EXPECT_NEAR(leaving.delta_position_b.x, -g.depth, 1e-15);
}

TEST(SegmentContact, MirrorSymmetryHoldsBitForBit) {
  // A slanted wall against a yawed, translating, spinning body: nothing
  // about the setup is symmetric except the mirror itself.
  ContactBody b = box(0.1, 0.05, 0.3);
  b.velocity = Vec2{1.3, -0.4};
  b.yaw_rate = 0.7;
  const Vec2 p{0.3, -0.6};
  const Vec2 q{0.45, 0.7};

  ContactParams params;
  params.restitution = 0.4;

  const ContactGeometry g = segment_contact(p, q, b);
  ASSERT_TRUE(g.touching);
  const ContactImpulse r = resolve_contact(wall_of(p, q), b, g, params);
  ASSERT_GT(r.jn, 0.0);

  const ContactBody bm = mirrored(b);
  const Vec2 pm{p.x, -p.y};
  const Vec2 qm{q.x, -q.y};
  const ContactGeometry gm = segment_contact(pm, qm, bm);
  ASSERT_TRUE(gm.touching);
  const ContactImpulse rm = resolve_contact(wall_of(pm, qm), bm, gm, params);

  // EXPECT_EQ on doubles, deliberately: the promise is bit-identity.
  EXPECT_EQ(g.depth, gm.depth);
  EXPECT_EQ(g.normal.x, gm.normal.x);
  EXPECT_EQ(g.normal.y, -gm.normal.y);
  EXPECT_EQ(g.point.x, gm.point.x);
  EXPECT_EQ(g.point.y, -gm.point.y);
  EXPECT_EQ(r.jn, rm.jn);
  EXPECT_EQ(r.jt, -rm.jt);
  EXPECT_EQ(r.delta_velocity_b.x, rm.delta_velocity_b.x);
  EXPECT_EQ(r.delta_velocity_b.y, -rm.delta_velocity_b.y);
  EXPECT_EQ(r.delta_yaw_rate_b, -rm.delta_yaw_rate_b);
  EXPECT_EQ(r.delta_position_b.x, rm.delta_position_b.x);
  EXPECT_EQ(r.delta_position_b.y, -rm.delta_position_b.y);
}

}  // namespace
