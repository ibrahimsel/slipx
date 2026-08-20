// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Planar rigid-body contact: one impulse with restitution and Coulomb
// friction between two oriented rectangles (ADR-0043), or between a
// rectangle and an immovable wall segment (ADR-0055).
//
// This is the mathematics of a collision and nothing else: who touches whom,
// and when, is the orchestrator's business, exactly as halting an agent is
// (ADR-0042). It sits in the core for the same reason load_transfer.hpp
// does: it is closed-form rigid-body mechanics, testable against momentum
// conservation with nothing else in the room, and somebody embedding SlipX
// with their own orchestrator needs exactly this and nothing above it.
// Header-only, allocates nothing, reads nothing outside its arguments.
//
// ============================================================================
// What this model is, and is not
// ============================================================================
//
// PLAUSIBLE AND DETERMINISTIC, NOT FITTED. Nothing here has been identified
// from data, and nothing here could be: restitution and friction between two
// foam-and-plastic bodies are not identifiable in a car park, and
// crash-testing is not a manoeuvre. What IS promised, and held by the
// invariant suite: momentum is conserved exactly as the impulse formulation
// conserves it, the friction impulse never leaves the Coulomb cone, mirror
// symmetry holds bit for bit, and the same inputs give the same outputs.
//
// One impulse per contact, no convergence loop (the pass count is part of
// the trajectory, ADR-0027), and one contact point per pair: the midpoint of
// the clipped incident edge. Two cars pressed perfectly flush therefore
// exchange a single central push and no couple, which is a stated
// simplification of compliant multi-point bumper contact. Penetration left
// behind by the discrete step is removed by a positional projection along
// the normal, split by inverse mass; this is a separator, not a stacking
// solver, and piling cars on top of each other is outside what it promises.
//
// Sign conventions are ISO 8855 as everywhere (conventions.hpp): world z up,
// yaw positive counter-clockwise, so a positive scalar cross r.cross(j) is a
// counter-clockwise (leftward) yaw impulse.

#ifndef SLIPX_CONTACT_HPP
#define SLIPX_CONTACT_HPP

#include <array>
#include <cmath>

#include "slipx/math.hpp"

namespace slipx {

// A rigid planar body, as a collision sees it. World frame throughout.
//
// The rectangle is the body's footprint: half_length forward and back,
// half_width to each side of `centre`, rotated by yaw. The centre may sit
// away from the CoG (a car's body is centred between the axles, its CoG
// need not be); impulse arms are measured from the CoG, which is what `cog`
// and `velocity` describe.
//
// Immovability is expressed the standard way: inv_mass and inv_izz are the
// RECIPROCALS, and zero means the body cannot be moved by any impulse. A
// DNF'd car (ADR-0042) enters with both at zero.
struct ContactBody {
  Vec2 cog;                  // centre of gravity, world           [m]
  double yaw = 0.0;          // heading, positive CCW              [rad]
  Vec2 velocity;             // CoG velocity, world                [m/s]
  double yaw_rate = 0.0;     //                                    [rad/s]
  double inv_mass = 0.0;     // 1/m, 0 = immovable                 [1/kg]
  double inv_izz = 0.0;      // 1/Izz, 0 = cannot be spun          [1/(kg m^2)]
  double centre_offset = 0.0;  // rectangle centre forward of the CoG,
                               // body frame. A car's footprint centre is
                               // its wheelbase midpoint, (lf - lr) / 2. [m]
  double half_length = 0.0;  //                                    [m]
  double half_width = 0.0;   //                                    [m]
};

// The collision constants. Properties of a pair of bodies colliding, not of
// one car, and PROVISIONAL BY NATURE: plausible for foam bumpers on plastic
// shells, identified from nothing. See the header note.
struct ContactParams {
  // Coefficient of restitution: 0 is a dead stop, 1 an elastic bounce.
  double restitution = 0.3;                                     //     [-]

  // Coulomb friction at the contact: the tangential impulse is clamped to
  // this multiple of the normal impulse.
  double friction = 0.5;                                        //     [-]

  // Below this closing speed the restitution is treated as zero, so two
  // cars rubbing side by side push apart instead of chattering. An
  // anti-jitter device, not physics, and documented as one.
  double restitution_min_speed = 0.1;                           //  [m/s]
};

// Where and how deeply two footprints touch.
struct ContactGeometry {
  bool touching = false;
  Vec2 normal;         // unit, pointing from body a toward body b
  double depth = 0.0;  // penetration along the normal, positive     [m]
  Vec2 point;          // contact point, world                       [m]
};

// What a resolved contact does to each body. Velocity deltas are world
// frame at the CoG; position deltas are the penetration projection. An
// immovable body's deltas are exactly zero.
struct ContactImpulse {
  double jn = 0.0;     // normal impulse magnitude, never negative  [N s]
  double jt = 0.0;     // tangential impulse, signed                [N s]
  Vec2 delta_velocity_a, delta_velocity_b;                     //   [m/s]
  double delta_yaw_rate_a = 0.0, delta_yaw_rate_b = 0.0;       // [rad/s]
  Vec2 delta_position_a, delta_position_b;                     //     [m]

  // Each body's contribution to the closing speed at the contact point,
  // along the normal and positive TOWARD the other body, yaw included.
  // Their sum is the closing speed. Reported for every touching pair,
  // approaching or not, because who was moving at whom is the fact a race
  // referee attributes fault from (ADR-0046) and it is already computed
  // here; a separating pair simply reports non-positive numbers.
  double approach_a = 0.0;   //                                     [m/s]
  double approach_b = 0.0;   //                                     [m/s]
};

namespace contact_detail {

// The rectangle's corners and axes, computed once. Fixed corner order:
// front-left, front-right, rear-right, rear-left, so that consecutive
// corners share an edge and the incident-edge walk below is a plain index
// step.
struct Rect {
  Vec2 centre;
  Vec2 fwd;   // unit, body x
  Vec2 lat;   // unit, body y (left)
  double hl = 0.0;
  double hw = 0.0;
  std::array<Vec2, 4> corner;
};

inline Rect make_rect(const ContactBody& b) {
  Rect r;
  const double c = std::cos(b.yaw);
  const double s = std::sin(b.yaw);
  r.fwd = Vec2{c, s};
  r.lat = Vec2{-s, c};
  r.centre = b.cog + r.fwd * b.centre_offset;
  r.hl = b.half_length;
  r.hw = b.half_width;
  r.corner[0] = r.centre + r.fwd * r.hl + r.lat * r.hw;   // front-left
  r.corner[1] = r.centre + r.fwd * r.hl - r.lat * r.hw;   // front-right
  r.corner[2] = r.centre - r.fwd * r.hl - r.lat * r.hw;   // rear-right
  r.corner[3] = r.centre - r.fwd * r.hl + r.lat * r.hw;   // rear-left
  return r;
}

// Half-extent of a rectangle projected onto a unit axis.
inline double extent_on(const Rect& r, const Vec2& axis) {
  return r.hl * std::abs(axis.dot(r.fwd)) + r.hw * std::abs(axis.dot(r.lat));
}

}  // namespace contact_detail

// Separating-axis test between the two footprints, with the contact point
// taken as the midpoint of the incident edge clipped to the reference face.
//
// Determinism notes, because they are the point. The candidate axes are
// visited in a fixed order (a's forward, a's lateral, b's forward, b's
// lateral) and the minimum-overlap comparison is strict, so ties resolve to
// the earlier axis, always. Every projection is built from dot products
// whose values are preserved exactly under a left-right mirror (products of
// two negated terms), which is what makes the mirror-symmetry invariant
// hold bit for bit rather than approximately.
inline ContactGeometry rectangle_contact(const ContactBody& a,
                                         const ContactBody& b) {
  using contact_detail::Rect;
  using contact_detail::extent_on;
  using contact_detail::make_rect;

  ContactGeometry out;
  const Rect ra = make_rect(a);
  const Rect rb = make_rect(b);
  const Vec2 d = rb.centre - ra.centre;

  const std::array<Vec2, 4> axes = {ra.fwd, ra.lat, rb.fwd, rb.lat};
  double min_overlap = 0.0;
  int min_axis = -1;
  for (int i = 0; i < 4; ++i) {
    const double overlap =
        extent_on(ra, axes[i]) + extent_on(rb, axes[i]) -
        std::abs(d.dot(axes[i]));
    if (overlap <= 0.0) return out;   // separated on this axis
    if (min_axis < 0 || overlap < min_overlap) {
      min_overlap = overlap;
      min_axis = i;
    }
  }

  // Normal along the minimum-overlap axis, pointing from a toward b. A
  // centre distance of exactly zero on that axis keeps the axis' own
  // orientation, deterministically.
  Vec2 n = axes[static_cast<std::size_t>(min_axis)];
  if (d.dot(n) < 0.0) n = -n;

  // Reference body: the one whose axis was chosen. Incident edge: the edge
  // of the other body whose outward normal is most anti-parallel to n. Edge
  // e joins corner[e] and corner[e+1], with outward normals in the same
  // fixed order: fwd, -lat, -fwd, lat.
  const bool ref_is_a = min_axis < 2;
  const Rect& ref = ref_is_a ? ra : rb;
  const Rect& inc = ref_is_a ? rb : ra;
  // n points from a to b; the reference face's outward normal points at the
  // incident body.
  const Vec2 ref_out = ref_is_a ? n : -n;

  const std::array<Vec2, 4> inc_normals = {inc.fwd, -inc.lat, -inc.fwd,
                                           inc.lat};
  int inc_edge = 0;
  double most_anti = inc_normals[0].dot(ref_out);
  for (int e = 1; e < 4; ++e) {
    const double align = inc_normals[static_cast<std::size_t>(e)].dot(ref_out);
    if (align < most_anti) {
      most_anti = align;
      inc_edge = e;
    }
  }
  Vec2 p0 = inc.corner[static_cast<std::size_t>(inc_edge)];
  Vec2 p1 = inc.corner[static_cast<std::size_t>((inc_edge + 1) % 4)];

  // Clip the incident edge to the reference face's side planes (the two
  // planes perpendicular to the face, at the face's ends).
  const bool face_is_fwd = (min_axis % 2) == 0;
  const Vec2 side = face_is_fwd ? ref.lat : ref.fwd;
  const double side_extent = face_is_fwd ? ref.hw : ref.hl;
  for (const double sign_s : {1.0, -1.0}) {
    // Keep points with side-coordinate <= side_extent.
    const double s0 = sign_s * side.dot(p0 - ref.centre);
    const double s1 = sign_s * side.dot(p1 - ref.centre);
    if (s0 > side_extent && s1 > side_extent) {
      // The whole edge is beyond a side plane: corner-on-corner grazing.
      // Fall back to the nearer endpoint, clamped; overlap said they touch.
      const Vec2 keep = s0 < s1 ? p0 : p1;
      p0 = keep;
      p1 = keep;
      break;
    }
    if (s0 > side_extent) {
      p0 = p0 + (p1 - p0) * ((s0 - side_extent) / (s0 - s1));
    } else if (s1 > side_extent) {
      p1 = p1 + (p0 - p1) * ((s1 - side_extent) / (s1 - s0));
    }
  }

  // Keep the clipped points at or below the reference face; the midpoint of
  // what remains is the contact point. At least one clipped point is below
  // the face whenever the boxes overlap, but floating point is floating
  // point, so the both-above case falls back to the deeper endpoint rather
  // than asserting.
  const double face = extent_on(ref, ref_out);
  const double d0 = ref_out.dot(p0 - ref.centre) - face;
  const double d1 = ref_out.dot(p1 - ref.centre) - face;
  if (d0 <= 0.0 && d1 <= 0.0) {
    out.point = (p0 + p1) * 0.5;
  } else if (d0 <= 0.0) {
    out.point = p0;
  } else if (d1 <= 0.0) {
    out.point = p1;
  } else {
    out.point = d0 < d1 ? p0 : p1;
  }

  out.touching = true;
  out.normal = n;
  out.depth = min_overlap;
  return out;
}

// Contact between an immovable wall segment and a footprint (ADR-0055).
//
// The wall is the segment from p to q, with no thickness. Contact exists
// when the segment passes through the body's rectangle, and the normal is
// the segment line's unit normal oriented toward the rectangle's CENTRE, so
// the resolution always pushes the body back to the side its centre is on.
// Resolving along the minimum-overlap axis instead would push a body whose
// centre had crossed the line out the far side, which is how a car squeezes
// through a wall; the centre-side rule cannot, provided one step's motion
// stays below the footprint's smallest half-extent (at 1 kHz and 40 m/s a
// car moves 4 cm per step against a 0.15 m half-width, so the failure is
// unreachable; a step-size change must re-check this arithmetic, exactly as
// the pair test's tunnelling note says).
//
// The contact point is the midpoint of the segment clipped to the
// rectangle. Near a polyline joint two segments each report a contact and
// the caller resolves them in its fixed order, the same order-dependence
// the pair pass accepts. A body centre exactly on the line keeps the
// normal the segment's own winding gives (p to q rotated +90 degrees),
// deterministically, as rectangle_contact does for a zero centre distance.
//
// Returned with the wall as body a: hand the result to
// resolve_contact(wall, body, geometry, params) with an immovable wall
// body (both reciprocals zero, at rest) and the normal points from the
// wall toward the body, exactly as that function expects.
inline ContactGeometry segment_contact(const Vec2& p, const Vec2& q,
                                       const ContactBody& body) {
  using contact_detail::Rect;
  using contact_detail::extent_on;
  using contact_detail::make_rect;

  ContactGeometry out;
  const Vec2 e = q - p;
  const double len2 = e.dot(e);
  if (!(len2 > 0.0)) return out;   // a degenerate segment touches nothing

  const Rect r = make_rect(body);

  // The segment in the rectangle's frame, then a Liang-Barsky clip against
  // the box. Fixed axis order, so ties resolve the same way every time.
  const Vec2 rel_p = p - r.centre;
  const Vec2 rel_q = q - r.centre;
  const double start[2] = {rel_p.dot(r.fwd), rel_p.dot(r.lat)};
  const double delta[2] = {rel_q.dot(r.fwd) - start[0],
                           rel_q.dot(r.lat) - start[1]};
  const double extent[2] = {r.hl, r.hw};
  double t0 = 0.0;
  double t1 = 1.0;
  for (int axis = 0; axis < 2; ++axis) {
    if (delta[axis] == 0.0) {
      // Parallel to this slab: inside it or not, for the whole segment.
      if (std::abs(start[axis]) > extent[axis]) return out;
      continue;
    }
    double enter = (-extent[axis] - start[axis]) / delta[axis];
    double exit = (extent[axis] - start[axis]) / delta[axis];
    if (enter > exit) {
      const double swap = enter;
      enter = exit;
      exit = swap;
    }
    if (enter > t0) t0 = enter;
    if (exit < t1) t1 = exit;
    if (t0 > t1) return out;   // misses the rectangle
  }

  // The line normal, oriented toward the body's centre (see the header
  // note: this is what stops a wall being squeezed through).
  const double len = std::sqrt(len2);
  Vec2 n{-e.y / len, e.x / len};
  const double side = n.dot(r.centre - p);
  if (side < 0.0) n = -n;

  // How far the rectangle reaches past the line on the centre's side. The
  // clip said the segment crosses the rectangle, so this is positive up to
  // corner grazing, which is excluded exactly as kissing faces are.
  const double depth = extent_on(r, n) - std::abs(side);
  if (!(depth > 0.0)) return out;

  out.touching = true;
  out.normal = n;
  out.depth = depth;
  out.point = (p + e * t0 + p + e * t1) * 0.5;
  return out;
}

// One impulse with restitution and Coulomb friction at the contact point,
// plus the positional projection that removes the penetration. Pure: apply
// the returned deltas or do not, nothing here has side effects.
inline ContactImpulse resolve_contact(const ContactBody& a,
                                      const ContactBody& b,
                                      const ContactGeometry& g,
                                      const ContactParams& p) {
  ContactImpulse out;
  if (!g.touching) return out;

  const Vec2 n = g.normal;
  const Vec2 ra = g.point - a.cog;
  const Vec2 rb = g.point - b.cog;

  // Velocity of each body's material point at the contact, world frame:
  // v + omega x r, which in the plane is omega * perp(r).
  const Vec2 va{a.velocity.x - a.yaw_rate * ra.y,
                a.velocity.y + a.yaw_rate * ra.x};
  const Vec2 vb{b.velocity.x - b.yaw_rate * rb.y,
                b.velocity.y + b.yaw_rate * rb.x};
  const Vec2 v_rel = vb - va;   // of b relative to a
  const double vn = v_rel.dot(n);
  out.approach_a = va.dot(n);
  out.approach_b = -vb.dot(n);

  // Normal impulse, only when the bodies are closing (vn < 0). A pair that
  // is penetrating but already separating gets the positional projection
  // below and no impulse: pushing them apart harder would add energy.
  if (vn < 0.0) {
    // Restitution only above the threshold speed; see ContactParams.
    const double e = (-vn > p.restitution_min_speed) ? p.restitution : 0.0;
    const double ra_x_n = ra.cross(n);
    const double rb_x_n = rb.cross(n);
    const double kn = a.inv_mass + b.inv_mass + a.inv_izz * ra_x_n * ra_x_n +
                      b.inv_izz * rb_x_n * rb_x_n;
    if (kn > 0.0) {
      out.jn = -(1.0 + e) * vn / kn;

      // Coulomb friction along the tangent, clamped to the cone. The
      // tangent is the normal rotated +90 degrees; the sign of the impulse
      // comes out of the relative tangential velocity, so no branch on it.
      const Vec2 t{-n.y, n.x};
      const double vt = v_rel.dot(t);
      const double ra_x_t = ra.cross(t);
      const double rb_x_t = rb.cross(t);
      const double kt = a.inv_mass + b.inv_mass + a.inv_izz * ra_x_t * ra_x_t +
                        b.inv_izz * rb_x_t * rb_x_t;
      if (kt > 0.0) {
        out.jt = clamp(-vt / kt, -p.friction * out.jn, p.friction * out.jn);
      }

      // Equal and opposite at the same point: this line is where momentum
      // conservation comes from, and the invariant test holds it.
      const Vec2 j = n * out.jn + t * out.jt;
      out.delta_velocity_a = -j * a.inv_mass;
      out.delta_velocity_b = j * b.inv_mass;
      out.delta_yaw_rate_a = -a.inv_izz * ra.cross(j);
      out.delta_yaw_rate_b = b.inv_izz * rb.cross(j);
    }
  }

  // Positional projection: the full penetration, split by inverse mass, so
  // the discrete step leaves no overlap behind. A separator, not a stacking
  // solver.
  const double inv_sum = a.inv_mass + b.inv_mass;
  if (inv_sum > 0.0) {
    const double share_a = a.inv_mass / inv_sum;
    const double share_b = b.inv_mass / inv_sum;
    out.delta_position_a = -n * (g.depth * share_a);
    out.delta_position_b = n * (g.depth * share_b);
  }
  return out;
}

}  // namespace slipx

#endif  // SLIPX_CONTACT_HPP
