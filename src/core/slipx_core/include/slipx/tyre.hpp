// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// MF-lite: the reduced Magic Formula tyre (CORE-06).
//
// The second piece of L2, and like slipx/load_transfer.hpp it is a pure
// function with no model around it yet, for the same reason: a tyre curve can
// be checked against closed-form properties (its slope at the origin, the
// height of its peak, how both move with vertical load) before anything
// integrates it, and a sign error found here is found in the one place it is
// visible.
//
// Public, header-only, allocates nothing, reads nothing outside its arguments
// (CORE-01, CORE-03). Somebody embedding SlipX to drive their own chassis model
// wants the tyre, and this is the whole of it.
//
// ============================================================================
// The curve
// ============================================================================
//
// The Magic Formula's shape term, with the sine, arctangent and the curvature
// correction that gives it a falling branch past the peak:
//
//   s(x) = sin(C * atan(B x - E * (B x - atan(B x))))
//
// and the lateral force at a vertical load Fz, in the ISO 8855 frame:
//
//   Fy = -mu_y(Fz) * Fz * s(alpha)
//
// The minus sign is the ISO restoring sign of conventions.hpp: a positive slip
// angle means the wheel's velocity lies to the LEFT of the wheel plane, and the
// force opposes it. It is the same sign L1's linear tyre carries, and the two
// agree exactly at small slip, which is not a coincidence; see below.
//
// Three properties of s(x) are worth knowing because the tests below assert
// them and because they are what separates this from L1's clip:
//
//   s is odd, so the tyre behaves identically in both directions.
//   |s| <= 1, so the force never exceeds mu Fz. It is a real bound and not a
//     clip: the curve approaches it smoothly and reaches it exactly once.
//   s has a genuine peak at finite slip and FALLS beyond it, provided C > 1
//     and E < 1. That falling branch is the mechanism by which a car spins.
//     L1 has no such branch: it slides at the clip and recovers the instant the
//     slip angle comes back, which real cars do not do.
//
// ============================================================================
// Load sensitivity
// ============================================================================
//
// A tyre's friction coefficient falls as it is pushed harder into the road:
//
//   mu(Fz) = mu_0 * (Fz / Fz_nom)^(-k_mu)
//
// k_mu is in the parameter list because a skidpad driven at two ballast
// configurations identifies it, which is the admission criterion of ADR-0009.
// It is the reason load transfer costs a car grip rather than merely moving it
// around: the outer tyre gains less than the inner one loses.
//
// The peak force is therefore mu_0 Fz_nom^k_mu Fz^(1-k_mu), which is
// sub-linear in load. Written that way rather than as mu(Fz) * Fz because
// mu(Fz) diverges as Fz goes to zero while the product does not, and a lifted
// wheel has exactly zero load. See peak_lateral_force.
//
// ============================================================================
// Where B comes from, and why it is not a parameter here
// ============================================================================
//
// B is derived at construction rather than read:
//
//   B = C_alpha / (C * mu_y0 * Fz_nom)
//
// The reasoning is ADR-0009's, applied to itself. C_alpha, the cornering
// stiffness, is identifiable: it is the low-slip slope of a skidpad. B alone is
// not, because only the product B * C * mu * Fz has a measurable meaning, and
// asking a team to identify B and C_alpha separately is asking them to
// identify the same number twice and then reconcile it.
//
// Deriving it buys a second thing. Expanding s(x) for small x gives
// s ~ C B alpha, so Fy ~ -C_alpha alpha exactly, which is L1's linear tyre.
// L2 agrees with L1 in the low-slip limit by construction rather than by luck,
// and the cross-tier convergence test measures a discretisation error rather
// than a parameter mismatch.
//
// B stays fixed as the load changes, so the cornering stiffness inherits the
// load sensitivity of mu:
//
//   C_alpha(Fz) = B C mu(Fz) Fz = C_alpha * (Fz / Fz_nom)^(1 - k_mu)
//
// which is the degressive behaviour a real tyre has, and it arrives without a
// second load-sensitivity parameter.
//
// ============================================================================
// Where the peak lands, and the trap in C and E
// ============================================================================
//
// Deriving B fixes the slope at the origin and mu fixes the height of the peak,
// so C and E are left deciding one thing between them: how far out the peak
// sits. The natural scale for that is the slip angle at which the linear tyre
// would have reached the peak,
//
//   alpha_lin = mu_y Fz / C_alpha
//
// and the actual peak is a multiple of it that depends on C and E alone, not on
// B, mu or the load. For a real tyre the multiple is somewhere between about
// 1.5 and 3.
//
// The trap is that the multiple is very sensitive to the pair, and the two
// bounds in tyre.schema.json are independent. C = 1.05 with E = 0.87 is legal
// and gives a multiple of about 21, which is a curve so flat that its peak is
// at a slip angle no car reaches: the schema's C > 1 bound was meant to keep
// out "a curve with no peak", and on its own it does not. Choosing C near the
// top of its band and E well below 1 keeps the peak where a tyre has one. The
// reference coefficients below are C = 1.68 and E = 0.42, a multiple of 2.7.
//
// This is a property of the Magic Formula rather than of this implementation,
// and it is stated here because an identified set that fits the low-slip data
// perfectly can still describe a tyre that never lets go.
//
// ============================================================================
// What is not here
// ============================================================================
//
// The longitudinal force law. Fx from slip ratio needs a slip stiffness
// C_kappa, the longitudinal counterpart of C_alpha, and tyre.schema.json at
// schema 0.1.0 has no field for it. C_kappa IS identifiable in a car park, from
// encoder slip ratio against IMU acceleration, so it belongs in the parameter
// set; it is simply not in the published one yet, and defaulting it silently is
// what SCH-02 forbids. Adding it is a schema change, and this header stops
// short of it rather than inventing a number.
//
// What is here is the half of combined slip that does not depend on that
// decision: friction_ellipse takes a longitudinal and a lateral force demand
// from wherever they came from and returns the pair the tyre can actually
// deliver. It is exact, testable on its own, and unchanged by whatever the
// longitudinal law turns out to be.
//
// Also absent, and absent deliberately: camber, turn slip, ply steer,
// conicity, self-aligning moment and the several Pacejka scaling factors. None
// of them is identifiable from a car park, and ADR-0009 is that an
// unidentifiable parameter is worse than an absent one.

#ifndef SLIPX_TYRE_HPP
#define SLIPX_TYRE_HPP

#include <cmath>

#include "slipx/math.hpp"

namespace slipx {

// The coefficients that come out of a tyre file, per TYRE rather than per
// axle. Every one is identifiable from a manoeuvre drivable in a car park with
// the sensors already on a competition car (ADR-0009); the table in that record
// names the manoeuvre for each.
//
// Cornering stiffness is NOT here. It arrives separately, as an argument to
// make_mf_lite, because it is already in VehicleParams for L1's linear tyre and
// carrying it twice would let the two copies disagree.
//
// The defaults are provisional in the sense of Provenance::kProvisional: they
// are plausible for a soft 1/10-scale tyre on carpet and have been measured
// against nothing.
struct TyreCoefficients {
  double mu_y0 = 1.10;   // peak lateral friction at the nominal load    [-]
  double mu_x0 = 1.20;   // peak longitudinal friction, nominal load     [-]
  double k_mu = 0.15;    // load sensitivity exponent, positive: mu
                         // falls as load rises                          [-]

  // Magic Formula shape factors. Dimensionless, and neither carries a sign
  // convention: the sign of the force lives in the force law. They are not
  // independent of each other in any way that matters: see the note above on
  // where the peak lands.
  // Relaxation length: the distance this tyre must roll before its lateral
  // force reaches the steady-state value for the slip it is being given
  // (CORE-07, ADR-0026). A property of the tyre rather than of the car wearing
  // it, which is why it lives here beside the other coefficients and matches
  // tyre.schema.json's relaxation.sigma. relaxation.hpp carries the
  // derivation. No effect below L2, which has no tyre transient.
  //
  // Identifiable from the rise time of yaw rate in a step steer at two speeds.
  // Provisional: roughly 1.5 wheel radii, which is where a full-size tyre
  // sits, measured on nothing.
  double relax_length = 0.08;                                  //         [m]

  double shape_c = 1.68;      // C. Above 1 or the curve has no peak     [-]
  double curvature_e = 0.42;  // E. Above 1 the curve folds back on
                              // itself and stops being a tyre           [-]
};

// One tyre, as the model uses it: the coefficients above with B and the nominal
// load resolved. Produced by make_mf_lite and then read-only, so that the
// derivation happens once at construction rather than per step (CORE-03).
struct MfLite {
  double b = 0.0;        // stiffness factor, derived. Per radian of
                         // slip angle for the lateral branch            [-]
  double c = 1.68;       // shape factor                                 [-]
  double e = 0.42;       // curvature factor                             [-]

  double mu_y0 = 1.10;   //                                              [-]
  double mu_x0 = 1.20;   //                                              [-]
  double k_mu = 0.15;    //                                              [-]

  // The load the coefficients describe, and the load at which the derived B
  // makes this tyre agree with the linear one. Per tyre.
  //
  // This is the STATIC per-tyre load of the car wearing the tyre, computed
  // from mass and weight distribution, rather than the nominal_load field of
  // tyre.schema.json. Two reasons: the core needs no new parameter for it, and
  // the static case is the one every other result is compared against, so
  // making it the reference point means the cross-tier comparison starts from
  // an exact agreement rather than a nearly-exact one.
  double fz_nom = 1.0;   //                                              [N]
};

// Build a tyre. c_alpha is the cornering stiffness of ONE tyre at fz_nom,
// positive, in [N/rad]; an axle value from VehicleParams is twice this.
// fz_nom is the static vertical load on that one tyre, in [N], and must be
// positive.
//
// No validation and no clamping: this is the core, and a parameter set that
// does not describe a possible tyre is rejected at the boundary by
// slipx::validate and by slipx_schema, not silently repaired here.
inline MfLite make_mf_lite(const TyreCoefficients& t, double c_alpha,
                           double fz_nom) {
  MfLite out;
  out.b = c_alpha / (t.shape_c * t.mu_y0 * fz_nom);
  out.c = t.shape_c;
  out.e = t.curvature_e;
  out.mu_y0 = t.mu_y0;
  out.mu_x0 = t.mu_x0;
  out.k_mu = t.k_mu;
  out.fz_nom = fz_nom;
  return out;
}

// The Magic Formula shape term, s(x) above. Dimensionless, odd, bounded by 1.
//
// slip is a slip angle in radians for the lateral branch and a dimensionless
// slip ratio for the longitudinal one, which is why it is named neither.
inline double mf_shape(double b, double c, double e, double slip) {
  const double bx = b * slip;
  const double atan_bx = std::atan(bx);
  return std::sin(c * std::atan(bx - e * (bx - atan_bx)));
}

// Friction coefficient at a vertical load [-]. Domain: fz > 0 and fz_nom > 0.
//
// Diverges as fz goes to zero, which is correct and is why the force laws
// below do not call it. Exported because it is the quantity a user plotting a
// friction budget wants, and because it is the one place the sign of the
// exponent is written down.
inline double mu_at_load(double mu0, double k_mu, double fz, double fz_nom) {
  return mu0 * std::pow(fz / fz_nom, -k_mu);
}

// The peak force law is mu_y0 Fz_nom^k_mu Fz^(1-k_mu), and it splits cleanly
// into a half that depends on the tyre alone and a half that depends on the
// load alone. The two functions below are those halves, and the product of
// them in that order is exactly peak_lateral_force.
//
// The split is here because of what L2 does with it. A double-track step
// evaluates both peaks at four wheels twice per derivative and five times per
// step, and Fz_nom^k_mu is the same number every one of those times: hoisting
// it turned forty calls to pow per step into eight. The grouping is written
// out rather than left to the compiler because the compiler may not regroup
// it, and because a caller that hoists is entitled to the same bits as one
// that does not.

// The tyre-only half, mu0 Fz_nom^k_mu, for both branches. Units are awkward
// and that is honest: the halves are not forces, only their product is.
struct NominalPeak {
  double x = 0.0;   // longitudinal
  double y = 0.0;   // lateral
};

inline NominalPeak nominal_peak(const MfLite& t) {
  const double nominal = std::pow(t.fz_nom, t.k_mu);
  return {t.mu_x0 * nominal, t.mu_y0 * nominal};
}

// The load-only half, Fz^(1-k_mu). Exactly zero at zero load, which is the
// whole reason the law is grouped this way: a lifted wheel must give zero
// force and not the inf * 0 = NaN that the literal mu(Fz) * Fz produces.
// Zero load is not an edge case at L2, where quasi_static_loads clamps a
// lifted wheel to exactly zero, so any car that reaches its rollover
// threshold takes this path.
inline double load_factor(const MfLite& t, double fz) {
  if (fz <= 0.0) return 0.0;
  return std::pow(fz, 1.0 - t.k_mu);
}

// Peak lateral force magnitude at a vertical load [N]. Never negative.
inline double peak_lateral_force(const MfLite& t, double fz) {
  return nominal_peak(t).y * load_factor(t, fz);
}

// Peak longitudinal force magnitude at a vertical load [N]. Never negative.
inline double peak_longitudinal_force(const MfLite& t, double fz) {
  return nominal_peak(t).x * load_factor(t, fz);
}

// Pure-slip lateral force [N] at slip angle alpha [rad] and vertical load
// fz [N].
//
// ISO 8855: positive alpha gives negative Fy. See conventions.hpp, and
// test_conventions.cpp, which asserts it rather than trusting this comment.
inline double mf_lite_fy(const MfLite& t, double alpha, double fz) {
  return -peak_lateral_force(t, fz) * mf_shape(t.b, t.c, t.e, alpha);
}

// Cornering stiffness at a vertical load [N/rad], positive. The slope of
// -Fy against alpha at the origin.
//
//   C_alpha(Fz) = C_alpha(Fz_nom) * (Fz / Fz_nom)^(1 - k_mu)
//
// Not used by the force law, which gets the same behaviour out of a fixed B.
// It is here because it is the quantity an identification run measures, so a
// user checking a fitted set against their skidpad data wants to read it back.
inline double cornering_stiffness_at_load(const MfLite& t, double fz) {
  return t.b * t.c * peak_lateral_force(t, fz);
}

// A tyre force pair after the friction budget has been applied.
struct CombinedForce {
  double fx = 0.0;       // longitudinal, positive forward               [N]
  double fy = 0.0;       // lateral, positive left (ISO 8855)            [N]

  // The demand exceeded the ellipse and both components were scaled down. The
  // point past which the tyre is doing everything it can, and the flag
  // StepDiagnostics::tyre_saturated reports at L2.
  bool saturated = false;
};

// Combined slip, as a friction ellipse (CORE-06).
//
// A tyre has one contact patch and one friction budget, so longitudinal and
// lateral force compete for it: a tyre braking at its limit has nothing left to
// steer with. The budget is an ellipse rather than a circle because mu_x and
// mu_y differ, and the demand is projected onto it along its own direction:
//
//   d = sqrt((fx / fx_max)^2 + (fy / fy_max)^2)
//   inside  (d <= 1): the tyre delivers what was asked
//   outside (d >  1): both components are scaled by 1/d
//
// Scaling both by the same factor is what makes this a projection along the
// demand direction rather than a clip of each axis separately. The ratio
// fx : fy is preserved, so a driver braking and steering together loses grip in
// both at once instead of losing all the steering first, which is the failure
// mode of an axis-wise clip and is the reason this is one function rather than
// two clamps at the call sites.
//
// fx_max and fy_max are the peak force magnitudes at that wheel's current load,
// from peak_longitudinal_force and peak_lateral_force. A wheel with zero load
// has a zero budget and delivers nothing, which is handled explicitly because
// the division is not.
//
// The limitation, stated plainly: an ellipse is a friction budget and not a
// combined-slip tyre model. A full Magic Formula derives the interaction from
// weighting functions of both slips, which reproduces the way peak lateral
// force moves to a different slip angle under braking. This does not. What it
// gets right is the bound, which is what decides whether the car makes the
// corner.
inline CombinedForce friction_ellipse(double fx, double fy, double fx_max,
                                      double fy_max) {
  CombinedForce out;
  if (fx_max <= 0.0 || fy_max <= 0.0) {
    out.saturated = (fx != 0.0 || fy != 0.0);
    return out;
  }

  const double nx = fx / fx_max;
  const double ny = fy / fy_max;

  // A tyre well inside its budget needs no hypot to prove it. The squared
  // demand is three instructions against a call, and the threshold carries a
  // margin far wider than any rounding hypot could do: nx^2 + ny^2 <= 0.99
  // puts the demand below 0.995, so no value that takes this branch could
  // have failed the exact test below. Anything nearer the limit than that,
  // including everything at the limit, still goes through hypot.
  //
  // The guard is there because L2 evaluates this fifty times a step and a
  // hypot is worth a dozen multiplies. Overflow does not slip through it: an
  // enormous nx squares to infinity, which fails the comparison and falls to
  // the exact path, which is what hypot is careful about in the first place.
  if (nx * nx + ny * ny <= 0.99) {
    out.fx = fx;
    out.fy = fy;
    return out;
  }

  const double demand = std::hypot(nx, ny);
  if (demand <= 1.0) {
    out.fx = fx;
    out.fy = fy;
    return out;
  }

  const double scale = 1.0 / demand;
  out.fx = fx * scale;
  out.fy = fy * scale;
  out.saturated = true;
  return out;
}

}  // namespace slipx

#endif  // SLIPX_TYRE_HPP
