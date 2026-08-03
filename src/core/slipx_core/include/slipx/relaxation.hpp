// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Tyre relaxation length (CORE-07).
//
// The third piece of L2, and the first one that is a genuine state rather than
// an algebraic function of the state. Load transfer and MF-lite both answer
// "given the car's condition right now, what force?". This one answers "and how
// long does the tyre take to get there?", which needs memory.
//
// Public and header-only for the same reason as load_transfer.hpp and tyre.hpp:
// somebody embedding SlipX around their own tyre model wants the transient, and
// this is the whole of what it is. It allocates nothing and reads nothing
// outside its arguments (CORE-01, CORE-03).
//
// ============================================================================
// The physics, and why the lag is in distance rather than time
// ============================================================================
//
// A tyre does not produce its lateral force the instant the slip angle appears.
// The force comes from the carcass being deflected sideways in the contact
// patch, and that deflection has to be built up by rolling: the tread band is
// laid down against the road ahead of the patch and dragged into position
// behind it. Until the tyre has rolled far enough, the deflection, and so the
// force, is smaller than the steady-state value.
//
// The distance it has to roll is the relaxation length, sigma, in metres. The
// standard first-order model, the "stretched string" of Pacejka chapter 7, is a
// lag in DISTANCE travelled rather than in time:
//
//   sigma d(alpha')/ds + alpha' = alpha
//
// where s is distance rolled. Dividing by ds/dt = vx turns it into the rate
// this header returns:
//
//   d(alpha')/dt = (|vx| / sigma) (alpha - alpha')
//
// The time constant is therefore sigma / |vx| and is not a constant at all: at
// 2 m/s a sigma of 0.08 m is a 40 ms lag, at 20 m/s it is 4 ms. That speed
// dependence is the whole content of the model. A fixed time constant would be
// simpler, would fit one speed, and would be wrong everywhere else.
//
// Two ends of the speed range are worth naming.
//
// As vx goes to zero the rate goes to zero and alpha' freezes at whatever it
// last held. That is correct rather than a degeneracy: a tyre that is not
// rolling cannot change its carcass deflection by rolling, which is why a
// stationary car with the wheel turned sits there rather than developing a
// cornering force. It also means this term introduces no standstill
// singularity, unlike the slip angle it lags, which needs VehicleParams::v_eps.
//
// At the top of the speed range the lag becomes fast and the equation becomes
// stiff. See the note on the step size below, which is a real constraint on a
// fixed-step integrator and is stated rather than guarded against.
//
// ============================================================================
// What is lagged: the slip angle, not the force
// ============================================================================
//
// CORE-07 words the requirement as a "first-order lag on lateral force", and
// this header lags the slip angle instead, feeding the lagged angle to an
// otherwise instantaneous MF-lite. ADR-0026 is the decision and the argument;
// the short version is that a lagged force is not bounded by the friction
// budget it was produced under.
//
// Concretely: a wheel carrying 12 N through a corner is producing about 13 N of
// lateral force. Lift that wheel, so its load and therefore its budget go to
// zero, and a lagged force decays towards zero over sigma / |vx| while the
// tyre it belongs to is in the air. For those few milliseconds the model
// reports a tyre pushing on a road it is not touching. Lagging the slip angle
// cannot do that: the force is evaluated from the current load every step, so
// it is inside the current budget by construction, and only the SLIP the tyre
// thinks it has carries any history.
//
// The cost is that this is a lag on the input to a nonlinear map rather than on
// its output, so the two agree exactly only where the map is linear, which is
// the small-slip region. Near the peak the lagged-angle force builds slightly
// differently from a lagged force. That difference is smaller than the
// difference between either of them and a real tyre, and only one of the two is
// incapable of inventing grip.
//
// ============================================================================
// Step size
// ============================================================================
//
// This is the stiffest term in L2 and it sets the step size, so the number is
// written here rather than left to be discovered.
//
// Explicit integration of d(x)/dt = -(1/tau) x is stable for dt < 2 tau, and
// tau = sigma / |vx| is smallest at top speed. For the provisional 1/10-scale
// numbers, sigma = 0.08 m and v_max = 20 m/s, tau is 4 ms and the bound is
// 8 ms. The default step is 1 ms (SIM-01), so there are eight steps of margin
// at the worst case the car can reach.
//
// That margin is comfortable and it is not infinite. A car parameterised for
// 40 m/s with a sigma of 0.02 m has tau = 0.5 ms and is unstable at the default
// step. relaxation_max_step below computes the bound, so a caller can check it
// against their step rather than finding out from a trajectory that oscillates.
//
// No clamping happens here. Silently limiting the rate would turn an unstable
// configuration into a plausible wrong answer, which is the failure this
// project refuses everywhere else.

#ifndef SLIPX_RELAXATION_HPP
#define SLIPX_RELAXATION_HPP

#include <cmath>
#include <limits>

#include "slipx/math.hpp"

namespace slipx {

// The rate of change of the lagged slip angle [rad/s].
//
//   alpha      the instantaneous slip angle the geometry asks for      [rad]
//   alpha_lag  the lagged slip angle the tyre currently has            [rad]
//   vx         longitudinal speed of the wheel centre, signed          [m/s]
//   sigma      relaxation length, positive                               [m]
//
// The magnitude of vx is used: the carcass relaxes over distance rolled, and a
// tyre rolling backwards relaxes just as one rolling forwards does. Reversing
// therefore lags rather than diverging, which a signed vx would not give.
//
// sigma must be positive. It is not checked here, for the same reason
// make_mf_lite does not check its arguments: this is the core, and a parameter
// set that describes no possible tyre is rejected at the boundary by
// slipx::validate, not silently repaired in the middle of a numerical path.
inline double relaxation_rate(double alpha, double alpha_lag, double vx,
                              double sigma) {
  return (std::fabs(vx) / sigma) * (alpha - alpha_lag);
}

// The lag time constant at a given speed [s]. tau = sigma / |vx|.
//
// Diverges at standstill, which is the correct answer to "how long until this
// stationary tyre builds its force" and is why the force law is written in
// terms of the rate above rather than this. Exported because it is the number a
// user comparing against a step-steer trace wants to read, and because it is
// where the sign-free speed dependence is written down once.
inline double relaxation_time_constant(double vx, double sigma) {
  const double speed = std::fabs(vx);
  if (speed <= 0.0) return std::numeric_limits<double>::infinity();
  return sigma / speed;
}

// The largest fixed step for which explicit integration of the lag is stable
// [s], at the speed given. dt < 2 sigma / |vx|.
//
// Returned rather than enforced. A caller integrating a car above this bound
// gets an oscillating slip angle and should be able to find out why without
// deriving it; a clamp here would hide it instead.
inline double relaxation_max_step(double vx, double sigma) {
  return 2.0 * relaxation_time_constant(vx, sigma);
}

// The exact solution of the lag over one step of constant slip angle and
// constant speed [rad].
//
//   alpha'(t + dt) = alpha + (alpha' - alpha) exp(-dt |vx| / sigma)
//
// NOT what the tiers integrate. They put relaxation_rate into the same state
// derivative as everything else, so that the integrator choice (CORE-13) covers
// the whole model rather than the whole model except this term, and so that a
// step is one arithmetic sequence rather than one with an exponential in the
// middle of it.
//
// It is here because it is the closed form the analytical tests check the
// integrated result against, and because it is the right answer for a caller
// stepping the transient on its own at a rate the bound above would refuse.
inline double relaxation_exact(double alpha, double alpha_lag, double vx,
                               double sigma, double dt) {
  const double decay = std::exp(-dt * std::fabs(vx) / sigma);
  return alpha + (alpha_lag - alpha) * decay;
}

}  // namespace slipx

#endif  // SLIPX_RELAXATION_HPP
