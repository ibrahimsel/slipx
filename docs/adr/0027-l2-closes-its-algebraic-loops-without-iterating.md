# ADR-0027: L2 closes its algebraic loops without iterating and without a wheel rotational state

- **Status:** Accepted
- **Date recorded:** 2026-08-03 (decision taken during P1)
- **Requirements:** CORE-02, CORE-03, CORE-04, CORE-11, CORE-12, NFR-02, SIM-01
- **Related:** [ADR-0004](0004-step-is-const-and-stateless.md),
  [ADR-0022](0022-load-transfer-is-quasi-static.md),
  [ADR-0025](0025-c-kappa-enters-the-core-ahead-of-the-schema.md),
  [ADR-0026](0026-relaxation-lags-the-slip-angle-not-the-force.md)

## Context

Assembling the double-track tier turns three independent pure functions into
one model, and doing so creates two circular dependencies that did not exist
while the pieces were separate.

**The vertical load loop.** Vertical load depends on acceleration
(`quasi_static_loads`), acceleration depends on tyre force, and tyre force
depends on vertical load. Nothing in that circle is optional: removing any leg
removes the reason load transfer is at this tier.

**The longitudinal loop.** Longitudinal force depends on slip ratio, slip ratio
depends on wheel speed, wheel speed obeys the rotational balance
`J dw/dt = T - Fx R`, and the torque in that balance is set by the force the
slip ratio produced.

The second one is not merely circular, it is stiff. A 1/10-scale wheel has a
rotational inertia of order 1e-5 kg m². Against a slip stiffness of order
100 N per unit slip, the wheel-speed mode has a time constant well under a
millisecond, which is shorter than SIM-01's 1 kHz default step. Integrating it
explicitly at that step is unconditionally unstable, and the standard symptom is
a wheel speed that oscillates and then diverges as the car slows down, where the
slip-ratio denominator also degenerates.

Options considered for the load loop:

**Iterate to convergence.** Rejected outright. A convergence tolerance makes the
number of arithmetic operations depend on the data, and NFR-02 promises a
bit-identical trajectory. It is exactly the place a nondeterminism would hide.

**One pass, using static loads.** Cheap, and it makes CoG height inert in the
force calculation, which removes the point of the tier.

**A fixed number of passes.** Chosen.

Options considered for the longitudinal loop:

**Full wheel dynamics with a wheel-inertia parameter.** Physically the right
structure, and the only way to represent lockup and wheelspin as dynamic
events. It needs either a sub-stepped integrator for the wheel states alone or a
documented maximum step that contradicts SIM-01's default, and it adds a
parameter and a standstill singularity. Deferred rather than dismissed: CORE-11
wants it.

**Quasi-static slip ratio.** Chosen.

## Decision

**The load loop is closed by exactly two passes.** Tyre forces are evaluated at
the static loads; the accelerations that result are fed to
`quasi_static_loads`; the forces are evaluated once more at those loads and
those are the ones the model uses. Two, always, whatever the residual. The count
is part of the model and therefore part of the trajectory, and changing it
changes every L2 reference hash.

**The longitudinal loop is closed by removing the state.** There is no wheel
rotational degree of freedom. The longitudinal force demand is split between the
four wheels and delivered up to each tyre's friction budget; the slip ratio
reported is the one that would produce the force actually delivered, through the
linear slip stiffness at that wheel's load; and the wheel speeds follow from
that slip ratio by its definition, `omega R = v (1 + kappa)`. Wheel speed is a
consistent report, not an independent state.

This is why `c_kappa` matters at all at this tier (ADR-0025). It does not enter
the force law. It sets the reported slip ratio and hence `VehicleState::omega_w`,
which the trajectory hash covers, so it does affect the published numbers, and it
is what an encoder-based validation compares against.

**The drive split is equal between the four wheels, not proportional to load.**
This was going to be load-proportional, on the argument that it is the split
that brings every tyre to its limit together. Measurement changed it. In a
corner the outer wheels carry more load, so a load-proportional split puts more
thrust outboard, and that asymmetry is a yaw moment turning the car into the
corner: worth about 2% of steady-state path radius at 0.36 g on the reference
car, which is larger than several effects this tier exists to represent. No
differential does that. An equal split produces no drive-induced yaw moment on a
symmetric car, which is the correct behaviour for a tier that does not model a
differential.

## Consequences

**L2 cannot represent a locked or a spinning wheel.** A wheel whose demand
exceeds its budget delivers what it can and reports the slip ratio that
corresponds; it never runs away to a slip ratio of one. ABS, traction control
and any study whose subject is wheel-speed dynamics are out of reach at this
tier, and a user reading `omega_w` should know it is derived rather than
integrated. This is the largest single limitation of the tier and it is stated
in `l2_double_track.cpp`'s header as well as here.

**Two passes is an approximation and is not marked as one anywhere in the
output.** The loads used are those implied by the accelerations that the static
loads produced, so they lag the converged answer by one pass. The residual is
second order in the load transfer and is small for a 1/10-scale car; it is not
zero, and a car with a much higher CoG would show it. There is no diagnostic
reporting the residual, which is a gap worth filling if anybody ever asks how
converged the loads are.

**The equal split is not a differential and will be replaced.** CORE-11 brings
spool, open and preloaded LSD, and 2WD against 4WD. When it lands, the split
stops being a constant and every L2 reference hash moves. That is anticipated
rather than accidental.

**Changing the pass count is a release event.** It is not a tuning knob or an
accuracy setting. Two is in the trajectory.

Reversing the wheel-state half means adding a rotational state per wheel, which
would move every L2 hash, add a wheel-inertia parameter, and require an answer
to the step-size problem in the Context above rather than a restatement of it.
