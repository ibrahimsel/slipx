# ADR-0031: Drivetrain and actuators close in closed form; only the servo and the state of charge add state

- **Status:** Accepted
- **Date recorded:** 2026-08-12
- **Requirements:** CORE-08, CORE-09, CORE-10, CORE-11, CORE-03, CORE-04, NFR-02
- **Related:** [ADR-0006](0006-diagnostics-report-nan-not-zero.md),
  [ADR-0022](0022-load-transfer-is-quasi-static.md),
  [ADR-0025](0025-c-kappa-enters-the-core-ahead-of-the-schema.md),
  [ADR-0027](0027-l2-closes-its-algebraic-loops-without-iterating.md),
  [ADR-0030](0030-schema-0-2-0-adds-c-kappa-and-the-actuator-fields.md)

## Context

Minimal L2 shipped with four deliberate gaps, each stated in its header: no
ESC curve, no battery, no servo, and an equal drive split standing in for a
differential. Filling them collides with two standing constraints.

First, ADR-0027: L2 has no wheel rotational state and closes its algebraic
loops in a fixed number of passes. A differential is naturally described in
wheel speeds, which do not exist as state here, and an LSD transfers torque on
a speed difference. Second, the ESC couples torque to current to voltage back
to torque: delivered torque draws current, current sags the pack through its
internal resistance, and the sagged voltage rescales the torque curve. Both
are algebraic loops of exactly the kind ADR-0027 refused to iterate on.

Alternatives considered:

**Add wheel rotational states after all.** The physically complete answer and
the only one that represents lockup and wheelspin as events. Rejected for the
same reason ADR-0027 rejected it: the wheel mode's time constant is well under
the 1 kHz step, so it needs sub-stepping or a stiff solver, and both put the
iteration structure into the trajectory.

**Iterate the electrical loop to convergence.** Rejected outright, as always:
a convergence tolerance makes the operation count data-dependent, and NFR-02
promises bit-identical trajectories.

**Supersede ADR-0027.** Not needed. Every mechanism in this slice has a closed
form inside the existing two-pass structure, and the one genuinely dynamic
component, the servo, is not stiff and belongs in the integrator vector as
ordinary state.

## Decision

### Differentials are torque-split rules, evaluated in closed form inside each force pass

The drive layout selects the driven wheels: `2WD_front` and `2WD_rear` send
the whole torque demand to that axle; `4WD` splits it 50/50 between the axles
with a locked centre. The 50/50 is stated deliberately: a typical 1/10-scale
4WD is a belt or shaft with no centre differential, and the schema
deliberately has no centre-diff field, so the locked centre is the honest
model rather than a simplification. Non-driven wheels receive no longitudinal
demand.

Within a driven axle, the split is the differential's:

- **Open**: equal torque to both wheels, each capped by its friction budget.
  The friction ellipse is evaluated for each wheel at half the axle demand;
  if one wheel delivers less than its half, the other is held to the same
  delivered force, because an open differential cannot support a torque
  difference across the axle. This reproduces the no-yaw-moment behaviour the
  measured equal split of ADR-0027 demanded, and it reproduces the open
  diff's characteristic failure: one lifted inside wheel and the whole axle
  delivers nothing, which is the teaching artefact.
- **Spool**: both wheels share one contact-patch speed. With the per-wheel
  linear slip stiffness `c_i` at that wheel's load (the same load-adjusted
  stiffness the slip-ratio report uses) and the wheel-centre speed `v_i`
  floored at `v_eps`, the wheel force at a common speed `omega R` is
  `F_i = a_i (omega R) - c_i` with `a_i = c_i / v_i`, and force balance
  against the axle demand gives the closed form
  `omega R = (F_axle + c_l + c_r) / (a_l + a_r)`. The per-wheel forces follow
  and the friction ellipse then caps each. The inner wheel of a corner is
  slower, so it carries the larger slip ratio and drives harder; the thrust
  bias is inboard, the yaw moment opposes the turn, and the spool's turn-in
  scrub and understeer push emerge with no additional mechanism.
- **Preloaded LSD**: compute the spool solution first. If the torque
  difference it implies across the axle is within the preload, the axle is
  effectively locked and the spool solution stands. Otherwise the clutch
  slips, and the split falls back to the open one biased by the preload
  toward the slower wheel: `T/2 + preload/2` to the slow side, `T/2 -
  preload/2` to the fast side, each wheel then capped by its own ellipse.

**Braking goes through the driven axle only.** A 1/10-scale car brakes
through its motor; there are no friction brakes, which is why the schema has
no brake bias field (ADR-0030 notes it in `decel_max`'s description). A
negative demand takes the same differential path as a positive one.
`decel_max` remains a command bound; the regen current limit below is usually
the operative one and is weaker. This changes behaviour: minimal L2 split
braking equally over four wheels, which no motor-braked 2WD car does.

**The `VehicleParams` default differential is open, on the default
`2WD_rear` layout.** The open diff is the one that preserves the measured
insight ADR-0027 recorded: no drive-induced yaw moment on a symmetric car.
The cross-tier low-lateral-acceleration agreement test runs on defaults and
must keep holding. The reference car's file stays `spool`, because that file
describes a real class of car and most 1/10 competition cars run a locked
rear axle; the file and the struct default answer different questions.

### The ESC and battery are evaluated once per step, at the entry state

Quasi-static over one 1 kHz step, exactly as load transfer is quasi-static
within one (ADR-0022): the electrical time constants of an ESC current loop
are far below a millisecond, and pack voltage and state of charge move on the
scale of seconds. The torque budget is therefore a per-step constant, like
the clipped command today, and the derivative stays a pure function.

The commanded acceleration is first clamped to `accel_max`/`decel_max` and
the `v_max` gate, as at every tier; the result becomes a wheel torque demand
`T = m a R`. The ESC then bounds it:

- **Curve**: `T_avail(omega) = torque_stall * s * (1 - omega / (omega_free *
  s))` with `s = pack_v / pack_nominal_v`, evaluated at the mean driven-wheel
  speed of the entry state, floored at zero. Both curve numbers scaling with
  `s` is DC motor behaviour and is what ADR-0030 promised the schema fields
  would mean.
- **Current limit**: positive torque is further capped at `torque_per_amp *
  current_max`.
- **Regen limit**: negative (braking or reverse) torque is capped in
  magnitude at `torque_per_amp * regen_current_max`. This is the only brake
  the model has.

**Battery**: open-circuit voltage is linear in state of charge between
`pack_v_empty` and `pack_v_full`. Sag closes in a fixed two passes, mirroring
ADR-0027's structure. Pass one takes `pack_v = ocv`, computes the delivered
torque and the electrical power `P` (mechanical wheel power over `efficiency`
when discharging; times `efficiency` when regenerating, so the loss is a loss
in both directions). Pass two solves the terminal-voltage identity
`V^2 - ocv V + R P = 0` in closed form, takes the upper root, and re-evaluates
the torque budget at that voltage. If the discriminant is negative the demand
exceeds the pack's maximum deliverable power and `V` is clamped to `ocv / 2`,
the peak-power point, which is the closed-form worst case. The delivered
torque from pass two is the budget the differential splits.

State of charge integrates `d(soc)/dt = -I / (3600 * pack_capacity_ah)` with
`I = P / V` from the pass-two values, negative under regen so the pack
charges. `soc` is integrator state, clamped to [0, 1] after the step;
`pack_v` is written to the state algebraically each step, exactly as the
per-wheel `Fz` are: a reported consequence, not a degree of freedom.

The degenerate configuration `pack_v_full = pack_v_empty = pack_nominal_v`
with zero internal resistance must reproduce the bare curve exactly, not
within a tolerance; `validate()` therefore bounds the pack voltages
non-strictly (`v_empty <= v_nominal <= v_full`), because that configuration
is both the no-battery test fixture and a legitimate "ideal supply" model.

### The servo is state

`steer` and `steer_rate` join the integrator vector at L2:

    d(steer)/dt      = clamp(steer_rate, -max_rate, +max_rate)
    d(steer_rate)/dt = wn^2 (delta_cmd - steer) - 2 zeta wn steer_rate

with `delta_cmd` the travel-clipped command, `wn` the bandwidth and `zeta`
the damping ratio from `limits.yaml`'s steering block. After integration
`steer` is clamped to `steer_max` and `steer_rate` is zeroed at the stop, so
the mechanical end stop is inelastic. The tyre model consumes the achieved
`steer`, so commanded versus achieved is now visible exactly as CORE-10
wanted: `DriveInput::steer_cmd` against `VehicleState::steer`.

An underdamped servo overshoots; that is physics, not a bug. The invariant
the tests hold is therefore not `achieved <= commanded` but the second-order
bound: the achieved angle never exceeds the command by more than the
overshoot fraction `exp(-pi zeta / sqrt(1 - zeta^2))`, about 4.6% at the
provisional damping of 0.7, and the achieved angle does not move before the
step that commands it.

### The state vector grows from 10 to 13; the hashes move once

`steer_rate` and `soc` are velocity-like (their derivatives are computed
rates), `steer` is position-like (its derivative is the clamped
`steer_rate`), following `integrator.hpp`'s grouping so semi-implicit Euler
advances the pair in the right order. `VehicleState` does not change shape:
the fields have existed since P0 and lower tiers leave them at their
defaults. L0 and L1 hashes therefore must not move, and the conformance check
asserts that; every L2 row moves, once, at the end of the slice, under the
standing hash discipline.

## Consequences

**A capped spool wheel breaks the shared-speed report.** Reported slip ratios
and wheel speeds follow the delivered force (ADR-0027's chain), so when the
friction ellipse caps one wheel of a spool axle the two reported wheel speeds
are no longer exactly equal. Force is the trajectory-authoritative quantity
and the discrepancy is the honest signature of the cap; representing the true
locked-axle dynamics through the cap would need the wheel state ADR-0027
declined.

**The torque budget is frozen for the step.** Within one step the RK4 stages
see one budget, evaluated at the entry state. At 1 kHz the error is far below
the model's own fidelity; the alternative re-evaluates the electrical loop
four times per step and moves it into the derivative, where its two-pass
structure would nest inside the load-transfer two-pass structure for no
measurable gain.

**The budget reads the entry state's driven-wheel speeds.** A state
constructed by hand at speed but with zero `omega_w` gets one step of
stall-region torque before the report catches up. The conformance run does
exactly this and is deterministic either way; it is noted here so nobody
mistakes the first-step budget for a bug.

**Braking is weak, honestly.** On the provisional numbers the regen cap of
0.4 N m is about 0.23 g, against the 12 m/s^2 `decel_max` command bound that
L0 and L1 deliver. An L2 car needs far more room to slow down, which is true
of the real cars and is now represented instead of idealised.

**There is no low-voltage cutoff.** `soc` clamps at zero and the curve keeps
evaluating at `pack_v_empty` under sag. A real ESC cuts drive before that.
The cutoff voltage is ESC firmware configuration, not something identifiable
from driving, so it is absent rather than guessed (ADR-0009's bar); a run
that reaches `soc = 0` has left the model's honest envelope and the plotted
`soc` says so.

**Reversal costs.** Undoing the quasi-static ESC means putting the electrical
loop into the derivative and re-answering the nesting question above. Undoing
the differential forms means wheel rotational state, which is ADR-0027's
reversal note verbatim. Both move every L2 hash again.
