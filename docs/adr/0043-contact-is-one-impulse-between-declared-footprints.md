# ADR-0043: Contact is one planar impulse between declared footprints

- **Status:** Proposed
- **Date recorded:** 2026-08-19
- **Requirements:** CORE-15 and SIM-04 in spirit; `docs/spec` is not present
  in this checkout, so no ID is cited as authority.
- **Related:** [ADR-0002](0002-no-eigen-in-the-core.md),
  [ADR-0004](0004-step-is-const-and-stateless.md),
  [ADR-0027](0027-l2-closes-its-algebraic-loops-without-iterating.md),
  [ADR-0042](0042-rollover-is-a-sim-event-that-freezes-the-agent.md)

## Context

Head-to-head racing needs cars that cannot occupy the same piece of track.
Everything below P3 was built for cars that never touch, and the ghost-race
demo says so on its face. Adding contact forces four decisions: what the
model is, where it runs, what geometry it runs on, and what happens to every
existing run's bit-identity when it lands.

The model question has a crowded space of wrong answers for this project. A
penalty-force model (a spring at the overlap) puts a stiff term into the
integrator and couples the step size to the contact stiffness; an iterative
constraint solver (the game-engine standard) converges to a tolerance, and
iteration counts that depend on floating-point comparisons are exactly the
nondeterminism ADR-0027 spent its budget avoiding; and a fitted contact
model would need data nobody has, since crash-testing 1/10 cars to identify
restitution curves is not a car-park manoeuvre. Meanwhile the honest goal is
modest: races need contact that is plausible, deterministic and attributable
(who hit whom, from which side, how hard), not contact that is validated.

## Decision

**Contact is a single planar impulse with restitution and Coulomb friction,
computed by a pure function in `slipx_core` (`slipx/contact.hpp`) and
applied by `slipx_sim` between steps, once per touching pair per step, in
fixed pair order. It acts between agents that declare a rectangular
footprint; an agent that declares none touches nothing.**

The pieces:

1. **The core owns the mathematics, the sim owns the collision.** The
   impulse is rigid-body mechanics with a closed form, so it belongs beside
   `load_transfer.hpp`: header-only, standard library only, no allocation,
   testable against momentum conservation without an orchestrator in the
   room. The decision of when two agents touch and what to do about it is
   run-level bookkeeping, which is the orchestrator's job, exactly as it is
   for rollover (ADR-0042). `VehicleModel::step` is untouched.
2. **Footprints are declared, not defaulted.** `AgentSpec` gains a
   `footprint_length` and `footprint_width`; both zero, the default, means
   the agent has no collision geometry and passes through everything.
   Contact therefore cannot change any existing run: single-agent
   conformance rows, the ghost race, every recorded manifest. That is
   asserted, not hoped, by a test that runs a footprinted single agent and a
   footprint-free pair against their old hashes. The car schema has carried
   `geometry.length` and `geometry.width` since 0.1.0, so the numbers exist
   in every car file; the rectangle is centred on the wheelbase midpoint,
   which is derivable from `lf` and `lr` and adds no parameter.
3. **One impulse, one pass, fixed order.** Pairs are visited in ascending
   index order and each touching pair gets exactly one impulse per step: no
   convergence loop, no re-visiting, for ADR-0027's reason (the pass count
   is part of the trajectory). At a 1 kHz step the interpenetration this
   leaves behind is millimetres, and a full positional projection along the
   contact normal, split by inverse mass, removes it in the same step.
4. **Restitution and friction are simulation configuration, labelled
   plausible.** They are properties of a collision between two foam-and-
   plastic bodies, not of one car, and nothing in a car park identifies
   them; defaults live in `ContactParams` with the label in the doc
   comment, the reference docs and the tutorial, all of which keep saying
   "plausible and deterministic, not fitted". Below a small closing speed
   the restitution is treated as zero, so cars rubbing side by side push
   apart instead of chattering; the threshold is an anti-jitter device and
   is documented as one.
5. **A DNF'd car is an immovable obstacle.** Its inverse mass and inverse
   yaw inertia enter the impulse as zero, so a moving car bounces off it
   and the frozen state stays frozen, which is what ADR-0042 promised
   contact would find.

## Consequences

- The model is deliberately not validated and cannot be: no data, no claim.
  Restitution and friction move the outcome of any given collision by more
  than either coefficient is known to, and every document that touches
  contact says so. What IS promised is determinism, momentum conservation,
  the friction cone and mirror symmetry, all held by the invariant suite.
- One contact point per pair per step (the midpoint of the clipped incident
  edge) makes face-on-face contact a single central push: two cars pressed
  flush cannot exchange a pure couple. Real bumper contact is compliant and
  multi-point; this is a stated simplification, in the same register as
  quasi-static load transfer.
- Sequential application means an impulse from pair (0,1) is visible to
  pair (0,2) in the same step. The order is fixed and recorded (agent
  index), so the result is deterministic but not permutation-invariant:
  renumbering the cars is a different race, which the trajectory-hash
  design already accepts.
- Detection is discrete-step: a pair that passes completely through each
  other between two steps would never touch. At 1 kHz two cars closing at
  40 m/s move 4 cm per step against a half-metre footprint, so the class
  cannot reach the failure; a future step-size change must re-check this
  arithmetic.
- There is no car-to-wall contact here. Track limits remain a report
  (M5.3), and what a wall does to a car is a different decision for the
  slice that needs it.
- The two-body impulse leaves wheel speeds and tyre relaxation states
  untouched; the next vehicle step recomputes them from the new velocities.
  One step of stale `omega_w` on a car that has just been hit is inside the
  model's honesty budget, and fixing it would mean the sim reaching into
  tier-specific state.
