# ADR-0042: Rollover is a sim-level event that freezes the agent

- **Status:** Proposed
- **Date recorded:** 2026-08-19
- **Requirements:** CORE-14 in spirit; `docs/spec` is not present in this
  checkout, so no ID is cited as authority.
- **Related:** [ADR-0004](0004-step-is-const-and-stateless.md),
  [ADR-0006](0006-diagnostics-report-nan-not-zero.md),
  [ADR-0022](0022-load-transfer-is-quasi-static.md),
  [ADR-0027](0027-l2-closes-its-algebraic-loops-without-iterating.md)

## Context

P3 makes races, and a race needs cars to be able to stop being in it. The
first way a car leaves a race that the physics can already see coming is
rollover: the quasi-static load model has clamped lifting wheels to zero and
reported `wheel_lifted` since L2 landed, with a comment promising that the
event and the halt would arrive in P3. This record is that promise coming
due, and it has to settle three things: what the detection signal is, which
layer owns the event, and what a car that has rolled becomes.

The alternatives actually on the table:

- **Simulate the roll.** A roll degree of freedom, spring and damper rates,
  ground contact for a tumbling body. Rejected for the same reason ADR-0022
  rejected suspension at L2: none of those parameters is identifiable in a
  car park, and the post-roll trajectory has no racing value beyond "the car
  stopped here". A rolled car is a result, not a dynamical system worth
  integrating.
- **Detect in the core.** A rollover flag inside `step`, or a model that
  refuses to advance past the condition. Rejected because an event is
  run-level bookkeeping: `step` is `const` and stateless (ADR-0004) and has
  no notion of an agent's lifecycle, only of one interval of dynamics. The
  core's whole contribution is already in place: the clamped per-wheel
  loads and the `wheel_lifted` flag are in the diagnostics, and they are
  exact, because the clamp writes a literal zero.
- **Detect on `wheel_lifted` or on the threshold formula.** A single wheel
  at zero is three-wheeling, which is routine near the limit at whichever
  axle lifts first and is not a rollover. The closed-form threshold
  `g t / 2h` describes only the pure-lateral steady state; it knows nothing
  of braking mid-corner, combined transfer or load sensitivity. The
  per-wheel loads capture all of that automatically, because they are the
  model's own output rather than a formula beside it.

## Decision

**Rollover is detected by `slipx_sim` after each step, from the step's own
diagnostics: both wheels of one side at zero vertical load. The agent is
then DNF: its policy is never called again, its model never steps again, its
pose freezes at the event, and its velocity-like states are zeroed.**

The pieces, and why each is the way it is:

1. **The signal is both wheels of one side at zero load.** A car supported
   entirely on one side's contact patches is the static rollover condition
   proper, and it is read from `StepDiagnostics::fz`, which the L2 model
   fills from the clamped load pass evaluated at the end-of-step state. The
   comparison is exact because the clamp is exact. Below L2 those entries
   are NaN (ADR-0006), every comparison is false, and tiers without load
   transfer cannot roll: that is a stated limitation of those tiers, not a
   gap in the detection.
2. **First occurrence fires; there is no persistence window.** A debounce
   length would be a parameter, nobody can identify it, and the quasi-static
   model stores no roll energy that a window could meaningfully integrate.
   The cost is stated under consequences.
3. **DNF freezes the car as an obstacle.** Position, yaw and every
   non-motion state keep their value from the event step; `vel_body`,
   `rates`, `omega_w` and `steer_rate` are set to zero. The zeroing is not
   cosmetic: a recording in which position is constant while velocity claims
   motion is internally inconsistent, and the contact model this phase adds
   next computes impulses from relative velocity, so a stationary obstacle
   must read as stationary. The kinetic energy the car had simply leaves the
   record; that is the roll and slide this model does not simulate, and the
   manifest says how the run ended rather than leaving the frozen state to
   imply it.
4. **The event carries its cause**: which side unloaded, at which step, at
   what time. It round-trips through snapshot and restore, `reset()` clears
   it, and the manifest reports per-agent status in the result section,
   excluded from the configuration digest, because how a run ended is an
   answer and not part of the question.
5. **A frozen agent keeps its slot.** Its constant state is still folded
   into the trajectory hash and a neutral input is still logged for it, so
   the log stays rectangular, replay applies the same detection and
   reproduces the same event, and no downstream consumer needs a variable
   agent count mid-run.

## Consequences

- Detection is quasi-static, so the event fires the instant the loads say
  zero. A real car's load transfer lags by roughly the roll mode period and
  its roll inertia absorbs short spikes, so L2 DNFs a car that a sharp
  transient only threatened to roll. The error is in the conservative
  direction for racing, and fixing it properly is L3's suspension work, not
  a tuning constant here.
- A DNF is terminal by construction: nothing un-freezes an agent short of
  `reset()`. The timeout policies task will add other routes to a halt and
  must not add a route back.
- The core is untouched and no reference hash moves; the conformance suite
  asserts this rather than assuming it. Detection reads diagnostics that
  were already computed, so the cost per agent per step is eight
  comparisons.
- Anything that replays a run below `slipx_sim` (calling `VehicleModel::step`
  directly) does not get the event and will integrate a car past its own
  rollover. That is the layering working as intended: the core describes
  dynamics, the orchestrator owns the race.
