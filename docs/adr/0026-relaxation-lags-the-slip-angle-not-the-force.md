# ADR-0026: Tyre relaxation lags the slip angle, not the lateral force

- **Status:** Accepted
- **Date recorded:** 2026-08-03 (decision taken during P1)
- **Requirements:** CORE-07, CORE-03, CORE-06, NFR-02
- **Related:** [ADR-0009](0009-mf-lite-over-full-pacejka.md),
  [ADR-0022](0022-load-transfer-is-quasi-static.md),
  [ADR-0023](0023-mf-lite-derives-b-from-cornering-stiffness.md),
  [ADR-0008](0008-reference-hashes-are-keyed-by-build.md)

## Context

A tyre does not produce its lateral force the instant a slip angle appears. The
force comes from the carcass being deflected sideways in the contact patch, and
that deflection has to be built up by rolling. The distance it takes is the
relaxation length, sigma, and the standard first-order model is a lag in
distance rather than in time:

```
sigma d(x)/ds + x = x_steady
```

so the time constant is sigma / |vx| and falls as the car speeds up. That much
is not in question and is what CORE-07 asks for.

What the requirement does not settle is what `x` is. CORE-07's wording is "first
order lag on lateral force", and that is one of the two standard formulations.
The other lags the slip angle and feeds the lagged angle to an otherwise
instantaneous tyre model. Both appear in the literature and both are described
as the relaxation length model.

They differ in one respect that matters at L2 specifically, because L2 is the
first tier with load transfer (ADR-0022). The two transients now interact: the
vertical load on a wheel moves quasi-statically, in the same instant as the
acceleration that causes it, while the lateral force lags. A lagged force
therefore carries history from a load condition that no longer exists.

The concrete case, using the provisional reference numbers. A wheel carrying
about 12 N through a corner produces roughly 13 N of lateral force. Take that
wheel to the rollover threshold, where `quasi_static_loads` clamps its load to
exactly zero, and its friction budget goes to zero in the same step. A lagged
force decays towards zero over sigma / |vx|, which at 10 m/s is 8 ms, or eight
default steps. For those eight steps the model reports a tyre pushing sideways
on a road it is not touching.

That is not a corner case in the sense of being rare. `peak_lateral_force` has
an explicit zero-load guard precisely because a lifted wheel is reachable by any
car that gets near its rollover threshold, which is most of what a racing
simulation is for.

Three options were considered.

**Lag the force, as CORE-07 words it, and clamp it to the current budget.** The
clamp fixes the invented grip and introduces a discontinuity in its place: the
force jumps rather than relaxing whenever the budget crosses the lagged value.
It also makes the relaxation model conditional on a limit it is not supposed to
know about, so the transient a step-steer fit measures is not the transient the
model runs.

**Lag the force and accept the excursion.** Defensible if the excursion were
small. It is not: at zero load the entire force is invented, and the quantity
being invented is the one the friction budget exists to bound.

**Lag the slip angle.** The force is evaluated from the lagged angle and the
current load every step, so it is inside the current budget by construction, and
the only thing carrying history is the slip the tyre thinks it has.

## Decision

Relaxation lags the slip angle. `VehicleState` gains a per-wheel lagged slip
angle, `alpha_lag`, `relaxation.hpp` returns its rate, and MF-lite is evaluated
at the lagged angle with the instantaneous vertical load.

The argument is the bound. `mf_lite_fy` at the current load can never exceed
`peak_lateral_force` at that load, whatever angle it is given, so no sequence of
loads and slip angles can produce a force the tyre could not deliver. Lagging
the output of the force law gives that property up; lagging its input cannot.
This is the same reasoning as ADR-0023's, one level up: put the state where the
physics constrains it rather than where the requirement happened to name it.

Two supporting points, neither decisive on its own.

The lagged angle is the quantity an identification run can see. A step steer
measures the rise time of yaw rate, and yaw rate responds to force, but the fit
recovers sigma either way; what the fit cannot do is separate a force transient
from the load transient happening at the same time. Keeping the state on the
slip side leaves load transfer as the only thing acting on load, which is one
mechanism per observable rather than two.

The state stays bounded. A slip angle is bounded by the geometry that produces
it, so `alpha_lag` cannot run away; a lagged force is bounded only by whatever
last wrote it.

The cost is stated plainly rather than argued around: this is a lag on the input
to a nonlinear map rather than on its output, so the two formulations agree
exactly only where the map is linear, which is the small-slip region. Near the
peak the force builds along a slightly different curve than a lagged force
would. That difference is smaller than the difference between either
formulation and a real tyre, and only one of the two can invent grip.

CORE-07's parenthetical is therefore not implemented literally. The SRS is
amended to say so and to point here, as it was for the stack order in
[ADR-0003](0003-dependencies-point-downward.md).

## Consequences

**Every published trajectory hash moves, including L0's and L1's.**
`TrajectoryHash::update(VehicleState)` feeds every field in a fixed order, and
`alpha_lag` is four more fields, so the hash changes for tiers that have no tyre
transient at all. ADR-0008's consequences list a change to the state layout the
hash covers as a legitimate reason for a hash to move, so this needs no record
of its own, but it is a release event: all twelve rows of
`conformance/reference_hashes.tsv` are rerecorded in the same commit, and any
result compared against the numbers published in `0.1.0a1` is no longer
comparable with one produced after it. This is the cost of taking the state
layout change once, deliberately, rather than accumulating it.

The lag is the stiffest term in L2 and it sets the step size. Explicit
integration is stable for dt < 2 sigma / |vx|, which for the provisional
parameters at top speed is 8 ms against a 1 ms default step: eight steps of
margin, comfortable and finite. `relaxation_max_step` computes the bound so a
caller can check it, and nothing clamps the rate, because a clamp would turn an
unstable configuration into a plausible wrong answer.

`alpha_lag` is zero at L0 and L1 rather than NaN, which sits against ADR-0006's
habit. The rule there is about reported quantities, where a plausible zero gets
believed. This is hashed state, and `hash.hpp` treats a NaN in a trajectory as
evidence that the run is already broken, so a tier parking NaN in the state
would poison every hash it produced. The distinction is recorded in
`state.hpp` beside the field.

Reversing this means moving the state from the slip angle to the force, which
would move every hash again and would have to say what it does about a lifted
wheel.
