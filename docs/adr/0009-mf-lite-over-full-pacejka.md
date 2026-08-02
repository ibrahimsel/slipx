# ADR-0009: MF-lite, and every tyre parameter must be identifiable in a car park

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0)
- **Requirements:** CORE-09, SCH-05
- **Related:** [ADR-0010](0010-tyres-are-compound-surface-pairs.md),
  [ADR-0013](0013-provenance-labels-are-printed.md)

## Context

This is the thesis of the project, so it is worth stating the problem before
the decision.

Take the tyre model in a game engine. Unity's `WheelCollider` and PhysX
friction curves are parameterised by extremum and asymptote points on the
slip/force curve. Nobody can measure those. They cannot be fitted to a rosbag,
and there is no principled way to express, in those terms, the difference
between a sponge tyre and a rubber tyre on the same floor. So the numbers get
guessed, and once they are in a config file somebody starts trusting them.

Full Pacejka 5.2 fails the same test from the opposite direction. It is a
better model in every academic sense and it has dozens of coefficients whose
identification needs a tyre rig. A 1/10-scale team has wheel encoders, an IMU
and LiDAR-based pose, and a car park. Handing them a model with thirty
coefficients means they populate it from a paper about a different tyre.

An unidentifiable parameter is worse than an absent one, because it does not
stay absent. It gets guessed, and a guess in a config file is indistinguishable
from a measurement.

## Decision

The tyre model is MF-lite: a reduced Magic Formula with load sensitivity and
combined slip. Every parameter must be identifiable from a manoeuvre drivable
in a car park with the sensors already on a competition car.

| Symbol | Meaning | Identifiable from |
|---|---|---|
| `C_alpha0` | Cornering stiffness at nominal load | Skidpad, low-slip region |
| `mu_y0`, `mu_x0` | Peak lateral / longitudinal friction | Circle-to-slip, straight-line accel |
| `k_mu` | Load sensitivity exponent | Skidpad at two ballast configurations |
| `B`, `C`, `E` | MF shape factors | Full slip sweep |
| `sigma` | Relaxation length | Step steer transient |

Identifiability is the admission criterion for a new parameter. "It would
improve the model" is not sufficient; the question is what manoeuvre produces
it.

## Consequences

The model is less capable than full Pacejka and says so. Peak behaviour is
approximated, and effects that need a rig to identify are absent rather than
defaulted.

MF-lite arrives with L2 in P1. What L1 has today is a linear tyre,
`Fy = -C_alpha * alpha`, clipped at `mu * Fz`. A clip is not a Magic Formula:
there is no peak, no falling branch beyond it, and therefore no mechanism by
which the car spins. L1 slides at the limit and recovers as soon as the slip
angle comes back. `StepDiagnostics` raises `tyre_saturated` the instant the clip
engages, so the point where L1 stops being believable is a number you can plot.

The schema accepts and validates the full MF-lite block now, at schema 0.1.0,
even though only the linear block is consumed. An identified tyre file
contributed today is still correct when L2 lands, which is what makes it
reasonable to ask for contributions before the consumer exists.

P2 owes this decision a fitter. The claim that every parameter is identifiable
is only fully discharged when there is a tool that turns a rosbag2 into a
`dynamics.yaml` with residuals and confidence intervals.
