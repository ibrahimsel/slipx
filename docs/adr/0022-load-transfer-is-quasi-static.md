# ADR-0022: L2 load transfer is quasi-static and introduces no suspension parameter

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken at the start of P1)
- **Requirements:** CORE-05, CORE-02, NFR-08
- **Related:** [ADR-0009](0009-mf-lite-over-full-pacejka.md),
  [ADR-0005](0005-tiers-throw-rather-than-fall-back.md),
  [ADR-0012](0012-no-scale-branching.md)

## Context

Load transfer is the first piece of L2 and the mechanism through which CoG
height, weight distribution and track width finally reach a trajectory. Below
L2 those parameters are inert, which is the whole reason SRS 2.4 calls L2 the
tier where the product claim becomes true.

The textbook treatment of load transfer in a four-wheeled car has two parts,
and each one invites a parameter.

The first is timing. Real load transfer is not instantaneous: the body rolls
against springs and dampers, and the load arrives at the outer wheels over
roughly the roll mode period, a few tens of milliseconds at this scale. Writing
that down needs a roll degree of freedom, a roll stiffness and a roll damping
rate.

The second is distribution. The total lateral transfer is fixed by the moment
`m ay h`, but how it splits between the front and rear axles is not: on a car
with suspension it follows the roll stiffness distribution, which is what a
setup engineer changes when they move an anti-roll bar. That is one more
parameter, and it is the one that decides whether the car understeers or
oversteers at the limit, so it is not a detail.

Three options were considered.

**Model the roll degree of freedom at L2.** The most faithful, and it costs a
roll angle and roll rate in `VehicleState` plus four suspension parameters. It
is also, precisely, what SRS 2.4 lists under L3: "suspension kinematics, roll
and pitch as real DOF". Putting it at L2 does not add it to the product, it
moves L3 down and leaves L3 empty.

**Quasi-static transfer with a roll stiffness distribution parameter.** No new
states, and the front/rear split becomes tunable. This is what most simulators
in this class do.

**Quasi-static transfer with the split taken from the yaw moment balance.** No
new states and no new parameter at all.

## Decision

At L2, load transfer is an instantaneous algebraic function of the body
accelerations, and the front/rear split of the lateral transfer follows the yaw
moment balance rather than a roll stiffness distribution.

Concretely, in the ISO 8855 frame of `conventions.hpp`:

```
dFz_long      = m ax h / L
dFz_lat_axle  = m_axle ay h / t_axle,   m_front = m l_r / L,  m_rear = m l_f / L
```

The second line is not an assumption about the chassis. In steady state, yaw
moment balance gives `l_f Fy_f = l_r Fy_r` and the two lateral forces sum to
`m ay`, so `Fy_f = m ay l_r / L` and `Fy_r = m ay l_f / L`. The axle shares fall
out of a moment balance that was already going to hold. Nothing was chosen.

The reason for preferring this to a roll stiffness distribution is the
admission criterion ADR-0009 set for tyre parameters, applied to the chassis:
what manoeuvre identifies it? A roll stiffness distribution is identified by
measuring the roll moment reacted at each axle, which needs either a rig or
suspension displacement sensors. A team with wheel encoders, an IMU and a car
park has neither. It would therefore be populated by guessing, and a guess that
decides whether the car oversteers at the limit is worse than no parameter at
all, because it is indistinguishable from a measurement once it is in a file.

`h`, `L`, `t_f` and `t_r` are all measurable with a ruler and a set of bathroom
scales, which is the standard the rest of the parameter set is held to.

## Consequences

L2 has no load transfer transient. A step of lateral acceleration moves the
load in the same instant, where a real car takes tens of milliseconds. The
visible effect is that the limit arrives slightly earlier in a fast transient
than it would on the car, and that a genuine roll-induced delay between steering
and the rear axle letting go is absent. This is a stated limitation of the tier,
in the same register as L1 having no saturation shape, and it is documented in
`load_transfer.hpp` rather than left for a user to discover.

L2 cannot represent an anti-roll bar, and setup changes that act only through
roll stiffness distribution have no effect at this tier. Under
[ADR-0005](0005-tiers-throw-rather-than-fall-back.md) that is reported as what
it is rather than approximated: the parameter does not exist, so nobody can
change it and see nothing happen.

The rigid-body split is the correct limit for the vehicles SlipX targets, which
is worth stating because it makes the decision less of a compromise than it
sounds. Most 1/10-scale RoboRacer chassis have short-travel, stiff suspension
and some have none at all, and the roll stiffness distribution of a car with no
suspension is not a small parameter, it is not a parameter.

Reversing this is an L3 job and it is additive: a roll degree of freedom and its
parameters arrive with the rest of the suspension model, at which point L2
continues to mean what it means today and a user picks the tier whose
assumptions they want. It would need a new ADR superseding this one, and it must
not be done by adding a roll stiffness distribution to L2, which would take the
cost of the parameter without the benefit of the state.
