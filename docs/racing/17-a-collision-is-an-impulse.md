# A collision is an impulse

Two cars touch for a few milliseconds and everything afterwards is
different. What happened in between is a force history nobody will ever
measure: two plastic shells flexing against each other with a stiffness
that would demand a microsecond timestep to integrate honestly. The way
out, standard across mechanics since Newton, is to stop asking for the
force and ask only for its integral. That integral is the **impulse**,

```
J = ∫ F dt        [N s]
```

and the claim of this article is that for a racing collision the impulse
is not a simplification of what matters. It is what matters. Every
quantity you care about after the contact, who slowed, who got turned,
who kept their lap, follows from one vector applied at one point.

## Why the integral is enough

The argument is a separation of timescales, and it should be stated
rather than assumed. A bumper contact lasts on the order of milliseconds.
Over that window the contact force peaks in the hundreds of newtons for a
3.5 kg car, while the tyres, the motor and drag are contributing a few
tens of newtons at most. So during the contact, to a good approximation,
only the contact force acts; and the car barely moves, since a few
milliseconds at a few metres per second is a few millimetres. The
collision therefore changes the car's **velocities** while leaving its
**position and heading** where they were, and the change is exactly the
impulse divided by the inertia:

```
Δv = J / m                    [m/s]
Δ(yaw rate) = (r × J) / Izz   [rad/s]
```

where `r` is the lever arm from the centre of gravity to the contact
point. The assumptions have names: the bodies are rigid, the contact is
instantaneous, and one point carries the whole exchange. Each is false in
detail and each failure is discussed at the end.

Because the two cars press on each other with equal and opposite force at
every instant, their impulses are equal and opposite too, and momentum is
conserved no matter what the force history looked like. That is the
attraction of the formulation: it promises nothing about the collision's
internals and therefore cannot be wrong about them.

![The anatomy of a contact](assets/contact-impulse.svg)

## The head-on case, worked

Two identical 3.5 kg cars meet head on, each doing 2 m/s, so the closing
speed is 4 m/s. For two bodies the algebra collapses onto the **reduced
mass** `m* = (m₁ m₂)/(m₁ + m₂)`, here 1.75 kg. The normal impulse is

```
Jn = (1 + e) · m* · (closing speed) = (1 + e) · 1.75 · 4
```

where `e` is the coefficient of restitution, of which more below. Take
e = 0.3: `Jn = 9.1 N s`. Each car's speed changes by 9.1 / 3.5 = 2.6 m/s,
so each goes from +2.0 m/s to −0.6 m/s, and they separate at 1.2 m/s,
which is 0.3 times the 4 m/s they closed at. That last sentence is not a
coincidence; it is the definition.

## Restitution: one number for everything you cannot know

Newton's kinematic definition: `e` is the ratio of separation speed to
closing speed along the contact normal. `e = 1` is a perfectly elastic
bounce; `e = 0` means the bodies stop closing and stay in touch. Energy
goes as the square: the fraction `1 − e²` of the closing-direction
kinetic energy is destroyed. In the head-on case above that is 91 per
cent of 14 J, nearly 13 J, gone into shell flex, mount rattle, sound and
scraped plastic. The impulse model does not know where it went; `e`
summarises the whole loss in one number, which is both its power and its
dishonesty.

What is `e` for a 1/10-scale car? Nobody knows, and this series does not
pretend otherwise. Full-size crash data puts vehicle-on-vehicle
restitution around 0.1 to 0.3 at moderate speeds, rising as impacts get
gentler; foam bumpers on plastic shells plausibly sit in the same range.
Measuring it would mean instrumented crash tests, which is not a car-park
manoeuvre, so any simulator's value is a plausible constant and should be
labelled as one. Treat collision outcomes accordingly: the *kind* of
outcome (who got turned, roughly how hard) is trustworthy, the exact
post-impact numbers are not.

## Friction at the contact

The impulse need not point along the contact normal. If the surfaces are
sliding across each other during the impact, friction acts along the
tangent, and it obeys the same Coulomb bound as a tyre does:

```
|Jt| ≤ μ · Jn
```

The tangential impulse tries to cancel the relative sliding at the
contact point and takes the bound when it cannot. Geometrically, the
total impulse must lie inside a **friction cone** of half-angle
`atan(μ)` about the normal, which is the cone drawn in the figure. For a
glancing blow the tangential part is what scrubs speed off the faster
car, and it is also a second contributor to the yaw moment, since it too
acts at the end of the lever arm.

## The yaw moment, or why a tap is a spin

Here is the number that makes contact matter at this scale. A gentle nudge:
closing speed 1 m/s along the normal, e = 0.3, so `Jn ≈ 2.3 N s`
(taking the reduced 1.75 kg; the lever arms reduce the effective mass a
little, so the true number is slightly smaller). The struck car's speed
changes by a modest 0.65 m/s. Now let that impulse land 0.12 m from the
centre of gravity, which on a 0.55 m car is not even the bumper corner:

```
Δ(yaw rate) = (0.12 × 2.3) / 0.05 ≈ 5.5 rad/s
```

That is over 300 degrees per second of yaw rate, acquired in a few
milliseconds. For comparison, the same car cornering at its friction
limit at 3 m/s sustains a yaw rate of about 3.3 rad/s. One tap, off
centre, delivers more rotation than the hardest corner the tyres can
hold, and the tyres now have to catch it.

The general statement uses the **radius of gyration** `k = √(Izz/m)`,
about 0.12 m for this car: a hit whose lever arm exceeds `k` turns the
car harder than it shoves it, in matched units, by the factor `r / k`.
Nearly all of a 0.55 m by 0.30 m body lies further than 0.12 m from the
centre of gravity. Full-size racing knows this arithmetic as the PIT
manoeuvre: a light touch, placed far behind an opponent's centre of
gravity, spins a car that a central shove of the same strength would
barely disturb.

## What the single impulse leaves out

Said plainly, because each item is a real gap:

- **Deformation.** The shells flex and the model's bodies do not; `e`
  carries the entire material story. There is no damage, and a tenth
  collision looks exactly like the first.
- **One contact point.** Real bumper contact is compliant and spread over
  a patch. A single point at the centre of the overlapping region means
  two cars pressed perfectly flush exchange a central push and no
  twisting couple at all.
- **Resting contact.** An impulse model resolves events, not sustained
  states. Cars leaning on each other through a long corner are a sequence
  of tiny impulses and small geometric corrections, which is plausible
  but is not a contact patch model, and stacking bodies on top of one
  another is entirely outside it.
- **Discrete detection.** A simulator checks for overlap once per step.
  At racing speeds and kilohertz steps two cars move millimetres per
  step, so passing through each other unnoticed needs closing speeds no
  1/10 car reaches; the arithmetic should be rechecked before anyone
  reuses the model with longer steps or faster bodies.

> **In SlipX.** Contact is exactly the model of this article: one impulse
> with restitution and Coulomb friction per touching pair per step,
> computed by a pure function in the core and applied by the orchestrator
> between steps (ADR-0043). Collision geometry is an oriented rectangle
> per agent, declared from the car file's own length and width; an agent
> that declares none touches nothing, which is what keeps every
> pre-contact trajectory bit-identical. The restitution and friction
> constants are labelled plausible, and every SlipX document that touches
> them repeats that they are fitted to nothing.

## In one paragraph

A collision is too fast to resolve and too violent to ignore, so
mechanics replaces the force history with its integral: one impulse,
applied at one point, equal and opposite on the two bodies, which is why
momentum survives whatever else happens. Restitution compresses all the
unknowable material behaviour into one plausible number, friction bounds
the impulse inside a cone, and the lever arm from the centre of gravity
converts whatever lands off-centre into yaw rate at a rate set by the
radius of gyration. At 1/10 scale that conversion is brutal: a tap that
costs half a metre per second of speed can hand the tyres more yaw rate
than a corner at the limit ever would.

## Further reading

- Stronge, *Impact Mechanics*, 2nd ed., Cambridge University Press,
  2018, for the rigid-body impact theory done properly, including where
  the Newtonian restitution used here misbehaves (frictional, eccentric
  impacts).
- Brach, *Mechanical Impact Dynamics: Rigid Body Collisions*, Wiley,
  1991, for planar vehicle collision analysis and measured restitution
  values.
- Baraff, *An Introduction to Physically Based Modeling: Rigid Body
  Simulation*, SIGGRAPH course notes, 1997, for how simulators turn this
  theory into code, impulses, contact points and all.
