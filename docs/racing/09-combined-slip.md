# 9. Combined slip: braking and turning at once

[Article 1](01-tyres-and-grip.md) introduced the friction ellipse and left it
as a boundary: everything the tyre can do lies inside a closed curve. A
boundary tells you what is impossible. It does not tell you what you get when
you ask for something that is.

That gap matters more than it sounds. Almost every interesting moment in a lap
is a combined one. Turning in while still braking, feeding in power before the
corner is finished, catching a slide with the brakes: in all of them the tyre
is being asked for longitudinal and lateral force at the same time, and what it
does with a request it cannot fill is the difference between a model that
predicts a spin and a model that predicts a car doing what it was told.

This article turns the boundary into a rule, does the arithmetic, and then
spends the second half on what the rule cannot do. The friction ellipse is
probably the most over-trusted object in vehicle dynamics, and the useful skill
is knowing which questions it is entitled to answer.

**Before you start.** Article 1, particularly the sections on the tyre curve
and the ellipse. [Article 2](02-load-transfer.md) is useful for the last part,
where the ellipse turns out to change size while you are using it.

## Two slips, one contact patch

A tyre makes lateral force by running at a **slip angle**: the wheel points one
way and travels another, the carcass deflects, and the force is the elastic
reaction. Article 1 has the picture.

It makes longitudinal force the same way, from a **slip ratio**. The tread
surface of a driven wheel moves slightly faster than the road under it, and of
a braked wheel slightly slower. Written with the wheel's angular speed `omega`,
its rolling radius `r` and the road speed `vx`:

```
kappa = (omega r - vx) / |vx|
```

A worked one, at 1/10 scale. Take a 50 mm wheel at 5 m/s, so a free-rolling
wheel turns at 100 rad/s. Drive it at 110 rad/s instead and its surface moves
at 5.5 m/s, giving `kappa = 0.10`: the tread is being dragged forward through
the contact patch by 10%, and the reaction to that is thrust. Brake it to
90 rad/s and `kappa = -0.10`, with the force reversed.

The two mechanisms are the same mechanism at ninety degrees, and here is the
part that matters: **they share one contact patch.** There is one area of rubber
on the road, one normal load pressing it down, and one friction budget. Spending
it forwards leaves less for sideways.

## The naive rule, and how it invents grip

The obvious way to build a combined-slip model is to keep the two axes
separate. Compute the lateral force from the slip angle using the lateral
curve, compute the longitudinal force from the slip ratio using the
longitudinal curve, and clip each one to its own limit. Two independent
one-dimensional problems, no coupling, easy to code.

It is wrong in a specific and dangerous way. Take one tyre of a 3.5 kg car at
rest, carrying a quarter of its weight:

```
Fz     = 3.5 * 9.81 / 4 = 8.58 N
mu_y   = 1.10           ->  lateral limit      9.44 N
mu_x   = 1.05           ->  longitudinal limit 9.01 N
```

Now ask it for 8.0 N of cornering force and 7.0 N of braking force at the same
time. Check each against its own limit and both pass, with room to spare: 8.0
is less than 9.44, and 7.0 is less than 9.01. The per-axis rule hands back
exactly what was asked for.

The magnitude of what it handed back is `sqrt(8.0^2 + 7.0^2) = 10.6 N`, from a
contact patch that cannot make more than about 9.4 N in any direction. Measured
against the boundary in the direction the demand points, the pair is 15%
outside it. The model has invented a seventh of the grip it just reported, and
it did so without a single value coming close to a limit.

At vehicle level that request is a car cornering at 9.1 m/s<sup>2</sup> while
braking at 8.0 m/s<sup>2</sup>: about 0.93 g sideways and 0.82 g backwards,
from tyres good for roughly 1.1 g. Each figure on its own is plausible.
Together they are fiction.

The failure is worst exactly where a lap is won. On the axes the two rules
agree, because a pure-cornering or pure-braking demand is limited by its own
axis and nothing else. The error grows as the demand moves onto the diagonal,
which is trail braking and corner exit, which is where the interesting driving
happens. A planner built on a model that invents grip on the diagonal will plan
a manoeuvre the car cannot execute, and the car will run wide for a reason the
controller was never told about.

## The projection rule

The fix is to treat the ellipse as one constraint on both components rather
than two constraints on one each. Normalise each component by its own limit and
measure the radius:

```
rho = sqrt( (Fy / (mu_y Fz))^2 + (Fx / (mu_x Fz))^2 )
```

Inside the boundary `rho <= 1` and the tyre delivers what was asked. Outside,
scale **both** components by `1 / rho`, which lands the pair exactly on the
boundary.

For the demand above:

```
rho    = sqrt( (8.0/9.44)^2 + (7.0/9.01)^2 ) = sqrt(0.718 + 0.604) = 1.150
Fy     = 8.0 / 1.150 = 7.0 N
Fx     = 7.0 / 1.150 = 6.1 N
```

![Clipping each axis against projecting onto the budget](assets/combined-slip.svg)

The tyre delivers 7.0 N of cornering force and 6.1 N of braking force, and the
driver who asked for 8.0 and 7.0 gets about 87% of each. Both fall short, in
proportion.

That proportionality is a modelling choice and worth naming rather than
assuming. Scaling both by the same factor preserves the **direction** of the
demanded force and sacrifices its magnitude, which is the neutral answer: the
request was for a particular mix of braking and cornering, and the tyre supplies
less of that mix. Other choices are defensible. A model can prioritise the
longitudinal component, on the grounds that a locked wheel really does give up
its cornering force entirely, and some do. What no defensible model does is
satisfy both demands in full.

> **In SlipX.** `friction_ellipse` in `slipx/tyre.hpp` is the projection, and
> the tier that uses it is L2. Writing it as a per-axis clip was one of the
> mutations tried against the test suite while it was being built, and the
> suite caught it. So was writing the budget as a circle.

## Why an ellipse rather than a circle

Because `mu_x` and `mu_y` are rarely equal. In the numbers above they differ by
about 4.5%, which is small enough to ignore in conversation and not small
enough to ignore in a model that is going to be fitted to data: if you measure
both and then average them, you have thrown away a measurement you paid for.

Which one is larger depends on the tyre. A treaded tyre usually does better
longitudinally, because the tread blocks brace against each other fore and aft
more effectively than they resist a sideways shear. Slicks are closer to equal.
At 1/10 scale, with foam or soft rubber on a smooth indoor floor, the honest
answer is that you should measure it, and
[article 7](07-fitting-a-tyre-model.md) has the manoeuvres.

If you can only measure one of them, use a circle and say so. A circle with a
measured radius is a better model than an ellipse with a guessed axis.

## What the ellipse cannot tell you

Now the limits, which are the reason this article exists rather than a
paragraph in article 1.

**It bounds the force. It does not produce it.** The ellipse is a statement
about which force pairs are achievable, not a mechanism. Ask a physical tyre
model what force comes out and it will tell you that the contact patch resists
the **slip velocity**, so the direction of the force is set by the direction of
the combined slip and not by what anybody demanded. In the brush model the two
components come out roughly in the ratio of `kappa` to `tan(alpha)`. With
`kappa = -0.10` and a slip angle of 5 degrees, that ratio is about 1.14, so the
force is more longitudinal than lateral whether or not that is the mix you
wanted. Direction is an output, and the projection rule treats it as an input.

**The peaks do not line up.** A tyre reaches its lateral peak at one slip angle
and its longitudinal peak at a quite different slip ratio, perhaps 7 degrees
and 0.12 for the same tyre. A point on the ellipse says how much force the tyre
makes, and says nothing at all about which pair of slips you would have to
produce to get it. Turning force back into slip is a separate problem, and a
model that has to do it either iterates or inverts.

**Inside the ellipse is not the same as under control.** This one is
counter-intuitive and worth sitting with. The ellipse bounds achievable force,
and a tyre pushed well past its peak slip angle is making *less* force than it
could, so it sits comfortably inside the boundary while the car is spinning.
Being inside the ellipse means the tyre is not asking the impossible. It does
not mean the tyre is on the useful side of its peak. The falling branch from
article 1 is invisible on this plot, and every stability question lives there.

**The ellipse changes size while you are using it.** Its axes are `mu_x Fz` and
`mu_y Fz`, and `Fz` moves the moment you touch the brakes or the steering
([article 2](02-load-transfer.md)). Worse, `mu` falls as `Fz` rises, so the
growth is less than proportional. Take the same 8.58 N tyre with a load
sensitivity exponent of 0.15, and transfer 3 N from one side of an axle to the
other:

```
loaded    Fz = 11.58 N,  mu = 1.10 * (11.58/8.58)^-0.15 = 1.052  ->  12.18 N
unloaded  Fz =  5.58 N,  mu = 1.10 * ( 5.58/8.58)^-0.15 = 1.173  ->   6.55 N
                                                             total  18.73 N
evenly    2 * 1.10 * 8.58 N                                  total  18.88 N
```

The pair of tyres has 0.8% less grip after the transfer than before it, and
neither ellipse is the one you drew before the corner. This is article 1's
load-sensitivity result arriving in the place it does the damage.

**And it knows nothing about the rest.** Temperature, camber, wear, the
difference between a rolling wheel and a locked one. A locked wheel is
`kappa = -1` and is simply not a state a force budget can describe.

## Why use it anyway

Because every parameter in it is a number you can actually measure.

The ellipse needs `mu_x` and `mu_y`, and both come out of manoeuvres you can
drive in a car park with the sensors already on the car: a skidpad for the
lateral one, a straight-line braking test for the longitudinal one. The full
combined-slip treatment in a complete Magic Formula implementation replaces the
ellipse with a set of weighting functions carrying a dozen or more coefficients,
and those coefficients are identifiable from a rig that sweeps slip angle and
slip ratio independently under controlled load. You do not have that rig, and no
amount of driving reproduces it.

An unidentifiable parameter is worse than an absent one, because the absent one
gets noticed and the unidentifiable one gets guessed, and then believed. A model
that reaches the friction limit approximately, using numbers you measured, tells
you more about your car than one that would reach it exactly if only you knew
fourteen quantities you have no way of knowing.

The cost is stated rather than hidden: on the diagonal, near the limit, in
transients, a friction ellipse is an approximation of the shape of the limit and
not a prediction of the force. If your question is "roughly how much do I have
left", it answers. If your question is "exactly what does the tyre do at
`kappa = -0.4` and 6 degrees of slip", it does not, and neither does anything
else you can fit in a car park.

## In one paragraph

A tyre has one contact patch, so longitudinal and lateral force come out of one
budget, and a model that limits them separately will hand back combinations
that do not exist. Normalising each component by its own limit and scaling both
down when the pair falls outside the unit circle fixes that, preserves the
direction of the demand, and costs two friction coefficients you can measure in
a car park. What it buys is a bound, not a mechanism: it says nothing about
which slips produce a given point, nothing about the falling branch beyond the
peak, and nothing about the fact that the ellipse itself shrinks and swells
under load transfer while you are drawing on it.

## Further reading

- Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012.
  Chapter 4 covers the Magic Formula including its combined-slip weighting
  functions, and is the place to look when you want to know exactly what the
  ellipse is approximating. The brush model, which is where the
  force-follows-slip-direction result comes from, is developed earlier in the
  book.
- Milliken and Milliken, *Race Car Vehicle Dynamics*, SAE International, 1995,
  chapter 2, for measured combined-slip data with the ellipse drawn over it.
  Seeing how well a real tyre fits the curve is worth more than the derivation.
- Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012, chapter 13,
  for a control-oriented presentation you can implement directly.

---

Related: [Tyres and grip](01-tyres-and-grip.md) introduces the ellipse this
article turns into a rule, [load transfer](02-load-transfer.md) is what changes
its size mid-corner, and [speed and the g-g diagram](06-speed-and-the-gg-diagram.md)
is the same budget promoted from one tyre to the whole car. Back to the
[index](README.md).
