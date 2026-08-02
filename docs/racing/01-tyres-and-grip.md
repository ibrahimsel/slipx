# 1. Tyres and grip

Everything a car does to change its speed or direction, it does through four
contact patches. On a 1/10-scale racecar each one is about the size of a
thumbnail. Steering, braking, accelerating and spinning are all the same
physical event seen from different angles: a rubber patch being asked for a
force, and either supplying it or not.

This is the article the other five depend on. If you take one thing from the
series, take the shape of the curve in the second figure.

**Before you start.** Nothing beyond school mechanics: forces, Newton's second
law, and enough trigonometry to be unbothered by `atan2`. If "coefficient of
friction" rings a bell you are fully equipped, and one of the first things
below is why that bell is slightly out of tune.

## Friction, and why the school version misleads you

The version everybody learns is Coulomb's: the friction force between two
surfaces is at most `mu` times the normal force, it opposes sliding, and `mu`
is a property of the pair of materials. It is a good model of a crate on a
ramp. It is a poor model of a tyre, in three specific ways, and each of the
three is worth a section of this article.

1. A tyre only makes force **when it is slipping a little**. Not sliding, not
   locked, but deforming and creeping. Grip is a function of slip, and at zero
   slip it is zero.
2. `mu` is **not constant**. It falls as the tyre is pressed harder, which is
   why moving load around a car loses grip overall.
3. There is **one budget** shared between braking and cornering, and spending
   it on one leaves less for the other.

The first is the one that changes how you think, so start there.

## Slip angle

Point a rolling wheel one way and push the car sideways. The wheel does not
travel exactly where it points. The rubber in the contact patch distorts, the
patch itself sits slightly to one side of the wheel plane, and the wheel ends
up travelling at a small angle to the direction it is aimed.

That angle is the **slip angle**, written `alpha`.

![Slip angle: the wheel points one way and travels another](assets/slip-angle.svg)

Formally, take the velocity of the contact patch, resolve it into a component
along the wheel plane and a component across it, and

```
alpha = atan2(v_lat, v_lon) - delta
```

where `delta` is the steering angle of that wheel. In ISO 8855, where y points
left, `alpha` is positive when the wheel's velocity lies to the **left** of
where the wheel is pointing, and the force the tyre makes opposes that, so it
is negative:

```
Fy = -C_alpha * alpha        (small angles, C_alpha > 0)
```

That minus sign is the single most common place to get ISO and SAE mixed up.
Under SAE the slip angle carries the opposite sign and the same physics is
written `Fy = +C_alpha * alpha`. Both describe a restoring force. Check which
convention a source uses before you trust a sign from it.

`C_alpha` is the **cornering stiffness**, in newtons per radian. It is the
slope of the force curve at zero, and it is the single most useful number
about a tyre, because it is what determines how the car responds to small
steering inputs, which is nearly all of the steering inputs a well-driven car
receives.

The consequence people find least intuitive: **the steering wheel does not
generate cornering force.** It generates a slip angle, and the tyre generates
force from the slip angle. If a tyre has no slip angle it makes no lateral
force no matter how hard it is being steered, which is exactly what happens
when a car is stationary, and exactly what happens when a wheel is in the air.

### Slip ratio

The longitudinal version of the same idea. A driven wheel turns slightly
faster than the road passes under it, and a braked wheel slightly slower. The
fractional difference is the **slip ratio**, `kappa`:

```
kappa = (omega * R_e - v_lon) / max(|v_lon|, v_eps)
```

with `omega` the wheel's angular speed and `R_e` its effective rolling radius.
Positive under drive, negative under braking. Everything said below about
lateral force applies, with the same shape of curve, to longitudinal force.

The `v_eps` in the denominator is not physics. It is a floor to stop the
expression dividing by zero at a standstill, and it is worth noticing because
every implementation needs one and each one puts it somewhere slightly
different.

## The tyre curve

Now plot the force a tyre makes against the slip angle you give it. This is
the shape:

![Lateral force against slip angle](assets/tyre-curve.svg)

Three regions, and they are three different cars to drive.

**The rising part**, up to two or three degrees for a typical small tyre. Force
is close to proportional to slip angle. More steering gives more grip, which
means the car does what it is told and mistakes correct themselves. Nearly all
normal driving happens here, and a linear model of the tyre is genuinely
accurate in this region.

**The peak.** All the grip there is. In a lap time sense this is where you want
to be, and it is a narrow target: notice how flat the curve is near the top,
which is why the fastest drivers are not obviously working harder than the
merely quick ones.

**The falling branch**, past the peak. More slip angle gives **less** force.
This is where a car spins, and the mechanism is worth spelling out because it
is the reason the region matters so much. Suppose the rear tyres go slightly
past their peak. They make less force. Less rear force means the car rotates
further than intended, which increases the rear slip angle, which takes the
tyres further past the peak, which makes even less force. It is positive
feedback, and it is fast. Beyond a certain point no steering input recovers it
because the front tyres cannot generate a moment large enough to arrest the
yaw.

Note what follows for models: a tyre model **without a falling branch cannot
spin a car**. A linear model that is simply clipped at some maximum force, the
dashed line in the figure, slides at the limit and then recovers the instant
the slip angle comes back, because at every slip angle above the limit it is
still making maximum force. That is a qualitatively different vehicle, not a
slightly less accurate one.

### The Magic Formula

The usual way to write the curve down is Pacejka's Magic Formula, so called
because it is an empirical fit with no derivation behind it. It reproduces the
measured shape with a handful of coefficients:

```
Fy = D sin(C atan(B alpha - E (B alpha - atan(B alpha))))
```

`D` is the peak, `C` sets how far the curve falls away after it, `B` sets the
initial slope through `C_alpha = B C D`, and `E` adjusts the curvature near the
peak. The full version has dozens of coefficients covering camber, conicity,
turn slip and more.

> **In SlipX.** The library uses a reduced version, "MF-lite", chosen so that
> every coefficient can be identified from a manoeuvre you can drive in a car
> park with the sensors already on a competition car. The reasoning is in
> [ADR-0009](../adr/0009-mf-lite-over-full-pacejka.md), and the short version
> is that a parameter nobody can measure does not stay absent: it gets guessed,
> and a guess in a configuration file is indistinguishable from a measurement.

## Load sensitivity

Here is the second place the school model misleads. Press a tyre down twice as
hard and it does **not** make twice the grip. It makes appreciably less than
twice.

![Load sensitivity](assets/load-sensitivity.svg)

The friction coefficient itself falls with vertical load, roughly as a power
law:

```
mu(Fz) = mu_0 * (Fz / Fz_nominal)^(-k_mu)
```

with `k_mu` somewhere around 0.1 to 0.2 for many tyres. The mechanism is that
the contact patch grows less than proportionally with load and the rubber is
worked harder within it, but for present purposes the fact is enough.

The consequence is the one that matters, and it is not obvious, so here it is
with numbers. Take a pair of tyres on one axle, each carrying 8.6 N, with
`mu_0 = 1.10` at that load and `k_mu = 0.15`. Together they can make about
`2 * 1.10 * 8.6 = 18.9 N`.

Now move 4 N from one to the other, so they carry 4.6 N and 12.6 N. The lightly
loaded one now enjoys `mu = 1.10 * (4.6/8.6)^-0.15 = 1.21` and makes 5.6 N. The
heavily loaded one suffers `mu = 1.10 * (12.6/8.6)^-0.15 = 1.04` and makes
13.1 N. Together: 18.7 N. Slightly less than before.

Move 7 N instead, so they carry 1.6 N and 15.6 N, and the total falls to
about 18.0 N.

**Unequal loading always loses grip**, and the more unequal, the more is lost.
That single fact is the whole reason the next article exists, because driving a
car is a continuous exercise in moving load between its tyres whether you
intend to or not.

## The friction ellipse

The third correction. A tyre has one contact patch, and braking and cornering
draw on the same one.

![The friction ellipse](assets/friction-ellipse.svg)

Take the longitudinal force and the lateral force as the two axes of a plot.
Everything the tyre can do lies inside a closed curve. Approximately:

```
(Fx / (mu_x Fz))^2 + (Fy / (mu_y Fz))^2 <= 1
```

It is an ellipse rather than a circle because the longitudinal and lateral
peak friction coefficients are rarely equal. People say "friction circle" out
of habit and usually mean this.

Reading it is the useful part. Inside the boundary the tyre has grip to spare
and will do as it is told. On the boundary, every extra newton of braking must
be paid for with a newton of cornering, which is what a driver means by "I ran
out of grip on the way in". The corners of the plot are the interesting
region: a car that brakes in a straight line, turns, and then accelerates
traces a cross through the middle and leaves the diagonal parts of the boundary
completely unused. Using them is called trail braking, and the
[g-g diagram article](06-speed-and-the-gg-diagram.md) is about exactly that.

Combine this with load sensitivity and the picture sharpens: the ellipse is
not fixed. It shrinks on an unloaded tyre and grows on a loaded one, but grows
less than proportionally, so a car with load piled onto its outside tyres has
a smaller total ellipse to work with than the same car with its load spread
evenly.

## Relaxation length

One last property, easy to miss and responsible for a surprising amount of
transient behaviour. When you suddenly change a tyre's slip angle, the force
does not appear instantly. The rubber has to distort, and that takes rolling
distance to happen.

The distance constant is called the **relaxation length**, `sigma`, and it is
of the order of the contact patch length: a few centimetres on a small tyre.
The force approaches its steady value roughly exponentially in distance
travelled, so

```
sigma * dFy/ds + Fy = Fy_steady
```

which, at a speed `v`, is a first-order lag with time constant `sigma / v`. At
5 m/s with `sigma = 0.05 m` that is 10 ms. Small, but not nothing: it is the
same order as a control loop period, and it is part of why a car feels less
responsive at low speed than the steady-state maths says it should.

`sigma` is identifiable from the delay between a step steering input and the
resulting yaw response, which makes the step steer a standard identification
manoeuvre.

## In one paragraph

A tyre makes force by slipping. The relationship between slip and force rises,
peaks, and then falls, and the falling part is what spins cars and what
distinguishes a real tyre model from a clipped linear one. The peak friction
coefficient drops as the tyre is loaded harder, so spreading load evenly across
four tyres produces more total grip than concentrating it on two. Longitudinal
and lateral force share one budget shaped like an ellipse. And none of it
happens instantly: force follows slip with a lag measured in centimetres of
rolling.

## Further reading

- Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012.
  Chapter 4 is the Magic Formula; chapter 7 covers transient behaviour and
  relaxation length.
- Milliken and Milliken, *Race Car Vehicle Dynamics*, SAE International, 1995,
  chapter 2. The best plain-language treatment of slip angle and the friction
  ellipse, with real tyre data.
- Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012, chapter
  13, for a control-oriented presentation with the equations in a form you can
  implement directly.

---

Next: [2. Load transfer](02-load-transfer.md) ·
[Series index](README.md)
