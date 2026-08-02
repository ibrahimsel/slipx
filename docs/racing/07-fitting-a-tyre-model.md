# 7. Fitting a tyre model

[Article 1](01-tyres-and-grip.md) gave you the shape of a tyre curve: it rises,
it peaks, it falls, and the peak drops as the tyre is loaded harder. That is
enough to reason with and not enough to simulate with. A simulator wants
numbers, and somebody has to produce them.

This article is about where those numbers come from when all you have is the
car, a car park, and the sensors already bolted to it. It is the least glamorous
part of vehicle dynamics and the part that decides whether anything else you do
means anything.

**Before you start.** Article 1, and specifically the Magic Formula and load
sensitivity sections. Some familiarity with least squares helps but is not
assumed.

## The gap between a model and a fit

A model is a family of curves. A fit is one member of it. The Magic Formula in
its reduced form,

```
Fy = -mu_y(Fz) Fz sin(C atan(B alpha - E (B alpha - atan(B alpha))))
mu_y(Fz) = mu_y0 (Fz / Fz_nom)^(-k_mu)
```

describes an enormous family, including many curves that are not tyres. Picking
the right member needs measurements, and the measurements you can take are
determined by the sensors you have, not by the ones the model would prefer.

The honest starting question is therefore not "what is the best tyre model" but
**"what can I measure, and which parameters does that pin down?"** Get that
order wrong and you end up with a model whose parameters you fill in from a
paper about a different tyre on a different surface, which is a guess wearing a
citation.

## What you actually have

A competition 1/10-scale car carries, typically:

- **Wheel encoders**, giving wheel angular speed, and therefore slip ratio once
  you know the rolling radius.
- **An IMU**, giving body accelerations and yaw rate.
- **A LiDAR**, which after scan matching against a map gives pose, and by
  differentiation gives velocity in the world frame. This is how you get the
  vehicle's **sideslip angle**, the difference between where it points and where
  it is going, which an IMU cannot give you.
- **A steering command**, and if you are lucky a steering angle measurement. The
  difference matters and is discussed below.

What you do not have: a tyre test rig, a load cell in the suspension, a
force-measuring wheel hub, or a way to hold a tyre at a fixed slip angle and
sweep the load. Every parameter has to be inferred from motion of the whole car.

The one advantage of small scale is real and worth stating: you can drive the
car past its limit, repeatedly, in a space the size of a tennis court, without
hurting anybody or breaking anything expensive. Full-size vehicle dynamicists
would love that. It means the peak and the falling branch, the hardest region to
get data in at full scale, is the easy region at 1/10 scale.

## Four manoeuvres and what each one buys

| Manoeuvre | What you drive | What comes out |
|---|---|---|
| Skidpad | Constant radius, speed slowly increasing | `C_alpha`, and `mu_y0` at the point it lets go |
| Straight line | Full throttle from rest, then full braking | `mu_x0`, and longitudinal slip stiffness |
| Ballasted skidpad | The skidpad again with a mass strapped on | `k_mu` |
| Step steer | Straight at constant speed, then a sudden fixed steering input | `sigma`, and the yaw response |

Two of them deserve working through, because the arithmetic shows where the
difficulty is.

### The skidpad, and why it does not give you everything

Drive a constant radius `R` at a slowly increasing speed. Lateral acceleration
is `ay = v^2 / R`, which the IMU also reads directly, so you have a redundant
measurement and a way to check yourself. The steady-state cornering equation
relates the steering angle to it:

```
delta = L / R + K ay
```

`L / R` is the geometric (Ackermann) steering angle, the amount you would need
if the tyres made force with no slip angle at all. Everything above that is the
tyre's contribution, and the slope `K` is the **understeer gradient**:

```
K = (m / L) (l_r / C_f - l_f / C_r)
```

So plot `delta` against `ay`, fit a straight line, and its slope is `K`. Here is
the problem, with numbers. Take a 3.5 kg car, wheelbase 0.32 m, centre of
gravity in the middle, front and rear axle cornering stiffnesses of 120 and
130 N/rad:

```
K = (3.5 / 0.32) (0.16/120 - 0.16/130) = 1.12e-3 rad per m/s^2
```

On a 3 m skidpad the Ackermann angle is `0.32 / 3 = 0.107 rad`, about 6.1
degrees. At 5 m/s<sup>2</sup> of lateral acceleration the tyre contribution is
`1.12e-3 * 5 = 5.6e-3 rad`, about 0.32 degrees. **You are looking for a 5%
change in a small angle.** If what you are recording is the steering *command*
rather than the steering *angle*, servo nonlinearity and slop in the linkage are
comfortably larger than the effect you are trying to measure, and your fit will
be measuring the servo.

The second problem is that `K` is one number and you wanted two. It constrains a
combination of `C_f` and `C_r` and cannot separate them: a car with stiffer
tyres at both ends has the same understeer gradient. To split them you need one
more independent measurement, and the usual one is the body sideslip angle
`beta` from LiDAR pose, because in steady state the rear axle slip angle is

```
alpha_r = beta - l_r * r / v
```

and the rear lateral force is `m ay l_f / L` from the yaw moment balance, which
gives you `C_r` directly and then `C_f` from `K`.

This is a good illustration of a general rule: **an axle's parameters are
identifiable only if something in your sensor set distinguishes the two axles.**
Yaw rate and lateral acceleration alone do not.

### Load sensitivity needs two experiments, not one

`k_mu` is the exponent in `mu(Fz) = mu_0 (Fz/Fz_nom)^(-k_mu)`, and a single
skidpad cannot see it, because a single skidpad has one set of tyre loads.

Run the skidpad, note the lateral acceleration at which the car lets go. Strap
1 kg to the chassis over the centre of gravity, run it again, note the new
figure. If friction were load-independent the two would be identical, since
`ay_max = mu g` has no mass in it. They are not: the heavier car lets go
earlier, and the ratio gives you `k_mu` in one line.

Worked: a 3.5 kg car lets go at 10.8 m/s<sup>2</sup> and the same car at 4.5 kg
lets go at 10.5 m/s<sup>2</sup>. Then

```
mu_1 / mu_2 = 10.8 / 10.5 = 1.029 = (Fz_2 / Fz_1)^(k_mu) = (4.5/3.5)^(k_mu)
k_mu = ln(1.029) / ln(1.286) = 0.113
```

Notice how small the effect is: 28% more mass changed the limit by 3%. This is
a real measurement and it is a delicate one, so it wants several runs and an
honest error bar. It is also the reason `k_mu` values quoted in the literature
vary so much.

> **In SlipX.** The admission criterion for a tyre parameter is exactly this
> question: what manoeuvre produces it? A parameter that fails the test is left
> out rather than defaulted, because a default in a configuration file is
> indistinguishable from a measurement six months later. The reasoning is
> [ADR-0009](../adr/0009-mf-lite-over-full-pacejka.md).

## The trap: parameters that are not independent

Here is the thing nobody tells you until you have already been caught by it.

The Magic Formula's four lateral coefficients are `B`, `C`, `D` and `E`, and
`D` is the peak force. But the initial slope, which is the cornering stiffness
you measured on the skidpad, is

```
C_alpha = B C D
```

So the four coefficients carry three independent pieces of information about the
low-slip and peak behaviour, not four. Fit all four freely to the same data and
you get a valley of equally good answers rather than a minimum: halve `B`,
double `C`, and the curve barely moves near the origin. Two people fitting the
same rosbag will report different `B` and both will be right.

The fix is to fit the quantities you can measure and *derive* the rest:

1. `mu_y0` comes from the limit run, and `D = mu_y0 Fz`.
2. `C_alpha` comes from the low-slip region of the skidpad.
3. `C` and `E` come from the shape between those two, which needs slip sweep
   data.
4. `B` is then **not fitted at all**. It is `C_alpha / (C D)`.

This is not a numerical trick, it is a statement about which quantities have
physical meaning. `C_alpha` is a slope you can measure. `B` on its own is a
number that only means anything in the product `B C D`.

> **In SlipX.** `B` is derived at construction rather than read from the tyre
> file, for the reason above. It has a second benefit: since `Fy` tends to
> `-C_alpha alpha` at small slip by construction, the Magic Formula tier and the
> simpler linear-tyre tier agree exactly in the low-slip limit, so a difference
> between them is a real modelling difference and never a parameter mismatch.

## What C and E actually control

With `C_alpha` and `D` fixed by measurement, `C` and `E` are left deciding one
thing between them: **how far out the peak sits**. The natural yardstick is the
slip angle at which the linear tyre would have reached the peak,

```
alpha_lin = mu_y Fz / C_alpha
```

and the real peak is a multiple of that. The multiple depends on `C` and `E`
alone, and on nothing else: not on the cornering stiffness, not on the friction
coefficient, not on the load.

![Three tyres with the same slope and the same peak, peaking at different slip angles](assets/peak-location.svg)

All three curves in the figure have the same tangent at the origin and reach the
same height. For a plausible tyre the multiple is between about 1.5 and 3, which
is the first two curves. The third is the trap: `C = 1.43` with `E = 0.87` is a
perfectly ordinary looking pair, and it produces a tyre whose peak is at 24
degrees of slip angle. That car never lets go. It just gets vaguer.

Two lessons follow.

**A fit that matches the low-slip data perfectly can still be badly wrong.** The
low-slip region constrains `C_alpha`, which you have already pinned. It says
almost nothing about `C` and `E`, and they are what decide whether your
simulated car has a limit. If your slip sweep data stops at 5 degrees, your `C`
and `E` are decoration.

**Sanity-check the peak location, not just the residuals.** Compute
`alpha_peak / alpha_lin` for your fitted set. If it is above about 4, the fit
has found a curve that agrees with your data and does not describe a tyre.

## What you cannot get in a car park

Stated plainly, because a model's limitations are part of the model:

- **Temperature.** Tyre grip depends strongly on temperature and yours has no
  thermometer in the rubber. Everything above is a fit at whatever temperature
  the tyre happened to be, and a set fitted on a cold first run genuinely
  differs from one fitted after ten minutes of driving.
- **Wear.** Same problem, slower.
- **Camber.** Measuring camber stiffness means holding a tyre at a fixed camber
  angle and measuring force, which needs a rig.
- **Self-aligning moment.** Needs a torque measurement at the steering axis.
- **Anything about the suspension separately from the tyre.** The car park
  measures the two in series and cannot tell them apart.

The correct response to all of these is to leave them out of the model, not to
put a plausible number in. A model that says "I do not represent temperature" is
usable. A model with a guessed temperature coefficient is a model that lies at a
temperature nobody checked.

## Label what you produced

The last step is bookkeeping and it is the one most often skipped. A parameter
set should carry, alongside its numbers:

- **The surface.** A tyre is not a tyre, it is a tyre *on a surface*. Carpet,
  polished concrete and a sports hall floor are three different sets of numbers
  for the same physical wheel, and carrying coefficients across surfaces is one
  of the most common ways a simulation quietly stops matching reality.
- **How the numbers were obtained**: measured, identified from data, or assumed
  plausible. These are not the same claim and they should not look the same in a
  file.
- **Residuals and how much data.** A fit from one skidpad run at one speed is
  not the same object as a fit from twenty runs, even if the numbers agree.

> **In SlipX.** Tyres are referenced as a `(compound, surface)` pair rather than
> embedded in the car file, so that the same chassis on two floors is two
> configurations rather than one silently reused set
> ([ADR-0010](../adr/0010-tyres-are-compound-surface-pairs.md)). Every parameter
> set carries a provenance label of `measured`, `identified` or `provisional`,
> and tooling prints the label rather than only documenting it.

## In one paragraph

Fitting a tyre model is deciding which parameters your sensors can actually
determine, and refusing to invent the rest. A skidpad gives you cornering
stiffness and peak friction; a ballasted repeat gives you load sensitivity; a
slip sweep gives you the shape of the peak; a step steer gives you the lag. The
Magic Formula's coefficients are not independent, so fit the measurable ones and
derive the rest, or two people will fit the same data and report different
numbers. And check where your fitted peak lands, because a curve can match every
data point you have and still describe a tyre that never lets go.

## Further reading

- Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012,
  chapter 4. The Magic Formula and, more to the point here, a discussion of
  which coefficients are identifiable from which kind of test.
- Milliken and Milliken, *Race Car Vehicle Dynamics*, SAE International, 1995,
  chapter 2, for real tyre data and what measured curves look like before
  anybody has fitted anything to them.
- Ljung, *System Identification: Theory for the User*, 2nd ed., Prentice Hall,
  1999. The general theory, and in particular what identifiability means and how
  to tell when a parameter is not.
- Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012, chapter 2,
  for the steady-state cornering equation and the understeer gradient in the
  form used above.

---

Previous: [6. Speed and the g-g diagram](06-speed-and-the-gg-diagram.md) ·
[Series index](README.md)
