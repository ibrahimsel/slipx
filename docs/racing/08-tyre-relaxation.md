# 8. Tyre relaxation: why grip arrives late

Everything in [article 1](01-tyres-and-grip.md) was steady state. You give the
tyre a slip angle, it gives you a lateral force, and the curve tells you how
much. That is a fair description of a car going round a long constant-radius
corner and it is not a description of anything else, because it says the force
appears the instant the slip angle does.

It does not. The delay is small, it is measurable with the sensors you already
have, and it is the reason a steering controller tuned at walking pace
misbehaves at racing pace. This article is about where the delay comes from, how
big it is, and the one property that makes it worth understanding rather than
just tuning around.

**Before you start.** Article 1, particularly the section on slip angle. Article
7 if you want to measure the number yourself. Nothing else.

## Where the delay comes from

Go back to what actually produces lateral force. A tyre carcass is not rigid.
When the wheel is pointed slightly away from where it is travelling, the tread
in the contact patch is dragged sideways relative to the wheel rim, the carcass
deflects, and the force you measure is the elastic reaction to that deflection.
No deflection, no force.

Now ask how the deflection gets there. The tread band is laid down onto the road
at the front of the contact patch and picked up again at the back. A given piece
of rubber acquires its sideways offset by being carried through the patch while
the rim moves laterally underneath it. Until enough new rubber has been laid
down at the new angle, the average deflection across the patch, and therefore
the force, is smaller than the steady-state value.

The important consequence follows immediately. **Building the deflection
requires rolling, not waiting.** A tyre standing still with the wheel turned
does not slowly develop a cornering force, which you can confirm on any parked
car: turn the wheel, and the car does not creep sideways. The tyre has to travel
for the force to arrive.

## The relaxation length

So the natural variable is distance, not time. Write `s` for distance rolled
since the steering input, and the standard first-order model is

```
sigma dx/ds + x = x_steady
```

where `sigma` is the **relaxation length**, in metres, and `x` is the quantity
building up. Solve it for a step input and you get an exponential in distance:

```
x(s) = x_steady (1 - exp(-s / sigma))
```

`sigma` is therefore the distance over which the tyre gets 63% of the way to its
steady-state force, and about 95% of the way there after three of them.

For a full-size car `sigma` is around 0.5 m, roughly one and a half tyre radii.
The same ratio at 1/10 scale, where the wheel radius is about 0.05 m, puts it
near **0.08 m**: three inches of travel to develop most of a corner's grip.

![Tyre relaxation in time and in distance](assets/relaxation.svg)

The two panels are the whole idea. On the left, the same step steer at 3 m/s and
at 12 m/s, plotted against time: two completely different responses. On the
right, the identical two runs plotted against distance rolled: one curve. The
tyre does not have a response time. It has a response *distance*, and the
response time is whatever that distance divides out to at the speed you happen
to be doing.

## What that costs you, in numbers

Divide through by speed and you get a time constant that is not constant:

```
tau = sigma / v
```

For `sigma = 0.08 m`:

| Speed | `tau` | Distance to 95% | Time to 95% |
|---|---|---|---|
| 1 m/s | 80 ms | 0.24 m | 240 ms |
| 3 m/s | 27 ms | 0.24 m | 80 ms |
| 8 m/s | 10 ms | 0.24 m | 30 ms |
| 15 m/s | 5.3 ms | 0.24 m | 16 ms |

The middle column never changes, which is the point. The right-hand column
changes by a factor of fifteen across the speed range of a 1/10-scale car, and
that is the number a controller experiences.

Put it in terms of something you can see. A slalom through cones at 0.5 m
spacing, taken at 3 m/s, gives you about 170 ms between direction changes and a
tyre that needs 80 ms to deliver most of its force. Nearly half of each gate is
spent with the tyre still catching up, and the car's actual lateral acceleration
never reaches what the steering angle is asking for. Take the same slalom at
8 m/s and the tyre is comfortably ahead of the gates; the limit becomes the
steering servo instead.

## Why this bites controllers specifically

A path-tracking controller is a feedback loop around the steering angle, and
relaxation puts a first-order lag inside that loop. Two things follow, and both
are the sort of problem that gets misdiagnosed.

**The lag is speed-varying, so fixed gains cannot be right everywhere.** Phase
lag at a given oscillation frequency scales with `tau`, so a gain tuned to be
crisp at 2 m/s is fifteen times less damped, in phase terms, when the same car
is doing 15 m/s. The usual symptom is a car that tracks beautifully during slow
testing and develops a steering oscillation when you finally give it some
straight-line speed. The instinct is to blame the estimator or the servo.

**It is not the only lag, and the others behave differently.** A steering servo
has a genuine time constant, fixed in seconds regardless of speed. Estimator
delay is fixed. Communication and control-period delay are fixed. Relaxation is
the one term that gets *shorter* as you go faster, which means the total loop
delay is not a single number and its composition changes with speed. If you are
going to schedule gains against anything, speed is the variable that has a
physical reason behind it.

There is a pleasant corollary. Relaxation is worst at low speed, and low speed is
where you have the most margin for error. The regime where the lag is a
millisecond-scale nuisance is also the regime where everything else is hard.

## Measuring it

[Article 7](07-fitting-a-tyre-model.md) lists the step steer as the manoeuvre
that produces `sigma`, and this is why. Drive straight at a constant speed, apply
a fixed steering angle as fast as the servo allows, and record yaw rate.

The yaw rate does not step; it rises with a time constant. That time constant is
not `sigma / v` on its own, because the car's own yaw inertia contributes a lag
of its own and the two are in series. The trick that separates them is to **run
the same test at two speeds**. The yaw-inertia term scales one way with speed and
the relaxation term scales as `1/v`, so two runs give you two equations, and
`sigma` falls out.

Two practical warnings. Do it well below the friction limit, or you are fitting
the tyre's saturation rather than its transient. And measure the steering angle
rather than assuming the command was achieved, or you will fit the servo's lag
and call it a tyre property, which is the single most common way this
measurement goes wrong.

## What is lagged: a subtlety worth the paragraph

There are two ways to implement this, and they are not equivalent.

You can lag the **force**: compute the steady-state force from the current slip
angle, then filter it. Or you can lag the **slip angle**: filter the slip angle
first, then look up the force from the tyre curve at the filtered value. Both
reproduce the exponential above. Both are called the relaxation length model.

They differ when the vertical load changes quickly, which in a corner it does.
Suppose a wheel on the inside of a hard corner unloads to nothing, right at the
point of lifting. Its grip budget is `mu Fz`, so its budget is now zero: whatever
force it was producing, it can produce none. A lagged *force* does not know that.
It decays towards zero over `sigma / v`, which at 10 m/s is 8 ms, and for those
8 ms your model reports a tyre pushing sideways on a road it is not touching.

A lagged *slip angle* cannot do this. The force is looked up from the current
load every instant, so it is inside the current budget by construction, and the
only thing carrying history is the slip the tyre thinks it has, which is exactly
the physical claim: the carcass is still deflected, but a deflected carcass on a
lifted wheel pushes on nothing.

The price is that filtering the input to a curved function is not the same as
filtering its output, so the two agree exactly only in the straight part of the
tyre curve. Near the limit they differ slightly. One of them, however, cannot
invent grip that does not exist, and that is a better property to have than
agreement with the other one.

> **In SlipX.** The double-track tier carries a lagged slip angle per wheel as a
> genuine state, which is why adding it changed every published trajectory hash.
> The reasoning above is recorded in ADR-0026.

## Limits of this model

Stated plainly, because a first-order lag is a rough instrument.

- **It is first order.** A real transient has a small amount of overshoot from
  carcass inertia. First order has none, and cannot represent it.
- **`sigma` is treated as a constant and is not one.** It falls as vertical load
  rises and falls as the slip angle approaches the peak, by perhaps 30% across
  the working range. Modelling that needs measurements a car park does not
  provide, so the usual choice is one number identified near the operating
  point.
- **It says nothing about load transfer.** The vertical load moving from one
  wheel to another has its own transient, through the suspension, of the order
  of tens of milliseconds. That is a different mechanism with a different time
  constant, and quasi-static load transfer models ([article
  2](02-load-transfer.md)) ignore it entirely.
- **It is a lateral story here, and the longitudinal one exists too.**
  Longitudinal force has its own relaxation length, generally shorter. It gets
  much less attention because a driver rarely steps the throttle the way they
  step the steering.
- **At a standstill the model freezes**, which is right, and it means the lagged
  slip angle after a stop is whatever it was when the car stopped. That is
  physically sensible and occasionally surprising in a simulation that starts
  from a strange state.

## What to take away

The tyre's response is a distance, not a time. One relaxation length of travel
gets you 63% of the force, three gets you 95%, and dividing by speed converts
that into whatever delay your controller is actually fighting. It is roughly
80 mm on a 1/10-scale car, which is 27 ms at 3 m/s and 5 ms at 15 m/s.

If you remember one operational thing: when a car that tracked well slowly
starts oscillating quickly, the plant changed, not the tuning.

---

Related: [Fitting a tyre model](07-fitting-a-tyre-model.md) covers the step
steer that measures `sigma`, alongside the other manoeuvres, and [load
transfer](02-load-transfer.md) covers the other transient in the same corner.
Back to the [index](README.md).
