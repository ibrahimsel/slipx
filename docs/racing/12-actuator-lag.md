# 12. Actuator lag: the steering angle you asked for is not the one you have

[Article 8](08-tyre-relaxation.md) was about a delay between the steering angle
and the grip. This one is about a delay one step earlier in the chain: between
the steering angle you commanded and the steering angle the front wheels
actually have.

It is a bigger delay than the tyre's for almost the whole speed range of a
1/10-scale car, it is described by three separate numbers that get confused with
each other constantly, and unlike the tyre's delay it does not get better when
you go faster.

**Before you start.** [Article 8](08-tyre-relaxation.md), because the last
section of this one is that article read in reverse. Some familiarity with what
a second-order system is will help, but the numbers below stand on their own.

## Three numbers, not one

When somebody says a servo is "slow", they could mean any of three things, and
which one they mean changes what you should do about it.

- **Slew rate limit**, in rad/s. A hard ceiling on how fast the output angle can
  change, at all, ever.
- **Bandwidth**, in rad/s. How quickly the servo's control loop converges on a
  new setpoint when it is nowhere near the slew limit.
- **Damping ratio**, dimensionless. Whether it converges smoothly, overshoots,
  or rings.

Only the first of these appears on a hobby servo's datasheet, and it appears in
disguise. A specification like "0.10 s per 60 degrees at 6 V" is a slew rate:
60 degrees is 1.05 rad, so that servo moves at about **10 rad/s**. The other two
numbers you have to measure.

## The slew limit, and whether it binds

A rate limit either engages or it does not, and you can work out which without
simulating anything.

The step response of a second-order system has a single peak in its rate, and
that peak works out to

```
peak rate = A * wn * f(zeta)
```

where `A` is the step size, `wn` is the bandwidth and `f(zeta)` depends on the
damping ratio alone. At the common damping ratio of 0.7, `f` is about 0.46.

Take a servo with `wn = 45 rad/s` and `zeta = 0.7` on a car with 0.40 rad of
steering travel. The largest step anyone can command is lock to centre, 0.40
rad, and its peak rate is

```
0.40 * 45 * 0.46 = 8.3 rad/s
```

against a 10 rad/s limit. **This servo never slews.** Not at full lock, not
ever. Rearranged, the limit only binds for steps larger than
`10 / (45 * 0.46) = 0.48 rad`, which is beyond the mechanical stop.

Fit a stiffer servo, `wn = 200 rad/s`, and the same full-lock step wants
`0.40 * 200 * 0.46 = 37 rad/s`, nearly four times the limit. Now it slews, and
the response changes shape completely.

![Bandwidth-limited and slew-limited responses, and how they compare with the tyre](assets/servo-step.svg)

The left panel is the two cases side by side and they are easy to tell apart in
logged data. A bandwidth-limited response is a smooth S with a slight overshoot.
A slew-limited response is a **straight line** at the limit, and then it arrives.
If your logged steering angle has a straight segment in it, you are rate
limited, and no amount of controller tuning will change that segment.

Which matters, because a rate limit is a nonlinearity. Once it engages,
superposition is gone: the small-signal behaviour you tuned against no longer
predicts the large-signal behaviour, and rate-limited loops are a classic source
of limit cycles that appear only on big inputs and vanish when you go looking
for them gently.

## Bandwidth and damping

Below the rate limit, a position servo is a control loop driving an inertia, and
the standard second-order form fits it well:

```
d2(delta)/dt2 = wn^2 (delta_cmd - delta) - 2 zeta wn d(delta)/dt
```

The first term is proportional feedback on position error and the second is
damping, from the loop's derivative term plus whatever friction the mechanism
has. It is worth remembering that this is a control loop and not a spring: `wn`
is a gain, so it moves when the supply voltage moves or the load changes.

At `wn = 45 rad/s` (7.2 Hz) and `zeta = 0.7`, the standard results are:

| Quantity | Formula | Value |
|---|---|---|
| Overshoot | `exp(-pi zeta / sqrt(1 - zeta^2))` | 4.6% |
| Time to the overshoot peak | `pi / (wn sqrt(1 - zeta^2))` | 98 ms |
| Rise time, 10% to 90% | about `2.1 / wn` at this damping | 47 ms |
| Settled within 5% | about `3 / (zeta wn)` | 95 ms |

Damping trades the overshoot against the rise time, and does it steeply:

| `zeta` | Overshoot | Rise time at `wn = 45` |
|---|---|---|
| 0.3 | 37% | 29 ms |
| 0.5 | 16% | 36 ms |
| 0.7 | 4.6% | 47 ms |
| 0.9 | 0.15% | 64 ms |
| 1.0 | 0 | 75 ms |

0.7 is the usual choice because it is roughly where the curve turns: below it
the overshoot grows much faster than the rise time shrinks.

**The overshoot is not a fault.** 4.6% of a 0.40 rad step is 0.018 rad, about
one degree of extra steering held briefly around the 98 ms mark. Whether that
matters depends entirely on where you are on the tyre curve: in the linear
region it is nothing, and near the peak it is enough to push an axle over the
top and give you a momentary snap of oversteer that has no cause anywhere in
your planner.

It also means that "the achieved angle never exceeds the command" is not a
property a real steering system has. Any check written on that assumption fires
spuriously on every underdamped servo, which is most of them.

## How it compares with the tyre's own lag

[Article 8](08-tyre-relaxation.md) made the case that the tyre's response is a
distance, not a time, so its time constant is `sigma / v` and shrinks as you go
faster. The servo is the opposite: its time constant is fixed in seconds, set by
a control loop that knows nothing about road speed.

Compare them at the same threshold. The tyre reaches 95% of its steady force
after three relaxation lengths, `3 sigma / v`, which at `sigma = 0.08 m` is
0.24 m of travel. The servo settles within 5% at about 95 ms.

| Speed | Tyre, `3 sigma / v` | Servo | Slower one |
|---|---|---|---|
| 1 m/s | 240 ms | 95 ms | tyre |
| 2.5 m/s | 96 ms | 95 ms | equal |
| 5 m/s | 48 ms | 95 ms | servo, 2x |
| 10 m/s | 24 ms | 95 ms | servo, 4x |
| 15 m/s | 16 ms | 95 ms | servo, 6x |

The crossover is at about **2.5 m/s**, which is walking pace. Above it, and
therefore for essentially all of a racing lap, **the steering actuator is the
slower element and the tyre is not.** Article 8's advice to schedule gains
against speed is still right, but the term it was scheduling against is the
smaller of the two nearly everywhere.

The useful way to hold both ideas at once: total loop delay is a sum, and only
one term in it moves with speed. At a crawl the composition is mostly tyre; at
racing speed it is mostly servo, plus the fixed estimator and control-period
delays that were always there.

## The slalom, in the frequency domain

Article 8 worked a slalom at 0.5 m cone spacing and found the tyre comfortably
ahead of the gates at 8 m/s. Do the same sum for the servo.

At 6 m/s, 0.5 m spacing gives 83 ms per gate, and a full left-right cycle spans
two gates, so the steering is being commanded at about **6 Hz**. The servo's
natural frequency is 7.2 Hz. You are driving it at 0.84 of `wn`, and the
standard second-order frequency response there gives:

- **magnitude 0.83**: you get 83% of the steering amplitude you asked for, so
  your effective steering gain has quietly dropped by a sixth;
- **phase lag 76 degrees**: more than a fifth of a cycle.

Seventy-six degrees of phase lag inside a feedback loop is not a detail; it is
most of a stability margin. And neither number appears anywhere in the vehicle
model, the tyre model or the planner. The car is simply not doing what it was
told, in a way that gets worse the more urgently you tell it.

## The travel stop

One more mechanism, briefly, because it interacts badly with everything above.

The steering has a hard mechanical end stop, and it is inelastic: the rack
arrives and stops. If the command sits beyond the stop, the servo pushes against
it, and any integrator upstream keeps accumulating error against an output that
cannot move. This is integral windup with a mechanical cause, and it needs the
same treatment: clamp the integrator when the actuator is saturated, and make
the saturation visible to whatever is doing the clamping.

## Measuring your own numbers

All of it is measurable on the bench and better on the car.

- **Slew rate**: command a full-lock step and log the achieved angle. Read the
  slope off the straight part. If there is no straight part, you are not rate
  limited and you have learned that instead.
- **`wn` and `zeta`**: command a step small enough not to slew, using the
  calculation above to choose the size, and fit the response. Or sweep a sine
  and take `wn` from the phase crossing 90 degrees and `zeta` from the size of
  the resonant peak.
- **Do it at the operating voltage.** A servo's loop gain and its slew rate both
  scale with supply voltage much as the drive motor's do
  ([article 11](11-motor-esc-and-battery.md)). A 6 V datasheet figure is not a
  7.4 V figure.
- **Do it with the car's weight on the wheels.** Datasheet speeds are unloaded.
  A loaded steering rack is slower, and by an amount that depends on your
  geometry.

## Limits of this model

- **Second order is a fit, not a mechanism.** A digital servo runs a sampled
  control loop, often at a few hundred hertz, with quantised position feedback.
  Near its bandwidth, the sampling is part of the answer.
- **Deadband and stiction are not represented at all.** Below some command
  change, a real servo does not move. Around the straight-ahead position, where
  a path tracker spends most of its time making small corrections, that is
  frequently a bigger problem than the lag.
- **Backlash and linkage compliance add angle error that is not a lag.** It does
  not decay with time; it depends on the direction you last moved and on how
  hard the tyres are pushing back.
- **The steering ratio is not constant.** Road wheel angle is a nonlinear
  function of servo angle across the range, and Ackermann geometry means the two
  front wheels do not even agree with each other.
- **No temperature, and no voltage sag.** The last is worth a sentence of its
  own: the servo draws from the same pack as the drive motor, so a fast steering
  reversal during hard acceleration is exactly when the supply is weakest, which
  makes the servo slowest precisely when you were asking the most of it.

> **In SlipX.** The double-track tier carries the steering angle and its rate as
> integrated state, with a slew limit and an inelastic travel stop, so commanded
> and achieved angle are separate quantities you can compare. Lower tiers steer
> instantly, which is one of the things that makes them lower tiers. ADR-0031
> records the choice.

## What to take away

A steering actuator has three limits and they are not interchangeable: a slew
rate that either engages or does not, a bandwidth that sets how quickly it
converges, and a damping ratio that decides whether it overshoots on the way. On
a plausible 1/10-scale setup the slew limit never engages, the bandwidth costs
you about 95 ms of settling, and at slalom frequencies you are getting 83% of
the amplitude you asked for with 76 degrees of phase lag.

Above walking pace the servo is a bigger lag than the tyre, and unlike the tyre
it does not improve as the car speeds up. If you remember one operational thing:
**log the achieved steering angle**. Almost every mistake in this article is
invisible if you only ever log the command.

---

Related: [tyre relaxation](08-tyre-relaxation.md) is the other half of this
comparison, and [the motor, the ESC and the battery](11-motor-esc-and-battery.md)
covers the actuator at the other end of the car. Back to the
[index](README.md).
