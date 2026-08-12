# 10. Differentials: what a driven axle does with two wheels

Put a motor on an axle and you have a decision to make that nobody tells you
about. The two wheels on that axle are going round a corner on different
circles, so they need to turn at different speeds, and the motor has one output
shaft. Something has to reconcile those.

There are three usual answers, they behave completely differently, and the
choice is visible in the car's balance long before it is visible in a lap time.
This article is about the geometry that forces the decision, what each answer
costs, and how to tell from data which one you have.

**Before you start.** [Article 1](01-tyres-and-grip.md) for slip ratio and the
friction budget, and [article 2](02-load-transfer.md) for why the inside wheel
of a corner has less grip than the outside one. [Article 9](09-combined-slip.md)
helps but is not required.

## The geometry that starts the argument

A car going round a corner of radius `R` has its two wheels on an axle tracing
different circles: the outer wheel on `R + t/2` and the inner on `R - t/2`,
where `t` is the track width. They take the same time to get round, so their
speeds are in the same ratio.

![One axle, two path radii](assets/differential-speeds.svg)

At 1/10 scale the track is about 0.24 m, which is large compared with the radii
these cars actually turn in. With 0.32 m of wheelbase and 0.40 rad of steering
travel, the tightest circle available is `L / tan(delta)`, about 0.76 m. So the
speed differences are not small:

| Corner radius | Outer wheel must turn faster by |
|---|---|
| 0.76 m (full lock) | 38% |
| 1 m | 27% |
| 2 m | 13% |
| 4 m | 6.2% |
| 10 m | 2.4% |

A full-size car in the same table looks much tamer, not because its track is
different in kind but because it never turns in 1 m. Small cars live at the top
of this table.

## Three ways to reconcile them

**A solid axle, or "spool", equalises speed.** Both wheels are bolted to the
same shaft. Whatever the geometry wants, they turn together.

**An open differential equalises torque.** This is the defining property and it
is worth stating precisely, because "it lets the wheels turn at different
speeds" is only half of it. A bevel differential's gear train gives you two
things at once: the two output speeds average to the input speed, and the two
output torques are equal. The speeds are free; the torques are not.

**A limited-slip differential equalises torque up to a point, then stops.** It
adds a clutch that will hold a certain torque difference across the axle before
it slips. Below that difference the axle is effectively solid; above it, the
axle behaves as an open one with a fixed bias added.

Each of the three is right about something and wrong about something else.

## What locking costs: the axle fights itself

If both wheels must turn at the same speed but the road wants them to turn at
different speeds, the difference has to appear as slip. Take the axle speed to
be the mean of what the two wheels want, and each wheel is off by about
`t / (2R)` in slip ratio, one positive and one negative.

At `R = 4 m` on our 0.24 m track that is a slip ratio of about 0.03 each way.
That sounds negligible until you put a number on the force. A soft 1/10-scale
tyre has a longitudinal slip stiffness of very roughly 120 N per unit slip, so
0.03 is about 3.7 N, and the two wheels differ by 7.4 N.

Compare that with the tyres' whole budget. At rest each wheel carries about
8.6 N and at a friction coefficient of 1.1 has about 9.4 N to spend. So each
rear tyre is spending around 40% of everything it has on an argument with the
other rear tyre, and the two contributions cancel as far as forward motion is
concerned.

That is the first cost. The second is a yaw moment. The inner wheel is being
driven and the outer one dragged, each 0.12 m from the centreline, so
`2 x 3.7 x 0.12 = 0.89 N m` of moment is trying to straighten the car out.

To get a feel for the size of that, ask what the front axle would have to do to
cancel it. The front axle acts about 0.16 m ahead of the centre of mass, so it
needs another 5.6 N of lateral force, and at a front cornering stiffness of
about 120 N/rad that is 0.047 rad, or **2.7 degrees more front slip angle**.
The steering angle for a 4 m corner is only about 4.6 degrees of Ackermann to
begin with.

That estimate names its assumptions and you should hold it loosely: it holds
speed and radius fixed and ignores the rear axle rebalancing in response. Treat
it as the size of the effect, not a prediction. But the size of the effect is
the point. **A locked axle understeers, and not subtly.**

It understeers off the throttle too, which surprises people. With no net drive
torque at all, the constraint is still there: the inner wheel is still slower
than it wants to be and the outer one faster, so the inner still drives and the
outer still drags. The moment has the same sign whether you are on the power or
off it. This is the "scrub" you can hear from a spooled car on a tight corner
at walking pace.

### And in a tight enough corner it is worse than linear

The forced slip ratio is `t / (2R)`, which grows without limit as the corner
tightens. A rubber tyre's longitudinal force peaks somewhere near a slip ratio
of 0.1 and falls off past it. Setting `t / (2R) = 0.1` gives `R = 1.2 m`.

So on a 1/10-scale car **any corner tighter than about 1.2 m puts at least one
wheel of a locked axle past its longitudinal peak**, where more slip buys less
force and the wheel's lateral grip has collapsed as well
([article 9](09-combined-slip.md)). That is most hairpins on most tracks these
cars run.

## What an open differential costs: the weaker wheel decides

Equal torque both sides sounds fair until you notice which wheel it is being
fair to.

In a corner, load transfers to the outside ([article 2](02-load-transfer.md)).
For our car at 0.64 g the rear axle's outer wheel carries about 11.3 N and the
inner about 5.9 N, so their longitudinal budgets are roughly 12.4 N and 6.4 N.
The differential insists both get the same torque, so the whole axle is capped
at twice the weaker one: 12.9 N. A locked axle could have used
`12.4 + 6.4 = 18.8 N`.

Push harder and it gets worse, because load transfer is what is doing the
damage. At 1.0 g the inner rear is down to about 4.3 N, the open axle can
deliver about 9.4 N, and the locked axle about 18.9 N: a factor of two.

Those are whole-tyre budgets, and in a corner much of each is already spent on
cornering ([article 9](09-combined-slip.md)), so the absolute figures are
optimistic. The ratio between them is what the comparison rests on, and it is
unaffected.

The famous version of this is one wheel in the air, or on ice. Its budget is
zero, so equal torque means zero torque, and the car sits there with one wheel
spinning uselessly.

> **A note on when that actually happens.** A 1/10-scale car with a 0.06 m
> centre of mass height and a 0.24 m track has a static rollover threshold of
> `t / (2h)`, which is 2.0 g. Its tyres give up at about 1.1 g. So this car
> slides long before it lifts a wheel, and the dramatic open-differential
> failure needs a kerb, a bump or a much taller car, not merely a hard corner.
> Raise the centre of mass to 0.18 m and the threshold falls to 0.67 g, below
> the friction limit, and then it lifts.

The compensating virtue is real, though, and it is the reason the open
differential is the sane default: **equal torque both sides produces no yaw
moment at all** on a symmetric car. Equal forces at equal moment arms cancel.
Whatever an open differential does to your traction, it does not silently
change your balance when you touch the throttle.

## The limited-slip differential, in between

A preloaded limited-slip differential holds a fixed torque difference `T_p`
across the axle before the clutch gives up. Below that difference the axle is
locked and behaves exactly like a spool; above it, the clutch slips and the
axle behaves like an open differential with `T_p` added to the slower wheel and
subtracted from the faster one.

The bias always goes to the slower wheel, which in a corner is the inner one
and on a slippery surface is the one with grip. That second case is why they
exist.

Worked, on the numbers above: a preload of 0.2 N m across the axle is a force
difference of `0.2 / 0.05 = 4 N` at the contact patch. Our 4 m corner wanted
7.4 N of difference, so the clutch slips and holds 4 N. A 10 m corner wants
only about 2.9 N, so the clutch holds and the axle is a spool. **The same
differential is a spool on fast corners and half a spool on slow ones**, which
is exactly the design intent: lock up where the speed difference is small and
the traction matters, let go where the geometry is fighting you hardest.

## The other axle, and the other pair

Two footnotes that come up as soon as you leave rear-wheel drive.

**Driving the steered axle** puts the drive force into the same contact patches
that are deciding where the car points. The friction budget is shared
([article 9](09-combined-slip.md)), so throttle costs you steering directly
rather than through a yaw moment. It also makes the differential's torque split
a steering input in its own right, which is what people mean by torque steer.

**Four-wheel drive with a locked centre** adds a front-to-rear speed constraint
on top of the left-to-right one. The front axle traces a larger radius than the
rear, and at low speed the ratio is exactly `1 / cos(delta)`. Even at our full
lock of 0.40 rad that is 8.6%, against 38% across the axle, so the centre
constraint is about four times gentler than a spool. It is not nothing, but if
a 4WD car scrubs on turn-in, look at the axle differentials first.

## Telling them apart from data

You do not need a rig for this, and you do not need to take anything apart.
Wheel encoders on both ends of the driven axle are enough.

Drive a steady circle at low speed and low lateral acceleration, well inside
the friction limit, and log both driven wheel speeds along with the path radius
(from a pose estimate, or just from the circle you drove). Then compare the
measured speed ratio against the geometric one, `(R + t/2) / (R - t/2)`:

- An **open** differential tracks the geometric ratio closely.
- A **spool** holds the ratio at 1.0 and forces the difference into slip.
- An **LSD** tracks the geometric ratio on gentle corners and departs from it as
  you tighten up, and the radius at which it departs tells you the preload.

Two warnings. Do it below the friction limit, or wheelspin will confuse the
measurement with the thing you are measuring. And do it in both directions and
average: any asymmetry in the result is a hint that something else is wrong,
usually alignment.

## Limits of all of this

Stated plainly, because the model above is a set of rules about torque and says
nothing about several real mechanisms.

- **Preload is only the constant part of a real limited-slip differential.**
  A Salisbury unit's locking torque rises with the input torque through its
  ramp angles, and it commonly has different ramps for drive and overrun. A
  single preload number cannot express that.
- **Viscous and gerotor units sense speed difference, not torque**, and they do
  it with a lag of their own. They are a different device that happens to sit
  in the same hole.
- **There is no wheel inertia here.** Lockup and wheelspin are dynamic events
  with their own time constants, and a set of algebraic torque-split rules
  describes the settled state on either side of them, not the transition.
- **Drivetrain friction and shaft windup are ignored.** Both are small at 1/10
  scale and neither is zero.
- **None of this says which is faster.** That is a lap-time question with a
  driver or a controller in the loop, and the answer depends on the track, the
  surface and how much understeer your path tracker can absorb before it starts
  sawing at the wheel.

> **In SlipX.** The double-track tier models all three as closed-form torque
> splits inside its existing force passes, with the open differential as the
> struct default because it is the one that adds no yaw moment. The reasoning,
> including why no wheel rotational state was added to support them, is in
> ADR-0031.

## What to take away

The geometry gives you no choice: on any corner a 1/10-scale car actually
drives, the two wheels of an axle want speeds that differ by several per cent
to several tens of per cent. You can honour that and give up traction to the
unloaded inside wheel, or refuse it and pay in scrub, understeer and slip ratio
that goes past the tyre's peak in anything tighter than about 1.2 m.

If you remember one operational thing: a car that pushes wide on corner entry
with the throttle closed is telling you about its differential, not its tyres.

---

Related: [load transfer](02-load-transfer.md) explains why the inside wheel has
so much less to give, and [combined slip](09-combined-slip.md) explains why
spending the budget on slip ratio costs you cornering force as well. Back to the
[index](README.md).
