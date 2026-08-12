# 11. The motor, the ESC and the battery: where acceleration comes from

Most planning code treats acceleration as a number you ask for. You compute a
speed profile, you subtract, you divide by the timestep, and out comes a
command. Somewhere below that is a motor which has its own opinion, and the
opinion changes with how fast you are already going and with how long the car
has been switched on.

This article is about the shape of that opinion. It is a straight line, two
horizontal clips and a scale factor, and once you have seen it you can predict
most of what a small electric drivetrain does without measuring anything else.

**Before you start.** Nothing from earlier in this series is strictly required.
[Article 6](06-speed-and-the-gg-diagram.md) is the natural companion, because
this article is largely a list of reasons the g-g diagram's top and bottom edges
are not where you assumed.

## The torque-speed line

A permanent-magnet motor has two constants that are really one constant. It
makes torque in proportion to current, `T = k I`, and it generates a voltage in
proportion to speed, `e = k w`, and in SI units the two `k` are the same number.

Put a terminal voltage `V` across it. The winding sees `V` minus the generated
voltage, so the current is `(V - k w) / R_m` where `R_m` is the winding
resistance, and therefore

```
T(w) = k (V - k w) / R_m
```

That is a straight line falling with speed, and it is easiest to remember by its
two ends:

- **Stall torque** `T_stall = k V / R_m`, at zero speed.
- **Free speed** `w_free = V / k`, where the generated voltage has caught up with
  the terminal voltage and no current flows.

Between them the line is `T = T_stall (1 - w / w_free)`. Everything else in this
article is a modification of that line.

A gearbox does not change the shape. Reducing by a ratio `n` multiplies torque by
`n` and divides speed by `n`, so the line at the wheel is the same line with
different numbers on the axes. That is a good reason to write the whole thing at
the wheel and never mention `k`, `R_m` or the gear ratio again: two numbers at
the wheel say everything three numbers at the motor did.

For a 1/10-scale car, plausible wheel-referred values are a stall torque around
**2 N m** and a free speed around **480 rad/s**, which with a 0.05 m wheel is
**24 m/s** of road speed.

## The current limit, and why it clips the launch

The stall torque above is what the motor would make if you let the full stall
current through it. Nobody does. At zero speed there is no generated voltage,
so the current is limited only by the winding resistance, and a small motor's
winding will not survive that for long. Every ESC therefore has a current
limit, and because torque is proportional to current, a current limit is a flat
ceiling on torque.

![Torque and power against speed, with the current limit](assets/torque-speed.svg)

Say the limit works out at 1.2 N m at the wheel. The line only falls to 1.2 N m
at `24 (1 - 1.2/2) = 9.6 m/s`, so **everything below 9.6 m/s is flat** and the
line is the limit only above that.

Now the surprising part. The peak power of a straight torque-speed line sits at
exactly half the free speed, here 12 m/s, where the torque is 1 N m and the
power is `1 x 240 = 240 W`. Since 12 m/s is above 9.6 m/s, **the cap does not
touch the peak power at all.** A current limit costs you launch torque and
nothing else.

And on this car it does not really cost you the launch either. Work out what
the rear tyres could put down, allowing for load transferring rearwards as you
accelerate ([article 2](02-load-transfer.md)):

```
m a = mu (m g / 2 + m a h / L)
```

With `mu = 1.1`, `h = 0.06 m` and `L = 0.32 m`, this solves to
`a = 6.80 m/s^2`. The current-limited torque of 1.2 N m gives
`1.2 / 0.05 / 3.5 = 6.86 m/s^2`. The two agree to within one per cent: on these
numbers the ESC's limit lands almost exactly where the tyres give up anyway.
The unclipped 2 N m would have asked for 11.4 m/s^2 against a budget of about
6.8, which is not a launch, it is wheelspin.

That agreement is a coincidence of one plausible parameter set and not a law,
but it is the right instinct about small electric cars: **they are usually
traction-limited off the line and current-limited nowhere that matters.**

## The battery scales the whole line

Both ends of the torque-speed line are proportional to `V`. So halving the
voltage halves the stall torque and halves the free speed, the line shrinks
towards the origin in both directions at once, and the peak power, being a
product of the two, goes as `V^2`.

That square is the whole story of why a car gets slower during a run.

A 3S lithium-polymer pack is three cells in series: about 4.2 V each when full
and about 3.3 V each when you should stop, so **12.6 V down to 9.9 V**. That is
a 21% fall in voltage and therefore a **38% fall in peak power**, with the free
speed dropping from 24 m/s to about 19 m/s along the way. Nothing has broken.
The car is simply a different car at the end of the run, and any controller
holding a fixed model of its own acceleration is now wrong by a third.

On top of that slow drift there is a fast one. A pack has internal resistance,
so the terminal voltage under load is `V = V_oc - I R_i`, and the loss goes
straight into the square above.

For a good 5 Ah pack, `R_i` including the connectors might be 20 mohm. Drawing
25 A, which is roughly what 240 W at 12 V looks like, that is 0.5 V, which is
4% of voltage and 8% of power. Real but not dramatic.

The dramatic case is a tired pack. Internal resistance rises as a pack ages and
as it drains, and a pack at 60 mohm pulling a 40 A burst loses 2.4 V, a fifth of
everything, and therefore about a third of its power. **Pack condition changes
your car more than pack capacity does**, and it is the number nobody on a
grid checks.

Energy, by contrast, is rarely the binding constraint over a short run: 5.2 Ah
at about 11 V is 58 Wh, which would take a quarter of an hour to spend at the
240 W peak, and no car averages its peak. You will notice the voltage long
before you run out of energy.

## Braking is not the mirror image of accelerating

A 1/10-scale competition car has no friction brakes. There is nothing at the
wheels but the motor, so braking means running the motor as a generator and
pushing current back into the pack. It is much weaker than people expect, for
two independent reasons.

**The pack will not take it.** A lithium pack discharges happily at twenty or
fifty times its capacity in amps and charges at perhaps one to five times. ESC
firmware caps regenerative current well below drive current in consequence, and
a factor of three is typical: 40 A back against 120 A out.

**It acts through the driven wheels only.** A rear-drive car brakes with two
tyres, not four, and the two it uses are the two that unload under braking
([article 2](02-load-transfer.md)).

Put numbers on it. A 40 A regen limit at the same torque-per-amp gives 0.4 N m
at the wheel, which is 8 N of retarding force and `8 / 3.5 = 2.3 m/s^2`, about
**0.23 g**. The tyres, if you had brakes at all four corners, would have given
you something near 1.1 g.

The consequence is a braking distance you have to plan around rather than
discover. From 10 m/s:

| Limited by | Deceleration | Distance to stop |
|---|---|---|
| Regen current | 0.23 g | 22 m |
| Tyres, four wheels | 1.1 g | 4.6 m |

Nearly five times the distance. A g-g diagram drawn for this car
([article 6](06-speed-and-the-gg-diagram.md)) is not an ellipse; it is an
ellipse with the bottom flattened almost to the axis, and a speed profile
computed with a symmetric friction limit will ask for braking the car cannot
begin to deliver.

There is a third effect worth knowing. Regenerative torque comes from the
generated voltage, which is proportional to speed, so it fades as you slow down
and reaches zero at a standstill. The last metre is a coast, and a motor-braked
car cannot hold itself on a slope.

## What this means for a controller

Four consequences, in rough order of how often they bite.

- **`a_max` is not a constant.** It depends on speed through the line and on
  voltage through the scale, and both are things you can measure. A feedforward
  term that evaluates the curve at the current wheel speed and pack voltage
  costs nothing and removes most of the error.
- **Your braking authority is a small fraction of your cornering authority.**
  Plan braking points from the drivetrain limit, not the friction limit, unless
  you have measured otherwise.
- **The plant drifts within a single run.** The same command produces visibly
  less acceleration at minute eight than at minute one. Adaptive or
  voltage-scheduled gains have a real physical justification here, unlike most
  places they get proposed.
- **Saturation needs to be reported, not silently applied.** A controller that
  asks for 3 N m and receives 1.2 N m, with no way to know, will integrate its
  error into something unpleasant. If the ESC can tell you it is saturated,
  listen.

## Measuring all of it in a car park

Every number above is identifiable with wheel encoders, an IMU and whatever
telemetry the ESC already logs.

**The line and the current limit, together, from one run.** Full throttle from
rest along a straight, logging wheel speed and longitudinal acceleration. Torque
at the wheel is `m a r`, so plotting `m a r` against wheel speed draws the
envelope directly: a flat portion at the current limit, then a falling straight
line. Fit the line for `T_stall` and `w_free` and read the limit off the flat.

**The internal resistance,** from the same run, if the ESC logs pack voltage and
pack current: plot voltage against current and the slope is `-R_i`. The
intercept is the open-circuit voltage at that state of charge.

**The open-circuit curve,** by letting the pack rest for a few minutes at
intervals through a discharge and reading the voltage. Slow, tedious and
completely reliable.

**The regen limit,** by braking hard from a known speed and differentiating.

One trap is worth more than the rest of this section put together. **Many ESCs
report phase current, not pack current, and they are wildly different numbers.**
An ESC is a switching converter: at low speed it runs a low duty cycle, so it
can pass 100 A through the motor while drawing 10 A from the battery. Torque
follows the phase current; sag, heat and state of charge follow the pack
current. If you fit an internal resistance against phase current you will get an
answer roughly ten times too small at low speed and it will not be wrong
consistently enough for you to notice.

## Limits of this model

- **The straight line is a first-order description of the motor.** Iron losses,
  commutation timing and inductance all bend it slightly, more so at high speed.
- **There is no temperature anywhere.** Copper resistance rises about 0.4% per
  kelvin, so a motor 60 K above ambient makes noticeably less torque for the
  same current, and a hot pack's internal resistance moves too. Everything above
  describes a cold car.
- **Open-circuit voltage is not linear in state of charge.** A lithium cell has
  a long flat plateau in the middle and knees at both ends. A straight line
  through it is a fair approximation over the working range and a bad one near
  empty.
- **Internal resistance is treated as a constant** and is not one: it rises as
  the pack drains and falls as it warms.
- **There is no low-voltage cutoff here.** A real ESC stops delivering drive
  below a configured cell voltage, abruptly, which is a much more dramatic end
  to a run than a curve that keeps shrinking.
- **Nothing here models the ESC's own control loop**, its startup behaviour, or
  the difference between sensored and sensorless commutation at low speed, which
  is where most of the practical misery at 1/10 scale actually lives.

> **In SlipX.** The double-track tier evaluates the curve, the current and regen
> caps, and the pack sag once per step at the entry state, so the available
> torque is a constant within the step. Why that is a legitimate approximation
> at a 1 kHz step, and what it costs, is in ADR-0031.

## What to take away

Torque falls linearly with speed, a current limit clips the top of that line
flat without touching the peak power, and the pack voltage scales both ends at
once so that power goes as voltage squared. A full-to-empty run costs about 38%
of your peak power, and regenerative braking gives you perhaps a fifth of the
deceleration your tyres could have supported.

If you remember one operational thing: **plan your braking points from the
motor, not from the tyres.** On a car with no brakes, that is where the limit
actually is.

---

Related: [speed and the g-g diagram](06-speed-and-the-gg-diagram.md) builds the
speed profile that this article's limits deform, and
[differentials](10-differentials.md) covers what happens to the torque once the
ESC has decided how much of it there is. Back to the [index](README.md).
