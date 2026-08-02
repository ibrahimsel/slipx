# 2. Load transfer

A car's grip is not a fixed quantity. It changes continuously as you drive,
because braking, accelerating and cornering all move weight between the four
tyres, and the previous article established that unevenly loaded tyres make
less grip than evenly loaded ones.

This is the mechanism through which the physical layout of a car, where the
battery sits, how wide the wheels are set, how the mass is split front to
rear, finally reaches the trajectory. It is also the answer to the most common
beginner's question about small racecars: why does lowering the battery make
the car quicker?

**Before you start.** Read [1. Tyres and grip](01-tyres-and-grip.md), in
particular the load sensitivity section. You need moments about a point, which
is school mechanics.

## The mechanism

The inertial force of a decelerating or cornering car acts at its **centre of
gravity**, which is some height `h` above the road. The reaction that opposes
it comes from the tyres, which are on the road, at height zero.

Two equal and opposite forces separated by a distance make a couple. That
couple has to be balanced by something, and the only thing available is a
redistribution of the vertical loads on the tyres. That is all load transfer
is. It is not the springs, and it is not the body leaning: a go-kart with no
suspension at all transfers load exactly the same way, it just does not visibly
roll while doing it.

Say that again, because it is the part people get wrong: **the body rolling is
a symptom, not the cause.** Stiffening the springs changes how much the car
leans and how quickly the load arrives. It does not change how much load moves.

## Longitudinal: braking and accelerating

![Longitudinal load transfer under braking](assets/load-transfer-long.svg)

Take moments about the front contact patch. The car has mass `m`, wheelbase
`L`, and its CoG is at height `h` and at distance `l_f` behind the front axle
and `l_r` ahead of the rear one, with `l_f + l_r = L`.

At rest, the axle loads are set by where the CoG sits along the wheelbase:

```
Fz_front = m g l_r / L          Fz_rear = m g l_f / L
```

Note that the **front** load carries `l_r`. A CoG close to the front axle means
a small `l_f` and a large `l_r`, and the front carries most of the weight.
Getting this the wrong way round is the classic sign error in the formula.

Under a longitudinal acceleration `a_x`, positive forward, the inertial couple
`m a_x h` is balanced by moving load rearward:

```
dFz = m a_x h / L
Fz_front = m g l_r / L - dFz      Fz_rear = m g l_f / L + dFz
```

Accelerating loads the rear, braking loads the front. Only three things appear:
the mass, the CoG height, and the wheelbase. Not the springs, not the dampers,
not the tyres.

For a 3.5 kg car with `h = 0.06 m`, `L = 0.32 m` braking at 10 m/s², the
transfer is `3.5 * 10 * 0.06 / 0.32 = 6.6 N`, against a total weight of
34.3 N. Under braking that hard, the front axle carries about 23.7 N and the
rear about 10.6 N, so roughly seventy per cent of the car is on the front
tyres. Which is why the rear of a car gets light and nervous under heavy
braking, and why braking while already turning is a delicate business.

## Lateral: cornering

![Lateral load transfer in a corner](assets/load-transfer-lat.svg)

Exactly the same argument, rotated ninety degrees. The lever is still `h`, but
the base is now the **track width** `t`, the distance between the left and
right wheels, and the track is always much smaller than the wheelbase. Lateral
transfer is therefore the larger effect, usually by a factor of two or three.

```
dFz = m a_y h / t
```

with, in ISO 8855, a positive `a_y` meaning a left turn, which loads the
**right-hand** wheels. Cornering left puts the load on the right. It reads
backwards until you remember that the car's inertia pushes outward, and the
outside of a left turn is the right.

### Splitting it between the axles

The total is fixed by the equation above. How it divides between the front and
rear axles is a separate question and, on a car with suspension, it is set by
the roll stiffness distribution, which is what an anti-roll bar changes. That
is the main tool a race engineer has for adjusting the balance of a car, which
[article 4](04-understeer-and-oversteer.md) picks up.

On a car with no meaningful suspension, and most 1/10-scale chassis are close
to that, the split follows from a moment balance instead. In steady state, yaw
moment balance requires `l_f Fy_f = l_r Fy_r`, and the two axle forces sum to
`m a_y`, so:

```
Fy_front = m a_y l_r / L        Fy_rear = m a_y l_f / L
```

which is exactly the static weight split. Each axle transfers load in
proportion to the lateral force it is making:

```
dFz_front = (m l_r / L) a_y h / t_f      dFz_rear = (m l_f / L) a_y h / t_r
```

> **In SlipX.** This is the split the library uses, and the reason is
> identifiability rather than convenience: a roll stiffness distribution cannot
> be measured with wheel encoders and an IMU, whereas this one follows from a
> balance that was already going to hold and introduces no parameter at all.
> Written up in [ADR-0022](../adr/0022-load-transfer-is-quasi-static.md), along
> with what it costs.

## What it costs you

Now combine this with load sensitivity from the previous article, and the point
of the whole exercise appears.

A car cornering hard has its outside tyres heavily loaded and its inside tyres
barely loaded. The outside pair is deep into the load-sensitive region where
`mu` has dropped; the inside pair has plenty of `mu` and almost no vertical
load to apply it to. The **total** grip available to the axle is less than the
same car would have if the load were spread evenly.

So load transfer is not neutral bookkeeping. It is a loss. Everything a
chassis builder does about it is aimed at making it smaller:

- **Lower the centre of gravity.** `h` appears in both formulae, and it is
  usually the only one you can change substantially after the car is built. On
  a 1/10 car the battery is the heaviest single component and mounting it low
  is the cheapest handling improvement available.
- **Widen the track.** `t` is in the denominator of the lateral term.
- **Lengthen the wheelbase.** `L` is in the denominator of the longitudinal
  term. This trades against agility, so it is a real compromise rather than a
  free win.

## Wheel lift and the rollover threshold

Push the lateral transfer far enough and the inside wheels reach zero load.
Set the inner load to zero, with no longitudinal acceleration, and the
condition falls out remarkably cleanly:

```
a_y = g t / (2 h)
```

The mass cancels. So does the weight distribution. The **static stability
factor**, `t / 2h`, is a property of the chassis geometry alone, which is why
it is the number a chassis gets judged on and why it appears on vehicle safety
ratings for full-size cars.

For a 1/10 car with `t = 0.24 m` and `h = 0.06 m`, that is `9.81 * 0.24 /
0.12 = 19.6 m/s²`, or two g. Since the tyres will let go somewhere around 1.1 g
on most surfaces, the car slides long before it tips, which is the normal and
much preferable outcome. Raise the CoG to 0.12 m and the threshold halves to
one g, which is uncomfortably close to the grip limit: the car is now capable
of tripping over itself in a hard turn. Add a sticky surface, or a kerb to trip
on, and it will.

This is the monotonicity to keep in mind: **raising the centre of gravity
lowers the rollover threshold**, always, on every car, regardless of anything
else.

## What this model leaves out

The treatment above is **quasi-static**: it assumes the load arrives the
instant the acceleration does. A real car with suspension takes tens of
milliseconds, roughly the period of the roll mode, and during that window the
loads and therefore the grip are somewhere between the old and the new values.
Transient handling, which is what a driver feels on turn-in, lives almost
entirely in that window.

Modelling it needs a roll degree of freedom and spring and damper rates. That
is a real model with real parameters and it is a step up in complexity, which
is why most simulators, including this one, treat load transfer as
instantaneous and say so.

Also absent: aerodynamic downforce, which changes the total load rather than
redistributing it and is negligible at 1/10 scale below about 15 m/s; banking,
which changes the direction of the reaction; and the unsprung mass, which
transfers load through the wheels rather than through the body.

## In one paragraph

Load transfer happens because inertia acts at the centre of gravity and the
reaction acts at the road, and the two are separated by the CoG height. It
moves load front to rear over the wheelbase and side to side over the track,
which is smaller and therefore matters more. Because peak friction falls with
load, moving load about always costs total grip, so lower and wider is quicker.
Push it far enough and the inside wheels lift, at a lateral acceleration of
`g t / 2h` that depends on nothing but the chassis geometry.

## Further reading

- Milliken and Milliken, *Race Car Vehicle Dynamics*, SAE International, 1995,
  for load transfer including the roll stiffness distribution and what a race
  engineer does with it. Look for the chapters on ride and roll rates and on
  chassis set-up.
- Guiggiani, *The Science of Vehicle Dynamics*, 2nd ed., Springer, 2018,
  chapter 3. More mathematically careful than most, and explicit about which
  assumptions each formula rests on.
- Gillespie, *Fundamentals of Vehicle Dynamics*, SAE International, 1992,
  chapters 6 and 9, for the rollover threshold treated properly.

---

Previous: [1. Tyres and grip](01-tyres-and-grip.md) ·
Next: [3. Vehicle models](03-vehicle-models.md) ·
[Series index](README.md)
