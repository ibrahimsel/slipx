# 6. Speed and the g-g diagram

Given a path, how fast should the car go at each point along it, and where
should it start braking? This is the second half of the racing problem, and
unlike the racing line it has a clean and almost embarrassingly simple answer
that you can implement in twenty lines.

**Before you start.** [1. Tyres and grip](01-tyres-and-grip.md) for the friction
ellipse. [5. The racing line](05-the-racing-line.md) is useful context but not
required: this article takes the path as given.

## The g-g diagram

Take everything the car can do and plot it as longitudinal acceleration against
lateral acceleration. The result is called a g-g diagram, and it is the friction
ellipse from article 1 promoted from one tyre to the whole vehicle.

![The g-g diagram and one corner traced through it](assets/gg-diagram.svg)

The boundary is the envelope of what the car can achieve. Reading it:

- **Bottom** is braking, **top** is accelerating, **sides** are cornering.
- The boundary is close to elliptical wherever grip is the limit.
- The **top is flat**, because forward acceleration usually runs out of motor
  before it runs out of grip. A 1/10-scale car might brake at 1.2 g and
  accelerate at 0.8 g, so the shape is lopsided. This is not a modelling
  artefact, it is the drivetrain.
- On a full-size racecar with aerodynamics the whole diagram grows with speed,
  so it is really a stack of diagrams indexed by speed. At 1/10 scale, below
  about 15 m/s, downforce is negligible and one diagram does.

### Trail braking, and what the diagram makes obvious

Now trace what the car actually does through one corner. The blue path in the
figure: brake in a straight line, then release the brake progressively as
steering is added, reach the apex at pure cornering, then unwind the steering
as power is applied.

Compare that with the naive alternative: brake in a straight line, finish
braking, turn, corner, straighten, accelerate. That traces a **cross** through
the diagram, along the vertical axis and then the horizontal one, and leaves
the diagonal parts of the boundary entirely unused. Everything in the corners
of the plot is grip the car had and did not spend.

**Trail braking** is the technique of filling those in: overlapping the end of
the braking with the start of the cornering so the total demand stays on the
boundary throughout. Everything on the diagonal is real grip, so this is a real
gain, not a driving affectation. The same argument applies on the exit, where
the steering is unwound as power is fed in.

For an autonomous car this is a statement about the controller architecture. A
system whose longitudinal and lateral controllers are independent, one tracking
a speed profile and one tracking a path, has no representation of the shared
budget. It will naturally trace the cross. Getting the diagonals needs the two
axes coupled, which is one of the strongest arguments for model predictive
control here: the friction ellipse is a constraint MPC can hold directly.

## From curvature to a speed profile

Now the practical algorithm. You have a path, sampled at points along its
length `s`, with a curvature `kappa(s)` at each one. You want a speed at each.

![Building a speed profile with a forward and a backward pass](assets/speed-profile.svg)

### Step one: the grip limit

At each point, cornering alone bounds the speed:

```
v_grip(s) = sqrt(mu g / kappa(s))
```

capped at the car's top speed on the straights, where curvature is near zero.
This is the dashed line in the figure. It is not yet a speed profile, because it
is not **drivable**: it has jumps in it, and a car cannot change speed
instantaneously.

### Step two: the forward pass

Sweep from the start to the end. You cannot accelerate faster than the car
allows, so:

```
v[i] = min(v_grip[i], sqrt(v[i-1]^2 + 2 a_accel ds))
```

using `v² = u² + 2as`. This draws the ramp coming out of each corner, where the
car is accelerating hard but has not yet reached the speed the next section
would permit.

### Step three: the backward pass

Sweep from the end to the start. You must already be slow enough by the time
you arrive, so:

```
v[i] = min(v[i], sqrt(v[i+1]^2 + 2 a_brake ds))
```

This draws the braking zone, backwards from each corner into whatever precedes
it.

That is the whole algorithm. The answer is the pointwise minimum of the three,
and the two passes cannot be collapsed into one because acceleration and
braking constrain the profile from opposite directions in space.

Note what falls out of it: **a braking point is a property of the corner you
are approaching, not of where you happen to be.** A tighter corner needs a
longer braking zone and therefore an earlier braking point, even though the
corner itself is shorter. This is why "brake at the 50 metre board" is a
statement about a specific corner and does not transfer.

### Refining it: using the ellipse

The version above uses one number for acceleration and one for braking,
independent of how hard the car is cornering. That is the cross, again, and it
is conservative: it never lets the car brake and corner at once.

The refinement is to make the available longitudinal acceleration depend on the
lateral demand, straight off the g-g boundary:

```
a_x_available = a_x_max * sqrt(1 - (a_y / a_y_max)^2)
```

Substituted into both passes, this gives a profile that trail brakes. It also
makes each pass implicit, since `a_y` depends on the speed you are solving for,
so implementations either iterate a couple of times or use the previous
iteration's speed. Two iterations is usually enough.

## Where this gets optimistic

The algorithm is a point-mass argument and inherits every limitation of one.

**`mu` is not a constant.** It varies with surface, temperature and load, and
[article 2](02-load-transfer.md) showed that a hard-cornering car has less total
grip than the same car standing still. Profiles are usually computed with a
deliberately conservative `mu` for this reason. If your car cannot follow the
profile you computed, suspect this before suspecting the controller.

**The car takes time to rotate.** The profile assumes the required lateral
acceleration is available the moment the curvature demands it. Yaw dynamics and
the tyre relaxation length from [article 1](01-tyres-and-grip.md) mean it is
not, quite.

**Load transfer is invisible.** Braking hard into a corner unloads the rear
axle at exactly the moment the corner asks it to generate lateral force. A point
mass has no axles and cannot express that. It is a large part of why the entry
to a corner is where cars actually spin, and no amount of care with the speed
profile will show it to you.

**Discretisation matters.** With too coarse a sample spacing `ds`, the
sharpest curvature point can fall between samples and the profile will be
optimistic exactly where it matters most.

## In one paragraph

The g-g diagram shows every combination of longitudinal and lateral
acceleration a car can reach, and driving fast means staying on its boundary
rather than tracing a cross through the middle, which is what trail braking is
for. Turning a path into a speed profile is three steps: a grip limit from
curvature, a forward pass for the acceleration limit, and a backward pass for
the braking limit, taking the smallest at each point. Coupling the longitudinal
limit to the lateral demand through the friction ellipse turns the same
algorithm into one that trail brakes.

## Further reading

- Subosits and Gerdes, "From the Racetrack to the Road: Real-time Trajectory
  Replanning for Autonomous Driving", *IEEE Transactions on Intelligent
  Vehicles*, 2019, for speed profile generation done carefully, including the
  friction ellipse coupling.
- Velenis and Tsiotras, "Optimal Velocity Profile Generation for Given
  Acceleration Limits: Receding Horizon Implementation", *American Control
  Conference*, 2005. The forward-backward algorithm treated as an optimisation
  problem rather than a recipe, which is the reference to cite when someone asks
  whether the two-pass method is actually optimal.
- Milliken and Milliken, *Race Car Vehicle Dynamics*, SAE International, 1995,
  for the g-g diagram in its original context, including measured ones from
  real cars.
- The
  [TUMFTM/global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)
  repository implements exactly this alongside the racing line, and reading its
  velocity profile module is worth an hour.

---

Next: [7. Fitting a tyre model](07-fitting-a-tyre-model.md) ·
Previous: [5. The racing line](05-the-racing-line.md) ·
[Series index](README.md)
