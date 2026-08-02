# 5. The racing line

A racetrack has width. What you do with it is the difference between a quick
lap and a slow one, and the reasoning is less obvious than it looks, because
the fastest way round is neither the shortest path nor the one with the highest
cornering speed.

**Before you start.** You need the friction limit from
[1. Tyres and grip](01-tyres-and-grip.md). Everything here treats the car as a
point mass with a maximum lateral acceleration, which is
[rung zero](03-vehicle-models.md) and is enough.

## The one fact everything follows from

A car cornering at radius `R` and speed `v` needs a lateral acceleration
`v²/R`, and the tyres can supply at most `mu g`. So:

```
v_max = sqrt(mu g R)
```

Speed goes as the **square root of the radius**. Double the radius and you gain
about forty per cent more speed through the corner.

The track's width is therefore not decoration. It is radius, available for the
taking, and radius is speed.

## Why the shortest path is not the fastest

Hug the inside kerb all the way round and you drive the shortest distance. You
also drive the tightest radius, so you are slow everywhere. The extra distance
of a wider line is bought back with interest, provided you use the width in the
right places.

![The geometric line and the late apex](assets/racing-line.svg)

The **geometric line** is the extreme of this trade: the single largest-radius
arc that fits between the boundaries, entering at the outside, touching the
inside at the middle of the corner, and exiting at the outside. Out, in, out.
It maximises the radius, so it maximises the constant speed you can hold
through the corner itself.

The point on the inside that the line touches is the **apex**. The geometric
line's apex is at the geometric middle of the corner, and a line whose apex is
further round is called a **late apex**.

If a corner were all that existed, the geometric line would be the answer. A
corner is never all that exists.

## Why the late apex usually wins

A racetrack is corners joined by straights, and you spend the straight
accelerating from whatever speed you left the corner with. Exit speed does not
just affect the exit: it sets your speed for the entire following straight, and
the time saved is roughly proportional to the length of that straight.

So there is a trade. Take a later apex and you:

- turn in later and use a **tighter** entry radius, so you must be slower on
  the way in;
- reach the inside later, and past the apex the required radius **opens out**,
  so you can start straightening the car sooner;
- get to full throttle earlier, and carry the benefit all the way down the
  straight.

You give up speed in a place that lasts a fraction of a second to gain speed in
a place that lasts several. Whenever the following straight is long enough, the
trade pays, and on most tracks it usually is.

Which gives the rule of thumb worth remembering:

> **The best line through a corner depends on what comes next, not on the
> corner.**

A corner leading onto the longest straight on the track deserves a very late
apex and a slow entry. A corner leading immediately into another corner might
be driven with a deliberately compromised line so that the *second* corner can
be taken well. The section of track a lap time is won in is often not the
section that feels fast.

## Corner sequences and compromise

Real tracks put corners in groups, and then the choice is not per corner.

- **Double apex.** A long corner where you touch the inside twice, staying off
  it in the middle. Usually faster than one arc, because a single arc through a
  long corner has a smaller radius than two joined ones.
- **Chicane.** Left then right in quick succession. The line is dominated by
  getting the car settled for the exit, and the entry to the first part is often
  driven well below its own limit.
- **The corner before the long straight.** Optimise this one and let the ones
  before it suffer if necessary. It is the highest-value corner on the lap by a
  wide margin.

The general form: the fastest lap is not the concatenation of the fastest
corners. It is a global optimisation, and greedy corner-by-corner reasoning
reliably gets it wrong.

## How this gets computed

You will meet three formulations. They optimise different things and the
differences matter.

**Shortest path.** Minimise arc length. Cheap, and mostly wrong for the reason
in the first section, but occasionally the right answer in a very tight,
low-speed section where the car never approaches the grip limit anyway.

**Minimum curvature.** Minimise the integral of curvature squared along the
path. Since `v_max` goes as `sqrt(R)`, minimising curvature maximises the
achievable speed pointwise, and it is a quadratic program: fast, convex, and
solvable over a whole track in seconds. It is the workhorse. Its blind spot is
that it optimises the path with no notion of where you are accelerating, so it
does not naturally produce the late apex.

**Minimum time.** Actually minimise lap time, with the path and the speed
profile optimised together subject to the vehicle model and the friction limit.
This is the real problem. It produces late apexes on its own, because it can
see the straight after the corner. It is also nonlinear, non-convex, needs a
vehicle model and a friction model to be meaningful, and takes minutes to hours
rather than seconds.

In practice: minimum curvature to get a good line quickly, minimum time when
the lap actually matters and you trust your parameters. A common compromise
weights the two, trading a little curvature for a better exit.

> **In SlipX.** Nothing in this article needs the library, and the library does
> not compute racing lines. The connection is upstream: a minimum-time line is
> only as good as the friction and tyre model underneath it, and a line computed
> against a guessed `mu` is a confident answer to a question nobody asked. This
> is the argument for identifiable parameters, arriving from the planning side.

## Where the point-mass picture breaks

Everything above assumes the car is a point that instantly achieves whatever
acceleration the friction limit allows. Real cars have limits it ignores.

- **The car takes time to rotate.** Yaw dynamics mean a path with a sharp
  curvature change is not drivable at speed even if every point on it is within
  the friction limit. Optimisers handle this by constraining curvature rate as
  well as curvature.
- **Load transfer.** [Article 2](02-load-transfer.md): the grip available
  depends on what the car is doing, so `mu g` is not a constant along the path.
- **Friction varies.** Surface, temperature, rubber laid down, dust in the
  corners of a sports hall. A line optimised for a uniform `mu` is a line for a
  track you are not on.
- **Track limits are a rule, not a wall.** What counts as leaving the track
  varies between rule sets, and the optimal line hugs whatever the rule
  actually is.

## In one paragraph

Cornering speed goes as the square root of radius, so track width is worth
using, and the shortest path is not the fastest. The geometric line maximises
radius through the corner; a late apex gives up some of that to straighten the
exit earlier and carry more speed down the following straight, which usually
pays. The best line therefore depends on what follows the corner, and finding it
properly is a global optimisation over the whole lap, most cheaply approximated
by minimising curvature and most correctly by minimising time.

## Further reading

- Heilmeier, Wischnewski, Hermansdorfer, Betz, Lienkamp and Lohmann, "Minimum
  curvature trajectory planning and control for an autonomous race car",
  *Vehicle System Dynamics*, 2020. The minimum-curvature formulation as actually
  used on a full-size autonomous racecar, and the accompanying code is at
  [TUMFTM/global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization),
  which is the fastest way to see a real implementation.
- Kelly and Sharp, "Time-optimal control of the race car: a numerical method to
  emulate the ideal driver", *Vehicle System Dynamics*, 2010. The minimum-time
  problem posed properly.
- Lot and Biral, "A curvilinear abscissa approach for the lap time optimization
  of racing vehicles", *IFAC World Congress*, 2014. The curvilinear coordinate
  formulation that most modern solvers use.
- Betz et al., "Autonomous Vehicles on the Edge: A Survey on Autonomous Vehicle
  Racing", *IEEE Open Journal of Intelligent Transportation Systems*, 2022,
  section on global planning, for how these fit together in a full stack.

---

Previous: [4. Understeer and oversteer](04-understeer-and-oversteer.md) ·
Next: [6. Speed and the g-g diagram](06-speed-and-the-gg-diagram.md) ·
[Series index](README.md)
