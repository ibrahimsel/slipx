# Why simulated cars fall through walls

Every racing game player has seen it: a car brushes a fence at speed and is
suddenly on the other side, or halfway through a barrier and then spat out
the wrong way. It looks like a bug, and it is, but it is a *specific* bug
with two distinct mechanisms, and both come from the same awkward fact: a
simulated wall is usually a line with no thickness. This article works
through what hitting an unmovable object does, why thin walls are missable
at all, and the one rule that keeps them solid.

## A wall is a collision with something that cannot move

[Article 17](17-a-collision-is-an-impulse.md) worked the head-on crash of
two 3.5 kg cars through a single impulse and the reduced mass
`m* = (m₁ m₂)/(m₁ + m₂)`. A wall is the limit of that algebra as one mass
goes to infinity: `m*` tends to the car's own mass, and nothing else
changes. One 3.5 kg car into a wall at 4 m/s with restitution `e = 0.3`:

```
Jn = (1 + e) · m · (closing speed) = 1.3 · 3.5 · 4 = 18.2 N s
```

The car's speed changes by 18.2 / 3.5 = 5.2 m/s, from +4.0 to −1.2 m/s: it
leaves at 0.3 times its arrival speed, which is again the definition of
restitution. The wall's velocity change is `Jn` divided by the wall's mass,
and the wall is bolted to a building that is bolted to the planet, so the
honest divisor is about 6 × 10²⁴ kg and the answer is 3 × 10⁻²⁴ m/s. That
is why simulators do not model the wall's motion at all: immovability is
implemented by giving the wall an inverse mass of exactly zero, so no
finite impulse moves it, and the arithmetic above falls out of the
two-body formula without a special case.

So far, nothing can go wrong. The failures live in the geometry, not the
impulse.

## Thin things are missable

A simulation checks for contact once per step, against where the car is
*now*. Between checks the car simply teleports forward by (speed × step).
A wall drawn as a line has no interior, so the only thing the check can
catch is the car's footprint straddling the line at the instant of the
check. If one step's travel exceeds the footprint's reach, the car can be
entirely on the left at one check and entirely on the right at the next,
and no overlap ever existed to detect. This is called **tunnelling**.

The arithmetic decides whether it can happen to you. A 1/10-scale car at
4 m/s stepped at 1 kHz moves 4 mm per step; its rectangular footprint
reaches 150 mm sideways from its centre (a 300 mm-wide body). To jump the
wall unseen, one step would have to carry the centre from clear on one
side to clear on the other, more than 150 mm, which at a millisecond per
step means over 150 m/s. Not reachable; a kilohertz simulation of a small
car cannot tunnel through a wall its footprint is checked against.

Now run the same numbers at a game's 60 Hz with a full-size car. One frame
is 16.7 ms; 150 mm of reach is crossed at just 9 m/s, a parking-lot speed
for a vehicle doing 200 km/h in the game. That is exactly why full-scale,
low-rate simulators clip through thin fences, and why game engines grow
"continuous collision detection": swept tests that check the path travelled
rather than the pose arrived at. The assumption being bought there is
named: discrete detection is sound only while one step's travel stays
below the geometry's reach, and that inequality must be rechecked whenever
the step gets longer or the bodies get faster.

## Detected is not solved: a side must be chosen

Suppose the overlap *is* detected: the footprint straddles the line. The
resolver's job is to remove the overlap, and for a wall with no thickness
there are two ways out, one per side. Between two solid rectangles the
usual answer is the direction of least overlap, and it is fine there,
because pushing a mostly-left box further left is also pushing it toward
where its volume already is. Against a line that logic betrays you: the
cheapest way out is toward whichever side the car's centre currently
occupies, so the moment the centre crosses the line, "cheapest" flips, and
the resolver helpfully completes the escape by pushing the car out the far
side. The car did not tunnel; the collision system itself carried it
through. That is the squeezing-through failure, and contact between cars
can trigger it: a shove from an opponent advances the centre a little
further into the wall each step until it crosses.

![Two resolution rules against a thin wall](assets/wall-side-rule.svg)

The rule that holds is blunt: **always resolve toward the side the car's
centre is on**, whatever it costs, and never let the centre cross between
checks. The second half is the tunnelling inequality again, now applied to
the half-width instead of the full reach: if every step removes the whole
overlap, then for the centre to reach the line within one step the car
must travel its own half-width, 150 mm, in one millisecond. The rule and
the step size guard each other, and both halves are load-bearing.

## What this model leaves out

- **The wall's material.** Restitution carries the entire story of foam
  bumper against plywood or air duct, and nobody has measured it; the
  worked `e = 0.3` above is plausible, not identified. There is no dent,
  no scuff and no energy left in the barrier.
- **Open ends.** A barrier blocks what it geometrically covers. A polyline
  with a gap has a gap, and a car can drive around the last segment;
  nothing closes a wall that was not drawn closed.
- **Leaning on the wall.** An impulse model resolves events. A car
  scraping a wall through a corner is a train of small impulses and
  positional corrections, the same resting-contact caveat article 17
  states for cars.
- **Teleports.** The centre-side rule reads the car's pose now; it has no
  memory. A car *placed* across a wall by scenario setup belongs, as far
  as the resolver can tell, to the side it was placed on. The guard
  assumes motion is continuous between checks, and a teleport is not
  motion.

## In one paragraph

A wall is a collision with an object of infinite mass, which the impulse
algebra of article 17 handles by giving the wall zero inverse mass: the
car rebounds at `e` times its arrival speed and the wall absorbs the
momentum into the planet. The failures are geometric. A zero-thickness
wall can be tunnelled if one step's travel exceeds the footprint's reach
(150 m/s for a kilohertz 1/10 car; 9 m/s for a 60 Hz full-size one, which
is why games clip), and it can be squeezed through by any resolver that
pushes out the nearer side once the car's centre has crossed the line. The
fix is one rule with two halves: resolve to the centre's side, and keep
steps short enough that the centre can never cross between checks.

> **In SlipX.** Walls are polyline segments handed to the simulation as
> immovable contact geometry (ADR-0055), resolved through exactly the
> impulse of ADR-0043 with the wall's inverse mass at zero, and the
> penetration is always removed toward the side of the wall line the
> car's centre is on, never the minimum overlap. The raycaster, the
> occupancy map and the contact pass share one set of polylines, so the
> wall a LiDAR sees is the wall a bumper feels. Declaring no walls
> changes nothing, bit for bit, which is what keeps every wall-free
> trajectory and its published hash intact.

## Further reading

- [A collision is an impulse](17-a-collision-is-an-impulse.md), for the
  impulse, restitution and friction machinery this article takes as
  given.
- Christer Ericson, *Real-Time Collision Detection*, Morgan Kaufmann,
  2005. The standard reference; chapter 5 covers the swept and
  continuous tests that replace discrete detection when the tunnelling
  inequality fails.
- Erin Catto, [Continuous Collision](https://box2d.org/publications/)
  (GDC 2013), on how a production physics engine detects and resolves
  the fast-moving thin-geometry cases games actually hit.
