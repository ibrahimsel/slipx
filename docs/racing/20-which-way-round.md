# Which way round?

Nothing in a track's geometry makes one direction special. A loop of tarmac
can be lapped clockwise or anticlockwise, and both are equally valid races;
some venues run both in one weekend. So the direction of a race is not a
fact you can measure off the track. It is a *convention, announced*: race
control declares it, the grid is set up facing it, and everything downstream
(lap counting, overtaking rules, marshals waving at a car going the wrong
way) presumes everyone heard. This article works through how a race states
its direction, how progress along a track is measured so that direction has
a sign, and how a wrong-way car is caught without ever being able to sense
"wrongness" directly.

## Progress is an arc length, not a lap count

The natural coordinate along a track is arc length: pick a start point and a
direction, and every point on the centreline is "s metres along". This is
the one-dimensional part of the track-aligned coordinates (often called
[Frenet coordinates](https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas))
that most racing software plans in. Take a stadium-shaped circuit: two 8 m
straights joined by two semicircular ends of 3 m radius. One lap is

```
L = 2 · 8 + 2 · π · 3 = 16 + 18.85 = 34.85 m
```

A car's progress is measured by projecting its position onto the centreline
(finding the nearest centreline point) and reading off that point's s. The
key move is to work with *differences*: at each update, how far did s move
since last time, with its sign? Summing those signed steps gives a running
distance driven along the track, and the lap count is just that total
divided by L and rounded down.

Why not simply count crossings of the start line? Because line crossing is a
one-bit event and cars are not one-bit objects. A car that crosses the line
sideways in a spin crosses it twice. A car that reverses over it gains a lap
it did not drive. A car nudged back and forth while parked on the line can
gain several. Summed signed progress gets all three right without special
cases: the spin nets to zero extra, the reverse subtracts, the wobble sums
to nothing.

One subtlety: at the start line, s jumps from 34.84 back to 0.01. The raw
difference says the car went backwards 34.83 m; the truth is it went forward
2 cm. The fix is to take the *shorter way round*: any step larger than half
a lap has wrapped, so add or subtract L until it is within ±L/2. That buys
an assumption worth naming: no single update may move the car more than half
a lap. At 100 updates per second, half of this lap is 17.4 m per update, or
1 740 m/s. Safe for any car; not safe for a teleport, which is why race
control treats "the marshals repositioned the car" as a separate operation
that resets the reference rather than being measured as motion.

## The sign of s is the race direction

Once progress is a signed number, direction stops being metaphysics. Racing
the announced way means s increases; driving against it means s decreases.
And reversing an entire race needs no new machinery at all: run the same
centreline in the opposite order. The venue does not change, so what must
survive a reversal is exactly the physical stuff:

- the start line stays where it is (s = 0 is the same paint both ways);
- the lap length is the same 34.85 m;
- the drivable corridor is the same corridor, though what was "1 m to the
  left" becomes "1 m to the right", because left and right belong to the
  direction of travel, not to the ground.

What flips is everything defined *by* the traversal: tangents point the
other way, the grid faces the other way, and a point 5 m along one way is
34.85 − 5 = 29.85 m along the other. If your map data orders the
centreline's points, that ordering *is* the direction, and a reversed race
is the same list walked backwards.

> **In SlipX.** This is exactly the mechanism: a track object can produce
> its own reversal (same manifest, same start point, widths swapped, arc
> length re-derived), and race control given `reversed` races that object.
> Nothing downstream carries a direction flag, which means nothing
> downstream can get the sign half right. The ROS bridge announces the
> direction the way race control does, by latching the ordered centreline
> once on a topic before the heat.

## Catching a wrong-way car

A camera-less referee cannot see "wrongness"; it can only see s. The naive
rule "rule the car the moment ds/dt < 0" is wrong twice over. A car
wobbling on the spot has ds/dt cross zero constantly and would be ruled
while parked. And a single ruling per sample floods the record: at 100
updates per second, one three-second excursion is three hundred rulings.

The robust judgment uses a *high-water mark with hysteresis*:

1. Track the furthest progress the car has ever reached, `s_max`.
2. If `s_max − s` exceeds a threshold `d`, rule "wrong way", once.
3. Do not rule again until the car has regained `s_max`, so one excursion
   is one ruling however long it lasts.

Worked through: threshold `d = 1 m`, car spun at `s_max = 52.0 m` of
cumulative progress.

```
progress 52.0 → spin → 51.4   deficit 0.6   no ruling (below d)
                       51.0   deficit 1.0   no ruling (not yet past d)
                       50.9   deficit 1.1   RULED, once
                       47.0   deficit 5.0   still the same excursion
car turns round        52.0   deficit 0.0   re-armed
                       50.8   deficit 1.2   a new excursion, a new ruling
```

The parked wobbler never accumulates a metre of deficit and is never ruled.
The threshold choice is a trade: too small and localisation noise rules on
honest cars (a projection can jitter by centimetres where the centreline's
straight-line segments cut a corner), too large and a car can reverse half
a lap with impunity. One metre on a 35 m lap is about 3 % of a lap, several
car lengths of a 1/10-scale car, and far above centimetre-scale noise;
scale it with the track and with how noisy your localisation is.

Two operations must *reset* the reference instead of feeding it. A restart
or set-back that moves the car backwards along the track is the referee's
doing, not the car's driving, so `s_max` is rebased to wherever the car was
put. And the first measurement after any reset seeds the mark without being
judged, because a deficit needs two honest samples.

> **In SlipX.** Race control implements exactly this monitor and emits a
> `wrong_way` event carrying the deficit at the moment of ruling. It
> records and does not penalise: the competition ruleset it mechanises has
> no wrong-way rule, and inventing a penalty would be refereeing by a rule
> nobody agreed to.

## What a car can and cannot know

Note what none of this requires: the car's cooperation. Direction is
announced, measured and judged entirely from outside. That matches the
sensing reality: a LiDAR scan of a corridor looks the same driven either
way, so a scan-only driver *cannot* sense the race direction. It knows the
direction the way a human driver knows it, because it was placed on the
grid facing the right way, and it keeps knowing it only as long as it does
not spin. A driver that wants to survive a spin needs either odometry (its
accumulated heading change flips by half a turn relative to the track's
curl) or the announced map. That is a design fact, not a simulation
artefact: real autonomous racing stacks receive their direction from
configuration, not from perception.

## Limitations

The projection story assumes the centreline is a well-behaved single loop:
on a figure-of-eight or a pinched track, two stretches of road pass near
each other and a nearest-point projection can jump between them, taking
apparent progress with it. The half-lap wrap rule assumes updates are
frequent relative to speed, which holds for cars and fails for teleports;
and the wrong-way monitor judges progress only, so a car moving along the
announced direction in reverse gear is, to this referee, racing correctly.
Arguably it is: the referee's job here is direction, and a car that is slow
and backwards but pointed at the finish is a problem for its driver, not
for the rulebook.
