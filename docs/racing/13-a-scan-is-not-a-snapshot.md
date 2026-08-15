# A scan is not a snapshot

A spinning laser scanner hands you an array of ranges and an array of angles,
and every tool you are likely to use will draw them as a shape. That drawing
is a lie whenever the car is moving, and the size of the lie is bigger than
most people guess.

This article is about where that error comes from, how big it is at 1/10
scale, and what has to be true for you to correct it.

## What the sensor actually does

A 2D scanning LiDAR has one laser and one mirror. The mirror turns, and the
laser fires as it goes: one measurement, then a fraction of a degree of
rotation, then the next. A scan is not a picture taken all at once. It is a
sequence of measurements taken one at a time, spread evenly over a full
revolution of the mirror.

The scan rate is the rate at which the mirror completes a revolution. A
Hokuyo UST-10LX, which is the common choice in this class, spins at 40 Hz and
emits 1080 measurements over 270 degrees of arc. So the revolution takes
25 milliseconds, and the 1080 measurements are 23 microseconds apart.

If the sensor is bolted to something stationary, none of this matters. Every
ray leaves from the same place, and the array of ranges really is the shape of
the room.

## The error

Now put it on a car doing 7 m/s, which is quick but not unreasonable for a
1/10-scale car on a straight.

In the 25 milliseconds the mirror takes to come round, the car travels

    7 m/s * 0.025 s = 0.175 m

so the first ray of the scan and the last ray of the scan are fired from
points 17.5 cm apart. A 1/10-scale car is about 30 cm long and 20 cm wide.
The scan you are about to treat as a single observation was taken from two
positions nearly a car-width apart.

That is the translation term, and it is the smaller one.

## Rotation is worse, and it gets worse with range

Take the same car through a 3 m radius corner at 7 m/s. Its yaw rate is

    r = v / R = 7 / 3 = 2.33 rad/s

and over one revolution of the mirror that is

    2.33 rad/s * 0.025 s = 0.058 rad, about 3.3 degrees

Three degrees sounds negligible. It is not, because a rotation of the sensor
moves a measured point by an amount proportional to how far away the point is.
A wall 5 m away is displaced by

    5 m * 0.058 rad = 0.29 m

and a wall 8 m down a straight by 0.47 m. The translation error is the same
17 cm wherever the wall is; the rotation error grows without limit as the
range does.

This is the asymmetry worth carrying away. **Translation distorts near
geometry, rotation distorts far geometry, and far geometry is what you were
planning against.**

## What it looks like

The characteristic symptoms, in rough order of how often they catch people
out:

- **Straight walls bend.** A corridor scanned while the car yaws comes back
  with a gentle curve in it, in the direction the car is turning. A
  wall-following controller reading a curve where there is a straight will
  steer to correct a curvature that does not exist.
- **The scan does not close.** On a full-circle unit, the first ray and the
  last ray point at nearly the same place and were taken 25 ms apart, so they
  disagree. Scan matching against a previous scan sees a seam.
- **Corners move.** A feature detected from a distorted scan is placed at the
  wrong angle, and the error changes with the car's speed and yaw rate, which
  means it changes with what the car was doing rather than with what is there.

## Correcting it

The correction has a name, deskewing, and the arithmetic is not the hard part.
If you know the pose of the sensor at the time of every individual ray, you
transform each measured point into a common frame and the distortion is gone.
The hard part is the "if".

To deskew you need three things, and none of them is free:

**A timestamp per ray, not per scan.** Most drivers give you one stamp for the
whole message. If all you have is that, the best you can do is assume the rays
are evenly spaced across the revolution and reconstruct the rest. That is
usually true and worth checking, because a driver that reports the time the
message was assembled rather than the time the mirror started is out by a
whole scan period.

**A pose estimate over the scan interval, not at a point in it.** You need
where the sensor was 25 milliseconds ago as well as where it is now, at a rate
fast enough to interpolate between. This is where wheel odometry and an IMU
earn their place: they run at hundreds of hertz, which the LiDAR does not.

**Consistency about which clock everything is on.** Deskewing with a pose
history that is itself late by an unknown amount replaces one error with a
different one. Sensor latency and sensor rate are separate quantities, and
neither implies the other.

## The thing people get wrong first

The tempting shortcut is to correct the scan using the car's velocity at the
moment the scan finished, applied uniformly. It is one line and it helps, and
it is wrong in a specific way: it assumes the velocity was constant over the
revolution, which is exactly false in the situation where the distortion is
largest, which is a corner entry.

Under braking into a corner, the yaw rate over a 25 ms window can change by
a good fraction of itself. Correcting with the final yaw rate over-corrects
the early rays, and correcting with the initial one under-corrects the late
ones. The correction is a rotation applied to a lever arm of several metres,
so a 20 per cent error in the rate is still centimetres at the far wall.

## Assumptions in the numbers above

- One revolution per scan, with rays evenly spaced in time. True for a
  mechanical spinning unit; solid-state and multi-line sensors distribute
  their measurements differently and the argument has to be redone.
- The sensor is at the centre of rotation. It is not: it is on a mast, usually
  ahead of the centre of mass, so the yaw rate also swings it through an arc
  and adds a translation term of its own.
- No suspension movement, which at this scale and on a flat surface is a
  reasonable simplification and stops being one outdoors.
- Ranges are exact. They are not, but the noise is of the order of
  centimetres and independent between rays, so it averages down; distortion
  is systematic and does not.

## Why this is a modelling decision and not a detail

It would be easy to treat this as a small correction to be applied afterwards.
It is better understood the other way round: the distorted scan is what the
sensor genuinely measured, and the undistorted one is a reconstruction that
depends on estimates you may not have. A stack tested only against
undistorted scans has never been tested against its own deskewing, and will
meet the real thing for the first time on a real car.

> **In SlipX.** Rays carry individual timestamps and each is cast from the
> emitter's pose at that ray's own time, so the distortion emerges from the
> motion the vehicle model produced rather than being added afterwards as a
> function of speed. A stationary car therefore produces an undistorted scan
> without anything special-casing standing still.

## Further reading

- Zhang and Singh, ["LOAM: Lidar Odometry and Mapping in Real-time"][loam],
  Robotics: Science and Systems, 2014. Section IV treats the distortion
  correction as part of the odometry rather than as preprocessing, which is
  the framing this article is arguing for.
- Hokuyo, [UST-10LX specification][hokuyo], for the scan rate and angular
  resolution used in the worked numbers.

[loam]: https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf
[hokuyo]: https://www.hokuyo-aut.jp/search/single.php?serial=167
