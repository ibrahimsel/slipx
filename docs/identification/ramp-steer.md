# Ramp steer

**Identifies:** `shape_c`, `curvature_e`, `mu_y0`; cross-checks `c_alpha_f`
and `c_alpha_r`.
**Needs first:** the skidpad's stiffnesses, and localisation running.
**Space:** 15 m × 15 m. The car spirals inward from a wide arc; the space
bounds the starting radius.
**Risk:** moderate. The manoeuvre ends at or just past the grip peak, so it
ends in a slide by design. Keep the spiral's centre clear.

## What this adds over the skidpad

The skidpad samples the tyre curve at whatever slip angles its speed ladder
happens to produce, densely at the bottom and sparsely near the top. Ramp
steer sweeps the slip angle continuously through the whole working range in
one run: hold the speed constant and wind the steering in slowly, and the
car tightens its arc while the axle slip angles climb from zero through the
linear range, past the point where the curve bends, to the peak.

`shape_c` and `curvature_e` live in exactly that bend. On the reference
tyre the force is within a few per cent of linear until about 2° of slip,
peaks at 6.9°, and the two shape factors set how sharp the transition is
and how far beyond the linear extrapolation the peak sits. A fit fed only
skidpad data pins the ends of the curve and guesses the middle; this
manoeuvre is the middle.

## Quasi-static, or it measures the wrong thing

The sweep must be slow enough that every sample is effectively a steady
state, or relaxation and yaw transients contaminate the curve (they belong
to [step steer](step-steer.md), which measures them on purpose). Wind the
steering at no more than about 0.02 rad/s at the road wheel. At that rate a
sweep to full lock takes around 20 seconds, and each second of it moves the
operating point less than the tyre's own settling distance smears it.

## Procedure

1. Enter a wide arc at constant speed. 3 m/s suits the reference car: fast
   enough that full lock is past the grip peak, slow enough that the spiral
   fits the site.
2. Hold the speed with the drive command (the tightening arc adds tyre
   drag; the speed controller must make it up, and the logged drive command
   records that it did).
3. Wind the steering in at the fixed slow rate until either the front
   washes out (the arc stops tightening), the rear steps out, or full lock.
4. Unwind gently, straighten, and repeat in the other direction. Three
   sweeps each way is a solid session.

## What to record

The standard bag. The fit reconstructs, at every instant: axle slip angles
from the pose (as on the skidpad), axle lateral forces from `m · a_y`
partitioned by the static weight split, and traces out force against slip
angle per axle. That curve, both axles overlaid with the fitted MF-lite, is
the single most useful plot the whole library produces.

## What good data looks like

- A smooth force-against-slip curve that is straight, then bends, then
  flattens; the reference tyre's peak sits near 6.9° and loses only a few
  per cent by 10°.
- Both sweep directions producing the same curve mirrored.
- The speed trace flat through the sweep. A sagging entry speed stretches
  the slip-angle axis mid-sweep.

## Failure modes

- **Sweep too fast.** The curve shows hysteresis: the inbound and outbound
  branches disagree, because the tyre force lags the slip angle by its
  relaxation length. Halve the rate; the two branches must lie on top of
  each other before the shape factors mean anything.
- **Speed climbs as the driver compensates late.** `a_y` and slip angle
  both move and the curve smears. Fix the speed loop before fixing the
  tyre model.
- **The site ends the spiral before the peak.** `mu_y0` from this run is a
  lower bound only; the fitter labels it so, and circle-to-slip settles it.
