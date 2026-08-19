# Skidpad

**Identifies:** `c_alpha_f` and `c_alpha_r`, separately; a first look at
`mu_y0`.
**Needs first:** `mass`, `lf`, `lr` (bench). Localisation must be running:
this manoeuvre leans on the LiDAR pose.
**Space:** 8 m × 8 m for a 3 m radius circle with margin. Chalk or tape the
circle; driving a constant radius by eye adds scatter the fit then wears.
**Risk:** low until the top of the speed ladder, where the car starts to
slide wide. Runoff on the outside of the circle only.

## Why the pose makes both stiffnesses identifiable

The classic skidpad analysis fits the understeer gradient, which on a
near-balanced car is a small difference of large numbers: the reference car's
gradient works out to 3 × 10⁻⁴ rad per m/s² of lateral acceleration, a
steering correction of about a twentieth of a degree across the entire speed
ladder. Fitting
that alone would identify the *difference* of the axle stiffnesses and
barely constrain their sum.

The localisation pose breaks the degeneracy. With position and heading over
time, the body sideslip angle is observable (course angle minus heading),
and with it each axle's slip angle separately:

```
alpha_f = delta − atan((v_y + lf · r) / v_x)
alpha_r =       − atan((v_y − lr · r) / v_x)
```

In steady state on a circle, each axle's lateral force is also known without
touching the tyre model, straight from force balance:

```
F_f = m · a_y · lr / (lf + lr)        F_r = m · a_y · lf / (lf + lr)
```

Force over slip angle, axle by axle, is the cornering stiffness. On the
reference car at 2 m/s on a 3 m circle: `a_y` = 1.33 m/s², front axle force
2.33 N, front slip angle about 0.32°, and 2.33 N / 0.0055 rad recovers the
420 N/rad it was generated from. The whole linear range on this circle is
below about 3 m/s; that is where these numbers are trustworthy.

## Procedure

1. Drive the marked circle at the lowest speed the car can hold smoothly
   (about 1 m/s). Hold it for at least three full laps.
2. Step the speed up by 0.25 m/s and hold for three laps again. Continue
   until the car begins to run visibly wide of the circle, then stop the
   ladder: the top of it belongs to [circle-to-slip](circle-to-slip.md),
   which exists to probe the limit deliberately.
3. Repeat the whole ladder in the opposite direction. Left and right circles
   cancel IMU mounting misalignment and any left-right asymmetry in the car,
   and a systematic left-right disagreement is itself worth knowing about.

## What to record

The standard bag. The pose is the load-bearing signal; check before driving
that localisation is tracking (a pose that jumps re-localises mid-circle
poisons the sideslip estimate for that lap).

## What good data looks like

- Yaw rate constant within a few per cent over each held lap.
- Slip angles growing linearly with `a_y` at the bottom of the ladder and
  visibly faster than linearly at the top: the departure from the line is
  the tyre entering its nonlinear range, and it should appear near 60 to
  70 per cent of the eventual limit.
- Left and right circles giving the same stiffnesses after averaging.

## Failure modes

- **Radius drifting with speed.** Driving wide at higher speed moves `a_y`
  off the intended ladder. It does not bias the fit (each sample carries its
  own measured radius) but it compresses the sampled range; steer to the
  marked line.
- **Localisation degrades as yaw rate rises.** Sideslip inherits the error
  and the rear slip angle, which is small, inherits it worst. Visible as the
  rear stiffness estimate scattering while the front holds steady; slow the
  ladder down and check scan quality on this site.
- **Fitting the gradient anyway.** If the pose is not usable and the fit
  falls back to steering angle against speed, it identifies the difference
  of the stiffnesses, not the pair. The fitter reports the correlation; the
  answer is to fix localisation, not to trust the pair.
