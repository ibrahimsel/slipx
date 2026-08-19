# Step steer

**Identifies:** `relax_length`, `steer_bandwidth`, `steer_damping`;
cross-checks `izz` (the bench pendulum measures it better).
**Needs first:** the skidpad's stiffnesses and the bench `izz`.
**Space:** 20 m × 10 m: a straight approach, then the arc the step settles
into.
**Risk:** moderate. Keep the step amplitude in the linear range; this
manoeuvre needs no sliding at all.

## Three delays, one response

Command a steering step while running straight and the yaw rate does not
step; it arrives late, three times over:

1. **The servo** takes the road wheel to the commanded angle: a slew phase,
   then a second-order settle. On the reference car a 0.15 rad step slews
   for 15 ms (`steer_rate_max` 10 rad/s) and settles in about 50 ms more
   (`steer_bandwidth` 45 rad/s, `steer_damping` 0.7).
2. **The tyres** build force only as they roll: `relax_length` 0.08 m is
   40 ms of delay at 2 m/s but 10 ms at 8 m/s.
3. **The chassis** integrates the force into yaw rate. At 1/10 scale this is
   the *fastest* of the three at low speed (single-digit milliseconds), not
   the slowest, which surprises people arriving from full-size practice.

A single step response is the sum of all three and fits none of them. The
separation comes from how each scales with speed: the servo's delay is
speed-independent, the tyre's shrinks as 1/v, and the chassis mode both
quickens and loses damping as speed rises, until at the top of the range the
yaw rate visibly overshoots and rings. On the reference car at 8 m/s that
ring sits near 7 Hz, well inside the gyro's bandwidth, and its frequency is
set by `izz`: that is the cross-check on the pendulum number.

So the procedure is not "a step steer"; it is the same step at several
speeds, and the fitter consumes the family.

## Procedure

1. Run straight at the first target speed (2 m/s) with the steering
   centred.
2. Command a step to a fixed amplitude chosen to stay linear: 0.10 to
   0.15 rad suits the reference car at low speed. Command it as a true step;
   the servo model is being identified, so do not ramp the command in
   software.
3. Hold the step until the yaw rate is clearly settled (a second is
   plenty), then straighten and slow down.
4. Repeat left and right, three of each, then move up the speed ladder:
   2, 4, 6, 8 m/s. **Shrink the amplitude as the speed rises** so the
   lateral acceleration stays comfortably inside the linear range; at 8 m/s
   even a 0.05 rad step is already a firm manoeuvre.

## What to record

The standard bag, and the exact timestamp of the step command matters more
here than anywhere else in the library: the whole measurement is the time
between that command and what the gyro then does. If the drive stack stamps
commands at publication rather than at actuation, note the stack's own
latency; it appears in the fit as an extra dead time, and it belongs in the
latency configuration, not inside the servo model.

## What good data looks like

- Yaw rate rising smoothly to a steady value: about 1.4 rad/s for a
  0.15 rad step at 3 m/s on the reference car.
- The rise time shrinking as speed rises, from around 100 ms at 2 m/s
  towards the servo-dominated floor.
- At the top of the ladder, a small overshoot and one or two visible
  oscillations before settling. No overshoot at any speed means the servo
  is slower than its datasheet or the step amplitude is saturating the
  slew limit for most of its travel.

## Failure modes

- **Amplitude too large.** The tyre leaves the linear range mid-transient
  and the response mixes saturation into what should be a lag measurement.
  Visible as a steady-state yaw rate below the linear prediction; halve the
  amplitude.
- **The software ramps the command.** Everything downstream then fits a lag
  that includes the ramp. The command trace in the bag exposes this
  immediately: it must be a step.
- **Only one speed.** The three delays cannot be separated and the fitter
  reports them fully correlated. Drive the ladder.
