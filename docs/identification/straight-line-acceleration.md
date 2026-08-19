# Straight-line acceleration

**Identifies:** `torque_stall`, `omega_free`, `current_max`, `c_kappa`,
`mu_x0`, `v_max`.
**Needs first:** `mass`, `wheel_radius`, `h_cog`, `lf`, `lr` (bench);
`roll_resist`, `drag_coeff` (coastdown).
**Space:** a 60 m straight with generous overrun. This is the fastest thing
in the library.
**Risk:** the highest speed of any manoeuvre. Walk the full length first;
the car covers 60 m in about five seconds.

## The three regimes

Full throttle from rest, on the reference car, passes through three regimes,
and each one is a different parameter's moment:

1. **Launch, traction or current limited.** The current limit caps wheel
   torque at `torque_per_amp · current_max` = 0.01 · 120 = 1.2 N·m, which at
   `wheel_radius` 0.05 m asks the driven axle for 24 N. The rear axle can
   supply about 25 N once load transfer and load sensitivity are counted, so
   the launch runs at a
   large slip ratio just short of breakaway. This phase identifies `c_kappa`
   (from the slip ratio the force demands) and, if the tyre does let go,
   `mu_x0` (from the acceleration ceiling while it slips).
2. **The knee.** The torque-speed curve `torque_stall · (1 − ω/omega_free)`
   falls to the current cap at ω = 192 rad/s, about 9.6 m/s on the reference
   car. Below the knee acceleration is flat; above it acceleration tracks
   the curve. The knee's position and the slope above it identify
   `torque_stall`, `omega_free` and `current_max` together.
3. **Terminal.** Drive force meets rolling resistance plus drag; on the
   reference car 40·(1 − v/24) N meets 0.51 + 0.015·v² N at very nearly
   20 m/s. If the straight is long enough to reach it, that is `v_max`
   observed directly. If not, `v_max` comes from the fitted curve and the
   resistance fit, and the emitted file says it was extrapolated.

## Measuring slip without a fifth wheel

On a rear-wheel-drive car the front wheels are undriven, so the front
encoders read ground speed and the rear encoders read wheel speed, and the
slip ratio is right there:

```
kappa = (omega_rear · wheel_radius − v_ground) / v_ground
```

At launch the reference car's 12 N per driven tyre against `c_kappa`
120 N per unit slip predicts the rear encoders reading roughly 10 per cent
fast. That is far outside encoder noise, which is the reason this parameter
is identifiable at all. On a four-wheel-drive car there is no undriven axle;
ground speed comes from the differentiated localisation pose instead, which
is noisier, and the confidence interval on `c_kappa` will honestly say so.

## Procedure

1. From standstill, command full throttle. Hold the steering centred; a
   1/10-scale car at full launch torque will try to yaw if the surface is
   uneven.
2. Hold full throttle until the acceleration reads near zero (terminal
   speed) or the braking marker arrives, whichever is first.
3. Command zero torque, coast a moment, then brake gently to rest.
4. Repeat in both directions, at least three runs each way. If the tyre
   never visibly slipped at launch (slip ratio under a few per cent), repeat
   a pair of runs on a lower-grip patch of the same car park if one exists,
   or accept that `mu_x0` is not reachable on this surface and leave it
   `provisional`: the file must not carry a friction number the surface
   never demonstrated.

## What to record

The standard bag. Battery voltage telemetry, if the ESC publishes it, earns
its keep here: sag under the launch current is directly visible and tightens
the `pack_internal_resistance` cross-check, though the bench number remains
the one the file carries.

## What good data looks like

- A flat acceleration plateau of about 6.5 m/s² up to roughly 10 m/s, then a
  straight-line decay towards zero near 20 m/s (reference-car numbers).
- Rear wheel speed leading front wheel speed by 5 to 10 per cent during the
  plateau, converging to equal as the car stops accelerating.
- Runs in opposite directions agreeing once averaged.

## Failure modes

- **The launch bogs instead of slipping.** The current limit is set lower
  than the tyre can use; `mu_x0` is unobservable on this surface. The fit
  still gets the ESC parameters; the file keeps `mu_x0` provisional.
- **The straight ends before the knee.** `torque_stall` and `current_max`
  cannot be separated (the data never leaves the capped regime). The fitter
  will report the pair as correlated; find a longer straight or accept the
  cap as the only identified number.
- **Battery sag across the session.** Later runs launch measurably softer.
  Interleave directions and keep the session short, or the fit smears a
  drifting pack voltage into the torque curve.
