# The manoeuvre library

How every parameter in `VehicleParams` gets a number, using nothing but a car
park and the sensors already on a stock competition car: wheel encoders, an
IMU, and a pose from LiDAR localisation. No dyno, no tyre rig, no force
platform, ever. A parameter that cannot be identified this way is either
measured on the bench with household tools, taken from a datasheet, or does
not belong in the model.

This is the procedure library: what to drive, where, and what to record. The
concepts behind it (why a slip angle is fittable at all, what a residual
means) are in the [racing series](../racing/README.md), particularly
[Fitting a tyre model](../racing/07-fitting-a-tyre-model.md). The tooling
that consumes these recordings is `slipx_id`, which ingests a rosbag2
recording and emits a `dynamics.yaml` with per-parameter residuals and a
populated provenance block.

## The six manoeuvres

Run them in this order. Each one consumes numbers the previous ones produced,
and the order is chosen so that no fit has to estimate a parameter another
manoeuvre pins down better.

| | Manoeuvre | Identifies | Space |
|---|---|---|---|
| 1 | [Coastdown](coastdown.md) | `roll_resist`, `drag_coeff` | 40 m straight |
| 2 | [Straight-line acceleration](straight-line-acceleration.md) | `torque_stall`, `omega_free`, `current_max`, `c_kappa`, `mu_x0`, `v_max` | 60 m straight |
| 3 | [Skidpad](skidpad.md) | `c_alpha_f`, `c_alpha_r`, first look at `mu_y0` | 8 m × 8 m |
| 4 | [Ramp steer](ramp-steer.md) | `shape_c`, `curvature_e`, `mu_y0`, cross-checks `c_alpha` | 15 m × 15 m |
| 5 | [Step steer](step-steer.md) | `izz`, `relax_length`, `steer_bandwidth`, `steer_damping` | 20 m × 10 m |
| 6 | [Circle-to-slip](circle-to-slip.md) | `mu_y0` (definitive), `k_mu` with ballast, balance | 10 m × 10 m |

## Before you drive: the bench measurements

These are measured, not identified, and their provenance label is `measured`.
Household tools are within the rules; a rig is not.

| Parameter | Method |
|---|---|
| `mass` | Kitchen scales. Include the battery and everything the car races with. |
| `lf`, `lr` | Balance the car on a rod under the chassis; the balance point is the CoG. Alternatively weigh each axle: `lf = wheelbase · (rear axle weight / total)`. |
| `track_front`, `track_rear` | Tape measure, tyre centre to tyre centre. |
| `wheel_radius` | Roll the car one wheel revolution along a tape measure and divide by 2π. This is the loaded rolling radius, which is the one the model wants, so measure it with the car's weight on the wheel. |
| `h_cog` | Weigh the front axle twice: once level, once with the rear axle raised on a block of known height. The weight shift gives the CoG height; the formula and a worked example are in [Load transfer](../racing/02-load-transfer.md). |
| `steer_max` | Full lock against a protractor printed on paper, or from the geometry of the turning circle: drive the tightest possible circle slowly and take `wheelbase / radius`. |
| `izz` | Bifilar pendulum: hang the car level from two parallel strings of length `L` a distance `d` apart, twist it a few degrees, and time the oscillation with the car's own gyro. `izz = m·g·d²·T² / (16·π²·L)`. Strings are household tools. |
| `pack_v_full`, `pack_v_empty`, `pack_nominal_v` | Multimeter at full charge and at storage-empty; nominal from the cell chemistry (3.7 V per cell for LiPo). |
| `pack_capacity_ah` | The label on the pack. |
| `pack_internal_resistance` | Most hobby chargers display it. Otherwise: multimeter voltage at rest and under a known load. |
| `lsd_preload` | Hold one wheel, turn the other with a spring scale on a known lever arm; the torque at which it starts to slip is the preload. Only meaningful when the car has an LSD. |
| `layout`, `differential` | Read off the car. These are facts, not parameters. |

## Coverage: where every parameter comes from

Every field of [`VehicleParams`](../reference/vehicle-params.md), accounted
for. A parameter with no credible source from a car park does not get a
guessed number; it keeps the `provisional` label and the file says so.

| Parameter | Source | Label it earns |
|---|---|---|
| `mass`, `lf`, `lr`, `track_*`, `wheel_radius`, `h_cog`, `steer_max`, `izz` | Bench | `measured` |
| `ixx`, `iyy` | Nothing below L3 reads them. Leave the defaults. | `provisional` |
| `roll_resist`, `drag_coeff` | Coastdown | `identified` |
| `torque_stall`, `omega_free`, `current_max` | Straight-line acceleration | `identified` |
| `torque_per_amp` | Motor datasheet Kv and gear ratio, restated at the wheel. | `provisional` |
| `drive_efficiency` | Not observable without electrical power measurement. | `provisional` |
| `regen_current_max` | ESC configuration software states it directly. | `measured` |
| `v_max` | Straight-line acceleration (terminal speed), or the ESC curve extrapolated when the straight is too short, and the file says which. | `identified` |
| `c_kappa` | Straight-line acceleration, launch phase | `identified` |
| `mu_x0` | Straight-line acceleration, traction-limited launch | `identified` |
| `c_alpha_f`, `c_alpha_r` | Skidpad, linear range | `identified` |
| `mu_y0` | Circle-to-slip; ramp steer and skidpad cross-check it | `identified` |
| `shape_c`, `curvature_e` | Ramp steer | `identified` |
| `k_mu` | Circle-to-slip with and without ballast | `identified` |
| `relax_length` | Step steer | `identified` |
| `steer_bandwidth`, `steer_damping` | Step steer, small amplitude | `identified` |
| `steer_rate_max` | Servo datasheet transit time (`60°` in the quoted seconds, in rad/s at the road wheel through the linkage ratio) | `provisional` |
| `mu_clip` | Set it to the identified `mu_y0`. It is L1's clip, not a new fact about the car. | follows `mu_y0` |
| `pack_*` | Bench | `measured` |
| `lsd_preload` | Bench | `measured` |
| `provenance` | Written by the fitter, per the table above. Mixed sources make the set `identified` at best; a single `provisional` field does not poison the label, but the per-parameter labels travel in the file. | |
| `v_eps` | Numerical, not physical. Nobody identifies it. | |

## What to record

Record everything, every run, as one rosbag2 recording per manoeuvre:

- wheel encoder counts or speeds, per wheel, at the native rate;
- the IMU (specific force and angular rate), at the native rate;
- the localisation pose, with its timestamps;
- the commands the car was given (drive and steering), because a fit against
  inputs nobody logged is not a fit;
- the battery voltage if the ESC publishes telemetry. It is optional
  everywhere, and useful in the straight-line fit.

Timestamps matter more than rates. Every fit in `slipx_id` aligns signals by
timestamp, never by sample index, so a dropped message costs one sample, not
an offset in everything after it.

## Safety, once for the whole library

These cars reach 20 m/s, weigh 3 to 4 kg, and have no crumple zone: a
misjudged manoeuvre ends in a wall, a shin, or a written-off LiDAR. Every
procedure states its own space requirement; add margin for the failure case
you are testing towards, because several of these manoeuvres exist precisely
to find the limit of grip. Keep people out of the run-off direction, start
every sequence at the lowest speed, and treat the first breakaway as the
signal to stop raising the speed. Nothing in this library needs a slide
longer than a car length.

## Claim discipline

A parameter set produced by this library is `identified`, never `validated`.
Validation is a separate act: replay measured inputs through the fitted model
and compare trajectories, which `slipx_id` does for you (the report is part of
its output). "Validated" belongs to a set only when that report exists and is
attached, and the tooling prints the label either way.
