# Sign conventions and units

Sign errors and unit errors are the dominant bug class in vehicle dynamics
code. This page is normative, and every claim on it has a corresponding
assertion in `test_conventions.cpp`: if the tests and this page ever disagree,
the tests are right and one of them is a bug. The header is
`slipx/conventions.hpp`.

## Axes: ISO 8855

Vehicle body frame, origin at the centre of gravity:

| Axis | Direction |
|---|---|
| `x` | forward, along the longitudinal axis |
| `y` | to the **left** |
| `z` | up |

Right-handed. This is **not** the SAE frame, which has `y` to the right and
`z` down. A sign that looks wrong against a textbook is usually a textbook
using SAE.

World frame: `x`, `y` in the ground plane, `z` up. Yaw is measured from world
`x`, positive counter-clockwise viewed from above, so **a left turn increases
yaw**. Roll is positive right-side-down and pitch is positive nose-up, both by
the right-hand rule about the body `x` and `y` axes.

`VehicleState::rates` holds body-frame roll, pitch and yaw rate in rad/s,
positive about the corresponding body axis by the right-hand rule.

## Steering

The road wheel angle `δ` is **positive for a left turn**. Positive `δ`
therefore produces positive yaw rate and positive lateral acceleration in
steady state.

`DriveInput::steer_cmd` is a commanded **road wheel angle in radians**. It is
not a normalised stick position and not a servo pulse width; converting from
either is the caller's job, because the conversion is a property of the
hardware and not of the vehicle.

## Tyre slip: ISO 8855

For a wheel whose velocity in the wheel-carrier frame has longitudinal
component `v_lon` and lateral component `v_lat` (positive left):

```
α = atan2(v_lat, v_lon) − δ_wheel                    [rad]
```

so `α` is positive when the wheel's velocity vector lies to the **left** of
the wheel plane. The lateral force opposes it, which in the ISO frame makes it
negative:

```
Fy = −C_α · α        (linear region, C_α > 0)        [N]
```

**This minus sign is the single most common place to mix ISO up with SAE.**
Under SAE the slip angle carries the opposite sign and the same physics is
written `Fy = +C_α · α`. Both describe a restoring force. SlipX is ISO
throughout.

Slip ratio, with `ω` the wheel speed and `R_e` the effective rolling radius:

```
κ = (ω·R_e − v_lon) / max(|v_lon|, v_eps)            [-]
```

positive under drive, negative under braking. Used from L2; L0 and L1 have no
wheel states to compute it from.

## Wheel ordering

Every fixed-size per-wheel array in the library uses this order, without
exception:

| Index | Wheel |
|---|---|
| 0 | front left |
| 1 | front right |
| 2 | rear left |
| 3 | rear right |

Front and rear is by axle; left and right is by the ISO `y` axis, so "front
left" is the wheel a passenger facing forward would call front-left.

## Units

SI everywhere, without exception and without prefixes: metres, kilograms,
seconds, radians, newtons, newton-metres, kilogram square metres, volts,
amperes. Not degrees, not millimetres, not km/h.

Schema files may present friendlier units to a human, but the conversion
happens in `slipx_schema` and never crosses into the core. Every public
parameter names its unit in its doc comment.

State of charge is the one dimensionless quantity that could be mistaken for a
percentage: it is a **fraction in [0, 1]**.
