# `VehicleState`, `DriveInput` and `StepDiagnostics`

What a step reads, what it writes, and which of it each tier can fill in.
Header: `slipx/state.hpp`. Python: the same names on `slipx`.

## One state type for every tier

`VehicleState` is one struct for all four tiers rather than a variant or a
per-tier type. Two reasons, both load-bearing:

1. It is trivially copyable and fixed size, so snapshot and restore is a
   `memcpy` and a replay buffer is an array.
2. A controller written against L1 can be pointed at L2 without changing a
   line. If the state type changed with the tier, that experiment would need a
   code change and nobody would ever run it.

The cost is that a lower tier leaves fields it cannot represent untouched.
That cost is documented per field rather than hidden.

## `VehicleState`

| Field | Units | From | Notes |
|---|---|---|---|
| `pos` | m | L0 | World position of the CoG. |
| `yaw` | rad | L0 | Heading, positive counter-clockwise. |
| `pitch` | rad | L3 | Positive nose-up. |
| `roll` | rad | L3 | Positive right-side-down. |
| `vel_body` | m/s | L0 | Body-frame velocity: x forward, y left, z up. |
| `rates` | rad/s | L0 | Body roll, pitch and yaw rate. Only `z` is a real degree of freedom below L3. |
| `omega_w[4]` | rad/s | L2 | Wheel speeds, positive forward. Zero at L0 and L1, which have no wheel states. |
| `steer` | rad | L0 | **Achieved** road wheel angle, positive left. Equals the command at L0 and L1, which have no servo. |
| `steer_rate` | rad/s | L2 | |
| `soc` | – | L2 | Battery state of charge, a fraction in [0, 1]. |
| `pack_v` | V | L2 | Pack terminal voltage. |
| `Fz[4]` | N | L2 | Vertical tyre loads. Below L2 there is no load transfer to report. |
| `alpha_lag[4]` | rad | L2 | Lagged slip angle per wheel: the slip the carcass has actually built up, trailing the slip the geometry asks for by the relaxation length. |

Accessors, so calling code need not remember that `rates.z` is the yaw rate:
`yaw_rate`, `vx`, `vy`, `speed()`, `sideslip()`. In Python `yaw_rate`, `vx`
and `vy` are read-only properties; `speed()` and `sideslip()` are methods.

`sideslip()` is the body slip angle: the wedge between where the car points
and where it is actually going, positive when the velocity vector is to the
left of the vehicle `x` axis.

### Why `alpha_lag` is zero rather than NaN below L2

The NaN rule below applies to **reported** quantities, where a plausible zero
would be believed. `alpha_lag` is hashed state, and the trajectory hash treats
a NaN as evidence the run is already broken, so a tier that parked NaN there
would poison every hash it produced.

## `DriveInput`

Commanded, pre-actuator: what the controller asked for, not what the car
achieved.

| Field | Units | Notes |
|---|---|---|
| `steer_cmd` | rad | Road wheel angle, positive left. |
| `accel_cmd` | m/s² | Longitudinal acceleration demand, positive forward. At L0 and L1 it is applied directly, subject to `accel_max` and `decel_max`. From L2 it becomes a torque demand through the ESC model. |

From L2, the gap between `DriveInput::steer_cmd` and `VehicleState::steer` is
the servo, and it is visible on purpose.

## `StepDiagnostics`

Optional. Passing `nullptr` costs nothing, so the hot path stays cheap;
passing a pointer gives the numbers needed to plot exactly why the car spun.

### The NaN rule

**A quantity the tier cannot represent is set to NaN, never to zero.** Zero is
a plausible slip angle and would be silently believed. NaN is loud, and a plot
of L0 slip angles is empty, which is the correct answer to the question.

Sinks carry the rule through: an absent quantity produces no line and no
legend entry, never a flat trace at zero.

| Field | Units | From | Notes |
|---|---|---|---|
| `alpha[4]` | rad | L2 | Per-wheel slip angle, ISO sign. |
| `kappa[4]` | – | L2 | Per-wheel slip ratio. |
| `fx[4]` | N | L2 | Longitudinal tyre force. |
| `fy[4]` | N | L2 | Lateral tyre force. |
| `fz[4]` | N | L2 | Vertical tyre load. |
| `tyre_saturated[4]` | bool | L1 | Per-wheel saturation. A `bool` has no NaN, so at the single-track tiers both wheels of an axle carry that axle's flag, while the float arrays above stay NaN rather than duplicating a value the tier never computed. |
| `alpha_front`, `alpha_rear` | rad | L1 | Axle-resolved, which is what a single-track tier can actually report. At L2 these are the axle sums. |
| `fy_front`, `fy_rear` | N | L1 | |
| `fz_front`, `fz_rear` | N | L1 | Axle vertical load. |
| `ax`, `ay` | m/s² | L0 | Specific forces at the CoG in the body frame. |
| `load_transfer_long` | N | L2 | Front-to-rear. |
| `load_transfer_lat` | N | L2 | Left-to-right. |
| `drive_torque` | N·m | L2 | Total wheel torque the ESC delivered after the curve, current and regen limits. |
| `pack_current` | A | L2 | Battery terminal current, positive discharging, negative charging under regen. |
| `steer_saturated` | bool | L0 | Command clipped to `steer_max`. |
| `accel_saturated` | bool | L0 | Demand clipped to `accel_max` or `decel_max`. |
| `speed_saturated` | bool | L0 | Demand clipped by `v_max`. |
| `esc_saturated` | bool | L2 | Torque demand clipped by the ESC curve, the current limit or the regen limit. Always false below L2. |
| `tier` | – | L0 | Which tier produced these numbers, so a plot cannot be mislabelled after the fact. An `int` in C++, a `Tier` in Python. |

### `ax` and `ay` are what an accelerometer reads

They include the `v_y·r` and `v_x·r` transport terms rather than being the raw
time derivatives of the body velocities. That is deliberate: they are the
quantity a validation report compares against a real IMU trace, and an IMU at
the CoG measures specific force.

### `drive_torque` is a demand, not an outcome

It is what the ESC delivered to the driveline. The tyres may put down less
than it implies once the friction ellipse has spoken; the per-wheel `fx` are
what they actually delivered.

## A paid-for trap

In Python, `sim.state(i)` and `sim.diagnostics(i)` return **references that
the next step overwrites**. Copy before keeping:

```python
history.append(sim.state(0).copy())   # not sim.state(0)
```

`VehicleState.copy()` is a `memcpy`, so this is cheap. Keeping the reference
instead gives a history whose every entry is the final state.
