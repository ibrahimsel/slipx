# `VehicleParams`: every parameter, its units and its sign

The plain struct through which every parameter enters the core. There is no
parsing here, no file path, no version negotiation and no defaulting: the
boundary is a struct a caller can fill in by hand, which is what makes the
core embeddable. Parsing lives above, in `slipx_schema`.

Header: `slipx/params.hpp`. Python: `slipx.VehicleParams`, same field names.

**Units are SI throughout and sign conventions are ISO 8855**
([conventions](conventions.md)). The defaults below are the struct's own, and
they are `provisional`: plausible for a 1/10-scale competition chassis,
measured against no vehicle. The reference car file differs from them in
places, and says so.

**"From"** in the tables is the lowest tier at which the parameter changes the
trajectory. A parameter with "L2" has no effect at L0 or L1: not a small
effect, none. That is the teaching artefact rather than a bug: a student who
changes the CoG height at L1 and sees nothing happen has learnt what a
single-track model is.

## Mass and inertia

| Field | Units | Default | From | Notes |
|---|---|---|---|---|
| `mass` | kg | 3.5 | L0 | Total, sprung plus unsprung. |
| `izz` | kg·m² | 0.05 | L1 | Yaw inertia about the CoG z axis. |
| `ixx` | kg·m² | 0.02 | L3 | Roll inertia. Unused until L3 exists. |
| `iyy` | kg·m² | 0.06 | L3 | Pitch inertia. Unused until L3 exists. |

## Geometry

| Field | Units | Default | From | Notes |
|---|---|---|---|---|
| `lf` | m | 0.16 | L0 | CoG to front axle, positive forward. |
| `lr` | m | 0.16 | L0 | CoG to rear axle, positive rearward. |
| `track_front` | m | 0.24 | L2 | Front track width. |
| `track_rear` | m | 0.24 | L2 | Rear track width. |
| `h_cog` | m | 0.06 | L2 | CoG height above ground. Nothing below L2 can transfer load, so nothing below L2 can use it. |
| `wheel_radius` | m | 0.05 | L2 | Effective rolling radius. |

`wheelbase()` returns `lf + lr`. Weight distribution is `lr / wheelbase`
forward, so a car with `lf < lr` is nose-heavy.

## Tyres

| Field | Units | Default | From | Notes |
|---|---|---|---|---|
| `c_alpha_f` | N/rad | 420.0 | L1 | Front **axle** cornering stiffness, both tyres summed. Positive; the restoring sign lives in the force law. |
| `c_alpha_r` | N/rad | 455.0 | L1 | Rear axle cornering stiffness. |
| `mu_clip` | – | 1.1 | L1 | Peak friction, used at L1 only, to clip the linear tyre so a step steer cannot produce unbounded force. A clip, not a Magic Formula: L1's stated limitation is that it has no saturation shape, and clipping keeps that visible. |
| `tyre_front` | – | see below | L2 | MF-lite coefficients, per axle. |
| `tyre_rear` | – | see below | L2 | Usually the same tyre as the front, allowed not to be. |
| `c_kappa` | N per unit slip | 120.0 | L2 | Longitudinal slip stiffness per **tyre** at its static load. One value for all four: the manoeuvre that identifies it cannot separate the axles. |

Cornering stiffness is per axle because a single-track model has one tyre per
axle and splitting it would be a fiction. MF-lite coefficients are per axle to
match, and are per tyre once the model has resolved them.

### `TyreCoefficients`

| Field | Units | Default | Notes |
|---|---|---|---|
| `mu_y0` | – | 1.10 | Peak lateral friction at the nominal load. |
| `mu_x0` | – | 1.20 | Peak longitudinal friction at the nominal load. |
| `k_mu` | – | 0.15 | Load sensitivity exponent, positive: μ falls as load rises. |
| `relax_length` | m | 0.08 | Distance the tyre must roll before its lateral force reaches steady state. |
| `shape_c` | – | 1.68 | Magic Formula `C`. Must exceed 1 or the curve has no peak. |
| `curvature_e` | – | 0.42 | Magic Formula `E`. Above 1 the curve folds back on itself and stops being a tyre. |

The stiffness factor `B` is **not** a field: it is derived at construction from
the cornering stiffness and the static load, because `B` alone is not
measurable. `C` and `E` together decide how far out the peak sits, and they
are more sensitive than they look. Both points, with worked numbers, are in
[the tyre model derivation](tyre-model.md).

## Drivetrain

| Field | Units | Default | From | Notes |
|---|---|---|---|---|
| `accel_max` | m/s² | 8.0 | L0 | Peak forward acceleration. From L2 this bounds the *command*; the ESC decides what is delivered. |
| `decel_max` | m/s² | 12.0 | L0 | Peak braking deceleration, as a positive magnitude. |
| `v_max` | m/s | 20.0 | L0 | Top speed. |
| `layout` | – | `kRearWheelDrive` | L2 | `kRearWheelDrive`, `kFrontWheelDrive` or `kAllWheelDrive` (locked centre, 50/50). |
| `differential` | – | `kOpen` | L2 | `kOpen`, `kSpool` or `kLsd`. |
| `lsd_preload` | N·m | 0.0 | L2 | Locking preload across the axle. Consumed only when `differential` is `kLsd`. |

There is no brake bias parameter anywhere, and there will not be one: a
1/10-scale car brakes through its motor, so braking goes through the driven
axle only.

The struct default is an **open** differential, while the reference car file
says **spool**. That is deliberate on both sides: an open diff produces no
drive-induced yaw moment on a symmetric car, which keeps the struct defaults
agreeing with the single-track tiers at low lateral acceleration, whereas most
1/10 competition cars really do run a locked rear axle. The default describes
the neutral baseline; the file describes a real class of car.

## ESC

Stated at the **wheels** and at `pack_nominal_v`, so that no gear ratio and no
motor constant is a parameter. Used from L2; below L2 the acceleration limits
above are the whole model.

```
T_avail(ω) = torque_stall · s · (1 − ω / (omega_free · s)),   s = pack_v / pack_nominal_v
```

then capped by `torque_per_amp × current_max`. Negative (braking) torque is
capped by `torque_per_amp × regen_current_max`, and that regen cap is the only
brake the model has.

| Field | Units | Default | Notes |
|---|---|---|---|
| `torque_stall` | N·m | 2.0 | Total wheel torque at zero wheel speed, full throttle, before the current limit. |
| `omega_free` | rad/s | 480.0 | Wheel speed at which drive torque reaches zero. |
| `torque_per_amp` | N·m/A | 0.01 | Wheel torque per ampere of motor current. |
| `drive_efficiency` | – | 0.85 | Wheel power over battery-terminal power, in (0, 1]. Losses apply in both directions. |
| `current_max` | A | 120.0 | ESC drive current limit. |
| `regen_current_max` | A | 40.0 | Regen current limit. Its own number, and usually well below the drive limit. |

## Battery

Open-circuit voltage is linear in state of charge; internal resistance
produces sag under load. Used from L2. Setting
`pack_v_full = pack_v_empty = pack_nominal_v` with zero internal resistance is
the ideal-supply configuration, and is valid.

| Field | Units | Default | Notes |
|---|---|---|---|
| `pack_nominal_v` | V | 11.1 | The voltage the ESC curve is stated at. 3S LiPo nominal. |
| `pack_v_full` | V | 12.6 | Open-circuit voltage at `soc = 1`. |
| `pack_v_empty` | V | 9.9 | Open-circuit voltage at `soc = 0`. |
| `pack_capacity_ah` | A·h | 5.2 | |
| `pack_internal_resistance` | Ω | 0.020 | |

## Steering

The road wheel angle follows the command instantaneously at L0 and L1, clipped
to travel. From L2 the servo is a slew-limited second-order lag, and `steer`
and `steer_rate` become integrated state.

| Field | Units | Default | From | Notes |
|---|---|---|---|---|
| `steer_max` | rad | 0.40 | L0 | Road wheel travel, symmetric, as a positive magnitude. Positive command is left. |
| `steer_rate_max` | rad/s | 10.0 | L2 | Servo slew limit. |
| `steer_bandwidth` | rad/s | 45.0 | L2 | Second-order natural frequency. |
| `steer_damping` | – | 0.7 | L2 | Damping ratio. Below 1 the servo overshoots, which is physics rather than a bug. |

## Resistance

Both act against the direction of travel.

| Field | Units | Default | From | Notes |
|---|---|---|---|---|
| `drag_coeff` | kg/m | 0.015 | L0 | `0.5·ρ·Cd·A`. |
| `roll_resist` | – | 0.015 | L0 | Rolling resistance coefficient. |

Aerodynamic drag is close to negligible at 1/10 scale below about 15 m/s, and
a modelled aero map is out of scope. A drag term is here anyway because
coastdown is one of the identification manoeuvres, and a coastdown fit without
a drag term fits the rolling resistance wrong.

## Provenance and numerics

| Field | Units | Default | Notes |
|---|---|---|---|
| `provenance` | – | `kProvisional` | `kProvisional`, `kIdentified` or `kMeasured`. Carried into the core so tooling can print it rather than merely documenting it. |
| `v_eps` | m/s | 0.5 | Numerical floor for the longitudinal speed in slip-angle denominators. |

`v_eps` is **not** a physical parameter. It is the documented mitigation for
the single-track model's singularity at standstill, and it is exposed rather
than hidden so that it appears in the run manifest and so that a caller who
changes it knows they have changed the model.

`provenance` defaults to provisional because the defaults in this struct are
provisional. Nothing in SlipX may present them as anything else, and
`Car.summary()` prints the label on every load.

## Validation

```cpp
const char* problem = slipx::validate(params);   // nullptr if usable
```

Returns a static message naming the offending field, or `nullptr`. This is a
sanity check on physical impossibility (negative mass, zero wheelbase) and
**not** schema validation. Range checking, dimensional legality and inertia
plausibility live in `slipx_schema`, which the core is not allowed to know
about. The returned string is a literal, so this allocates nothing and works
with `-fno-exceptions`.
