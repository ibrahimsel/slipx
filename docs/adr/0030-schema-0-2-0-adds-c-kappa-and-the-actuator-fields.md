# ADR-0030: Schema 0.2.0 adds `c_kappa` and the actuator fields in one bump

- **Status:** Accepted
- **Date recorded:** 2026-08-12
- **Requirements:** SCH-01, SCH-02, CORE-08, CORE-09, CORE-10, CORE-11
- **Related:** [ADR-0009](0009-mf-lite-over-full-pacejka.md),
  [ADR-0011](0011-schema-refuses-a-newer-minor.md),
  [ADR-0023](0023-mf-lite-derives-b-from-cornering-stiffness.md),
  [ADR-0025](0025-c-kappa-enters-the-core-ahead-of-the-schema.md)

## Context

Two forces meet at the same version number. ADR-0025 put the longitudinal
slip stiffness `c_kappa` into the core ahead of the schema, and the loader's
refusal message has been promising "schema 0.2.0 adds it" ever since. And the
next core slice (ESC, battery, steering servo, differential) will need every
one of its parameters to arrive from a car file, because a field the loader
cannot fill must produce a refusal that names it, never a default.

A parser refuses a file with a newer minor (ADR-0011), so every minor bump
obsoletes installed parsers for newly written files. Bumping once for
`c_kappa` and again weeks later for the actuator fields would spend that cost
twice. The audit below was made so the bump happens once.

The audit of what the drivetrain slice needs, against what 0.1.0 already has:

- **Steering servo**: slew limit, natural frequency, damping. Already in
  `limits.schema.json` as `steering.max_rate`, `steering.bandwidth`,
  `steering.damping`, written at 0.1.0 precisely so that this day would not
  need them added. Nothing to do.
- **Differential and layout**: `drivetrain.layout` (2WD front or rear, 4WD),
  `drivetrain.differential` (spool, open, lsd), `lsd_preload` with a
  conditional requirement. Already in `dynamics.schema.json`. Nothing to do.
- **Battery**: `pack_nominal_v`, `pack_capacity_ah`,
  `pack_internal_resistance` already exist in `limits.schema.json`. Missing:
  the open-circuit voltage at the ends of the charge range, without which
  state of charge cannot map to a voltage.
- **ESC**: `current_max` and `regen_current_max` exist, in amperes. Missing:
  the torque-speed curve itself, and a way to express an ampere limit as a
  torque the physics can apply.
- **Tyre**: `c_kappa` is missing entirely; ADR-0025 is the record of that.

An alternative parameterisation of the ESC was considered and rejected: the
electrical one, with motor velocity constant, winding resistance, gear ratio
and drivetrain efficiency as separate fields. It is how a motor datasheet
speaks, but three of its four numbers cannot be identified from driving the
car, and multiplying datasheet values together is exactly the guessing that
the identifiability bar (ADR-0009) exists to prevent. The mechanical
parameterisation below is measurable in one full-throttle straight-line run
with wheel encoders and an IMU: the force-speed line gives the curve, its
plateau gives the current limit's torque, and ESC telemetry gives the amperes
alongside.

## Decision

`slipx_schema` moves to 0.2.0. The 0.1.0 to 0.2.0 migration is the identity
for every document kind: every field below is optional at the schema level,
and a migrated file gains nothing it did not carry. What changes is what the
loader can fill, and what it refuses by name when the field is absent.

`tyre.schema.json`:

- `linear.c_kappa` (optional): longitudinal slip stiffness per tyre at the
  nominal load, positive, in newtons per unit slip ratio, bounded (0, 5000].
  Identifiable from encoder slip ratio against IMU longitudinal acceleration;
  the bound is set by the stiffest plausible 1/10-scale tyre, at roughly
  2.5 times the ceiling already accepted for cornering stiffness.
- `mf_lite.B` becomes optional. ADR-0023 derives `B` from cornering stiffness
  and never consumes it from a file, and a schema that requires a field
  nothing reads is asking contributors for a number in order to ignore it.
  When `B` is present it is cross-checked against the derived value and a
  disagreement is a named warning, because a silently ignored `B` is a
  parameter its author believed was in effect.
- A plausibility warning on the `C`/`E` pair. The two are bounded
  independently, but together they set where the tyre's peak sits relative to
  the slip angle at which the linear tyre would saturate, and legal values
  can put that multiple above 20 where a real tyre sits between 1.5 and 3.
  The loader warns when the multiple exceeds 4, and does not reject: the
  described curve exists, it is just probably not the tyre the author meant.
  Warn rather than refuse follows the standing error/warning division in
  `rules.py`; the threshold 4 leaves headroom above the honest band before
  objecting.

`limits.schema.json`:

- A new optional `esc` section, stated at the wheels and at `pack_nominal_v`
  so that no gear ratio or motor constant is needed:
  - `torque_stall` [N m]: total drive torque at zero wheel speed, full
    throttle, before the current limit; the zero-speed intercept of the
    measured force-speed line.
  - `omega_free` [rad/s]: the wheel speed at which drive torque reaches zero.
  - `torque_per_amp` [N m/A]: wheel torque per ampere of motor current. This
    is what turns `current_max` and `regen_current_max` into torque caps the
    physics can apply.
  - `efficiency` [-]: mechanical power at the wheels over electrical power at
    the battery terminals, for computing the battery current draw.

  Both curve numbers scale linearly with actual pack voltage in the model;
  that is DC motor behaviour and costs no additional field.
- `electrical.pack_v_full` and `electrical.pack_v_empty` [V] (optional): the
  open-circuit voltage at state of charge 1 and 0.

`dynamics.schema.json`, `car.schema.json`, `sensors.schema.json` and
`provenance.schema.json` change version and nothing else.

The ESC and battery fields live in `limits.yaml`, not `dynamics.yaml`,
although an early layout note put "ESC, servo" in the latter. The tree's own
argument decides it: `limits.yaml` exists so that swapping a servo or a
battery changes one file, and a motor or ESC swap is the same kind of event.
The servo fields were already there.

## Consequences

A parser at 0.1.0 refuses a 0.2.0 file, by ADR-0011, so files written against
0.2.0 require upgrading slipx. That is the cost the single bump minimises.

A migrated 0.1.0 tyre file still has no `c_kappa`, still loads for L0 and L1,
and still gets the refusal naming the field for L2. The refusal's text stops
promising a future schema and starts naming the file's own gap.

Every new field is optional, so JSON Schema alone no longer states what L2
needs; the loader's refusal-by-name is the operative statement. That is
already true of `mf_lite` at 0.1.0 and is the price of migrations that cannot
invent values.

The amp-denominated `current_max` and `regen_current_max` stay authoritative
for what they measure. Without `torque_per_amp` they cannot reach the
physics, and the loader treats an ESC section that provides current limits
but no `torque_per_amp` as incomplete for L2, by name.

`nominal_load` remains optional, so the `B` cross-check and the load
sensitivity both remain unavailable for files that omit it; the check reports
that it could not run rather than pretending it passed.
