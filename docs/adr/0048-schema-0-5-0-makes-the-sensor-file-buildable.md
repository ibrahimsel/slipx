# ADR-0048: schema 0.5.0 makes the sensor file carry what the sensor models consume

- **Status:** Proposed
- **Date recorded:** 2026-08-19
- **Requirements:** SCH-01, SCH-02, SENSE-02, SENSE-03
- **Related:** ADR-0047 (the sensor rig), ADR-0025 (the loader refuses
  rather than defaults), ADR-0041 (the 0.4.0 bump), ADR-0005 (an
  unimplemented tier throws)

## Context

`sensors.yaml` was designed at 0.1.0 to settle the shape of the car
directory before anything consumed it, and it says so in its own header. It
carries a name, a type, a rate, a phase, a latency and a dropout
probability, and it deliberately left the noise parameters as a free object
"because the fields differ per sensor type and slipx_sense has not yet
fixed them". slipx_sense has now fixed them, and ADR-0047's rig takes them
as plain structs; what is missing is the file format that fills those
structs. Two of the old file's choices no longer survive contact with the
implementation:

- The free `noise` object validates anything, so it can carry parameters no
  model reads, which is the ignored-field failure SCH-02 exists to prevent.
- The latency block's `jitter_stddev` names a distribution the models do
  not implement: transport jitter is uniform, bounded by the constant so
  that nothing is stamped before it was measured, and the LiDAR model has
  carried it that way since it was built. A field that asserts a standard
  deviation and is consumed as anything else is a lie in the file format.

The alternatives considered: keep the free noise object and have the loader
pick out the keys it recognises (rejected: an unrecognised key is a
parameter its author believed was in effect); default what the file does
not carry from the C++ spec defaults (rejected: ADR-0025, a defaulted noise
density is a fidelity claim nobody made); require the new fields at the
schema level (rejected: a migrated 0.4.0 file would become invalid, where
the house pattern since 0.2.0 is that migrated files stay valid and the
refusal happens, by name, at the point of use).

## Decision

Schema 0.5.0 restructures the sensor entry so that everything a sensor
model consumes is a named, typed field, optional at the schema level and
refused by name at build time.

- Each sensor entry may carry a block named after its type (`lidar_2d`,
  `imu`, `wheel_encoder`) holding that model's full parameter set, strictly
  validated (`additionalProperties: false`, every field required within the
  block: a partially specified sensor is a mistake, not a smaller sensor).
  The free `noise` object is gone.
- `latency.jitter_stddev` becomes `latency.jitter`, the half-width of the
  uniform jitter, bounded by `constant` at build time (JSON Schema cannot
  compare two fields, so the loader and the rig enforce the bound and name
  it). The 0.4.0 to
  0.5.0 migration converts the value by the square root of three, which
  preserves the variance the author stated; that is a reparameterisation of
  the same physical quantity, not an invented value. A migration cannot
  restructure a non-empty free `noise` object, because its keys were never
  defined, so it fails loudly and asks the author to restate it in the
  0.5.0 shape; an empty one is dropped, since it carried nothing.
- The loader side (`slipx.sensors_for`) maps validated entries onto the
  rig's spec structs and refuses, naming the sensor and the field, anything
  it cannot fill: a missing block, a missing latency, a `lidar_3d` entry
  (unbuilt until P4, and an unbuilt thing throws rather than substituting a
  simpler one, ADR-0005). `mount` stays unconsumed: the TF tree on the real
  car owns transforms, and the field is carried for it, not for the rig.

The reference car's `sensors.yaml` is rewritten at 0.5.0 with the full
parameter set, labelled provisional like everything else it carries: the
numbers are plausible for the hardware its comments name, and none of them
has been measured.

## Consequences

- Every 0.4.0 document of every kind migrates by identity except `sensors`,
  which migrates by the jitter conversion above; migrated files remain
  valid and lose nothing. A 0.4.0 sensors file that used the free `noise`
  object stops migrating and says why; no file in the tree or, to our
  knowledge, outside it did.
- A sensors file is now verbose: an IMU entry states nine numbers. That is
  the cost of ADR-0025 applied consistently, and it is the point: each of
  those nine is a claim someone can measure, and a file that omits one gets
  a refusal naming it instead of a simulation quietly running a sensor
  nobody described.
- The schema version moves to 0.5.0 in every `$id` and in
  `SCHEMA_VERSION`; the distribution version does not move (NFR-09, they
  are never compared).
- The C++ rig and the schema can drift only at the mapping in
  `slipx.sensors_for`, which is one function and is tested against the
  reference file; a field added to a spec struct without a schema field
  shows up there as an unfillable parameter, which is the loud version of
  the problem.
