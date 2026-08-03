# ADR-0025: The longitudinal slip stiffness enters the core ahead of the schema

- **Status:** Accepted
- **Date recorded:** 2026-08-03 (decision taken during P1)
- **Requirements:** CORE-06, CORE-01, SCH-02, NFR-09
- **Related:** [ADR-0009](0009-mf-lite-over-full-pacejka.md),
  [ADR-0023](0023-mf-lite-derives-b-from-cornering-stiffness.md),
  [ADR-0005](0005-tiers-throw-rather-than-fall-back.md),
  [ADR-0011](0011-schema-refuses-a-newer-minor.md)

## Context

ADR-0023 anchored the lateral branch of MF-lite to a measurable number. The
stiffness factor is derived rather than read,

```
B = C_alpha / (C mu_y0 Fz_nom)
```

where `C_alpha` is the cornering stiffness, present in `tyre.schema.json` as
`linear.c_alpha` and described there as identifiable from the low-slip region of
a skidpad.

The longitudinal branch is the same formula in slip ratio and wants the same
treatment:

```
B_x = C_kappa / (C mu_x0 Fz_nom)
```

`C_kappa`, the slip stiffness, is the initial slope of longitudinal force
against slip ratio in newtons per unit slip. **`tyre.schema.json` at schema
0.1.0 has no field for it.** The file carries `mu_x0`, so the height of the
longitudinal curve is specified, and `linear` carries `c_alpha` and nothing
else, so the slope is not. There is a peak with no approach to it.

Schema 0.1.0 is published, in `0.1.0a1` on PyPI, so this is not an oversight
that can be edited away.

The awkward part is that `C_kappa` passes ADR-0009's own admission test
cleanly. Wheel encoders give slip ratio, the IMU gives longitudinal
acceleration, and a straight-line acceleration run gives the slope directly.
It is missing because nobody wrote the field, not because it fails the
criterion.

Five options were considered.

**Reuse the lateral `B` for the longitudinal branch.** Asserts that a tyre's
longitudinal and lateral stiffnesses are equal. They are not, typically by a
factor of one and a half or more. It is a physical claim smuggled in as code
reuse, and nothing in the output would say it had been made.

**Read `mf_lite.B` from the file for the longitudinal branch.** Same physical
objection, and it contradicts ADR-0023, which had just made `B` derived and
checked rather than consumed.

**Default `C_kappa` to a plausible number.** Precisely what SCH-02 forbids and
what ADR-0009 exists to prevent: a parameter nobody identified, guessed once and
trusted afterwards.

**Hold the longitudinal branch until schema 0.2.0 carries the parameter.**
Correct in the end state, and rejected on sequencing. It puts CORE-07 and the
assembly of the tier behind a schema bump with its own migration machinery,
version gating and SCH-04 consistency rules, for a parameter whose physics is
not in question.

**Add the field to the core now and open the schema bump as separate work.**
Chosen.

## Decision

`VehicleParams` gains a longitudinal slip stiffness, and MF-lite's longitudinal
branch derives its stiffness factor from it exactly as the lateral branch
derives `B` from the cornering stiffness. The schema is not changed here.

`VehicleParams` is the core's boundary, not the schema's. ADR-0003 is the reason
that distinction exists: parameters arrive as a plain struct and parsing lives
elsewhere, so the core is free to require a number the current schema cannot
supply. This is also the case where that freedom is worth something rather than
merely permitted. Under the "library is the product" thesis the primary customer
is a C++ embedder who constructs `VehicleParams` directly and never sees a tyre
file at all. That customer is not blocked by a schema version, and holding the
physics back on their behalf would be holding it back for nobody.

The loader does not paper over the gap. Asked to build L2 from a tyre file that
carries no slip stiffness, it raises an error naming the missing parameter. It
does not default the value, it does not substitute the cornering stiffness, and
it does not build a lower tier instead.

That last point is worth stating explicitly because it looks adjacent to
ADR-0005 and is not a violation of it. ADR-0005 forbids an unimplemented tier
falling back to a simpler one, and the failure it describes is a trajectory
labelled L2 that is actually L1. A refusal is the opposite: nothing is produced
and nothing is mislabelled. The user gets an error rather than a worse answer
wearing the right name.

Adding `c_kappa` to `tyre.schema.json` is scheduled as schema 0.2.0, tracked
separately from P1's slices. Under ADR-0015 the schema version moves
independently of `slipx_core`'s, and `tools/version_check.py` refuses to couple
them, so nothing about this forces a core release.

## Consequences

Between this decision and schema 0.2.0, L2 is constructible from C++ and not
reachable from a stock tyre file, so it is effectively unreachable from Python.
That is a real cost and the main argument for the option that was rejected. It
is accepted because the alternative is either an unmeasured constant in the
force law or a tier that cannot represent traction loss under power, which is
the thing a double-track model is built for.

The tier therefore lands with a gap between what the core can do and what a
contributed parameter set can reach. Anybody reading `tyre.schema.json` 0.1.0
and wondering why their tyre file will not drive an L2 model is hitting this
record. The error message must name the parameter and say that schema 0.2.0
carries it, because an error that says only "missing parameter" sends the reader
looking for a field that does not exist.

`tyre.schema.json` now has two fields in the derived-and-checked category rather
than one. `mf_lite.B` was the first, by ADR-0023. When 0.2.0 adds `c_kappa`, the
loader gains a second consistency check of the same shape, and the two should be
implemented together rather than one now and one later.

Schema 0.2.0 is a minor bump, so under ADR-0011 a 0.1.0 parser refuses a 0.2.0
file rather than ignoring the new field. A contributor who adds `c_kappa` to a
file therefore makes it unreadable by a released `0.1.0a1`, which is the
intended behaviour and will still surprise somebody.

Reversing this means removing the field and holding the longitudinal branch
after all, which would need a new record and would have to explain what changed
about the sequencing, since the physics will not have changed.
