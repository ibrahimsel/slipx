# ADR-0023: MF-lite derives the stiffness factor B rather than reading it

- **Status:** Accepted
- **Date recorded:** 2026-08-03 (decision taken during P1)
- **Requirements:** CORE-06, SCH-02, SCH-05, NFR-08
- **Related:** [ADR-0009](0009-mf-lite-over-full-pacejka.md),
  [ADR-0010](0010-tyres-are-compound-surface-pairs.md),
  [ADR-0022](0022-load-transfer-is-quasi-static.md)

## Context

MF-lite is the second piece of L2. Its lateral branch is

```
Fy = -mu_y(Fz) Fz sin(C atan(B alpha - E (B alpha - atan(B alpha))))
```

and `tyre.schema.json` at schema 0.1.0 already accepts `B`, `C` and `E` as three
independent numbers in an `mf_lite` block, alongside `linear.c_alpha`, the
cornering stiffness, in a block of its own.

Those four numbers are not independent. Expanding the formula at small slip
gives

```
Fy -> -(B C mu_y Fz) alpha        so       C_alpha = B C D,   D = mu_y Fz
```

A tyre file that supplies all four therefore states the same fact twice and may
state it inconsistently. Worse, fitting all four to one data set has no unique
answer: halving `B` and doubling `C` leaves the curve near the origin almost
unchanged, so an optimiser finds a valley rather than a minimum and two people
fitting the same rosbag report different coefficients.

ADR-0009 set the admission criterion for a tyre parameter: what manoeuvre
identifies it? Applied here it answers the question. `C_alpha` is the low-slip
slope of a skidpad and is measurable. `B` on its own is not measurable at all;
it has meaning only inside the product `B C D`.

There is a second, separate question of the same kind. Load sensitivity is an
exponent about a nominal load, `mu(Fz) = mu_y0 (Fz/Fz_nom)^(-k_mu)`, and
`tyre.schema.json` carries an optional `nominal_load` field for it. The core
also has a nominal load available for free: the static per-tyre load, computed
from mass, weight distribution and `static_loads` in `load_transfer.hpp`.

Three options were considered for `B`.

**Consume `mf_lite.B` as written.** Simplest, and it is what the schema looks
like it is asking for. It accepts an inconsistent file silently, and the
inconsistency is invisible: the curve still has the right peak, it just has the
wrong slope, which is the region nearly all driving happens in.

**Consume `B` and reject a file where `B C D` disagrees with `c_alpha`.** Keeps
the schema literal and catches the inconsistency. It also makes every
contributor supply a number they cannot measure and then makes their file fail
validation when they get it wrong.

**Derive `B` and check the file's value against the derivation.** One authority
for the slope, no unmeasurable parameter required from a contributor.

## Decision

`B` is derived at construction from the cornering stiffness and the nominal
load,

```
B = C_alpha / (C mu_y0 Fz_nom)
```

and is never read from a parameter set. `Fz_nom` is the static per-tyre load of
the car wearing the tyre, computed from the vehicle parameters, rather than the
schema's `nominal_load` field.

`B` stays fixed as the vertical load changes, so the cornering stiffness
inherits the load sensitivity of `mu`:

```
C_alpha(Fz) = B C mu(Fz) Fz = C_alpha(Fz_nom) (Fz / Fz_nom)^(1 - k_mu)
```

This is the degressive load dependence a real tyre has, and it arrives without a
second load-sensitivity parameter to identify.

Two further reasons, beyond identifiability.

The tiers agree by construction. `Fy` tends to `-C_alpha alpha` exactly as the
slip angle goes to zero, which is L1's linear tyre, so the cross-tier
convergence check measures discretisation error rather than a parameter
mismatch. Had `B` been read, the two tiers would agree only to the accuracy with
which somebody happened to have fitted `B`, and every cross-tier comparison
would have carried that error without saying so.

Using the static per-tyre load rather than the schema's `nominal_load` keeps the
reference point at the operating condition every other result is compared
against. It also means the core needs no new parameter: it already knows the
mass and the weight distribution, and `static_loads` already computes the
number.

## Consequences

`tyre.schema.json`'s `mf_lite.B` becomes derived-and-checked rather than
consumed. The field stays, because removing it from a published schema is a
breaking change and because it is worth checking; when the loader is wired up
for L2 it must compare the file's `B` against the derivation and report a
disagreement rather than silently preferring one. Until that is done, `B` in a
tyre file is inert, which is a thing to be aware of when reading a file and
wondering why editing it changes nothing.

`nominal_load` is likewise not consumed by the core. That is a live
inconsistency with the field's own description, which says it is required for
load sensitivity to mean anything, and it means a set identified at a load quite
different from the car's static load will have its `k_mu` applied about the
wrong reference. The loader should warn when the two differ by more than a
modest factor. This is the cost of the decision and it is real.

Contributors supply `C_alpha`, `mu_y0`, `mu_x0`, `k_mu`, `C`, `E` and `sigma`.
Every one of those has a manoeuvre attached to it in ADR-0009's table. `B` does
not appear on that list, which is the point.

The decision transfers the whole burden of the curve's shape onto `C` and `E`,
and they turn out to be sharper than they look. With the slope and the peak both
fixed by measurement, the pair decides one remaining thing: how far out the peak
sits, measured against `alpha_lin = mu_y Fz / C_alpha`. The multiple depends on
`C` and `E` alone and is between about 1.5 and 3 for a real tyre. The schema
bounds `C` and `E` independently, and `C = 1.05` with `E = 0.87` is legal and
gives a multiple of about 21, which is a tyre whose peak no car reaches. The
schema's `C > 1` bound was written to exclude "a curve with no peak" and on its
own does not. Whether a schema 0.2.0 consistency rule should constrain the pair
is open and is not decided here; `tyre.hpp` documents the trap and
`test_analytical.cpp` asserts it.

Reversing this means taking `B` as a parameter again, which would need a new ADR
and would have to say what manoeuvre identifies it.
