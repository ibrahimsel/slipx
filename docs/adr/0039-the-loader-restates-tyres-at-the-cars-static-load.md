# ADR-0039: The loader restates a tyre's coefficients at the car's static load

- **Status:** Proposed
- **Date recorded:** 2026-08-19 (found while designing the P2 fitter)
- **Requirements:** SCH-02, SCH-05 in spirit; `docs/spec` is not present in
  this checkout, so no ID is cited as authority.
- **Related:** [ADR-0010](0010-tyres-are-compound-surface-pairs.md),
  [ADR-0023](0023-mf-lite-derives-b-from-cornering-stiffness.md),
  [ADR-0025](0025-c-kappa-enters-the-core-ahead-of-the-schema.md)

## Context

A tyre file states its coefficients at its own `nominal_load`, whose schema
description says exactly why it exists: "k_mu is an exponent about this
load". The core, deliberately and with its reasons documented on
`MfLite::fz_nom`, states every tyre at the static per-tyre load of the car
wearing it. Someone has to bridge the two references, and until now nobody
did: the loader copied `mu_y0`, `mu_x0`, `c_alpha` and `c_kappa` through
unchanged, which is only correct when the file's `nominal_load` happens to
equal the car's static load.

That coincidence holds for the reference car and for no other pairing, and
tyres are deliberately shared across cars: ADR-0010 made a tyre a
`(compound, surface)` pair precisely so that one measured file serves many
chassis, and the P2 registry exists to circulate such files. A 4.5 kg car
loading a tyre identified under a 3.5 kg one would silently run with every
load-dependent coefficient misreferenced by the mass ratio raised to the
tyre's own exponents. This is the silent misparameterisation SCH-02 exists
to prevent, wearing the one disguise the refusal logic could not see.

The alternatives: refuse a mismatched `nominal_load` outright, which would
make shared tyre files unusable and delete ADR-0010's point; ignore the
field forever, which is the bug; or restate the coefficients at load time.

## Decision

**The loader restates the load-dependent coefficients at the car's static
per-tyre load, exactly, and records that it did so.** MF-lite's load laws
make the restatement lossless rather than approximate: friction and both
stiffnesses are power laws in `Fz / Fz_nom`,

```
mu(Fz)      = mu_0    · (Fz / Fz_nom)^(-k_mu)
C_alpha(Fz) = C_alpha · (Fz / Fz_nom)^(1 - k_mu)
c_kappa(Fz) = c_kappa · (Fz / Fz_nom)^(1 - k_mu)
```

so moving the reference from the file's load to the car's multiplies the
frictions by `ratio^(-k_mu)` and the stiffnesses by `ratio^(1 - k_mu)`,
leaves `C`, `E` and the relaxation length untouched, and leaves the derived
`B = C_alpha / (C · mu_y0 · Fz_nom)` algebraically invariant. The same
physical tyre, restated. The restatement lands in `slipx_schema`'s assembly
step, where the arithmetic's inputs are still named and validated, per the
existing rule that no derived quantity is computed in the bindings.

Three honest edges, each recorded rather than silent: a file with no
`nominal_load` is taken as stated at the car's static load, with a note; a
file with a `nominal_load` but no `k_mu` (every 0.1.0-era file) cannot be
restated, gets a note, and cannot reach L2 anyway; and a ratio beyond 3 in
either direction is restated but warned about, because a tyre stated at a
third of the car's corner weight is more likely a units or pairing error
than a heavy car, and `strict=True` turns that warning into a refusal as
usual. A ratio within one part in 10⁹ of unity is left bit-identical, so a
file written at the car's own static load is used exactly as written.

The reference car's tyre file now states `nominal_load` as the exact static
per-tyre load of the reference car, making the identity restatement explicit
rather than coincidental, and every number ADR-0032 chose survives to the
bit.

## Consequences

- A car file whose tyre was stated at a different load now loads different
  (correct) numbers than it did before this record. Nothing published moves:
  the conformance scenario runs on struct defaults, and the reference
  pairing is the identity by construction.
- `slipx_schema` gains a written-down copy of standard gravity, because it
  may not import the core. The value is a defined constant; it cannot drift
  by accident.
- The fitter (`slipx_id`) can emit a tyre file at whatever load the test car
  had and trust every future pairing to restate it, which is what makes
  registry sharing honest. The ballast manoeuvre for `k_mu` also becomes
  meaningful in-model: a ballasted car restates the same tyre at a higher
  reference, which is exactly the physical claim.
- One value of `c_kappa` still serves all four wheels, so on a car with
  unequal weight distribution the loader restates each tyre's `c_kappa` at
  its own axle's static load and then averages the two, as it already
  averaged the unrestated values; the residual error is the second-order
  amount the single-parameter design already accepted (ADR-0030).
