# Changelog

Semantic versioning (NFR-09), with the pre-1.0 caveat that a minor bump may
break the API. `tools/version_check.py` holds the four places a version is
written in agreement; the release workflow refuses a tag that disagrees with
the tree.

Two numbers are versioned here and they move independently. The distribution
and `slipx_core` share one; `slipx_schema` has its own. They are equal today by
coincidence, and a release that bumps one does not touch the other.

**A changed reference hash is a release event.** If a version changes any row
in `conformance/reference_hashes.tsv`, it is recorded here with the reason,
because every result anybody has previously compared against that row becomes
incomparable.

## Unreleased

P1 in progress: L2, the double-track tier. `VehicleModel::create` still throws
for L2 until the tier is complete; the pieces below are usable on their own but
no tier consumes them yet.

- New public header `slipx/load_transfer.hpp`: quasi-static longitudinal and
  lateral load transfer from mass, CoG height, wheelbase and track (CORE-05),
  plus the static rollover threshold `g t / (2 h)`. Header-only and pure.
  Reasoning in [ADR-0022](docs/adr/0022-load-transfer-is-quasi-static.md).
- New public header `slipx/tyre.hpp`: MF-lite, a reduced Magic Formula with
  load sensitivity and a combined-slip friction ellipse (CORE-06). The
  stiffness factor `B` is derived from the cornering stiffness rather than read
  from a parameter set, so MF-lite reproduces L1's linear tyre exactly at small
  slip and contributors are never asked for a number they cannot measure.
  Reasoning in
  [ADR-0023](docs/adr/0023-mf-lite-derives-b-from-cornering-stiffness.md).
  The pure-slip longitudinal law is absent: it needs a slip stiffness that
  `tyre.schema.json` 0.1.0 does not carry. `friction_ellipse` is independent of
  that and is here. See
  [ADR-0025](docs/adr/0025-c-kappa-enters-the-core-ahead-of-the-schema.md).
- New public header `slipx/relaxation.hpp`: tyre relaxation length, a
  first-order lag in distance rolled rather than in time, so the time constant
  is `sigma / speed` (CORE-07). It lags the slip angle rather than the lateral
  force, because a lagged force is not bounded by the friction budget it was
  produced under and would report grip on a lifted wheel. Reasoning in
  [ADR-0026](docs/adr/0026-relaxation-lags-the-slip-angle-not-the-force.md).
- `VehicleParams` gains `relax_length` [m]. `VehicleState` gains a per-wheel
  `alpha_lag` [rad].

- **Tier L2, the double-track model, is implemented** (CORE-02, CORE-12).
  `VehicleModel::create` no longer throws for it. Ten states: the six of L1
  plus a lagged slip angle per wheel. Four contact patches with per-corner
  vertical loads, MF-lite with a real peak and falling branch, a combined-slip
  friction ellipse, and the tyre transient. `StepDiagnostics` per-wheel slip
  angles, slip ratios, forces, loads and saturation flags are numbers at L2
  where they are NaN below it.

  It is the MINIMAL double-track and the gaps are deliberate: no differential
  or drive layout (CORE-11), no ESC (CORE-08), no battery (CORE-09), no
  steering servo (CORE-10), parallel steer rather than Ackermann, and no wheel
  rotational state, so a locked or spinning wheel is not representable.
  `l2_double_track.cpp`'s header lists every one of them and
  [ADR-0027](docs/adr/0027-l2-closes-its-algebraic-loops-without-iterating.md)
  gives the reasoning for the last.

- `VehicleParams` gains `tyre_front` and `tyre_rear` (MF-lite coefficients per
  axle, including the relaxation length, which is a property of the tyre) and
  `c_kappa`. The `relax_length` field added earlier in this cycle moved into
  `TyreCoefficients`; nothing released ever carried it at the top level.

- **L2 cannot be built from a tyre file at schema 0.1.0.**
  `Car.params_for_tier` raises, naming the missing longitudinal slip stiffness
  and the schema version that adds it, rather than defaulting it. That is a
  refusal and not the tier substitution ADR-0005 forbids: nothing is returned
  at all. Building L2 from a `VehicleParams` constructed in C++ or Python works
  today. See
  [ADR-0025](docs/adr/0025-c-kappa-enters-the-core-ahead-of-the-schema.md).

- New reference rows for L2 under both integrators. They are additions, so no
  previously published number became incomparable by their arrival.

### Every reference hash moves

**All twelve rows of `conformance/reference_hashes.tsv` are rerecorded.**
`alpha_lag` is part of `VehicleState`, the trajectory hash covers the state
layout in a fixed field order, and four new fields per step change the hash for
every tier including L0 and L1, neither of which has a tyre transient at all.

A result compared against the numbers published in `0.1.0a1` is **not
comparable** with one produced from this tree. The old and new values, for
x86-64 `RelWithDebInfo`, identical across GCC 11, GCC 13 and Clang 18 as
before and measured separately under each:

| Case | `0.1.0a1` | Now |
|---|---|---|
| L0 / rk4 | `d74f90169a5951c2` | `cf6aba9e280a24b9` |
| L0 / semi_implicit_euler | `44b1d28010f293c4` | `4cb3269ec5ba7ac3` |
| L1 / rk4 | `d44a9a68616ec899` | `f4da160a691289eb` |
| L1 / semi_implicit_euler | `9a2532ced2e1e06d` | `2e2fb5a549ad190c` |

This is a change to the state layout the hash covers, which
[ADR-0008](docs/adr/0008-reference-hashes-are-keyed-by-build.md) lists as a
legitimate reason for a hash to move. It was taken once, deliberately, rather
than accumulated across the rest of the tier.

## 0.1.0a1

First publication. Pre-release: the version number is the one part of a release
that can never be withdrawn, so the first artefacts under this name are spent
proving the packaging path against a live index rather than being pinned.

- `slipx_core` with tiers L0 (kinematic bicycle, 4 states) and L1 (dynamic
  bicycle, linear tyres, 6 states) behind one `VehicleModel` interface. RK4 and
  semi-implicit Euler. No dependency outside the C++ standard library.
- L2 and L3 are not implemented. `VehicleModel::create` throws for them rather
  than falling back to L1, because a trajectory labelled L2 that is actually L1
  is worse than no trajectory.
- `slipx_sim`: fixed-step lockstep orchestrator, per-agent seeded RNG, run
  manifests, trajectory hashing, replay from an input log.
- `slipx_schema` 0.1.0: six JSON Schema documents and a reference parser with
  version gating and migration machinery.
- Python bindings via pybind11, the car loader, and the `slipx-conformance`
  console script. The reference car ships inside the wheel.
- Reference trajectory hashes published for x86-64 under GCC 11, GCC 13 and
  Clang 18 at `RelWithDebInfo`.

No parameter set here has been validated against a real car. Every shipped set
is labelled `provisional` and the tooling prints the label (NFR-08).
