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
- No reference hash moves: nothing L0 or L1 integrates has changed.

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
