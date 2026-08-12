# Architecture decision records

One file per decision, numbered, never renumbered. A record is written when a
decision constrains work that comes after it, and it is written whether the
decision was hard or obvious, because the obvious ones are the ones that get
quietly reversed by somebody who never saw the reasoning.

These are public. `docs/spec` is gitignored and `CLAUDE.local.md` is not
checked in, so a decision recorded only in either of those is a decision an
outside contributor cannot read. This directory is where the reasoning lives
for anyone who is not us.

A record is never edited to say something different. It is superseded by a new
record, and the old one is marked `Superseded by ADR-NNNN` with its text left
intact. The value of the file is the reasoning at the time, including reasoning
that later turned out to be wrong.

Format is [Michael Nygard's][nygard], with a `Requirements` field added because
this project cites SRS requirement IDs inline and a decision that implements
one should say so.

[nygard]: https://cognitect.com/blog/2011/11/15/documenting-architecture-decisions

Use [`template.md`](template.md) for a new record.

## The core

| # | Decision | Status |
|---|---|---|
| [0001](0001-record-architecture-decisions.md) | Record architecture decisions in this directory | Accepted |
| [0002](0002-no-eigen-in-the-core.md) | `slipx_core` depends only on the C++ standard library | Accepted |
| [0003](0003-dependencies-point-downward.md) | Dependencies point strictly downward, and the binding layer sits above what it binds | Accepted |
| [0004](0004-step-is-const-and-stateless.md) | `step` is `const` and stateless | Accepted |
| [0005](0005-tiers-throw-rather-than-fall-back.md) | An unimplemented tier throws rather than falling back | Accepted |
| [0006](0006-diagnostics-report-nan-not-zero.md) | Diagnostics report NaN, not zero, for what a tier cannot represent | Accepted |
| [0022](0022-load-transfer-is-quasi-static.md) | L2 load transfer is quasi-static and introduces no suspension parameter | Accepted |
| [0026](0026-relaxation-lags-the-slip-angle-not-the-force.md) | Tyre relaxation lags the slip angle, not the lateral force | Accepted |
| [0027](0027-l2-closes-its-algebraic-loops-without-iterating.md) | L2 closes its algebraic loops without iterating and without a wheel rotational state | Accepted |
| [0028](0028-runs-are-emitted-to-sinks-viewers-are-external.md) | Runs are emitted to sinks, and interactive viewers stay external | Accepted |

## Determinism

| # | Decision | Status |
|---|---|---|
| [0007](0007-determinism-is-scoped-to-a-build.md) | Determinism is promised within one build and not across platforms | Accepted |
| [0008](0008-reference-hashes-are-keyed-by-build.md) | Reference trajectory hashes are keyed by build, and the reference runners are pinned | Accepted |

## Parameters and the schema

| # | Decision | Status |
|---|---|---|
| [0009](0009-mf-lite-over-full-pacejka.md) | MF-lite, and every tyre parameter must be identifiable in a car park | Accepted |
| [0010](0010-tyres-are-compound-surface-pairs.md) | Tyres are referenced as a `(compound, surface)` pair | Accepted |
| [0011](0011-schema-refuses-a-newer-minor.md) | The parser refuses a newer minor rather than ignoring fields it does not know | Accepted |
| [0012](0012-no-scale-branching.md) | No `if (scale == ...)` branching in code | Accepted |
| [0013](0013-provenance-labels-are-printed.md) | Every parameter set carries a provenance label and the tooling prints it | Accepted |
| [0023](0023-mf-lite-derives-b-from-cornering-stiffness.md) | MF-lite derives the stiffness factor `B` rather than reading it | Accepted |
| [0025](0025-c-kappa-enters-the-core-ahead-of-the-schema.md) | The longitudinal slip stiffness enters the core ahead of the schema | Accepted |

## Licence, packaging and release

| # | Decision | Status |
|---|---|---|
| [0014](0014-apache-2-throughout.md) | Apache-2.0 throughout, and no copyleft anywhere near the core | Accepted |
| [0015](0015-independent-versioning.md) | `slipx_core` and `slipx_schema` are versioned independently, and it is checked | Accepted |
| [0016](0016-one-distribution-two-packages.md) | One distribution, two importable packages, and `slipx_core` is not on PyPI | Accepted |
| [0017](0017-first-release-is-a-pre-release.md) | The first published release is `0.1.0a1`, a pre-release | Accepted, superseded in part by 0029 |
| [0018](0018-wheel-coverage.md) | Wheels for five platforms on CPython 3.9 to 3.13 | Accepted |
| [0019](0019-trusted-publishing.md) | Publish with Trusted Publishing, not an API token | Accepted |
| [0020](0020-wheels-assert-nothing-about-their-hash.md) | A released wheel asserts nothing about its own trajectory hash | Accepted |
| [0021](0021-readme-is-the-pypi-description.md) | The README is the PyPI long description | Accepted |
| [0029](0029-no-0-1-0-final-first-final-is-0-2-0.md) | `0.1.0` final is never published; the first final release is `0.2.0` | Accepted |

## Looking at a run

| # | Decision | Status |
|---|---|---|
| [0024](0024-a-run-viewer-is-in-scope.md) | A run viewer is in scope, and it lives in the Python package | Accepted |
