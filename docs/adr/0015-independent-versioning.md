# ADR-0015: `slipx_core` and `slipx_schema` are versioned independently, and it is checked

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (the enforcing check was added at release time)
- **Requirements:** NFR-09, SCH-01
- **Related:** [ADR-0011](0011-schema-refuses-a-newer-minor.md),
  [ADR-0016](0016-one-distribution-two-packages.md),
  [ADR-0017](0017-first-release-is-a-pre-release.md)

## Context

Two things in SlipX have public versions and they change for unrelated reasons.
`slipx_core`'s version describes an API and an ABI. `slipx_schema`'s version
describes a file format that files in the wild declare conformance to.

Sharing one number couples them in both directions and both are bad. A schema
addition the core never sees would force a core release, and consumers would be
asked to re-pin a library that did not change. A core ABI break would advance
the schema version, and every car file in existence would suddenly declare an
older schema than the current one, triggering migrations for a format that did
not change.

The pressure to couple them is not theoretical. They are both `0.1.0` today,
they are released in one distribution
([ADR-0016](0016-one-distribution-two-packages.md)), and the equality looks
like an invariant to anyone who did not know it was a coincidence.

## Decision

The two are versioned independently and semantically (NFR-09), with the pre-1.0
caveat that a minor bump may break the API.

The distribution version tracks `slipx_core`. `slipx_schema` carries its own,
declared in car files and gated on load.

`tools/version_check.py` enforces the half that can be enforced and deliberately
does not enforce the other half. It checks that the distribution version is
written identically in the four places it appears, and it reports the schema
version **without ever comparing it to anything**, printing a note when the two
happen to be equal saying that this is a coincidence the check will never
enforce.

The four places, and why there is no way to make there be one:

| Where | Read by |
|---|---|
| `pyproject.toml` | the build backend, before CMake runs |
| `CMakeLists.txt` | C++ consumers doing `find_package` |
| `include/slipx/version.hpp` | the run manifest, and any C++ consumer with no Python near it |
| `slipx/version.py` | importable without the extension being built |

Each is authoritative for a different consumer. Generating three from the fourth
would put a build step between a git checkout and a readable version, which is
worse than duplication plus a check.

CMake cannot express a PEP 440 pre-release suffix, so `project(VERSION)` carries
the numeric release part only and the check knows that: `0.1.0a1` in
`pyproject.toml` requires `0.1.0` in `CMakeLists.txt`.

## Consequences

A version bump is a four-file edit, five with the C++ numeric triple. The check
runs in the CI policy job on every commit, so a partial bump fails while the
commit that did it is still on screen rather than at tag time.

The release workflow passes the git tag to `--expect`, so a tag and a
distribution cannot disagree about what was released. Given that a version on
PyPI can never be reused, that is a failure worth catching before the upload
rather than after.

The two numbers will eventually diverge, and that is the success condition, not
a problem to reconcile. The moment they do, anything that assumed they were
equal breaks, which is why the check refuses to be the thing that assumed it.
