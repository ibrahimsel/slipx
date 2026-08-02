# ADR-0016: One distribution, two importable packages, and `slipx_core` is not on PyPI

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0)
- **Requirements:** CORE-01, NFR-06, NFR-09
- **Related:** [ADR-0003](0003-dependencies-point-downward.md),
  [ADR-0015](0015-independent-versioning.md)

## Context

Three things could plausibly be published to PyPI: the bindings, the schema
package, and the core.

The core is a C++ library. Its consumers are simulators written in C++, and the
way to depend on it is `add_subdirectory` or `find_package`. Publishing it to a
Python index would attach a Python packaging story to a library whose entire
selling point is that it has no dependencies and needs no ecosystem
([ADR-0002](0002-no-eigen-in-the-core.md)).

The bindings and the schema package are a real question. They are separate
layers with separate versions and a strictly one-way dependency, which argues
for two distributions. Against that: a student typing `pip install slipx` and
getting a package that cannot open a car file has been failed, and the second
`pip install` is a step at which people give up.

## Decision

One distribution named `slipx`, providing two importable packages: `slipx` (the
bindings plus the car loader) and `slipx_schema` (the schemas and the reference
parser).

They remain separately versioned
([ADR-0015](0015-independent-versioning.md)) and the dependency between them
still runs one way only. What is shared is the installer, not the version
number.

`slipx_core` is not on PyPI and is not intended to be.

## Consequences

`pip install slipx` produces something that can immediately load a car, because
the reference car directory ships inside the wheel. That is the difference
between an installed package that can demonstrate itself and one that requires a
clone first, and it is why the P0 exit gate is expressible as a single command.

Somebody who wants the physics without the file format still gets `pyyaml` and
`jsonschema` as dependencies. They are small, and the alternative was the extra
install step above. The extension module itself has no Python dependencies at
all, so the cost is confined to the distribution metadata.

The two packages cannot be released independently even though they are versioned
independently. If that ever becomes a real constraint, splitting into two
distributions is the change, and it would need this record superseded rather
than the versioning scheme altered.

C++ consumers are unaffected by any of this, which is the intended shape.
