# ADR-0017: The first published release is `0.1.0a1`, a pre-release

- **Status:** Accepted; superseded in part by
  [ADR-0029](0029-no-0-1-0-final-first-final-is-0-2-0.md) (`0.1.0` final is
  never cut)
- **Date recorded:** 2026-08-02
- **Requirements:** NFR-09
- **Related:** [ADR-0015](0015-independent-versioning.md),
  [ADR-0019](0019-trusted-publishing.md),
  [ADR-0021](0021-readme-is-the-pypi-description.md)

## Context

The goal was to publish and reserve the name `slipx`. The tree produced `0.1.0`,
and a version number on PyPI can never be reused, even after the release is
deleted.

Two questions were tangled together and are worth separating.

**Is `0.1.0` an honest claim?** The worry was that no parameter set has been
validated against a real car and L2 does not exist, so `0.1.0` over-claims. On
examination it does not, because that is not what a version number encodes.
`0.x` already says the API is unstable, the `Development Status :: 3 - Alpha`
classifier says alpha, `car.summary()` prints PROVISIONAL on every load
([ADR-0013](0013-provenance-labels-are-printed.md)), and the README says it in
the second paragraph. Deflating the number to `0.0.1` would be a fourth, weaker
statement of something three stronger mechanisms already make, and it would
misdescribe what is actually there: a complete, tested, CI-gated L0/L1 stack
with 148 C++ tests and 87 Python tests behind it. Under-claiming is a discipline
failure in the same way over-claiming is.

**What is irreversible about a first upload?** Everything except the code. The
long description rendering, the project URLs, the sdist contents, the wheel
tags, the console script on a machine that never had a checkout. Each of those
is a metadata mistake that cannot be fixed without burning a version, and the
first upload is the first time any of them meets a real index. Two were already
found by inspection before publishing: the project URLs pointed at a repository
that does not exist, and the README's banner was a relative path that renders
broken on PyPI.

## Decision

The first artefacts published under the name `slipx` are pre-releases, starting
at `0.1.0a1`. `0.1.0` final is cut from the same tree once a pre-release has
rendered correctly on the live index.

The reasoning in one sentence: the version number is the one part of a release
that can never be withdrawn, so the first numbers spent under a name buy proof
that the packaging works rather than being pinned by anybody.

This is not modesty about the code. It is scepticism about the metadata.

## Consequences

`pip install slipx` still works while only pre-releases exist, because pip
selects a pre-release when there is no final release. So the name is reserved
and the P0 exit gate is closeable against `a1` by a third party.

Once `0.1.0` final exists, `a1` is never selected again, so anything wrong with
it stops mattering.

The TestPyPI rehearsal spends `0.1.0a1` on that index too, with `skip-existing`
set. If the rehearsal reveals a metadata bug, `a1` cannot be re-uploaded there,
and the response is to bump to `a2` everywhere and rehearse that, which then
becomes the first PyPI release. Keeping the rehearsal artefact byte-identical
to the release artefact is worth more than the alpha number, which is precisely
why an alpha number was spent.

`kVersion` in the C++ header carries the `a1` suffix while the numeric triple
stays `0/1/0`. The suffix reaches the run manifest, and two artefacts recording
the same core version there have to be the same core.
