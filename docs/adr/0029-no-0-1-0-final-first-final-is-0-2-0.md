# ADR-0029: `0.1.0` final is never published; the first final release is `0.2.0`

- **Status:** Accepted
- **Date recorded:** 2026-08-12
- **Requirements:** NFR-09
- **Related:** [ADR-0017](0017-first-release-is-a-pre-release.md), which this
  supersedes in part, [ADR-0008](0008-reference-hashes-are-keyed-by-build.md),
  [ADR-0020](0020-wheels-assert-nothing-about-their-hash.md)

## Context

ADR-0017 published `0.1.0a1` as a deliberate pre-release and planned to cut
`0.1.0` final from the same tree, commit `644cb12`, once a pre-release had
rendered and installed correctly from the live index. It has: the PyPI page
renders, a cold install works, and the conformance script runs out of the
wheel. By ADR-0017's own plan, `0.1.0` is due.

The tree has moved since. L2 is implemented, the sink layer exists, and all
twelve reference hashes were rerecorded because `alpha_lag` entered the state
layout. The current tree cannot be `0.1.0`: its hashes are incompatible with
the numbers published alongside `a1`. So `0.1.0` would have to be cut from
`644cb12`, a snapshot that every line of work since has superseded.

The alternatives actually considered:

1. Tag and publish `0.1.0` from `644cb12`, as ADR-0017 planned.
2. Publish nothing until M4 and release `0.2.0` as the first final.

## Decision

`0.1.0` is never published. The first final release of the `slipx`
distribution is `0.2.0`, cut at milestone M4 when the double-track tier is
complete, reachable from a car file and visible.

The reasoning: ADR-0017's plan assigned `0.1.0` final one job, to stop the
pre-release being selected once the metadata was proven sound. But the
metadata was proven sound by `a1` itself, and nothing in `a1` is defective
enough to need displacing. Publishing `0.1.0` from `644cb12` today would be
content-identical to the pre-release users already get, while creating a
second published hash table (the `a1` numbers under a final version) weeks
before `0.2.0` deliberately invalidates it. A release that adds no capability,
changes no install outcome and doubles the set of numbers people can compare
against is a support obligation bought with nothing.

The version number `0.1.0` is skipped, not reused. It must never be published
later to fill the gap: a `0.1.0` uploaded after `0.2.0` would sort below the
current release yet postdate it, and the CHANGELOG would be unreadable.

## Consequences

Until `0.2.0` is released, plain `pip install slipx` keeps resolving to the
pre-release, which pip does only because no final exists. Tools that refuse
pre-releases by default (Poetry, for one) cannot install SlipX until `0.2.0`.
That is a real cost carried for the weeks between this record and M4, and it
is the strongest argument for option 1; it loses because the fix is to finish
M4, not to publish a stale snapshot.

ADR-0017's decision stands: the first artefacts under the name were
pre-releases, and that reasoning is unchanged. Its consequence "`0.1.0` final
is cut from the same tree once a pre-release has rendered correctly" is
superseded by this record.

The CHANGELOG's `0.1.0a1` section remains the only record of that tree;
`0.2.0`'s section describes changes relative to `0.1.0a1`, there being no
`0.1.0` in between.

Reversing this means publishing `0.1.0` from `644cb12` before `0.2.0` exists;
after `0.2.0`, the ordering argument above makes it unpublishable forever.
