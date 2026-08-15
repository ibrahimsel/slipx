# ADR-0036: Schema 0.3.0 adds the track manifest as a document kind

- **Status:** Proposed
- **Date recorded:** 2026-08-15 (decision taken during P1)
- **Requirements:** SCH-01, SCH-02. No scene requirement ID is cited:
  `docs/spec` is not present in this checkout.
- **Related:** [ADR-0011](0011-schema-refuses-a-newer-minor.md),
  [ADR-0015](0015-independent-versioning.md),
  [ADR-0030](0030-schema-0-2-0-adds-c-kappa-and-the-actuator-fields.md),
  [ADR-0034](0034-a-track-is-geometry-plus-a-declared-surface.md),
  [ADR-0035](0035-track-geometry-is-converted-never-redistributed.md)

## Context

ADR-0034 splits a track into geometry and a manifest, and the manifest needs a
schema. That raises a question the previous schema bump did not: 0.2.0 added
fields to documents that already existed, and this adds a document kind while
touching no existing field. Whether that is a version change at all is
arguable.

The case for leaving the version alone is that no car file changes meaning, no
existing parser misreads anything, and the released 0.2.0 parser has no
`load_track` to be confused by a track file in the first place. On that
reading the version records the shape of documents, and no document's shape
moved.

The case against is what ADR-0011 is for. The version is the version of the
schema *set*, and the protection it buys is that a parser handed a file from
the future refuses instead of guessing. If a track file declared 0.2.0, then
the meaning of "this file is schema 0.2.0" would depend on which release you
were holding, which is precisely the ambiguity the version exists to remove.

## Decision

The schema goes to 0.3.0, and 0.3.0 adds one document kind, `track`, changing
no field of any kind that already existed.

The migration from 0.2.0 is the identity for every kind. It is registered
explicitly per kind, as the 0.1.0 step was, because a gap in the chain is a
release bug and an implicit identity would hide a real one.

`track` is registered at the earlier steps too, which is the odd part of this
record. No track file written at 0.1.0 or 0.2.0 exists, because there were no
tracks then. The entries are there so that a file somebody hand-wrote with the
wrong version in it is read and then validated against the real schema, which
produces a message about the field that is actually wrong, rather than the
"this is a release bug, not a problem with your file" that a hole in the chain
reports.

Two absences in `track.schema.json` are decisions rather than omissions, and
both are enforced by `additionalProperties: false` so they arrive as refusals
rather than as silence:

**No friction.** A track declares a surface identifier and nothing about grip.
Friction lives in the `(compound, surface)` tyre file (ADR-0010), which is the
parameter the identification programme exists to measure, and a number here
would be a second source for it supplied by a track author who measured
nothing.

**No banking.** No tier consumes it, so accepting the field would mean
dropping the value.

The surface identifier is constrained to lower case by pattern. It is matched
exactly everywhere it is used, so two spellings of one surface would be two
surfaces, and settling that when the file is written is cheaper than
reconciling a registry later.

The geometry block, naming the source and its licence, is required. That is
unusual for a schema, which normally describes physics rather than provenance,
and it is required because ADR-0035 means every track for a real venue arrived
from somewhere else under somebody else's terms.

## Consequences

The distribution version and the schema version are now different numbers
again, which is the normal state and the one ADR-0015 exists to protect.
`tools/version_check.py` continues to refuse to compare them.

Every `$id` moved to `0.3.0`, including those of the six documents that did
not otherwise change. That is churn with no meaning attached to it, and the
alternative, a set of files whose `$id`s disagree about which schema version
they belong to, is worse.

The reference car directory still declares 0.2.0 and loads by migration. That
is deliberate: it keeps the migration path exercised by the files that ship
rather than only by a test fixture, and there is nothing in 0.3.0 for a car
file to say.

A parser at 0.2.0 handed a 0.3.0 track file refuses it and says to upgrade.
That is the behaviour ADR-0011 chose, and this is the first release where it
does something a user will actually see.
