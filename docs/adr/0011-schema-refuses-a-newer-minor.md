# ADR-0011: The parser refuses a newer minor rather than ignoring fields it does not know

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0)
- **Requirements:** SCH-01, SCH-02
- **Related:** [ADR-0015](0015-independent-versioning.md),
  [ADR-0006](0006-diagnostics-report-nan-not-zero.md)

## Context

Every SlipX file carries a `schema_version`. The parser has to decide what to do
with each of the four cases it can meet: its own version, an older minor, a
newer minor, a different major.

Three of those are uncontroversial: parse, migrate forward, refuse. The fourth,
a newer minor, is where the common choice and the right choice differ.

The common choice is to tolerate it. A minor version by definition only adds
fields, an old parser ignores fields it does not know, and being liberal in what
you accept is the received wisdom.

Applied here, "ignore a field you do not know" means ignoring a parameter that
the file's author believed was in effect. The run succeeds. It produces a
plausible number. It is not the car that was described, and nothing anywhere
says so. That is precisely the silent-wrong-answer failure the whole schema
layer exists to prevent.

## Decision

A parser accepts its own major version at its own minor or older, migrates older
minors forward, and refuses everything else. A newer minor is refused with a
message written for the person holding the file, not for a log.

Refusal messages name the actual remedy: install a release implementing that
schema major, or migrate the file.

## Consequences

An old install refuses a new file instead of half-reading it. From the user's
point of view this is a worse experience at the moment it happens and a better
one than discovering, later, that a parameter they set had no effect.

Every schema minor bump needs migration machinery written in the same change,
because older files must still load. At schema 0.1.0 there is nothing to
migrate from, which is the cheapest possible moment to have built the
machinery and the reason it was built then.

`schema_version` is required to be a quoted string. YAML reads `0.1` as a float
and `0.1.0` as a string, so an unquoted version is a type error rather than a
version, and the parser says exactly that rather than failing further down.

This rule applies per file, before anything in the document is believed. A file
whose version cannot be handled is never partially parsed.
