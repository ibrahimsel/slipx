# ADR-0041: Schema 0.4.0 opens the compound vocabulary and ties a fit to its data

- **Status:** Proposed
- **Date recorded:** 2026-08-19 (decision taken while landing the fitter's
  emission path)
- **Requirements:** SCH-02, SCH-03, SCH-06 in spirit; `docs/spec` is not
  present in this checkout, so no ID is cited as authority.
- **Related:** [ADR-0010](0010-tyres-are-compound-surface-pairs.md),
  [ADR-0011](0011-schema-refuses-a-newer-minor.md),
  [ADR-0040](0040-the-fitter-reads-rosbag2-without-ros.md)

## Context

The tyre schema constrained `compound` to the enum `["sponge", "rubber"]`.
That was defensible when every tyre file was hand-written against the
RoboRacer rules, and it broke the moment an identification tool existed:
`slipx-id`'s first emitted car could not name its own tyres, because the
fitted compound of an arbitrary car is not guaranteed to be one of two
words. Surfaces already made the opposite choice, for a reason written in
the schema itself: they are a community vocabulary, and hard-coding a list
makes a contributor's reality unrepresentable. The same argument applies to
compounds, word for word, once contributors fit tyres.

Separately, P2's registry needs a submission to be tied to the data it was
fitted from. The provenance block carried the story (source, method, date,
contributor, residuals) but nothing that pins the story to bytes: two
submissions could cite "our skidpad bags" and nobody could say whether the
bags were the same, changed, or ever existed.

## Decision

**Schema 0.4.0.** Three changes, all backwards compatible, so every 0.3.0
document is already a valid 0.4.0 document and the migration steps are
identities:

1. **`compound` becomes a pattern-constrained string** (the same slug
   pattern surfaces use), in the tyre file and in the car's tyre
   references. Sponge and rubber remain the RoboRacer-legal values;
   checking competition legality moves to where it belongs, the registry's
   review and the competition's scrutineering, because a parser that
   enforces one ruleset's tyre list cannot serve a second ruleset at all.
2. **Provenance gains an optional `data` block**: the recordings the fit
   consumed, each named and SHA-256 digested. `slipx-id` fills it
   automatically, so for anyone using the tool the cost is nothing.
3. **The registry's acceptance bar becomes code**:
   `slipx_schema.rules.check_registry_submission` refuses, naming each
   obligation, a submission that is not labelled `identified` with a
   contributor, per-parameter residuals, an attached validation report and
   the `data` block. Locally none of that is required; the registry is
   where a parameter set becomes a claim other people tune against, so the
   bar lives at that door and nowhere else.

## Consequences

- The parser no longer rejects a competition-illegal compound, so a car
  file can describe a car the RoboRacer rules would not admit. That is the
  point: SlipX models cars, the ruleset defines competitions, and the
  registry check plus scrutineering own legality. SCH-03's dimensional
  checks are untouched.
- Free compound naming can fragment the registry's vocabulary ("sponge",
  "sponge-soft", "softsponge" as three entries). The acceptance bar is
  review by a human with the check's output in front of them, and naming
  convergence is review work, not parser work.
- A digest ties a submission to bytes, not to truth: fabricating a bag and
  digesting it is still possible (ADR-0040 already owns this). What the
  digest ends is silent drift between the data reviewed and the data cited.
- Anything pinned to schema 0.3.0 (none known outside the tests) sees a
  minor bump; the version gate and migration chain handle both directions
  as always (ADR-0011).
