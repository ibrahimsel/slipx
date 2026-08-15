# ADR-0035: Track geometry is fetched and converted by the user, never redistributed by us

- **Status:** Proposed
- **Date recorded:** 2026-08-15 (decision taken during P1)
- **Requirements:** the P1 track deliverables. No SRS ID is cited: `docs/spec`
  is not present in this checkout, and a guessed ID is worse than none.
- **Related:** [ADR-0014](0014-apache-2-throughout.md),
  [ADR-0013](0013-provenance-labels-are-printed.md),
  [ADR-0034](0034-a-track-is-geometry-plus-a-declared-surface.md)

## Context

The plan was to ship one real track with a published real-world counterpart,
Porto or an equivalent from the public F1TENTH map set, so that a parameter
set fitted in P2 could be cross-checked against something outside the project.
Checking the licences before writing the loader turned that plan over.

| Source | What it has | Licence |
|---|---|---|
| `f1tenth/f1tenth_racetracks` | centrelines in exactly this format | GPL-3.0 |
| `TUMFTM/racetrack-database` | the upstream of the above | LGPL-3.0 |
| `f1tenth/f1tenth_simulator` | Porto, as an occupancy grid | none stated |
| `CPS-TUWien/f1tenth_maps` | a collection including Porto | none stated |
| `f1tenth/f1tenth_gym` | occupancy grids, no Porto, no centreline | MIT |

The centrelines themselves originate as OpenStreetMap GPS points, so the data
is additionally ODbL, which is share-alike for a derived database.

Two of those are copyleft and two have no licence at all, which is the more
absolute problem: a repository with no licence grants nothing, so Porto in
particular cannot be redistributed by anybody, by us least of all. The one
permissively licensed source has neither Porto nor a centreline in it.

Copyleft in this tree is not a paperwork problem. The whole strategy is
`slipx_core` embedded in projects we do not maintain, ADR-0014 exists because
a copyleft licence anywhere near that makes it legally awkward for exactly the
people the project is for, and `tools/licence_scan.py` fails on copyleft
licence text anywhere in the tree by design.

Options considered:

**Vendor the GPL centreline in its own directory and carve it out of the
licence scan.** Rejected. The carve-out is the whole of the decision: a rule
that is suspended the first time it is inconvenient was never a rule, and the
suspension would sit in the repository whose selling point is that it has no
such entanglements. It is also not clear that a GPL data file in an
Apache-2.0 distribution is a thing we may do, and "not clear" is the answer
that matters when the audience is other people's employers.

**Ship only a track we generate.** Clean, and it gives up the reason M5.2
wanted a real track. Kept, but as a component of the decision rather than the
whole of it.

**Redraw a real venue ourselves and licence it Apache-2.0.** Genuinely
attractive later. Rejected for the first slice because the geometry would then
be ours to be wrong about, with no measurement behind it, which trades a
licence problem for a provenance one.

## Decision

SlipX ships the converter, not the data.

An Apache-2.0 tool fetches a centreline from a source the user names, converts
it into the track directory of ADR-0034, and writes the source, its licence
and the retrieval date into the track manifest's provenance block. Porto stays
one command away and zero bytes of it are in this repository or in a wheel.

Alongside it, one small track that we generate ships in the tree, with
provenance stating plainly that it is generated and has no real-world
counterpart. It is what CI runs, what the examples open, and what the tests
assert against. That keeps the shipped, tested path free of any network
dependency, which is a property worth having for its own sake and not only for
the licence.

The converter states the licence of what it fetched rather than leaving the
user to discover it, and it says so for the same reason ADR-0013 makes the
tooling print provenance labels: an obligation nobody was told about is an
obligation nobody meets. A user who converts an OSM-derived centreline and
then publishes their converted track is subject to ODbL share-alike on it, and
the manifest the converter writes says that in the file itself.

## Consequences

CI never exercises a real track, only the generated one and the converter's
fixtures. That is a real gap: a parsing bug that only a real file triggers,
a degenerate segment or a duplicated point, would not be caught. The fixtures
have to be written to include those cases deliberately, because the file that
would have found them by accident is not there.

A published result run on a converted track is not reproducible from this
repository alone. Whoever publishes it has to publish their track directory
too, and the run manifest records the track's provenance so the omission is
visible rather than silent.

The cross-check M5.2 wanted is deferred, not obtained. A P2 fit validated
against a real venue still needs the venue, and this record only makes getting
it the user's action instead of ours. Redrawing a venue under our own licence
stays open and would supersede the shipped-track half of this decision without
touching the converter half.

The converter is the first thing in the tree that touches the network, and it
must stay out of anything the deterministic path can reach. It is a tool, run
once, producing files; no run, sink or manifest may fetch anything.

Reversing this record means either accepting copyleft in the tree, which is
ADR-0014's decision and not this one's, or a source appearing that is
permissively licensed and carries real centrelines. The second would be good
news and would need only the shipped-track half revisited.
