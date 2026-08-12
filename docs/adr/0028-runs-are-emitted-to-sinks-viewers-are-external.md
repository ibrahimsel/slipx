# ADR-0028: Runs are emitted to sinks, and interactive viewers stay external

- **Status:** Accepted
- **Date recorded:** 2026-08-12 (decision taken during P1)
- **Requirements:** VIZ-01 to VIZ-06, SINK-01 to SINK-05, NFR-01, NFR-04, NFR-08
- **Related:** [ADR-0024](0024-a-run-viewer-is-in-scope.md),
  [ADR-0003](0003-dependencies-point-downward.md),
  [ADR-0006](0006-diagnostics-report-nan-not-zero.md),
  [ADR-0014](0014-apache-2-throughout.md),
  [ADR-0016](0016-one-distribution-two-packages.md)

## Context

ADR-0024 put a run viewer in scope, in the Python package, writing a
self-contained animated SVG. It considered three options and none of them was
"emit a log format an existing robotics viewer already reads". That is a gap
rather than a rejection: the record's own closing paragraph anticipates the
viewer growing interaction, a timeline or a comparison mode and says the answer
then is a separate tool consuming the P3 event stream. It assumed we would
build that tool.

Rerun and Foxglove are the reason we should not. Both are mature, both are
built for exactly the scrubbing-and-comparing problem that ADR-0024 deferred,
and building a timeline scrubber is not what this project is for.

The question is therefore not "SVG or Rerun". It is which artefact is
*published* and which is a development convenience, and whether SlipX owns a
viewer or owns an encoder.

Neither tool can be the published artefact, for four reasons that are specific
rather than aesthetic.

**Self-containment.** VIZ-01's output renders in a browser, a README, a pull
request comment and a slide with nothing installed. An `.rrd` or an `.mcap`
needs its viewer. For a project whose teaching claim is that a student can plot
exactly why the car spun, "first install Rerun" is a materially weaker promise.

**VIZ-02 does not survive the trip.** The provenance label and the trajectory
hash have to be drawn into the image, because the picture outlives the console
line printed beside it. In a log format they can be logged as a text entity,
but they are no longer on the face of the thing that gets pasted into a slide.
That is the NFR-08 failure the requirement exists to prevent.

**`.rrd` is not an archival format, by Rerun's own account.** The promise is
that the current version loads the previous version's files; older files are
not guaranteed, and there is an open proposal for a separate `.rra` archive
format precisely because `.rrd` is not one. A project that keys reference
hashes to builds and treats a record as a snapshot of its own date cannot
publish a run artefact in a format with a one-version compatibility window.

**Foxglove Studio is proprietary.** The open-source edition was discontinued
and the viewer is closed. This is not an NFR-01 violation, because a viewer the
user installs is not a dependency of the distribution, and it is worth stating
plainly rather than discovering later. MCAP is unaffected and is the part that
matters: the format, the libraries and the CLI are Apache-2.0 and actively
developed.

There is also a release-surface cost that applies to any SDK dependency.
ADR-0018's matrix is CPython 3.9 to 3.13 across manylinux_2_28 x86_64 and
aarch64, macOS x86_64 and arm64, and Windows AMD64. A hard dependency has to
resolve on all of them, gains an entry in `licence_scan.py`, and contradicts
the thesis of a library whose product is a core with no dependencies.

## Decision

SlipX emits runs to **sinks**. It does not own an interactive viewer.

`slipx.viz` is unchanged and keeps VIZ-01 to VIZ-06 in full: the self-contained
animated SVG, standard library only, provenance drawn in, no track. That is the
published artefact and the thing a reader is shown first.

Alongside it, the Python package grows a sink interface: a small protocol taking
recorded states, diagnostics and the run manifest, with one implementation per
output format. `slipx.viz`'s SVG writer is one implementation of it and is not
privileged in the code, only in the documentation.

**MCAP is the first sink and the only one written by default.** It is
Apache-2.0, it is designed for archival, Foxglove and a growing set of other
tooling read it, and it is what P3's structured event stream should be encoded
as in any case. Writing it commits us to a format and not to a vendor.

**Rerun is a second, optional sink.** Apache-2.0, and genuinely the better tool
for inspecting a run while developing one. There is a concrete case for it: the
load-proportional drive split was found by measuring a 2.3% steady-state radius
floor across many runs, and a per-corner longitudinal force trace would have
shown it in one.

Both SDK dependencies are **optional extras**, never install requirements.
Neither may be imported at package import time, and the core distribution must
install and pass its tests with both absent.

Two boundaries, both hard, both inherited from ADR-0024 and restated because a
sink is where they are easiest to lose.

**A sink writes a file and never opens a window.** Rerun's SDK can spawn a
viewer process; SlipX never calls it. NFR-04 stays true by construction rather
than by care, exactly as ADR-0024 arranged for the renderer.

**A sink emits nothing that is not in the recorded state, diagnostics or
manifest.** VIZ-03 is not a property of the SVG writer, it is a property of the
boundary. No track, no synthesised quantity, no interpolation presented as
data.

One consequence of ADR-0006 has to be handled per sink rather than assumed.
Diagnostics use NaN for anything a tier cannot represent, and a sink that
encodes NaN as a plotted zero produces the believable lie that record exists to
prevent. Each sink owes a test that an unrepresentable quantity arrives at the
viewer as absent, not as zero.

## Consequences

The distribution grows an optional-dependency surface it did not have. That
means extras in `pyproject.toml`, skipped tests when an extra is absent, and a
CI job that installs them, because an optional path with no CI is an optional
path that is broken. `licence_scan.py` gains entries it must be taught to
treat as optional rather than vendored.

Two more formats now track the state layout. ADR-0024 already accepted that
every state change touches the viewer and called that coupling desirable; this
multiplies it by the number of sinks. The mitigation is that sinks share one
recorder and one protocol, so a new state field is added in one place and the
sinks consume what they understand.

We now depend on somebody else's format stability for the development
experience, having refused to depend on it for the published artefact. If Rerun
breaks `.rrd` again the optional sink needs work; nothing published breaks,
which is the whole point of the split.

The SVG writer must not become the poor relation. It is the artefact in the
README and the papers, and the failure mode of this decision is that everybody
uses Rerun day to day and the SVG quietly rots. VIZ-04 and VIZ-06 are the guard:
they are released artefacts regenerated from `slipx_core`, so a rotted SVG
writer is a visible failure and not a private inconvenience.

This does not supersede ADR-0024. That record's decision stands in full, and
this one answers a question it did not ask. What it does amend is 0024's
closing paragraph: the eventual interactive viewer is not a thing we build and
move to its own repository, it is a thing that already exists and reads our
output. Reversing *this* record means either dropping the sinks and going back
to one hard-coded writer, or the opposite, adopting a viewer as the primary
surface and demoting the SVG, which would need a record superseding both.
