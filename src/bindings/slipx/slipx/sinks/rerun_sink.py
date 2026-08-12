# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The Rerun sink: optional, and the better tool for looking at a run you are
still developing (SINK-03).

Apache-2.0, and there is a concrete case for it rather than a general
preference: L2's load-proportional drive split was found by measuring a 2.3%
steady-state radius floor across many runs, and a per-corner longitudinal force
trace would have shown it in one.

It is a second sink and not the published artefact, and ADR-0028 gives the four
reasons. The one that governs this file is that ``.rrd`` is not an archival
format by Rerun's own account: the promise is that the current version loads
the previous version's files, which is a one-version compatibility window and
not something a project that treats a record as a snapshot of its own date can
publish runs in. So this writes a development convenience. MCAP writes the
thing you keep.

Two things this file does not do, both deliberate.

It never opens a window (SINK-04). The SDK will spawn a viewer, connect to a
running one or serve over a socket, and none of that is called here: a
:class:`~rerun.RecordingStream` is constructed, a file sink is attached, the
columns go in and the stream closes. NFR-04 then holds by construction.

It never sends a quantity the run did not contain (SINK-05). The recording
carries NaN for anything the tier could not represent, and a NaN scalar in a
time series is not absent, it is a point Rerun will happily place a gap or a
marker at depending on the view. So the columns are filtered before they are
sent, frame by frame against their own time index, and a column with nothing
left in it is not sent at all. That is why :func:`column_plan` is a pure
function with no SDK in it: what is sent is decidable, and therefore testable,
without asking the SDK what it received.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterator, List, Mapping, Sequence, Tuple, Union

from .protocol import SinkUnavailable, resolve_path
from .recording import AgentRecord, Recording

#: (entity path, times, values) for one column that has something in it.
Column = Tuple[str, Tuple[float, ...], Tuple[float, ...]]


def _import_rerun():
    """Import the SDK, here rather than at module level (SINK-03)."""
    try:
        import rerun
    except ImportError as exc:  # pragma: no cover - depends on the install
        raise SinkUnavailable(
            "writing a Rerun recording needs the `rerun-sdk` package, which "
            "SlipX does not install by default: both SDK-backed sinks are "
            "optional extras (SINK-03). Install it with "
            "`pip install \"slipx[rerun]\"`. MCAP is the default sink and "
            "needs no viewer to be useful."
        ) from exc
    return rerun


def _entity(agent: str, group: str, column: str) -> str:
    """``Fz.FL`` under ``car``'s state becomes ``/car/state/Fz/FL``.

    The dots become slashes because that is the same nesting the MCAP sink
    builds as a nested object, and a viewer's entity tree is where it shows up
    here. One recorder, two formats, the same shape.
    """
    return "/" + "/".join([agent, group, column.replace(".", "/")])


def _present(
    times: Sequence[float], values: Sequence[float]
) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    """Drop the frames whose value is NaN, and the times that go with them.

    Per frame rather than per column, because a column can be absent for part
    of a run: a wheel that lifts has no friction budget to invert a slip ratio
    from, and the diagnostic is NaN for exactly as long as it is off the
    ground. Sending those frames would draw the wheel back onto the road.
    """
    kept_times: List[float] = []
    kept_values: List[float] = []
    for time, value in zip(times, values):
        if value == value:  # False for NaN, and for nothing else
            kept_times.append(time)
            kept_values.append(value)
    return tuple(kept_times), tuple(kept_values)


def column_plan(recording: Recording, agent: AgentRecord) -> Iterator[Column]:
    """Every column this sink will send for one agent, and nothing else.

    Pure: no SDK, no file, no side effect. This is the whole of the SINK-05
    decision, so it is a function a test can call and compare against what the
    tier is entitled to report.
    """
    groups: Sequence[Tuple[str, Mapping[str, Tuple[float, ...]]]] = (
        ("state", agent.state),
        ("diagnostics", agent.diagnostics),
    )
    for group, columns in groups:
        for name, values in columns.items():
            times, kept = _present(recording.times, values)
            if kept:
                yield _entity(agent.name, group, name), times, kept

    # Saturation flags as zero and one. A bool has no NaN and is never absent,
    # so it needs no filtering; plotting it as a scalar is how a flag is read
    # against the traces it explains.
    for name, flags in agent.flags.items():
        yield (
            _entity(agent.name, "diagnostics", name),
            tuple(recording.times),
            tuple(1.0 if flag else 0.0 for flag in flags),
        )


class RerunSink:
    """Write a run as one ``.rrd`` file (SINK-03).

    One scalar time series per recorded column, under an entity tree that
    mirrors the state layout, plus the provenance label and the run manifest as
    static text so that a reader who opens the recording sees what the numbers
    are worth (NFR-08).

    Writes a file and stops (SINK-04).
    """

    suffix = ".rrd"

    def __init__(
        self,
        *,
        application_id: str = "slipx",
        timeline: str = "sim_time",
    ):
        self._rerun = _import_rerun()
        self._application_id = application_id
        self._timeline = timeline

    def write(self, recording: Recording, path: Union[str, Path]) -> Path:
        resolved = resolve_path(path, self.suffix)
        rerun = self._rerun

        # recording_id from the trajectory hash, so two encodings of one run
        # are one recording and two different runs never collide.
        #
        # send_properties=False because the SDK would otherwise offer to write
        # a start-of-recording wall clock into the file. A wall clock is not in
        # the recorded state, the diagnostics or the manifest (SINK-05), and it
        # would make two encodings of the same run differ for a reason that has
        # nothing to do with the run.
        #
        # Belt and braces, and measured as such: with rerun-sdk 0.36 the
        # properties do not reach the file anyway, because the file sink is
        # attached below before anything is logged. Flipping this flag alone
        # changes nothing today. Both are kept, because which of the two is
        # load-bearing is the SDK's business and not ours, and the test that
        # matters asserts the file rather than either mechanism.
        stream = rerun.RecordingStream(
            application_id=self._application_id,
            recording_id=recording.trajectory_hash,
            send_properties=False,
        )
        with stream:
            stream.save(resolved)
            self._write_provenance(stream, recording)
            for agent in recording.agents:
                self._write_agent(stream, recording, agent)

        return resolved

    # ------------------------------------------------------------ internals

    def _write_provenance(self, stream, recording: Recording) -> None:
        rerun = self._rerun
        stream.log(
            "/provenance",
            rerun.TextDocument(
                _provenance_document(recording),
                media_type=rerun.MediaType.MARKDOWN,
            ),
            static=True,
        )

    def _write_agent(self, stream, recording: Recording, agent) -> None:
        rerun = self._rerun
        for entity, times, values in column_plan(recording, agent):
            stream.send_columns(
                entity,
                indexes=[rerun.TimeColumn(self._timeline, duration=list(times))],
                columns=rerun.Scalars.columns(scalars=list(values)),
            )


def _provenance_document(recording: Recording) -> str:
    """The NFR-08 label, the hashes and the manifest, as one text entity.

    ADR-0028 is explicit that this is weaker than VIZ-02's label drawn into the
    picture, and that the weakness is one of the reasons a log format is not
    the published artefact. It is still the strongest thing a log format
    offers, so it is done properly: the whole manifest, not a digest of a
    document nobody kept.
    """
    lines = [
        "# SlipX run",
        "",
        recording.provenance_line(),
        "",
        "No parameter set shipped with SlipX has been validated against a "
        "real car. The honest phrasing for anything built on these numbers is "
        "\"physically structured and identifiable\", not \"validated\".",
        "",
        "A quantity a tier cannot represent is absent from this recording, "
        "not zero (ADR-0006).",
        "",
        "| agent | tier | provenance | parameters | trajectory |",
        "|---|---|---|---|---|",
    ]
    for agent in recording.agents:
        lines.append(
            f"| {agent.name} | {agent.tier} | {agent.provenance} "
            f"| {agent.params_digest} | {agent.trajectory_hash} |"
        )
    lines.extend(["", "## Run manifest", "", "```json", recording.manifest_json, "```"])
    return "\n".join(lines)
