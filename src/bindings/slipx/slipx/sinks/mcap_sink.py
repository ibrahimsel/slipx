# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The MCAP sink: the default, and the one P3's event stream will reuse.

MCAP by default (SINK-02) because the format, the reference libraries and the
CLI are Apache-2.0, because it is designed for archival rather than for
whichever version of one viewer is current, and because a structured event
stream (P3) should be encoded as the same thing rather than as a second format
nobody has tooling for. Adopting it commits us to a format and not to a vendor:
Foxglove reads it, so do a growing set of other tools, and the CLI can inspect
a file with nothing else installed.

The messages are JSON with a JSON Schema per channel. Not protobuf, and the
reason is SINK-05. JSON has no NaN, so a quantity a tier cannot represent is
written by *leaving the key out*, which is exactly the semantics required and
is not a convention a reader has to be told about. A protobuf message with
`double fz_fl = 3;` has no absent, only zero, and zero is the believable lie
ADR-0006 exists to prevent. The encoder is asked for strict JSON
(``allow_nan=False``), so if a NaN ever reaches it the write fails loudly
instead of emitting a token no other JSON parser accepts.

The channel and schema layout is not a public interface yet. P3's event stream
has to live in the same file, and fixing the topic names before knowing what
the events look like would mean either breaking them or working around them.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Tuple, Union

from .protocol import SinkUnavailable, resolve_path
from .recording import (
    DIAGNOSTIC_COLUMNS,
    FLAG_COLUMNS,
    STATE_COLUMNS,
    AgentRecord,
    Recording,
)

_ABSENT = (
    "A key that is absent is a quantity the tier that produced this run "
    "cannot represent. It is not zero and must not be plotted as zero "
    "(ADR-0006, SINK-05)."
)


def _import_writer():
    """Import the encoder, here rather than at module level (SINK-03)."""
    try:
        from mcap.writer import CompressionType, Writer
    except ImportError as exc:  # pragma: no cover - depends on the install
        raise SinkUnavailable(
            "writing MCAP needs the `mcap` package, which SlipX does not "
            "install by default: both SDK-backed sinks are optional extras "
            "(SINK-03). Install it with `pip install \"slipx[mcap]\"`."
        ) from exc
    return Writer, CompressionType


def _nest(flat: Mapping[str, Any]) -> Dict[str, Any]:
    """``{"Fz.FL": 8.6}`` becomes ``{"Fz": {"FL": 8.6}}``.

    One level, which is all the recorder's names use. Nested rather than left
    as dotted keys because a viewer treats a dot in a message path as a
    descent, and a key with a dot in it then addresses nothing.
    """
    out: Dict[str, Any] = {}
    for name, value in flat.items():
        head, _, tail = name.partition(".")
        if tail:
            out.setdefault(head, {})[tail] = value
        else:
            out[head] = value
    return out


def _schema_for(columns: Iterable[str], title: str, description: str) -> bytes:
    """A JSON Schema over the full column layout, with nothing required.

    Generated from the recorder's column list rather than written out, so a new
    state field is added in one place (recording.py) and arrives here without
    an edit. Nothing is in ``required``: absence is the whole point.
    """
    properties: Dict[str, Any] = {}
    for name in columns:
        head, _, tail = name.partition(".")
        kind = "boolean" if head.endswith("saturated") else "number"
        if tail:
            group = properties.setdefault(
                head,
                {"type": "object", "properties": {}, "additionalProperties": False},
            )
            group["properties"][tail] = {"type": kind}
        else:
            properties[head] = {"type": kind}

    document = {
        "$schema": "http://json-schema.org/draft-07/schema#",
        "title": title,
        "description": f"{description} {_ABSENT}",
        "type": "object",
        "properties": properties,
        "additionalProperties": False,
    }
    return json.dumps(document, indent=2).encode("utf-8")


def _encode(payload: Mapping[str, Any]) -> bytes:
    # allow_nan=False on purpose. Python's json writes a bare NaN token by
    # default, which no other JSON parser accepts and which would put the
    # unrepresentable quantity back into the file under a different disguise.
    return json.dumps(payload, allow_nan=False).encode("utf-8")


class McapSink:
    """Write a run as one MCAP file (SINK-02).

    Two channels per agent, ``state`` and ``diagnostics``, plus one
    ``provenance`` channel carrying the NFR-08 label so that a reader who opens
    the file and nothing else still sees what the numbers are worth.

    Writes a file and stops. Nothing here starts a process, opens a socket or
    launches a viewer (SINK-04): reading the result is the reader's business
    and their choice of tool.
    """

    suffix = ".mcap"

    def __init__(self, *, topic_prefix: str = "/slipx", compression: str = "zstd"):
        self._writer_class, compression_types = _import_writer()
        self._prefix = topic_prefix.rstrip("/")
        try:
            self._compression = {
                "zstd": compression_types.ZSTD,
                "lz4": compression_types.LZ4,
                "none": compression_types.NONE,
            }[compression]
        except KeyError:
            raise ValueError(
                f"unknown compression {compression!r}: zstd, lz4 or none"
            ) from None

    def write(self, recording: Recording, path: Union[str, Path]) -> Path:
        resolved = resolve_path(path, self.suffix)

        with resolved.open("wb") as stream:
            writer = self._writer_class(stream, compression=self._compression)
            writer.start(
                profile="slipx",
                library=f"slipx {recording.slipx_version}",
            )
            self._write_metadata(writer, recording)
            channels = self._register(writer, recording)
            self._write_messages(writer, recording, channels)
            writer.finish()

        return resolved

    # ------------------------------------------------------------ internals

    def _write_metadata(self, writer, recording: Recording) -> None:
        """Provenance and the run manifest, as metadata records.

        NFR-08 is why the label is here and on a channel and in every channel's
        own metadata. A picture outlives the console line printed beside it,
        and so does a log file; a reader who finds this on a shared drive in a
        year has to be able to tell that no parameter set shipped with SlipX
        has been validated against a real car.
        """
        writer.add_metadata(
            "slipx",
            {
                "provenance": recording.provenance,
                "provenance_line": recording.provenance_line(),
                "trajectory_hash": recording.trajectory_hash,
                "slipx_version": recording.slipx_version,
                "slipx_core_version": recording.core_version,
                "schema_version": recording.schema_version,
                "integrator": recording.integrator,
                "git_sha": recording.git_sha,
                "dt": repr(recording.dt),
                "stride": str(recording.stride),
                "frames": str(len(recording)),
                "absent_means": _ABSENT,
            },
        )
        # The manifest whole and unedited, because a digest of a document
        # nobody kept is not evidence of anything (NFR-08, ADR-0013).
        writer.add_metadata("slipx.manifest", {"json": recording.manifest_json})

    def _register(self, writer, recording: Recording) -> Dict[int, Tuple[int, int]]:
        state_schema = writer.register_schema(
            name="slipx.VehicleState",
            encoding="jsonschema",
            data=_schema_for(
                STATE_COLUMNS,
                "slipx.VehicleState",
                "The recorded vehicle state, ISO 8855 and SI throughout.",
            ),
        )
        diagnostic_schema = writer.register_schema(
            name="slipx.StepDiagnostics",
            encoding="jsonschema",
            data=_schema_for(
                tuple(DIAGNOSTIC_COLUMNS) + tuple(FLAG_COLUMNS),
                "slipx.StepDiagnostics",
                "Per-step diagnostics: slip, tyre forces, loads and saturation.",
            ),
        )

        channels: Dict[int, Tuple[int, int]] = {}
        for agent in recording.agents:
            metadata = self._agent_metadata(recording, agent)
            channels[agent.index] = (
                writer.register_channel(
                    topic=f"{self._prefix}/{agent.name}/state",
                    message_encoding="json",
                    schema_id=state_schema,
                    metadata=metadata,
                ),
                writer.register_channel(
                    topic=f"{self._prefix}/{agent.name}/diagnostics",
                    message_encoding="json",
                    schema_id=diagnostic_schema,
                    metadata=metadata,
                ),
            )
        return channels

    def _agent_metadata(
        self, recording: Recording, agent: AgentRecord
    ) -> Dict[str, str]:
        return {
            "slipx.agent": agent.name,
            "slipx.tier": agent.tier,
            "slipx.provenance": agent.provenance,
            "slipx.params_digest": agent.params_digest,
            "slipx.seed": str(agent.seed),
            "slipx.trajectory_hash": agent.trajectory_hash,
            "slipx.run_trajectory_hash": recording.trajectory_hash,
        }

    def _write_messages(
        self, writer, recording: Recording, channels: Dict[int, Tuple[int, int]]
    ) -> None:
        provenance_schema = writer.register_schema(
            name="slipx.Provenance",
            encoding="jsonschema",
            data=json.dumps(
                {
                    "$schema": "http://json-schema.org/draft-07/schema#",
                    "title": "slipx.Provenance",
                    "description": (
                        "How the parameters behind this run were obtained "
                        "(NFR-08). No parameter set shipped with SlipX has "
                        "been validated against a real car."
                    ),
                    "type": "object",
                    "properties": {
                        "text": {"type": "string"},
                        "provenance": {"type": "string"},
                        "trajectory_hash": {"type": "string"},
                    },
                },
                indent=2,
            ).encode("utf-8"),
        )
        provenance_channel = writer.register_channel(
            topic=f"{self._prefix}/provenance",
            message_encoding="json",
            schema_id=provenance_schema,
        )

        first = _nanoseconds(recording.times[0])
        writer.add_message(
            channel_id=provenance_channel,
            log_time=first,
            publish_time=first,
            sequence=0,
            data=_encode(
                {
                    "text": recording.provenance_line(),
                    "provenance": recording.provenance,
                    "trajectory_hash": recording.trajectory_hash,
                }
            ),
        )

        for frame, time in enumerate(recording.times):
            stamp = _nanoseconds(time)
            for agent in recording.agents:
                state_channel, diagnostic_channel = channels[agent.index]
                writer.add_message(
                    channel_id=state_channel,
                    log_time=stamp,
                    publish_time=stamp,
                    sequence=frame,
                    data=_encode(_nest(agent.state_frame(frame))),
                )
                diagnostics = agent.diagnostics_frame(frame)
                diagnostics.update(agent.flags_frame(frame))
                writer.add_message(
                    channel_id=diagnostic_channel,
                    log_time=stamp,
                    publish_time=stamp,
                    sequence=frame,
                    data=_encode(_nest(diagnostics)),
                )


def _nanoseconds(time: float) -> int:
    """Simulation time as an integer log time.

    Simulation time is steps * dt and never an accumulated sum, so this is a
    rounding of an exact quantity rather than of a drifted one.
    """
    return int(round(time * 1e9))
