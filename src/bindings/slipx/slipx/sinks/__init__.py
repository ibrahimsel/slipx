# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Emitting a run to something that can show it to you (SINK-01 to SINK-05).

SlipX owns the encoder, not the viewer. Scrubbing a timeline and comparing two
runs are solved problems, Rerun and Foxglove solve them, and building a third
timeline scrubber is not what this project is for. So a finished run is
recorded once and handed to a sink, and a sink writes a file (ADR-0028).

    import slipx
    from slipx import sinks

    sim = slipx.make_conformance_run(slipx.ConformanceSpec())
    run = sinks.record_run(sim, duration=2.0, stride=10)
    sinks.write(run, "step_steer")           # step_steer.mcap

MCAP is the default and the only format written without an extra installed
being asked for (SINK-02): it is Apache-2.0 down to the CLI, it is designed for
archival rather than for one viewer version, and it is the encoding P3's
structured event stream should use, so the two are one format and not two.

Rerun is a second sink and an optional extra (SINK-03). Both SDK-backed sinks
are optional: SlipX installs, imports and passes its tests with neither
present, and neither SDK is imported when this package is imported. That is
why the implementations are reached through :func:`sink_for` rather than
imported at the top of this file.

    pip install "slipx[mcap]"        # the default sink's encoder
    pip install "slipx[rerun]"       # the optional one
    pip install "slipx[sinks]"       # both

The SVG sink needs nothing installed at all: it is the standard library and a
string. It writes one self-contained animated document with the provenance
label and the trajectory hash drawn into the picture, which makes it the sink
for a run that is going to be looked at by somebody rather than loaded by
something.

    sinks.write(run, "step_steer", format="svg")   # step_steer.svg

No sink opens a window (SINK-04) and no sink writes anything the run did not
contain (SINK-05). Both rules are documented on :class:`RunSink`, and the
second one reduces to "NaN is absent" because
:func:`~slipx.sinks.recording.record_run` has already normalised every
quantity a tier cannot represent to NaN.
"""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import Dict, Tuple, Union

from .protocol import RunSink, SinkUnavailable, resolve_path
from .recording import (
    DIAGNOSTIC_COLUMNS,
    FLAG_COLUMNS,
    STATE_COLUMNS,
    WHEELS,
    AgentRecord,
    Recording,
    record_run,
    represented,
)

#: MCAP unless asked otherwise (SINK-02).
DEFAULT_FORMAT = "mcap"

# Format name to (module, class). A table rather than a chain of ifs, so that
# adding a format is adding a row and no format is special-cased anywhere in
# the code (SINK-01). The import is deferred to the lookup because an SDK
# imported here would be an SDK imported at package import time (SINK-03).
_FORMATS: Dict[str, Tuple[str, str]] = {
    "mcap": (".mcap_sink", "McapSink"),
    "rerun": (".rerun_sink", "RerunSink"),
    "svg": (".svg_sink", "SvgSink"),
}


def formats() -> Tuple[str, ...]:
    """Every format name :func:`sink_for` accepts, whether installed or not.

    Whether or not: a format whose extra is missing is still a format SlipX
    knows how to write, and reporting it as unknown would send somebody
    looking for a spelling mistake instead of an install.
    """
    return tuple(sorted(_FORMATS))


def sink_for(format: str = DEFAULT_FORMAT, **kwargs) -> RunSink:
    """Construct the sink for one format name.

    Raises:
        ValueError: no sink writes that format.
        SinkUnavailable: the sink exists and its optional extra is not
            installed.
    """
    try:
        module_name, class_name = _FORMATS[format]
    except KeyError:
        raise ValueError(
            f"no sink writes {format!r}. SlipX writes: "
            + ", ".join(formats())
        ) from None

    module = importlib.import_module(module_name, __name__)
    return getattr(module, class_name)(**kwargs)


def write(
    recording: Recording,
    path: Union[str, Path],
    format: str = DEFAULT_FORMAT,
    **kwargs,
) -> Path:
    """Write a recording in one format. The short way to reach a sink."""
    return sink_for(format, **kwargs).write(recording, path)


__all__ = [
    "DEFAULT_FORMAT",
    "DIAGNOSTIC_COLUMNS",
    "FLAG_COLUMNS",
    "STATE_COLUMNS",
    "WHEELS",
    "AgentRecord",
    "Recording",
    "RunSink",
    "SinkUnavailable",
    "formats",
    "record_run",
    "represented",
    "resolve_path",
    "sink_for",
    "write",
]
