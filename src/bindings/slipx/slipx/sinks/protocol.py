# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""What a sink is (SINK-01).

One protocol, one implementation per output format, and no format privileged
in the code. The SVG writer of VIZ-01 is an implementation of this and is the
published artefact only in the documentation (ADR-0028); nothing here knows
that, and nothing here should ever grow a branch on the format.
"""

from __future__ import annotations

from pathlib import Path
from typing import Union

try:  # pragma: no cover - Protocol is stdlib from 3.8, typing_extensions is not
    from typing import Protocol, runtime_checkable
except ImportError:  # pragma: no cover
    Protocol = object  # type: ignore[assignment]

    def runtime_checkable(cls):  # type: ignore[misc]
        return cls

from .recording import Recording


class SinkUnavailable(ImportError):
    """A sink exists but the library it encodes with is not installed.

    Its own type, because "you asked for a format whose extra is not
    installed" and "SlipX is broken" are different problems and a bare
    ImportError from three frames down cannot tell them apart. Both SDK-backed
    sinks are optional extras and never install requirements (SINK-03), so
    this is an ordinary outcome and not a defect.
    """


@runtime_checkable
class RunSink(Protocol):
    """Writes a :class:`~slipx.sinks.recording.Recording` to one file.

    Two hard boundaries, both restated from ADR-0028 because a sink is where
    they are easiest to lose.

    A sink **writes a file and never opens a window** (SINK-04). Where an SDK
    offers to spawn a viewer, connect to a running one or serve over a socket,
    a sink does not call it. NFR-04 then holds by construction rather than by
    care.

    A sink **emits nothing that is not in the recording** (SINK-05), and
    carries NaN through as absent rather than as zero. The recording has
    already normalised every unrepresentable quantity to NaN, so the rule at
    this boundary is uniform: if it is NaN, do not write it.
    """

    #: The file extension this format uses, including the dot.
    suffix: str

    def write(self, recording: Recording, path: Union[str, Path]) -> Path:
        """Encode ``recording`` at ``path`` and return the path written.

        Must be pure with respect to the run: encoding a recording cannot
        change it, and no sink is given anything mutable to change.
        """
        ...


def resolve_path(path: Union[str, Path], suffix: str) -> Path:
    """Give ``path`` the sink's suffix if it has none of its own."""
    resolved = Path(path)
    if not resolved.suffix:
        resolved = resolved.with_suffix(suffix)
    resolved.parent.mkdir(parents=True, exist_ok=True)
    return resolved
