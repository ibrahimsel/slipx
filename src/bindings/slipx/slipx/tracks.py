# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Loading a track directory into the scene's Track (ADR-0049).

The same join `cars.py` performs for the car: ``slipx_schema`` validates the
directory (manifest, version gate, provenance), and the scene's C++ ``Track``
is built from what it validated. The surface-to-tyre refusal runs on both
sides on purpose: the Python check names the offending tyre in the loader's
vocabulary, and the C++ build enforces the same rule for whoever bypasses
Python entirely.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Tuple

from ._slipx import SceneTrack

__all__ = ["load_scene_track"]


def load_scene_track(
    directory: str | Path,
    tyre_pairs: Iterable[Tuple[str, str]],
) -> SceneTrack:
    """Load a validated track directory into a :class:`SceneTrack`.

    ``tyre_pairs`` is the list of ``(compound, surface)`` pairs the cars
    bring, usually read from their tyre files; a track whose declared
    surface no tyre was identified on is refused rather than run on the
    wrong friction. Pass one pair per axle or a deduplicated set; the check
    only asks that every pair matches the surface.
    """
    try:
        import slipx_schema
    except ImportError as exc:  # pragma: no cover - environment-specific
        raise ImportError(
            "loading a track directory needs slipx_schema, which is not "
            "installed. The scene itself does not: build a SceneTrack "
            "directly from a centreline CSV and manifest fields if you "
            "would rather not depend on the parser."
        ) from exc

    track = slipx_schema.load_track(directory)
    return SceneTrack.build(
        centreline_csv=str(track.centreline),
        name=track.name,
        surface=track.surface,
        closed=track.closed,
        geometry_source=track.geometry_source,
        geometry_licence=track.geometry_licence,
        provenance_label=track.provenance.label,
        tyre_pairs=list(tyre_pairs),
    )
