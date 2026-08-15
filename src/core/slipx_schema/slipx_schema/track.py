# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Reading a track directory (ADR-0034, ADR-0036).

A track directory is a manifest and a centreline CSV beside it::

    paddock_square/
      track.yaml
      centreline.csv

This module reads the manifest. It deliberately does not read the CSV.

That division is worth stating plainly, because the obvious thing to do here
is parse the geometry too. ``slipx_scene`` already parses it, in C++, and a
second parser in Python would be a second set of rules about what a track file
may contain, drifting away from the first at whatever rate the two are edited.
The manifest is where the rules are, so the manifest is what this reads; the
geometry is opened only far enough to check that it is there, so that a
manifest naming a file nobody shipped fails at load rather than at run.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List

from .errors import SurfaceMismatchError, TrackDirectoryError, ValidationError
from .model import Provenance
from .validate import validate_document

#: The manifest's filename inside a track directory. A convention, like
#: ``car.yaml``; the loader accepts a direct path to a manifest too.
TRACK_MANIFEST = "track.yaml"


@dataclass(frozen=True)
class Track:
    """A validated track manifest.

    Attributes:
        centreline: Absolute path to the geometry. It exists, and nothing here
            has read it: what the numbers in it are is ``slipx_scene``'s
            question.
        geometry_licence: The terms the geometry arrived under. SlipX ships
            none of it, so this is somebody else's licence in every case that
            is not a generated track, and it is required for that reason.
    """

    name: str
    schema_version: str
    directory: Path
    surface: str
    closed: bool
    centreline: Path
    geometry_source: str
    geometry_licence: str
    provenance: Provenance
    geometry_retrieved: str = ""
    geometry_notes: str = ""
    description: str = ""
    raw: Dict[str, Any] = field(default_factory=dict)

    def summary(self) -> str:
        """What tooling prints when it loads a track.

        The provenance label and the geometry licence both appear, for the
        same reason: neither is something a reader should have to go and look
        up before deciding whether they may publish what they just ran.
        """
        shape = "closed" if self.closed else "open"
        lines = [
            f"{self.name} (schema {self.schema_version})",
            f"  provenance: {self.provenance.summary()}",
            f"  surface {self.surface}, {shape}",
            f"  geometry: {self.geometry_source} [{self.geometry_licence}]",
        ]
        if self.geometry_retrieved:
            lines.append(f"  retrieved: {self.geometry_retrieved}")
        return "\n".join(lines)


def _manifest_path(path: Path) -> Path:
    """Accept either a track directory or the manifest itself."""
    path = Path(path)
    if path.is_dir():
        manifest = path / TRACK_MANIFEST
        if not manifest.exists():
            raise TrackDirectoryError(
                f"{path} is not a track directory: it has no {TRACK_MANIFEST}. "
                f"A track is a manifest and a centreline CSV beside it."
            )
        return manifest
    if not path.exists():
        raise TrackDirectoryError(f"{path} does not exist")
    return path


def load_track(path: Path | str) -> Track:
    """Read and validate a track directory, or a manifest by path.

    Raises:
        TrackDirectoryError: the directory or the centreline it names is not
            there.
        SchemaVersionError: the manifest declares a version this parser will
            not read (SCH-01).
        ValidationError: the manifest is not a valid track document.
    """
    # Imported here rather than at module scope: loader imports this module's
    # sibling model and errors, and going through the package root at import
    # time would make the two files import each other.
    from .loader import _gate_version, _provenance_from, _read_yaml

    manifest = _manifest_path(Path(path))
    directory = manifest.parent

    document = _read_yaml(manifest)
    document = _gate_version(document, "track", manifest)

    errors = validate_document("track", document, file=str(manifest))
    if errors:
        raise ValidationError(errors, context=str(manifest))

    geometry = document["geometry"]
    centreline = (directory / document["centreline"]).resolve()
    if not centreline.exists():
        raise TrackDirectoryError(
            f"{manifest}: names the centreline {document['centreline']}, "
            f"which is not in {directory}. SlipX ships no track geometry, so "
            f"a converted track that was moved without its CSV looks exactly "
            f"like this."
        )

    return Track(
        name=document["name"],
        schema_version=document["schema_version"],
        directory=directory,
        surface=document["surface"],
        closed=document["closed"],
        centreline=centreline,
        geometry_source=geometry["source"],
        geometry_licence=geometry["licence"],
        geometry_retrieved=geometry.get("retrieved", ""),
        geometry_notes=geometry.get("notes", ""),
        provenance=_provenance_from(document["provenance"]),
        description=document.get("description", ""),
        raw=document,
    )


def check_surface(track: Track, tyres: List[Any]) -> None:
    """Refuse a car whose tyres were not identified on this track's surface.

    ``tyres`` is the tyres the run will be driven on, normally a car's front
    and rear entries, and every one of them has to be for this surface. The
    check is over all of them rather than looking for one match, because a car
    with one axle on carpet and one on asphalt is precisely the mistake that a
    search for one match waves through.

    ``slipx_scene`` makes the same check in C++ when a Track is built. This
    one exists so that a Python tool sweeping cars across tracks finds out
    before it constructs anything, and so the message can name the tyre files.

    Raises:
        SurfaceMismatchError: naming the surface, the offending tyre and
            everything that was offered.
    """
    if not tyres:
        raise SurfaceMismatchError(
            f"track \"{track.name}\" is {track.surface} and no tyres were "
            f"offered. Friction comes from a (compound, surface) tyre file or "
            f"the run does not start."
        )

    offered = ", ".join(f"({t.compound}, {t.surface})" for t in tyres)
    for tyre in tyres:
        if tyre.surface != track.surface:
            raise SurfaceMismatchError(
                f"track \"{track.name}\" declares the surface "
                f"\"{track.surface}\", and the tyre ({tyre.compound}, "
                f"{tyre.surface}) is not for it. Offered: {offered}. Fit a "
                f"tyre identified on \"{track.surface}\", or run on a track "
                f"whose surface these tyres were measured on."
            )
