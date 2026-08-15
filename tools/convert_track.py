#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Convert somebody else's centreline into a SlipX track directory (ADR-0035).

SlipX ships no third-party track geometry. Every public centreline set in this
format is copyleft, and the F1TENTH maps of real venues carry no licence at
all, so a real track cannot be redistributed here by us. It can be fetched by
the person who wants it, which is what this does.

    python3 tools/convert_track.py \\
        --source https://raw.githubusercontent.com/f1tenth/f1tenth_racetracks/main/Spielberg/Spielberg_centerline.csv \\
        --name spielberg --surface asphalt --closed \\
        --out ~/tracks/spielberg

What it writes is a track directory: the centreline, unchanged in shape, and a
manifest recording where the geometry came from, under what licence, and on
what date. The licence is written into the file rather than left for the user
to remember, because an obligation nobody was told about is an obligation
nobody meets.

Three things this tool will not do:

  It will not guess the surface. Friction is resolved from a (compound,
  surface) tyre file, so the surface identifier decides which measurement
  applies to a run, and a default would silently pick one.

  It will not guess whether the track closes. A track that starts and
  finishes near the same place may be a lap or may be a straight, and a
  tolerance on the closing distance would quietly decide.

  It will not write a copyleft or unlicensed track into the SlipX repository.
  That is the whole point of the decision this implements, and a rule enforced
  by a note in a document is a rule that lasts until somebody is in a hurry.

This tool is not part of any run. Nothing in a simulation, a sink or a
manifest fetches anything; determinism and a network are not compatible and
this is the only file in the tree that touches one.
"""

from __future__ import annotations

import argparse
import datetime
import sys
import urllib.request
from pathlib import Path
from typing import List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent

#: Licences under which geometry may be redistributed inside this repository.
#: Deliberately short. Anything else is somebody else's to publish, not ours.
PERMISSIVE = {"apache-2.0", "mit", "bsd-2-clause", "bsd-3-clause", "cc0-1.0"}

#: What the public sources are actually licensed under, checked rather than
#: recalled. A user should not have to go and read a LICENSE file to find out
#: that the track they just converted cannot be published with their result.
#:
#: Keyed by a substring of the URL. The dates these were checked are in
#: ADR-0035; a repository can be relicensed, so --licence overrides this.
KNOWN_SOURCES: List[Tuple[str, str, str]] = [
    (
        "f1tenth/f1tenth_racetracks",
        "GPL-3.0",
        "Centrelines derived from the TUM racetrack database, which is in "
        "turn derived from OpenStreetMap, so ODbL share-alike applies to the "
        "geometry as a database as well.",
    ),
    (
        "TUMFTM/racetrack-database",
        "LGPL-3.0",
        "Centrelines fetched from OpenStreetMap and smoothed, so ODbL "
        "share-alike applies to the geometry as a database as well.",
    ),
    (
        "f1tenth/f1tenth_simulator",
        "none stated",
        "This repository states no licence, so it grants no permission to "
        "redistribute. Convert it for your own use only.",
    ),
    (
        "CPS-TUWien/f1tenth_maps",
        "none stated",
        "This repository states no licence, so it grants no permission to "
        "redistribute. Convert it for your own use only.",
    ),
]


def identify(source: str) -> Tuple[Optional[str], str]:
    """The licence of a known source, and what else is worth knowing about it."""
    for fragment, licence, note in KNOWN_SOURCES:
        if fragment.lower() in source.lower():
            return licence, note
    return None, ""


def is_permissive(licence: str) -> bool:
    return licence.strip().lower() in PERMISSIVE


def check_destination(out: Path, licence: str, name: str) -> None:
    """Refuse to write geometry we may not redistribute into this repository.

    The rule it enforces is ADR-0035, and it is enforced here rather than
    described in a document because a rule that lives only in a document lasts
    until somebody is in a hurry. ``licence_scan.py`` would not catch this: a
    CSV of coordinates carries no licence text for it to find.
    """
    if is_permissive(licence):
        return
    if REPO_ROOT not in out.parents and out != REPO_ROOT:
        return
    raise SystemExit(
        f"refusing to write {licence} geometry into the SlipX repository at "
        f"{out}.\n"
        f"SlipX is Apache-2.0 and ships no third-party track geometry "
        f"(ADR-0035); a copyleft or unlicensed CSV in this tree would end the "
        f"embedding claim the whole project rests on. Convert it somewhere "
        f"else, for example --out ~/tracks/{name}."
    )


def fetch(source: str, timeout: float) -> str:
    """Read the source, from a URL or from a path already on this machine."""
    if source.startswith("http://"):
        raise SystemExit(
            "refusing to fetch over http. Use https, or download the file "
            "yourself and pass the path."
        )
    if source.startswith("https://"):
        with urllib.request.urlopen(source, timeout=timeout) as response:
            return response.read().decode("utf-8", errors="replace")

    path = Path(source).expanduser()
    if not path.exists():
        raise SystemExit(f"{source}: no such file, and it is not an https URL")
    return path.read_text(encoding="utf-8", errors="replace")


def convert(text: str, source: str) -> List[Tuple[float, float, float, float]]:
    """Read a four-column centreline out of ``text``.

    This parses geometry, which the SlipX loader in ``slipx_schema``
    deliberately does not: the loader's job is the manifest, and the geometry
    belongs to ``slipx_scene``. The difference is that this is reading a
    foreign file whose shape is not ours to define, which is what a converter
    is for.

    The output columns are the input columns. Nothing is smoothed, resampled
    or reordered, so a track converted here is the track that was published;
    if it needs fixing, it needs fixing upstream where somebody else can
    benefit from the fix.
    """
    rows: List[Tuple[float, float, float, float]] = []
    for number, line in enumerate(text.splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = [f.strip() for f in stripped.replace(";", ",").split(",")]
        if len(fields) != 4:
            raise SystemExit(
                f"{source}:{number}: expected 4 comma-separated fields "
                f"(x_m, y_m, w_tr_right_m, w_tr_left_m), found {len(fields)}. "
                f"A raceline file has seven columns and is not a centreline; "
                f"this tool wants the centreline."
            )
        try:
            values = tuple(float(f) for f in fields)
        except ValueError as exc:
            raise SystemExit(f"{source}:{number}: {exc}") from exc
        rows.append(values)  # type: ignore[arg-type]

    if len(rows) < 2:
        raise SystemExit(f"{source}: found {len(rows)} points, which is not a track")
    return rows


def drop_repeated_last_point(
    rows: List[Tuple[float, float, float, float]]
) -> Tuple[List[Tuple[float, float, float, float]], bool]:
    """Remove a final point that repeats the first.

    Published closed centrelines differ on this: some repeat the start as the
    last row to make the loop explicit, some do not. SlipX declares closure in
    the manifest and derives the closing chord, so a repeated start is a
    zero-length segment and the loader refuses it. Dropping it is a change to
    the file, so it is reported rather than done quietly.
    """
    if len(rows) > 2 and rows[0][:2] == rows[-1][:2]:
        return rows[:-1], True
    return rows, False


def write_track(
    out: Path,
    rows: List[Tuple[float, float, float, float]],
    name: str,
    surface: str,
    closed: bool,
    source: str,
    licence: str,
    note: str,
) -> None:
    out.mkdir(parents=True, exist_ok=True)
    today = datetime.date.today().isoformat()

    csv_lines = [
        f"# Converted from {source}",
        f"# Geometry licence: {licence}. Not SlipX's to relicense.",
        "# x_m,y_m,w_tr_right_m,w_tr_left_m",
    ]
    for x, y, w_right, w_left in rows:
        csv_lines.append(f"{x:.6f},{y:.6f},{w_right:.6f},{w_left:.6f}")
    (out / "centreline.csv").write_text(
        "\n".join(csv_lines) + "\n", encoding="utf-8", newline="\n"
    )

    notes = note or "No note recorded for this source."
    manifest = f"""# Converted track. NOT generated by SlipX and not SlipX's to relicense.
#
# The geometry below came from somebody else, under the licence stated in the
# geometry block. SlipX is Apache-2.0 and redistributes no track geometry
# (ADR-0035); this directory is yours, and publishing a result run on it means
# publishing this directory under the geometry's own terms.

schema_version: "0.3.0"
name: {name}
description: >-
  Converted by tools/convert_track.py from a published centreline. The
  geometry is unchanged: nothing was smoothed, resampled or reordered.

surface: {surface}
closed: {"true" if closed else "false"}

centreline: centreline.csv

geometry:
  source: >-
    {source}
  licence: {licence}
  retrieved: "{today}"
  notes: >-
    {notes}

provenance:
  schema_version: "0.3.0"
  label: provisional
  source: >-
    {source}
  method: >-
    Converted, not measured. The geometry is as published by the source above;
    whatever survey or tracing produced it is that project's to describe.
  date: "{today}"
  notes: >-
    The surface identifier was supplied on the command line and describes the
    venue this geometry represents, not anything measured from it.
"""
    (out / "track.yaml").write_text(manifest, encoding="utf-8", newline="\n")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--source", required=True,
                        help="https URL or local path of a four-column centreline CSV")
    parser.add_argument("--name", required=True,
                        help="track identifier: lower case, no spaces")
    parser.add_argument("--surface", required=True,
                        help="surface identifier, e.g. asphalt or carpet. Not "
                             "guessed: it decides which tyre measurement applies")
    parser.add_argument("--out", required=True, type=Path,
                        help="track directory to write")
    parser.add_argument("--licence",
                        help="SPDX identifier of the geometry's licence. Looked "
                             "up for known sources; required for anything else")
    parser.add_argument("--timeout", type=float, default=30.0)

    closure = parser.add_mutually_exclusive_group(required=True)
    closure.add_argument("--closed", action="store_true",
                         help="the centreline is a lap")
    closure.add_argument("--open", action="store_true",
                         help="the centreline starts and finishes in different places")

    args = parser.parse_args()

    known_licence, note = identify(args.source)
    licence = args.licence or known_licence
    if licence is None:
        raise SystemExit(
            f"{args.source} is not a source this tool knows the licence of, "
            f"and --licence was not given. Look it up and state it: a track "
            f"whose terms nobody recorded is one nobody can publish a result "
            f"on. If it genuinely has no licence, pass --licence 'none stated'."
        )
    if args.licence and known_licence and args.licence != known_licence:
        print(
            f"note: this tool has {args.source} recorded as {known_licence}, "
            f"and you passed {args.licence}. Using yours.",
            file=sys.stderr,
        )

    out = args.out.expanduser().resolve()
    permissive = is_permissive(licence)
    check_destination(out, licence, args.name)

    text = fetch(args.source, args.timeout)
    rows = convert(text, args.source)
    rows, dropped = drop_repeated_last_point(rows)

    write_track(out, rows, args.name, args.surface, args.closed,
                args.source, licence, note)

    print(f"{out}: {len(rows)} points, surface {args.surface}, "
          f"{'closed' if args.closed else 'open'}")
    if dropped:
        print("  the final point repeated the first and was dropped; closure "
              "is declared in the manifest instead")
    print(f"  geometry licence: {licence}")
    if note:
        print(f"  {note}")
    if not permissive:
        print(
            "  this geometry is not yours to redistribute under Apache-2.0. "
            "Publishing a result run on it means publishing this directory "
            "under the terms above.",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
