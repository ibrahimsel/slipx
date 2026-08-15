# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The track converter (ADR-0035).

Nothing here touches the network. The converter's fetch step is one call to
urllib and is not the interesting part; what is interesting is what it does
with a file once it has one, and in particular the two refusals that keep the
licence promise: geometry it may not redistribute never lands in this tree,
and geometry whose terms nobody recorded never gets written at all.
"""

from __future__ import annotations

from pathlib import Path

import pytest

import convert_track
from slipx_schema import load_track

REPO_ROOT = Path(__file__).resolve().parents[2]

TUM_STYLE = """# x_m,y_m,w_tr_right_m,w_tr_left_m
0.0,0.0,1.5,1.5
1.0,0.0,1.5,1.5
1.0,1.0,1.5,1.5
"""


# ------------------------------------------------------- knowing the licence


def test_the_public_sources_are_recorded_with_their_licences() -> None:
    # These are the reason ADR-0035 exists. A user should not have to go and
    # read a LICENSE file to discover that the track they just converted
    # cannot be published with their result.
    gpl, _ = convert_track.identify(
        "https://raw.githubusercontent.com/f1tenth/f1tenth_racetracks/main/x.csv"
    )
    assert gpl == "GPL-3.0"

    lgpl, note = convert_track.identify(
        "https://github.com/TUMFTM/racetrack-database/blob/master/tracks/Austin.csv"
    )
    assert lgpl == "LGPL-3.0"
    assert "OpenStreetMap" in note or "ODbL" in note

    # No licence at all is the harder stop of the two, and it is recorded as a
    # licence rather than as an absence so that it reaches the manifest.
    none, note = convert_track.identify(
        "https://github.com/f1tenth/f1tenth_simulator/blob/master/maps/porto.yaml"
    )
    assert none == "none stated"
    assert "redistribute" in note


def test_an_unknown_source_has_no_recorded_licence() -> None:
    assert convert_track.identify("https://example.invalid/track.csv") == (None, "")


# ------------------------------------------------------------ the two guards


def test_copyleft_geometry_may_not_be_written_into_this_repository() -> None:
    # The rule of ADR-0035, enforced rather than documented. licence_scan.py
    # cannot catch this one: a CSV of coordinates carries no licence text.
    target = REPO_ROOT / "examples" / "tracks" / "spielberg"

    with pytest.raises(SystemExit) as caught:
        convert_track.check_destination(target, "GPL-3.0", "spielberg")

    message = str(caught.value)
    assert "refusing" in message
    assert "ADR-0035" in message


def test_unlicensed_geometry_may_not_be_written_into_this_repository() -> None:
    target = REPO_ROOT / "examples" / "tracks" / "porto"

    with pytest.raises(SystemExit, match="refusing"):
        convert_track.check_destination(target, "none stated", "porto")


def test_copyleft_geometry_is_fine_anywhere_else(tmp_path) -> None:
    # The user's own machine is the whole point: SlipX does not redistribute
    # the geometry, and converting it for your own use is not redistribution.
    convert_track.check_destination(tmp_path / "spielberg", "GPL-3.0", "spielberg")


def test_permissive_geometry_may_live_in_the_repository() -> None:
    convert_track.check_destination(
        REPO_ROOT / "examples" / "tracks" / "generated", "Apache-2.0", "generated"
    )


# --------------------------------------------------------------- conversion


def test_a_four_column_centreline_converts() -> None:
    rows = convert_track.convert(TUM_STYLE, "test")

    assert len(rows) == 3
    assert rows[0] == (0.0, 0.0, 1.5, 1.5)


def test_a_raceline_is_refused_by_name() -> None:
    # Seven columns is the raceline file that sits next to the centreline in
    # the same repositories, and picking the wrong one is the easiest mistake
    # to make. The message says which file is wanted.
    raceline = "0.0;0.0;0.0;0.0;0.0;5.0;0.0\n"

    with pytest.raises(SystemExit, match="centreline"):
        convert_track.convert(raceline, "test")


def test_a_repeated_final_point_is_dropped_and_reported() -> None:
    # Published closed centrelines disagree about this. SlipX declares closure
    # in the manifest and derives the closing chord, so a repeated start is a
    # zero-length segment the loader refuses.
    rows = convert_track.convert(
        TUM_STYLE + "0.0,0.0,1.5,1.5\n", "test"
    )
    trimmed, dropped = convert_track.drop_repeated_last_point(rows)

    assert dropped is True
    assert len(trimmed) == 3

    untouched, dropped = convert_track.drop_repeated_last_point(rows[:3])
    assert dropped is False
    assert len(untouched) == 3


# ------------------------------------------------------------- the round trip


def test_what_the_converter_writes_is_a_track_the_loader_accepts(tmp_path) -> None:
    # The converter and the loader are the two ends of ADR-0035, and a
    # converter that writes something the loader refuses would strand every
    # user at exactly the point they wanted a track.
    out = tmp_path / "spielberg"
    rows = convert_track.convert(TUM_STYLE, "test")

    convert_track.write_track(
        out,
        rows,
        name="spielberg",
        surface="asphalt",
        closed=True,
        source="https://example.invalid/spielberg_centerline.csv",
        licence="GPL-3.0",
        note="Test fixture.",
    )

    track = load_track(out)

    assert track.name == "spielberg"
    assert track.surface == "asphalt"
    assert track.closed is True
    assert track.geometry_licence == "GPL-3.0"
    # The date is written by the converter, not by the user, because a public
    # map changes without changing its name.
    assert track.geometry_retrieved
    # And the licence reaches the summary, where somebody might read it.
    assert "GPL-3.0" in track.summary()


def test_the_written_centreline_keeps_the_geometry_it_was_given(tmp_path) -> None:
    # Nothing is smoothed, resampled or reordered. A track converted here is
    # the track that was published, so a fault in it is upstream's to fix
    # where everybody benefits.
    out = tmp_path / "keep"
    rows = convert_track.convert(TUM_STYLE, "test")

    convert_track.write_track(
        out, rows, "keep", "asphalt", True, "test", "Apache-2.0", ""
    )

    written = [
        line
        for line in (out / "centreline.csv").read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
    ]
    assert written == ["0.000000,0.000000,1.500000,1.500000",
                       "1.000000,0.000000,1.500000,1.500000",
                       "1.000000,1.000000,1.500000,1.500000"]
