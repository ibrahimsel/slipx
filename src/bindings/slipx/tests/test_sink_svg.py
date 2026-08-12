# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The SVG sink, read back out of the document it wrote.

No ``importorskip`` anywhere in this file, deliberately: this sink has no
optional extra and its whole point is that it is always available. If these
tests can be skipped on somebody's machine, something has grown a dependency.

The assertions are made against the parsed XML rather than against substrings.
A sink that believes it left a trace out and did not is exactly the failure
these rules exist for, and a regex over angle brackets is not evidence.
"""

from __future__ import annotations

import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import slipx
from slipx import sinks
from slipx.sinks import svg_sink

SVG_NS = "{http://www.w3.org/2000/svg}"


def parse(path_or_text):
    if isinstance(path_or_text, Path):
        path_or_text = path_or_text.read_text(encoding="utf-8")
    return ET.fromstring(path_or_text)


def texts(root):
    return [
        "".join(element.itertext())
        for element in root.iter(f"{SVG_NS}text")
    ]


def traces(root):
    """The polylines that are data, not the frame or the path of the run."""
    return [
        element
        for element in root.iter(f"{SVG_NS}polyline")
        if element.get("class", "").startswith("s")
    ]


# ------------------------------------------------------------------ writing


def test_the_sink_writes_one_svg_file(l1_recording, tmp_path):
    written = sinks.write(l1_recording, tmp_path / "run", format="svg")

    assert written == tmp_path / "run.svg"
    root = parse(written)
    assert root.tag == f"{SVG_NS}svg"


def test_the_sink_satisfies_the_protocol():
    sink = sinks.sink_for("svg")

    assert isinstance(sink, sinks.RunSink)
    assert sink.suffix == ".svg"


def test_the_sink_needs_nothing_outside_the_standard_library():
    """Parsed from the source, because an import that only runs on one path is
    still a dependency and only a reader of every line would find it.

    Named exactly rather than checked against a list of standard modules: the
    point is not "these happen to be stdlib today" but "this file imports
    these five things and adding a sixth is a decision".
    """
    source = Path(svg_sink.__file__).read_text(encoding="utf-8")
    absolute = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            absolute.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            absolute.add(node.module.split(".")[0])

    assert absolute == {"__future__", "math", "pathlib", "typing"}


def test_writing_the_same_run_twice_gives_the_same_bytes(l2_recording, tmp_path):
    """Byte-identical, unlike the Rerun sink, and it is a stronger promise.

    Nothing here is a container format with chunk boundaries somebody else
    decides: it is a string this project builds, so any difference between two
    encodings of one run is a defect in this file. A wall clock or an unordered
    dictionary would show up here and nowhere else.
    """
    first = sinks.write(l2_recording, tmp_path / "a", format="svg")
    second = sinks.write(l2_recording, tmp_path / "b", format="svg")

    assert first.read_bytes() == second.read_bytes()


def test_writing_a_run_does_not_change_it(step_steer_sim, tmp_path):
    sim = step_steer_sim(slipx.Tier.L2_DoubleTrack)
    recording = sinks.record_run(sim, duration=0.4, stride=10)
    before = recording.trajectory_hash

    sinks.write(recording, tmp_path / "run", format="svg")

    assert sim.trajectory_hash() == before
    assert recording.trajectory_hash == before


# -------------------------------------------------- the label is in the image


def test_the_provenance_label_and_the_hash_are_drawn_into_the_picture(
    l2_recording, tmp_path
):
    """A rendered run gets pasted into a slide with everything around it
    discarded, so the label has to survive being cropped to the image.

    Both, separately: the provenance line contains the hash, but a reader
    checking a result against a published number should find it on its own
    rather than inside a sentence.
    """
    root = parse(sinks.write(l2_recording, tmp_path / "run", format="svg"))
    drawn = texts(root)

    assert any(l2_recording.provenance_line() in line for line in drawn)
    assert any(l2_recording.provenance in line for line in drawn)

    # The hash on its own, in the monospace footer, and NOT counted by finding
    # it inside the provenance line: that line already contains it, so an
    # assertion over all the text would pass with the footer deleted. Asked of
    # the footer specifically, because retyping a hash out of a sentence set
    # in a proportional face is exactly what this element exists to avoid.
    footer = [
        "".join(element.itertext())
        for element in root.iter(f"{SVG_NS}text")
        if element.get("class") == "mono"
    ]
    assert footer
    assert any(
        l2_recording.trajectory_hash in line and "trajectory" in line
        for line in footer
    )


def test_the_hash_is_in_the_document_title_too(l2_recording, tmp_path):
    """So a browser tab, and anything that indexes the file, carries it."""
    root = parse(sinks.write(l2_recording, tmp_path / "run", format="svg"))
    title = root.find(f"{SVG_NS}title")

    assert title is not None
    assert l2_recording.trajectory_hash in title.text


# ---------------------------------------- nothing is drawn that was not run


def test_every_drawn_quantity_is_a_recorded_column(l2_recording):
    """The element inventory, against the recording's own column names.

    A name here that the recording does not have is this sink inventing a
    quantity, which is the specific failure ADR-0024 exists to prevent.
    """
    agent = l2_recording.agents[0]
    recorded = {f"state.{name}" for name in sinks.STATE_COLUMNS}
    recorded |= {f"diagnostics.{name}" for name in sinks.DIAGNOSTIC_COLUMNS}

    drawn = set(svg_sink.drawn_sources(agent))

    assert drawn <= recorded, sorted(drawn - recorded)
    # And it is not vacuously empty: the map's three columns plus every panel.
    assert {"state.pos.x", "state.pos.y", "state.yaw"} <= drawn
    assert len(drawn) > 20


def test_the_drawn_traces_are_exactly_the_planned_ones(l2_recording, tmp_path):
    """The plan is a pure function; this is the check that the document agrees
    with it, so a test may reason about the plan instead of the XML."""
    agent = l2_recording.agents[0]
    planned = sum(len(panel.series) for panel in svg_sink.panel_plan(agent))
    root = parse(sinks.write(l2_recording, tmp_path / "run", format="svg"))

    # One polyline per series, because none of these columns has a NaN gap in
    # the middle of this run. A gap would split one series into several, which
    # is why this is `>=` in spirit and `==` here: asserting the exact number
    # is what makes a spurious extra stroke visible.
    assert len(traces(root)) == planned
    assert planned > 0


def test_no_track_and_no_car_body_are_drawn(l2_recording, tmp_path):
    """ADR-0024, at the level of the elements themselves.

    A track would be a filled shape or a second path; a car body would be a
    rectangle or polygon somewhere in the plotting area. The only rectangles
    in the document are the background card and the panel frames, and the only
    filled marker is the position cursor.
    """
    root = parse(sinks.write(l2_recording, tmp_path / "run", format="svg"))

    classes = {
        element.get("class")
        for element in root.iter(f"{SVG_NS}rect")
    }
    assert classes == {"card", "frame"}
    assert not list(root.iter(f"{SVG_NS}polygon"))
    assert not list(root.iter(f"{SVG_NS}path"))


def test_a_left_turn_on_the_road_is_a_left_turn_on_the_page(
    l2_recording, tmp_path
):
    """ISO 8855 puts y to the left; SVG puts y down. One negation, or the map
    is a mirror image of the run.

    It is a silent failure: the path is the right shape, the labels are right,
    the hash is right, and the car goes the other way. Nothing else in the
    document disagrees with it, so it has to be asserted against the recorded
    column directly.
    """
    root = parse(sinks.write(l2_recording, tmp_path / "run", format="svg"))
    path = [
        element
        for element in root.iter(f"{SVG_NS}polyline")
        if element.get("class") == "path"
    ]
    assert len(path) == 1

    drawn_y = [
        float(pair.split(",")[1]) for pair in path[0].get("points").split()
    ]
    recorded_y = l2_recording.agents[0].state["pos.y"]
    assert len(drawn_y) == len(recorded_y)

    lowest = min(range(len(recorded_y)), key=lambda i: recorded_y[i])
    highest = max(range(len(recorded_y)), key=lambda i: recorded_y[i])
    assert recorded_y[highest] - recorded_y[lowest] > 1e-6, (
        "the run must go somewhere lateral for this to test anything"
    )
    assert drawn_y[highest] < drawn_y[lowest], (
        "more pos.y must be further UP the page"
    )


def test_the_document_references_nothing_outside_itself(l2_recording, tmp_path):
    """Self-contained: no external image, font file, stylesheet or script.

    A document that fetches something is a document that renders differently
    depending on where it is opened, and one that renders as a broken box
    offline.
    """
    text = sinks.write(l2_recording, tmp_path / "run", format="svg").read_text(
        encoding="utf-8"
    )

    for forbidden in ("<image", "<script", "xlink:href", "url(", "@import",
                      "http://", "https://"):
        # The SVG namespace declaration is the one URL that has to be there and
        # is not a fetch.
        cleaned = text.replace('xmlns="http://www.w3.org/2000/svg"', "")
        assert forbidden not in cleaned, forbidden


# ----------------------------------------------------- NaN arrives absent


def test_a_single_track_run_draws_no_per_wheel_panel(l1_recording, tmp_path):
    """The absence test every sink owes, asked of this one.

    A NaN plotted as zero is a flat line at the bottom of a panel, which reads
    as a measurement of nothing rather than as the absence of a measurement.
    So the panel is not drawn at all, and the document is shorter.
    """
    agent = l1_recording.agents[0]
    titles = {panel.title for panel in svg_sink.panel_plan(agent)}

    assert "vertical load per wheel" not in titles
    assert "slip angle per wheel" not in titles
    assert "slip ratio per wheel" not in titles
    assert "drive torque" not in titles
    assert "state of charge" not in titles

    # And the tier's own quantities are there, so the absence above is a rule
    # and not an empty drawing.
    assert "forward speed" in titles
    assert "axle lateral force" in titles

    root = parse(sinks.write(l1_recording, tmp_path / "run", format="svg"))
    drawn_titles = " ".join(texts(root))
    assert "per wheel" not in drawn_titles
    assert "state of charge" not in drawn_titles


def test_a_kinematic_run_draws_less_still(step_steer_sim, tmp_path):
    """L0 has no tyre at all, so even the axle forces are absent.

    Worth its own case rather than folding into the L1 one: the two tiers fail
    to represent different things, and a rule that only ever ran at one of
    them would not have noticed the difference.
    """
    recording = sinks.record_run(
        step_steer_sim(slipx.Tier.L0_Kinematic), duration=0.4, stride=10
    )
    titles = {panel.title for panel in svg_sink.panel_plan(recording.agents[0])}

    assert "axle lateral force" not in titles
    assert "vertical load per wheel" not in titles
    assert titles == {
        "forward speed",
        "yaw rate",
        "lateral acceleration",
        "steer angle, achieved",
    }

    root = parse(sinks.write(recording, tmp_path / "run", format="svg"))
    assert len(traces(root)) == 4


def test_a_double_track_run_draws_them(l2_recording, tmp_path):
    agent = l2_recording.agents[0]
    titles = {panel.title for panel in svg_sink.panel_plan(agent)}

    for expected in (
        "forward speed",
        "yaw rate",
        "lateral acceleration",
        "steer angle, achieved",
        "axle lateral force",
        "vertical load per wheel",
        "slip angle per wheel",
        "slip ratio per wheel",
        "drive torque",
        "state of charge",
    ):
        assert expected in titles


def test_the_document_is_shorter_when_there_is_less_to_draw(
    l1_recording, l2_recording, tmp_path
):
    """The absence rule, visible in the geometry rather than only in the plan.

    An unrepresentable quantity does not become an empty panel with a title
    and a legend, because an empty panel is a claim that something was
    measured and came out flat.
    """
    l1 = parse(sinks.write(l1_recording, tmp_path / "l1", format="svg"))
    l2 = parse(sinks.write(l2_recording, tmp_path / "l2", format="svg"))

    assert float(l1.get("height")) < float(l2.get("height"))
    assert len(traces(l1)) < len(traces(l2))


def test_a_gap_inside_a_column_breaks_the_trace_rather_than_bridging_it():
    """A wheel off the ground has no slip ratio for as long as it is off it.

    Drawn as one polyline, the line would run straight across the gap and
    assert a value nobody computed. Tested on the private helper because
    manufacturing a lifted wheel through the whole tier, at a speed and a
    steer angle that produce one, would be testing the tier.
    """
    nan = float("nan")
    values = (1.0, 2.0, nan, nan, 3.0, 4.0)
    times = (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)

    runs = svg_sink._traces(values, times, lambda t: t, lambda v: v)

    assert len(runs) == 2
    assert runs[0] == [(0.0, 1.0), (1.0, 2.0)]
    assert runs[1] == [(4.0, 3.0), (5.0, 4.0)]


# ------------------------------------------------------------ presentation


def test_the_document_is_legible_on_a_light_and_a_dark_page(
    l2_recording, tmp_path
):
    """One render, two themes, and an opaque card under both.

    A transparent background with dark strokes disappears entirely on a dark
    page, which is the most common way a diagram breaks after somebody pastes
    it somewhere.
    """
    text = sinks.write(l2_recording, tmp_path / "run", format="svg").read_text(
        encoding="utf-8"
    )

    assert "prefers-color-scheme: dark" in text

    root = parse(text)
    card = [
        element
        for element in root.iter(f"{SVG_NS}rect")
        if element.get("class") == "card"
    ]
    assert len(card) == 1
    assert card[0].get("fill") is None, "the card takes its fill from the theme"
    assert float(card[0].get("width")) == float(root.get("width"))


def test_the_run_is_animated_over_its_own_duration(l2_recording, tmp_path):
    root = parse(sinks.write(l2_recording, tmp_path / "run", format="svg"))
    duration = l2_recording.times[-1]

    animations = list(root.iter(f"{SVG_NS}animate")) + list(
        root.iter(f"{SVG_NS}animateTransform")
    )
    assert animations
    for element in animations:
        assert element.get("dur") == "%.3fs" % duration
        assert element.get("repeatCount") == "indefinite"


def test_the_marker_rests_on_the_start_of_the_path_before_it_moves(
    l2_recording, tmp_path
):
    """An untransformed group sits at the origin, which is the page corner.

    A viewer that does not run SMIL, or a screenshot taken before the
    animation starts, would otherwise show the car parked outside the plot.
    """
    root = parse(sinks.write(l2_recording, tmp_path / "run", format="svg"))
    groups = [
        element
        for element in root.iter(f"{SVG_NS}g")
        if element.get("transform", "").startswith("translate(")
    ]

    assert len(groups) == 1
    assert groups[0].get("transform") != "translate(0.000 0.000)"


def test_a_named_agent_can_be_chosen_and_an_unknown_one_names_the_rest(
    l2_recording, tmp_path
):
    """One picture is one car: overlaying two cars' traces on shared axes says
    they are comparable, and nothing in the recording says that."""
    sink = sinks.sink_for("svg", agent="car")
    assert sink.write(l2_recording, tmp_path / "run").exists()

    with pytest.raises(KeyError) as raised:
        sinks.sink_for("svg", agent="lorry").write(l2_recording, tmp_path / "x")

    assert "car" in str(raised.value)
