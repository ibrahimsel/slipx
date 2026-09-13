# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The RViz config generator's view fitting.

The generator needs no ROS 2, only yaml: it reads a track and a car and
writes a config, so it is tested by running it against the shipped circuit
and reading the numbers back out of what it wrote. The fit is the thing
under test: the track plus its margin must land inside the requested view
box in both axes, with the binding axis deciding the scale.
"""

from __future__ import annotations

import csv
import re
import sys
from pathlib import Path

import pytest

pytest.importorskip("yaml", reason="the generator reads the car's yaml")

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "examples" / "ros"))

import make_race_rviz as generator  # noqa: E402

TRACK = REPO / "examples" / "tracks" / "paddock_gp"
MARGIN = 4.0  # 2 m each side, the generator's own allowance


def track_extent() -> tuple:
    xs, ys = [], []
    with open(TRACK / "centreline.csv", encoding="utf-8", newline="") as f:
        for row in csv.reader(f):
            if row and not row[0].lstrip().startswith("#"):
                xs.append(float(row[0]))
                ys.append(float(row[1]))
    return max(xs) - min(xs), max(ys) - min(ys)


def generate(tmp_path: Path, monkeypatch, *extra: str) -> str:
    out = tmp_path / "race.rviz"
    monkeypatch.setattr(sys, "argv", [
        "make_race_rviz.py", "--agents", "2", "--track", str(TRACK),
        "--out", str(out), *extra])
    assert generator.main() == 0
    return out.read_text(encoding="utf-8")


def scale_of(config: str) -> float:
    return float(re.search(r"^      Scale: (\d+)$", config, re.M).group(1))


def geometry_of(config: str) -> tuple:
    width = int(re.search(r"^  Width: (\d+)$", config, re.M).group(1))
    height = int(re.search(r"^  Height: (\d+)$", config, re.M).group(1))
    return width, height


def test_size_parses_width_by_height():
    assert generator._size("1300x430") == (1300, 430)
    assert generator._size("1300X430") == (1300, 430)


@pytest.mark.parametrize("text", ["1300", "ax430", "0x430", "1300x-1", ""])
def test_size_refuses_and_names_the_text(text):
    with pytest.raises(SystemExit) as refusal:
        generator._size(text)
    assert repr(text) in str(refusal.value)


def test_default_view_is_the_square_box(tmp_path, monkeypatch):
    dx, dy = track_extent()
    config = generate(tmp_path, monkeypatch)
    assert scale_of(config) == round(900.0 / (max(dx, dy) + MARGIN))
    assert geometry_of(config) == (1400, 900)


def test_wide_view_is_bound_by_the_track_width(tmp_path, monkeypatch):
    dx, dy = track_extent()
    assert dx > 3 * dy, "the circuit is wide; that is what the fit is for"
    config = generate(tmp_path, monkeypatch,
                      "--view", "1300x430", "--window", "1730x536")
    scale = scale_of(config)
    # Inside the box in both axes, at the largest integer scale that is.
    assert scale * (dx + MARGIN) <= 1300 + 0.5 * (dx + MARGIN)
    assert scale * (dy + MARGIN) <= 430 + 0.5 * (dy + MARGIN)
    assert scale == round(min(1300 / (dx + MARGIN), 430 / (dy + MARGIN)))
    assert scale > round(900.0 / (max(dx, dy) + MARGIN))
    assert geometry_of(config) == (1730, 536)


def test_short_view_is_bound_by_the_track_height(tmp_path, monkeypatch):
    dx, dy = track_extent()
    config = generate(tmp_path, monkeypatch, "--view", "5000x430")
    assert scale_of(config) == round(430 / (dy + MARGIN))
    assert scale_of(config) < round(5000 / (dx + MARGIN))


def test_tall_view_is_bound_by_the_other_axis(tmp_path, monkeypatch):
    dx, dy = track_extent()
    wide = scale_of(generate(tmp_path, monkeypatch, "--view", "1300x430"))
    tall = scale_of(generate(tmp_path, monkeypatch, "--view", "430x1300"))
    assert tall == round(430 / (dx + MARGIN))
    assert tall < wide
