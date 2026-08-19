# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The validation report (M6.4): replay, divergence, and the drawing."""

from __future__ import annotations

import xml.etree.ElementTree as ElementTree
from pathlib import Path

import pytest

import slipx
from slipx_id import report, synthetic


@pytest.fixture(scope="module")
def runs():
    params = slipx.VehicleParams()
    return params, [
        synthetic.slalom(params, speed=3.0, amplitude=0.15, duration=6.0),
        synthetic.slalom(params, speed=5.0, amplitude=0.08, duration=6.0),
    ]


def test_the_true_model_reproduces_its_own_runs(runs, tmp_path: Path) -> None:
    # The floor of the metric: replaying a run through the parameters that
    # generated it should diverge only by the replay's starting-state
    # construction and the sensors' sampling.
    params, recordings = runs
    path, worst = report.generate(
        params, recordings, tmp_path / "report.svg", date="2026-08-19"
    )
    assert path.exists()
    assert worst < 2.0, f"self-replay diverged {worst:.2f}%"


def test_a_wrong_model_is_visibly_wrong(runs, tmp_path: Path) -> None:
    # The metric must measure something: a car with a fifth of its
    # cornering stiffness missing cannot quietly validate.
    params, recordings = runs
    wrong = params.copy()
    wrong.c_alpha_f *= 0.8
    wrong.c_alpha_r *= 0.8
    _, honest = report.generate(
        params, recordings, tmp_path / "honest.svg", date="x"
    )
    _, wrong_worst = report.generate(
        wrong, recordings, tmp_path / "wrong.svg", date="x"
    )
    assert wrong_worst > 3.0 * max(honest, 1.0)


def test_the_report_is_a_wellformed_theme_aware_svg(runs, tmp_path: Path) -> None:
    params, recordings = runs
    path, worst = report.generate(
        params, recordings, tmp_path / "report.svg", date="2026-08-19"
    )
    text = path.read_text(encoding="utf-8")
    root = ElementTree.fromstring(text)
    assert root.tag.endswith("svg")

    # The provenance label is printed, not implied; these parameters are
    # provisional and the report must say so even while validating them.
    assert "PROVISIONAL" in text
    # The three channels, their divergences, and both themes.
    for expected in (
        "yaw rate",
        "lateral acceleration",
        "speed",
        "diverges",
        "prefers-color-scheme: dark",
        "worst-channel divergence",
        "on nothing else",
    ):
        assert expected in text, expected


def test_headline_is_the_worst_not_the_mean(runs, tmp_path: Path) -> None:
    params, recordings = runs
    comparisons = [report.compare(params, rec) for rec in recordings]
    worst = report.headline(comparisons)
    every = [
        d.percent for c in comparisons for d in c.divergences
    ]
    assert worst == max(every)
