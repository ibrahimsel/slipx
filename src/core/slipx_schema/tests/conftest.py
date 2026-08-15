# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Fixtures for the slipx_schema tests.

Every test that needs a broken car starts from the reference car and breaks
exactly one thing. That is deliberate: a hand-written fixture drifts away from
the real thing, and then the tests pass against a car nobody ships.
"""

from __future__ import annotations

import shutil
from pathlib import Path
from typing import Any, Callable, Dict

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[4]
REFERENCE_CAR = REPO_ROOT / "examples" / "cars" / "reference_1_10"
REFERENCE_TRACK = REPO_ROOT / "examples" / "tracks" / "paddock_stadium"


@pytest.fixture
def reference_car() -> Path:
    """The shipped reference car directory, read-only."""
    assert REFERENCE_CAR.is_dir(), f"reference car missing at {REFERENCE_CAR}"
    return REFERENCE_CAR


@pytest.fixture
def reference_track() -> Path:
    """The shipped generated track directory, read-only."""
    assert REFERENCE_TRACK.is_dir(), f"reference track missing at {REFERENCE_TRACK}"
    return REFERENCE_TRACK


@pytest.fixture
def track_factory(tmp_path: Path) -> Callable[..., Path]:
    """Copy the shipped track and apply one edit to its manifest.

    Same reasoning as ``car_factory``: a hand-written track manifest drifts
    away from the one that ships, and then the tests pass against a file
    nobody has.
    """
    counter = {"n": 0}

    def make(edit: Callable[[Dict[str, Any]], None] | None = None) -> Path:
        counter["n"] += 1
        destination = tmp_path / f"track{counter['n']}"
        shutil.copytree(REFERENCE_TRACK, destination)

        if edit is not None:
            target = destination / "track.yaml"
            with target.open(encoding="utf-8") as handle:
                document = yaml.safe_load(handle)
            edit(document)
            with target.open("w", encoding="utf-8") as handle:
                yaml.safe_dump(document, handle, sort_keys=False)

        return destination

    return make


def derived_b(tyre_path: Path) -> float:
    """The B the loader derives for a tyre file, computed from that file.

    A test that wants a B agreeing with the linear block must not hard-code
    the number: when the reference car's cornering stiffness moved (ADR-0032)
    every literal became a value that quietly disagreed, and a test asserting
    "no warning" turns into a test asserting nothing at all.
    """
    tyre = yaml.safe_load(tyre_path.read_text(encoding="utf-8"))
    return tyre["linear"]["c_alpha"] / (
        tyre["mf_lite"]["C"] * tyre["friction"]["mu_y0"] * tyre["nominal_load"]
    )


@pytest.fixture
def car_factory(tmp_path: Path) -> Callable[..., Path]:
    """Copy the reference car and apply one edit to it.

    Usage::

        path = car_factory("dynamics.yaml", lambda d: d["geometry"].update(width=0.9))
    """
    counter = {"n": 0}

    def make(filename: str = "", edit: Callable[[Dict[str, Any]], None] | None = None) -> Path:
        counter["n"] += 1
        destination = tmp_path / f"car{counter['n']}"
        shutil.copytree(REFERENCE_CAR, destination)

        if filename and edit is not None:
            target = destination / filename
            with target.open(encoding="utf-8") as handle:
                document = yaml.safe_load(handle)
            edit(document)
            with target.open("w", encoding="utf-8") as handle:
                yaml.safe_dump(document, handle, sort_keys=False)

        return destination

    return make
