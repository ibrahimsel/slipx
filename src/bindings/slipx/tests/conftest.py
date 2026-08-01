# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Fixtures for the binding tests.

Shares the reference car with the slipx_schema tests rather than keeping a
second copy: two fixtures describing the same car drift apart, and then one
suite passes against a car nobody ships.
"""

from __future__ import annotations

import shutil
from pathlib import Path
from typing import Any, Callable, Dict

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[4]
REFERENCE_CAR = REPO_ROOT / "examples" / "cars" / "reference_1_10"


@pytest.fixture
def car_factory(tmp_path: Path) -> Callable[..., Path]:
    """Copy the reference car and apply one edit to it."""
    counter = {"n": 0}

    def make(
        filename: str = "", edit: Callable[[Dict[str, Any]], None] | None = None
    ) -> Path:
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
