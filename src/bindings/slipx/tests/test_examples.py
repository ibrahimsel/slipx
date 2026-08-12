# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The examples are executed, not proofread.

An example that no longer runs is worse than no example: it is the first thing
a new user tries, and it fails in front of them rather than in front of us.
Each one is run as a subprocess exactly as the README tells somebody to run
it, and a few load-bearing lines of its output are checked, because an example
that runs and prints nothing useful has also failed.

These skip when the checkout is not there, which is the case for a wheel
installed from PyPI: the examples ship in the repository, not in the package.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
EXAMPLES = REPO_ROOT / "examples"


def run_example(name: str, *args: str) -> str:
    path = EXAMPLES / name
    if not path.exists():
        pytest.skip(f"{path} not present; not running from a checkout")

    env = dict(os.environ)
    existing = env.get("PYTHONPATH", "")
    entries = [p for p in sys.path if p]
    env["PYTHONPATH"] = os.pathsep.join(entries + ([existing] if existing else []))

    result = subprocess.run(
        [sys.executable, str(path), *args],
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode == 0, (
        f"{name} exited {result.returncode}\n{result.stdout}\n{result.stderr}"
    )
    return result.stdout


def test_example_01_drives_the_reference_car() -> None:
    out = run_example("01_load_a_car_and_drive_it.py")

    # The provenance label leads, on every path that shows a number to a
    # person. An example that quietly dropped it would be teaching the wrong
    # habit to everyone who copied it.
    assert "PROVISIONAL" in out
    assert "yaw rate" in out

    # Per-wheel quantities are numbers at L2, so none of the four wheel lines
    # may print nan here; the same lines at L0 are exactly the ones that
    # would. Only those lines are checked: "provenance" contains "nan", which
    # is a good reminder that a substring search over a whole document proves
    # less than it looks like it does.
    wheels = [line for line in out.splitlines() if "alpha=" in line]
    assert len(wheels) == 4
    assert not any("nan" in line for line in wheels)


def test_example_02_measures_the_cross_tier_gap() -> None:
    out = run_example("02_where_the_tiers_disagree.py")

    assert "L0 R" in out and "L1 R" in out and "L2 R" in out
    # The claim the example exists to make is a measured one, so it has to
    # actually appear rather than being described in the prose.
    assert "agree within 1%" in out
    assert "spool" in out


def test_example_03_writes_a_file_and_no_window(tmp_path: Path) -> None:
    out = run_example("03_record_a_run.py", str(tmp_path))

    svg = tmp_path / "slalom.svg"
    l0 = tmp_path / "slalom_l0.svg"
    assert svg.exists() and l0.exists()
    assert "trajectory hash:" in out

    # The provenance label and the hash are drawn INTO the picture, because a
    # plot gets pasted into a slide without whatever text was printed beside
    # it.
    document = svg.read_text(encoding="utf-8")
    assert "provisional" in document.lower()
    assert "<svg" in document

    # A tier that cannot compute a quantity leaves the panel out. The L0
    # document is smaller for that reason, and if it ever stops being smaller,
    # something is being drawn that the run did not record.
    assert l0.stat().st_size < svg.stat().st_size
