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

import math
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


def test_the_shipped_tracks_regenerate_byte_for_byte(tmp_path: Path) -> None:
    """make_tracks.py, run from a copy, must reproduce the checked-in files.

    The CSVs are fixtures for both test suites and inputs to every demo, so
    drift between the generator and its committed output would be a silent
    fork: the tree asserting one geometry while the script documents
    another. Running a copy from tmp also proves the generator carries no
    dependence on where it sits, and the generator's own checks (closure,
    on-locus samples, width envelope, wall clearance) all run on the way.
    """
    generator = EXAMPLES / "tracks" / "make_tracks.py"
    if not generator.exists():
        pytest.skip(f"{generator} not present; not running from a checkout")

    script = tmp_path / "make_tracks.py"
    script.write_text(generator.read_text(encoding="utf-8"), encoding="utf-8")
    result = subprocess.run(
        [sys.executable, str(script)], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr

    for name in ("paddock_stadium", "paddock_gp"):
        assert f"{name}/centreline.csv" in result.stdout
        produced = (tmp_path / name / "centreline.csv").read_bytes()
        shipped = (EXAMPLES / "tracks" / name / "centreline.csv").read_bytes()
        assert produced == shipped, f"{name} drifted from its generator"


def test_the_circuit_checks_have_teeth() -> None:
    """The generator's own checker must refuse what the envelope forbids.

    The byte-compare above proves the shipped output matches the script; it
    proves nothing about the assertions, which are what stop a future edit
    shipping a track whose walls fold over a corner. So hand the checker
    the real geometry with a width profile that crowds the chicane arcs and
    demand a refusal; a checker that waves it through would let a geometry
    regression land as a byte change nobody reads.
    """
    generator = EXAMPLES / "tracks" / "make_tracks.py"
    if not generator.exists():
        pytest.skip(f"{generator} not present; not running from a checkout")

    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "make_tracks_under_test", generator)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    points = module.circuit_centreline()
    widths = module.circuit_widths(len(points))
    module.check_circuit(points, widths)   # the shipped profile passes

    # The width envelope: a profile that ignores the designed extremes.
    with pytest.raises(AssertionError):
        module.check_circuit(points, [1.7] * len(points))

    # The arc-crowding guard, reached on its own: keep the envelope legal
    # (the 0.70 pinch survives at the chicane's ends, steps stay under the
    # continuity limit, nothing exceeds the lap maximum) while ramping the
    # chicane's interior to 0.96, which leaves the 1 m arcs less than the
    # 0.05 m of inner radius the guard demands.
    count = len(points)
    lap = module.CIRCUIT_LAP_M
    crowded = list(widths)
    inside = [i for i in range(count)
              if module.CHICANE_IN < i * lap / count < module.CHICANE_OUT]
    for rank, i in enumerate(inside):
        ramp = min(rank, len(inside) - 1 - rank) * 0.009
        crowded[i] = min(0.70 + ramp, 0.96)
    with pytest.raises(AssertionError):
        module.check_circuit(points, crowded)

    # The lane-clearance guard, on a lap that genuinely self-intersects:
    # out along y = 0 and straight back along y = 1.2 with 0.7 m walls,
    # which is two lanes sharing the same tarmac.
    out_and_back = ([(i * 0.1, 0.0) for i in range(100)]
                    + [((99 - i) * 0.1, 1.2) for i in range(100)])
    with pytest.raises(AssertionError):
        module.check_lane_clearance(out_and_back, [0.7] * 200, 20.0)

    # The on-locus guard: one sample nudged a micron off its segment.
    nudged = list(points)
    nudged[100] = (points[100][0], points[100][1] + 1.0e-6)
    with pytest.raises(AssertionError):
        module.check_circuit(nudged, widths)

    # The continuity guard: one width stepped by more than a wall can
    # mitre, with the envelope otherwise kept legal.
    stepped = list(widths)
    stepped[300] = stepped[300] + 0.02
    with pytest.raises(AssertionError):
        module.check_circuit(points, stepped)

    # The closure guard: the same walk, told it ended a millimetre short.
    true_end = module._CIRCUIT_END
    try:
        module._CIRCUIT_END = (true_end[0], 1.0e-3, 0.0, 2.0 * math.pi)
        with pytest.raises(AssertionError):
            module.check_circuit(points, widths)
    finally:
        module._CIRCUIT_END = true_end


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
