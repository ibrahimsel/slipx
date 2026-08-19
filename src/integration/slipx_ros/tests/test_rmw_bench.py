# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The transport benchmark's harness, exercised end to end (ADR-0052).

One row of the real thing: a bridge process and one client process per
agent, mediated by the zenoh router, which is the multi-host wiring on a
single machine; a second host adds a physical wire to exactly this code
path and nothing else (ADR-0051). Passing here means the sync authority
holds across process boundaries and a router, not merely across threads.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy", reason="the benchmark needs a ROS 2 environment")

REPO = Path(__file__).resolve().parents[4]


def test_a_router_mediated_lockstep_row_completes():
    zenohd = Path(os.environ.get("ROS_DISTRO_PREFIX", "/opt/ros/jazzy")) \
        / "lib" / "rmw_zenoh_cpp" / "rmw_zenohd"
    if not zenohd.exists():
        pytest.skip("rmw_zenoh's router is not installed")

    outcome = subprocess.run(
        [sys.executable, "-m", "slipx_ros.rmw_bench",
         "--agents", "2", "--steps", "50", "--json",
         "--configurations", "zenoh"],
        cwd=str(REPO), capture_output=True, text=True, timeout=300)
    assert outcome.returncode == 0, outcome.stderr

    rows = [json.loads(line) for line in outcome.stdout.splitlines()
            if line.startswith("{")]
    assert len(rows) == 1, outcome.stdout
    row = rows[0]
    assert "error" not in row, row
    assert row["configuration"] == "zenoh"
    assert row["agents"] == 2
    assert row["steps"] == 50, "every announced step was answered"
    assert row["steps_per_s"] > 1.0
