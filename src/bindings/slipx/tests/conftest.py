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


# ------------------------------------------------------------------- runs
#
# Two recorded runs, shared by every sink test, because each sink owes the same
# NaN-arrives-absent test (SINK-05) and it has to be run against the same data
# or the sinks are not being compared.


def _l2_params():
    """The reference numbers, filled in by hand rather than loaded.

    Deliberately not load_reference_car(): the sink tests are about recording
    and encoding, and coupling them to the schema layer would make a schema
    failure read as a sink failure. The struct is the core's boundary and
    always was. These are the same provisional numbers the C++ test support
    header uses.
    """
    import slipx

    params = slipx.VehicleParams()
    params.mass = 3.5
    params.izz = 0.05
    params.lf = 0.16
    params.lr = 0.16
    params.h_cog = 0.06
    params.c_alpha_f = 120.0
    params.c_alpha_r = 130.0
    params.mu_clip = 1.1
    return params


def _step_steer_sim(tier, steer: float = 0.12, speed: float = 4.0):
    import slipx

    config = slipx.SimulationConfig()
    config.dt = 1e-3
    sim = slipx.Simulation(config)

    spec = slipx.AgentSpec()
    spec.name = "car"
    spec.tier = tier
    spec.params = _l2_params()
    initial = slipx.VehicleState()
    initial.vel_body.x = speed
    spec.initial_state = initial

    def policy(state, time, rng):
        return slipx.DriveInput(
            steer if time >= 0.05 else 0.0,
            4.0 * (speed - state.vel_body.x),
        )

    spec.policy = policy
    sim.add_agent(spec)
    return sim


@pytest.fixture
def step_steer_sim():
    """Build the same step-steer run again, at whichever tier is asked for."""
    return _step_steer_sim


@pytest.fixture
def l1_recording():
    """A single-track run: no per-wheel anything, which is the point of it."""
    import slipx

    sim = _step_steer_sim(slipx.Tier.L1_Bicycle)
    return slipx.sinks.record_run(sim, duration=0.4, stride=10)


@pytest.fixture
def l2_recording():
    """A double-track run: four contact patches, and per-wheel arrays that are
    real (CORE-12)."""
    import slipx

    sim = _step_steer_sim(slipx.Tier.L2_DoubleTrack)
    return slipx.sinks.record_run(sim, duration=0.4, stride=10)
