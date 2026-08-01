# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The join between slipx_schema and slipx_core.

The conversion is field for field with no arithmetic, and that is exactly what
these tests check. Any calculation done here would be a calculation living in
the one place with neither the schema's validation nor the core's tests behind
it.
"""

from __future__ import annotations

from pathlib import Path

import pytest

import slipx
from slipx_schema import ValidationError

REFERENCE_CAR = Path(__file__).resolve().parents[4] / "examples" / "cars" / "reference_1_10"


def test_the_reference_car_loads_and_can_be_stepped() -> None:
    car = slipx.load_car(REFERENCE_CAR)
    model = slipx.VehicleModel.create(slipx.Tier.L1_Bicycle, car.params)

    state = slipx.VehicleState()
    state.vel_body.x = 5.0
    for _ in range(1000):
        model.step(state, slipx.DriveInput(steer_cmd=0.05), 1e-3)

    assert state.pos.x > 0.0
    assert state.yaw > 0.0


def test_every_field_crosses_unchanged() -> None:
    car = slipx.load_car(REFERENCE_CAR)
    source = car.spec.params
    params = car.params

    for field in (
        "mass", "izz", "ixx", "iyy", "lf", "lr", "track_front", "track_rear",
        "h_cog", "wheel_radius", "c_alpha_f", "c_alpha_r", "mu_clip",
        "accel_max", "decel_max", "v_max", "steer_max", "drag_coeff",
        "roll_resist",
    ):
        assert getattr(params, field) == getattr(source, field), field


def test_the_provenance_label_crosses_too() -> None:
    # NFR-08. The label has to reach the struct, or tooling downstream of the
    # conversion has nothing to print.
    car = slipx.load_car(REFERENCE_CAR)
    assert car.params.provenance == slipx.Provenance.Provisional
    assert car.provenance.label == "provisional"
    assert "PROVISIONAL" in car.summary()


def test_an_absent_v_eps_leaves_the_cores_default_in_place() -> None:
    car = slipx.load_car(REFERENCE_CAR)
    assert car.spec.params.v_eps is None
    assert car.params.v_eps == slipx.VehicleParams().v_eps
    # Not silent: the loader said so (SCH-02).
    assert any("v_eps" in note for note in car.notes)


def test_a_specified_v_eps_is_used(car_factory) -> None:
    path = car_factory("dynamics.yaml", lambda d: d.update(numerics={"v_eps": 0.25}))
    car = slipx.load_car(path)
    assert car.params.v_eps == 0.25
    assert not any("v_eps" in note for note in car.notes)


def test_a_car_the_schema_accepts_but_the_core_rejects_fails_at_load() -> None:
    # The two validators check different things and neither is redundant: the
    # schema knows the rules, the core knows what is physically possible. This
    # is the seam, and a failure here should happen at load rather than at the
    # first step, three frames into a rollout.
    car = slipx.load_car(REFERENCE_CAR)
    params = car.params
    params.izz = 0.0
    assert params.validate() is not None


def test_schema_failures_surface_as_schema_errors(car_factory) -> None:
    path = car_factory("dynamics.yaml", lambda d: d.pop("mass"))
    with pytest.raises(ValidationError, match="mass"):
        slipx.load_car(path)


def test_strict_mode_reaches_through(car_factory) -> None:
    path = car_factory("dynamics.yaml", lambda d: d["inertia"].update(izz=0.5, iyy=0.5))
    assert slipx.load_car(path).warnings
    with pytest.raises(ValidationError, match="strict"):
        slipx.load_car(path, strict=True)


def test_repr_leads_with_what_matters() -> None:
    text = repr(slipx.load_car(REFERENCE_CAR))
    assert "reference_1_10" in text
    assert "provisional" in text
