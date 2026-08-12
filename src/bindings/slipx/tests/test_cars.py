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


def test_the_reference_car_is_reachable_without_knowing_where_it_is() -> None:
    """NFR-10, and the second clause of the P0 exit gate.

    An installed wheel carries the car directory inside the package; a
    checkout has it under examples/cars. Both are resolved by the same call,
    so a user who has never seen the repository layout can still load a car.
    """
    path = slipx.reference_car_path()
    assert path.is_dir()
    assert (path / "car.yaml").is_file()

    car = slipx.load_reference_car()
    assert car.name == "reference_1_10"
    assert car.provenance.label == "provisional"


def test_the_two_reference_car_locations_are_the_same_car() -> None:
    """The install copies the repository directory rather than duplicating it.

    If these ever disagree, a hash reproduced from an installed wheel would
    stop meaning what the same hash reproduced from a checkout means, which
    is the whole content of the exit gate.
    """
    resolved = slipx.load_car(slipx.reference_car_path())
    explicit = slipx.load_car(REFERENCE_CAR)
    assert resolved.params.mass == explicit.params.mass
    assert resolved.params.izz == explicit.params.izz
    assert resolved.params.c_alpha_f == explicit.params.c_alpha_f


def test_the_reference_car_parameterises_l2_and_can_be_stepped() -> None:
    # Schema 0.2.0 made the old refusal message come true: the tyre file
    # carries linear.c_kappa and the loader fills the MF-lite blocks, so L2
    # is reachable from a car file (ADR-0030).
    car = slipx.load_reference_car()
    params = car.params_for_tier(slipx.Tier.L2_DoubleTrack)

    assert params.validate() is None
    assert params.c_kappa == 120.0
    for block in (params.tyre_front, params.tyre_rear):
        assert block.shape_c == 1.68
        assert block.curvature_e == 0.42
        assert block.k_mu == 0.15
        assert block.relax_length == 0.045
        assert block.mu_y0 == 1.10
        assert block.mu_x0 == 1.05

    # And the label still leads: reachable does not mean measured.
    assert params.provenance == slipx.Provenance.Provisional
    assert car.summary().splitlines()[1].strip().startswith("provenance: PROVISIONAL")

    model = slipx.VehicleModel.create(slipx.Tier.L2_DoubleTrack, params)
    state = slipx.VehicleState()
    state.vel_body.x = 4.0
    for _ in range(500):
        model.step(state, slipx.DriveInput(steer_cmd=0.08), 1e-3)
    assert state.pos.x > 0.0
    assert state.yaw > 0.0


def test_the_files_c_kappa_reaches_the_params_not_the_cores_default(
    car_factory,
) -> None:
    # Mutation escape made into a test: the reference car's 120.0 happens to
    # equal the core's default, so a params_for_tier that silently forgot to
    # copy the loaded value would pass every reference-car assertion. A value
    # the default cannot produce proves the copy happens.
    path = car_factory()
    tyre = path / "tyres" / "sponge_carpet.yaml"
    tyre.write_text(
        tyre.read_text(encoding="utf-8").replace("c_kappa: 120.0", "c_kappa: 150.0"),
        encoding="utf-8",
    )
    car = slipx.load_car(path)
    params = car.params_for_tier(slipx.Tier.L2_DoubleTrack)
    assert params.c_kappa == 150.0
    assert slipx.VehicleParams().c_kappa != 150.0


def test_l2_is_still_refused_by_name_for_a_file_without_c_kappa(car_factory) -> None:
    # ADR-0025's refusal path survives 0.2.0: a tyre file that lacks the
    # field, which every 0.1.0 file does, is refused with the field named,
    # never defaulted. A refusal produces nothing; a default would produce a
    # trajectory labelled L2 resting on a number nobody measured.
    path = car_factory()
    tyre = path / "tyres" / "sponge_carpet.yaml"
    tyre.write_text(
        "\n".join(
            line
            for line in tyre.read_text(encoding="utf-8").splitlines()
            if "c_kappa" not in line
        )
        + "\n",
        encoding="utf-8",
    )
    car = slipx.load_car(path)

    with pytest.raises(ValueError, match="c_kappa"):
        car.params_for_tier(slipx.Tier.L2_DoubleTrack)

    # The message names both files' gaps and nothing that is present, and it
    # explains that migrating the version cannot supply the measurement.
    try:
        car.params_for_tier(slipx.Tier.L2_DoubleTrack)
    except ValueError as exc:
        message = str(exc)
    assert "front tyre linear.c_kappa" in message
    assert "rear tyre linear.c_kappa" in message
    assert "0.2.0" in message
    assert "ADR-0025" in message
    assert "mf_lite" not in message
    assert "k_mu" not in message
    assert "sigma" not in message

    # And this is not the ADR-0005 failure: no lower tier is handed back.
    # L0 and L1 keep working through the same call.
    for tier in (slipx.Tier.L0_Kinematic, slipx.Tier.L1_Bicycle):
        params = car.params_for_tier(tier)
        assert params.validate() is None
        assert params.mass == car.params.mass
