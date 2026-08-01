# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The Python API is the C++ API.

These tests do not re-test the physics. The C++ suite does that against the
same code, and a second set of physics assertions in another language would be
two things to keep in step. What is tested here is the boundary: that values
cross it unchanged, that conventions survive the trip, and that the refusals
the core makes are still refusals on this side.
"""

from __future__ import annotations

import math

import pytest

import slipx


def test_versions_are_reported_and_independent() -> None:
    assert slipx.__version__ == "0.1.0"
    assert slipx.core_version == "0.1.0"

    # slipx_schema is versioned independently (NFR-09), so this is a separate
    # number that happens to agree today and need not tomorrow.
    import slipx_schema

    assert slipx_schema.SCHEMA_VERSION


def test_default_params_are_usable_and_provisional() -> None:
    params = slipx.VehicleParams()
    assert params.validate() is None
    assert params.wheelbase == pytest.approx(params.lf + params.lr)
    # NFR-08: the weakest claim is the safe default.
    assert params.provenance == slipx.Provenance.Provisional


def test_impossible_params_are_refused_with_the_field_named() -> None:
    params = slipx.VehicleParams()
    params.mass = -1.0
    assert "mass" in params.validate()

    with pytest.raises(ValueError, match="mass"):
        slipx.VehicleModel.create(slipx.Tier.L1_Bicycle, params)


def test_unimplemented_tiers_raise_rather_than_substituting() -> None:
    # CORE-02. A trajectory labelled L2 that is actually L1 is worse than no
    # trajectory, so this must never quietly succeed.
    params = slipx.VehicleParams()
    for tier in (slipx.Tier.L2_DoubleTrack, slipx.Tier.L3_Extended):
        with pytest.raises(ValueError, match="not implemented"):
            slipx.VehicleModel.create(tier, params)


@pytest.mark.parametrize("tier", [slipx.Tier.L0_Kinematic, slipx.Tier.L1_Bicycle])
def test_iso_8855_conventions_survive_the_binding(tier) -> None:
    model = slipx.VehicleModel.create(tier, slipx.VehicleParams())
    state = slipx.VehicleState()
    state.vel_body.x = 4.0
    diagnostics = slipx.StepDiagnostics()

    for _ in range(500):
        model.step(state, slipx.DriveInput(steer_cmd=0.1, accel_cmd=0.0), 1e-3,
                   diagnostics)

    assert state.yaw > 0.0, "positive steer turns left"
    assert state.pos.y > 0.0
    assert state.yaw_rate > 0.0
    assert diagnostics.ay > 0.0
    assert diagnostics.tier == tier


def test_angles_are_radians_across_the_boundary() -> None:
    model = slipx.VehicleModel.create(slipx.Tier.L1_Bicycle, slipx.VehicleParams())
    state = slipx.VehicleState()
    state.vel_body.x = 5.0
    model.step(state, slipx.DriveInput(steer_cmd=0.2), 1e-3)
    assert state.steer == pytest.approx(0.2)


def test_diagnostics_are_optional_and_do_not_perturb_the_run() -> None:
    params = slipx.VehicleParams()
    quiet = slipx.VehicleModel.create(slipx.Tier.L1_Bicycle, params)
    loud = slipx.VehicleModel.create(slipx.Tier.L1_Bicycle, params)

    a = slipx.VehicleState()
    a.vel_body.x = 5.0
    b = slipx.VehicleState()
    b.vel_body.x = 5.0
    diagnostics = slipx.StepDiagnostics()

    for _ in range(1000):
        quiet.step(a, slipx.DriveInput(steer_cmd=0.08), 1e-3)
        loud.step(b, slipx.DriveInput(steer_cmd=0.08), 1e-3, diagnostics)

    assert a.pos.x == b.pos.x
    assert a.pos.y == b.pos.y
    assert a.yaw == b.yaw


def test_unrepresentable_quantities_are_nan_not_zero() -> None:
    # The NaN convention is the teaching surface, and it has to survive the
    # binding or a Python user plots zeros and believes them.
    model = slipx.VehicleModel.create(slipx.Tier.L0_Kinematic, slipx.VehicleParams())
    state = slipx.VehicleState()
    state.vel_body.x = 4.0
    diagnostics = slipx.StepDiagnostics()
    model.step(state, slipx.DriveInput(steer_cmd=0.1), 1e-3, diagnostics)

    assert math.isnan(diagnostics.alpha_front), "L0 has no tyres to slip"
    assert math.isnan(diagnostics.load_transfer_long)
    assert not math.isnan(diagnostics.ay)

    l1 = slipx.VehicleModel.create(slipx.Tier.L1_Bicycle, slipx.VehicleParams())
    state = slipx.VehicleState()
    state.vel_body.x = 4.0
    l1.step(state, slipx.DriveInput(steer_cmd=0.1), 1e-3, diagnostics)
    assert not math.isnan(diagnostics.alpha_front)
    assert math.isnan(diagnostics.load_transfer_long), "CORE-05 arrives at L2"


def test_state_is_copyable_and_a_copy_is_independent() -> None:
    # Snapshot and restore is a memcpy (CORE-03, SIM-08), and that has to be
    # true through the binding too or a Python user's saved state is a view.
    model = slipx.VehicleModel.create(slipx.Tier.L1_Bicycle, slipx.VehicleParams())
    state = slipx.VehicleState()
    state.vel_body.x = 5.0

    snapshot = state.copy()
    for _ in range(1000):
        model.step(state, slipx.DriveInput(steer_cmd=0.2), 1e-3)

    assert snapshot.pos.x == 0.0
    assert state.pos.x > 0.0


def test_the_model_copies_its_parameters() -> None:
    params = slipx.VehicleParams()
    params.mass = 7.5
    model = slipx.VehicleModel.create(slipx.Tier.L1_Bicycle, params)
    params.mass = 999.0
    assert model.params.mass == 7.5


def test_l0_ignores_everything_it_does_not_represent() -> None:
    # The teaching artefact (SRS 2.4), asserted from Python because this is
    # where a student will first try it.
    def run(mutate) -> tuple[float, float]:
        params = slipx.VehicleParams()
        mutate(params)
        model = slipx.VehicleModel.create(slipx.Tier.L0_Kinematic, params)
        state = slipx.VehicleState()
        state.vel_body.x = 5.0
        for _ in range(2000):
            model.step(state, slipx.DriveInput(steer_cmd=0.15, accel_cmd=0.0), 1e-3)
        return state.pos.x, state.pos.y

    def heavier(params):
        params.mass = 12.0
        params.izz = 0.4
        params.h_cog = 0.15
        params.c_alpha_f = 400.0

    assert run(lambda p: None) == run(heavier)


def test_model_repr_names_the_tier_and_integrator() -> None:
    model = slipx.VehicleModel.create(
        slipx.Tier.L1_Bicycle, slipx.VehicleParams(), slipx.Integrator.SemiImplicitEuler
    )
    text = repr(model)
    assert "L1_Bicycle" in text
    assert "semi_implicit_euler" in text
    assert model.state_dimension == 6
