# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The tyre model, evaluable pointwise from Python.

A vehicle model answers "what did this car do". These answer "what does this
tyre do", which is the question a tyre plot, a check of a fitted parameter set
and a hand calculation all ask. Before they existed, the only way to see a
tyre curve was to drive a car across it and read the diagnostics back, and the
tutorial series ended up drawing its curves from a second Magic Formula that
agreed with this one by hand until it did not (ADR-0032).

So the property these tests are really protecting is that there is one tyre
model in the project, and that anything drawing a tyre curve is drawing this
one.
"""

from __future__ import annotations

import math

import pytest

import slipx


@pytest.fixture
def reference_tyre():
    """The reference car's front tyre, built the way the tier builds it."""
    params = slipx.load_reference_car().params_for_tier(
        slipx.Tier.L2_DoubleTrack
    )
    fz_nom = params.mass * 9.80665 / 4.0
    tyre = slipx.make_mf_lite(params.tyre_front, params.c_alpha_f / 2.0,
                              fz_nom)
    return tyre, fz_nom, params


def test_b_is_derived_and_never_read_from_a_parameter_set(reference_tyre):
    """ADR-0023: MF-lite reproduces the linear tyre exactly at small slip.

    That is the whole reason B is derived rather than fitted, and it is a
    property of the arithmetic rather than of a tolerance, so it is asserted
    against the cornering stiffness that went in.
    """
    tyre, fz_nom, params = reference_tyre

    assert tyre.b == pytest.approx(
        params.c_alpha_f / 2.0 / (tyre.c * tyre.mu_y0 * fz_nom), rel=1e-12
    )
    # The slope at the origin IS the cornering stiffness, per tyre.
    assert slipx.cornering_stiffness_at_load(tyre, fz_nom) == pytest.approx(
        params.c_alpha_f / 2.0, rel=1e-12
    )


def test_the_curve_leaves_the_origin_along_the_linear_tyre(reference_tyre):
    """Tangency, asserted as tangency rather than as a fixed tolerance.

    "Agrees with the linear tyre at small slip" is a statement about how the
    error behaves as the slip angle shrinks, not about its size at one angle.
    The leading departure is cubic in `B alpha`, so the RELATIVE error goes as
    `(B alpha)^2` and falls by a hundred for every factor of ten. A fixed
    tolerance would pass a curve that left the origin along the wrong line and
    happened to be close at the angle somebody picked.
    """
    tyre, fz_nom, params = reference_tyre
    c_alpha = params.c_alpha_f / 2.0

    errors = []
    for alpha in (1e-5, 1e-4, 1e-3):
        linear = c_alpha * alpha
        relative = abs(-slipx.mf_lite_fy(tyre, alpha, fz_nom) - linear) / linear
        assert relative < 2.0 * (tyre.b * alpha) ** 2
        errors.append(relative)

    for coarse, fine in zip(errors[1:], errors):
        assert coarse > 50.0 * fine, "the departure must be second order"


def test_the_sign_is_iso_8855_and_not_sae(reference_tyre):
    """A positive slip angle means the tyre runs to the left of where it
    points, and the force it makes opposes that, so it is negative."""
    tyre, fz_nom, _ = reference_tyre

    assert slipx.mf_lite_fy(tyre, 0.05, fz_nom) < 0.0
    assert slipx.mf_lite_fy(tyre, -0.05, fz_nom) > 0.0
    assert slipx.mf_lite_fy(tyre, 0.0, fz_nom) == 0.0
    # Odd, exactly: nothing about a tyre distinguishes left from right.
    assert slipx.mf_lite_fy(tyre, 0.05, fz_nom) == -slipx.mf_lite_fy(
        tyre, -0.05, fz_nom
    )


def test_the_reference_tyre_peaks_where_a_tyre_peaks(reference_tyre):
    """The specific defect ADR-0032 fixed, pinned so it cannot come back.

    A peak at 24 degrees is not a soft tyre, it is a tyre the car never
    reaches, and it makes MF-lite's falling branch dead code in every figure
    and every run.
    """
    tyre, fz_nom, _ = reference_tyre

    forces = [
        (abs(slipx.mf_lite_fy(tyre, math.radians(deg / 20.0), fz_nom)),
         deg / 20.0)
        for deg in range(1, 601)
    ]
    peak_force, peak_deg = max(forces)

    assert 4.0 < peak_deg < 10.0, f"peak at {peak_deg} degrees"
    # And there really is a falling branch past it, which is the point of
    # having a peak at all.
    beyond = abs(slipx.mf_lite_fy(tyre, math.radians(2.5 * peak_deg), fz_nom))
    assert beyond < 0.95 * peak_force


def test_peak_force_falls_short_of_mu_fz_as_the_load_rises(reference_tyre):
    """Load sensitivity, which is why load transfer costs a car grip."""
    tyre, fz_nom, _ = reference_tyre

    # At the nominal load the exponent has nothing to bite on.
    assert slipx.peak_lateral_force(tyre, fz_nom) == pytest.approx(
        tyre.mu_y0 * fz_nom, rel=1e-12
    )

    # A transfer, so the two loads still average the nominal one: that is what
    # makes the comparison below about load sensitivity and not about the axle
    # carrying more weight.
    heavy, light = 1.5 * fz_nom, 0.5 * fz_nom
    assert slipx.peak_lateral_force(tyre, heavy) < tyre.mu_y0 * heavy
    assert slipx.peak_lateral_force(tyre, light) > tyre.mu_y0 * light

    # And the pair together is worth less than two at the mean, which is the
    # whole of what load transfer does to an axle.
    transferred = (slipx.peak_lateral_force(tyre, heavy)
                   + slipx.peak_lateral_force(tyre, light))
    assert transferred < 2.0 * slipx.peak_lateral_force(tyre, fz_nom)

    assert slipx.peak_lateral_force(tyre, 0.0) == 0.0
    assert slipx.peak_longitudinal_force(tyre, 0.0) == 0.0


def test_the_ellipse_scales_both_components_and_keeps_the_direction():
    inside = slipx.friction_ellipse(2.0, 3.0, 9.0, 9.4)
    assert not inside.saturated
    assert inside.fx == 2.0 and inside.fy == 3.0

    outside = slipx.friction_ellipse(7.0, 8.0, 9.0, 9.4)
    assert outside.saturated
    # On the boundary, exactly.
    assert math.hypot(outside.fx / 9.0, outside.fy / 9.4) == pytest.approx(
        1.0, rel=1e-12
    )
    # And pointing where it was asked to: both components scale by one factor.
    assert outside.fx / outside.fy == pytest.approx(7.0 / 8.0, rel=1e-12)

    # A wheel with no load has no budget and delivers nothing.
    lifted = slipx.friction_ellipse(5.0, 5.0, 0.0, 0.0)
    assert lifted.fx == 0.0 and lifted.fy == 0.0 and lifted.saturated


def test_the_bound_tyre_is_the_one_the_tier_uses():
    """The property all of this exists for, checked end to end.

    A tyre built here and evaluated here must produce the lateral force the
    double-track tier reports for a wheel at that slip angle and that load. If
    it does not, there are two tyre models in the project again.
    """
    params = slipx.load_reference_car().params_for_tier(
        slipx.Tier.L2_DoubleTrack
    )
    fz_nom = params.mass * 9.80665 / 4.0
    tyre = slipx.make_mf_lite(params.tyre_front, params.c_alpha_f / 2.0,
                              fz_nom)

    model = slipx.VehicleModel.create(slipx.Tier.L2_DoubleTrack, params)
    state = slipx.VehicleState()
    state.vel_body.x = 5.0
    diagnostics = slipx.StepDiagnostics()
    for _ in range(4000):
        model.step(state, slipx.DriveInput(0.06, slipx.hold_speed(state, 5.0)),
                   1e-3, diagnostics)

    for wheel in (0, 1):  # the front pair, which carries no drive force
        alpha = diagnostics.alpha[wheel]
        fz = diagnostics.fz[wheel]
        assert fz > 0.0
        expected = slipx.mf_lite_fy(tyre, alpha, fz)
        # Not exact: the tier lags the slip angle before the force law
        # (ADR-0026), so the reported alpha and the alpha the force was built
        # from differ by whatever the transient has left. In a settled corner
        # that is small, and the point of the case is that they are the same
        # curve rather than the same number.
        assert diagnostics.fy[wheel] == pytest.approx(expected, rel=0.02)
        assert not diagnostics.tyre_saturated[wheel]
