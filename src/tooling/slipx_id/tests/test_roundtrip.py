# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The synthetic self-test (M6.2): generate, fit, recover.

Data is generated from known parameters through the forward model, the
staged fits run exactly as they would on a real bag, and every MF-lite
parameter must come back. This is the only test of the fitter that needs no
hardware, so it is the foundation the rest of P2 stands on (ADR-0038).

One honest asymmetry, asserted rather than hidden: the Magic Formula shape
pair (C, E) is recovered as a *curve*, not as coordinates. Steady-state
driving cannot sit on the falling branch of the tyre curve, and the model's
falling branch is gentle, so whole families of (C, E) describe nearly the
same tyre; the fitter reports the correlation, and what must round-trip is
the force the tyre produces, which is what every consumer of the file
actually uses.
"""

from __future__ import annotations

import pytest

import slipx
from slipx_id import stages, synthetic


def _test_car() -> "slipx.VehicleParams":
    """Struct defaults with the ESC opened up, so the launch is traction
    limited rather than current limited and mu_x0 is observable."""
    params = slipx.VehicleParams()
    params.current_max = 400.0
    params.accel_max = 12.0
    return params


@pytest.fixture(scope="module")
def pipeline():
    """One full identification session, shared by every assertion below."""
    params = _test_car()
    bench = synthetic.bench_of(params)

    coast = [
        synthetic.coastdown(params, v, duration=6.0) for v in (10.0, 7.0, 4.0)
    ]
    resistances = stages.fit_coastdown(coast)

    lateral_recs = []
    for speed in (1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0):
        lateral_recs.append(
            synthetic.skidpad(params, speed=speed, steer=0.107, duration=6.0)
        )
    lateral_recs.append(
        synthetic.ramp_steer(params, speed=3.0, rate=0.01, peak=0.35)
    )
    lateral_recs.append(
        synthetic.ramp_steer(params, speed=3.0, rate=-0.01, peak=0.35)
    )
    heavy = synthetic.ballasted(params, 0.35)
    lateral_recs.append(
        synthetic.skidpad(heavy, speed=2.0, steer=0.107, duration=6.0)
    )
    lateral_recs.append(
        synthetic.ramp_steer(heavy, speed=3.0, rate=0.01, peak=0.35)
    )
    lateral = stages.fit_lateral(lateral_recs, bench, sample_stride=3)

    launch = synthetic.launch(params, duration=5.0)
    c_kappa = stages.fit_c_kappa(launch, resistances, lateral.k_mu)
    mu_x0, _spread = stages.fit_mu_x0(
        launch, resistances, lateral.k_mu, window=(0.5, 3.0)
    )

    steps = [
        synthetic.step_steer(params, speed=2.0, amplitude=0.10),
        synthetic.step_steer(params, speed=6.0, amplitude=0.05),
        synthetic.step_steer(params, speed=5.0, amplitude=0.25),
    ]
    base = params.copy()
    base.c_alpha_f = lateral.c_alpha_f
    base.c_alpha_r = lateral.c_alpha_r
    base.roll_resist = resistances.value("roll_resist")
    base.drag_coeff = resistances.value("drag_coeff")
    base.c_kappa = c_kappa.value("c_kappa")
    for tyre in (base.tyre_front, base.tyre_rear):
        tyre.mu_y0 = lateral.mu_y0
        tyre.mu_x0 = mu_x0
        tyre.k_mu = lateral.k_mu
        tyre.shape_c = lateral.shape_c
        tyre.curvature_e = lateral.curvature_e
    transient = stages.fit_transient(steps, base)

    return {
        "params": params,
        "bench": bench,
        "resistances": resistances,
        "lateral": lateral,
        "c_kappa": c_kappa,
        "mu_x0": mu_x0,
        "transient": transient,
    }


def _recovered(got: float, want: float, tolerance: float) -> None:
    assert got == pytest.approx(want, rel=tolerance), (
        f"fitted {got:.5g} against true {want:.5g}, "
        f"outside {tolerance:.0%}"
    )


def test_coastdown_recovers_the_resistances(pipeline) -> None:
    truth = pipeline["params"]
    fit = pipeline["resistances"]
    _recovered(fit.value("roll_resist"), truth.roll_resist, 0.01)
    _recovered(fit.value("drag_coeff"), truth.drag_coeff, 0.01)


def test_lateral_recovers_stiffness_peak_and_load_sensitivity(pipeline) -> None:
    truth = pipeline["params"]
    lateral = pipeline["lateral"]
    _recovered(lateral.c_alpha_f, truth.c_alpha_f, 0.015)
    _recovered(lateral.c_alpha_r, truth.c_alpha_r, 0.015)
    _recovered(lateral.mu_y0, truth.tyre_front.mu_y0, 0.02)
    _recovered(lateral.k_mu, truth.tyre_front.k_mu, 0.15)


def test_longitudinal_recovers_slip_stiffness_and_peak(pipeline) -> None:
    truth = pipeline["params"]
    _recovered(pipeline["c_kappa"].value("c_kappa"), truth.c_kappa, 0.06)
    _recovered(pipeline["mu_x0"], truth.tyre_front.mu_x0, 0.06)


def test_transient_recovers_the_delays(pipeline) -> None:
    truth = pipeline["params"]
    transient = pipeline["transient"]
    _recovered(
        transient.value("relax_length"), truth.tyre_front.relax_length, 0.15
    )
    _recovered(
        transient.value("steer_bandwidth"), truth.steer_bandwidth, 0.08
    )
    _recovered(transient.value("steer_damping"), truth.steer_damping, 0.12)


def test_the_shape_pair_round_trips_as_a_curve(pipeline) -> None:
    # The identifiable object. Whatever coordinates the fitter settled on,
    # the lateral force it predicts must track the true tyre through the
    # whole working range, at the nominal load and under transfer, and past
    # the peak (the true peak sits near 0.12 rad).
    truth = pipeline["params"]
    bench = pipeline["bench"]
    lateral = pipeline["lateral"]
    transient = pipeline["transient"]

    fitted = slipx.TyreCoefficients()
    fitted.mu_y0 = lateral.mu_y0
    fitted.mu_x0 = pipeline["mu_x0"]
    fitted.k_mu = lateral.k_mu
    fitted.relax_length = transient.value("relax_length")
    fitted.shape_c = transient.value("shape_c")
    fitted.curvature_e = transient.value("curvature_e")

    static = bench.static_front_per_tyre
    true_tyre = slipx.make_mf_lite(
        truth.tyre_front, 0.5 * truth.c_alpha_f, static
    )
    fit_tyre = slipx.make_mf_lite(fitted, 0.5 * lateral.c_alpha_f, static)

    for fz, tolerance in ((static, 0.03), (1.6 * static, 0.04)):
        for i in range(1, 43):
            alpha = i * 0.005  # up to 0.21 rad
            want = -slipx.mf_lite_fy(true_tyre, alpha, fz)
            got = -slipx.mf_lite_fy(fit_tyre, alpha, fz)
            assert got == pytest.approx(want, rel=tolerance), (
                f"curve off by {abs(got - want) / want:.1%} at "
                f"alpha={alpha:.3f}, fz={fz:.2f}"
            )


def test_the_shape_entanglement_is_reported_not_hidden(pipeline) -> None:
    # A fit that printed confident C and E coordinates without flagging that
    # they trade off would be exactly the confident lie the claim discipline
    # forbids. At least one stage that fitted the pair must name it.
    def names_pair(report) -> bool:
        return any(
            {"shape_c", "curvature_e"} == {a, b}
            or {"mu_y0", "shape_c"} == {a, b}
            for a, b, _ in report.correlations
        )

    assert names_pair(pipeline["lateral"].report) or names_pair(
        pipeline["transient"]
    )


def test_ballast_restatement_is_the_same_physical_tyre() -> None:
    # ballasted() must apply the ADR-0039 power laws: the derived stiffness
    # factor B is then invariant, which is the whole-curve statement.
    params = _test_car()
    heavy = synthetic.ballasted(params, 0.35)

    bench = synthetic.bench_of(params)
    heavy_bench = synthetic.bench_of(heavy)

    def b_of(p, static):
        tyre = slipx.make_mf_lite(p.tyre_front, 0.5 * p.c_alpha_f, static)
        return tyre.b

    assert b_of(heavy, heavy_bench.static_front_per_tyre) == pytest.approx(
        b_of(params, bench.static_front_per_tyre), rel=1e-12
    )
