# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The load restatement of ADR-0039.

A tyre file states its coefficients at its own ``nominal_load``; the core
states every tyre at the static per-tyre load of the car wearing it. The
loader bridges the two exactly, because MF-lite's load dependence is a power
law, and these tests hold the bridge to that standard: not approximately
right, but the same physical tyre restated.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from slipx_schema import ValidationError, load_car
from slipx_schema.loader import GRAVITY


def _static_per_tyre(car) -> float:
    """The reference car is 50/50, so one number serves both axles."""
    p = car.params
    return 0.5 * p.mass * GRAVITY * p.lr / (p.lf + p.lr)


def test_the_reference_pairing_is_the_identity(reference_car: Path) -> None:
    # The shipped tyre file states its nominal_load as the reference car's
    # static per-tyre load to the digit, so nothing is scaled and the numbers
    # ADR-0032 chose load exactly as written.
    car = load_car(reference_car)

    assert car.params.c_alpha_f == 420.0
    assert car.params.c_alpha_r == 420.0
    assert car.tyre_front.mu_y0 == 1.10
    assert car.tyre_front.mu_x0 == 1.05
    assert car.params.c_kappa == 120.0
    assert not any("restated" in note for note in car.notes)


def test_a_heavier_car_restates_the_shared_tyre(car_factory) -> None:
    # The registry case: the same (compound, surface) file under a car twice
    # the mass. Twice the static load means the frictions come down by
    # 2^-k_mu and the stiffnesses go up by 2^(1-k_mu), exactly.
    path = car_factory("dynamics.yaml", lambda d: d.update(mass=7.0))
    car = load_car(path)

    ratio = 2.0
    k = 0.15
    assert car.params.c_alpha_f == pytest.approx(
        420.0 * ratio ** (1.0 - k), rel=1e-9
    )
    assert car.tyre_front.mu_y0 == pytest.approx(1.10 * ratio**-k, rel=1e-9)
    assert car.tyre_front.mu_x0 == pytest.approx(1.05 * ratio**-k, rel=1e-9)
    assert car.params.c_kappa == pytest.approx(
        120.0 * ratio ** (1.0 - k), rel=1e-9
    )
    # L1's clip follows the restated friction: it describes this car's tyre
    # at this car's load, not the file's car.
    assert car.params.mu_clip == pytest.approx(1.10 * ratio**-k, rel=1e-9)
    assert any("restated" in note for note in car.notes)

    # C, E and the relaxation length are load-free and must not move.
    assert car.tyre_front.mf_lite["C"] == 1.68
    assert car.tyre_front.mf_lite["E"] == 0.42
    assert car.tyre_front.sigma == 0.045


def test_the_restatement_leaves_the_physical_tyre_unchanged(car_factory) -> None:
    # B = c_alpha / (C * mu_y0 * fz_nom) is the invariant: the restated
    # coefficients describe the same curve. Held as arithmetic on the loaded
    # values rather than trusted from the derivation.
    heavier = load_car(car_factory("dynamics.yaml", lambda d: d.update(mass=7.0)))
    original = load_car(car_factory())

    def b_of(car) -> float:
        t = car.tyre_front
        return t.c_alpha / (t.mf_lite["C"] * t.mu_y0 * t.nominal_load)

    assert b_of(heavier) == pytest.approx(b_of(original), rel=1e-12)

    # And the friction the power law predicts at any absolute load agrees
    # between the two statements of the tyre.
    for fz in (4.0, 8.58081875, 17.0, 30.0):
        k = 0.15
        mu_original = original.tyre_front.mu_y0 * (
            fz / original.tyre_front.nominal_load
        ) ** (-k)
        mu_restated = heavier.tyre_front.mu_y0 * (
            fz / heavier.tyre_front.nominal_load
        ) ** (-k)
        assert mu_restated == pytest.approx(mu_original, rel=1e-12)


def test_without_k_mu_the_file_is_taken_as_stated(car_factory) -> None:
    # A 0.1.0-era file has a nominal_load and no exponent, so no restatement
    # is possible. The values pass through untouched and the note says why;
    # L2 is refused elsewhere for the same missing field.
    import yaml

    heavier = car_factory("dynamics.yaml", lambda d: d.update(mass=7.0))
    tyre_path = heavier / "tyres" / "sponge_carpet.yaml"
    document = yaml.safe_load(tyre_path.read_text(encoding="utf-8"))
    del document["friction"]["k_mu"]
    tyre_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    car = load_car(heavier)
    assert car.params.c_alpha_f == 420.0
    assert car.tyre_front.mu_y0 == 1.10
    assert any("no friction.k_mu" in note for note in car.notes)


def test_an_extreme_ratio_warns_and_strict_refuses(car_factory) -> None:
    # A tyre stated at a fifth of the car's corner weight is more likely a
    # units or reference error than a heavy car. The restatement still
    # happens (the arithmetic is right either way), the warning says so, and
    # strict mode makes it a refusal.
    def fifth(document) -> None:
        document["nominal_load"] = 8.58081875 / 5.0

    path = car_factory("tyres/sponge_carpet.yaml", fifth)
    car = load_car(path)

    assert any("units or reference error" in str(w) for w in car.warnings)
    assert car.params.c_alpha_f == pytest.approx(
        420.0 * 5.0**0.85, rel=1e-9
    )

    with pytest.raises(ValidationError):
        load_car(path, strict=True)


def test_a_small_mismatch_is_still_restated(car_factory) -> None:
    # One per cent is far outside the identity window and well inside what a
    # rounded file produces. The window exists for operation-order ulps, not
    # for tidying away real mismatches.
    def one_percent_high(document) -> None:
        document["nominal_load"] = 8.58081875 * 1.01

    path = car_factory("tyres/sponge_carpet.yaml", one_percent_high)
    car = load_car(path)

    ratio = 1.0 / 1.01
    assert car.params.c_alpha_f == pytest.approx(
        420.0 * ratio**0.85, rel=1e-9
    )
    assert any("restated" in note for note in car.notes)


def test_an_uneven_car_restates_each_axle_at_its_own_load(car_factory) -> None:
    # A nose-heavy car: lf shrinks, lr grows, the front axle carries more.
    # The same tyre file lands on both axles and must be restated per axle,
    # so the loaded car has different axle stiffnesses from one file, and
    # the front (heavier) axle gets the stiffer, lower-friction statement.
    def nose_heavy(document) -> None:
        document["geometry"]["lf"] = 0.12
        document["geometry"]["lr"] = 0.20

    path = car_factory("dynamics.yaml", nose_heavy)
    car = load_car(path)

    weight = 3.5 * GRAVITY
    static_front = 0.5 * weight * 0.20 / 0.32
    static_rear = 0.5 * weight * 0.12 / 0.32
    k = 0.15
    ratio_front = static_front / 8.58081875
    ratio_rear = static_rear / 8.58081875

    assert ratio_front > 1.0 > ratio_rear, "the fixture is nose-heavy"
    assert car.params.c_alpha_f == pytest.approx(
        420.0 * ratio_front ** (1.0 - k), rel=1e-9
    )
    assert car.params.c_alpha_r == pytest.approx(
        420.0 * ratio_rear ** (1.0 - k), rel=1e-9
    )
    assert car.tyre_front.mu_y0 == pytest.approx(
        1.10 * ratio_front**-k, rel=1e-9
    )
    assert car.tyre_rear.mu_y0 == pytest.approx(1.10 * ratio_rear**-k, rel=1e-9)
    # One c_kappa serves all four wheels: the mean of the two per-axle
    # restatements (ADR-0039), and the loader's own note about the axles
    # disagreeing names the values it averaged.
    assert car.params.c_kappa == pytest.approx(
        0.5 * 120.0 * (ratio_front ** (1.0 - k) + ratio_rear ** (1.0 - k)),
        rel=1e-9,
    )


def test_without_a_nominal_load_the_assumption_is_noted(car_factory) -> None:
    def strip(document) -> None:
        del document["nominal_load"]

    path = car_factory("tyres/sponge_carpet.yaml", strip)
    car = load_car(path)

    assert car.params.c_alpha_f == 420.0
    assert any("states no nominal_load" in note for note in car.notes)
