# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""SCH-02 error quality, SCH-03 legality, SCH-04 consistency, SCH-05 tyres."""

from __future__ import annotations

import re

import pytest

from conftest import derived_b
from slipx_schema import (
    LENGTH_MAX_M,
    LENGTH_MIN_M,
    ValidationError,
    WIDTH_MAX_M,
    WIDTH_MIN_M,
    load_car,
)

# --------------------------------------------------------------------- SCH-02


def test_a_missing_field_names_itself_and_says_why_it_was_not_defaulted(
    car_factory,
) -> None:
    path = car_factory("dynamics.yaml", lambda d: d.pop("mass"))
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)

    message = str(excinfo.value)
    assert "mass" in message
    assert "no default" in message
    assert "dynamics.yaml" in message
    assert "SCH-02" in message


def test_an_out_of_range_value_reports_the_permitted_range(car_factory) -> None:
    path = car_factory("dynamics.yaml", lambda d: d.update(mass=99.0))
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)

    errors = excinfo.value.errors
    assert any(e.path == "mass" for e in errors)
    permitted = next(e.permitted for e in errors if e.path == "mass")
    assert ">" in permitted and "<=" in permitted


def test_the_permitted_range_carries_the_unit(car_factory) -> None:
    # The range comes out of the schema, and the unit out of the schema's own
    # description, so the message cannot drift away from the rule.
    path = car_factory("dynamics.yaml", lambda d: d["geometry"].update(lf=5.0))
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)
    assert "[m]" in str(excinfo.value)


def test_a_nested_field_reports_its_full_path(car_factory) -> None:
    path = car_factory("dynamics.yaml", lambda d: d["geometry"].update(wheel_radius=-1.0))
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)
    assert any(e.path == "geometry.wheel_radius" for e in excinfo.value.errors)


def test_unknown_fields_are_refused_not_ignored(car_factory) -> None:
    # An ignored field is a parameter its author believed was in effect. This
    # is also the only defence against a typo in a field name, which otherwise
    # silently reverts a car to whatever the omitted value would have been.
    path = car_factory("dynamics.yaml", lambda d: d.update(masss=3.5))
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)
    assert "masss" in str(excinfo.value)
    assert "believed was in effect" in str(excinfo.value)


def test_wrong_enum_value_lists_what_is_allowed(car_factory) -> None:
    path = car_factory(
        "dynamics.yaml", lambda d: d["drivetrain"].update(differential="viscous")
    )
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)
    permitted = str(excinfo.value)
    assert "spool" in permitted and "open" in permitted and "lsd" in permitted


def test_a_conditional_requirement_is_enforced(car_factory) -> None:
    # An LSD without a preload is an LSD nobody has specified.
    path = car_factory(
        "dynamics.yaml", lambda d: d["drivetrain"].update(differential="lsd")
    )
    with pytest.raises(ValidationError, match="lsd_preload"):
        load_car(path)


# --------------------------------------------------------------------- SCH-03


@pytest.mark.parametrize(
    "field,value",
    [
        ("width", WIDTH_MIN_M - 0.001),
        ("width", WIDTH_MAX_M + 0.001),
        ("length", LENGTH_MIN_M - 0.001),
        ("length", LENGTH_MAX_M + 0.001),
    ],
)
def test_a_car_outside_the_ruleset_dimensions_is_refused(
    car_factory, field: str, value: float
) -> None:
    # SCH-03: a car that validates shall be a legal car.
    path = car_factory("dynamics.yaml", lambda d: d["geometry"].update({field: value}))
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)
    assert "SCH-03" in str(excinfo.value)
    assert "could not be entered" in str(excinfo.value)


def test_dimensions_at_the_limits_are_legal(car_factory) -> None:
    # Boundaries are inclusive. A car built exactly to the maximum is legal,
    # and refusing it would be a rule this project invented.
    path = car_factory(
        "dynamics.yaml",
        lambda d: d["geometry"].update(width=WIDTH_MAX_M, length=LENGTH_MAX_M),
    )
    assert load_car(path).params.mass == 3.5


def test_a_wheelbase_longer_than_the_car_is_refused(car_factory) -> None:
    # Each number passes its own range check; only together are they absurd.
    path = car_factory(
        "dynamics.yaml", lambda d: d["geometry"].update(lf=0.35, lr=0.35)
    )
    with pytest.raises(ValidationError, match="exceeds the overall length"):
        load_car(path)


def test_a_track_wider_than_the_car_is_refused(car_factory) -> None:
    path = car_factory("dynamics.yaml", lambda d: d["geometry"].update(track_front=0.34))
    with pytest.raises(ValidationError, match="wheels cannot be outside"):
        load_car(path)


# --------------------------------------------------------------------- SCH-04


def test_an_inertia_tensor_violating_the_triangle_inequality_is_refused(
    car_factory,
) -> None:
    # Follows from the definition of the inertia tensor, so a violation does
    # not describe an unusual car; it describes no rigid body.
    path = car_factory(
        "dynamics.yaml", lambda d: d["inertia"].update(ixx=0.01, iyy=0.01, izz=0.10)
    )
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)
    assert "triangle inequality" in str(excinfo.value)
    assert "No rigid body" in str(excinfo.value)


def test_a_non_positive_definite_tensor_is_refused(car_factory) -> None:
    path = car_factory(
        "dynamics.yaml",
        lambda d: d["inertia"].update(ixx=0.025, iyy=0.070, izz=0.050, ixy=0.5),
    )
    with pytest.raises(ValidationError, match="positive definite"):
        load_car(path)


def test_a_small_product_of_inertia_is_accepted(car_factory) -> None:
    # Asymmetric cars exist. The check must refuse impossibility, not novelty.
    path = car_factory("dynamics.yaml", lambda d: d["inertia"].update(ixy=0.002))
    assert load_car(path).params.izz == 0.050


def test_an_inertia_from_a_full_scale_paper_is_warned_about(car_factory) -> None:
    # The single most likely way a 1/10 car acquires a wrong inertia: a value
    # copied from a vehicle dynamics paper about a real car. Two orders of
    # magnitude out, and every individual field still inside its range.
    path = car_factory(
        "dynamics.yaml", lambda d: d["inertia"].update(izz=2.0, ixx=1.5, iyy=1.5)
    )
    car = load_car(path)
    assert any("uniform box" in w for w in car.warnings)
    assert any("units" in w for w in car.warnings)


def test_a_suspiciously_high_cog_is_warned_about_not_refused(car_factory) -> None:
    path = car_factory("dynamics.yaml", lambda d: d["geometry"].update(h_cog=0.20))
    car = load_car(path)
    assert any("h_cog" in w for w in car.warnings)
    assert car.params.h_cog == 0.20  # loaded, because a tall car is possible


def test_a_tyre_whose_stiffness_and_friction_disagree_is_warned_about(
    car_factory,
) -> None:
    # c_alpha and mu_y0 together imply a saturation slip angle. If it comes out
    # at 60 degrees, the two numbers are not describing the same tyre.
    path = car_factory()
    tyre = path / "tyres" / "sponge_carpet.yaml"
    text = tyre.read_text(encoding="utf-8")
    softened = re.sub(r"^  c_alpha: .*$", "  c_alpha: 8.0", text, flags=re.M)
    assert softened != text, "the c_alpha line moved; this edit did nothing"
    tyre.write_text(softened, encoding="utf-8")
    car = load_car(path)
    assert any("linear region" in w for w in car.warnings)


# --------------------------------------------------------------------- SCH-05


def test_an_unresolvable_tyre_reference_stops_the_run(car_factory) -> None:
    # Never a fallback. Substituting some other surface's coefficients is
    # exactly how asphalt numbers end up on a sports hall floor.
    path = car_factory(
        "dynamics.yaml", lambda d: d["tyres"]["front"].update(surface="asphalt")
    )
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)
    assert "SCH-05" in str(excinfo.value)
    assert "sponge_asphalt.yaml" in str(excinfo.value)


def test_a_tyre_file_disagreeing_with_its_own_name_is_refused(car_factory) -> None:
    path = car_factory()
    tyre = path / "tyres" / "sponge_carpet.yaml"
    tyre.write_text(
        tyre.read_text(encoding="utf-8").replace("surface: carpet", "surface: concrete", 1),
        encoding="utf-8",
    )
    with pytest.raises(ValidationError, match="disagree"):
        load_car(path)


def test_different_tyres_front_and_rear_are_supported_and_reported(car_factory) -> None:
    # A real possibility, and one L1 cannot fully represent: it has a single
    # friction clip. The loader takes the weaker and says so rather than
    # averaging quietly.
    path = car_factory()
    rubber = path / "tyres" / "rubber_carpet.yaml"
    rubber.write_text(
        (path / "tyres" / "sponge_carpet.yaml")
        .read_text(encoding="utf-8")
        .replace("compound: sponge", "compound: rubber")
        .replace("mu_y0: 1.10", "mu_y0: 0.85")
        .replace("mu_x0: 1.05", "mu_x0: 0.80"),
        encoding="utf-8",
    )

    import yaml

    dynamics_path = path / "dynamics.yaml"
    document = yaml.safe_load(dynamics_path.read_text(encoding="utf-8"))
    document["tyres"]["rear"]["compound"] = "rubber"
    dynamics_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    car = load_car(path)
    assert car.tyre_front.compound == "sponge"
    assert car.tyre_rear.compound == "rubber"
    assert car.params.mu_clip == 0.85
    assert any("takes the lower" in note for note in car.notes)


def test_a_c_e_pair_below_the_peak_threshold_is_quiet(car_factory) -> None:
    # C = 1.5 with E = 0.6 puts the peak at about 3.8 times the linear
    # saturation angle, under the threshold of 4. Unusual but not warned:
    # the band a real tyre occupies is 1.5 to 3, and the rule leaves
    # headroom above it before objecting (ADR-0030).
    path = car_factory()
    tyre = path / "tyres" / "sponge_carpet.yaml"
    tyre.write_text(
        tyre.read_text(encoding="utf-8")
        .replace("C: 1.68", "C: 1.5")
        .replace("E: 0.42", "E: 0.6"),
        encoding="utf-8",
    )
    car = load_car(path)
    assert not any("peak" in w for w in car.warnings)


def test_a_c_e_pair_with_a_peak_no_car_reaches_is_warned_about(car_factory) -> None:
    # C = 1.45 with E = 0.65 puts the peak at about 4.4 times the linear
    # saturation angle, just over the threshold. Every individual bound in
    # the schema accepts the pair; only the pair is wrong.
    path = car_factory()
    tyre = path / "tyres" / "sponge_carpet.yaml"
    tyre.write_text(
        tyre.read_text(encoding="utf-8")
        .replace("C: 1.68", "C: 1.45")
        .replace("E: 0.42", "E: 0.65"),
        encoding="utf-8",
    )
    car = load_car(path)
    assert any("peak" in w and "slip sweep" in w for w in car.warnings)


def test_the_peak_multiple_is_the_number_tyre_hpp_documents() -> None:
    # The C++ header states 2.7 for the reference pair and "above 20" for the
    # legal-but-pathological one; the two statements must be the same
    # function, or the warning and the model disagree about the same tyre.
    from slipx_schema.rules import mf_lite_peak_multiple

    assert mf_lite_peak_multiple(1.68, 0.42) == pytest.approx(2.7, abs=0.1)
    assert mf_lite_peak_multiple(1.05, 0.87) > 20.0
    # E at exactly 1 with a low C has no finite peak at all, and legal values
    # can say that; the function reports it rather than looping.
    assert mf_lite_peak_multiple(1.05, 1.0) == float("inf")


def test_a_stated_b_that_disagrees_with_the_derived_value_is_warned_about(
    car_factory,
) -> None:
    # B is derived from the linear block and never consumed (ADR-0023). A
    # stated B that disagrees would be silently ignored, which is exactly the
    # in-effect-but-ignored parameter the rules exist to surface.
    path = car_factory()
    tyre = path / "tyres" / "sponge_carpet.yaml"
    tyre.write_text(
        tyre.read_text(encoding="utf-8").replace("mf_lite:\n", "mf_lite:\n  B: 9.0\n"),
        encoding="utf-8",
    )
    car = load_car(path)
    assert any("B = 9.0" in w and "never consumed" in w for w in car.warnings)


def test_a_stated_b_that_agrees_with_the_derived_value_is_quiet(car_factory) -> None:
    path = car_factory()
    tyre = path / "tyres" / "sponge_carpet.yaml"
    agreeing = f"mf_lite:\n  B: {derived_b(tyre):.4f}\n"
    tyre.write_text(
        tyre.read_text(encoding="utf-8").replace("mf_lite:\n", agreeing),
        encoding="utf-8",
    )
    car = load_car(path)
    assert not any("mf_lite.B" in w for w in car.warnings)


def test_a_stated_b_without_a_nominal_load_cannot_be_checked_and_says_so(
    car_factory,
) -> None:
    path = car_factory()
    tyre = path / "tyres" / "sponge_carpet.yaml"
    text = tyre.read_text(encoding="utf-8")
    text = "\n".join(
        line for line in text.splitlines() if not line.startswith("nominal_load:")
    ) + "\n"
    text = text.replace("mf_lite:\n", "mf_lite:\n  B: 9.0\n")
    tyre.write_text(text, encoding="utf-8")
    car = load_car(path)
    assert any("cannot be checked" in w for w in car.warnings)


def test_c_kappa_bounds_are_enforced_on_both_sides(car_factory) -> None:
    # Both sides, because a mutation dropping only the upper bound escaped
    # the negative-value test: the bound reflects what encoder slip ratio
    # against IMU acceleration can plausibly identify on a 1/10 tyre.
    for bad in ("-5.0", "6000.0"):
        path = car_factory()
        tyre = path / "tyres" / "sponge_carpet.yaml"
        tyre.write_text(
            tyre.read_text(encoding="utf-8").replace(
                "c_kappa: 120.0", f"c_kappa: {bad}"
            ),
            encoding="utf-8",
        )
        with pytest.raises(ValidationError, match="c_kappa"):
            load_car(path)


def test_a_tyre_file_must_carry_its_own_provenance(car_factory) -> None:
    path = car_factory()
    tyre = path / "tyres" / "sponge_carpet.yaml"
    text = tyre.read_text(encoding="utf-8")
    tyre.write_text(text[: text.index("provenance:")], encoding="utf-8")
    with pytest.raises(ValidationError, match="provenance"):
        load_car(path)


def test_an_identified_set_must_carry_residuals(car_factory) -> None:
    # SCH-06 in miniature: an identified parameter without a residual is a
    # provisional parameter with a better title.
    path = car_factory("provenance.yaml", lambda d: d.update(label="identified"))
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)
    assert "residuals" in str(excinfo.value)
    assert "contributor" in str(excinfo.value)
