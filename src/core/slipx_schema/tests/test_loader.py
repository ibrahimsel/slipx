# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Loading a car directory end to end, and NFR-08's labelling rules."""

from __future__ import annotations

from pathlib import Path

import pytest

from slipx_schema import (
    CarDirectoryError,
    SCHEMA_VERSION,
    ValidationError,
    load_car,
    validate_car,
)


def test_reference_car_loads_and_is_internally_consistent(reference_car: Path) -> None:
    car = load_car(reference_car)

    assert car.name == "reference_1_10"
    assert car.schema_version == SCHEMA_VERSION
    assert car.params.mass == 3.5
    assert car.params.wheelbase == pytest.approx(0.32)

    # Cornering stiffness in a tyre file is per tyre; an axle has two.
    assert car.params.c_alpha_f == 2 * car.tyre_front.c_alpha
    assert car.params.c_alpha_r == 2 * car.tyre_rear.c_alpha

    # The reference car ships clean. If it ever stops doing so, either the
    # shipped car or the plausibility bounds need attention, and a warning
    # nobody reads is how a bad default survives.
    assert car.warnings == []


def test_reference_car_is_labelled_provisional(reference_car: Path) -> None:
    # NFR-08. No parameter set in SlipX has been validated against a real car,
    # and the shipped one must say so in the object, not only in the prose.
    car = load_car(reference_car)
    assert car.provenance.label == "provisional"
    assert car.params.provenance_label == "provisional"
    assert not car.provenance.is_measured_or_identified


def test_summary_leads_with_the_provenance_label(reference_car: Path) -> None:
    # NFR-08 requires the label in TOOLING OUTPUT, not only in documentation.
    summary = load_car(reference_car).summary()
    assert "PROVISIONAL" in summary
    assert "not measured against any vehicle" in summary


def test_defaulting_is_reported_rather_than_silent(reference_car: Path) -> None:
    # SCH-02 prohibits silent defaulting. The reference car omits
    # numerics.v_eps, so the loader must say so out loud.
    car = load_car(reference_car)
    assert car.params.v_eps is None
    assert any("v_eps" in note for note in car.notes)
    assert any("tyres/" in note for note in car.notes)


def test_a_car_with_a_measured_chassis_and_guessed_tyres_is_guessed(car_factory) -> None:
    # The weakest applicable claim is the only honest one: a car is not
    # 'measured' because one of its files is.
    path = car_factory(
        "provenance.yaml",
        lambda d: d.update(
            label="measured",
            source="Weighed on four scales",
            method="Corner scales, battery fitted",
        ),
    )
    car = load_car(path)

    assert car.provenance.label == "measured"
    assert car.tyre_front.provenance.label == "provisional"
    assert car.params.provenance_label == "provisional"
    assert any("as a whole" in note for note in car.notes)


def test_missing_directory_and_missing_file_are_distinguishable(
    car_factory, tmp_path: Path
) -> None:
    with pytest.raises(CarDirectoryError):
        load_car(tmp_path / "not-here")

    path = car_factory()
    (path / "dynamics.yaml").unlink()
    with pytest.raises(CarDirectoryError, match="dynamics.yaml"):
        load_car(path)


def test_manifest_naming_a_file_that_is_not_there(car_factory) -> None:
    path = car_factory("car.yaml", lambda d: d.update(dynamics="does_not_exist.yaml"))
    with pytest.raises(CarDirectoryError, match="does_not_exist.yaml"):
        load_car(path)


def test_empty_and_malformed_files_are_refused(car_factory) -> None:
    path = car_factory()
    (path / "dynamics.yaml").write_text("", encoding="utf-8")
    with pytest.raises(CarDirectoryError, match="empty"):
        load_car(path)

    path = car_factory()
    (path / "dynamics.yaml").write_text("- a\n- list\n", encoding="utf-8")
    with pytest.raises(CarDirectoryError, match="mapping"):
        load_car(path)


def test_yaml_is_loaded_safely(car_factory) -> None:
    # A car file is a data file. Registry entries will be downloaded from
    # strangers, and a loader that can construct arbitrary Python objects is a
    # supply-chain problem waiting for its first contribution.
    path = car_factory()
    (path / "dynamics.yaml").write_text(
        "schema_version: '0.1.0'\n"
        "mass: !!python/object/apply:os.system ['echo pwned']\n",
        encoding="utf-8",
    )
    with pytest.raises(Exception) as excinfo:
        load_car(path)
    assert "python/object" in str(excinfo.value) or "could not determine" in str(
        excinfo.value
    ).lower()


def test_all_errors_are_reported_together(car_factory) -> None:
    # Three mistakes should take one run to find, not three.
    def break_several(document):
        del document["mass"]
        del document["resistance"]
        document["geometry"]["lf"] = 5.0

    path = car_factory("dynamics.yaml", break_several)
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)

    message = str(excinfo.value)
    assert len(excinfo.value.errors) == 3
    assert "mass" in message
    assert "resistance" in message
    assert "geometry.lf" in message


def test_structural_failures_stop_before_the_semantic_rules(car_factory) -> None:
    # Deliberate ordering. The SCH-03 and SCH-04 rules read fields the schema
    # has just failed to guarantee are present or numeric, so running them on
    # a structurally broken document would produce a page of consequential
    # nonsense underneath the one error that matters.
    def break_structure_and_legality(document):
        del document["mass"]
        document["geometry"]["width"] = 0.9  # legal as a number, illegal as a car

    path = car_factory("dynamics.yaml", break_structure_and_legality)
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)

    assert [e.requirement for e in excinfo.value.errors] == ["SCH-02"]

    # And once the structure is sound, the legality rule does fire.
    path = car_factory("dynamics.yaml", lambda d: d["geometry"].update(width=0.9))
    with pytest.raises(ValidationError) as excinfo:
        load_car(path)
    assert any(e.requirement == "SCH-03" for e in excinfo.value.errors)


def test_validate_car_reports_instead_of_raising(car_factory, reference_car: Path) -> None:
    assert validate_car(reference_car).ok

    path = car_factory("dynamics.yaml", lambda d: d.pop("mass"))
    report = validate_car(path)
    assert not report.ok
    assert report.errors


def test_strict_mode_promotes_warnings_to_errors(car_factory) -> None:
    # izz an order of magnitude too large: possible to type, impossible to
    # believe. Loads with a warning by default; refused under strict, which is
    # what a competition harness should use.
    path = car_factory("dynamics.yaml", lambda d: d["inertia"].update(izz=0.5, iyy=0.5))

    car = load_car(path)
    assert any("izz" in w for w in car.warnings)

    with pytest.raises(ValidationError, match="strict"):
        load_car(path, strict=True)


def test_the_raw_documents_are_kept(reference_car: Path) -> None:
    # Whatever the parser did not model is still available, so that a consumer
    # needing a field this version ignores is not forced to reimplement the
    # loader.
    car = load_car(reference_car)
    assert car.raw["dynamics"]["drivetrain"]["differential"] == "spool"
    assert car.raw["limits"]["electrical"]["pack_nominal_v"] == 11.1
    assert len(car.sensors) == 3
