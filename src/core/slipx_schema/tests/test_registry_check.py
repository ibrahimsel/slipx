# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Schema 0.4.0: the compound vocabulary and the registry's acceptance bar
(ADR-0041)."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from slipx_schema import ValidationError, load_car
from slipx_schema.rules import check_registry_submission


def _rename_compound(car_dir: Path, new_compound: str) -> None:
    tyre_path = car_dir / "tyres" / "sponge_carpet.yaml"
    document = yaml.safe_load(tyre_path.read_text(encoding="utf-8"))
    document["compound"] = new_compound
    (car_dir / "tyres" / f"{new_compound}_carpet.yaml").write_text(
        yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
    )
    dynamics_path = car_dir / "dynamics.yaml"
    dynamics = yaml.safe_load(dynamics_path.read_text(encoding="utf-8"))
    dynamics["tyres"]["front"]["compound"] = new_compound
    dynamics["tyres"]["rear"]["compound"] = new_compound
    dynamics_path.write_text(
        yaml.safe_dump(dynamics, sort_keys=False), encoding="utf-8"
    )


def test_a_compound_outside_the_old_enum_loads(car_factory) -> None:
    # 0.4.0 opened the vocabulary: an identification tool fits tyres the
    # two-word list never anticipated, and the same argument that made
    # surfaces free text applies.
    path = car_factory()
    _rename_compound(path, "foam-35")
    car = load_car(path)
    assert car.tyre_front.compound == "foam-35"


def test_a_malformed_compound_is_still_refused(car_factory) -> None:
    path = car_factory()
    tyre_path = path / "tyres" / "sponge_carpet.yaml"
    document = yaml.safe_load(tyre_path.read_text(encoding="utf-8"))
    document["compound"] = "Sponge Tyre!"  # spaces and case: not a slug
    tyre_path.write_text(
        yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
    )
    with pytest.raises(ValidationError):
        load_car(path)


class TestRegistrySubmission:
    def _accepted(self) -> dict:
        return {
            "schema_version": "0.4.0",
            "label": "identified",
            "source": "a car",
            "method": "the manoeuvre library",
            "date": "2026-08-19",
            "contributor": "A. Team",
            "residuals": {"mu_y0": {"value": 1.1, "stddev": 0.02}},
            "validation_report": "report.html",
            "data": [{"name": "skidpad", "sha256": "0" * 64}],
        }

    def test_a_complete_submission_clears_the_bar(self) -> None:
        assert check_registry_submission(self._accepted()) == []

    @pytest.mark.parametrize(
        "removal, named",
        [
            ("contributor", "contributor"),
            ("residuals", "residuals"),
            ("validation_report", "validation_report"),
            ("data", "data"),
        ],
    )
    def test_each_missing_obligation_is_named(self, removal, named) -> None:
        document = self._accepted()
        del document[removal]
        errors = check_registry_submission(document)
        assert any(named in error.path for error in errors)

    def test_a_provisional_set_is_refused_outright(self) -> None:
        document = self._accepted()
        document["label"] = "provisional"
        errors = check_registry_submission(document)
        assert any("label" in error.path for error in errors)

    def test_empty_residuals_do_not_count(self) -> None:
        document = self._accepted()
        document["residuals"] = {}
        errors = check_registry_submission(document)
        assert any("residuals" in error.path for error in errors)
