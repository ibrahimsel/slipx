# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""SCH-01: version gating and migration."""

from __future__ import annotations

import pytest

from slipx_schema import SCHEMA_VERSION, SchemaVersionError, Version, compatibility, load_car
from slipx_schema import migrate as migrate_module


def test_version_parsing_and_ordering() -> None:
    assert Version.parse("1.2.3") == Version(1, 2, 3)
    assert Version.parse("0.1.0") < Version.parse("0.2.0")
    assert Version.parse("0.9.0") < Version.parse("1.0.0")

    for bad in ("1.2", "1.2.3.4", "v1.2.3", "one.two.three", ""):
        with pytest.raises(ValueError, match="semantic version"):
            Version.parse(bad)


def test_current_major_at_or_below_current_minor_is_accepted() -> None:
    current = Version.parse(SCHEMA_VERSION)
    ok, _ = compatibility(current)
    assert ok
    ok, _ = compatibility(Version(current.major, current.minor, current.patch + 7))
    assert ok


def test_unknown_major_is_refused_with_a_usable_explanation() -> None:
    ok, reason = compatibility(Version(99, 0, 0))
    assert not ok
    assert "major" in reason
    assert "changed meaning" in reason
    assert "99" in reason


def test_newer_minor_is_refused_rather_than_partially_understood() -> None:
    # The tempting choice is to accept it and ignore the unknown fields. That
    # is precisely the silent behaviour SCH-02 forbids: the file's author
    # believed those parameters were in effect.
    current = Version.parse(SCHEMA_VERSION)
    ok, reason = compatibility(Version(current.major, current.minor + 1, 0))
    assert not ok
    assert "does not know about" in reason


def test_missing_version_is_refused(car_factory) -> None:
    path = car_factory("dynamics.yaml", lambda d: d.pop("schema_version"))
    with pytest.raises(SchemaVersionError, match="no schema_version"):
        load_car(path)


def test_unquoted_version_is_refused_with_the_reason(car_factory) -> None:
    # YAML reads 0.1 as a float and "0.1.0" as a string. A file whose version
    # changes type depending on how many dots were typed is a file that should
    # be corrected, not guessed at.
    path = car_factory("dynamics.yaml", lambda d: d.update(schema_version=0.1))
    with pytest.raises(SchemaVersionError, match="quoted string"):
        load_car(path)


def test_future_major_in_any_file_is_refused(car_factory) -> None:
    path = car_factory("limits.yaml", lambda d: d.update(schema_version="9.0.0"))
    with pytest.raises(SchemaVersionError, match="9.0.0"):
        load_car(path)


def test_tyre_files_are_version_gated_too(car_factory) -> None:
    path = car_factory()
    tyre = path / "tyres" / "sponge_carpet.yaml"
    text = tyre.read_text(encoding="utf-8").replace(
        'schema_version: "0.2.0"', 'schema_version: "7.0.0"', 1
    )
    tyre.write_text(text, encoding="utf-8")
    with pytest.raises(SchemaVersionError, match="7.0.0"):
        load_car(path)


# --------------------------------------------------------------- migrations


def test_the_0_1_0_to_0_2_0_migration_exists_for_every_kind() -> None:
    # 0.2.0 added only optional fields (ADR-0030), so each step is the
    # identity; it is still registered explicitly per kind, because a gap in
    # the chain is a loud release bug and an implicit identity would hide a
    # real one.
    assert migrate_module.available() == [
        (kind, 1)
        for kind in sorted(
            ("car", "dynamics", "limits", "sensors", "provenance", "tyre")
        )
    ]


def test_a_0_1_0_car_directory_still_loads_by_migration(car_factory) -> None:
    # A car directory written against schema 0.1.0, byte for byte: every file
    # declares 0.1.0, the tyre file has no c_kappa and carries the B that
    # 0.1.0 required. It must keep loading for the tiers it always served,
    # and the migration must not invent the field it lacks.
    path = car_factory()
    for name in (
        "car.yaml", "dynamics.yaml", "limits.yaml", "provenance.yaml",
        "sensors.yaml", "tyres/sponge_carpet.yaml",
    ):
        target = path / name
        target.write_text(
            target.read_text(encoding="utf-8").replace(
                'schema_version: "0.2.0"', 'schema_version: "0.1.0"'
            ),
            encoding="utf-8",
        )
    tyre = path / "tyres" / "sponge_carpet.yaml"
    text = tyre.read_text(encoding="utf-8")
    lines = [
        line for line in text.splitlines() if "c_kappa" not in line
    ]
    text = "\n".join(lines) + "\n"
    # The B a 0.1.0 file was required to state, consistent with the linear
    # block so the derived-B check stays quiet.
    text = text.replace("mf_lite:\n", "mf_lite:\n  B: 3.78\n")
    tyre.write_text(text, encoding="utf-8")

    car = load_car(path)
    assert car.schema_version == "0.2.0"  # migrated forward
    assert car.tyre_front.c_kappa is None  # not invented
    assert car.params.c_kappa is None
    assert car.warnings == []


def test_the_migration_chain_runs_stepwise(monkeypatch: pytest.MonkeyPatch) -> None:
    # A synthetic two-step chain, to check the mechanism before a real schema
    # change needs it. Writing this under release pressure, with a registry of
    # contributed files depending on it, is the situation being avoided.
    monkeypatch.setattr(migrate_module, "_MIGRATIONS", {}, raising=True)
    monkeypatch.setattr(migrate_module, "CURRENT", Version(0, 3, 0), raising=True)

    calls = []

    def zero_to_one(document):
        calls.append("0->1")
        out = dict(document)
        out["added_in_1"] = True
        return out

    def one_to_two(document):
        calls.append("1->2")
        out = dict(document)
        assert out["added_in_1"] is True, "each step must see the previous one's work"
        out["added_in_2"] = True
        return out

    def two_to_three(document):
        calls.append("2->3")
        return dict(document)

    migrate_module._MIGRATIONS[("dynamics", 0)] = zero_to_one
    migrate_module._MIGRATIONS[("dynamics", 1)] = one_to_two
    migrate_module._MIGRATIONS[("dynamics", 2)] = two_to_three

    result = migrate_module.migrate("dynamics", {"schema_version": "0.0.0"}, Version(0, 0, 0))

    assert calls == ["0->1", "1->2", "2->3"]
    assert result["added_in_1"] and result["added_in_2"]
    assert result["schema_version"] == "0.3.0"


def test_a_gap_in_the_chain_is_a_loud_release_bug(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(migrate_module, "_MIGRATIONS", {}, raising=True)
    monkeypatch.setattr(migrate_module, "CURRENT", Version(0, 2, 0), raising=True)

    with pytest.raises(KeyError, match="release bug"):
        migrate_module.migrate("dynamics", {"schema_version": "0.0.0"}, Version(0, 0, 0))


def test_registering_the_same_migration_twice_is_refused(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(migrate_module, "_MIGRATIONS", {}, raising=True)

    @migrate_module.register("dynamics", 0)
    def _first(document):  # pragma: no cover - never called
        return document

    with pytest.raises(ValueError, match="duplicate"):

        @migrate_module.register("dynamics", 0)
        def _second(document):  # pragma: no cover - never called
            return document
