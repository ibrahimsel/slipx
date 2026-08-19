# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The sensor file at schema 0.5.0 (ADR-0048).

The file now carries what the sensor models consume: typed per-sensor blocks,
strictly validated when present, and a latency jitter that is the half-width
of the uniform spread the models implement. The 0.4.0 step converts the old
jitter_stddev by the square root of three (variance-preserving) and fails
loudly on a free-form noise object, whose keys were never defined.
"""

from __future__ import annotations

import math

import pytest

from slipx_schema import load_car
from slipx_schema.errors import ValidationError


def sensor_named(car, name):
    for entry in car.sensors:
        if entry["name"] == name:
            return entry
    raise AssertionError(f"no sensor named {name}")


def test_the_reference_sensors_file_carries_the_typed_blocks(reference_car):
    car = load_car(reference_car)

    scan = sensor_named(car, "scan")
    assert scan["type"] == "lidar_2d"
    assert scan["lidar_2d"]["rays"] == 1080
    assert scan["latency"]["jitter"] == 0.003

    imu = sensor_named(car, "imu")
    assert imu["imu"]["accel_noise_density"] == 0.002

    encoder = sensor_named(car, "odom")
    assert encoder["wheel_encoder"]["wheels_used"] == [False, False, True, True]


# ------------------------------------------------------------- the migration


def to_0_4_0(document):
    """Rewrite the shipped sensors document into its 0.4.0 shape."""
    document["schema_version"] = "0.4.0"
    for entry in document["sensors"]:
        for block in ("lidar_2d", "imu", "wheel_encoder"):
            entry.pop(block, None)
        latency = entry.get("latency")
        if latency and "jitter" in latency:
            latency["jitter_stddev"] = latency.pop("jitter")


def test_a_0_4_0_file_migrates_with_the_variance_preserved(car_factory):
    recorded = {}

    def edit(document):
        to_0_4_0(document)
        recorded["stddev"] = document["sensors"][0]["latency"]["jitter_stddev"]

    car = load_car(car_factory("sensors.yaml", edit))

    latency = sensor_named(car, "scan")["latency"]
    assert "jitter_stddev" not in latency
    assert latency["jitter"] == pytest.approx(
        recorded["stddev"] * math.sqrt(3.0)
    )


def test_a_migrated_file_still_loads_and_still_refuses_at_use(car_factory):
    # The house pattern since 0.2.0: a migrated file gains nothing it did not
    # carry. The typed blocks are absent, the document is valid, and whoever
    # builds sensors from it refuses by name (that refusal lives in the slipx
    # package, beside the rig; here the claim is only that loading works).
    car = load_car(car_factory("sensors.yaml", to_0_4_0))
    assert "lidar_2d" not in sensor_named(car, "scan")


def test_a_free_noise_object_fails_loudly_rather_than_being_dropped(
    car_factory,
):
    def edit(document):
        to_0_4_0(document)
        document["sensors"][0]["noise"] = {"stddev": 0.02}

    with pytest.raises(ValueError, match="noise.*restate|Restate"):
        load_car(car_factory("sensors.yaml", edit))


def test_an_empty_noise_object_carried_nothing_and_is_removed(car_factory):
    def edit(document):
        to_0_4_0(document)
        document["sensors"][0]["noise"] = {}

    car = load_car(car_factory("sensors.yaml", edit))
    assert "noise" not in sensor_named(car, "scan")


# ------------------------------------------------------- strict validation


def test_an_unknown_field_in_a_typed_block_is_refused(car_factory):
    def edit(document):
        document["sensors"][0]["lidar_2d"]["beam_divergence"] = 0.01

    with pytest.raises(ValidationError, match="beam_divergence"):
        load_car(car_factory("sensors.yaml", edit))


def test_a_partial_typed_block_is_refused_not_defaulted(car_factory):
    def edit(document):
        del document["sensors"][1]["imu"]["gyro_noise_density"]

    with pytest.raises(ValidationError, match="gyro_noise_density"):
        load_car(car_factory("sensors.yaml", edit))


def test_a_block_for_another_type_is_refused(car_factory):
    # An imu block on a lidar_2d sensor is a parameter set its author
    # believed was in effect; ignoring it is the SCH-02 failure.
    def edit(document):
        document["sensors"][0]["imu"] = document["sensors"][1]["imu"]

    with pytest.raises(ValidationError):
        load_car(car_factory("sensors.yaml", edit))


def test_wheels_used_must_name_all_four_wheels(car_factory):
    def edit(document):
        document["sensors"][2]["wheel_encoder"]["wheels_used"] = [True, True]

    with pytest.raises(ValidationError, match="wheels_used"):
        load_car(car_factory("sensors.yaml", edit))


def test_the_old_jitter_field_is_refused_at_0_5_0(car_factory):
    # jitter_stddev names a distribution the models do not implement; at
    # 0.5.0 it is not a legal field, so a file that skipped the migration by
    # declaring the new version gets refused rather than reinterpreted.
    def edit(document):
        latency = document["sensors"][0]["latency"]
        latency["jitter_stddev"] = latency.pop("jitter")

    with pytest.raises(ValidationError, match="jitter_stddev"):
        load_car(car_factory("sensors.yaml", edit))
