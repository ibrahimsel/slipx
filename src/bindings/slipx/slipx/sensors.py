# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""sensors.yaml to the sensor rig (ADR-0047, ADR-0048).

This module is the one place the sensor file format and the sensor models
meet: each validated entry from a car directory's ``sensors.yaml`` is mapped
onto the matching spec struct, and anything the file does not carry is
refused by name rather than defaulted. A defaulted noise density would be a
fidelity claim nobody made, which is ADR-0025's argument applied to sensing.

The mapping is deliberately dumb: field for field, no arithmetic. Any
derived quantity belongs in slipx_schema where its inputs are still named.
"""

from __future__ import annotations

from typing import Any, Dict, List

from ._slipx import (
    AgentSensors,
    EncoderSensor,
    ImuSensor,
    LidarSensor,
)

__all__ = ["sensors_for"]


def _named(entry: Dict[str, Any]) -> str:
    return str(entry.get("name", "?"))


def _require(entry: Dict[str, Any], field: str) -> Any:
    if field not in entry:
        raise ValueError(
            f"sensor '{_named(entry)}': no '{field}'. Nothing is defaulted "
            f"(ADR-0025): a sensor parameter that came from nowhere would be "
            f"a fidelity claim nobody made. State it in sensors.yaml, at "
            f"schema 0.5.0 or later (ADR-0048)."
        )
    return entry[field]


def _block(entry: Dict[str, Any]) -> Dict[str, Any]:
    kind = entry["type"]
    if kind not in entry:
        raise ValueError(
            f"sensor '{_named(entry)}': no '{kind}' block. Schema 0.5.0 "
            f"carries the model's parameters in a block named after the "
            f"sensor type, and a file migrated from an older version does "
            f"not gain one (ADR-0048); add it rather than trusting a "
            f"default."
        )
    return entry[kind]


def _schedule(entry: Dict[str, Any]) -> Dict[str, float]:
    latency = _require(entry, "latency")
    for field in ("constant", "jitter"):
        if field not in latency:
            raise ValueError(
                f"sensor '{_named(entry)}': latency has no '{field}'. An "
                f"ideal transport is a claim like any other; state constant "
                f"and jitter, zero if that is what is meant (ADR-0048)."
            )
    constant = float(latency["constant"])
    jitter = float(latency["jitter"])
    if jitter > constant:
        raise ValueError(
            f"sensor '{_named(entry)}': latency jitter {jitter} exceeds the "
            f"constant {constant}. Jitter is symmetric about the constant, "
            f"so this would stamp a message before the instant it measured "
            f"[s]."
        )
    return {
        "rate": float(_require(entry, "rate")),
        "phase": float(_require(entry, "phase")),
        "constant": constant,
        "jitter": jitter,
    }


def _lidar(entry: Dict[str, Any]) -> LidarSensor:
    schedule = _schedule(entry)
    block = _block(entry)
    sensor = LidarSensor()
    sensor.name = entry["name"]
    sensor.phase = schedule["phase"]
    sensor.spec.rate_hz = schedule["rate"]
    sensor.spec.latency_s = schedule["constant"]
    sensor.spec.latency_jitter_s = schedule["jitter"]
    sensor.spec.rays = int(block["rays"])
    sensor.spec.angle_min = float(block["angle_min"])
    sensor.spec.angle_max = float(block["angle_max"])
    sensor.spec.range_min = float(block["range_min"])
    sensor.spec.range_max = float(block["range_max"])
    sensor.spec.noise_base_m = float(block["noise_base"])
    sensor.spec.noise_per_metre = float(block["noise_per_metre"])
    sensor.spec.dropout_probability = float(
        _require(entry, "dropout_probability")
    )
    return sensor


def _imu(entry: Dict[str, Any]) -> ImuSensor:
    schedule = _schedule(entry)
    block = _block(entry)
    sensor = ImuSensor()
    sensor.name = entry["name"]
    sensor.rate_hz = schedule["rate"]
    sensor.phase = schedule["phase"]
    sensor.latency_s = schedule["constant"]
    sensor.latency_jitter_s = schedule["jitter"]
    sensor.spec.accel_noise_density = float(block["accel_noise_density"])
    sensor.spec.gyro_noise_density = float(block["gyro_noise_density"])
    sensor.spec.accel_bias_walk = float(block["accel_bias_walk"])
    sensor.spec.gyro_bias_walk = float(block["gyro_bias_walk"])
    sensor.spec.accel_scale_error = float(block["accel_scale_error"])
    sensor.spec.gyro_scale_error = float(block["gyro_scale_error"])
    sensor.spec.accel_bias_x = float(block["accel_bias_x"])
    sensor.spec.accel_bias_y = float(block["accel_bias_y"])
    sensor.spec.gyro_bias_z = float(block["gyro_bias_z"])
    return sensor


def _encoder(entry: Dict[str, Any]) -> EncoderSensor:
    schedule = _schedule(entry)
    block = _block(entry)
    sensor = EncoderSensor()
    sensor.name = entry["name"]
    sensor.rate_hz = schedule["rate"]
    sensor.phase = schedule["phase"]
    sensor.latency_s = schedule["constant"]
    sensor.latency_jitter_s = schedule["jitter"]
    sensor.spec.counts_per_revolution = float(block["counts_per_revolution"])
    sensor.spec.wheel_radius = float(block["wheel_radius"])
    sensor.spec.wheels_used = [bool(w) for w in block["wheels_used"]]
    return sensor


def sensors_for(car) -> AgentSensors:
    """Build one agent's :class:`AgentSensors` from a loaded car.

    Takes what :func:`slipx.load_car` returns (or the underlying
    ``slipx_schema`` car). A car directory with no sensors file, or an empty
    one, produces an empty ``AgentSensors``: a legal car for a dynamics-only
    study, and the cheap opponent in a race.

    Refuses by name, rather than defaulting, a sensor whose entry cannot
    fill its model's parameters: a missing typed block (every file migrated
    from before 0.5.0), a missing schedule or latency field, and the
    ``lidar_3d`` type, which arrives in P4 and is refused rather than
    substituted by a simpler sensor (ADR-0005's rule, applied to sensing).
    Everything else about the entry was already validated against the
    schema when the car loaded.
    """
    spec = getattr(car, "spec", car)
    entries: List[Dict[str, Any]] = list(getattr(spec, "sensors", []))

    sensors = AgentSensors()
    lidars: List[LidarSensor] = []
    imus: List[ImuSensor] = []
    encoders: List[EncoderSensor] = []

    for entry in entries:
        kind = entry.get("type")
        if kind == "lidar_2d":
            lidars.append(_lidar(entry))
        elif kind == "imu":
            imus.append(_imu(entry))
        elif kind == "wheel_encoder":
            encoders.append(_encoder(entry))
        elif kind == "lidar_3d":
            raise ValueError(
                f"sensor '{_named(entry)}': lidar_3d is not built until P4. "
                f"Refused rather than substituted: a simpler sensor standing "
                f"in for the one the file describes is the fallback this "
                f"project forbids (ADR-0005)."
            )
        else:
            raise ValueError(
                f"sensor '{_named(entry)}': unknown type '{kind}'. The "
                f"schema should have refused this file."
            )

    sensors.lidars = lidars
    sensors.imus = imus
    sensors.encoders = encoders
    return sensors
