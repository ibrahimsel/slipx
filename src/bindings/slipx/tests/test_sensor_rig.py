# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""sensors.yaml through the sensor rig (ADR-0047, ADR-0048).

The C++ suite owns the rig's semantics (schedules, latency, distortion,
streams); what these tests own is the wiring above it: a car directory's
sensor file becomes the rig's spec structs with nothing defaulted, refusals
arrive by name, and the whole chain runs from Python without touching the
trajectory.
"""

from __future__ import annotations

import math

import pytest

import slipx


def wall_world(wall_x=10.0):
    def world(agent, pose, bearing):
        hit = slipx.Hit()
        along = math.cos(bearing)
        if along > 1e-9 and pose.x < wall_x:
            hit.hit = True
            hit.range = (wall_x - pose.x) / along
        return hit

    return world


def l2_agent(car, speed=3.0):
    spec = slipx.AgentSpec()
    spec.tier = slipx.Tier.L2_DoubleTrack
    spec.params = car.params_for_tier(slipx.Tier.L2_DoubleTrack)
    spec.initial_state.vel_body.x = speed
    spec.policy = lambda state, time, rng: slipx.DriveInput(0.0, 0.5)
    return spec


def to_0_4_0(document):
    document["schema_version"] = "0.4.0"
    for entry in document["sensors"]:
        for block in ("lidar_2d", "imu", "wheel_encoder"):
            entry.pop(block, None)
        latency = entry.get("latency")
        if latency and "jitter" in latency:
            latency["jitter_stddev"] = latency.pop("jitter")


# ------------------------------------------------------------- the mapping


def test_the_reference_car_builds_the_full_suite():
    sensors = slipx.sensors_for(slipx.load_reference_car())

    assert len(sensors.lidars) == 1
    assert len(sensors.imus) == 1
    assert len(sensors.encoders) == 1

    scan = sensors.lidars[0]
    assert scan.name == "scan"
    assert scan.spec.rays == 1080
    assert scan.spec.rate_hz == 40.0
    assert scan.spec.latency_s == 0.012
    assert scan.spec.latency_jitter_s == 0.003
    assert scan.spec.dropout_probability == 0.001

    imu = sensors.imus[0]
    assert imu.phase == 0.25
    assert imu.spec.accel_noise_density == 0.002

    encoder = sensors.encoders[0]
    assert encoder.spec.wheels_used == [False, False, True, True]
    assert encoder.spec.wheel_radius == 0.05


def test_a_car_without_a_sensors_file_is_the_cheap_opponent(car_factory):
    path = car_factory("car.yaml", lambda d: d.pop("sensors"))
    sensors = slipx.sensors_for(slipx.load_car(path))
    assert len(sensors.lidars) == 0
    assert len(sensors.imus) == 0
    assert len(sensors.encoders) == 0


def test_a_pre_0_5_0_file_refuses_at_use_naming_the_block(car_factory):
    # The migrated file loads (the house pattern: migration invents nothing)
    # and the refusal happens here, where the sensor is built.
    car = slipx.load_car(car_factory("sensors.yaml", to_0_4_0))
    with pytest.raises(ValueError, match="lidar_2d"):
        slipx.sensors_for(car)


def test_lidar_3d_is_refused_rather_than_substituted(car_factory):
    def edit(document):
        entry = document["sensors"][0]
        entry["type"] = "lidar_3d"
        entry.pop("lidar_2d")
        entry.pop("dropout_probability")

    car = slipx.load_car(car_factory("sensors.yaml", edit))
    with pytest.raises(ValueError, match="P4"):
        slipx.sensors_for(car)


def test_a_missing_field_is_refused_by_name(car_factory):
    def edit(document):
        document["sensors"][0].pop("dropout_probability")

    car = slipx.load_car(car_factory("sensors.yaml", edit))
    with pytest.raises(ValueError, match="dropout_probability"):
        slipx.sensors_for(car)


def test_a_jitter_beyond_its_constant_is_refused(car_factory):
    def edit(document):
        document["sensors"][1]["latency"]["jitter"] = 0.5

    car = slipx.load_car(car_factory("sensors.yaml", edit))
    with pytest.raises(ValueError, match="jitter"):
        slipx.sensors_for(car)


def test_a_missing_phase_is_refused_by_name(car_factory):
    def edit(document):
        document["sensors"][1].pop("phase")

    car = slipx.load_car(car_factory("sensors.yaml", edit))
    with pytest.raises(ValueError, match="phase"):
        slipx.sensors_for(car)


def test_a_missing_latency_is_refused_by_name(car_factory):
    # An ideal transport is a claim like any other: state it, zero if that
    # is what is meant.
    def edit(document):
        document["sensors"][0].pop("latency")

    car = slipx.load_car(car_factory("sensors.yaml", edit))
    with pytest.raises(ValueError, match="latency"):
        slipx.sensors_for(car)


# ------------------------------------------------------------- the rig


def test_the_reference_suite_runs_and_observation_is_free():
    car = slipx.load_reference_car()
    sensors = slipx.sensors_for(car)

    def run(sensed):
        sim = slipx.Simulation()
        sim.add_agent(l2_agent(car))
        rig = slipx.SensorRig(sim, wall_world(), seed=7)
        if sensed:
            rig.attach(0, sensors)
        for _ in range(60):
            sim.advance()
            rig.collect()
        return sim, rig

    bare, _ = run(False)
    sensed, rig = run(True)

    # Observation is free: the sensored car drove the identical trajectory,
    # to the hash (ADR-0047).
    assert bare.trajectory_hash() == sensed.trajectory_hash()

    # And the suite actually observed: a scan of the wall, an IMU reading
    # gravity on its z axis, an odometry that believes its own radius.
    scan = rig.latest_scan(0, "scan")
    assert scan is not None
    assert any(ray.valid for ray in scan.rays)
    imu = rig.latest_imu(0, "imu")
    assert imu.sample.az == pytest.approx(9.80665, abs=0.5)
    odometry = rig.latest_odometry(0, "odom")
    assert odometry.sample.distance > 0.0


def test_two_identically_seeded_rigs_observe_identical_streams():
    car = slipx.load_reference_car()
    sensors = slipx.sensors_for(car)

    def observe():
        sim = slipx.Simulation()
        sim.add_agent(l2_agent(car))
        rig = slipx.SensorRig(sim, wall_world(), seed=11)
        rig.attach(0, sensors)
        for _ in range(60):
            sim.advance()
            rig.collect()
        scan = rig.latest_scan(0, "scan")
        imu = rig.take_imu(0, "imu")
        return (
            [(ray.range if ray.valid else None) for ray in scan.rays],
            [(r.sample.time, r.sample.ax, r.stamp_time) for r in imu],
        )

    assert observe() == observe()


def test_an_encoder_below_l2_is_refused_by_name():
    car = slipx.load_reference_car()
    sensors = slipx.sensors_for(car)

    sim = slipx.Simulation()
    spec = slipx.AgentSpec()
    spec.tier = slipx.Tier.L1_Bicycle
    sim.add_agent(spec)

    rig = slipx.SensorRig(sim, wall_world(), seed=1)
    with pytest.raises(ValueError, match="wheel encoders need L2"):
        rig.attach(0, sensors)


def test_the_native_track_world_feeds_the_rig():
    # The whole racing chain without a single Python call per ray
    # (ADR-0049): a validated track directory, the reference car's tyre
    # pairs, two footprinted cars, and the scan sees the opponent ahead
    # nearer than the wall while the side rays see the corridor.
    from pathlib import Path

    track_dir = (
        Path(__file__).resolve().parents[4]
        / "examples" / "tracks" / "paddock_stadium"
    )
    car = slipx.load_reference_car()
    pairs = [
        (car.spec.tyre_front.compound, car.spec.tyre_front.surface),
        (car.spec.tyre_rear.compound, car.spec.tyre_rear.surface),
    ]
    track = slipx.load_scene_track(track_dir, pairs)
    assert track.closed
    assert track.length > 20.0

    sim = slipx.Simulation()
    for x in (-4.0, -2.0):
        spec = l2_agent(car, speed=0.0)
        spec.initial_state.pos.x = x
        spec.initial_state.pos.y = -3.0
        spec.footprint_length = 0.50
        spec.footprint_width = 0.30
        sim.add_agent(spec)

    world = slipx.TrackWorld(track, sim, max_range=30.0)
    rig = slipx.SensorRig(sim, world, seed=3)
    rig.attach(0, slipx.sensors_for(car))
    for _ in range(40):
        sim.advance()
        rig.collect()

    scan = rig.latest_scan(0, "scan")
    assert scan is not None
    forward = min(
        (ray for ray in scan.rays if ray.valid), key=lambda r: abs(r.angle)
    )
    left = min(
        (ray for ray in scan.rays if ray.valid),
        key=lambda r: abs(r.angle - 1.5707963),
    )
    assert forward.range == pytest.approx(1.75, abs=0.1)
    assert left.range == pytest.approx(0.75, abs=0.1)


def test_the_wrong_surface_is_refused_with_both_names():
    from pathlib import Path

    track_dir = (
        Path(__file__).resolve().parents[4]
        / "examples" / "tracks" / "paddock_stadium"
    )
    with pytest.raises(Exception, match="carpet"):
        slipx.load_scene_track(track_dir, [("slick", "asphalt")])


def test_attach_after_collect_is_refused():
    sim = slipx.Simulation()
    spec = slipx.AgentSpec()
    spec.tier = slipx.Tier.L1_Bicycle
    sim.add_agent(spec)

    rig = slipx.SensorRig(sim, seed=1)
    sim.advance()
    rig.collect()
    with pytest.raises(RuntimeError, match="collect"):
        rig.attach(0, slipx.AgentSensors())
