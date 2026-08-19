# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The CDR codecs and the bag container, held to byte-level round trips."""

from __future__ import annotations

import math
import sqlite3
from pathlib import Path

import pytest

from slipx_id import rosbag
from slipx_id.channels import Channel


class TestCdr:
    def test_imu_round_trips(self) -> None:
        sample = rosbag.ImuSample(
            stamp_ns=1_234_567_890,
            angular_velocity=(0.1, -0.2, 1.5),
            linear_acceleration=(0.5, -2.5, 9.8),
        )
        decoded = rosbag.decode_imu(rosbag.encode_imu(sample))
        assert decoded == sample

    def test_pose_round_trips_through_the_quaternion(self) -> None:
        for yaw in (-3.0, -0.5, 0.0, 1.2, 3.1):
            sample = rosbag.PoseSample(
                stamp_ns=42, x=1.25, y=-3.5, yaw=yaw
            )
            decoded = rosbag.decode_pose_stamped(
                rosbag.encode_pose_stamped(sample)
            )
            assert decoded.x == sample.x and decoded.y == sample.y
            assert decoded.yaw == pytest.approx(yaw, abs=1e-12)

    def test_joint_state_round_trips(self) -> None:
        sample = rosbag.JointSample(
            stamp_ns=7,
            names=("wheel_front_left", "a_longer_joint_name", "x"),
            velocities=(10.0, -20.5, 0.25),
        )
        decoded = rosbag.decode_joint_state(rosbag.encode_joint_state(sample))
        assert decoded == sample

    def test_drive_round_trips_at_float32_precision(self) -> None:
        sample = rosbag.DriveSample(
            stamp_ns=9, steering_angle=0.107, speed=0.0, acceleration=3.5
        )
        decoded = rosbag.decode_drive(rosbag.encode_drive(sample))
        assert decoded.steering_angle == pytest.approx(0.107, abs=1e-7)
        assert decoded.acceleration == pytest.approx(3.5, abs=1e-6)

    def test_alignment_survives_odd_length_strings(self) -> None:
        # A frame_id whose length breaks 8-byte alignment is exactly where a
        # hand-rolled CDR goes wrong; the doubles after it must still land.
        for name in ("", "a", "ab", "abc", "abcd", "abcde"):
            sample = rosbag.JointSample(
                stamp_ns=1, names=(name,), velocities=(1.5,)
            )
            decoded = rosbag.decode_joint_state(
                rosbag.encode_joint_state(sample)
            )
            assert decoded.velocities == (1.5,)

    def test_big_endian_is_refused_by_name(self) -> None:
        payload = bytearray(rosbag.encode_imu(
            rosbag.ImuSample(0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        ))
        payload[1] = 0x00  # CDR_BE
        with pytest.raises(ValueError, match="little-endian"):
            rosbag.decode_imu(bytes(payload))


class TestBagContainer:
    def test_write_and_read_a_bag(self, tmp_path: Path) -> None:
        imu = rosbag.encode_imu(
            rosbag.ImuSample(5_000_000, (0.0, 0.0, 0.4), (1.0, 2.0, 0.0))
        )
        directory = rosbag.write_bag(
            tmp_path / "run", {"/imu": (rosbag.IMU_TYPE, [(5_000_000, imu)])}
        )
        assert (directory / "metadata.yaml").exists()

        bag = rosbag.read_bag(directory)
        assert bag["/imu"].type == rosbag.IMU_TYPE
        stamp, payload = bag["/imu"].messages[0]
        assert stamp == 5_000_000
        assert rosbag.decode_imu(payload).angular_velocity[2] == 0.4

    def test_the_written_bag_is_a_real_rosbag2_schema(self, tmp_path: Path) -> None:
        directory = rosbag.write_bag(tmp_path / "run", {})
        database = next(directory.glob("*.db3"))
        connection = sqlite3.connect(database)
        tables = {
            row[0]
            for row in connection.execute(
                "SELECT name FROM sqlite_master WHERE type='table'"
            )
        }
        connection.close()
        assert {"schema", "metadata", "topics", "messages"} <= tables

    def test_a_wrong_topic_type_is_refused_by_name(self, tmp_path: Path) -> None:
        imu = rosbag.encode_imu(
            rosbag.ImuSample(1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        )
        directory = rosbag.write_bag(
            tmp_path / "run",
            {
                "/pose": (rosbag.IMU_TYPE, [(1, imu)]),
                "/imu": (rosbag.IMU_TYPE, [(1, imu)]),
                "/joint_states": (rosbag.IMU_TYPE, [(1, imu)]),
                "/drive": (rosbag.IMU_TYPE, [(1, imu)]),
            },
        )
        with pytest.raises(ValueError, match="sensor_msgs/msg/Imu"):
            rosbag.bag_channels(directory, rosbag.DEFAULT_TOPIC_MAP)

    def test_a_missing_topic_names_what_is_there(self, tmp_path: Path) -> None:
        directory = rosbag.write_bag(tmp_path / "run", {})
        with pytest.raises(ValueError, match="no topic '/pose'"):
            rosbag.bag_channels(directory, rosbag.DEFAULT_TOPIC_MAP)


class TestRecordingBridge:
    def test_a_recording_survives_the_bag(self, tmp_path: Path) -> None:
        # Write a hand-built recording out and read it back: every channel
        # returns with its timestamps, at worst float32-rounded where the
        # wire format is float32 (the drive command).
        from slipx_id.reconstruct import Bench
        from slipx_id.synthetic import ManoeuvreRecording

        times = tuple(0.01 * (i + 1) for i in range(50))
        command_times = tuple(0.001 * i for i in range(500))

        def wave(scale: float, ts) -> Channel:
            return Channel(
                ts, tuple(scale * math.sin(3.0 * t) for t in ts)
            )

        bench = Bench(3.5, 0.16, 0.16, 0.06, 0.24, 0.24, 0.05, 0.05)
        recording = ManoeuvreRecording(
            name="wave",
            dt=1e-3,
            bench=bench,
            channels={
                "pose.x": wave(1.0, times),
                "pose.y": wave(2.0, times),
                "pose.yaw": wave(0.5, times),
                "imu.ax": wave(3.0, times),
                "imu.ay": wave(4.0, times),
                "imu.yaw_rate": wave(1.5, times),
                "wheel.FL": wave(40.0, times),
                "wheel.FR": wave(41.0, times),
                "wheel.RL": wave(42.0, times),
                "wheel.RR": wave(43.0, times),
                "cmd.steer": wave(0.2, command_times),
                "cmd.accel": wave(2.0, command_times),
            },
        )
        directory = rosbag.write_recording(recording, tmp_path / "wave")
        returned = rosbag.read_recording(directory, bench, name="wave")

        for name in recording.channels:
            original = recording.channel(name)
            loaded = returned.channel(name)
            assert len(original) == len(loaded), name
            tolerance = 1e-6 if name.startswith("cmd.") else 1e-12
            for a, b in zip(original.values, loaded.values):
                assert b == pytest.approx(a, abs=tolerance), name
            for a, b in zip(original.times, loaded.times):
                assert b == pytest.approx(a, abs=1e-9), name
