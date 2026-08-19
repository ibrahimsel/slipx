# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The bridge node (ADR-0050).

One node, one simulation, N agents, one thread: spin the callbacks, advance
the paced simulation, publish what the sensor rig delivered. The physics,
the sensors and every ray are native (ADR-0047, ADR-0049); what happens here
is message assembly, and the RMW benchmark measures whether that is fast
enough before anything is promised about it.

Commands hold like a servo: the latest ``AckermannDriveStamped`` per agent
is applied every step, the steering angle directly and the speed through a
proportional acceleration demand whose gain is a named mechanisation of the
speed controller a VESC runs. The published ``odom`` is the encoder's own
belief, dead-reckoned from its speed and the commanded steering angle
through the kinematic bicycle, so it diverges from ground truth by exactly
the slip and the lag: wrong the way the real one is wrong.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan

import slipx


@dataclass
class BridgeConfig:
    """Everything the bridge decides, recorded in its manifest."""

    track_dir: Path
    car_dirs: List[Path]
    real_time_factor: float = 1.0
    ground_truth: bool = True
    ground_truth_rate_hz: float = 100.0
    clock_rate_hz: float = 100.0
    # The speed loop standing in for a VESC's controller: a mechanisation,
    # named here rather than buried (the same rule RaceConfig follows).
    speed_gain: float = 4.0
    seed: int = 0
    grid_spacing_m: float = 1.5
    out_dir: Optional[Path] = None
    namespace: str = "car_"
    tier: "slipx.Tier" = None  # default L2, resolved in __post_init__

    def __post_init__(self) -> None:
        if self.tier is None:
            self.tier = slipx.Tier.L2_DoubleTrack


def _sim_time(t: float) -> TimeMsg:
    seconds = int(t)
    nanoseconds = int(round((t - seconds) * 1.0e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return TimeMsg(sec=seconds, nanosec=nanoseconds)


def _yaw_to_quaternion(msg, yaw: float) -> None:
    msg.z = math.sin(0.5 * yaw)
    msg.w = math.cos(0.5 * yaw)


def _centreline_points(track_dir: Path) -> List[Tuple[float, float]]:
    """The centreline's (x, y) samples, for placing the grid."""
    import slipx_schema

    track = slipx_schema.load_track(track_dir)
    points: List[Tuple[float, float]] = []
    with open(track.centreline, encoding="utf-8", newline="") as handle:
        for row in csv.reader(handle):
            if not row or row[0].lstrip().startswith("#"):
                continue
            points.append((float(row[0]), float(row[1])))
    return points


def _grid_pose(points: List[Tuple[float, float]], distance: float
               ) -> Tuple[float, float, float]:
    """Pose on the centreline at an arc distance from the first sample."""
    travelled = 0.0
    for a, b in zip(points, points[1:]):
        step = math.hypot(b[0] - a[0], b[1] - a[1])
        if travelled + step >= distance and step > 0.0:
            u = (distance - travelled) / step
            heading = math.atan2(b[1] - a[1], b[0] - a[0])
            return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]),
                    heading)
        travelled += step
    raise ValueError(
        f"the grid needs {distance:.1f} m of centreline and the track is "
        f"shorter; fewer cars or a longer track"
    )


class Bridge(Node):
    """The simulation, spoken over ROS 2. See the module docstring."""

    def __init__(self, config: BridgeConfig) -> None:
        super().__init__("slipx_bridge")
        self.config = config

        # The same loaders every consumer uses: nothing here parses.
        self.cars = [slipx.load_car(path) for path in config.car_dirs]
        pairs = []
        for car in self.cars:
            pairs.append((car.spec.tyre_front.compound,
                          car.spec.tyre_front.surface))
            pairs.append((car.spec.tyre_rear.compound,
                          car.spec.tyre_rear.surface))
        self.track = slipx.load_scene_track(config.track_dir, pairs)

        sim_config = slipx.SimulationConfig()
        sim_config.master_seed = config.seed
        # A live run is decided partly by message timing, so the manifest
        # must say NOT REPRODUCIBLE: validation mode, which also paces the
        # wall clock so the bridge does not reimplement pacing (ADR-0050).
        sim_config.mode = slipx.RunMode.Validation
        sim_config.real_time_factor = config.real_time_factor
        self.sim = slipx.Simulation(sim_config)
        # Replay from the log is the one reproducibility promise a live run
        # keeps (ADR-0044), so the log is always on.
        self.sim.set_input_logging(True)

        # The latest command per agent, held like a servo holds one.
        self._commands: List[Tuple[float, float]] = [
            (0.0, 0.0) for _ in self.cars
        ]
        self._acceleration_warned: List[bool] = [False for _ in self.cars]

        points = _centreline_points(config.track_dir)
        for index, car in enumerate(self.cars):
            spec = slipx.AgentSpec()
            spec.name = car.name
            spec.tier = config.tier
            spec.params = car.params_for_tier(config.tier)
            geometry = car.spec.raw["dynamics"]["geometry"]
            spec.footprint_length = float(geometry["length"])
            spec.footprint_width = float(geometry["width"])
            x, y, heading = _grid_pose(points,
                                       index * config.grid_spacing_m)
            spec.initial_state.pos.x = x
            spec.initial_state.pos.y = y
            spec.initial_state.yaw = heading
            spec.policy = self._make_policy(index)
            self.sim.add_agent(spec)

        # Sensors: each car's own file, and the composed world (walls plus
        # the other cars), sized to the longest range any sensor asks for.
        self._sensors = [slipx.sensors_for(car) for car in self.cars]
        max_range = 1.0
        for sensors in self._sensors:
            for lidar in sensors.lidars:
                max_range = max(max_range, lidar.spec.range_max)
        self.world = slipx.TrackWorld(self.track, self.sim, max_range)
        self.rig = slipx.SensorRig(self.sim, self.world, seed=config.seed)
        for index, sensors in enumerate(self._sensors):
            self.rig.attach(index, sensors)

        # Frame ids come from each sensor's mount, the field carried for
        # exactly this, prefixed with the agent's namespace.
        self._frames: List[Dict[str, str]] = []
        for index, car in enumerate(self.cars):
            frames: Dict[str, str] = {}
            for entry in car.spec.sensors:
                mount = entry.get("mount", entry["name"])
                frames[entry["name"]] = f"{self._ns(index)}/{mount}"
            self._frames.append(frames)

        # The encoder's own pose belief, dead-reckoned like a VESC driver's.
        self._reckoned = [
            {"x": self.sim.state(i).pos.x, "y": self.sim.state(i).pos.y,
             "yaw": self.sim.state(i).yaw, "distance": 0.0}
            for i in range(len(self.cars))
        ]

        self._scan_pubs: List[Dict[str, object]] = []
        self._imu_pubs: List[Dict[str, object]] = []
        self._odom_pubs: List[Dict[str, object]] = []
        self._gt_pubs: List[object] = []
        for index, sensors in enumerate(self._sensors):
            ns = self._ns(index)
            self._scan_pubs.append({
                lidar.name: self.create_publisher(
                    LaserScan, f"{ns}/{lidar.name}", qos_profile_sensor_data)
                for lidar in sensors.lidars
            })
            self._imu_pubs.append({
                imu.name: self.create_publisher(
                    Imu, f"{ns}/{imu.name}", qos_profile_sensor_data)
                for imu in sensors.imus
            })
            self._odom_pubs.append({
                encoder.name: self.create_publisher(
                    Odometry, f"{ns}/{encoder.name}",
                    qos_profile_sensor_data)
                for encoder in sensors.encoders
            })
            if config.ground_truth:
                self._gt_pubs.append(self.create_publisher(
                    Odometry, f"{ns}/ground_truth/odom", 10))
            self.create_subscription(
                AckermannDriveStamped, f"{ns}/drive",
                self._make_drive_callback(index), 10)

        self._clock_pub = self.create_publisher(Clock, "/clock", 10)
        dt = self.sim.dt
        self._clock_divisor = max(1, round(1.0 / (dt * config.clock_rate_hz)))
        self._gt_divisor = max(
            1, round(1.0 / (dt * config.ground_truth_rate_hz)))

        # One executor, held: rclpy.spin_once(node) builds a fresh executor
        # per call, which is measurable at a kilohertz.
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self)

    # ------------------------------------------------------------- plumbing

    def _ns(self, index: int) -> str:
        return f"/{self.config.namespace}{index}"

    def _make_policy(self, index: int):
        gain = self.config.speed_gain

        def policy(state, _time, _rng):
            steer, speed = self._commands[index]
            command = slipx.DriveInput()
            command.steer_cmd = steer
            command.accel_cmd = gain * (speed - state.vel_body.x)
            return command

        return policy

    def _make_drive_callback(self, index: int):
        def callback(msg: AckermannDriveStamped) -> None:
            drive = msg.drive
            if drive.acceleration != 0.0 and not self._acceleration_warned[index]:
                self._acceleration_warned[index] = True
                self.get_logger().warning(
                    f"{self._ns(index)}/drive sets acceleration, which this "
                    f"bridge ignores: speed is tracked by a proportional "
                    f"demand (gain {self.config.speed_gain}), the "
                    f"mechanisation of a VESC's speed loop"
                )
            self._commands[index] = (drive.steering_angle, drive.speed)

        return callback

    # ------------------------------------------------------------- stepping

    def step_once(self) -> None:
        """Spin the callbacks, advance one paced step, publish deliveries."""
        self._executor.spin_once(timeout_sec=0.0)
        self.sim.advance()
        self.rig.collect()
        self._publish_due()

    def run(self) -> None:
        while rclpy.ok():
            self.step_once()

    def _publish_due(self) -> None:
        steps = self.sim.step_count
        now = self.sim.time

        if steps % self._clock_divisor == 0:
            self._clock_pub.publish(Clock(clock=_sim_time(now)))

        for index, sensors in enumerate(self._sensors):
            for lidar in sensors.lidars:
                publisher = self._scan_pubs[index][lidar.name]
                for scan in self.rig.take_scans(index, lidar.name):
                    publisher.publish(
                        self._laser_scan(index, lidar, scan))
            for imu in sensors.imus:
                publisher = self._imu_pubs[index][imu.name]
                for reading in self.rig.take_imu(index, imu.name):
                    publisher.publish(self._imu(index, imu, reading))
            for encoder in sensors.encoders:
                publisher = self._odom_pubs[index][encoder.name]
                for reading in self.rig.take_odometry(index, encoder.name):
                    publisher.publish(
                        self._odometry(index, encoder, reading))

        if self._gt_pubs and steps % self._gt_divisor == 0:
            for index, publisher in enumerate(self._gt_pubs):
                publisher.publish(self._ground_truth(index, now))

    # ------------------------------------------------------------- messages

    def _laser_scan(self, index: int, lidar, scan) -> LaserScan:
        spec = lidar.spec
        msg = LaserScan()
        msg.header.stamp = _sim_time(scan.stamp_time)
        msg.header.frame_id = self._frames[index].get(
            lidar.name, f"{self._ns(index)}/{lidar.name}")
        msg.angle_min = spec.angle_min
        span = spec.angle_max - spec.angle_min
        msg.angle_increment = span / float(spec.rays)
        msg.angle_max = spec.angle_min + msg.angle_increment * (spec.rays - 1)
        msg.time_increment = 1.0 / (spec.rate_hz * float(spec.rays))
        msg.scan_time = 1.0 / spec.rate_hz
        msg.range_min = spec.range_min
        msg.range_max = spec.range_max
        # NaN for a dropped or out-of-window ray, never zero: zero is a wall
        # against the mast (ADR-0006, on the wire).
        msg.ranges = [
            ray.range if ray.valid else math.nan for ray in scan.rays
        ]
        return msg

    def _imu(self, index: int, imu, reading) -> Imu:
        sample = reading.sample
        msg = Imu()
        msg.header.stamp = _sim_time(reading.stamp_time)
        msg.header.frame_id = self._frames[index].get(
            imu.name, f"{self._ns(index)}/{imu.name}")
        msg.linear_acceleration.x = sample.ax
        msg.linear_acceleration.y = sample.ay
        msg.linear_acceleration.z = sample.az
        msg.angular_velocity.z = sample.yaw_rate
        # No orientation estimate: the covariance convention for "field not
        # populated" is minus one in the first element.
        msg.orientation_covariance[0] = -1.0
        return msg

    def _odometry(self, index: int, encoder, reading) -> Odometry:
        sample = reading.sample
        # The dead reckoner a VESC driver runs: distance from the encoder,
        # heading from the COMMANDED steering angle through the kinematic
        # bicycle. It drifts under slip and lag because the real one does.
        reckoned = self._reckoned[index]
        travelled = sample.distance - reckoned["distance"]
        reckoned["distance"] = sample.distance
        steer, _speed = self._commands[index]
        wheelbase = self.cars[index].params.lf + self.cars[index].params.lr
        reckoned["yaw"] += travelled * math.tan(steer) / wheelbase
        reckoned["x"] += travelled * math.cos(reckoned["yaw"])
        reckoned["y"] += travelled * math.sin(reckoned["yaw"])

        msg = Odometry()
        msg.header.stamp = _sim_time(reading.stamp_time)
        msg.header.frame_id = f"{self._ns(index)}/odom"
        msg.child_frame_id = self._frames[index].get(
            encoder.name, f"{self._ns(index)}/base_link")
        msg.pose.pose.position.x = reckoned["x"]
        msg.pose.pose.position.y = reckoned["y"]
        _yaw_to_quaternion(msg.pose.pose.orientation, reckoned["yaw"])
        msg.twist.twist.linear.x = sample.speed
        return msg

    def _ground_truth(self, index: int, now: float) -> Odometry:
        state = self.sim.state(index)
        msg = Odometry()
        msg.header.stamp = _sim_time(now)
        msg.header.frame_id = "map"
        msg.child_frame_id = f"{self._ns(index)}/base_link"
        msg.pose.pose.position.x = state.pos.x
        msg.pose.pose.position.y = state.pos.y
        _yaw_to_quaternion(msg.pose.pose.orientation, state.yaw)
        msg.twist.twist.linear.x = state.vel_body.x
        msg.twist.twist.linear.y = state.vel_body.y
        msg.twist.twist.angular.z = state.yaw_rate
        return msg

    # ------------------------------------------------------------ manifests

    def write_manifests(self) -> Optional[Path]:
        """The run manifest and the bridge's own, side by side.

        The simulation's manifest says what it can verify (and that a
        validation run is NOT REPRODUCIBLE); the bridge manifest records
        what the simulation cannot know it was part of.
        """
        out = self.config.out_dir
        if out is None:
            return None
        out = Path(out)
        out.mkdir(parents=True, exist_ok=True)
        (out / "run_manifest.json").write_text(
            self.sim.manifest().to_json(), encoding="utf-8")
        bridge = {
            "bridge": "slipx_ros",
            "slipx": slipx.__version__,
            "track": str(self.config.track_dir),
            "cars": [str(path) for path in self.config.car_dirs],
            "ground_truth": self.config.ground_truth,
            "real_time_factor": self.config.real_time_factor,
            "speed_gain": self.config.speed_gain,
            "seed": self.config.seed,
            "namespace": self.config.namespace,
            "reproducibility": (
                "a live run is decided partly by message timing; replay "
                "the recorded input log for bit-identity (ADR-0044)"
            ),
        }
        (out / "bridge_manifest.json").write_text(
            json.dumps(bridge, indent=2) + "\n", encoding="utf-8")
        return out


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="SlipX to ROS 2: the bridge (ADR-0050)")
    parser.add_argument("--track", required=True, type=Path)
    parser.add_argument("--car", dest="cars", action="append", required=True,
                        type=Path, help="car directory; repeat per agent")
    parser.add_argument("--real-time-factor", type=float, default=1.0)
    parser.add_argument("--no-ground-truth", action="store_true",
                        help="do not offer ground truth topics; recorded in "
                             "the bridge manifest")
    parser.add_argument("--speed-gain", type=float, default=4.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=Path, default=None,
                        help="directory for the run and bridge manifests")
    arguments = parser.parse_args(argv)

    rclpy.init()
    bridge = Bridge(BridgeConfig(
        track_dir=arguments.track,
        car_dirs=list(arguments.cars),
        real_time_factor=arguments.real_time_factor,
        ground_truth=not arguments.no_ground_truth,
        speed_gain=arguments.speed_gain,
        seed=arguments.seed,
        out_dir=arguments.out,
    ))
    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.write_manifests()
        bridge.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
