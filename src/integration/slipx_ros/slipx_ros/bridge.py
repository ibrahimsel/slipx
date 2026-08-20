# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The bridge node (ADR-0050).

One node, one simulation, N agents, one thread: spin the callbacks, advance
the paced simulation, publish what the sensor rig delivered. The physics,
the sensors and every ray are native (ADR-0047, ADR-0049); what happens here
is message assembly, and the RMW benchmark measures whether that is fast
enough before anything is promised about it.

TF is REP 105 shaped (ADR-0053): identity mounts on ``tf_static`` (the
sensor models cast from the vehicle origin), ``odom -> base_link`` from the
dead reckoner, and, only while ground truth is offered, a ``map -> odom``
correction that makes the chain compose to the true pose. ``--no-tf``
removes the broadcast for a stack that owns its tree.

The map (ADR-0054) is the raycaster's own walls rasterised to a latched
``OccupancyGrid`` on ``/map``: never a re-derived offset, so the map cannot
disagree with the scans, and never gated on ground truth, because a map is
geometry a real car also has (SLAM gave it one). ``--no-map`` for a stack
that runs its own map server.

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
import threading
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import UInt64
from tf2_msgs.msg import TFMessage

import slipx

from .race_sync import ACK_QOS, ANNOUNCE_QOS, STEP_TOPIC, stamp_to_step

#: The QoS the tf2_ros broadcasters use, restated here because the bridge
#: publishes TFMessage directly and keeps its imports to message packages.
TF_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
TF_STATIC_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

#: The QoS a map server latches with: published once, delivered to every
#: subscriber however late it joins.
MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@dataclass
class BridgeConfig:
    """Everything the bridge decides, recorded in its manifest."""

    track_dir: Path
    car_dirs: List[Path]
    real_time_factor: float = 1.0
    ground_truth: bool = True
    ground_truth_rate_hz: float = 100.0
    # TF (ADR-0053): the dead reckoner's odom -> base_link, identity mounts
    # on tf_static, and, only while ground truth is offered, the map -> odom
    # correction that makes the chain compose to the true pose.
    tf: bool = True
    # The map (ADR-0054): the raycaster's own walls, rasterised and latched
    # once. Not gated on ground truth, because a map is geometry a real car
    # also has; off for a stack that runs its own map server.
    map: bool = True
    map_resolution_m: float = 0.05
    clock_rate_hz: float = 100.0
    # The speed loop standing in for a VESC's controller: a mechanisation,
    # named here rather than buried (the same rule RaceConfig follows).
    speed_gain: float = 4.0
    seed: int = 0
    grid_spacing_m: float = 1.5
    out_dir: Optional[Path] = None
    namespace: str = "car_"
    tier: "slipx.Tier" = None  # default L2, resolved in __post_init__
    # Lockstep (ADR-0051): the bridge announces each step and advances only
    # when every agent has answered through its mailbox (ADR-0044). The
    # policy names what a miss does once the wall-clock timeout expires;
    # "wait" waits forever, which is for development, where a hang you can
    # see beats a forfeit you cannot.
    lockstep: bool = False
    lockstep_policy: str = "wait"
    lockstep_timeout_s: float = 0.0

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
        if config.lockstep:
            # Commands are functions of step indices, not the wall clock, so
            # a lockstep run keeps deterministic mode and its manifest's
            # promise (ADR-0051); the timeout is the escape from a hung
            # stack, answered per agent by the policy below (ADR-0044).
            sim_config.barrier_timeout = config.lockstep_timeout_s
        else:
            # A live run is decided partly by message timing, so the
            # manifest must say NOT REPRODUCIBLE: validation mode, which
            # also paces the wall clock so the bridge does not reimplement
            # pacing (ADR-0050).
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

        policies = {
            "wait": slipx.TimeoutPolicy.Wait,
            "coast": slipx.TimeoutPolicy.Coast,
            "freeze": slipx.TimeoutPolicy.Freeze,
            "dnf": slipx.TimeoutPolicy.Dnf,
        }
        if config.lockstep_policy not in policies:
            raise ValueError(
                f"unknown lockstep_policy '{config.lockstep_policy}'; one "
                f"of {sorted(policies)} (ADR-0044's answers to a miss)")

        self._mailboxes: List[Optional[slipx.CommandMailbox]] = []
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
            if config.lockstep:
                # The barrier's doorway (ADR-0044): an agent has one command
                # source, and in lockstep it is the mailbox.
                spec.mailbox = slipx.CommandMailbox()
                spec.timeout_policy = policies[config.lockstep_policy]
                self._mailboxes.append(spec.mailbox)
            else:
                spec.policy = self._make_policy(index)
                self._mailboxes.append(None)
            self.sim.add_agent(spec)

        # Sensors: each car's own file, and the composed world (walls plus
        # the other cars), sized to the longest range any sensor asks for.
        self._sensors = [slipx.sensors_for(car) for car in self.cars]
        max_range = 1.0
        for sensors in self._sensors:
            for lidar in sensors.lidars:
                max_range = max(max_range, lidar.spec.range_max)
        self.world = slipx.TrackWorld(self.track, self.sim, max_range)
        # The walls the scans see are the walls the physics enforces: the
        # raycaster's own polylines, handed straight across, so a car can no
        # more end a step across a wall than a ray can pass one. Latched
        # here because walls must precede the first advance.
        self.sim.add_wall(self.world.wall_left, self.track.closed)
        self.sim.add_wall(self.world.wall_right, self.track.closed)
        self.rig = slipx.SensorRig(self.sim, self.world, seed=config.seed)
        for index, sensors in enumerate(self._sensors):
            self.rig.attach(index, sensors)

        # Frame ids come from each sensor's mount, the field carried for
        # exactly this, prefixed with the agent's namespace. Unlike topics
        # the prefix has no leading slash: tf2 refuses to look up any frame
        # that starts with one, so a slashed frame id can never join a tree.
        self._frames: List[Dict[str, str]] = []
        for index, car in enumerate(self.cars):
            frames: Dict[str, str] = {}
            for entry in car.spec.sensors:
                mount = entry.get("mount", entry["name"])
                frames[entry["name"]] = f"{self._frame_ns(index)}/{mount}"
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
            if config.lockstep:
                self.create_subscription(
                    UInt64, f"{ns}/drive_ack",
                    self._make_ack_callback(index), ACK_QOS)

        # TF (ADR-0053). The mounts are latched once and are identity: the
        # sensor models cast from the vehicle origin, so identity is the
        # modelled truth, not a convenience.
        self._tf_pub: Optional[object] = None
        self._tf_static_pub: Optional[object] = None
        if config.tf:
            self._tf_pub = self.create_publisher(TFMessage, "/tf", TF_QOS)
            self._tf_static_pub = self.create_publisher(
                TFMessage, "/tf_static", TF_STATIC_QOS)
            self._tf_static_pub.publish(self._static_mounts())

        # The map (ADR-0054), latched once. The seed for the free-space
        # fill is the first centreline sample, which is inside the drivable
        # band by construction.
        self._map_pub: Optional[object] = None
        if config.map:
            self._map_pub = self.create_publisher(
                OccupancyGrid, "/map", MAP_QOS)
            self._map_pub.publish(self._occupancy_grid(points[0]))

        self._clock_pub = self.create_publisher(Clock, "/clock", 10)
        dt = self.sim.dt
        self._clock_divisor = max(1, round(1.0 / (dt * config.clock_rate_hz)))
        self._gt_divisor = max(
            1, round(1.0 / (dt * config.ground_truth_rate_hz)))

        if config.lockstep:
            # Latched, so a client joining late receives the current step
            # immediately: start-up is a wait, never a deadlock (ADR-0051).
            self._announce_pub = self.create_publisher(
                UInt64, STEP_TOPIC, ANNOUNCE_QOS)

        # One executor, held: rclpy.spin_once(node) builds a fresh executor
        # per call, which is measurable at a kilohertz. In lockstep it runs
        # on its own thread, because advance() blocks at the barrier until
        # the callbacks it depends on have run (ADR-0051).
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------- plumbing

    def _ns(self, index: int) -> str:
        return f"/{self.config.namespace}{index}"

    def _frame_ns(self, index: int) -> str:
        """The agent's frame prefix: the topic namespace without the leading
        slash, which tf2 forbids in a frame id."""
        return f"{self.config.namespace}{index}"

    def _static_mounts(self) -> TFMessage:
        """base_link to every sensor mount, identity, latched once."""
        message = TFMessage()
        for index in range(len(self.cars)):
            base = f"{self._frame_ns(index)}/base_link"
            for frame in sorted(set(self._frames[index].values())):
                if frame == base:
                    continue
                transform = TransformStamped()
                transform.header.stamp = _sim_time(0.0)
                transform.header.frame_id = base
                transform.child_frame_id = frame
                transform.transform.rotation.w = 1.0
                message.transforms.append(transform)
        return message

    def _odom_edge(self, index: int, now: float) -> TransformStamped:
        """odom to base_link: the dead reckoner's belief, as a transform."""
        reckoned = self._reckoned[index]
        transform = TransformStamped()
        transform.header.stamp = _sim_time(now)
        transform.header.frame_id = f"{self._frame_ns(index)}/odom"
        transform.child_frame_id = f"{self._frame_ns(index)}/base_link"
        transform.transform.translation.x = reckoned["x"]
        transform.transform.translation.y = reckoned["y"]
        _yaw_to_quaternion(transform.transform.rotation, reckoned["yaw"])
        return transform

    def _map_correction(self, index: int, now: float) -> TransformStamped:
        """map to odom, chosen so the chain composes to the true pose.

        The correction a perfect localiser would publish (REP 105): with
        truth T and belief R as poses, the edge is T composed with the
        inverse of R, so map -> odom -> base_link lands exactly on T while
        odom -> base_link alone stays the honest, drifting belief.
        """
        state = self.sim.state(index)
        reckoned = self._reckoned[index]
        yaw = state.yaw - reckoned["yaw"]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        transform = TransformStamped()
        transform.header.stamp = _sim_time(now)
        transform.header.frame_id = "map"
        transform.child_frame_id = f"{self._frame_ns(index)}/odom"
        transform.transform.translation.x = state.pos.x - (
            cos_yaw * reckoned["x"] - sin_yaw * reckoned["y"])
        transform.transform.translation.y = state.pos.y - (
            sin_yaw * reckoned["x"] + cos_yaw * reckoned["y"])
        _yaw_to_quaternion(transform.transform.rotation, yaw)
        return transform

    def _translate(self, index: int, steer: float, speed: float,
                   vx: float) -> "slipx.DriveInput":
        """Ackermann (steer, speed) to the core's input: the speed loop."""
        command = slipx.DriveInput()
        command.steer_cmd = steer
        command.accel_cmd = self.config.speed_gain * (speed - vx)
        return command

    def _make_policy(self, index: int):
        def policy(state, _time, _rng):
            steer, speed = self._commands[index]
            return self._translate(index, steer, speed, state.vel_body.x)

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
            if self.config.lockstep:
                # The stamp is the tag (ADR-0051). Reading the state here,
                # from the spin thread, is safe by construction: integration
                # for the awaited step begins only after every mailbox holds
                # an entry for it, so the state is at rest at every post.
                step = stamp_to_step(msg.header.stamp, self.sim.dt)
                command = self._translate(
                    index, drive.steering_angle, drive.speed,
                    self.sim.state(index).vel_body.x)
                try:
                    self._mailboxes[index].post(step, command)
                except ValueError as error:
                    self.get_logger().warning(
                        f"{self._ns(index)}/drive tagged step {step} "
                        f"refused and dropped: {error}")

        return callback

    def _make_ack_callback(self, index: int):
        def callback(msg: UInt64) -> None:
            try:
                self._mailboxes[index].ack(int(msg.data))
            except ValueError as error:
                self.get_logger().warning(
                    f"{self._ns(index)}/drive_ack step {msg.data} refused "
                    f"and dropped: {error}")

        return callback

    # ------------------------------------------------------------- stepping

    def start_spinning(self) -> None:
        """Run the executor on its own thread (lockstep needs it: advance()
        blocks at the barrier until the callbacks it depends on have run)."""
        if self._spin_thread is None:
            self._spin_thread = threading.Thread(
                target=self._executor.spin, daemon=True)
            self._spin_thread.start()

    def wait_for_clients(self, per_agent: int = 1,
                         timeout_s: float = 10.0) -> bool:
        """Wait until every drive topic has publishers, for lockstep starts.

        Discovery takes a moment, and a lockstep bridge that announces into
        the void waits at its first barrier until the timeout policy rules;
        waiting for the match instead makes start-up deterministic.
        """
        import time as wall

        deadline = wall.monotonic() + timeout_s
        while wall.monotonic() < deadline:
            matched = all(
                self.count_publishers(f"{self._ns(i)}/drive") >= per_agent
                for i in range(len(self.cars))
            )
            if matched:
                return True
            wall.sleep(0.01)
        return False

    def step_once(self) -> None:
        """One step: callbacks, one advance, then publish what was due.

        Live mode spins the executor inline and the advance paces the wall
        clock; lockstep announces the step about to be computed and the
        advance blocks at the barrier until every agent answered.
        """
        if self.config.lockstep:
            self.start_spinning()
            self._announce_pub.publish(UInt64(data=self.sim.step_count))
        else:
            self._executor.spin_once(timeout_sec=0.0)
        self.sim.advance()
        self.rig.collect()
        self._publish_due()

    def run(self) -> None:
        while rclpy.ok():
            self.step_once()

    def shutdown(self) -> None:
        """Stop the spin thread, if one runs, before destroying the node."""
        if self._spin_thread is not None:
            self._executor.shutdown()
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None

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

        if self._tf_pub is not None and steps % self._gt_divisor == 0:
            message = TFMessage()
            for index in range(len(self.cars)):
                message.transforms.append(self._odom_edge(index, now))
                if self.config.ground_truth:
                    message.transforms.append(
                        self._map_correction(index, now))
            self._tf_pub.publish(message)

    # ------------------------------------------------------------- messages

    def _laser_scan(self, index: int, lidar, scan) -> LaserScan:
        spec = lidar.spec
        msg = LaserScan()
        msg.header.stamp = _sim_time(scan.stamp_time)
        msg.header.frame_id = self._frames[index].get(
            lidar.name, f"{self._frame_ns(index)}/{lidar.name}")
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
            imu.name, f"{self._frame_ns(index)}/{imu.name}")
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
        msg.header.frame_id = f"{self._frame_ns(index)}/odom"
        msg.child_frame_id = self._frames[index].get(
            encoder.name, f"{self._frame_ns(index)}/base_link")
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
        msg.child_frame_id = f"{self._frame_ns(index)}/base_link"
        msg.pose.pose.position.x = state.pos.x
        msg.pose.pose.position.y = state.pos.y
        _yaw_to_quaternion(msg.pose.pose.orientation, state.yaw)
        msg.twist.twist.linear.x = state.vel_body.x
        msg.twist.twist.linear.y = state.vel_body.y
        msg.twist.twist.angular.z = state.yaw_rate
        return msg

    def _occupancy_grid(self, seed_xy: Tuple[float, float]) -> OccupancyGrid:
        """The walls as a latched map, rasterised from the raycaster's own
        polylines (``TrackWorld.wall_left`` / ``wall_right``), never from a
        re-derived offset: a map that disagreed with the scans would be
        worse than no map. Occupied where a wall crosses a cell, free where
        the seed can reach without crossing one, unknown elsewhere, which
        is the shape a SLAM map of the same walls has.
        """
        resolution = self.config.map_resolution_m
        left = self.world.wall_left
        right = self.world.wall_right
        closed = self.track.closed

        xs = [p[0] for p in left] + [p[0] for p in right]
        ys = [p[1] for p in left] + [p[1] for p in right]
        # A margin of a few cells keeps the walls off the border, so the
        # outside exists in the grid and stays unknown.
        margin = 4.0 * resolution
        origin_x = min(xs) - margin
        origin_y = min(ys) - margin
        width = int(math.ceil((max(xs) - origin_x + margin) / resolution))
        height = int(math.ceil((max(ys) - origin_y + margin) / resolution))
        data = [-1] * (width * height)

        def cell(x: float, y: float) -> Tuple[int, int]:
            return (int((x - origin_x) / resolution),
                    int((y - origin_y) / resolution))

        def mark(ax: float, ay: float, bx: float, by: float) -> None:
            # Every cell the segment passes through (a supercover grid
            # walk), so the wall is a barrier the four-connected fill
            # below cannot slip through at a diagonal.
            col, row = cell(ax, ay)
            end_col, end_row = cell(bx, by)
            dx = bx - ax
            dy = by - ay
            step_col = 1 if dx > 0.0 else -1
            step_row = 1 if dy > 0.0 else -1
            edge_x = origin_x + (col + (step_col > 0)) * resolution
            edge_y = origin_y + (row + (step_row > 0)) * resolution
            t_col = (edge_x - ax) / dx if dx != 0.0 else math.inf
            t_row = (edge_y - ay) / dy if dy != 0.0 else math.inf
            dt_col = abs(resolution / dx) if dx != 0.0 else math.inf
            dt_row = abs(resolution / dy) if dy != 0.0 else math.inf
            for _ in range(abs(end_col - col) + abs(end_row - row) + 1):
                data[row * width + col] = 100
                if col == end_col and row == end_row:
                    break
                if t_col <= t_row:
                    col += step_col
                    t_col += dt_col
                else:
                    row += step_row
                    t_row += dt_row
            data[end_row * width + end_col] = 100

        for wall in (left, right):
            # A closed track's last point joins back to the first, exactly
            # as the raycaster's segment list treats it.
            spans = len(wall) if closed else len(wall) - 1
            for i in range(spans):
                j = (i + 1) % len(wall)
                mark(wall[i][0], wall[i][1], wall[j][0], wall[j][1])

        seed_col, seed_row = cell(seed_xy[0], seed_xy[1])
        if data[seed_row * width + seed_col] == 100:
            raise ValueError(
                f"map_resolution_m {resolution} puts a wall in the seed "
                f"cell on the centreline: the track is narrower than the "
                f"map can draw, so choose a finer resolution"
            )
        data[seed_row * width + seed_col] = 0
        frontier = deque([(seed_col, seed_row)])
        while frontier:
            col, row = frontier.popleft()
            for near_col, near_row in ((col + 1, row), (col - 1, row),
                                       (col, row + 1), (col, row - 1)):
                if 0 <= near_col < width and 0 <= near_row < height:
                    index = near_row * width + near_col
                    if data[index] == -1:
                        data[index] = 0
                        frontier.append((near_col, near_row))

        msg = OccupancyGrid()
        msg.header.stamp = _sim_time(0.0)
        msg.header.frame_id = "map"
        msg.info.map_load_time = _sim_time(0.0)
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = data
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
            "tf": self.config.tf,
            "map": self.config.map,
            "map_resolution_m": self.config.map_resolution_m,
            "real_time_factor": self.config.real_time_factor,
            "speed_gain": self.config.speed_gain,
            "seed": self.config.seed,
            "namespace": self.config.namespace,
            "lockstep": self.config.lockstep,
            "lockstep_policy": self.config.lockstep_policy,
            "lockstep_timeout_s": self.config.lockstep_timeout_s,
            "reproducibility": (
                "commands are functions of step indices, so the run "
                "reproduces when the clients do, and always from the "
                "input log (ADR-0051)"
                if self.config.lockstep else
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
                        help="do not offer ground truth topics or the map "
                             "TF edge; recorded in the bridge manifest")
    parser.add_argument("--no-tf", action="store_true",
                        help="do not broadcast TF at all, for a stack that "
                             "owns its tree; recorded in the bridge manifest")
    parser.add_argument("--no-map", action="store_true",
                        help="do not latch the walls on /map, for a stack "
                             "that runs its own map server; recorded in "
                             "the bridge manifest")
    parser.add_argument("--map-resolution", type=float, default=0.05,
                        help="occupancy grid cell size [m]")
    parser.add_argument("--speed-gain", type=float, default=4.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=Path, default=None,
                        help="directory for the run and bridge manifests")
    parser.add_argument("--lockstep", action="store_true",
                        help="announce each step and advance only when "
                             "every agent has answered (ADR-0051)")
    parser.add_argument("--lockstep-policy", default="wait",
                        choices=["wait", "coast", "freeze", "dnf"])
    parser.add_argument("--lockstep-timeout", type=float, default=0.0,
                        help="wall seconds before a miss is ruled [s]")
    arguments = parser.parse_args(argv)

    rclpy.init()
    bridge = Bridge(BridgeConfig(
        track_dir=arguments.track,
        car_dirs=list(arguments.cars),
        real_time_factor=arguments.real_time_factor,
        ground_truth=not arguments.no_ground_truth,
        tf=not arguments.no_tf,
        map=not arguments.no_map,
        map_resolution_m=arguments.map_resolution,
        speed_gain=arguments.speed_gain,
        seed=arguments.seed,
        out_dir=arguments.out,
        lockstep=arguments.lockstep,
        lockstep_policy=arguments.lockstep_policy,
        lockstep_timeout_s=arguments.lockstep_timeout,
    ))
    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.write_manifests()
        bridge.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
