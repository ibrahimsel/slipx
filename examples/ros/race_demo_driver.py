# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Drive N bridge agents from ROS 2, so a grid actually races.

One process, one node, N agents. Two modes, mirroring the C++ reference
stack's split (examples/cpp/reference_stack.hpp): each consumes a different
half of what the bridge publishes, so a working race validates that half
end to end.

- ``gap``: scan only. Follow-the-gap: steer at the farthest smoothed ray
  after carving a safety bubble around the nearest return. Opponents appear
  in the scan exactly as walls do, so avoidance falls out rather than being
  coded, and twenty of these on one track jostle like a race.
- ``pursuit``: ground truth against the centreline the bridge announces on
  ``/race/centreline``, the textbook geometric controller. It ignores the
  scan entirely, so it validates the geometry, the model and the announced
  direction, not the sensors; twenty of them parade on the same line, and
  a ``--reversed`` bridge turns the parade round with no change here.

These exist to exercise the simulator, not to win anything.

Run in a sourced ROS 2 environment with slipx on PYTHONPATH::

    python race_demo_driver.py --agents 20 --mode gap
"""

from __future__ import annotations

import argparse
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

MAX_STEER = 0.40      # the reference car's full lock              [rad]
WHEELBASE = 0.32      # lf + lr from dynamics.yaml                 [m]

#: The QoS the bridge latches the centreline with: joining late still
#: delivers the announcement.
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class GapFollower:
    """Scan in, drive out. Stateless between scans."""

    def __init__(self, top_speed: float) -> None:
        self.top_speed = top_speed

    def drive(self, msg: LaserScan) -> AckermannDriveStamped:
        # Decimate by 4: 270 beams is plenty to pick a gap, and pure
        # Python over the full 1080 at 40 Hz times 20 agents is not free.
        stride = 4
        increment = msg.angle_increment * stride
        angle_min = msg.angle_min
        limit = msg.range_max
        ranges = [
            limit if math.isnan(r) else min(max(r, 0.0), limit)
            for r in msg.ranges[::stride]
        ]

        # Only the forward 200 degrees can be driven at.
        n = len(ranges)
        lo = max(0, int((-1.75 - angle_min) / increment))
        hi = min(n, int((1.75 - angle_min) / increment) + 1)
        sector = ranges[lo:hi]

        # Safety bubble: blank the beams a car's half width subtends at the
        # nearest return, so the gap chooser cannot pick a path that clips
        # whatever is closest (a wall or an opponent).
        nearest = min(range(len(sector)), key=sector.__getitem__)
        half_width = math.atan2(0.25, max(sector[nearest], 0.1))
        bubble = int(half_width / increment) + 1
        for i in range(max(0, nearest - bubble),
                       min(len(sector), nearest + bubble + 1)):
            sector[i] = 0.0

        # Smooth, then steer at the farthest beam. A forward-bias cosine
        # keeps it from diving at a gap behind the axle line.
        window = 3
        best_index, best_score = 0, -1.0
        for i in range(len(sector)):
            a = max(0, i - window)
            b = min(len(sector), i + window + 1)
            depth = sum(sector[a:b]) / (b - a)
            angle = angle_min + (lo + i) * increment
            score = depth * (0.4 + 0.6 * math.cos(0.5 * angle))
            if score > best_score:
                best_index, best_score = i, score
        target = angle_min + (lo + best_index) * increment
        steer = max(-MAX_STEER, min(MAX_STEER, 0.8 * target))

        # Speed from forward clearance, so it brakes into traffic and
        # corners and stretches its legs on the straights.
        centre = int((0.0 - angle_min) / increment)
        ahead = min(ranges[max(0, centre - 12):centre + 13])
        speed = max(0.7, min(self.top_speed, 0.5 + 0.55 * ahead))

        command = AckermannDriveStamped()
        command.drive.steering_angle = steer
        command.drive.speed = speed
        return command


class PurePursuit:
    """Ground truth in, drive out, against the centreline."""

    def __init__(self, points: List[Tuple[float, float]],
                 lookahead: float, top_speed: float) -> None:
        self.points = points
        self.lookahead = lookahead
        self.top_speed = top_speed
        self.cumulative = [0.0]
        for a, b in zip(points, points[1:]):
            self.cumulative.append(
                self.cumulative[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = self.cumulative[-1] + math.hypot(
            points[0][0] - points[-1][0], points[0][1] - points[-1][1])

    def point_at(self, s: float) -> Tuple[float, float]:
        s = s % self.length
        for i in range(len(self.points) - 1):
            if self.cumulative[i + 1] >= s:
                a, b = self.points[i], self.points[i + 1]
                span = self.cumulative[i + 1] - self.cumulative[i]
                u = (s - self.cumulative[i]) / span if span > 0.0 else 0.0
                return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]))
        # Between the last sample and the first: the closing segment.
        a, b = self.points[-1], self.points[0]
        run = s - self.cumulative[-1]
        span = self.length - self.cumulative[-1]
        u = run / span if span > 0.0 else 0.0
        return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]))

    def drive(self, msg: Odometry) -> AckermannDriveStamped:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)

        nearest = min(
            range(len(self.points)),
            key=lambda i: (self.points[i][0] - x) ** 2
                          + (self.points[i][1] - y) ** 2)
        goal = self.point_at(self.cumulative[nearest] + self.lookahead)

        dx, dy = goal[0] - x, goal[1] - y
        alpha = math.atan2(
            -math.sin(yaw) * dx + math.cos(yaw) * dy,
            math.cos(yaw) * dx + math.sin(yaw) * dy)
        steer = math.atan2(2.0 * WHEELBASE * math.sin(alpha),
                           self.lookahead)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        command = AckermannDriveStamped()
        command.drive.steering_angle = steer
        command.drive.speed = self.top_speed * (
            1.0 - 0.5 * abs(steer) / MAX_STEER)
        return command


class Driver(Node):
    def __init__(self, arguments) -> None:
        super().__init__("race_demo_driver")
        self._arguments = arguments
        self._drive_pubs = []
        # Pursuit controllers exist only once the bridge's latched
        # announcement arrives: the centreline, in the direction raced, is
        # race control's to give, not this driver's to read off disk.
        self._pursuit: List[Optional[PurePursuit]] = [None] * arguments.agents
        if arguments.mode == "pursuit":
            self.create_subscription(
                Path, "/race/centreline", self._take_centreline, LATCHED_QOS)
        for index in range(arguments.agents):
            ns = f"/{arguments.namespace}{index}"
            publisher = self.create_publisher(
                AckermannDriveStamped, f"{ns}/drive", 10)
            self._drive_pubs.append(publisher)
            if arguments.mode == "gap":
                controller = GapFollower(arguments.speed)
                self.create_subscription(
                    LaserScan, f"{ns}/scan",
                    self._relay(index, controller),
                    qos_profile_sensor_data)
            else:
                self.create_subscription(
                    Odometry, f"{ns}/ground_truth/odom",
                    self._relay_pursuit(index), 10)

    def _take_centreline(self, msg: Path) -> None:
        points = [(pose.pose.position.x, pose.pose.position.y)
                  for pose in msg.poses]
        self._pursuit = [
            PurePursuit(points, self._arguments.lookahead,
                        self._arguments.speed)
            for _ in range(self._arguments.agents)
        ]

    def _relay(self, index: int, controller):
        def callback(msg) -> None:
            self._drive_pubs[index].publish(controller.drive(msg))

        return callback

    def _relay_pursuit(self, index: int):
        def callback(msg: Odometry) -> None:
            controller = self._pursuit[index]
            if controller is None:
                return   # no announcement yet: sit still rather than guess
            self._drive_pubs[index].publish(controller.drive(msg))

        return callback


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--agents", type=int, default=20)
    parser.add_argument("--mode", choices=["gap", "pursuit"], default="gap")
    parser.add_argument("--namespace", default="car_")
    parser.add_argument("--speed", type=float, default=3.0,
                        help="top speed [m/s]")
    parser.add_argument("--lookahead", type=float, default=1.2,
                        help="pursuit lookahead [m]")
    arguments = parser.parse_args()

    rclpy.init()
    node = Driver(arguments)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy's signal handler raises the second of these out of spin on
        # SIGTERM; both simply mean the race is over.
        pass
    finally:
        node.destroy_node()
        # try_shutdown, because rclpy's own signal handler has usually shut
        # the context first, and shutting twice is a traceback at every exit.
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
