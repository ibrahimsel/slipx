# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Drive N bridge agents from ROS 2, so a grid actually races.

One process, one node, N agents, each with its own controller instance and
its own parameters. Four modes:

- ``gap``: scan only. Follow-the-gap: steer at the farthest smoothed ray
  after carving a safety bubble around the nearest return. Opponents appear
  in the scan exactly as walls do, so avoidance falls out rather than being
  coded, and twenty of these on one track jostle like a race.
- ``pursuit``: ground truth against the centreline the bridge announces on
  ``/race/centreline``, the textbook geometric controller. It ignores the
  scan entirely, so it validates the geometry, the model and the announced
  direction, not the sensors; twenty of them parade on the same line, and
  a ``--reversed`` bridge turns the parade round with no change here.
- ``racer``: both halves at once. Pure pursuit sets the line and the pace,
  the scan overlays traffic, and a blocked racer leans out of the line and
  keeps its foot in rather than lifting. Every car draws its own top speed,
  lookahead and aggression from the seeded deal.
- ``mixed``: the show. A seeded deal hands most cars a racer card and the
  rest a gap card, with the parameters spread, because a field of identical
  cars single-files within a lap and identical laps are a parade. The deal
  is printed at start-up, one line per car, so the field is knowable. The
  blind pursuit controller is deliberately not dealt into traffic: a car
  that cannot see is not brave, it is a hazard.

The first two modes mirror the C++ reference stack's split
(examples/cpp/reference_stack.hpp): each consumes a different half of what
the bridge publishes, so a working race validates that half end to end.

These exist to exercise the simulator, not to win anything.

Run in a sourced ROS 2 environment with slipx on PYTHONPATH::

    python race_demo_driver.py --agents 20 --mode mixed --seed 0
"""

from __future__ import annotations

import argparse
import bisect
import math
import random
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


def _yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


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
        self._near: Optional[int] = None

    def point_at(self, s: float) -> Tuple[float, float]:
        s = s % self.length
        index = bisect.bisect_right(self.cumulative, s) - 1
        if index >= len(self.points) - 1:
            # Between the last sample and the first: the closing segment.
            a, b = self.points[-1], self.points[0]
            span = self.length - self.cumulative[-1]
            run = s - self.cumulative[-1]
        else:
            a, b = self.points[index], self.points[index + 1]
            span = self.cumulative[index + 1] - self.cumulative[index]
            run = s - self.cumulative[index]
        u = run / span if span > 0.0 else 0.0
        return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]))

    def nearest_index(self, x: float, y: float) -> int:
        """The nearest centreline sample, searched near the last answer.

        A global argmin every tick is O(n) per message, which at twenty
        cars and an announced centreline of two thousand samples is real
        money in pure Python. A car cannot teleport between messages, so
        the answer lives near the previous one; the global search runs on
        the first call, and again whenever the local answer is implausibly
        far away, which is what a car carried across the track by a shunt
        looks like.
        """
        points = self.points
        n = len(points)

        def d2(i: int) -> float:
            return (points[i][0] - x) ** 2 + (points[i][1] - y) ** 2

        if self._near is None:
            self._near = min(range(n), key=d2)
            return self._near

        window = 60
        best, best_d = self._near, d2(self._near)
        for k in range(-window, window + 1):
            i = (self._near + k) % n
            distance = d2(i)
            if distance < best_d:
                best, best_d = i, distance
        if best_d > 9.0:
            best = min(range(n), key=d2)
        self._near = best
        return best

    def steer_at(self, x: float, y: float, yaw: float,
                 lookahead: Optional[float] = None) -> float:
        reach = self.lookahead if lookahead is None else lookahead
        nearest = self.nearest_index(x, y)
        goal = self.point_at(self.cumulative[nearest] + reach)

        dx, dy = goal[0] - x, goal[1] - y
        alpha = math.atan2(
            -math.sin(yaw) * dx + math.cos(yaw) * dy,
            math.cos(yaw) * dx + math.sin(yaw) * dy)
        steer = math.atan2(2.0 * WHEELBASE * math.sin(alpha), reach)
        return max(-MAX_STEER, min(MAX_STEER, steer))

    def drive(self, msg: Odometry) -> AckermannDriveStamped:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        steer = self.steer_at(x, y, _yaw_of(msg))

        command = AckermannDriveStamped()
        command.drive.steering_angle = steer
        command.drive.speed = self.top_speed * (
            1.0 - 0.5 * abs(steer) / MAX_STEER)
        return command


class Racer:
    """Ground truth for pace, the scan for traffic: pursuit that overtakes.

    Pure pursuit against the announced centreline sets the line and the
    speed. The scan overlays what the line does not know about: when the
    cone the car is steering into is blocked inside its braking distance,
    the racer leans toward the clearer side and keeps its foot in, where a
    defensive controller would lift and queue. How late it brakes and how
    far it leans are the aggression parameter, which is what the seeded
    deal varies car to car.

    Holds state deliberately, unlike the GapFollower: the chosen side is
    kept while the block lasts, because re-choosing every scan oscillates
    in traffic, and a stuck detector backs out of a pile-up instead of
    pushing into it forever. That is controller state, not simulation
    state; the sim underneath stays exactly as deterministic as it was.
    """

    def __init__(self, pursuit: PurePursuit, aggression: float) -> None:
        self.pursuit = pursuit
        self.aggression = aggression
        self.pose: Optional[Tuple[float, float, float, float]] = None
        self._side = 0          # +1 leaning left, -1 leaning right, 0 clear
        self._slow_ticks = 0
        self._reversing = 0

    def take_odom(self, msg: Odometry) -> None:
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     _yaw_of(msg), msg.twist.twist.linear.x)

    def drive(self, msg: LaserScan) -> Optional[AckermannDriveStamped]:
        if self.pose is None:
            return None   # no ground truth yet: sit still rather than guess
        x, y, yaw, speed = self.pose

        command = AckermannDriveStamped()
        if self._reversing > 0:
            # Backing out of a pile-up. Straight back: steering while
            # reversing swings the nose into whatever the car is already
            # touching.
            self._reversing -= 1
            command.drive.speed = -0.8
            command.drive.steering_angle = 0.0
            return command

        stride = 4
        increment = msg.angle_increment * stride
        angle_min = msg.angle_min
        limit = msg.range_max
        ranges = [
            limit if math.isnan(r) else min(max(r, 0.0), limit)
            for r in msg.ranges[::stride]
        ]

        def clearance(lo: float, hi: float) -> float:
            a = max(0, int((lo - angle_min) / increment))
            b = min(len(ranges), int((hi - angle_min) / increment) + 1)
            return min(ranges[a:b]) if b > a else 0.0

        # The lookahead grows with speed, or a fast car steers at a point
        # under its own nose and weaves.
        reach = max(self.pursuit.lookahead, 0.32 * speed)
        steer = self.pursuit.steer_at(x, y, yaw, lookahead=reach)

        # The block test looks where the car intends to go, not where the
        # nose points: in a corner those differ by the whole corner.
        ahead = clearance(steer - 0.18, steer + 0.18)

        # Braking distance under the bridge's P speed loop plus a margin
        # that shrinks with aggression: the aggressive card brakes later.
        brake_zone = speed * speed / 16.0 + 0.5 + (1.0 - self.aggression) * 0.7

        if ahead < brake_zone:
            left = clearance(steer + 0.15, steer + 0.65)
            right = clearance(steer - 0.65, steer - 0.15)
            # Hysteresis: pick a side once per block, switch only if the
            # other side becomes much clearer than the chosen one.
            if self._side == 0 or abs(left - right) > 1.5:
                self._side = 1 if left >= right else -1
            lean = (0.22 + 0.28 * self.aggression) * (1.0 - ahead / brake_zone)
            steer = steer + self._side * lean
        else:
            self._side = 0
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        # Speed: the pursuit slowdown in corners, capped by how much room
        # there actually is, with aggression deciding how much of the gap
        # to spend.
        top = self.pursuit.top_speed
        target = top * (1.0 - 0.45 * abs(steer) / MAX_STEER)
        target = min(target, 0.5 + (0.6 + 0.5 * self.aggression) * ahead)

        # The one non-negotiable: something dead ahead inside a nose
        # length stops the car, whatever the card says.
        panic = clearance(-0.45, 0.45)
        if panic < 0.30:
            target = 0.0

        # Stuck means slow with something close in front for a sustained
        # spell; a queue creeping forward does not trip it.
        if speed < 0.15 and panic < 0.6:
            self._slow_ticks += 1
        else:
            self._slow_ticks = 0
        if self._slow_ticks > 60:      # 1.5 s at the 40 Hz scan rate
            self._slow_ticks = 0
            self._reversing = 36       # 0.9 s of backing out

        command.drive.steering_angle = steer
        command.drive.speed = max(target, 0.0)
        return command


def deal_grid(agents: int, mode: str, seed: int, base_speed: float,
              base_lookahead: float) -> List[dict]:
    """One card per car: which controller, with which numbers.

    Seeded, so the same seed is the same field, and spread, because a grid
    of identical cars single-files within a lap and identical laps are a
    parade. ``mixed`` deals mostly racers with some gap followers as
    rolling traffic; ``racer`` deals racers only. The blind pursuit
    controller is never dealt into traffic.
    """
    rng = random.Random(seed)
    cards: List[dict] = []
    for _ in range(agents):
        kind = "racer"
        if mode == "mixed" and rng.random() >= 0.7:
            kind = "gap"
        if kind == "racer":
            cards.append({
                "kind": "racer",
                "top_speed": base_speed * rng.uniform(0.85, 1.2),
                "lookahead": base_lookahead * rng.uniform(0.85, 1.35),
                "aggression": rng.uniform(0.25, 0.95),
            })
        else:
            cards.append({
                "kind": "gap",
                "top_speed": base_speed * rng.uniform(0.75, 1.0),
            })
    return cards


class Driver(Node):
    def __init__(self, arguments) -> None:
        super().__init__("race_demo_driver")
        self._arguments = arguments
        self._drive_pubs = []
        # Controllers that need the centreline exist only once the bridge's
        # latched announcement arrives: the centreline, in the direction
        # raced, is race control's to give, not this driver's to read off
        # disk.
        self._pursuit: List[Optional[PurePursuit]] = [None] * arguments.agents
        self._racers: List[Optional[Racer]] = [None] * arguments.agents
        self._cards: Optional[List[dict]] = None

        if arguments.mode in ("racer", "mixed"):
            self._cards = deal_grid(arguments.agents, arguments.mode,
                                    arguments.seed, arguments.speed,
                                    arguments.lookahead)
            for index, card in enumerate(self._cards):
                if card["kind"] == "racer":
                    self.get_logger().info(
                        f"{arguments.namespace}{index}: racer, top "
                        f"{card['top_speed']:.2f} m/s, lookahead "
                        f"{card['lookahead']:.2f} m, aggression "
                        f"{card['aggression']:.2f}")
                else:
                    self.get_logger().info(
                        f"{arguments.namespace}{index}: gap follower, top "
                        f"{card['top_speed']:.2f} m/s")

        needs_centreline = arguments.mode != "gap" and (
            self._cards is None
            or any(card["kind"] == "racer" for card in self._cards))
        if needs_centreline:
            self.create_subscription(
                Path, "/race/centreline", self._take_centreline, LATCHED_QOS)

        for index in range(arguments.agents):
            ns = f"/{arguments.namespace}{index}"
            publisher = self.create_publisher(
                AckermannDriveStamped, f"{ns}/drive", 10)
            self._drive_pubs.append(publisher)

            card = self._cards[index] if self._cards else None
            if arguments.mode == "gap" or (card and card["kind"] == "gap"):
                top = card["top_speed"] if card else arguments.speed
                controller = GapFollower(top)
                self.create_subscription(
                    LaserScan, f"{ns}/scan",
                    self._relay(index, controller),
                    qos_profile_sensor_data)
            elif arguments.mode == "pursuit":
                self.create_subscription(
                    Odometry, f"{ns}/ground_truth/odom",
                    self._relay_pursuit(index), 10)
            else:
                self.create_subscription(
                    LaserScan, f"{ns}/scan",
                    self._relay_racer(index),
                    qos_profile_sensor_data)
                self.create_subscription(
                    Odometry, f"{ns}/ground_truth/odom",
                    self._feed_racer(index), 10)

    def _take_centreline(self, msg: Path) -> None:
        points = [(pose.pose.position.x, pose.pose.position.y)
                  for pose in msg.poses]
        if self._arguments.mode == "pursuit":
            self._pursuit = [
                PurePursuit(points, self._arguments.lookahead,
                            self._arguments.speed)
                for _ in range(self._arguments.agents)
            ]
            return
        for index, card in enumerate(self._cards or []):
            if card["kind"] == "racer":
                pursuit = PurePursuit(points, card["lookahead"],
                                      card["top_speed"])
                self._racers[index] = Racer(pursuit, card["aggression"])

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

    def _relay_racer(self, index: int):
        def callback(msg: LaserScan) -> None:
            racer = self._racers[index]
            if racer is None:
                return   # no announcement yet: sit still rather than guess
            command = racer.drive(msg)
            if command is not None:
                self._drive_pubs[index].publish(command)

        return callback

    def _feed_racer(self, index: int):
        def callback(msg: Odometry) -> None:
            racer = self._racers[index]
            if racer is not None:
                racer.take_odom(msg)

        return callback


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--agents", type=int, default=20)
    parser.add_argument("--mode",
                        choices=["gap", "pursuit", "racer", "mixed"],
                        default="gap")
    parser.add_argument("--namespace", default="car_")
    parser.add_argument("--speed", type=float, default=3.0,
                        help="top speed, the centre of the dealt spread "
                             "[m/s]")
    parser.add_argument("--lookahead", type=float, default=1.2,
                        help="pursuit lookahead, the centre of the dealt "
                             "spread [m]")
    parser.add_argument("--seed", type=int, default=0,
                        help="seed for the dealt grid; the same seed is "
                             "the same field")
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
