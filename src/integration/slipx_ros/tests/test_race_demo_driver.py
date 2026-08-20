# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The demo driver's controllers, fed synthetic messages.

No bridge and no spinning executor: the controllers are plain objects that
map a message to a command, so they are tested as functions with scans and
odometry built by hand. What needs a live bridge, the topics, the QoS and
the latched announcement, is test_bridge.py's business.

These tests run only where ROS 2 exists; anywhere else they skip cleanly.
"""

from __future__ import annotations

import math
import sys
from argparse import Namespace
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="the driver needs a ROS 2 environment")

from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from nav_msgs.msg import Path as PathMsg  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "examples" / "ros"))

import race_demo_driver as demo  # noqa: E402


# ------------------------------------------------------------ message forges


def scan_of(fill: float = 10.0, beams: int = 1080) -> LaserScan:
    """The reference car's LiDAR shape: 270 degrees, 1080 beams.

    The increment divides the span by the ray count, not by count minus
    one, because that is how the sensor model spaces its beams; synthetic
    scans should carry bearings the bridge could actually publish.
    """
    msg = LaserScan()
    msg.angle_min = -2.35619449019234
    msg.angle_max = 2.35619449019234
    msg.angle_increment = (msg.angle_max - msg.angle_min) / beams
    msg.range_min = 0.02
    msg.range_max = 30.0
    msg.ranges = [fill] * beams
    return msg


def block(msg: LaserScan, lo: float, hi: float, distance: float) -> LaserScan:
    """Plant a return at ``distance`` across the bearing band [lo, hi]."""
    for i in range(len(msg.ranges)):
        angle = msg.angle_min + i * msg.angle_increment
        if lo <= angle <= hi:
            msg.ranges[i] = distance
    return msg


def odom_of(x: float, y: float, yaw: float, vx: float = 0.0) -> Odometry:
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    msg.twist.twist.linear.x = vx
    return msg


def straight_line(length: int = 60) -> list:
    """A centreline running along +x at 1 m spacing."""
    return [(float(i), 0.0) for i in range(length)]


# ------------------------------------------------------------- gap follower


def test_gap_runs_flat_out_on_an_open_scan() -> None:
    command = demo.GapFollower(3.0).drive(scan_of())

    assert abs(command.drive.steering_angle) < 0.1
    assert command.drive.speed == pytest.approx(3.0)


def test_gap_steers_away_from_a_wall_on_the_right() -> None:
    # ISO 8855: positive steer is to the left. A wall filling the right
    # side and the middle of the scan leaves the open gap on the left, so
    # the answer must be a left steer, and the mirrored wall the mirrored
    # answer. The bands reach past straight ahead deliberately: the gap
    # follower steers at the nearest edge of the widest gap, so a wall
    # that stops short of centre is answered with a steer of nearly zero.
    right_wall = block(scan_of(), -1.6, 0.3, 0.7)
    left_wall = block(scan_of(), -0.3, 1.6, 0.7)

    steer_from_right = demo.GapFollower(3.0).drive(right_wall)
    steer_from_left = demo.GapFollower(3.0).drive(left_wall)

    assert steer_from_right.drive.steering_angle > 0.1
    assert steer_from_left.drive.steering_angle < -0.1


def test_gap_brakes_into_traffic() -> None:
    blocked = block(scan_of(), -0.3, 0.3, 1.0)

    command = demo.GapFollower(3.0).drive(blocked)

    assert command.drive.speed < 1.5


def test_gap_treats_a_dropped_ray_as_absent_not_as_a_wall() -> None:
    # The bridge delivers dropouts as NaN, never zero (a controller that
    # read NaN as 0 m would emergency-brake on every dropout). Absent
    # means "nothing seen out to the sensor's limit", so a NaN beam must
    # drive exactly like the same beam at range_max. Beam 500 survives
    # the stride-4 decimation, so the comparison is not vacuous.
    dropped = scan_of()
    dropped.ranges[500] = math.nan
    at_limit = scan_of()
    at_limit.ranges[500] = at_limit.range_max

    a = demo.GapFollower(3.0).drive(dropped)
    b = demo.GapFollower(3.0).drive(at_limit)

    assert a.drive.speed == b.drive.speed
    assert a.drive.steering_angle == b.drive.steering_angle


# ------------------------------------------------------------- pure pursuit


def test_pursuit_steers_toward_the_line() -> None:
    pursuit = demo.PurePursuit(straight_line(), 1.2, 3.0)

    assert pursuit.steer_at(5.0, -0.3, 0.0) > 0.0
    assert pursuit.steer_at(5.0, 0.3, 0.0) < 0.0
    assert pursuit.steer_at(5.0, 0.0, 0.0) == pytest.approx(0.0, abs=1e-6)


def test_pursuit_slows_for_its_own_steering() -> None:
    pursuit = demo.PurePursuit(straight_line(), 1.2, 3.0)

    on_line = pursuit.drive(odom_of(5.0, 0.0, 0.0))
    offset = pursuit.drive(odom_of(5.0, 0.5, 0.0))

    assert on_line.drive.speed == pytest.approx(3.0)
    assert offset.drive.speed < on_line.drive.speed


def test_point_at_wraps_the_closing_segment() -> None:
    # A square lap whose closing chord is implied, exactly like a closed
    # track's centreline: arc length past the last sample must interpolate
    # back toward the first, not fall off the table.
    square = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
    pursuit = demo.PurePursuit(square, 1.0, 3.0)

    assert pursuit.length == pytest.approx(16.0)
    assert pursuit.point_at(14.0) == pytest.approx((0.0, 2.0))
    assert pursuit.point_at(16.0) == pytest.approx((0.0, 0.0))
    assert pursuit.point_at(17.0) == pytest.approx((1.0, 0.0))


def test_pursuit_never_commands_past_full_lock() -> None:
    # From far off the line the geometry asks for more steering than the
    # car has. The clamp lives in the controller, not only in the sim's
    # limits, because a command the car cannot follow misleads anything
    # layered on top (the racer adds its lean to this number).
    pursuit = demo.PurePursuit(straight_line(), 1.2, 3.0)

    steer = pursuit.steer_at(5.0, -5.0, 0.0)

    assert steer == pytest.approx(demo.MAX_STEER)


def test_windowed_nearest_matches_brute_force() -> None:
    # The cached local search is an optimisation, and an optimisation that
    # changes the answer is a bug. Walk a pose around the lap with lateral
    # noise and demand the windowed answer is never worse than brute force.
    points = [(10.0 * math.cos(2 * math.pi * i / 200),
               10.0 * math.sin(2 * math.pi * i / 200)) for i in range(200)]
    pursuit = demo.PurePursuit(points, 1.2, 3.0)
    import random
    rng = random.Random(11)

    for step in range(150):
        s = (step * 0.9) % pursuit.length
        gx, gy = pursuit.point_at(s)
        x, y = gx + rng.uniform(-0.4, 0.4), gy + rng.uniform(-0.4, 0.4)
        got = pursuit.nearest_index(x, y)
        best = min(range(len(points)),
                   key=lambda i: (points[i][0] - x) ** 2
                                 + (points[i][1] - y) ** 2)
        got_d = math.hypot(points[got][0] - x, points[got][1] - y)
        best_d = math.hypot(points[best][0] - x, points[best][1] - y)
        assert got_d <= best_d + 1e-9


def test_a_shunted_car_recovers_the_line() -> None:
    # The local window is only valid while the car stays near its last
    # answer. A shunt that carries it across the infield must fall back to
    # the global search rather than tracking the wrong side of the lap.
    points = [(10.0 * math.cos(2 * math.pi * i / 200),
               10.0 * math.sin(2 * math.pi * i / 200)) for i in range(200)]
    pursuit = demo.PurePursuit(points, 1.2, 3.0)
    pursuit.nearest_index(10.0, 0.0)

    across = pursuit.nearest_index(-10.0, 0.1)

    expected = min(range(len(points)),
                   key=lambda i: (points[i][0] + 10.0) ** 2
                                 + (points[i][1] - 0.1) ** 2)
    assert across == expected


# -------------------------------------------------------------------- racer


def racer_on_a_straight(aggression: float = 0.6, top: float = 4.0,
                        vx: float = 3.0) -> demo.Racer:
    pursuit = demo.PurePursuit(straight_line(), 1.2, top)
    racer = demo.Racer(pursuit, aggression)
    racer.take_odom(odom_of(5.0, 0.0, 0.0, vx=vx))
    return racer


def test_a_racer_without_ground_truth_sits_still() -> None:
    racer = demo.Racer(demo.PurePursuit(straight_line(), 1.2, 4.0), 0.5)

    assert racer.drive(scan_of()) is None


def test_a_clear_racer_holds_the_pursuit_line() -> None:
    racer = racer_on_a_straight()

    command = racer.drive(scan_of())

    assert abs(command.drive.steering_angle) < 0.02
    assert command.drive.speed == pytest.approx(4.0, abs=0.1)


def test_a_blocked_racer_leans_out_of_the_line_and_keeps_going() -> None:
    # A car dead ahead inside the braking zone, with the left side walled
    # off: the racer must lean right, and must not stop.
    racer = racer_on_a_straight()
    scan = block(scan_of(), -0.2, 0.2, 1.6)
    block(scan, 0.1, 1.6, 1.0)

    command = racer.drive(scan)

    assert command.drive.steering_angle < -0.05
    assert command.drive.speed > 0.5


def test_the_block_test_follows_the_intended_line_not_the_nose() -> None:
    # In a corner the pursuit steer points well away from dead ahead. A
    # return planted straight ahead but outside the steered cone must not
    # slow the car: that return is the outside wall it is already turning
    # away from.
    pursuit = demo.PurePursuit(straight_line(), 1.2, 4.0)
    racer = demo.Racer(pursuit, 0.6)
    racer.take_odom(odom_of(5.0, -0.8, 0.0, vx=3.0))   # well right of the line
    steer = pursuit.steer_at(5.0, -0.8, 0.0)
    assert steer > 0.25

    clear = racer.drive(scan_of())
    racer2 = demo.Racer(demo.PurePursuit(straight_line(), 1.2, 4.0), 0.6)
    racer2.take_odom(odom_of(5.0, -0.8, 0.0, vx=3.0))
    ahead_only = racer2.drive(block(scan_of(), -0.05, 0.05, 1.2))

    assert ahead_only.drive.speed == pytest.approx(clear.drive.speed)


def test_aggression_buys_later_braking() -> None:
    hot = racer_on_a_straight(aggression=0.95)
    cold = racer_on_a_straight(aggression=0.25)
    scan = block(scan_of(), -0.2, 0.2, 2.0)

    fast = hot.drive(scan)
    slow = cold.drive(block(scan_of(), -0.2, 0.2, 2.0))

    assert fast.drive.speed > slow.drive.speed


def test_something_at_the_nose_stops_the_car_whatever_the_card_says() -> None:
    racer = racer_on_a_straight(aggression=0.95)

    command = racer.drive(block(scan_of(), -0.1, 0.1, 0.2))

    assert command.drive.speed == 0.0


def test_a_stuck_racer_backs_out_then_resumes() -> None:
    racer = racer_on_a_straight(vx=0.0)
    jammed = block(scan_of(), -0.3, 0.3, 0.4)

    commands = [racer.drive(block(scan_of(), -0.3, 0.3, 0.4))
                for _ in range(61)]
    assert all(c.drive.speed >= 0.0 for c in commands)

    reverse = racer.drive(jammed)
    assert reverse.drive.speed < 0.0
    for _ in range(35):
        assert racer.drive(jammed).drive.speed < 0.0

    forward_again = racer.drive(jammed)
    assert forward_again.drive.speed >= 0.0


def test_aggression_narrows_the_brake_zone_itself() -> None:
    # Not just the speed cap: with a car 1.3 m ahead at 3 m/s, the keen
    # card's braking zone has not been reached and it holds its line,
    # while the cautious card is already inside its zone and leaning.
    # This pins the aggression term in the zone, which the speed
    # comparison above cannot see.
    hot = racer_on_a_straight(aggression=0.95)
    cold = racer_on_a_straight(aggression=0.25)

    committed = hot.drive(block(scan_of(), -0.2, 0.2, 1.3))
    cautious = cold.drive(block(scan_of(), -0.2, 0.2, 1.3))

    assert abs(committed.drive.steering_angle) < 0.02
    assert abs(cautious.drive.steering_angle) > 0.04


def test_creeping_through_traffic_is_not_stuck() -> None:
    # The slow counter must reset the moment the car moves or the road
    # opens, or a whole race of stop-and-go queueing eventually trips a
    # spurious reverse in clean traffic.
    racer = racer_on_a_straight(vx=0.0)
    jammed = block(scan_of(), -0.3, 0.3, 0.4)

    for _ in range(3):
        for _ in range(40):                    # a spell of being boxed in
            assert racer.drive(jammed).drive.speed >= 0.0
        racer.take_odom(odom_of(5.0, 0.0, 0.0, vx=2.0))
        assert racer.drive(scan_of()).drive.speed > 0.0   # it moves again
        racer.take_odom(odom_of(5.0, 0.0, 0.0, vx=0.0))

    # 3 x 40 boxed-in ticks with recoveries between must never reverse.


# --------------------------------------------------------------------- deal


def test_the_deal_is_the_seed() -> None:
    assert demo.deal_grid(20, "mixed", 0, 4.0, 1.2) == \
        demo.deal_grid(20, "mixed", 0, 4.0, 1.2)
    assert demo.deal_grid(20, "mixed", 0, 4.0, 1.2) != \
        demo.deal_grid(20, "mixed", 1, 4.0, 1.2)


def test_mixed_deals_racers_and_traffic_but_never_a_blind_card() -> None:
    cards = demo.deal_grid(40, "mixed", 0, 4.0, 1.2)
    kinds = {card["kind"] for card in cards}

    assert kinds == {"racer", "gap"}
    racers = [card for card in cards if card["kind"] == "racer"]
    assert len(racers) > len(cards) / 2


def test_racer_mode_deals_only_racers_with_spread() -> None:
    cards = demo.deal_grid(20, "racer", 3, 4.0, 1.2)

    assert all(card["kind"] == "racer" for card in cards)
    tops = [card["top_speed"] for card in cards]
    assert min(tops) >= 4.0 * 0.85 and max(tops) <= 4.0 * 1.2
    assert max(tops) - min(tops) > 0.2, "the field must not be identical"
    assert all(0.25 <= card["aggression"] <= 0.95 for card in cards)


# ---------------------------------------------------------------- the node


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_the_announcement_builds_exactly_the_racer_cards(ros) -> None:
    arguments = Namespace(agents=6, mode="mixed", namespace="car_",
                          speed=4.0, lookahead=1.2, seed=0)
    node = demo.Driver(arguments)
    try:
        announcement = PathMsg()
        for x, y in [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            announcement.poses.append(pose)
        node._take_centreline(announcement)

        for card, racer in zip(node._cards, node._racers):
            assert (racer is None) == (card["kind"] != "racer")
            if racer is not None:
                assert racer.pursuit.top_speed == card["top_speed"]
                assert racer.pursuit.lookahead == card["lookahead"]
                assert racer.aggression == card["aggression"]
    finally:
        node.destroy_node()
