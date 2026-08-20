# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The bridge, spoken to over real topics (ADR-0050).

These tests run only where ROS 2 exists (the WSL distribution, a sourced
Jazzy environment); anywhere else they skip cleanly, which is what keeps the
Windows pytest run green. The probe node is a second rclpy node in the same
process, so what is asserted crossed the RMW like any stack's messages
would.

A live bridge on the default domain leaks its topics and TF into these
tests (the map edge fails the declining test from outside); run them with
a spare ROS_DOMAIN_ID while one is up.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="the bridge needs a ROS 2 environment")

from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402

from slipx_ros import Bridge, BridgeConfig  # noqa: E402

REPO = Path(__file__).resolve().parents[4]
TRACK = REPO / "examples" / "tracks" / "paddock_stadium"
CAR = REPO / "examples" / "cars" / "reference_1_10"


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def probe(ros):
    node = rclpy.create_node("probe")
    yield node
    node.destroy_node()


@pytest.fixture
def bridge(ros, tmp_path):
    # A large real-time factor: validation mode paces the wall clock, and
    # the tests should not spend their budget sleeping to prove it.
    node = Bridge(BridgeConfig(
        track_dir=TRACK,
        car_dirs=[CAR, CAR],
        real_time_factor=1000.0,
        seed=3,
        out_dir=tmp_path,
    ))
    yield node
    node.destroy_node()


def pump(bridge_node, probe_node, steps):
    for _ in range(steps):
        bridge_node.step_once()
        rclpy.spin_once(probe_node, timeout_sec=0.0)


def test_the_clock_is_sim_time(bridge, probe):
    clocks = []
    probe.create_subscription(Clock, "/clock", clocks.append, 10)
    pump(bridge, probe, 300)
    assert clocks, "the clock must tick"
    last = clocks[-1].clock
    stamped = last.sec + last.nanosec * 1.0e-9
    # The clock is steps * dt, not a wall clock: it can only lag the
    # simulation by less than one clock period.
    assert stamped == pytest.approx(bridge.sim.time, abs=0.011)
    assert stamped > 0.2


def test_a_drive_command_moves_the_car_and_only_that_car(bridge, probe):
    # The FRONT car is commanded, so it drives away up the straight; the
    # silent car behind must stay put. Commanding the rear one would ram
    # the parked leader, which is a fine race and a bad unit test.
    publisher = probe.create_publisher(
        AckermannDriveStamped, "/car_1/drive", 10)
    command = AckermannDriveStamped()
    command.drive.speed = 2.0
    # Publish repeatedly: the command holds like a servo once received, but
    # discovery between two nodes takes a moment and nothing before it
    # counts as received.
    for _ in range(20):
        publisher.publish(command)
        pump(bridge, probe, 50)

    assert bridge.sim.state(1).vel_body.x > 1.0, "the commanded car drives"
    assert abs(bridge.sim.state(0).vel_body.x) < 0.05, "the silent car coasts"


def test_ground_truth_reports_the_true_state_per_agent(bridge, probe):
    car_0, car_1 = [], []
    probe.create_subscription(
        Odometry, "/car_0/ground_truth/odom", car_0.append, 10)
    probe.create_subscription(
        Odometry, "/car_1/ground_truth/odom", car_1.append, 10)
    pump(bridge, probe, 300)
    assert car_0 and car_1, "ground truth must flow when enabled"
    assert car_0[-1].pose.pose.position.x == pytest.approx(
        bridge.sim.state(0).pos.x, abs=0.05)
    # Each agent's own truth, not one car's truth broadcast twice: the two
    # sit a grid spacing apart.
    assert car_1[-1].pose.pose.position.x == pytest.approx(
        bridge.sim.state(1).pos.x, abs=0.05)
    assert abs(car_1[-1].pose.pose.position.x -
               car_0[-1].pose.pose.position.x) > 1.0
    assert car_0[-1].header.frame_id == "map"


def test_the_scan_crosses_the_wire_with_nan_never_zero(bridge, probe):
    scans = []
    probe.create_subscription(
        LaserScan, "/car_0/scan", scans.append, qos_profile_sensor_data)
    # The 40 Hz scanner needs a revolution simulated before the first scan
    # exists, then 12 ms of latency before it is visible.
    pump(bridge, probe, 400)
    assert scans, "a scan must arrive"
    scan = scans[-1]
    assert len(scan.ranges) == 1080
    assert scan.header.frame_id == "car_0/laser_frame", \
        "the frame id is the sensor's mount, namespaced without the " \
        "leading slash tf2 forbids"
    finite = [r for r in scan.ranges if math.isfinite(r)]
    assert len(finite) > 500, "a corridor surrounds the car"
    # Over EVERY scan received, not just the last: a dropout is a rare draw
    # and a zero smuggled in for one would hide from a single-scan check.
    for received in scans:
        assert all(
            r >= received.range_min
            for r in received.ranges if math.isfinite(r)
        ), "an invalid ray is NaN on the wire, never a zero"
    # The stamp is the sensor's own time (revolution end plus drawn
    # latency), which does not sit on the step grid the way a stamp
    # laundered from the publishing step would.
    off_grid = 0
    for received in scans:
        nanoseconds = received.header.stamp.nanosec
        if nanoseconds % 1_000_000 != 0:
            off_grid += 1
    assert off_grid > 0, "scan stamps are sensor times, not step times"
    # The opponent on the grid ahead appears nearer than any wall could be:
    # the forward rays see a car.
    forward = scan.ranges[len(scan.ranges) // 2]
    assert math.isfinite(forward)
    assert forward < 1.5


def test_the_walls_are_physics_not_just_pixels(bridge):
    # The polylines the scans and the map are built from are also latched
    # into the simulation as immovable contact geometry, so a car can no
    # more end a step across a wall than a ray can pass one. On a closed
    # track every wall point starts one segment.
    expected = len(bridge.world.wall_left) + len(bridge.world.wall_right)
    assert bridge.sim.wall_segment_count == expected
    manifest = bridge.sim.manifest()
    assert manifest.wall_segments == expected
    assert manifest.walls_digest != ""


def test_odometry_is_the_encoders_belief(bridge, probe):
    # The front car again, driven to a steady speed before comparing: under
    # acceleration the driven wheels spin faster than the ground goes past,
    # and the first run of this test failed by exactly that slip, which is
    # the effect the encoder exists to demonstrate, not a tolerance to widen.
    odoms = []
    probe.create_subscription(
        Odometry, "/car_1/odom", odoms.append, qos_profile_sensor_data)
    publisher = probe.create_publisher(
        AckermannDriveStamped, "/car_1/drive", 10)
    command = AckermannDriveStamped()
    command.drive.speed = 2.0
    for _ in range(40):
        publisher.publish(command)
        pump(bridge, probe, 50)

    assert odoms, "odometry must flow"
    last = odoms[-1]
    # Steady and straight: the belief agrees with the truth to within the
    # count quantisation (120 counts per revolution at 100 Hz is coarse).
    assert last.twist.twist.linear.x == pytest.approx(
        bridge.sim.state(1).vel_body.x, abs=0.3)
    assert last.pose.pose.position.x > bridge.sim.state(1).pos.x - 1.0


def test_the_imu_reports_the_acceleration_and_gravity(bridge, probe):
    from sensor_msgs.msg import Imu as ImuMsg

    imus = []
    probe.create_subscription(
        ImuMsg, "/car_1/imu", imus.append, qos_profile_sensor_data)
    publisher = probe.create_publisher(
        AckermannDriveStamped, "/car_1/drive", 10)
    command = AckermannDriveStamped()
    command.drive.speed = 2.0
    for _ in range(20):
        publisher.publish(command)
        pump(bridge, probe, 50)

    assert imus, "IMU messages must flow"
    # A car accelerating straight: the specific force is on x, not y, and a
    # planar car's z axis reads standard gravity plus the unit's errors.
    assert max(m.linear_acceleration.x for m in imus) > 0.5
    assert max(abs(m.linear_acceleration.y) for m in imus) < 0.3
    assert imus[-1].linear_acceleration.z == pytest.approx(9.80665, abs=0.5)
    assert imus[-1].orientation_covariance[0] == -1.0, \
        "no orientation is estimated, and the message says so"


def test_odometry_curves_with_the_commanded_steer(bridge, probe):
    # The dead reckoner takes its heading from the COMMANDED steering angle
    # through the kinematic bicycle, like the VESC driver it mechanises. On
    # a gentle arc at low speed it tracks the true yaw closely; a reckoner
    # that dropped the steer, or misplaced the wheelbase, is out by the
    # whole turn.
    odoms = []
    probe.create_subscription(
        Odometry, "/car_1/odom", odoms.append, qos_profile_sensor_data)
    publisher = probe.create_publisher(
        AckermannDriveStamped, "/car_1/drive", 10)
    command = AckermannDriveStamped()
    command.drive.speed = 1.5
    command.drive.steering_angle = 0.25
    for _ in range(30):
        publisher.publish(command)
        pump(bridge, probe, 50)

    assert odoms, "odometry must flow"
    orientation = odoms[-1].pose.pose.orientation
    reckoned_yaw = 2.0 * math.atan2(orientation.z, orientation.w)
    true_yaw = bridge.sim.state(1).yaw
    difference = math.atan2(math.sin(reckoned_yaw - true_yaw),
                            math.cos(reckoned_yaw - true_yaw))
    assert abs(difference) < 0.35, \
        f"reckoned {reckoned_yaw:.2f} rad against true {true_yaw:.2f} rad"
    assert abs(reckoned_yaw) > 0.3, "the commanded steer must curve the belief"


def map_qos():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def map_cell(msg, resolution, x, y):
    """The map value at a world point, with the bridge's own float64 cell
    arithmetic (info.resolution is float32 and rounds differently on a
    point that sits exactly on a cell edge, which wall vertices do)."""
    col = int((x - msg.info.origin.position.x) / resolution)
    row = int((y - msg.info.origin.position.y) / resolution)
    return msg.data[row * msg.info.width + col]


def test_the_map_is_the_walls_latched_once(bridge, probe):
    maps = []
    # The subscription joins after the bridge published: only the latch
    # can deliver it, which is what a stack launching later relies on.
    probe.create_subscription(OccupancyGrid, "/map", maps.append, map_qos())
    pump(bridge, probe, 200)
    assert len(maps) == 1, "the map is latched once, not streamed"
    grid = maps[0]
    resolution = bridge.config.map_resolution_m
    assert grid.header.frame_id == "map"
    assert grid.info.resolution == pytest.approx(resolution, rel=1e-6)
    assert len(grid.data) == grid.info.width * grid.info.height

    # Every wall vertex is the raycaster's own polyline, and every one
    # must land in an occupied cell; a re-derived or misplaced wall fails
    # here on the first vertex.
    for wall in (bridge.world.wall_left, bridge.world.wall_right):
        for x, y in wall:
            assert map_cell(grid, resolution, x, y) == 100

    # The cars start on the drivable band, which the fill reached.
    for index in range(2):
        state = bridge.sim.state(index)
        assert map_cell(grid, resolution, state.pos.x, state.pos.y) == 0

    # Beyond the walls is unknown, never free: the grid corner is outside
    # the track, and the stadium's infield is enclosed by the inner wall.
    assert grid.data[0] == -1
    assert map_cell(grid, resolution, 0.0, 0.0) == -1

    out = bridge.write_manifests()
    manifest = json.loads(
        (out / "bridge_manifest.json").read_text(encoding="utf-8"))
    assert manifest["map"] is True
    assert manifest["map_resolution_m"] == resolution


def test_the_map_agrees_with_the_raycaster(bridge):
    import slipx

    grid = bridge._occupancy_grid(
        (bridge.sim.state(0).pos.x, bridge.sim.state(0).pos.y))
    resolution = bridge.config.map_resolution_m
    state = bridge.sim.state(0)
    origin = slipx.Pose()
    origin.x = state.pos.x
    origin.y = state.pos.y

    # Side and rear bearings only: the opponent parked on the grid ahead
    # is in the world (and in the scans) but is not a wall, so it does not
    # belong to the map.
    for offset_deg in (90, 120, 150, 180, 210, 240, 270):
        bearing = state.yaw + math.radians(offset_deg)
        hit = bridge.world(0, origin, bearing)
        assert hit.hit, "a stadium surrounds the car on every side"
        # March the same ray across the map: the first occupied cell must
        # sit where the raycaster put the wall.
        step = resolution / 4.0
        marched = None
        for k in range(1, int(12.0 / step)):
            distance = k * step
            value = map_cell(grid, resolution,
                             origin.x + distance * math.cos(bearing),
                             origin.y + distance * math.sin(bearing))
            if value == 100:
                marched = distance
                break
        assert marched is not None, f"no wall on the map at {offset_deg} deg"
        assert marched == pytest.approx(hit.range, abs=0.1), \
            f"map wall at {marched:.3f} m, raycast wall at " \
            f"{hit.range:.3f} m, bearing {offset_deg} deg"


def test_the_map_survives_declining_ground_truth(ros, tmp_path):
    # The map is geometry, not truth-telling: a real car has one because
    # SLAM gave it one, localiser or no localiser. Gating it on ground
    # truth is the mutation this test exists to catch.
    node = Bridge(BridgeConfig(
        track_dir=TRACK,
        car_dirs=[CAR],
        real_time_factor=1000.0,
        ground_truth=False,
        out_dir=tmp_path,
    ))
    probe_node = rclpy.create_node("map_probe")
    try:
        maps = []
        probe_node.create_subscription(
            OccupancyGrid, "/map", maps.append, map_qos())
        for _ in range(100):
            node.step_once()
            rclpy.spin_once(probe_node, timeout_sec=0.0)
        assert node._map_pub is not None
        assert maps, "the latched map must flow without ground truth"
    finally:
        probe_node.destroy_node()
        node.destroy_node()


def test_the_map_can_be_declined_and_the_manifest_says_so(ros, tmp_path):
    node = Bridge(BridgeConfig(
        track_dir=TRACK,
        car_dirs=[CAR],
        real_time_factor=1000.0,
        map=False,
        out_dir=tmp_path,
    ))
    try:
        # No map publisher exists at all: disabled means absent, not silent.
        assert node._map_pub is None
        out = node.write_manifests()
        manifest = json.loads(
            (out / "bridge_manifest.json").read_text(encoding="utf-8"))
        assert manifest["map"] is False
    finally:
        node.destroy_node()


def test_a_map_resolution_coarser_than_the_track_is_refused(ros):
    # A 5 m cell on a 1.5 m wide track puts the wall in the seed cell:
    # refused by name, never published as a map that is all wall.
    with pytest.raises(ValueError, match="map_resolution_m"):
        Bridge(BridgeConfig(
            track_dir=TRACK,
            car_dirs=[CAR],
            real_time_factor=1000.0,
            map_resolution_m=5.0,
        ))


def tf_buffer(probe_node):
    """A tf2 buffer fed by hand from /tf and /tf_static, no listener thread:
    what lands in it crossed the RMW like any stack's transforms would."""
    from rclpy.qos import (
        DurabilityPolicy, QoSProfile, ReliabilityPolicy)
    from tf2_msgs.msg import TFMessage
    from tf2_ros.buffer import Buffer

    buffer = Buffer()

    def dynamic(msg: TFMessage) -> None:
        for transform in msg.transforms:
            buffer.set_transform(transform, "probe")

    def static(msg: TFMessage) -> None:
        for transform in msg.transforms:
            buffer.set_transform_static(transform, "probe")

    probe_node.create_subscription(TFMessage, "/tf", dynamic, 100)
    probe_node.create_subscription(
        TFMessage, "/tf_static", static,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    return buffer


def test_tf_mounts_are_identity_and_latched(bridge, probe):
    from rclpy.qos import (
        DurabilityPolicy, QoSProfile, ReliabilityPolicy)
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage

    buffer = tf_buffer(probe)
    statics = []
    probe.create_subscription(
        TFMessage, "/tf_static", statics.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pump(bridge, probe, 100)
    for namespace in ("car_0", "car_1"):
        for mount in ("laser_frame", "imu_frame"):
            assert buffer.can_transform(
                f"{namespace}/base_link", f"{namespace}/{mount}", Time())
    # Identity is asserted on the wire, not through the buffer: tf2
    # normalises quaternions on lookup, so a wrong rotation could come
    # back laundered to identity (a mutation escaped exactly that way).
    assert statics, "the latched mounts must reach a late subscriber"
    transforms = statics[-1].transforms
    assert len(transforms) == 4, "two cars, two mounts each; base_link " \
        "itself is a mount and gets no self edge"
    for transform in transforms:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        assert translation.x == 0.0 and translation.y == 0.0, \
            "the sensor models cast from the vehicle origin"
        assert rotation.w == 1.0 and rotation.x == 0.0 \
            and rotation.y == 0.0 and rotation.z == 0.0


def test_tf_composes_to_the_true_pose_through_the_odom_correction(
        bridge, probe):
    # The reckoner drifts on a tight arc (slip, spin-up, quantisation); the
    # map -> odom correction must carry exactly that drift, so the chain
    # map -> odom -> base_link lands on the true pose while odom ->
    # base_link alone stays the honest belief.
    from rclpy.time import Time

    buffer = tf_buffer(probe)
    publisher = probe.create_publisher(
        AckermannDriveStamped, "/car_1/drive", 10)
    command = AckermannDriveStamped()
    # Fast on a tight arc: at the friction limit the car runs wide of the
    # kinematic line, which is where dead reckoning genuinely breaks.
    command.drive.speed = 3.0
    command.drive.steering_angle = 0.25
    for _ in range(40):
        publisher.publish(command)
        pump(bridge, probe, 50)

    state = bridge.sim.state(1)
    belief = buffer.lookup_transform("car_1/odom", "car_1/base_link", Time())
    drift = math.hypot(belief.transform.translation.x - state.pos.x,
                       belief.transform.translation.y - state.pos.y)
    # The operating point must be reachable before the assertion means
    # anything: without real drift, an identity correction would also pass.
    assert drift > 0.15, f"the manoeuvre must drift the belief ({drift:.3f} m)"

    composed = buffer.lookup_transform("map", "car_1/base_link", Time())
    assert composed.transform.translation.x == pytest.approx(
        state.pos.x, abs=0.05)
    assert composed.transform.translation.y == pytest.approx(
        state.pos.y, abs=0.05)
    rotation = composed.transform.rotation
    composed_yaw = 2.0 * math.atan2(rotation.z, rotation.w)
    difference = math.atan2(math.sin(composed_yaw - state.yaw),
                            math.cos(composed_yaw - state.yaw))
    assert abs(difference) < 0.03

    # And through the static mount: the scan is placeable in the map.
    placed = buffer.lookup_transform("map", "car_1/laser_frame", Time())
    assert placed.transform.translation.x == pytest.approx(
        state.pos.x, abs=0.05)


def test_tf_map_edge_declines_with_ground_truth(ros, tmp_path):
    from rclpy.time import Time

    node = Bridge(BridgeConfig(
        track_dir=TRACK,
        car_dirs=[CAR],
        real_time_factor=1000.0,
        ground_truth=False,
        out_dir=tmp_path,
    ))
    probe_node = rclpy.create_node("tf_probe")
    try:
        buffer = tf_buffer(probe_node)
        for _ in range(100):
            node.step_once()
            rclpy.spin_once(probe_node, timeout_sec=0.0)
        # The belief stays on the wire; the truth-bearing edge does not.
        assert buffer.lookup_transform(
            "car_0/odom", "car_0/base_link", Time()) is not None
        assert not buffer.can_transform("map", "car_0/odom", Time()), \
            "declining ground truth must also decline the map edge"
    finally:
        probe_node.destroy_node()
        node.destroy_node()


def test_tf_can_be_declined_entirely_and_the_manifest_says_so(ros, tmp_path):
    node = Bridge(BridgeConfig(
        track_dir=TRACK,
        car_dirs=[CAR],
        real_time_factor=1000.0,
        tf=False,
        out_dir=tmp_path,
    ))
    try:
        for _ in range(50):
            node.step_once()
        # No TF publisher exists at all: disabled means absent, not silent.
        assert node._tf_pub is None and node._tf_static_pub is None
        out = node.write_manifests()
        manifest = json.loads(
            (out / "bridge_manifest.json").read_text(encoding="utf-8"))
        assert manifest["tf"] is False
    finally:
        node.destroy_node()


def test_ground_truth_can_be_declined_and_the_manifests_say_so(
        ros, tmp_path):
    node = Bridge(BridgeConfig(
        track_dir=TRACK,
        car_dirs=[CAR],
        real_time_factor=1000.0,
        ground_truth=False,
        out_dir=tmp_path,
    ))
    try:
        for _ in range(50):
            node.step_once()
        out = node.write_manifests()
        bridge_manifest = json.loads(
            (out / "bridge_manifest.json").read_text(encoding="utf-8"))
        assert bridge_manifest["ground_truth"] is False
        assert "input log" in bridge_manifest["reproducibility"]
        run_manifest = (out / "run_manifest.json").read_text(encoding="utf-8")
        assert "validation" in run_manifest
        # No ground truth publisher exists at all: disabled means absent,
        # not silent.
        assert not node._gt_pubs
    finally:
        node.destroy_node()


def test_replay_from_the_log_is_the_promise_a_live_run_keeps(ros, tmp_path):
    node = Bridge(BridgeConfig(
        track_dir=TRACK,
        car_dirs=[CAR],
        real_time_factor=1000.0,
        out_dir=tmp_path,
    ))
    try:
        for _ in range(200):
            node.step_once()
        log = list(node.sim.input_log())
        hash_before = node.sim.trajectory_hash()
        node.sim.replay(log)
        assert node.sim.trajectory_hash() == hash_before, \
            "the input log replays the live run bit for bit (ADR-0044)"
    finally:
        node.destroy_node()
