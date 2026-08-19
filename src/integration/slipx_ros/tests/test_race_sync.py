# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Lockstep over real topics (ADR-0051).

The claims under test are the ones the record makes: the simulator never
outruns a client, timing cannot change a lockstep trajectory (a client that
dawdles produces bit for bit the run a prompt one does), the wrapping mode
costs three lines and one step of latency, and a silent agent is answered
by its timeout policy rather than hanging the race.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="lockstep needs a ROS 2 environment")

from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402

from slipx_ros import Bridge, BridgeConfig, RaceSyncClient  # noqa: E402
from slipx_ros.race_sync import stamp_to_step, step_to_stamp  # noqa: E402

REPO = Path(__file__).resolve().parents[4]
TRACK = REPO / "examples" / "tracks" / "paddock_stadium"
CAR = REPO / "examples" / "cars" / "reference_1_10"


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_the_tag_survives_the_stamp_exactly():
    for step in (0, 1, 24, 999, 100_000, 123_456_789):
        assert stamp_to_step(step_to_stamp(step, 1.0e-3), 1.0e-3) == step


class Stack:
    """A deterministic scripted stack on its own node and thread."""

    def __init__(self, namespace: str, dawdle_every: int = 0,
                 on_step=None) -> None:
        self.node = rclpy.create_node(
            f"stack_{namespace.strip('/').replace('/', '_')}")
        self.dawdle_every = dawdle_every
        self.steps_seen = []
        self.client = RaceSyncClient(
            self.node, namespace,
            on_step=on_step if on_step is not None else self._compute)
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin,
                                        daemon=True)
        self._thread.start()

    def _compute(self, step: int):
        self.steps_seen.append(step)
        if self.dawdle_every and step % self.dawdle_every == 0:
            time.sleep(0.002)  # a slow scheduler tick, on purpose
        command = AckermannDriveStamped()
        # Drive, then brake: braking is what makes the speed loop's
        # feedback visible, because a loop fed a zero velocity would just
        # stop pushing instead of pulling the car back down.
        command.drive.speed = 2.0 if step < 150 else 0.0
        command.drive.steering_angle = 0.05 * math.sin(step * 0.01)
        return command

    def close(self) -> None:
        self._executor.shutdown()
        self._thread.join(timeout=2.0)
        self.node.destroy_node()


def lockstep_bridge(tmp_path, cars=1, policy="wait", timeout=0.0,
                    seed=5) -> Bridge:
    return Bridge(BridgeConfig(
        track_dir=TRACK,
        car_dirs=[CAR] * cars,
        lockstep=True,
        lockstep_policy=policy,
        lockstep_timeout_s=timeout,
        seed=seed,
        out_dir=tmp_path,
    ))


def run_lockstep(tmp_path, steps: int, dawdle_every: int = 0):
    bridge = lockstep_bridge(tmp_path)
    stack = Stack("/car_0", dawdle_every=dawdle_every)
    try:
        assert bridge.wait_for_clients(), "discovery must complete"
        start_x = bridge.sim.state(0).pos.x
        for _ in range(steps):
            bridge.step_once()
        return (bridge.sim.trajectory_hash(),
                bridge.sim.state(0).pos.x - start_x,
                list(stack.steps_seen),
                bridge.sim.state(0).vel_body.x)
    finally:
        bridge.shutdown()
        bridge.destroy_node()
        stack.close()


def test_lockstep_is_reproducible_and_timing_cannot_change_it(ros, tmp_path):
    prompt = run_lockstep(tmp_path / "a", steps=250)
    again = run_lockstep(tmp_path / "b", steps=250)
    dawdling = run_lockstep(tmp_path / "c", steps=250, dawdle_every=50)

    assert prompt[0] == again[0], "the same race twice is the same race"
    assert prompt[0] == dawdling[0], \
        "a dawdling client cannot change one bit of the trajectory"
    assert prompt[1] == dawdling[1], "to the last bit of position"
    assert prompt[1] > 0.05, "the scripted stack actually drove"
    # The script brakes from step 150: a speed loop that actually reads the
    # car's velocity pulls it back down, where one fed a constant would
    # leave it coasting near full speed.
    assert prompt[3] < 0.8, "the brake phase must brake"


def test_the_simulator_never_outruns_the_client(ros, tmp_path):
    # Every step the simulator computed was announced and answered: the
    # client saw exactly the steps 0..N-1, each once, in order.
    _, _, seen, _ = run_lockstep(tmp_path, steps=200)
    assert seen == list(range(200))


def test_the_wrapping_mode_costs_three_lines_and_one_step(ros, tmp_path):
    bridge = lockstep_bridge(tmp_path)
    node = rclpy.create_node("wrapped_stack")
    client = RaceSyncClient(node, "/car_0")  # no callback: wrapping mode
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert bridge.wait_for_clients()
        # The stack publishes once, whenever it likes; the client answers
        # every announcement on its behalf and the command lands on the
        # next unanswered step.
        command = AckermannDriveStamped()
        command.drive.speed = 2.0
        tagged = client.publish(command)
        assert tagged == 0, "before any announcement, the first step"
        for _ in range(400):
            bridge.step_once()
        assert bridge.sim.state(0).vel_body.x > 0.5, \
            "one command, held like a servo, drives the car"
    finally:
        bridge.shutdown()
        bridge.destroy_node()
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()


def test_a_silent_agent_is_answered_by_its_policy_not_waited_for(
        ros, tmp_path):
    # Two agents, a client on one, nothing on the other: the timeout rules
    # each miss and the coast policy steps the silent car with the neutral
    # input, so the race continues (ADR-0044).
    bridge = lockstep_bridge(tmp_path, cars=2, policy="coast", timeout=0.02)
    stack = Stack("/car_0")
    try:
        # Only car_0 has a client; wait_for_clients would wait for both.
        deadline = time.monotonic() + 10.0
        while (bridge.count_publishers("/car_0/drive") < 1
               and time.monotonic() < deadline):
            time.sleep(0.01)
        for _ in range(100):
            bridge.step_once()
        assert bridge.sim.state(0).vel_body.x > 0.05, "the driven car moves"
        assert abs(bridge.sim.state(1).vel_body.x) < 0.01, \
            "the silent car coasts at rest instead of hanging the race"
        assert bridge.sim.agent_running(1)
    finally:
        bridge.shutdown()
        bridge.destroy_node()
        stack.close()


def test_a_silent_agent_can_be_ruled_out_entirely(ros, tmp_path):
    # A generous timeout: the answering client must never be ruled out by a
    # slow scheduler tick, and the silent one only pays it once.
    bridge = lockstep_bridge(tmp_path, cars=2, policy="dnf", timeout=0.2)
    stack = Stack("/car_0")
    try:
        deadline = time.monotonic() + 10.0
        while (bridge.count_publishers("/car_0/drive") < 1
               and time.monotonic() < deadline):
            time.sleep(0.01)
        for _ in range(20):
            bridge.step_once()
        assert not bridge.sim.agent_running(1), \
            "a dnf policy forfeits the silent car"
        assert bridge.sim.agent_running(0)
    finally:
        bridge.shutdown()
        bridge.destroy_node()
        stack.close()
