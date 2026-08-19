# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The lockstep client (ADR-0051).

The simulator is the sync authority: it announces the next step index on
``/race_sync/step`` and advances only when every lockstep agent has
answered, through the step-tagged mailboxes of ADR-0044. This module is the
client side, sized so that joining costs a student three lines::

    from slipx_ros.race_sync import RaceSyncClient

    sync = RaceSyncClient(self, "/car_0")     # in the node's __init__
    ...
    sync.publish(msg)                          # where publish(msg) was

In this wrapping mode the client acknowledges every announcement the stack
has not answered ("alive, hold my last command"), and a publish is tagged
for the next unanswered step, so commands land one step after they are
made. That keeps the sim from ever outrunning the stack, but WHICH step a
command lands on still follows the stack's own cadence; a stack that wants
its computation itself step-synchronous passes ``on_step=`` and computes
when announced, which is the strict form.

Tags travel as header stamps: at a millisecond step the mapping between
stamp nanoseconds and step index is exact in both directions, so the wire
stays vanilla ``AckermannDriveStamped`` plus a ``UInt64`` acknowledgement,
and no custom message package exists (ADR-0051).
"""

from __future__ import annotations

from typing import Callable, Optional

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import UInt64

STEP_TOPIC = "/race_sync/step"

#: Announcements are latched: a client that joins late receives the current
#: step immediately instead of waiting for the next one, which is what
#: makes start-up a wait rather than a deadlock.
ANNOUNCE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

#: Acknowledgements are latched too, and depth one suffices by protocol:
#: the bridge announces step k+1 only after k was answered, so at most one
#: acknowledgement is ever outstanding, and one sent an instant before the
#: subscription matched still arrives instead of hanging the barrier.
ACK_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def step_to_stamp(step: int, dt: float) -> TimeMsg:
    """The step's simulation time, exactly, as a stamp."""
    step_ns = int(round(dt * 1.0e9))
    total = int(step) * step_ns
    return TimeMsg(sec=total // 1_000_000_000,
                   nanosec=total % 1_000_000_000)


def stamp_to_step(stamp: TimeMsg, dt: float) -> int:
    """The stamp's step index; exact for the stamps step_to_stamp writes."""
    step_ns = int(round(dt * 1.0e9))
    total = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return int(round(total / step_ns))


class RaceSyncClient:
    """One agent's side of the barrier. See the module docstring."""

    def __init__(self, node: Node, namespace: str,
                 on_step: Optional[Callable[[int],
                                            Optional[AckermannDriveStamped]]]
                 = None,
                 dt: float = 1.0e-3) -> None:
        self._node = node
        self._dt = dt
        self._on_step = on_step
        self._current: Optional[int] = None
        # The highest step already answered, by command or acknowledgement.
        self._answered = -1

        self._drive = node.create_publisher(
            AckermannDriveStamped, f"{namespace}/drive", 10)
        self._ack = node.create_publisher(
            UInt64, f"{namespace}/drive_ack", ACK_QOS)
        node.create_subscription(
            UInt64, STEP_TOPIC, self._announced, ANNOUNCE_QOS)

    @property
    def current_step(self) -> Optional[int]:
        """The most recently announced step, None before the first."""
        return self._current

    def publish(self, msg: AckermannDriveStamped) -> int:
        """Send a command for the next unanswered step; returns its index.

        The drop-in replacement for ``publisher.publish(msg)``: the tag is
        chosen so it always increases (the mailbox's rule), which means a
        command made between announcements lands on the following step,
        exactly as a servo written to between control ticks would take
        effect at the next tick.
        """
        step = self._answered + 1
        msg.header.stamp = step_to_stamp(step, self._dt)
        self._drive.publish(msg)
        self._answered = step
        return step

    def _announced(self, msg: UInt64) -> None:
        step = int(msg.data)
        self._current = step
        if step <= self._answered:
            return
        if self._on_step is not None:
            command = self._on_step(step)
            if command is not None:
                command.header.stamp = step_to_stamp(step, self._dt)
                self._drive.publish(command)
                self._answered = step
                return
        # Alive, hold my last command (ADR-0044). In wrapping mode this is
        # every step the stack did not answer itself; in callback mode it
        # is a deliberate None.
        self._ack.publish(UInt64(data=step))
        self._answered = step


__all__ = ["RaceSyncClient", "STEP_TOPIC", "ANNOUNCE_QOS", "ACK_QOS",
           "step_to_stamp", "stamp_to_step"]
