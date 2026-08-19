# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""SlipX to ROS 2: the bridge (ADR-0050).

An rclpy node above the ``slipx`` Python package: it loads the track and the
car directories through the same loaders every other consumer uses, steps a
validation-mode simulation paced against the wall clock, and speaks the
F1TENTH dialect per agent under ``/car_N/``: ``drive`` in, ``scan``, ``imu``,
``odom`` and (unless disabled) ``ground_truth/odom`` out, with ``/clock``
for ``use_sim_time``.

Not part of the wheel: rclpy is a ROS package, not a pip dependency, so this
package is used from a sourced ROS 2 environment, in tree or installed by
hand. Run it as::

    python -m slipx_ros.bridge --track <track_dir> --car <car_dir> [--car ...]

An existing stack connects with a topic remap file and no code change; a
live run promises nothing about reproducibility beyond replay from its
input log, and both manifests the bridge writes say so.
"""

from .bridge import Bridge, BridgeConfig

__all__ = ["Bridge", "BridgeConfig"]
