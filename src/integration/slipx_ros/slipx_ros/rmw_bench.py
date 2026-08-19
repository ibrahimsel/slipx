# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The RMW benchmark (ADR-0052): what should carry a race.

Three transport configurations, the same lockstep race: Fast-DDS as Jazzy
ships it (multicast discovery), Fast-DDS against a discovery server, and
rmw_zenoh against its router. Each run is a bridge process and one client
process per agent, every agent fully sensored, so the numbers include the
message assembly the bridge does and the barrier round trip the transport
does; what differs between rows is the transport alone.

Two measurements per row, because they answer different questions:

- discovery seconds: from bridge start to every client's drive publisher
  matched. This is the number that decides whether twenty stacks can join
  a grid without a coffee break, and the one multicast trouble inflates.
- lockstep steps per second: barrier round trips through the transport,
  everything answering every step. This is the racing number.

Run it in a sourced ROS 2 environment::

    python -m slipx_ros.rmw_bench --agents 6 20 --steps 300

The orchestrator owns the helper daemons (fast-discovery-server,
rmw_zenohd) and a distinct ROS_DOMAIN_ID per row, so rows cannot discover
each other. Results print as a table and, with --json, as machine-readable
lines for the record.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Dict, List, Optional

REPO = Path(__file__).resolve().parents[3].parent
TRACK = REPO / "examples" / "tracks" / "paddock_stadium"
CAR = REPO / "examples" / "cars" / "reference_1_10"

CONFIGURATIONS = {
    "fastdds-multicast": {
        "env": {"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"},
        "daemon": None,
    },
    "fastdds-discovery-server": {
        "env": {
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "ROS_DISCOVERY_SERVER": "127.0.0.1:11811",
        },
        "daemon": ["fast-discovery-server", "-i", "0",
                   "-l", "127.0.0.1", "-p", "11811"],
    },
    "zenoh": {
        "env": {"RMW_IMPLEMENTATION": "rmw_zenoh_cpp"},
        # The router is a libexec, not on PATH; found through the prefix
        # the sourced environment names.
        "daemon": [str(Path(os.environ.get(
            "ROS_DISTRO_PREFIX", "/opt/ros/jazzy"))
            / "lib" / "rmw_zenoh_cpp" / "rmw_zenohd")],
    },
}


# ----------------------------------------------------------------- roles


def run_client(namespace: str) -> None:
    import math

    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped

    from .race_sync import RaceSyncClient

    rclpy.init()
    node = rclpy.create_node(
        f"bench_stack_{namespace.strip('/').replace('/', '_')}")

    def compute(step: int):
        command = AckermannDriveStamped()
        command.drive.speed = 2.0
        command.drive.steering_angle = 0.05 * math.sin(step * 0.01)
        return command

    RaceSyncClient(node, namespace, on_step=compute)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def run_bridge(agents: int, steps: int, out_dir: str) -> None:
    import rclpy

    from .bridge import Bridge, BridgeConfig

    born = time.monotonic()
    rclpy.init()
    bridge = Bridge(BridgeConfig(
        track_dir=TRACK,
        car_dirs=[CAR] * agents,
        lockstep=True,
        lockstep_policy="wait",
        out_dir=Path(out_dir),
        seed=1,
    ))
    started = time.monotonic()
    boot = started - born
    if not bridge.wait_for_clients(timeout_s=120.0):
        print(json.dumps({"error": "discovery timed out"}), flush=True)
        return
    # The wait AFTER the bridge is up, the clients having launched
    # alongside it: zero means discovery finished inside the bridge's own
    # boot, which the boot column makes legible.
    discovery = time.monotonic() - started

    begun = time.monotonic()
    for _ in range(steps):
        bridge.step_once()
    elapsed = time.monotonic() - begun

    print(json.dumps({
        "boot_s": round(boot, 3),
        "discovery_s": round(discovery, 3),
        # The simulator's own count, not the request echoed back: a loop
        # that stepped short would otherwise report a clean row.
        "steps": bridge.sim.step_count,
        "steps_per_s": round(steps / elapsed, 1),
    }), flush=True)
    bridge.shutdown()


# ------------------------------------------------------------ orchestrator


def run_row(name: str, agents: int, steps: int, row_index: int
            ) -> Optional[Dict]:
    configuration = CONFIGURATIONS[name]
    environment = dict(os.environ)
    environment.update(configuration["env"])
    # A distinct domain per row: rows must not discover each other, or a
    # previous row's dying nodes would count as clients.
    environment["ROS_DOMAIN_ID"] = str(42 + row_index)

    daemon = None
    if configuration["daemon"] is not None:
        daemon = subprocess.Popen(
            configuration["daemon"], env=environment,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0)  # let it bind before anyone needs it

    out_dir = tempfile.mkdtemp(prefix="slipx_rmw_bench_")
    clients: List[subprocess.Popen] = []
    bridge = None
    try:
        bridge = subprocess.Popen(
            [sys.executable, "-m", "slipx_ros.rmw_bench", "--role", "bridge",
             "--agents", str(agents), "--steps", str(steps),
             "--out", out_dir],
            env=environment, stdout=subprocess.PIPE, text=True)
        for index in range(agents):
            clients.append(subprocess.Popen(
                [sys.executable, "-m", "slipx_ros.rmw_bench",
                 "--role", "client", "--namespace", f"/car_{index}"],
                env=environment,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

        output, _ = bridge.communicate(timeout=240)
        for line in output.splitlines():
            line = line.strip()
            if line.startswith("{"):
                result = json.loads(line)
                result.update({"configuration": name, "agents": agents})
                return result
        return None
    except subprocess.TimeoutExpired:
        bridge.kill()
        return {"configuration": name, "agents": agents,
                "error": "timed out"}
    finally:
        for client in clients:
            client.terminate()
        for client in clients:
            try:
                client.wait(timeout=5)
            except subprocess.TimeoutExpired:
                client.kill()
        if daemon is not None:
            daemon.terminate()
            try:
                daemon.wait(timeout=5)
            except subprocess.TimeoutExpired:
                daemon.kill()
        shutil.rmtree(out_dir, ignore_errors=True)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--role", default="orchestrate",
                        choices=["orchestrate", "bridge", "client"])
    parser.add_argument("--agents", nargs="*", type=int, default=[6, 20])
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument("--namespace", default="/car_0")
    parser.add_argument("--out", default=".")
    parser.add_argument("--json", action="store_true",
                        help="print one JSON line per row as well")
    parser.add_argument("--configurations", nargs="*",
                        default=list(CONFIGURATIONS),
                        choices=list(CONFIGURATIONS))
    arguments = parser.parse_args(argv)

    if arguments.role == "client":
        run_client(arguments.namespace)
        return 0
    if arguments.role == "bridge":
        run_bridge(arguments.agents[0] if arguments.agents else 6,
                   arguments.steps, arguments.out)
        return 0

    rows = []
    row_index = 0
    for agents in arguments.agents:
        for name in arguments.configurations:
            print(f"# {name}, {agents} agents, {arguments.steps} steps...",
                  flush=True)
            row = run_row(name, agents, arguments.steps, row_index)
            row_index += 1
            if row is not None:
                rows.append(row)
                if arguments.json:
                    print(json.dumps(row), flush=True)

    print(f"\n{'configuration':28s} {'agents':>6s} {'boot':>7s} "
          f"{'discovery':>10s} {'steps/s':>8s}")
    for row in rows:
        if "error" in row:
            print(f"{row['configuration']:28s} {row['agents']:6d} "
                  f"{row['error']:>27s}")
        else:
            print(f"{row['configuration']:28s} {row['agents']:6d} "
                  f"{row['boot_s']:7.2f} {row['discovery_s']:10.3f} "
                  f"{row['steps_per_s']:8.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
