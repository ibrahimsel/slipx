# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

# Generate an RViz config for watching a bridge race: the latched map, a
# car body per agent, and per-car scans, each car in its own colour. The
# bodies are small URDFs written next to the config, one per car so each
# carries its own colour, loaded by RViz from file and placed by the TF
# the bridge already broadcasts (car_N/base_link); every visual hangs off
# that one link, so no joint state is needed. Dimensions come from the
# car's own dynamics.yaml rather than being retyped here.

from __future__ import annotations

import argparse
import colorsys
import csv
import math
from pathlib import Path

import yaml


def car_colour(index: int, agents: int) -> tuple:
    return colorsys.hsv_to_rgb(index / max(agents, 1), 0.85, 0.95)


def rviz_colour(rgb: tuple) -> str:
    return "; ".join(str(int(c * 255)) for c in rgb)


def urdf_colour(rgb: tuple) -> str:
    return f"{rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} 1.0"


def car_urdf(name: str, rgb: tuple, geometry: dict) -> str:
    """One link, every visual on it. The chassis and the nose wear the
    car's colour and a heading mark; wheels and the lidar puck do not."""
    length = float(geometry["length"])
    width = float(geometry["width"])
    lf = float(geometry["lf"])
    lr = float(geometry["lr"])
    half_track = 0.5 * float(geometry["track_front"])
    wheel_r = float(geometry["wheel_radius"])
    wheel_w = 0.05

    def wheel(x: float, y: float) -> str:
        return f"""
    <visual>
      <origin xyz="{x:.3f} {y:.3f} {wheel_r:.3f}" rpy="1.5708 0 0"/>
      <geometry><cylinder radius="{wheel_r:.3f}" length="{wheel_w:.3f}"/></geometry>
      <material name="tyre"/>
    </visual>"""

    wheels = "".join(
        wheel(x, y)
        for x in (lf, -lr)
        for y in (half_track, -half_track)
    )
    return f"""<?xml version="1.0"?>
<robot name="{name}">
  <material name="body"><color rgba="{urdf_colour(rgb)}"/></material>
  <material name="tyre"><color rgba="0.08 0.08 0.08 1.0"/></material>
  <material name="nose"><color rgba="0.95 0.95 0.95 1.0"/></material>
  <material name="puck"><color rgba="0.25 0.25 0.28 1.0"/></material>
  <link name="base_link">
    <visual>
      <origin xyz="0 0 {wheel_r + 0.025:.3f}" rpy="0 0 0"/>
      <geometry><box size="{length:.3f} {0.6 * width:.3f} 0.05"/></geometry>
      <material name="body"/>
    </visual>
    <visual>
      <origin xyz="{0.5 * length - 0.05:.3f} 0 {wheel_r + 0.055:.3f}" rpy="0 0 0"/>
      <geometry><box size="0.10 {0.6 * width:.3f} 0.012"/></geometry>
      <material name="nose"/>
    </visual>
    <visual>
      <origin xyz="0 0 {wheel_r + 0.075:.3f}" rpy="0 0 0"/>
      <geometry><cylinder radius="0.035" length="0.05"/></geometry>
      <material name="puck"/>
    </visual>{wheels}
  </link>
</robot>
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--agents", type=int, default=20)
    parser.add_argument("--namespace", default="car_")
    parser.add_argument("--track", type=Path, required=True,
                        help="track directory, for centring the view")
    parser.add_argument("--car", type=Path,
                        default=Path(__file__).resolve().parents[1]
                        / "cars" / "reference_1_10",
                        help="car directory, for the body dimensions")
    parser.add_argument("--out", type=Path, default=Path("race.rviz"))
    arguments = parser.parse_args()

    with open(arguments.car / "dynamics.yaml", encoding="utf-8") as handle:
        geometry = yaml.safe_load(handle)["geometry"]

    xs, ys = [], []
    with open(arguments.track / "centreline.csv", encoding="utf-8",
              newline="") as handle:
        for row in csv.reader(handle):
            if not row or row[0].lstrip().startswith("#"):
                continue
            xs.append(float(row[0]))
            ys.append(float(row[1]))
    centre_x = 0.5 * (min(xs) + max(xs))
    centre_y = 0.5 * (min(ys) + max(ys))
    extent = max(max(xs) - min(xs), max(ys) - min(ys)) + 4.0
    scale = 900.0 / extent

    out_dir = arguments.out.resolve().parent
    out_dir.mkdir(parents=True, exist_ok=True)

    displays = ["""    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Map
      Topic:
        Value: /map
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
      Color Scheme: map
      Draw Behind: true
    - Class: rviz_default_plugins/TF
      Enabled: false
      Name: TF
      Show Names: false
      Show Axes: true
      Show Arrows: false
      Marker Scale: 0.5"""]

    for index in range(arguments.agents):
        ns = f"/{arguments.namespace}{index}"
        rgb = car_colour(index, arguments.agents)
        urdf_path = out_dir / f"race_car_{index}.urdf"
        urdf_path.write_text(
            car_urdf(f"race_car_{index}", rgb, geometry), encoding="utf-8")
        displays.append(f"""    - Class: rviz_default_plugins/RobotModel
      Enabled: true
      Name: model {index}
      Description Source: File
      Description File: {urdf_path}
      TF Prefix: {arguments.namespace}{index}
      Visual Enabled: true
      Alpha: 1
    - Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: scan {index}
      Topic:
        Value: {ns}/scan
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
      Color Transformer: FlatColor
      Color: {rviz_colour(rgb)}
      Style: Points
      Size (Pixels): 2
      Alpha: 0.7
      Decay Time: 0
    - Class: rviz_default_plugins/Odometry
      Enabled: false
      Name: car {index}
      Topic:
        Value: {ns}/ground_truth/odom
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Shape:
        Value: Arrow
        Color: {rviz_colour(rgb)}
        Alpha: 1
        Shaft Length: 0.35
        Shaft Radius: 0.05
        Head Length: 0.15
        Head Radius: 0.09
      Position Tolerance: 0.05
      Angle Tolerance: 0.05
      Keep: 1
      Covariance:
        Value: false""")

    config = f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded: ~
Visualization Manager:
  Class: ""
  Displays:
{chr(10).join(displays)}
  Global Options:
    Background Color: 33; 33; 38
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: Top Down
      Scale: {scale:.0f}
      X: {centre_x:.2f}
      Y: {centre_y:.2f}
      Angle: 0
Window Geometry:
  Height: 900
  Width: 1400
"""
    arguments.out.write_text(config, encoding="utf-8")
    print(f"wrote {arguments.out} and {arguments.agents} car bodies "
          f"(view centred {centre_x:.1f}, {centre_y:.1f}, "
          f"scale {scale:.0f} px/m)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
