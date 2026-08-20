# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

# Generate an RViz config for watching a bridge race: the latched map,
# the TF tree, and per-car scans and ground-truth arrows, each car in its
# own colour. Stdlib only.

from __future__ import annotations

import argparse
import colorsys
import csv
import math
from pathlib import Path


def car_colour(index: int, agents: int) -> str:
    r, g, b = colorsys.hsv_to_rgb(index / max(agents, 1), 0.85, 0.95)
    return f"{int(r * 255)}; {int(g * 255)}; {int(b * 255)}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--agents", type=int, default=20)
    parser.add_argument("--namespace", default="car_")
    parser.add_argument("--track", type=Path, required=True,
                        help="track directory, for centring the view")
    parser.add_argument("--out", type=Path, default=Path("race.rviz"))
    arguments = parser.parse_args()

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
    scale = 800.0 / extent

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
        colour = car_colour(index, arguments.agents)
        displays.append(f"""    - Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: scan {index}
      Topic:
        Value: {ns}/scan
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
      Color Transformer: FlatColor
      Color: {colour}
      Style: Points
      Size (Pixels): 2
      Alpha: 0.7
      Decay Time: 0
    - Class: rviz_default_plugins/Odometry
      Enabled: true
      Name: car {index}
      Topic:
        Value: {ns}/ground_truth/odom
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Shape:
        Value: Arrow
        Color: {colour}
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
    print(f"wrote {arguments.out} for {arguments.agents} agents "
          f"(view centred {centre_x:.1f}, {centre_y:.1f}, "
          f"scale {scale:.0f} px/m)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
