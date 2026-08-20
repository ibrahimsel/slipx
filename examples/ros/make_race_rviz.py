# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

# Generate an RViz config for watching a bridge race: the latched map, a
# car body per agent, and per-car scans, each car in its own colour. The
# bodies are small URDFs written next to the config, one per car so each
# carries its own colour, loaded by RViz from file and placed by the TF
# the bridge already broadcasts (car_N/base_link); every visual hangs off
# that one link, so no joint state is needed. Dimensions come from the
# car's own dynamics.yaml rather than being retyped here.
#
# The body itself is a little open-wheel racer: a lofted low-poly shell,
# front and rear wings with endplates, wheels with hubs and the lidar
# puck. It is one Collada mesh per car with the colours embedded,
# because RViz (Jazzy) paints every visual of a link with the first
# visual's material, so a multi-part URDF cannot carry more than one
# colour per link; mesh-embedded materials are kept. The white front
# wing is the heading mark. Proportions are fractions of the car's own
# length, width and wheel radius, so a different car file changes the
# body with it.

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


def _tri(a: tuple, b: tuple, c: tuple, outward: tuple) -> tuple:
    """One triangle (normal, a, b, c), wound so its normal agrees with
    the outward hint. Orienting from the hint instead of by construction
    means no builder can get a face inside-out."""
    u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    n = (u[1] * v[2] - u[2] * v[1],
         u[2] * v[0] - u[0] * v[2],
         u[0] * v[1] - u[1] * v[0])
    if n[0] * outward[0] + n[1] * outward[1] + n[2] * outward[2] < 0:
        b, c = c, b
        n = (-n[0], -n[1], -n[2])
    norm = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2) or 1.0
    return ((n[0] / norm, n[1] / norm, n[2] / norm), a, b, c)


def _quad(a: tuple, b: tuple, c: tuple, d: tuple, outward: tuple) -> list:
    return [_tri(a, b, c, outward), _tri(a, c, d, outward)]


def _box(centre: tuple, size: tuple) -> list:
    x, y, z = centre
    hx, hy, hz = (0.5 * s for s in size)
    corner = {(sx, sy, sz): (x + sx * hx, y + sy * hy, z + sz * hz)
              for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)}
    tris = []
    for axis in range(3):
        for sign in (-1, 1):
            face = [p for k, p in sorted(corner.items()) if k[axis] == sign]
            outward = tuple(sign if i == axis else 0 for i in range(3))
            tris += _quad(face[0], face[1], face[3], face[2], outward)
    return tris


def _disc(centre: tuple, axis: str, radius: float, half_width: float,
          segments: int = 16) -> list:
    """A cylinder along y (a wheel) or z (the lidar puck)."""
    rings = []
    for offset in (-half_width, half_width):
        ring = []
        for k in range(segments):
            angle = 2.0 * math.pi * k / segments
            u, v = radius * math.cos(angle), radius * math.sin(angle)
            if axis == "y":
                ring.append((centre[0] + u, centre[1] + offset,
                             centre[2] + v))
            else:
                ring.append((centre[0] + u, centre[1] + v,
                             centre[2] + offset))
        rings.append(ring)
    tris = []
    for k in range(segments):
        j = (k + 1) % segments
        side = (rings[0][k], rings[0][j], rings[1][j], rings[1][k])
        mid = tuple(sum(p[i] for p in side) / 4.0 for i in range(3))
        tris += _quad(*side, tuple(m - c for m, c in zip(mid, centre)))
    for ring, sign in ((rings[0], -1.0), (rings[1], 1.0)):
        cap_centre = tuple(sum(p[i] for p in ring) / segments
                           for i in range(3))
        outward = (0, sign, 0) if axis == "y" else (0, 0, sign)
        for k in range(segments):
            tris.append(_tri(cap_centre, ring[k], ring[(k + 1) % segments],
                             outward))
    return tris


def _shell(geometry: dict) -> list:
    """The monocoque: hexagonal cross-sections lofted from tail to nose
    tip. Stations are (x, half width, floor z, top z); the hexagon
    bevels the floor and the top so the body reads as a shape, not a
    slab."""
    length = float(geometry["length"])
    width = float(geometry["width"])
    r = float(geometry["wheel_radius"])
    half = 0.5 * length
    stations = [
        (-half, 0.18 * width, 1.0 * r, 1.7 * r),            # tail
        (-float(geometry["lr"]), 0.28 * width, 0.6 * r, 2.0 * r),
        (-0.10 * length, 0.30 * width, 0.6 * r, 2.4 * r),   # cockpit crest
        (0.10 * length, 0.27 * width, 0.6 * r, 2.0 * r),
        (float(geometry["lf"]), 0.17 * width, 0.6 * r, 1.5 * r),
        (half - 0.06 * length, 0.05 * width, 0.9 * r, 1.3 * r),  # nose tip
    ]
    rings = []
    for x, hw, z0, z1 in stations:
        zm = 0.5 * (z0 + z1)
        rings.append([(x, -0.85 * hw, z0), (x, 0.85 * hw, z0), (x, hw, zm),
                      (x, 0.7 * hw, z1), (x, -0.7 * hw, z1), (x, -hw, zm)])
    tris = []
    for ring_a, ring_b in zip(rings, rings[1:]):
        axis_z = 0.25 * (ring_a[2][2] + ring_a[5][2]
                         + ring_b[2][2] + ring_b[5][2])
        for i in range(6):
            j = (i + 1) % 6
            quad = (ring_a[i], ring_a[j], ring_b[j], ring_b[i])
            centre = tuple(sum(p[k] for p in quad) / 4.0 for k in range(3))
            tris += _quad(*quad, (0.0, centre[1], centre[2] - axis_z))
    for ring, outward in ((rings[0], (-1.0, 0.0, 0.0)),
                          (rings[-1], (1.0, 0.0, 0.0))):
        centre = tuple(sum(p[k] for p in ring) / 6.0 for k in range(3))
        for i in range(6):
            tris.append(_tri(centre, ring[i], ring[(i + 1) % 6], outward))
    return tris


def car_parts(geometry: dict) -> dict:
    """Triangles per material: the shell and rear wing wear the car's
    colour ("body"), the front wing is white ("trim") so the heading
    reads at a glance, and tyres, hubs and the lidar puck are the same
    on every car."""
    length = float(geometry["length"])
    width = float(geometry["width"])
    lf = float(geometry["lf"])
    lr = float(geometry["lr"])
    half_track = 0.5 * float(geometry["track_front"])
    r = float(geometry["wheel_radius"])
    half = 0.5 * length
    wheel_w = 0.05

    parts = {"body": _shell(geometry), "trim": [], "tyre": [], "hub": [],
             "grey": []}
    # Front wing under the nose, with endplates; the one white part.
    wing_x = half - 0.055 * length
    parts["trim"] += _box((wing_x, 0.0, 0.6 * r),
                          (0.11 * length, 0.90 * width, 0.15 * r))
    for side in (1.0, -1.0):
        parts["trim"] += _box((wing_x, side * 0.45 * width, 0.9 * r),
                              (0.11 * length, 0.04 * width, 0.9 * r))
    # Rear wing on two struts, in the car's colour.
    wing_x = -(half - 0.045 * length)
    parts["body"] += _box((wing_x, 0.0, 2.7 * r),
                          (0.09 * length, 0.85 * width, 0.2 * r))
    for side in (1.0, -1.0):
        parts["body"] += _box((wing_x, side * 0.425 * width, 2.6 * r),
                              (0.13 * length, 0.04 * width, 1.0 * r))
        parts["grey"] += _box((wing_x, side * 0.15 * width, 2.2 * r),
                              (0.02 * length, 0.02 * width, 1.0 * r))
    # Wheels with a lighter hub proud of each outer face.
    for x in (lf, -lr):
        for side in (1.0, -1.0):
            y = side * half_track
            parts["tyre"] += _disc((x, y, r), "y", r, 0.5 * wheel_w)
            parts["hub"] += _disc(
                (x, y + side * (0.5 * wheel_w + 0.002), r), "y",
                0.55 * r, 0.006)
    # The lidar puck rides the cockpit crest.
    parts["grey"] += _disc((-0.10 * length, 0.0, 2.9 * r), "z",
                           0.7 * r, 0.5 * r)
    return parts


def car_dae(name: str, rgb: tuple, parts: dict) -> str:
    """The whole car as one Collada mesh with its materials embedded;
    see the header comment for why the colours cannot live in the URDF."""
    colours = {
        "body": f"{rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} 1",
        "tyre": "0.08 0.08 0.08 1",
        "trim": "0.95 0.95 0.95 1",
        "grey": "0.25 0.25 0.28 1",
        "hub": "0.75 0.75 0.78 1",
    }
    effects, materials, geometries, instances = [], [], [], []
    for kind, tris in parts.items():
        effects.append(f"""    <effect id="{kind}-fx">
      <profile_COMMON><technique sid="common"><phong>
        <ambient><color>{colours[kind]}</color></ambient>
        <diffuse><color>{colours[kind]}</color></diffuse>
        <specular><color>0.25 0.25 0.25 1</color></specular>
        <shininess><float>32</float></shininess>
      </phong></technique></profile_COMMON>
    </effect>""")
        materials.append(f'    <material id="{kind}-mat" name="{kind}">'
                         f'<instance_effect url="#{kind}-fx"/></material>')
        positions, normals = [], []
        for normal, *vertices in tris:
            for vertex in vertices:
                positions += [f"{c:.6f}" for c in vertex]
                normals += [f"{c:.6f}" for c in normal]
        count = 3 * len(tris)
        geometries.append(f"""    <geometry id="{kind}-geo"><mesh>
      <source id="{kind}-pos">
        <float_array id="{kind}-pos-a" count="{3 * count}">{" ".join(positions)}</float_array>
        <technique_common><accessor source="#{kind}-pos-a" count="{count}" stride="3">
          <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
        </accessor></technique_common>
      </source>
      <source id="{kind}-nor">
        <float_array id="{kind}-nor-a" count="{3 * count}">{" ".join(normals)}</float_array>
        <technique_common><accessor source="#{kind}-nor-a" count="{count}" stride="3">
          <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
        </accessor></technique_common>
      </source>
      <vertices id="{kind}-vtx"><input semantic="POSITION" source="#{kind}-pos"/></vertices>
      <triangles material="{kind}-sym" count="{len(tris)}">
        <input semantic="VERTEX" source="#{kind}-vtx" offset="0"/>
        <input semantic="NORMAL" source="#{kind}-nor" offset="0"/>
        <p>{" ".join(str(i) for i in range(count))}</p>
      </triangles>
    </mesh></geometry>""")
        instances.append(f"""        <instance_geometry url="#{kind}-geo">
          <bind_material><technique_common>
            <instance_material symbol="{kind}-sym" target="#{kind}-mat"/>
          </technique_common></bind_material>
        </instance_geometry>""")
    newline = "\n"
    return f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_effects>
{newline.join(effects)}
  </library_effects>
  <library_materials>
{newline.join(materials)}
  </library_materials>
  <library_geometries>
{newline.join(geometries)}
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="scene">
      <node id="{name}">
{newline.join(instances)}
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""


def car_urdf(name: str, dae_path: Path) -> str:
    """One link, one mesh visual; the mesh carries its own colours."""
    return f"""<?xml version="1.0"?>
<robot name="{name}">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="file://{dae_path}"/></geometry>
    </visual>
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
    parts = car_parts(geometry)

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
        dae_path = out_dir / f"race_car_{index}.dae"
        dae_path.write_text(car_dae(f"race_car_{index}", rgb, parts),
                            encoding="utf-8")
        urdf_path = out_dir / f"race_car_{index}.urdf"
        urdf_path.write_text(car_urdf(f"race_car_{index}", dae_path),
                             encoding="utf-8")
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
