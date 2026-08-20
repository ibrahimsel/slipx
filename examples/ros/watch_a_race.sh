#!/bin/bash
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0
#
# Watch a bridge race: N agents on the shipped track, a driver for each,
# and RViz over the lot. See race_demo_driver.py.
#
#   ./examples/ros/watch_a_race.sh [agents] [gap|pursuit] [top_speed]
#
# Closing RViz stops the bridge and the driver with it.
set -e

AGENTS=${1:-20}
MODE=${2:-gap}
SPEED=${3:-3.0}
PYTHON=${PYTHON:-python3}
REPO=$(cd "$(dirname "$0")/../.." && pwd)

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
export PYTHONPATH="$REPO/src/bindings/slipx:$REPO/src/core/slipx_schema:$REPO/src/integration/slipx_ros:$PYTHONPATH"

# Refuse early with the reason named, rather than half-starting: the usual
# cause is a python3 that lacks something the bridge imports (often a venv
# shadowing the system interpreter). Importing the bridge itself is the
# operative check; yaml and jsonschema are its lazy track-loading imports.
if ! "$PYTHON" -c "import slipx_ros.bridge, yaml, jsonschema" 2>/dev/null; then
    echo "error: $PYTHON cannot import slipx_ros.bridge, yaml and" \
         "jsonschema; set PYTHON to an interpreter that can" >&2
    exit 1
fi

TRACK="$REPO/examples/tracks/paddock_stadium"
CAR="$REPO/examples/cars/reference_1_10"
OUT="$REPO/runs/rviz_race"
mkdir -p "$OUT"

CARS=""
for ((i = 0; i < AGENTS; i++)); do CARS="$CARS --car $CAR"; done

"$PYTHON" "$REPO/examples/ros/make_race_rviz.py" \
    --agents "$AGENTS" --track "$TRACK" --out "$OUT/race.rviz"

"$PYTHON" -m slipx_ros.bridge --track "$TRACK" $CARS --out "$OUT" &
BRIDGE=$!
sleep 3
"$PYTHON" "$REPO/examples/ros/race_demo_driver.py" \
    --agents "$AGENTS" --mode "$MODE" --speed "$SPEED" --track "$TRACK" &
DRIVER=$!
trap 'kill $BRIDGE $DRIVER 2>/dev/null; wait 2>/dev/null' EXIT INT TERM

rviz2 -d "$OUT/race.rviz"
