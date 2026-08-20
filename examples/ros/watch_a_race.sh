#!/bin/bash
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0
#
# Watch a bridge race: N agents on a shipped track, a driver for each,
# and RViz over the lot. See race_demo_driver.py.
#
#   ./examples/ros/watch_a_race.sh [agents] [gap|pursuit|racer|mixed] \
#       [top_speed] [track]
#
# The default is the show: twenty cars dealt a seeded mix of greedy racers
# and gap followers on the paddock_gp circuit. The track argument is a
# directory name under examples/tracks or a path to a track directory;
# SEED=n in the environment changes the deal. Closing RViz stops the
# bridge and the driver with it.
set -e

AGENTS=${1:-20}
MODE=${2:-mixed}
SPEED=${3:-4.0}
TRACK_ARG=${4:-paddock_gp}
SEED=${SEED:-0}
PYTHON=${PYTHON:-python3}
REPO=$(cd "$(dirname "$0")/../.." && pwd)

if [ -d "$TRACK_ARG" ]; then
    TRACK=$TRACK_ARG
else
    TRACK="$REPO/examples/tracks/$TRACK_ARG"
fi

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
    --agents "$AGENTS" --mode "$MODE" --speed "$SPEED" --seed "$SEED" &
DRIVER=$!
trap 'kill $BRIDGE $DRIVER 2>/dev/null; wait 2>/dev/null' EXIT INT TERM

rviz2 -d "$OUT/race.rviz"
