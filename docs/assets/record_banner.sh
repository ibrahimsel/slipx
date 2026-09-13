#!/bin/bash
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0
#
# Record the README banner: the ROS demo race, as RViz shows it.
#
#   ./docs/assets/record_banner.sh [output.gif]
#
# Runs the same bridge, driver and RViz as examples/ros/watch_a_race.sh
# (twenty cars dealt the seeded mixed field on the paddock_gp circuit),
# lets the field spread out, records RViz's render panel for a few seconds
# with ffmpeg's x11grab, and encodes the frames as a GIF. Nothing in the
# banner is drawn by this script: every car, scan and wall in it is a
# bridge state that RViz displayed, and the view is the one the demo's own
# config generates.
#
# Needs a sourced ROS 2 environment with rviz2, an X display (WSLg counts),
# xwininfo, and an ffmpeg that has x11grab (FFMPEG=/path names one). None
# of that is a CI dependency, which is why the GIF is checked in rather
# than built.
#
# Environment: AGENTS, MODE, SPEED, TRACK and SEED mean what they do in
# watch_a_race.sh. WARMUP is the number of seconds the race runs before
# the recording starts, DURATION the seconds recorded, FPS the frame rate,
# VIEW the pixel box the track is fitted into and WINDOW the RViz window
# size that leaves at least that much render panel. ENCODE=0 stops after
# the lossless capture, which is how WARMUP and DURATION were chosen. WORK
# names the working directory (kept when given, deleted otherwise).
set -e

OUTPUT=${1:-$(cd "$(dirname "$0")" && pwd)/slipx-banner.gif}
AGENTS=${AGENTS:-20}
MODE=${MODE:-mixed}
SPEED=${SPEED:-4.0}
TRACK_ARG=${TRACK:-paddock_gp}
SEED=${SEED:-0}
WARMUP=${WARMUP:-40}
DURATION=${DURATION:-12}
FPS=${FPS:-20}
VIEW=${VIEW:-1300x430}
WINDOW=${WINDOW:-1730x536}
ENCODE=${ENCODE:-1}
PYTHON=${PYTHON:-python3}
FFMPEG=${FFMPEG:-ffmpeg}
REPO=$(cd "$(dirname "$0")/../.." && pwd)

if [ -d "$TRACK_ARG" ]; then
    TRACK=$TRACK_ARG
else
    TRACK="$REPO/examples/tracks/$TRACK_ARG"
fi

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
export PYTHONPATH="$REPO/src/bindings/slipx:$REPO/src/core/slipx_schema:$REPO/src/integration/slipx_ros:$PYTHONPATH"

for tool in rviz2 xwininfo "$FFMPEG"; do
    if ! command -v "$tool" > /dev/null; then
        echo "error: $tool is not on PATH" >&2
        exit 1
    fi
done
if ! "$FFMPEG" -hide_banner -devices 2> /dev/null | grep -q x11grab; then
    echo "error: $FFMPEG has no x11grab input; point FFMPEG at one that does" >&2
    exit 1
fi
if [ -z "$DISPLAY" ]; then
    echo "error: no DISPLAY; RViz needs an X server to draw into" >&2
    exit 1
fi
if ! "$PYTHON" -c "import slipx_ros.bridge, yaml, jsonschema" 2> /dev/null; then
    echo "error: $PYTHON cannot import slipx_ros.bridge, yaml and" \
         "jsonschema; set PYTHON to an interpreter that can" >&2
    exit 1
fi

if [ -n "$WORK" ]; then
    mkdir -p "$WORK"
    KEEP_WORK=1
else
    WORK=$(mktemp -d)
    KEEP_WORK=0
fi
CAR="$REPO/examples/cars/reference_1_10"
CARS=""
for ((i = 0; i < AGENTS; i++)); do CARS="$CARS --car $CAR"; done

"$PYTHON" "$REPO/examples/ros/make_race_rviz.py" \
    --agents "$AGENTS" --track "$TRACK" --view "$VIEW" --window "$WINDOW" \
    --out "$WORK/race.rviz"

"$PYTHON" -m slipx_ros.bridge --track "$TRACK" $CARS --out "$WORK/run" \
    > "$WORK/bridge.log" 2>&1 &
BRIDGE=$!
sleep 3
"$PYTHON" "$REPO/examples/ros/race_demo_driver.py" \
    --agents "$AGENTS" --mode "$MODE" --speed "$SPEED" --seed "$SEED" \
    > "$WORK/driver.log" 2>&1 &
DRIVER=$!
rviz2 -d "$WORK/race.rviz" > "$WORK/rviz.log" 2>&1 &
RVIZ=$!
STARTED=$(date +%s)

stop_all() {
    kill -INT $RVIZ $DRIVER $BRIDGE 2> /dev/null
    wait 2> /dev/null
    if [ "$KEEP_WORK" = 0 ]; then rm -rf "$WORK"; fi
}
trap stop_all EXIT INT TERM

# RViz's render panel is its own X window, the largest child of the RViz
# top level, so recording it and nothing else needs no cropping and no
# knowledge of where the toolbar and docks sit in this theme.
render_window() {
    local top
    top=$(xwininfo -root -tree 2> /dev/null | awk '/ - RViz":/ { print $1; exit }')
    [ -n "$top" ] || return 1
    xwininfo -id "$top" -tree 2> /dev/null \
        | sed -n 's/^ *\(0x[0-9a-f]*\) .* \([0-9]*\)x\([0-9]*\)+[-0-9]*+[-0-9]*  .*/\1 \2 \3/p' \
        | awk '$2 * $3 > best { best = $2 * $3; id = $1; size = $2 "x" $3 }
               END { if (id != "") print id, size }'
}

RENDER=""
for ((i = 0; i < 60; i++)); do
    RENDER=$(render_window) && [ -n "$RENDER" ] && break
    sleep 1
done
if [ -z "$RENDER" ]; then
    echo "error: no RViz window appeared; see $WORK/rviz.log" >&2
    KEEP_WORK=1
    exit 1
fi
WINDOW_ID=${RENDER%% *}
WINDOW_SIZE=${RENDER##* }
echo "recording RViz render panel $WINDOW_ID ($WINDOW_SIZE) after ${WARMUP}s" \
     "for ${DURATION}s at ${FPS} fps"

while [ $(( $(date +%s) - STARTED )) -lt "$WARMUP" ]; do sleep 1; done
"$FFMPEG" -y -hide_banner -loglevel error \
    -f x11grab -framerate "$FPS" -window_id "$WINDOW_ID" -i "$DISPLAY" \
    -t "$DURATION" -c:v ffv1 "$WORK/capture.mkv"

if [ "$ENCODE" = 0 ]; then
    echo "capture kept at $WORK/capture.mkv"
    KEEP_WORK=1
    exit 0
fi

# Two passes: one palette for the whole clip, then the frames against it.
# The map and the walls never move, so a rectangle diff keeps the file
# small; Bayer dithering keeps the scan points from shimmering between
# frames the way error diffusion would.
"$FFMPEG" -y -hide_banner -loglevel error -i "$WORK/capture.mkv" \
    -filter_complex "[0:v]split[a][b];[a]palettegen=max_colors=256:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle" \
    -loop 0 "$OUTPUT"
echo "wrote $OUTPUT ($(du -h "$OUTPUT" | cut -f1))"
