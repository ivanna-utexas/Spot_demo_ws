#!/usr/bin/env bash
# extract_scan_bag.sh — Extract 2D LaserScan from a raw 3D bag for ROS1 handoff.
#
# Replays a raw bag through scan_extractor_node and records the 2D output.
# The resulting bag contains only /mapping/scan/nav, /tf, /tf_static.
#
# Usage: ./scripts/extract_scan_bag.sh <raw_bag_dir> [--output <output_bag_dir>]
set -euo pipefail

source_setup_file() {
    local setup_file="$1"

    if [ ! -f "${setup_file}" ]; then
        return 0
    fi

    local had_nounset="0"
    case $- in
        *u*)
            had_nounset="1"
            set +u
            ;;
    esac

    # shellcheck disable=SC1090
    source "${setup_file}" --

    if [ "${had_nounset}" = "1" ]; then
        set -u
    fi
}

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <raw_bag_dir> [--output <output_bag_dir>]"
    exit 1
fi

RAW_BAG="$1"; shift
WS_ROOT="${HOME}/nav_ws"
OUTPUT=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --output) OUTPUT="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

# Resolve paths
if [[ ! -d "$RAW_BAG" ]]; then
    RAW_BAG="${WS_ROOT}/${RAW_BAG}"
fi
if [[ ! -d "$RAW_BAG" ]]; then
    echo "Error: bag directory not found"
    exit 1
fi

BAG_NAME=$(basename "$RAW_BAG")
if [[ -z "$OUTPUT" ]]; then
    OUTPUT="${WS_ROOT}/bags/replay/${BAG_NAME}_scan_only"
fi

echo "===== LaserScan Extraction for ROS1 Handoff ====="
echo "Input:  $RAW_BAG"
echo "Output: $OUTPUT"
echo ""

# Source workspace
source_setup_file /opt/ros/humble/setup.bash
source_setup_file "${WS_ROOT}/install/setup.bash"

# Launch scan extractor
echo "Starting scan_extractor_node..."
ros2 run spot_mapping_common scan_extractor_node \
    --ros-args \
    -p input_topic:=/velodyne_points \
    -p output_topic:=/mapping/scan/nav \
    -p frame_id:=flat_body \
    -p min_height:=-0.1 \
    -p max_height:=0.5 \
    -p use_sim_time:=true &
EXTRACTOR_PID=$!
sleep 2

# Start recording
echo "Starting bag recording..."
ros2 bag record \
    /mapping/scan/nav \
    /tf \
    /tf_static \
    -o "$OUTPUT" \
    -s sqlite3 &
RECORD_PID=$!
sleep 1

# Play the raw bag
echo "Playing raw bag..."
ros2 bag play "$RAW_BAG" --clock --rate 1.0
sleep 2

# Stop recording and extractor
kill $RECORD_PID 2>/dev/null || true
kill $EXTRACTOR_PID 2>/dev/null || true
wait $RECORD_PID 2>/dev/null || true
wait $EXTRACTOR_PID 2>/dev/null || true

echo ""
echo "===== Extraction Complete ====="
echo "Scan bag: $OUTPUT"
echo ""
echo "Next: Convert to ROS1 format for handoff:"
echo "  pip3 install rosbags"
echo "  rosbags-convert $OUTPUT --dst ${OUTPUT}.bag"
echo ""
echo "See README_navstack_handoff.md for full ROS1 validation procedure."
