#!/usr/bin/env bash
# record_mapping_bag.sh — Record a raw mapping bag with all required topics.
#
# Usage: ./scripts/record_mapping_bag.sh [--name run1] [--estimate-gb 25]
set -eo pipefail

NAME=""
ESTIMATE_GB=25
WS_ROOT="${HOME}/dance_ws_pedestrian_tracking"

# Source ROS env with -u disabled (Humble's setup.bash uses unbound vars)
set +u
source "${WS_ROOT}/scripts/lio_sam_env.sh"
set -u

while [[ $# -gt 0 ]]; do
    case $1 in
        --name) NAME="$2"; shift 2 ;;
        --estimate-gb) ESTIMATE_GB="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

DATE=$(date +%Y-%m-%d)
if [[ -n "$NAME" ]]; then
    RUN_DIR="bags/raw/${DATE}_${NAME}"
else
    RUN_DIR="bags/raw/${DATE}_$(date +%H%M%S)"
fi

BAG_PATH="${WS_ROOT}/${RUN_DIR}"

echo "===== Mapping Bag Recording ====="
echo "Output: $BAG_PATH"
echo ""

"${WS_ROOT}/scripts/check_storage_budget.sh" \
    --estimate-gb "$ESTIMATE_GB" \
    --path "${WS_ROOT}/bags/raw"

echo ""
echo "Running preflight..."
"${WS_ROOT}/scripts/preflight_runtime_conflicts.sh" || {
    echo "Preflight failed. Fix conflicts before recording."
    exit 1
}

echo ""
echo "Starting recording to: $BAG_PATH"
echo "Press Ctrl+C to stop."
echo ""

ros2 bag record \
    /velodyne_points \
    /vectornav/imu \
    /imu/data \
    /odometry \
    /tf \
    /tf_static \
    /mapping/imu \
    /mapping/diagnostics/pointcloud \
    /mapping/diagnostics/imu \
    /mapping/scan/nav \
    /joint_states \
    -o "$BAG_PATH" \
    --max-bag-duration 300 \
    -s sqlite3

echo ""
echo "Recording saved to: $BAG_PATH"
echo "Inspect with: ./scripts/inspect_bag_fields.sh $BAG_PATH"
