#!/usr/bin/env bash
# replay_mapping_bag.sh — Replay a raw bag for offline mapping.
#
# Replay is the authoritative map-building path (not live RViz).
# Outputs go to bags/replay/<date_run>/.
#
# Usage:
#   ./scripts/replay_mapping_bag.sh <bag_directory> [--rate 1.0] [--backend liorf|lio_sam]
#     [--pointcloud-topic /velodyne_points_upstream] [--imu-topic /vectornav/imu]
#     [--packet-topic /velodyne_packets]
set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <bag_directory> [--rate 1.0] [--backend liorf|lio_sam] [--pointcloud-topic TOPIC] [--imu-topic TOPIC] [--packet-topic TOPIC]"
    echo "Example: $0 bags/raw/2026-03-26_run1 --rate 0.5 --backend lio_sam"
    exit 1
fi

BAG_DIR="$1"; shift
RATE="1.0"
BACKEND="liorf"
POINTCLOUD_TOPIC=""
IMU_INPUT_TOPIC=""
PACKET_TOPIC=""
WS_ROOT="${HOME}/dance_ws_pedestrian_tracking"
PLAY_QOS_OVERRIDES=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --rate) RATE="$2"; shift 2 ;;
        --backend) BACKEND="$2"; shift 2 ;;
        --pointcloud-topic) POINTCLOUD_TOPIC="$2"; shift 2 ;;
        --imu-topic) IMU_INPUT_TOPIC="$2"; shift 2 ;;
        --packet-topic) PACKET_TOPIC="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

if [[ ! -d "$BAG_DIR" ]]; then
    # Try relative to workspace
    BAG_DIR="${WS_ROOT}/${BAG_DIR}"
    if [[ ! -d "$BAG_DIR" ]]; then
        echo "Error: bag directory not found: ${BAG_DIR}"
        exit 1
    fi
fi

# Create replay output directory (mirrors raw structure)
BAG_NAME=$(basename "$BAG_DIR")
REPLAY_DIR="${WS_ROOT}/bags/replay/${BAG_NAME}"
mkdir -p "$REPLAY_DIR"

echo "===== Mapping Bag Replay ====="
echo "Input:    $BAG_DIR"
echo "Output:   $REPLAY_DIR"
echo "Backend:  $BACKEND"
echo "Rate:     ${RATE}x"
echo ""

case "$BACKEND" in
    liorf)
        IMU_MODE="six_axis"
        source "${WS_ROOT}/scripts/liorf_env.sh"
        ;;
    lio_sam)
        IMU_MODE="use_quaternion"
        source "${WS_ROOT}/scripts/lio_sam_env.sh"
        ;;
    *)
        echo "Unknown backend: $BACKEND"
        exit 1
        ;;
esac

TOPIC_SELECTOR="${WS_ROOT}/scripts/select_mapping_bag_topics.py"
DEFAULT_PACKET_PLAY_QOS_OVERRIDES="${WS_ROOT}/config/rosbag2/lio_sam_packet_play_qos.yaml"
if [[ ! -f "${TOPIC_SELECTOR}" ]]; then
    echo "Error: missing topic selector: ${TOPIC_SELECTOR}"
    exit 1
fi

IFS=$'\t' read -r RESOLVED_POINTCLOUD_TOPIC RESOLVED_POINTCLOUD_COUNT POINTCLOUD_REASON RESOLVED_IMU_TOPIC RESOLVED_IMU_COUNT IMU_REASON RESOLVED_PACKET_TOPIC RESOLVED_PACKET_COUNT PACKET_REASON < <(
    python3 "${TOPIC_SELECTOR}" \
        "${BAG_DIR}" \
        --backend "${BACKEND}" \
        --pointcloud-topic "${POINTCLOUD_TOPIC}" \
        --imu-topic "${IMU_INPUT_TOPIC}" \
        --packet-topic "${PACKET_TOPIC}" \
        --format tsv
)

CONVERT_PACKETS_TO_POINTCLOUD="false"
PACKET_TO_POINTCLOUD_NOTE=""

if [[ -z "${RESOLVED_POINTCLOUD_TOPIC}" || "${RESOLVED_POINTCLOUD_COUNT}" == "0" ]]; then
    if [[ "${BACKEND}" == "lio_sam" && -n "${RESOLVED_PACKET_TOPIC}" && "${RESOLVED_PACKET_COUNT}" != "0" ]]; then
        CONVERT_PACKETS_TO_POINTCLOUD="true"
        RESOLVED_POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/velodyne_points}"
        RESOLVED_POINTCLOUD_COUNT="${RESOLVED_PACKET_COUNT}"
        POINTCLOUD_REASON="generated_from_packets"
        PACKET_TO_POINTCLOUD_NOTE="Generating ${RESOLVED_POINTCLOUD_TOPIC} from ${RESOLVED_PACKET_TOPIC} for replay."
        if [[ -f "${DEFAULT_PACKET_PLAY_QOS_OVERRIDES}" ]]; then
            PLAY_QOS_OVERRIDES="${DEFAULT_PACKET_PLAY_QOS_OVERRIDES}"
        fi
    else
        echo "Error: no usable PointCloud2 topic found in ${BAG_DIR}"
        exit 1
    fi
fi

if [[ -z "${RESOLVED_IMU_TOPIC}" || "${RESOLVED_IMU_COUNT}" == "0" ]]; then
    echo "Error: no usable IMU topic found in ${BAG_DIR}"
    exit 1
fi

echo "Step 1: Inspect bag..."
"${WS_ROOT}/scripts/inspect_bag_fields.sh" "$BAG_DIR"
echo ""
echo "Resolved replay topics:"
echo "  PointCloud: ${RESOLVED_POINTCLOUD_TOPIC} (${RESOLVED_POINTCLOUD_COUNT} msgs; ${POINTCLOUD_REASON})"
echo "  IMU:        ${RESOLVED_IMU_TOPIC} (${RESOLVED_IMU_COUNT} msgs; ${IMU_REASON})"
if [[ -n "${RESOLVED_PACKET_TOPIC}" ]]; then
    echo "  Packets:    ${RESOLVED_PACKET_TOPIC} (${RESOLVED_PACKET_COUNT} msgs; ${PACKET_REASON})"
fi
if [[ "${RESOLVED_POINTCLOUD_TOPIC}" != "/velodyne_points" ]]; then
    echo "  NOTE: Falling back from /velodyne_points because it is empty or unavailable in this bag."
fi
if [[ -n "${PACKET_TO_POINTCLOUD_NOTE}" ]]; then
    echo "  NOTE: ${PACKET_TO_POINTCLOUD_NOTE}"
fi
if [[ -n "${PLAY_QOS_OVERRIDES}" ]]; then
    echo "  NOTE: Using playback QoS overrides from ${PLAY_QOS_OVERRIDES}"
fi
echo ""

echo "Step 2: Launch ${BACKEND} in background..."
case "$BACKEND" in
    liorf)
        ros2 launch spot_liorf_bringup liorf_replay.launch.py \
            imu_mode:=six_axis \
            pointcloud_topic:="${RESOLVED_POINTCLOUD_TOPIC}" \
            imu_input_topic:="${RESOLVED_IMU_TOPIC}" \
            use_sim_time:=true &
        LAUNCH_PID=$!
        ;;
    lio_sam)
        ros2 launch lio_sam_bringup lio_sam_replay.launch.py \
            imu_mode:=use_quaternion \
            pointcloud_topic:="${RESOLVED_POINTCLOUD_TOPIC}" \
            imu_input_topic:="${RESOLVED_IMU_TOPIC}" \
            packet_topic:="${RESOLVED_PACKET_TOPIC}" \
            convert_packets_to_pointcloud:="${CONVERT_PACKETS_TO_POINTCLOUD}" \
            scan_output_topic:=/scan \
            use_sim_time:=true &
        LAUNCH_PID=$!
        ;;
    *)
        echo "Unknown backend: $BACKEND"
        exit 1
        ;;
esac

# Wait for nodes to start
sleep 5

echo "Step 3: Play bag..."
ROS2_BAG_PLAY_CMD=(
    ros2 bag play "$BAG_DIR"
    --clock
    --rate "$RATE"
    --read-ahead-queue-size 1000
)

if [[ -n "${PLAY_QOS_OVERRIDES}" ]]; then
    ROS2_BAG_PLAY_CMD+=(--qos-profile-overrides-path "${PLAY_QOS_OVERRIDES}")
fi

"${ROS2_BAG_PLAY_CMD[@]}"

echo ""
echo "Bag replay complete. Waiting for mapping to finish..."
sleep 10

# Stop the launch
kill $LAUNCH_PID 2>/dev/null || true
wait $LAUNCH_PID 2>/dev/null || true

echo ""
echo "===== Replay Complete ====="
echo "Check maps in: maps/${BACKEND}/raw/"
echo "Replay metadata: $REPLAY_DIR"

# Save replay metadata
cat > "${REPLAY_DIR}/replay_metadata.yaml" <<METAEOF
source_bag: ${BAG_DIR}
backend: ${BACKEND}
rate: ${RATE}
timestamp: $(date -Iseconds)
extrinsics: ${WS_ROOT}/config/mapping/extrinsics.yaml
imu_mode: ${IMU_MODE}
pointcloud_topic: ${RESOLVED_POINTCLOUD_TOPIC}
pointcloud_message_count: ${RESOLVED_POINTCLOUD_COUNT}
imu_input_topic: ${RESOLVED_IMU_TOPIC}
imu_message_count: ${RESOLVED_IMU_COUNT}
packet_topic: ${RESOLVED_PACKET_TOPIC}
packet_message_count: ${RESOLVED_PACKET_COUNT}
convert_packets_to_pointcloud: ${CONVERT_PACKETS_TO_POINTCLOUD}
play_qos_overrides: ${PLAY_QOS_OVERRIDES}
METAEOF

echo "Saved replay metadata to: ${REPLAY_DIR}/replay_metadata.yaml"
