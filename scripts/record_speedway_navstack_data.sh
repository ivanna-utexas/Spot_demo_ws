#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAGS_ROOT="${WS_ROOT}/bags/speedway_data_collection"

DATA_LABEL="Data"
REC_PID=""
CURRENT_BAG_PATH=""
CURRENT_LOG_PATH=""
DELETE_CURRENT_BAG=0

TOPICS=()
EXTRA_TOPICS=()
DEFAULT_TOPICS=(
    /velodyne_points
    /velodyne_packets
    /rgb/image_raw
    /rgb/camera_info
    /tf
    /tf_static
    /map
    /detection_markers
    /amcl_pose
    /cmd_vel
    /joy
    /body_pose  
)

usage() {
    cat <<EOF
Usage:
  $(basename "$0") [--label LABEL] [--topic TOPIC] [--extra-topic TOPIC]

What it does:
  1. Runs as an interactive recorder pane inside tmux/navstack
  2. Records LiDAR + camera topics into bags/speedway_data_collection
  3. Names bags as Spot-Speedway-\${LABEL}-YYYY-MM-DD-subject_[i]
  4. Lets you type 'dump' to delete the current bag and reuse the same index

Options:
  --label LABEL       Bag name label after Spot-Speedway- (default: Data)
  --topic TOPIC       Replace the default topic list; repeatable
  --extra-topic TOPIC Append an extra topic to the default list; repeatable
  -h, --help          Show this help

Default topics:
  /velodyne_points
  /velodyne_packets
  /rgb/image_raw
  /rgb/camera_info
  /ir/image_raw
  /rgb_to_depth/image_raw
  /tf
  /tf_static
  /camera/back/image
  /camera/frontleft/image
  /camera/frontright/image
  /camera/left/image
  /camera/right/image
  /depth/image_raw
EOF
}

cleanup() {
    if [ -n "${REC_PID}" ] && kill -0 "${REC_PID}" 2>/dev/null; then
        echo
        echo "Stopping active recording before exit..."
        stop_recording
        if [ "${DELETE_CURRENT_BAG}" -eq 1 ]; then
            delete_current_bag
        else
            echo "Kept partial recording at: ${CURRENT_BAG_PATH}"
        fi
    fi
}

trap cleanup EXIT

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "Missing required command: $1" >&2
        exit 1
    fi
}

source_ros_env() {
    if [ ! -f "${WS_ROOT}/scripts/lio_sam_env.sh" ]; then
        echo "Missing ROS environment helper: ${WS_ROOT}/scripts/lio_sam_env.sh" >&2
        exit 1
    fi

    case $- in
        *u*)
            set +u
            # shellcheck disable=SC1090
            source "${WS_ROOT}/scripts/lio_sam_env.sh"
            set -u
            ;;
        *)
            # shellcheck disable=SC1090
            source "${WS_ROOT}/scripts/lio_sam_env.sh"
            ;;
    esac
}

validate_label() {
    if [[ ! "$1" =~ ^[A-Za-z0-9_-]+$ ]]; then
        echo "Invalid label '${1}'. Use only letters, numbers, '-' or '_'." >&2
        exit 1
    fi
}

build_topic_list() {
    if [ "${#TOPICS[@]}" -eq 0 ]; then
        TOPICS=("${DEFAULT_TOPICS[@]}")
    fi

    if [ "${#EXTRA_TOPICS[@]}" -gt 0 ]; then
        TOPICS+=("${EXTRA_TOPICS[@]}")
    fi
}

bag_name_for_index() {
    local index="$1"
    local date_stamp

    date_stamp="$(date +%Y-%m-%d)"
    printf 'Spot-Speedway-%s-%s-subject_%s' "${DATA_LABEL}" "${date_stamp}" "${index}"
}

find_next_index() {
    local date_stamp prefix candidate base suffix max_index

    date_stamp="$(date +%Y-%m-%d)"
    prefix="Spot-Speedway-${DATA_LABEL}-${date_stamp}-subject_"
    max_index=-1

    if [ ! -d "${BAGS_ROOT}" ]; then
        echo "0"
        return
    fi

    for candidate in "${BAGS_ROOT}/${prefix}"*; do
        if [ ! -d "${candidate}" ]; then
            continue
        fi

        base="$(basename "${candidate}")"
        suffix="${base#${prefix}}"
        if [[ "${suffix}" =~ ^[0-9]+$ ]] && [ "${suffix}" -gt "${max_index}" ]; then
            max_index="${suffix}"
        fi
    done

    echo "$((max_index + 1))"
}

start_recording() {
    local bag_name="$1"

    CURRENT_BAG_PATH="${BAGS_ROOT}/${bag_name}"
    CURRENT_LOG_PATH="${BAGS_ROOT}/${bag_name}.record.log"
    DELETE_CURRENT_BAG=0

    mkdir -p "${BAGS_ROOT}"
    rm -f "${CURRENT_LOG_PATH}"

    echo
    echo "Starting recording:"
    echo "  Bag: ${CURRENT_BAG_PATH}"
    echo "  Log: ${CURRENT_LOG_PATH}"
    echo "  Topics:"
    printf '    %s\n' "${TOPICS[@]}"
    echo
    echo "Commands while recording: keep | dump | status | quit"
    echo "  keep   stop and keep the bag"
    echo "  dump   stop and delete the bag"
    echo "  quit   stop, keep the bag, and exit"
    echo

    ros2 bag record -o "${CURRENT_BAG_PATH}" "${TOPICS[@]}" > "${CURRENT_LOG_PATH}" 2>&1 &
    REC_PID="$!"

    sleep 2
    if ! kill -0 "${REC_PID}" 2>/dev/null; then
        wait "${REC_PID}" 2>/dev/null || true
        REC_PID=""
        echo "ros2 bag record exited immediately. See log: ${CURRENT_LOG_PATH}" >&2
        sed -n '1,120p' "${CURRENT_LOG_PATH}" >&2 || true
        exit 1
    fi
}

stop_recording() {
    local tries

    if [ -z "${REC_PID}" ]; then
        return
    fi

    if kill -0 "${REC_PID}" 2>/dev/null; then
        kill -INT "${REC_PID}" 2>/dev/null || true
    fi

    tries=0
    while kill -0 "${REC_PID}" 2>/dev/null; do
        tries=$((tries + 1))
        if [ "${tries}" -ge 50 ]; then
            break
        fi
        sleep 0.2
    done

    if kill -0 "${REC_PID}" 2>/dev/null; then
        kill -TERM "${REC_PID}" 2>/dev/null || true
        tries=0
        while kill -0 "${REC_PID}" 2>/dev/null; do
            tries=$((tries + 1))
            if [ "${tries}" -ge 25 ]; then
                break
            fi
            sleep 0.2
        done
    fi

    if kill -0 "${REC_PID}" 2>/dev/null; then
        kill -KILL "${REC_PID}" 2>/dev/null || true
    fi

    wait "${REC_PID}" 2>/dev/null || true
    REC_PID=""
}

delete_current_bag() {
    if [ -n "${CURRENT_BAG_PATH}" ]; then
        rm -rf "${CURRENT_BAG_PATH}"
    fi

    if [ -n "${CURRENT_LOG_PATH}" ] && [ -f "${CURRENT_LOG_PATH}" ]; then
        rm -f "${CURRENT_LOG_PATH}"
    fi
}

record_one_session() {
    local index="$1"
    local bag_name="$2"
    local command_input

    start_recording "${bag_name}"

    while true; do
        printf '[keep|dump|status|quit] > '
        IFS= read -r command_input

        case "${command_input}" in
            ""|keep|stop)
                stop_recording
                echo "Saved recording: ${CURRENT_BAG_PATH}"
                return
                ;;
            dump)
                DELETE_CURRENT_BAG=1
                stop_recording
                delete_current_bag
                DELETE_CURRENT_BAG=0
                echo "Discarded recording. The next session will reuse subject_${index}."
                return
                ;;
            status)
                echo "Recording: ${CURRENT_BAG_PATH}"
                echo "Log: ${CURRENT_LOG_PATH}"
                ;;
            quit)
                stop_recording
                echo "Saved recording: ${CURRENT_BAG_PATH}"
                exit 0
                ;;
            *)
                echo "Unknown command: ${command_input}"
                ;;
        esac
    done
}

prompt_for_next_action() {
    local next_index next_bag command_input

    while true; do
        next_index="$(find_next_index)"
        next_bag="$(bag_name_for_index "${next_index}")"

        echo
        echo "Next recording target: ${next_bag}"
        printf '[start|quit] > '
        IFS= read -r command_input

        case "${command_input}" in
            ""|start)
                record_one_session "${next_index}" "${next_bag}"
                return
                ;;
            quit)
                exit 0
                ;;
            *)
                echo "Unknown command: ${command_input}"
                ;;
        esac
    done
}

start_next_session() {
    local next_index next_bag

    next_index="$(find_next_index)"
    next_bag="$(bag_name_for_index "${next_index}")"
    record_one_session "${next_index}" "${next_bag}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --label)
            DATA_LABEL="$2"
            shift 2
            ;;
        --topic)
            TOPICS+=("$2")
            shift 2
            ;;
        --extra-topic)
            EXTRA_TOPICS+=("$2")
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage
            exit 1
            ;;
    esac
done

validate_label "${DATA_LABEL}"
build_topic_list

source_ros_env
require_cmd ros2

echo
echo "Recorder pane ready."
echo "Workspace: ${WS_ROOT}"
echo "Output root: ${BAGS_ROOT}"
echo "The first recording starts immediately."
start_next_session

while true; do
    prompt_for_next_action
done
