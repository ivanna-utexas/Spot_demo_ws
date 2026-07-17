#!/usr/bin/env bash
# record_lio_sam_raw_bag.sh — Record the minimum raw topics needed for
# later offline LIO-SAM replay + acquisition debugging.
#
# Usage:
#   ./scripts/record_lio_sam_raw_bag.sh [--name run1] [--estimate-gb 10] \
#       [--qos-overrides path/to/qos.yaml]
set -eo pipefail

NAME=""
ESTIMATE_GB=10
WS_ROOT="${HOME}/dance_ws_pedestrian_tracking"
REQUIRE_TEGRASTATS="${LIO_SAM_REQUIRE_TEGRASTATS:-0}"
QOS_OVERRIDES="${WS_ROOT}/config/rosbag2/lio_sam_capture_debug_qos.yaml"
BAG_TOPICS=(
    /velodyne_packets
    /vectornav/imu
)

# Source ROS env with nounset disabled (Humble setup scripts expect this).
set +u
source "${WS_ROOT}/scripts/lio_sam_env.sh"
set -u

while [[ $# -gt 0 ]]; do
    case "$1" in
        --name) NAME="$2"; shift 2 ;;
        --estimate-gb) ESTIMATE_GB="$2"; shift 2 ;;
        --qos-overrides) QOS_OVERRIDES="$2"; shift 2 ;;
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
MONITOR_LOG="${BAG_PATH}.tegrastats.log"
SYSTEM_LOG="${BAG_PATH}.system.log"
MONITOR_PID=""
MONITOR_KIND=""

have_passwordless_sudo() {
    command -v sudo >/dev/null 2>&1 && sudo -n true >/dev/null 2>&1
}

run_command_logged() {
    local label="$1"
    shift

    {
        printf "## %s\n" "${label}"
        printf "# timestamp %s\n" "$(date --iso-8601=seconds)"
        printf "$ %s\n" "$*"
    } >> "${SYSTEM_LOG}"

    if "$@" >> "${SYSTEM_LOG}" 2>&1; then
        printf "\n" >> "${SYSTEM_LOG}"
        return 0
    fi

    if have_passwordless_sudo; then
        printf "# retrying_with sudo -n\n" >> "${SYSTEM_LOG}"
        if sudo -n "$@" >> "${SYSTEM_LOG}" 2>&1; then
            printf "\n" >> "${SYSTEM_LOG}"
            return 0
        fi
    fi

    printf "# command_unavailable_or_failed\n\n" >> "${SYSTEM_LOG}"
    return 1
}

capture_power_snapshot() {
    local phase="$1"

    {
        printf "# power_snapshot %s %s\n" "${phase}" "$(date --iso-8601=seconds)"
    } >> "${SYSTEM_LOG}"

    run_command_logged "nvpmodel (${phase})" /usr/sbin/nvpmodel -q || true
    run_command_logged "jetson_clocks (${phase})" /usr/bin/jetson_clocks --show || true
}

cleanup() {
    if [[ -n "${MONITOR_PID}" ]] && kill -0 "${MONITOR_PID}" 2>/dev/null; then
        echo ""
        echo "Stopping ${MONITOR_KIND:-monitor} (pid ${MONITOR_PID})..."
        kill "${MONITOR_PID}" 2>/dev/null || true
        wait "${MONITOR_PID}" 2>/dev/null || true
        printf "# monitor_stop %s %s\n" "$(date --iso-8601=seconds)" "${MONITOR_KIND:-unknown}" >> "${MONITOR_LOG}" || true
    fi
    capture_power_snapshot stop || true
}
trap cleanup EXIT

start_tegrastats() {
    local tegra_bin=""
    if command -v tegrastats >/dev/null 2>&1; then
        tegra_bin="$(command -v tegrastats)"
    elif [[ -x /usr/bin/tegrastats ]]; then
        tegra_bin="/usr/bin/tegrastats"
    fi

    if [[ -z "${tegra_bin}" ]]; then
        echo "WARNING: tegrastats not found; continuing without tegrastats logging."
        echo "Expected tegrastats in PATH or at /usr/bin/tegrastats."
        echo "This usually means the recorder is running inside Docker without tegrastats exposed."
        if [[ "${REQUIRE_TEGRASTATS}" == "1" ]]; then
            echo "Set LIO_SAM_REQUIRE_TEGRASTATS=0 to allow recording without tegrastats."
            exit 1
        fi
        return 1
    fi

    printf "# monitor_source tegrastats\n# monitor_start %s\n" "$(date --iso-8601=seconds)" > "${MONITOR_LOG}"
    echo "Starting tegrastats logging to: ${MONITOR_LOG}"
    if have_passwordless_sudo; then
        sudo -n "${tegra_bin}" --interval 1000 >> "${MONITOR_LOG}" 2>&1 &
        MONITOR_PID=$!
        MONITOR_KIND="tegrastats (sudo)"
        return 0
    fi

    "${tegra_bin}" --interval 1000 >> "${MONITOR_LOG}" 2>&1 &
    MONITOR_PID=$!
    MONITOR_KIND="tegrastats"
    return 0
}

start_vmstat() {
    if ! command -v vmstat >/dev/null 2>&1; then
        return 1
    fi

    printf "# monitor_source vmstat\n# monitor_start %s\n" "$(date --iso-8601=seconds)" > "${MONITOR_LOG}"
    echo "Starting vmstat logging to: ${MONITOR_LOG}"
    {
        printf "# free_mib_snapshot %s\n" "$(date --iso-8601=seconds)"
        free -m
        printf "# vmstat_stream columns follow (see vmstat -w -t)\n"
        vmstat -w -t 1
    } >> "${MONITOR_LOG}" 2>&1 &
    MONITOR_PID=$!
    MONITOR_KIND="vmstat"
    return 0
}

start_monitor() {
    if start_tegrastats; then
        return 0
    fi
    if start_vmstat; then
        return 0
    fi

    printf "# monitor_unavailable %s\n" "$(date --iso-8601=seconds)" > "${MONITOR_LOG}" || true
    echo "WARNING: no system monitor available; continuing without telemetry logging."
    return 0
}

echo "===== Minimal LIO-SAM Raw Bag Recording ====="
echo "Output: $BAG_PATH"
echo "Topics:"
for topic in "${BAG_TOPICS[@]}"; do
    echo "  - ${topic}"
done
echo "Sidecar logs:"
echo "  - ${MONITOR_LOG}"
echo "  - ${SYSTEM_LOG}"
if [[ -f "${QOS_OVERRIDES}" ]]; then
    echo "QoS overrides:"
    echo "  - ${QOS_OVERRIDES}"
else
    echo "QoS overrides:"
    echo "  - ${QOS_OVERRIDES} (missing; rosbag will use defaults)"
fi
echo ""

"${WS_ROOT}/scripts/check_storage_budget.sh" \
    --estimate-gb "$ESTIMATE_GB" \
    --path "${WS_ROOT}/bags/raw"

echo ""
echo "Starting recording to: $BAG_PATH"
echo "Press Ctrl+C to stop."
echo ""

printf "# system_log_start %s\n" "$(date --iso-8601=seconds)" > "${SYSTEM_LOG}"
capture_power_snapshot start
start_monitor

ROS2_BAG_CMD=(
    ros2 bag record
)

ROS2_BAG_CMD+=("${BAG_TOPICS[@]}")

ROS2_BAG_CMD+=(
    -o "$BAG_PATH"
    --max-bag-duration 300
    -s sqlite3
)

if [[ -f "${QOS_OVERRIDES}" ]]; then
    ROS2_BAG_CMD+=(--qos-profile-overrides-path "${QOS_OVERRIDES}")
fi

"${ROS2_BAG_CMD[@]}"

echo ""
echo "Recording saved to: $BAG_PATH"
echo "monitor log saved to: ${MONITOR_LOG}"
echo "system log saved to: ${SYSTEM_LOG}"
echo "Inspect with: ./scripts/inspect_bag_fields.sh $BAG_PATH"
