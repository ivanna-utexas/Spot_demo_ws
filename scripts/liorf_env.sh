#!/usr/bin/env bash
# liorf_env.sh — Source this in liorf mapping tmux panes to override the
# global DDS env from bash_profile.shared and ensure CycloneDDS is correct.
#
# Usage (in tmuxinator pane):
#   source ~/dance_ws_pedestrian_tracking/scripts/liorf_env.sh && <command>
#
# This file is candidate-specific: it sets env vars needed by liorf.
# Backend-neutral env (DDS, workspace) is also re-asserted here so that
# the liorf session doesn't inherit stale or wrong values.
WS_ROOT="${HOME}/dance_ws_pedestrian_tracking"

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

# ---- DDS: Force CycloneDDS (the broken global env may set FastDDS or
#      wrong CYCLONEDDS_URI after docker bash_profile runs) ----
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${WS_ROOT}/config/cyclonedds.xml"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset FASTRTPS_DEFAULT_PROFILES_FILE 2>/dev/null || true
unset ROS_DISCOVERY_SERVER 2>/dev/null || true

# ---- ROS workspace ----
source_setup_file /opt/ros/humble/setup.bash
source_setup_file "${WS_ROOT}/install/setup.bash"

# ---- liorf-specific ----
# Output directory for liorf maps
export LIORF_MAP_DIR="${WS_ROOT}/maps/liorf"
mkdir -p "${LIORF_MAP_DIR}/raw" "${LIORF_MAP_DIR}/aligned" "${LIORF_MAP_DIR}/cleaned" "${LIORF_MAP_DIR}/projected"

# Extrinsics: point all liorf launch files at the single canonical calibration
export MAPPING_EXTRINSICS_FILE="${WS_ROOT}/config/mapping/extrinsics.yaml"

# Bags
export MAPPING_BAG_RAW_DIR="${WS_ROOT}/bags/raw"
export MAPPING_BAG_REPLAY_DIR="${WS_ROOT}/bags/replay"
mkdir -p "${MAPPING_BAG_RAW_DIR}" "${MAPPING_BAG_REPLAY_DIR}"

echo "[liorf_env] DDS=CycloneDDS, WS=${WS_ROOT}, extrinsics=${MAPPING_EXTRINSICS_FILE}"
