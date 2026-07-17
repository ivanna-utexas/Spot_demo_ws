#!/usr/bin/env bash
# lio_sam_env.sh — Candidate-specific environment for the ROS 2 LIO-SAM flow.
set -euo pipefail

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

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${WS_ROOT}/config/cyclonedds.xml"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset FASTRTPS_DEFAULT_PROFILES_FILE 2>/dev/null || true
unset ROS_DISCOVERY_SERVER 2>/dev/null || true

source_setup_file /opt/ros/humble/setup.bash
source_setup_file "${WS_ROOT}/install/setup.bash"

export LIO_SAM_MAP_DIR="${WS_ROOT}/maps/lio_sam"
mkdir -p "${LIO_SAM_MAP_DIR}/raw" "${LIO_SAM_MAP_DIR}/aligned" "${LIO_SAM_MAP_DIR}/cleaned" "${LIO_SAM_MAP_DIR}/projected"

export MAPPING_EXTRINSICS_FILE="${WS_ROOT}/config/mapping/extrinsics.yaml"
export MAPPING_BAG_RAW_DIR="${WS_ROOT}/bags/raw"
export MAPPING_BAG_REPLAY_DIR="${WS_ROOT}/bags/replay"
mkdir -p "${MAPPING_BAG_RAW_DIR}" "${MAPPING_BAG_REPLAY_DIR}"

echo "[lio_sam_env] DDS=CycloneDDS, WS=${WS_ROOT}, extrinsics=${MAPPING_EXTRINSICS_FILE}"
