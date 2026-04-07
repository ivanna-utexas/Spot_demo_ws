#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="${HOME}/nav_ws"
SAVE_RESOLUTION="0.0"
GRID_RESOLUTION="0.05"
HEIGHT_MIN="-0.10"
HEIGHT_MAX="0.50"
SKIP_CLEAN="0"
LAUNCH_NAVSTACK="0"

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

usage() {
    cat <<'EOF'
Usage:
  scripts/export_lio_sam_map_to_nav.sh <map-name> [options]

Options:
  --save-resolution <m>  Downsample LIO-SAM save output before writing GlobalMap.pcd
  --grid-resolution <m>  2D occupancy grid resolution passed to project_2d_map.py
  --height-min <m>       Minimum height for the 2D slice
  --height-max <m>       Maximum height for the 2D slice
  --skip-clean           Project GlobalMap.pcd directly instead of running clean_map.py
  --launch-navstack      Start tmux/navstack after activating the exported map
EOF
}

if [ $# -lt 1 ]; then
    usage
    exit 1
fi

MAP_NAME=""

while [ $# -gt 0 ]; do
    case "$1" in
        --save-resolution)
            SAVE_RESOLUTION="$2"
            shift 2
            ;;
        --grid-resolution)
            GRID_RESOLUTION="$2"
            shift 2
            ;;
        --height-min)
            HEIGHT_MIN="$2"
            shift 2
            ;;
        --height-max)
            HEIGHT_MAX="$2"
            shift 2
            ;;
        --skip-clean)
            SKIP_CLEAN="1"
            shift
            ;;
        --launch-navstack)
            LAUNCH_NAVSTACK="1"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            echo "Unknown option: $1" >&2
            usage
            exit 1
            ;;
        *)
            if [ -n "${MAP_NAME}" ]; then
                echo "Only one map name is supported. Got extra argument: $1" >&2
                exit 1
            fi
            MAP_NAME="$1"
            shift
            ;;
    esac
done

if [ -z "${MAP_NAME}" ]; then
    echo "Map name is required." >&2
    usage
    exit 1
fi

source_setup_file /opt/ros/humble/setup.bash
source_setup_file "${WS_ROOT}/install/setup.bash"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 CLI not found after sourcing the workspace." >&2
    exit 1
fi

if ! ros2 service type /lio_sam/save_map >/dev/null 2>&1; then
    echo "/lio_sam/save_map is not available." >&2
    echo "Start LIO-SAM first, then rerun this script." >&2
    exit 1
fi

RAW_DIR="${WS_ROOT}/maps/lio_sam/raw/${MAP_NAME}"
ALIGNED_DIR="${WS_ROOT}/maps/lio_sam/aligned/${MAP_NAME}"
CLEANED_DIR="${WS_ROOT}/maps/lio_sam/cleaned/${MAP_NAME}"
PROJECTED_DIR="${WS_ROOT}/maps/lio_sam/projected"
FINAL_YAML="${WS_ROOT}/maps/final/${MAP_NAME}.yaml"

mkdir -p "${ALIGNED_DIR}" "${CLEANED_DIR}" "${PROJECTED_DIR}" "${WS_ROOT}/maps/final"

echo "Saving LIO-SAM map to ${RAW_DIR}"
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
  "{resolution: ${SAVE_RESOLUTION}, destination: '/nav_ws/maps/lio_sam/raw/${MAP_NAME}'}"

if [ ! -f "${RAW_DIR}/GlobalMap.pcd" ]; then
    echo "Expected LIO-SAM save output not found: ${RAW_DIR}/GlobalMap.pcd" >&2
    exit 1
fi

cp "${RAW_DIR}/GlobalMap.pcd" "${ALIGNED_DIR}/${MAP_NAME}_GlobalMap.pcd"

if [ "${SKIP_CLEAN}" = "1" ]; then
    input_cloud="${ALIGNED_DIR}/${MAP_NAME}_GlobalMap.pcd"
    echo "Skipping clean_map.py; projecting ${input_cloud} directly."
else
    echo "Cleaning 3D map..."
    python3 "${WS_ROOT}/scripts/clean_map.py" \
      --input "${ALIGNED_DIR}" \
      --output "${CLEANED_DIR}" \
      --min-passes 1
    input_cloud="${CLEANED_DIR}/cleaned_map.pcd"
fi

echo "Projecting to 2D occupancy grid..."
python3 "${WS_ROOT}/scripts/project_2d_map.py" \
  --input "${input_cloud}" \
  --output "${PROJECTED_DIR}" \
  --name "${MAP_NAME}" \
  --resolution "${GRID_RESOLUTION}" \
  --height-min "${HEIGHT_MIN}" \
  --height-max "${HEIGHT_MAX}" \
  --promote

if [ ! -f "${FINAL_YAML}" ]; then
    echo "Expected promoted nav map not found: ${FINAL_YAML}" >&2
    exit 1
fi

if [ "${LAUNCH_NAVSTACK}" = "1" ]; then
    "${WS_ROOT}/scripts/use_nav_map.sh" "${MAP_NAME}" --launch
else
    "${WS_ROOT}/scripts/use_nav_map.sh" "${MAP_NAME}"
fi

echo
echo "LIO-SAM export complete."
echo "Raw save:      ${RAW_DIR}"
echo "Projected map: ${WS_ROOT}/maps/final/${MAP_NAME}.yaml"
