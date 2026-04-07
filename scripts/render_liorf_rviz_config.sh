#!/usr/bin/env bash
set -euo pipefail

source_config="${LIORF_RVIZ_SOURCE_CONFIG:-$HOME/nav_ws/src/LI_ORF/rviz/mapping.rviz}"
tmp_dir="${TMPDIR:-/tmp}"
user_name="${USER:-}"
if [ -z "$user_name" ]; then
    user_name="$(id -un 2>/dev/null || echo ros)"
fi
output_config="${LIORF_RVIZ_OUTPUT_CONFIG:-${tmp_dir}/liorf_mapping_${user_name}.rviz}"

path_topic="${LIORF_RVIZ_PATH_TOPIC:-/liorf/mapping/path}"
scan_topic="${LIORF_RVIZ_SCAN_TOPIC:-/liorf/mapping/cloud_registered_raw}"
map_topic="${LIORF_RVIZ_MAP_TOPIC:-/liorf/mapping/map_global}"
odom_topic="${LIORF_RVIZ_ODOM_TOPIC:-/liorf/mapping/odometry}"
fixed_frame="${LIORF_RVIZ_FIXED_FRAME:-odom}"

if [ ! -f "$source_config" ]; then
    echo "RViz source config not found: $source_config" >&2
    exit 1
fi

escape_sed_replacement() {
    printf '%s' "$1" | sed -e 's/[&|]/\\&/g'
}

escaped_path_topic="$(escape_sed_replacement "$path_topic")"
escaped_scan_topic="$(escape_sed_replacement "$scan_topic")"
escaped_map_topic="$(escape_sed_replacement "$map_topic")"
escaped_odom_topic="$(escape_sed_replacement "$odom_topic")"
escaped_fixed_frame="$(escape_sed_replacement "$fixed_frame")"

sed \
    -e "s|Value: /liorf/mapping/path|Value: ${escaped_path_topic}|g" \
    -e "s|Value: /liorf/mapping/cloud_registered_raw|Value: ${escaped_scan_topic}|g" \
    -e "s|Value: /liorf/mapping/map_global|Value: ${escaped_map_topic}|g" \
    -e "s|Value: /liorf/mapping/odometry|Value: ${escaped_odom_topic}|g" \
    -e "s|Fixed Frame: odom|Fixed Frame: ${escaped_fixed_frame}|g" \
    "$source_config" > "$output_config"

printf '%s\n' "$output_config"
