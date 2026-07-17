#!/usr/bin/env bash
# check_storage_budget.sh — Verify sufficient disk space before recording.
#
# Rule: free space >= max(75 GB, 3x estimated run size)
# Logs the estimate before capture begins.
#
# Usage: ./scripts/check_storage_budget.sh [--estimate-gb 25] [--path ~/dance_ws_pedestrian_tracking/bags/raw]
set -euo pipefail

RED='\033[0;31m'; GRN='\033[0;32m'; YEL='\033[0;33m'; RST='\033[0m'

ESTIMATE_GB=25  # default estimated run size
TARGET_PATH="${HOME}/dance_ws_pedestrian_tracking/bags/raw"

while [[ $# -gt 0 ]]; do
    case $1 in
        --estimate-gb) ESTIMATE_GB="$2"; shift 2 ;;
        --path) TARGET_PATH="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

mkdir -p "$TARGET_PATH"

AVAIL_GB=$(df -BG --output=avail "$TARGET_PATH" 2>/dev/null | tail -1 | tr -d ' G')
MIN_3X=$((ESTIMATE_GB * 3))
REQUIRED=$((MIN_3X > 75 ? MIN_3X : 75))

echo "===== Storage Budget Check ====="
echo "Target path:      $TARGET_PATH"
echo "Available:         ${AVAIL_GB} GB"
echo "Estimated run:     ${ESTIMATE_GB} GB"
echo "Required (3x):     ${MIN_3X} GB"
echo "Required (min 75): 75 GB"
echo "Threshold:         ${REQUIRED} GB"
echo ""

if [[ "${AVAIL_GB:-0}" -ge "$REQUIRED" ]]; then
    echo -e "${GRN}Storage OK: ${AVAIL_GB} GB >= ${REQUIRED} GB${RST}"
    exit 0
else
    echo -e "${RED}INSUFFICIENT STORAGE: ${AVAIL_GB} GB < ${REQUIRED} GB${RST}"
    echo "Recording refused. Free up space or reduce estimate."
    exit 1
fi
