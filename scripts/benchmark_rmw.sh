#!/usr/bin/env bash
# benchmark_rmw.sh — Diagnostic benchmark: CycloneDDS vs FastDDS latency.
#
# CycloneDDS remains the production middleware. FastDDS results are
# recorded for documentation only and are NOT adopted.
#
# Measures round-trip latency and throughput for PointCloud2-sized messages.
# Usage: ./scripts/benchmark_rmw.sh
set -euo pipefail

echo "===== RMW Middleware Benchmark (Diagnostic Only) ====="
echo "Production middleware: CycloneDDS (unchanged by this script)"
echo ""

WS_ROOT="${HOME}/dance_ws_pedestrian_tracking"

# Check which RMW implementations are available
echo "--- Available RMW implementations ---"
for rmw in rmw_cyclonedds_cpp rmw_fastrtps_cpp; do
    if dpkg -l "ros-humble-${rmw//_/-}" &>/dev/null; then
        echo "  [installed] $rmw"
    else
        echo "  [missing]   $rmw"
    fi
done
echo ""

# Benchmark function using ros2 topic delay and hz
benchmark_rmw() {
    local rmw=$1
    local duration=10

    echo "--- Benchmarking $rmw (${duration}s) ---"

    export RMW_IMPLEMENTATION=$rmw

    if [[ "$rmw" == "rmw_cyclonedds_cpp" ]]; then
        export CYCLONEDDS_URI="${WS_ROOT}/config/cyclonedds.xml"
    fi

    # Measure /velodyne_points rate and delay if topic is active
    if ros2 topic list 2>/dev/null | grep -q '/velodyne_points'; then
        echo "  Measuring /velodyne_points..."

        # Rate
        timeout ${duration} ros2 topic hz /velodyne_points --window 50 2>/dev/null | tail -5 || true
        echo ""

        # Delay (difference between header stamp and receive time)
        timeout ${duration} ros2 topic delay /velodyne_points --window 50 2>/dev/null | tail -5 || true
    else
        echo "  /velodyne_points not active — skipping live test"
        echo "  (Start velodyne driver first for live benchmarking)"
    fi
    echo ""
}

# Always benchmark CycloneDDS
benchmark_rmw rmw_cyclonedds_cpp

# Benchmark FastDDS only if installed
if dpkg -l ros-humble-rmw-fastrtps-cpp &>/dev/null; then
    benchmark_rmw rmw_fastrtps_cpp
else
    echo "--- FastDDS not installed — skipping ---"
fi

# Restore production middleware
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${WS_ROOT}/config/cyclonedds.xml"

echo ""
echo "===== Benchmark Complete ====="
echo "Production middleware remains: CycloneDDS"
echo "Record results in README_spot_liorf_mapping.md for documentation."
