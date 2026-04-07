#!/usr/bin/env bash
# measure_lidar_imu_skew.sh — Measure timestamp skew between lidar and IMU.
#
# Compares arrival-time differences between /velodyne_points and /imu/data
# to estimate the lidar-IMU time offset.
#
# Usage: ./scripts/measure_lidar_imu_skew.sh [--duration 30]
# Acceptance: median < 5ms, 99th percentile < 20ms
set -euo pipefail

DURATION=30

while [[ $# -gt 0 ]]; do
    case $1 in
        --duration) DURATION="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

echo "===== Lidar-IMU Timestamp Skew Measurement ====="
echo "Duration: ${DURATION}s"
echo ""

python3 - "$DURATION" <<'PYEOF'
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu
import time
import statistics

duration = float(sys.argv[1])

class SkewMeasurer(Node):
    def __init__(self):
        super().__init__('skew_measurer')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        self.sub_pc = self.create_subscription(
            PointCloud2, '/velodyne_points', self.pc_cb, qos)
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_cb, qos)

        self.last_pc_stamp_ns = None
        self.last_imu_stamp_ns = None
        self.skews_ms = []
        self.start = time.monotonic()
        self.get_logger().info(f'Measuring skew for {duration}s...')

    def stamp_to_ns(self, stamp):
        return stamp.sec * 10**9 + stamp.nanosec

    def pc_cb(self, msg):
        self.last_pc_stamp_ns = self.stamp_to_ns(msg.header.stamp)
        self._compute()

    def imu_cb(self, msg):
        self.last_imu_stamp_ns = self.stamp_to_ns(msg.header.stamp)

    def _compute(self):
        if time.monotonic() - self.start > duration:
            self.report()
            raise SystemExit(0)

        if self.last_pc_stamp_ns is None or self.last_imu_stamp_ns is None:
            return

        # For each PC message, find the time difference to the closest IMU stamp
        skew_ms = (self.last_pc_stamp_ns - self.last_imu_stamp_ns) / 1e6
        self.skews_ms.append(skew_ms)

    def report(self):
        n = len(self.skews_ms)
        if n < 10:
            self.get_logger().error(f'Only {n} skew samples — insufficient')
            return

        abs_skews = [abs(s) for s in self.skews_ms]
        sorted_abs = sorted(abs_skews)
        median = statistics.median(abs_skews)
        p99_idx = int(0.99 * len(sorted_abs))
        p99 = sorted_abs[min(p99_idx, len(sorted_abs) - 1)]
        mean_skew = statistics.mean(self.skews_ms)

        print(f'\n===== Lidar-IMU Skew Results =====')
        print(f'Samples:        {n}')
        print(f'Mean skew:      {mean_skew:.2f} ms (positive = IMU leads)')
        print(f'Median |skew|:  {median:.2f} ms')
        print(f'99th %ile:      {p99:.2f} ms')
        print(f'Min |skew|:     {sorted_abs[0]:.2f} ms')
        print(f'Max |skew|:     {sorted_abs[-1]:.2f} ms')
        print(f'')
        median_ok = median < 5.0
        p99_ok = p99 < 20.0
        print(f'Gate (median < 5ms):   {"PASS" if median_ok else "FAIL"} ({median:.2f}ms)')
        print(f'Gate (99th < 20ms):    {"PASS" if p99_ok else "FAIL"} ({p99:.2f}ms)')
        print(f'')
        print(f'Suggested time_offset_lidar_imu: {mean_skew / 1000:.6f} s')

rclpy.init()
node = SkewMeasurer()
try:
    rclpy.spin(node)
except SystemExit:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PYEOF
