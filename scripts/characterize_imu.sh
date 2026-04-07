#!/usr/bin/env bash
# characterize_imu.sh — Collect IMU statistics from Spot at rest.
#
# Measures:
#   - Accelerometer noise density and bias
#   - Gyroscope noise density and bias
#   - Gravity magnitude
#   - Effective publishing rate
#
# Usage: ./scripts/characterize_imu.sh [--topic /imu/data] [--duration 30]
# Requires: Spot powered on, IMU streaming, robot stationary on flat ground.
set -euo pipefail

TOPIC="/imu/data"
DURATION=30

while [[ $# -gt 0 ]]; do
    case $1 in
        --topic) TOPIC="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

echo "===== IMU Characterization ====="
echo "Topic: $TOPIC"
echo "Duration: ${DURATION}s"
echo "Ensure robot is STATIONARY on FLAT GROUND."
echo ""

python3 - "$TOPIC" "$DURATION" <<'PYEOF'
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import math
import time
import signal

topic = sys.argv[1]
duration = float(sys.argv[2])

class ImuCharacterizer(Node):
    def __init__(self):
        super().__init__('imu_characterizer')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=500
        )
        self.sub = self.create_subscription(Imu, topic, self.cb, qos)
        self.ax, self.ay, self.az = [], [], []
        self.gx, self.gy, self.gz = [], [], []
        self.stamps = []
        self.start = time.monotonic()
        self.get_logger().info(f'Collecting IMU data for {duration}s...')

    def cb(self, msg):
        if time.monotonic() - self.start > duration:
            self.report()
            raise SystemExit(0)

        self.ax.append(msg.linear_acceleration.x)
        self.ay.append(msg.linear_acceleration.y)
        self.az.append(msg.linear_acceleration.z)
        self.gx.append(msg.angular_velocity.x)
        self.gy.append(msg.angular_velocity.y)
        self.gz.append(msg.angular_velocity.z)
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.stamps.append(stamp_ns)

    def report(self):
        n = len(self.ax)
        if n < 100:
            self.get_logger().error(f'Only {n} samples — insufficient')
            return

        def mean(v): return sum(v) / len(v)
        def std(v):
            m = mean(v)
            return math.sqrt(sum((x - m)**2 for x in v) / (len(v) - 1))

        # Rate
        dt_total = (self.stamps[-1] - self.stamps[0]) / 1e9
        rate = (n - 1) / dt_total if dt_total > 0 else 0

        # Gravity
        grav = math.sqrt(mean(self.ax)**2 + mean(self.ay)**2 + mean(self.az)**2)

        # Noise density (std / sqrt(rate))
        sqrt_rate = math.sqrt(rate) if rate > 0 else 1
        acc_std_x, acc_std_y, acc_std_z = std(self.ax), std(self.ay), std(self.az)
        gyr_std_x, gyr_std_y, gyr_std_z = std(self.gx), std(self.gy), std(self.gz)

        acc_n = max(acc_std_x, acc_std_y, acc_std_z) / sqrt_rate
        gyr_n = max(gyr_std_x, gyr_std_y, gyr_std_z) / sqrt_rate

        print('\n===== IMU Characterization Results =====')
        print(f'Samples:      {n}')
        print(f'Duration:     {dt_total:.1f}s')
        print(f'Rate:         {rate:.1f} Hz')
        print(f'')
        print(f'--- Accelerometer (at rest) ---')
        print(f'Mean:         [{mean(self.ax):.4f}, {mean(self.ay):.4f}, {mean(self.az):.4f}] m/s²')
        print(f'Std:          [{acc_std_x:.6f}, {acc_std_y:.6f}, {acc_std_z:.6f}] m/s²')
        print(f'Noise density: {acc_n:.6f} m/s²/√Hz')
        print(f'')
        print(f'--- Gyroscope (at rest) ---')
        print(f'Mean:         [{mean(self.gx):.6f}, {mean(self.gy):.6f}, {mean(self.gz):.6f}] rad/s')
        print(f'Std:          [{gyr_std_x:.6f}, {gyr_std_y:.6f}, {gyr_std_z:.6f}] rad/s')
        print(f'Noise density: {gyr_n:.6f} rad/s/√Hz')
        print(f'')
        print(f'--- Gravity ---')
        print(f'Magnitude:    {grav:.4f} m/s² (expected 9.81 ± 0.3)')
        grav_ok = abs(grav - 9.81) <= 0.3
        print(f'Gate:         {"PASS" if grav_ok else "FAIL"}')
        print(f'')
        print(f'--- Rate ---')
        rate_ok = rate >= 200 * 0.9
        print(f'Gate (≥200Hz): {"PASS" if rate_ok else "FAIL"} ({rate:.1f} Hz)')
        print(f'')
        print(f'--- Suggested extrinsics.yaml values ---')
        print(f'imu_noise:')
        print(f'  acc_n: {acc_n:.6f}')
        print(f'  gyr_n: {gyr_n:.6f}')
        print(f'  acc_w: {acc_n * 0.02:.6f}  # estimated random walk')
        print(f'  gyr_w: {gyr_n * 0.006:.6f}  # estimated random walk')

rclpy.init()
node = ImuCharacterizer()
try:
    rclpy.spin(node)
except SystemExit:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PYEOF
