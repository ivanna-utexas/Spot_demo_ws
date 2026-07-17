#!/usr/bin/env python3
"""Monitor lidar/IMU timestamp alignment against the configured offset."""

import json
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

from spot_mapping_common.calibration import load_mapping_calibration


class TimingMonitorNode(Node):
    def __init__(self):
        super().__init__('timing_monitor_node')

        self.declare_parameter('pointcloud_topic', '/velodyne_points')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter(
            'extrinsics_file',
            '/home/ros/dance_ws_pedestrian_tracking/config/mapping/extrinsics.yaml',
        )
        self.declare_parameter('expected_offset_sec', float('nan'))
        self.declare_parameter('sample_window', 1000)
        self.declare_parameter('median_gate_ms', 5.0)
        self.declare_parameter('p99_gate_ms', 20.0)

        calibration = load_mapping_calibration(self.get_parameter('extrinsics_file').value)
        configured_offset = float(self.get_parameter('expected_offset_sec').value)
        if configured_offset != configured_offset:
            configured_offset = calibration['time_offset_lidar_imu']
        self.expected_offset_sec = configured_offset
        self.sample_window = int(self.get_parameter('sample_window').value)
        self.median_gate_ms = float(self.get_parameter('median_gate_ms').value)
        self.p99_gate_ms = float(self.get_parameter('p99_gate_ms').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter('pointcloud_topic').value,
            self.pointcloud_cb,
            qos,
        )
        self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self.imu_cb,
            qos,
        )
        self.diag_pub = self.create_publisher(String, '/mapping/diagnostics/timing', 10)

        self.imu_stamps_ns = deque(maxlen=max(self.sample_window, 10))
        self.offset_samples_sec = deque(maxlen=self.sample_window)
        self.create_timer(5.0, self.report_timer)
        self.get_logger().info(
            f'Timing monitor: expected lidar/imu offset={self.expected_offset_sec:.6f}s'
        )

    @staticmethod
    def _stamp_to_ns(stamp):
        return stamp.sec * 10**9 + stamp.nanosec

    def imu_cb(self, msg):
        self.imu_stamps_ns.append(self._stamp_to_ns(msg.header.stamp))

    def pointcloud_cb(self, msg):
        if not self.imu_stamps_ns:
            return

        cloud_stamp_ns = self._stamp_to_ns(msg.header.stamp)
        nearest_imu_ns = min(self.imu_stamps_ns, key=lambda stamp_ns: abs(stamp_ns - cloud_stamp_ns))
        measured_offset_sec = (nearest_imu_ns - cloud_stamp_ns) / 1e9
        self.offset_samples_sec.append(measured_offset_sec)

    def report_timer(self):
        if len(self.offset_samples_sec) < 10:
            self.get_logger().warn('Timing monitor does not have enough samples yet')
            return

        offsets_ms = [sample * 1000.0 for sample in self.offset_samples_sec]
        error_ms = [abs(offset - self.expected_offset_sec * 1000.0) for offset in offsets_ms]
        sorted_errors = sorted(error_ms)
        p99_index = min(len(sorted_errors) - 1, int(0.99 * len(sorted_errors)))
        median_error_ms = statistics.median(error_ms)
        p99_error_ms = sorted_errors[p99_index]
        mean_offset_ms = statistics.mean(offsets_ms)

        diag = {
            'sample_count': len(self.offset_samples_sec),
            'expected_offset_ms': round(self.expected_offset_sec * 1000.0, 3),
            'mean_offset_ms': round(mean_offset_ms, 3),
            'median_abs_error_ms': round(median_error_ms, 3),
            'p99_abs_error_ms': round(p99_error_ms, 3),
            'median_ok': median_error_ms < self.median_gate_ms,
            'p99_ok': p99_error_ms < self.p99_gate_ms,
        }

        msg = String()
        msg.data = json.dumps(diag)
        self.diag_pub.publish(msg)

        status = 'OK' if diag['median_ok'] and diag['p99_ok'] else 'WARN'
        self.get_logger().info(
            f'[{status}] mean_offset={diag["mean_offset_ms"]}ms '
            f'median_abs_error={diag["median_abs_error_ms"]}ms '
            f'p99_abs_error={diag["p99_abs_error_ms"]}ms'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TimingMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
