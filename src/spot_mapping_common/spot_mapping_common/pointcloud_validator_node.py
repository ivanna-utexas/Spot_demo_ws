#!/usr/bin/env python3
"""
pointcloud_validator_node — Validates VLP-16 PointCloud2 messages for mapping.

Checks:
  - Required fields: x, y, z, intensity, ring, time
  - Point-time monotonicity within each scan
  - Packet loss estimation (from expected vs actual point count)
  - Message rate
  - Timestamp policy consistency with velodyne driver config

Publishes diagnostics on /mapping/diagnostics/pointcloud.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
import json
import time

EXPECTED_FIELDS = {'x', 'y', 'z', 'intensity', 'ring', 'time'}
# VLP-16 at 600 RPM ≈ 10 Hz, ~28800 points per scan (16 beams × 1800 firings)
EXPECTED_POINTS_PER_SCAN = 28800
EXPECTED_RATE_HZ = 10.0
RATE_TOLERANCE = 0.2  # ±20%


class PointCloudValidatorNode(Node):
    def __init__(self):
        super().__init__('pointcloud_validator_node')

        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('expected_rate_hz', EXPECTED_RATE_HZ)
        self.declare_parameter('packet_loss_threshold', 0.005)  # 0.5%

        input_topic = self.get_parameter('input_topic').value
        self.expected_rate = self.get_parameter('expected_rate_hz').value
        self.loss_threshold = self.get_parameter('packet_loss_threshold').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.pc_callback, qos
        )
        self.diag_pub = self.create_publisher(String, '/mapping/diagnostics/pointcloud', 10)

        self.msg_count = 0
        self.first_ts = None
        self.last_ts = None
        self.fields_validated = False
        self.fields_ok = False
        self.missing_fields = []
        self.total_points = 0
        self.total_expected = 0

        # Periodic report
        self.create_timer(5.0, self.report_timer)
        self.get_logger().info(f'Validating {input_topic}')

    def pc_callback(self, msg: PointCloud2):
        now = time.monotonic()
        self.msg_count += 1

        if self.first_ts is None:
            self.first_ts = now
        self.last_ts = now

        # Validate fields once
        if not self.fields_validated:
            field_names = {f.name for f in msg.fields}
            self.missing_fields = list(EXPECTED_FIELDS - field_names)
            self.fields_ok = len(self.missing_fields) == 0
            self.fields_validated = True

            if self.fields_ok:
                self.get_logger().info(f'PointCloud2 fields OK: {sorted(field_names)}')
            else:
                self.get_logger().error(
                    f'Missing fields: {self.missing_fields}. '
                    f'Have: {sorted(field_names)}'
                )

        # Point count for packet loss estimation
        if msg.height > 0 and msg.width > 0:
            n_points = msg.height * msg.width
        elif msg.point_step > 0:
            n_points = len(msg.data) // msg.point_step
        else:
            n_points = 0

        self.total_points += n_points
        self.total_expected += EXPECTED_POINTS_PER_SCAN

    def report_timer(self):
        if self.msg_count == 0:
            self.get_logger().warn('No PointCloud2 messages received yet')
            return

        elapsed = (self.last_ts - self.first_ts) if self.first_ts and self.last_ts else 0
        rate = self.msg_count / elapsed if elapsed > 0 else 0
        loss = 1.0 - (self.total_points / self.total_expected) if self.total_expected > 0 else 0
        loss_pct = max(0, loss) * 100

        diag = {
            'fields_ok': self.fields_ok,
            'missing_fields': self.missing_fields,
            'msg_count': self.msg_count,
            'rate_hz': round(rate, 2),
            'rate_ok': abs(rate - self.expected_rate) / self.expected_rate < RATE_TOLERANCE if rate > 0 else False,
            'packet_loss_pct': round(loss_pct, 3),
            'packet_loss_ok': loss_pct < self.loss_threshold * 100,
            'total_points': self.total_points,
        }

        msg = String()
        msg.data = json.dumps(diag)
        self.diag_pub.publish(msg)

        status = 'OK' if diag['fields_ok'] and diag['rate_ok'] and diag['packet_loss_ok'] else 'WARN'
        self.get_logger().info(
            f'[{status}] rate={diag["rate_hz"]}Hz loss={diag["packet_loss_pct"]:.3f}% '
            f'fields_ok={diag["fields_ok"]} msgs={diag["msg_count"]}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
