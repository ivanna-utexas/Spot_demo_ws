#!/usr/bin/env python3
"""
scan_extractor_node — Extract 2D LaserScan from 3D PointCloud2 for nav_stack.

Subscribes to /velodyne_points, transforms points into a target frame,
projects a configurable height slice, and publishes on /mapping/scan/nav.

This is used for:
  - Live 2D costmap updates during mapping
  - Offline extraction to produce ROS1-compatible LaserScan bags

Parameters:
  - input_topic: PointCloud2 source (default: /velodyne_points)
  - output_topic: LaserScan output (default: /mapping/scan/nav)
  - frame_id: Output frame (default: flat_body)
  - min_height: Min height in output frame [m] (default: -0.1)
  - max_height: Max height in output frame [m] (default: 0.5)
  - range_min: Min scan range [m] (default: 0.2)
  - range_max: Max scan range [m] (default: 30.0)
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, LaserScan
import struct
import math
from tf2_ros import Buffer, TransformException, TransformListener


class ScanExtractorNode(Node):
    def __init__(self):
        super().__init__('scan_extractor_node')

        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/mapping/scan/nav')
        self.declare_parameter('frame_id', 'flat_body')
        self.declare_parameter('min_height', -0.1)
        self.declare_parameter('max_height', 0.5)
        self.declare_parameter('range_min', 0.2)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.radians(0.2))
        self.declare_parameter('target_frame', '')
        self.declare_parameter('transform_timeout_sec', 0.05)

        legacy_frame = self.get_parameter('frame_id').value
        target_frame = self.get_parameter('target_frame').value
        self.target_frame = target_frame or legacy_frame
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.transform_timeout = Duration(
            seconds=float(self.get_parameter('transform_timeout_sec').value)
        )

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.pc_callback, qos
        )
        self.pub = self.create_publisher(LaserScan, output_topic, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.n_beams = int((self.angle_max - self.angle_min) / self.angle_increment)
        self.get_logger().info(
            f'Scan extractor: {input_topic} → {output_topic} '
            f'in {self.target_frame} [{self.min_height}, {self.max_height}]m, {self.n_beams} beams'
        )

    @staticmethod
    def _quat_to_matrix(qx, qy, qz, qw):
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        return [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]

    def _lookup_transform(self, source_frame, stamp):
        source_frame = (source_frame or '').lstrip('/')
        target_frame = (self.target_frame or '').lstrip('/')
        if not target_frame or source_frame == target_frame:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=self.transform_timeout,
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot transform {source_frame} -> {target_frame}: {exc}'
            )
            return None

        rotation = transform.transform.rotation
        translation = transform.transform.translation
        return (
            self._quat_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w),
            [translation.x, translation.y, translation.z],
        )

    def pc_callback(self, msg: PointCloud2):
        # Find field offsets
        x_off = y_off = z_off = None
        for field in msg.fields:
            if field.name == 'x':
                x_off = field.offset
            elif field.name == 'y':
                y_off = field.offset
            elif field.name == 'z':
                z_off = field.offset

        if x_off is None or y_off is None or z_off is None:
            return

        # Initialize ranges to inf
        ranges = [float('inf')] * self.n_beams
        step = msg.point_step
        data = msg.data
        transform = self._lookup_transform(msg.header.frame_id, msg.header.stamp)
        if self.target_frame and msg.header.frame_id != self.target_frame and transform is None:
            return

        if transform is None:
            rot = None
            trans = None
        else:
            rot, trans = transform

        for i in range(0, len(data) - step + 1, step):
            x = struct.unpack_from('<f', data, i + x_off)[0]
            y = struct.unpack_from('<f', data, i + y_off)[0]
            z = struct.unpack_from('<f', data, i + z_off)[0]

            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue

            if rot is not None:
                x_t = rot[0][0] * x + rot[0][1] * y + rot[0][2] * z + trans[0]
                y_t = rot[1][0] * x + rot[1][1] * y + rot[1][2] * z + trans[1]
                z_t = rot[2][0] * x + rot[2][1] * y + rot[2][2] * z + trans[2]
            else:
                x_t, y_t, z_t = x, y, z

            if z_t < self.min_height or z_t > self.max_height:
                continue

            r = math.sqrt(x_t * x_t + y_t * y_t)
            if r < self.range_min or r > self.range_max:
                continue

            angle = math.atan2(y_t, x_t)
            if angle < self.angle_min or angle > self.angle_max:
                continue

            idx = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= idx < self.n_beams and r < ranges[idx]:
                ranges[idx] = r

        # Publish LaserScan
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.target_frame or msg.header.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1  # ~10Hz
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges

        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanExtractorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
