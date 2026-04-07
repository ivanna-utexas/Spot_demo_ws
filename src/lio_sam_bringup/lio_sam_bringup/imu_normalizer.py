#!/usr/bin/env python3
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def _covariance_from_param(value, fallback):
    if value is None:
        return list(fallback)
    if isinstance(value, (int, float)):
        diag = float(value)
        matrix = [0.0] * 9
        matrix[0] = diag
        matrix[4] = diag
        matrix[8] = diag
        return matrix
    seq = list(value)
    if len(seq) == 3:
        matrix = [0.0] * 9
        matrix[0] = float(seq[0])
        matrix[4] = float(seq[1])
        matrix[8] = float(seq[2])
        return matrix
    if len(seq) == 9:
        return [float(component) for component in seq]
    return list(fallback)


class ImuNormalizerNode(Node):
    def __init__(self):
        super().__init__('lio_sam_imu_normalizer')

        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/mapping/imu')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('mode', 'quaternion')
        self.declare_parameter('time_offset_sec', 0.0)
        self.declare_parameter('expected_rate_hz', 200.0)
        self.declare_parameter('gravity_nominal', 9.81)
        self.declare_parameter('gravity_tolerance', 0.3)
        self.declare_parameter('orientation_covariance', [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('angular_velocity_covariance', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('linear_acceleration_covariance', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.mode = self.get_parameter('mode').value
        self.time_offset_sec = float(self.get_parameter('time_offset_sec').value)
        self.expected_rate_hz = float(self.get_parameter('expected_rate_hz').value)
        self.gravity_nominal = float(self.get_parameter('gravity_nominal').value)
        self.gravity_tolerance = float(self.get_parameter('gravity_tolerance').value)

        self.orientation_covariance = _covariance_from_param(
            self.get_parameter('orientation_covariance').value,
            [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.angular_velocity_covariance = _covariance_from_param(
            self.get_parameter('angular_velocity_covariance').value,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.linear_acceleration_covariance = _covariance_from_param(
            self.get_parameter('linear_acceleration_covariance').value,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )

        self.sub = self.create_subscription(Imu, self.input_topic, self._imu_cb, sub_qos)
        self.pub = self.create_publisher(Imu, self.output_topic, pub_qos)
        self.diag_pub = self.create_publisher(String, '/mapping/diagnostics/imu', 10)

        self.count = 0
        self.first_wall = None
        self.last_wall = None
        self.last_stamp_ns = None
        self.monotonic_violations = 0
        self.gravity_samples = []
        self.max_gravity_samples = 500

        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f'IMU normalizer: {self.input_topic} -> {self.output_topic} '
            f'mode={self.mode} frame={self.frame_id} offset={self.time_offset_sec:.6f}s'
        )

    def _shift_stamp(self, msg):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        stamp_ns += int(round(self.time_offset_sec * 1_000_000_000))
        if stamp_ns < 0:
            stamp_ns = 0
        msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)

    def _imu_cb(self, msg):
        now = time.monotonic()
        self.count += 1
        if self.first_wall is None:
            self.first_wall = now
        self.last_wall = now

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            self.monotonic_violations += 1
        self.last_stamp_ns = stamp_ns

        if len(self.gravity_samples) < self.max_gravity_samples:
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            self.gravity_samples.append(math.sqrt(ax * ax + ay * ay + az * az))

        out = Imu()
        out.header = msg.header
        self._shift_stamp(out)
        out.header.frame_id = self.frame_id
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = self.linear_acceleration_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = self.angular_velocity_covariance

        if self.mode == 'six_axis':
            out.orientation.w = 1.0
            out.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            out.orientation = msg.orientation
            out.orientation_covariance = self.orientation_covariance

        self.pub.publish(out)

    def _report(self):
        if self.count == 0:
            self.get_logger().warn('No IMU messages received yet')
            return

        elapsed = (self.last_wall - self.first_wall) if self.first_wall and self.last_wall else 0.0
        rate_hz = self.count / elapsed if elapsed > 0.0 else 0.0
        gravity_mag = sum(self.gravity_samples) / len(self.gravity_samples) if self.gravity_samples else 0.0
        diag = {
            'mode': self.mode,
            'frame_id': self.frame_id,
            'msg_count': self.count,
            'rate_hz': round(rate_hz, 1),
            'rate_ok': rate_hz >= self.expected_rate_hz * 0.9 if self.expected_rate_hz > 0.0 else True,
            'monotonic_violations': self.monotonic_violations,
            'gravity_magnitude': round(gravity_mag, 3),
            'gravity_ok': abs(gravity_mag - self.gravity_nominal) <= self.gravity_tolerance,
        }

        msg = String()
        msg.data = json.dumps(diag)
        self.diag_pub.publish(msg)
        status = 'OK' if diag['rate_ok'] and diag['gravity_ok'] else 'WARN'
        self.get_logger().info(
            f'[{status}] rate={diag["rate_hz"]}Hz gravity={diag["gravity_magnitude"]}m/s^2 '
            f'mono_viol={diag["monotonic_violations"]} mode={self.mode}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuNormalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
