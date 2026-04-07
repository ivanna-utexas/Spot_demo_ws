#!/usr/bin/env python3
"""Normalize a generic sensor_msgs/Imu stream for mapping backends."""

import json
import math
import os
import time

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from spot_mapping_common.calibration import load_mapping_calibration


def _covariance_from_values(values, diagonal_default):
    if len(values) == 1:
        diag = [float(values[0])] * 3
    elif len(values) == 3:
        diag = [float(values[0]), float(values[1]), float(values[2])]
    elif len(values) == 9:
        return [float(value) for value in values]
    else:
        diag = list(diagonal_default)

    return [
        diag[0], 0.0, 0.0,
        0.0, diag[1], 0.0,
        0.0, 0.0, diag[2],
    ]


def _shift_stamp(stamp, offset_seconds):
    stamp_ns = stamp.sec * 10**9 + stamp.nanosec
    shifted_ns = stamp_ns - int(offset_seconds * 1e9)
    if shifted_ns < 0:
        shifted_ns = 0
    shifted = TimeMsg()
    shifted.sec = shifted_ns // 10**9
    shifted.nanosec = shifted_ns % 10**9
    return shifted


class ImuNormalizerNode(Node):
    def __init__(self):
        super().__init__('imu_normalizer_node')

        self.declare_parameter('mode', 'use_quaternion')
        self.declare_parameter('input_topic', '/vectornav/imu')
        self.declare_parameter('output_topic', '/mapping/imu')
        self.declare_parameter(
            'extrinsics_file', os.path.expanduser('~/nav_ws/config/mapping/extrinsics.yaml')
        )
        self.declare_parameter('expected_rate_hz', 400.0)
        self.declare_parameter('gravity_nominal', 9.81)
        self.declare_parameter('gravity_tolerance', 0.3)
        self.declare_parameter('orientation_covariance', [])
        self.declare_parameter('angular_velocity_covariance', [])
        self.declare_parameter('linear_acceleration_covariance', [])
        self.declare_parameter('output_frame_id', '')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        mode = str(self.get_parameter('mode').value).strip().lower()
        self.keep_quaternion = mode in ('use_quaternion', 'quaternion', 'corrected_quat', '9_axis')
        self.expected_rate = float(self.get_parameter('expected_rate_hz').value)
        self.gravity_nominal = float(self.get_parameter('gravity_nominal').value)
        self.gravity_tolerance = float(self.get_parameter('gravity_tolerance').value)
        self.output_frame_id = self.get_parameter('output_frame_id').value

        calibration = load_mapping_calibration(self.get_parameter('extrinsics_file').value)
        covariances = calibration['imu_covariance']
        self.time_offset_lidar_imu = calibration['time_offset_lidar_imu']

        self.orientation_cov_unknown = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.orientation_covariance = _covariance_from_values(
            self.get_parameter('orientation_covariance').value,
            [covariances['orientation'][0], covariances['orientation'][4], covariances['orientation'][8]],
        )
        self.angular_velocity_covariance = _covariance_from_values(
            self.get_parameter('angular_velocity_covariance').value,
            [
                covariances['angular_velocity'][0],
                covariances['angular_velocity'][4],
                covariances['angular_velocity'][8],
            ],
        )
        self.linear_acceleration_covariance = _covariance_from_values(
            self.get_parameter('linear_acceleration_covariance').value,
            [
                covariances['linear_acceleration'][0],
                covariances['linear_acceleration'][4],
                covariances['linear_acceleration'][8],
            ],
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

        self.sub = self.create_subscription(Imu, input_topic, self.imu_cb, sub_qos)
        self.pub = self.create_publisher(Imu, output_topic, pub_qos)
        self.diag_pub = self.create_publisher(String, '/mapping/diagnostics/imu', 10)

        self.count = 0
        self.first_wall = None
        self.last_wall = None
        self.last_stamp_ns = None
        self.monotonic_violations = 0
        self.gravity_samples = []
        self.max_gravity_samples = 500
        self.create_timer(5.0, self.report_timer)

        mode_label = 'use_quaternion' if self.keep_quaternion else 'strip_to_six_axis'
        self.get_logger().info(f'IMU normalizer: {input_topic} -> {output_topic}, mode={mode_label}')

    def imu_cb(self, msg: Imu):
        now = time.monotonic()
        self.count += 1
        if self.first_wall is None:
            self.first_wall = now
        self.last_wall = now

        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
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
        out.header.stamp = _shift_stamp(msg.header.stamp, self.time_offset_lidar_imu)
        if self.output_frame_id:
            out.header.frame_id = self.output_frame_id

        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = self.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = self.linear_acceleration_covariance

        if self.keep_quaternion:
            norm = math.sqrt(
                msg.orientation.x * msg.orientation.x
                + msg.orientation.y * msg.orientation.y
                + msg.orientation.z * msg.orientation.z
                + msg.orientation.w * msg.orientation.w
            )
            if norm > 1e-6:
                out.orientation.x = msg.orientation.x / norm
                out.orientation.y = msg.orientation.y / norm
                out.orientation.z = msg.orientation.z / norm
                out.orientation.w = msg.orientation.w / norm
            else:
                out.orientation.w = 1.0
            out.orientation_covariance = self.orientation_covariance
        else:
            out.orientation.w = 1.0
            out.orientation_covariance = self.orientation_cov_unknown

        self.pub.publish(out)

    def report_timer(self):
        if self.count == 0:
            self.get_logger().warn('No IMU messages received')
            return

        elapsed = (self.last_wall - self.first_wall) if self.first_wall and self.last_wall else 0.0
        rate = self.count / elapsed if elapsed > 0 else 0.0

        gravity_mag = 0.0
        gravity_ok = False
        if self.gravity_samples:
            gravity_mag = sum(self.gravity_samples) / len(self.gravity_samples)
            gravity_ok = abs(gravity_mag - self.gravity_nominal) <= self.gravity_tolerance

        diag = {
            'mode': 'use_quaternion' if self.keep_quaternion else 'strip_to_six_axis',
            'msg_count': self.count,
            'rate_hz': round(rate, 1),
            'rate_ok': rate >= self.expected_rate * 0.9,
            'monotonic_violations': self.monotonic_violations,
            'gravity_magnitude': round(gravity_mag, 3),
            'gravity_ok': gravity_ok,
        }

        message = String()
        message.data = json.dumps(diag)
        self.diag_pub.publish(message)

        status = 'OK' if diag['rate_ok'] and diag['gravity_ok'] else 'WARN'
        self.get_logger().info(
            f'[{status}] rate={diag["rate_hz"]}Hz gravity={diag["gravity_magnitude"]}m/s^2 '
            f'mono_viol={diag["monotonic_violations"]} mode={diag["mode"]}'
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
