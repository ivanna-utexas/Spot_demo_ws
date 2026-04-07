#!/usr/bin/env python3
"""
spot_imu_adapter_node — Proper ROS2 node version of the Phase 0 IMU adapter.

Subscribes to Spot's streamed IMU and republishes on /mapping/imu with:
  - Preserved packet timestamps
  - Computed covariances from measured noise parameters
  - Configurable mode: six_axis (default for liorf) or corrected_quat
  - Diagnostics: rate monitoring, gravity magnitude check, timestamp monotonicity

Reads noise parameters from the canonical extrinsics file.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import yaml
import os
import math
import json
import time


class SpotImuAdapterNode(Node):
    def __init__(self):
        super().__init__('spot_imu_adapter_node')

        # Parameters
        self.declare_parameter('mode', 'six_axis')
        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/mapping/imu')
        self.declare_parameter('extrinsics_file',
                               os.path.expanduser('~/nav_ws/config/mapping/extrinsics.yaml'))
        self.declare_parameter('expected_rate_hz', 200.0)
        self.declare_parameter('gravity_nominal', 9.81)
        self.declare_parameter('gravity_tolerance', 0.3)

        self.mode = self.get_parameter('mode').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        extrinsics_file = self.get_parameter('extrinsics_file').value
        self.expected_rate = self.get_parameter('expected_rate_hz').value
        self.gravity_nominal = self.get_parameter('gravity_nominal').value
        self.gravity_tolerance = self.get_parameter('gravity_tolerance').value

        # Load noise from canonical extrinsics
        self.acc_n = 0.01
        self.gyr_n = 0.005
        if os.path.isfile(extrinsics_file):
            try:
                with open(extrinsics_file) as f:
                    cal = yaml.safe_load(f)
                noise = cal.get('imu_noise', {})
                self.acc_n = noise.get('acc_n', self.acc_n)
                self.gyr_n = noise.get('gyr_n', self.gyr_n)
                self.get_logger().info(f'Loaded noise: acc_n={self.acc_n}, gyr_n={self.gyr_n}')
            except Exception as e:
                self.get_logger().warn(f'Cannot load extrinsics: {e}, using defaults')

        # Build covariance arrays
        self.orientation_cov_unknown = [0.0] * 9
        self.orientation_cov_unknown[0] = -1.0

        self.angular_vel_cov = [0.0] * 9
        for i in [0, 4, 8]:
            self.angular_vel_cov[i] = self.gyr_n ** 2

        self.linear_acc_cov = [0.0] * 9
        for i in [0, 4, 8]:
            self.linear_acc_cov[i] = self.acc_n ** 2

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=200
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200
        )

        self.sub = self.create_subscription(Imu, input_topic, self.imu_cb, sub_qos)
        self.pub = self.create_publisher(Imu, output_topic, pub_qos)
        self.diag_pub = self.create_publisher(String, '/mapping/diagnostics/imu', 10)

        # Diagnostics state
        self.count = 0
        self.first_wall = None
        self.last_wall = None
        self.last_stamp_ns = None
        self.monotonic_violations = 0
        self.gravity_samples = []
        self.max_gravity_samples = 500  # ~2.5s at 200Hz

        self.create_timer(5.0, self.report_timer)
        self.get_logger().info(
            f'IMU adapter: {input_topic} → {output_topic}, mode={self.mode}'
        )

    def imu_cb(self, msg: Imu):
        now = time.monotonic()
        self.count += 1
        if self.first_wall is None:
            self.first_wall = now
        self.last_wall = now

        # Timestamp monotonicity check
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            self.monotonic_violations += 1
        self.last_stamp_ns = stamp_ns

        # Gravity sample (first N messages)
        if len(self.gravity_samples) < self.max_gravity_samples:
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            self.gravity_samples.append(math.sqrt(ax*ax + ay*ay + az*az))

        # Republish
        out = Imu()
        out.header = msg.header
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = self.linear_acc_cov
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = self.angular_vel_cov

        if self.mode == 'six_axis':
            out.orientation.w = 1.0
            out.orientation_covariance = self.orientation_cov_unknown
        else:
            out.orientation = msg.orientation
            out.orientation_covariance = msg.orientation_covariance

        self.pub.publish(out)

    def report_timer(self):
        if self.count == 0:
            self.get_logger().warn('No IMU messages received')
            return

        elapsed = (self.last_wall - self.first_wall) if self.first_wall and self.last_wall else 0
        rate = self.count / elapsed if elapsed > 0 else 0

        gravity_mag = 0.0
        gravity_ok = False
        if self.gravity_samples:
            gravity_mag = sum(self.gravity_samples) / len(self.gravity_samples)
            gravity_ok = abs(gravity_mag - self.gravity_nominal) <= self.gravity_tolerance

        diag = {
            'mode': self.mode,
            'msg_count': self.count,
            'rate_hz': round(rate, 1),
            'rate_ok': rate >= self.expected_rate * 0.9,
            'monotonic_violations': self.monotonic_violations,
            'gravity_magnitude': round(gravity_mag, 3),
            'gravity_ok': gravity_ok,
        }

        msg = String()
        msg.data = json.dumps(diag)
        self.diag_pub.publish(msg)

        status = 'OK' if diag['rate_ok'] and diag['gravity_ok'] else 'WARN'
        self.get_logger().info(
            f'[{status}] rate={diag["rate_hz"]}Hz gravity={diag["gravity_magnitude"]}m/s² '
            f'mono_viol={diag["monotonic_violations"]} mode={self.mode}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SpotImuAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
