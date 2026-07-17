#!/usr/bin/env python3
"""
spot_imu_adapter.py — Minimal Phase 0 IMU adapter for mapping.

Subscribes to Spot's streamed IMU topic and republishes on /mapping/imu
with correct covariances for SLAM consumption.

Supports two modes (set via --mode):
  six_axis   : publishes linear_acceleration + angular_velocity only,
                zeroes out orientation (default for liorf until characterized)
  corrected_quat : passes through Spot's orientation quaternion as-is

This is the quick-script version. Phase 1 promotes this to a proper
ROS2 node in src/spot_mapping_common.

Usage:
  ros2 run --prefix 'python3' spot_imu_adapter.py  (standalone)
  python3 scripts/spot_imu_adapter.py --mode six_axis
"""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import yaml
import os


class SpotImuAdapter(Node):
    def __init__(self, mode: str, extrinsics_file: str):
        super().__init__('spot_imu_adapter')
        self.mode = mode
        self.get_logger().info(f'IMU adapter mode: {self.mode}')

        # Load noise params from canonical extrinsics
        self.acc_n = 0.01
        self.gyr_n = 0.005
        if os.path.isfile(extrinsics_file):
            try:
                with open(extrinsics_file) as f:
                    cal = yaml.safe_load(f)
                noise = cal.get('imu_noise', {})
                self.acc_n = noise.get('acc_n', self.acc_n)
                self.gyr_n = noise.get('gyr_n', self.gyr_n)
                self.get_logger().info(
                    f'Loaded noise from {extrinsics_file}: '
                    f'acc_n={self.acc_n}, gyr_n={self.gyr_n}'
                )
            except Exception as e:
                self.get_logger().warn(f'Failed to load extrinsics: {e}')

        # Covariance matrices (diagonal, row-major 3x3)
        self.orientation_cov_unknown = [0.0] * 9
        self.orientation_cov_unknown[0] = -1.0  # flag: unknown

        self.angular_vel_cov = [0.0] * 9
        self.angular_vel_cov[0] = self.gyr_n ** 2
        self.angular_vel_cov[4] = self.gyr_n ** 2
        self.angular_vel_cov[8] = self.gyr_n ** 2

        self.linear_acc_cov = [0.0] * 9
        self.linear_acc_cov[0] = self.acc_n ** 2
        self.linear_acc_cov[4] = self.acc_n ** 2
        self.linear_acc_cov[8] = self.acc_n ** 2

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, qos
        )
        self.pub = self.create_publisher(Imu, '/mapping/imu', qos)
        self.count = 0

    def imu_callback(self, msg: Imu):
        out = Imu()
        out.header = msg.header

        # Always pass through accel and gyro with computed covariances
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = self.linear_acc_cov
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = self.angular_vel_cov

        if self.mode == 'six_axis':
            # Zero orientation, flag covariance as unknown
            out.orientation.w = 1.0
            out.orientation.x = 0.0
            out.orientation.y = 0.0
            out.orientation.z = 0.0
            out.orientation_covariance = self.orientation_cov_unknown
        else:
            # corrected_quat: pass through Spot's orientation
            out.orientation = msg.orientation
            out.orientation_covariance = msg.orientation_covariance

        self.pub.publish(out)
        self.count += 1
        if self.count % 1000 == 0:
            self.get_logger().info(f'Republished {self.count} IMU messages')


def main():
    parser = argparse.ArgumentParser(description='Spot IMU adapter for mapping')
    parser.add_argument('--mode', default='six_axis',
                        choices=['six_axis', 'corrected_quat'],
                        help='IMU output mode (default: six_axis)')
    parser.add_argument('--extrinsics',
                        default=os.path.expanduser('~/dance_ws_pedestrian_tracking/config/mapping/extrinsics.yaml'),
                        help='Path to canonical extrinsics file')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = SpotImuAdapter(mode=args.mode, extrinsics_file=args.extrinsics)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
