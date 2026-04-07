#!/usr/bin/env python3
"""Republish supported sensor topics with a BEST_EFFORT QoS profile."""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from velodyne_msgs.msg import VelodyneScan


SUPPORTED_MESSAGE_TYPES = {
    'imu': Imu,
    'pointcloud2': PointCloud2,
    'velodyne_scan': VelodyneScan,
}


class BestEffortTopicRelayNode(Node):
    def __init__(self):
        super().__init__('best_effort_topic_relay_node')

        self.declare_parameter('input_topic', '')
        self.declare_parameter('output_topic', '')
        self.declare_parameter('message_type', 'pointcloud2')
        self.declare_parameter('depth', 32)

        input_topic = str(self.get_parameter('input_topic').value).strip()
        output_topic = str(self.get_parameter('output_topic').value).strip()
        message_type = str(self.get_parameter('message_type').value).strip().lower()
        depth = max(1, int(self.get_parameter('depth').value))

        if not input_topic or not output_topic:
            raise ValueError('input_topic and output_topic must both be set')

        try:
            msg_type = SUPPORTED_MESSAGE_TYPES[message_type]
        except KeyError as exc:
            supported = ', '.join(sorted(SUPPORTED_MESSAGE_TYPES))
            raise ValueError(f'Unsupported message_type "{message_type}". Expected one of: {supported}') from exc

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
        )

        self.pub = self.create_publisher(msg_type, output_topic, qos)
        self.sub = self.create_subscription(msg_type, input_topic, self._relay_cb, qos)
        self.get_logger().info(
            f'BEST_EFFORT relay: {input_topic} -> {output_topic} ({message_type}, depth={depth})'
        )

    def _relay_cb(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BestEffortTopicRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
