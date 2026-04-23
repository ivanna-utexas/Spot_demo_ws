#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import yaml
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class SimpleMapServer(Node):
    def __init__(self):
        super().__init__('simple_map_server')
        self.declare_parameter('yaml_filename', '')

        # QoS matching nav2 expectations: RELIABLE + TRANSIENT_LOCAL
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', latching_qos)
        self.meta_pub = self.create_publisher(MapMetaData, 'map_metadata', latching_qos)

        self.map_msg = None

        yaml_file = self.get_parameter('yaml_filename').value
        if not yaml_file:
            self.get_logger().error("No yaml_filename provided")
            return

        self.get_logger().info(f"Loading map from {yaml_file}")
        self.load_map(yaml_file)

        # Re-publish map periodically so late-joining subscribers get it
        self.create_timer(2.0, self.republish_map)

    def republish_map(self):
        if self.map_msg is not None:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.map_msg)
            self.meta_pub.publish(self.map_msg.info)

    def load_map(self, yaml_file):
        try:
            with open(yaml_file, 'r') as f:
                map_data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load yaml: {e}")
            return

        image_file = map_data.get('image', '')
        if not os.path.isabs(image_file):
            image_file = os.path.join(os.path.dirname(yaml_file), image_file)
            
        resolution = float(map_data.get('resolution', 0.05))
        origin = map_data.get('origin', [0.0, 0.0, 0.0])
        occupied_thresh = map_data.get('occupied_thresh', 0.65)
        free_thresh = map_data.get('free_thresh', 0.25)
        negate = map_data.get('negate', 0)

        self.get_logger().info(f"Loading image from {image_file}")
        img = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error("Failed to load image")
            return

        # Prepare MapMetaData
        meta = MapMetaData()
        meta.map_load_time = self.get_clock().now().to_msg()
        meta.resolution = resolution
        meta.width = img.shape[1]
        meta.height = img.shape[0]
        meta.origin = Pose()
        meta.origin.position.x = float(origin[0])
        meta.origin.position.y = float(origin[1])
        # origin[2] is yaw, need quaternion
        # Simple yaw to quat
        yaw = float(origin[2])
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        meta.origin.orientation.w = cy
        meta.origin.orientation.z = sy
        
        self.meta_msg = meta
        self.publish_map(img, meta, occupied_thresh, free_thresh, negate)

    def publish_map(self, img, meta, occupied_thresh, free_thresh, negate):
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = "map"
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.info = meta
        
        # ROS 2 map data: (-1: unknown, 0: free, 100: occupied)
        # Flip vertical because ROS map origin is bottom-left, image is top-left
        img_flipped = np.flipud(img)
        
        # Normalize to 0-1
        if negate:
             p = img_flipped / 255.0
        else:
             p = (255.0 - img_flipped) / 255.0
        
        # Create grid
        grid_data = np.full(img_flipped.shape, -1, dtype=np.int8)
        
        # Free
        grid_data[p < free_thresh] = 0
        # Occupied
        grid_data[p > occupied_thresh] = 100
        
        map_msg.data = grid_data.flatten().tolist()
        
        self.map_msg = map_msg
        self.get_logger().info("Publishing map...")
        self.map_pub.publish(map_msg)
        self.meta_pub.publish(meta)
        self.get_logger().info("Map published.")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
