#!/usr/bin/env python3

"""
Twist to Ackermann Curvature Drive Converter

This node bridges Nav2's cmd_vel (Twist) output to the VESC driver's 
ackermann_curvature_drive input for Ackermann steering vehicles.

Converts:
- geometry_msgs/Twist -> amrl_msgs/AckermannCurvatureDriveMsg
- linear.x (m/s) -> velocity
- angular.z (rad/s) -> curvature (1/turn_radius)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from amrl_msgs.msg import AckermannCurvatureDriveMsg


class TwistToAckermannConverter(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann_converter')
        
        # Declare parameters
        self.declare_parameter('input_topic', 'cmd_vel')
        self.declare_parameter('output_topic', '/ackermann_curvature_drive')
        # meters, from vesc.lua
        self.declare_parameter('wheelbase', 0.324)
        # 1/meters (inverse of min turn radius)
        # using a conservative value of 0.8m turn radius
        self.declare_parameter('max_curvature', 1.25)
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_curvature = self.get_parameter('max_curvature').value
        
        # Create subscriber and publisher
        self.twist_sub = self.create_subscription(
            Twist,
            input_topic,
            self.twist_callback,
            10
        )
        
        self.ackermann_pub = self.create_publisher(
            AckermannCurvatureDriveMsg,
            output_topic,
            10
        )
        
        self.get_logger().info(f'Twist to Ackermann converter started')
        self.get_logger().info(f'  Input: {input_topic} (Twist)')
        self.get_logger().info(f'  Output: {output_topic} (AckermannCurvatureDrive)')
        self.get_logger().info(f'  Wheelbase: {self.wheelbase} m')
        self.get_logger().info(f'  Max curvature: {self.max_curvature} (1/m)')
    
    def twist_callback(self, twist_msg):
        """
        Convert Twist message to AckermannCurvatureDriveMsg.
        
        For Ackermann steering:
        - curvature = omega / v (when v != 0)
        - curvature = tan(steering_angle) / wheelbase
        
        Where:
        - omega = angular velocity (twist.angular.z)
        - v = linear velocity (twist.linear.x)
        """
        ackermann_msg = AckermannCurvatureDriveMsg()
        
        # Extract linear and angular velocities
        linear_vel = twist_msg.linear.x
        angular_vel = twist_msg.angular.z
        
        # Calculate curvature
        if abs(linear_vel) > 0.001:  # Avoid division by zero
            curvature = angular_vel / linear_vel
            
            # Clamp curvature to vehicle limits
            if abs(curvature) > self.max_curvature:
                curvature = self.max_curvature if curvature > 0 else -self.max_curvature
                self.get_logger().debug(
                    f'Curvature clamped to ±{self.max_curvature}',
                    throttle_duration_sec=1.0
                )
        else:
            # If not moving forward, no curvature
            curvature = 0.0
        
        # Populate Ackermann message
        ackermann_msg.velocity = linear_vel
        ackermann_msg.curvature = curvature
        
        # Publish
        self.ackermann_pub.publish(ackermann_msg)
        
        # Debug logging (throttled)
        self.get_logger().debug(
            f'v={linear_vel:.2f} m/s, ω={angular_vel:.2f} rad/s -> '
            f'v={ackermann_msg.velocity:.2f} m/s, κ={ackermann_msg.curvature:.3f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermannConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
