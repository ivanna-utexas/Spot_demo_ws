#!/usr/bin/env python3

"""
Launch file for the Twist to Ackermann Curvature Drive converter node.

This node bridges Nav2's cmd_vel (Twist) output to the VESC driver's 
ackermann_curvature_drive input.

Usage:
    ros2 launch spot_nav twist_to_ackermann.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='cmd_vel',
        description='Input Twist topic (usually from Nav2 controller)'
    )
    
    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/ackermann_curvature_drive',
        description='Output AckermannCurvatureDrive topic (for VESC driver)'
    )
    
    declare_wheelbase = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.324',
        description='Vehicle wheelbase in meters (should match vesc.lua config)'
    )
    
    declare_max_curvature = DeclareLaunchArgument(
        'max_curvature',
        default_value='5.0',
        description='Maximum curvature in 1/meters (inverse of minimum turn radius)'
    )

    # Get launch configurations
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    wheelbase = LaunchConfiguration('wheelbase')
    max_curvature = LaunchConfiguration('max_curvature')

    # Twist to Ackermann converter node
    converter_node = Node(
        package='spot_nav',
        executable='twist_to_ackermann',
        name='twist_to_ackermann_converter',
        output='screen',
        parameters=[{
            'input_topic': input_topic,
            'output_topic': output_topic,
            'wheelbase': wheelbase,
            'max_curvature': max_curvature,
        }]
    )

    return LaunchDescription([
        declare_input_topic,
        declare_output_topic,
        declare_wheelbase,
        declare_max_curvature,
        converter_node,
    ])
