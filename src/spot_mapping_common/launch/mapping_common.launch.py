"""
mapping_common.launch.py — Backend-neutral mapping infrastructure.

Launches:
  - pointcloud_validator_node (validates VLP-16 PointCloud2)
  - spot_imu_adapter_node (adapts Spot IMU for SLAM consumption)
  - imu_normalizer_node (normalizes generic sensor_msgs/Imu streams)
  - scan_extractor_node (2D LaserScan from 3D cloud)

This launch file is included by candidate-specific bringup (e.g. liorf).
It does NOT launch the Velodyne driver or any SLAM backend.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_imu_source = DeclareLaunchArgument(
        'imu_source', default_value='spot',
        description='IMU adapter type: spot or generic'
    )
    declare_imu_mode = DeclareLaunchArgument(
        'imu_mode', default_value='six_axis',
        description='IMU mode: six_axis, corrected_quat, or use_quaternion'
    )
    declare_pointcloud_input = DeclareLaunchArgument(
        'pointcloud_input_topic', default_value='/velodyne_points',
        description='PointCloud2 input topic for validators and scan extraction'
    )
    declare_imu_input = DeclareLaunchArgument(
        'imu_input_topic', default_value='/imu/data',
        description='Spot IMU input topic'
    )
    declare_imu_output = DeclareLaunchArgument(
        'imu_output_topic', default_value='/mapping/imu',
        description='Normalized IMU output topic for downstream SLAM'
    )
    declare_imu_expected_rate = DeclareLaunchArgument(
        'imu_expected_rate_hz', default_value='200.0',
        description='Expected IMU publish rate for diagnostics'
    )
    declare_extrinsics = DeclareLaunchArgument(
        'extrinsics_file',
        default_value=os.path.expanduser('~/nav_ws/config/mapping/extrinsics.yaml'),
        description='Path to canonical extrinsics YAML'
    )
    declare_scan_frame = DeclareLaunchArgument(
        'scan_frame', default_value='flat_body',
        description='Frame for 2D scan extraction'
    )
    declare_scan_output = DeclareLaunchArgument(
        'scan_output_topic', default_value='/mapping/scan/nav',
        description='LaserScan output topic for 2D navigation consumers'
    )
    declare_scan_transform_timeout = DeclareLaunchArgument(
        'scan_transform_timeout_sec', default_value='0.05',
        description='TF lookup timeout for pointcloud-to-scan projection'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use sim time for bag replay'
    )
    declare_imu_output_frame = DeclareLaunchArgument(
        'imu_output_frame', default_value='imu_link',
        description='Frame id for normalized IMU output'
    )
    declare_enable_timing_monitor = DeclareLaunchArgument(
        'enable_timing_monitor', default_value='false',
        description='Enable lidar/IMU skew diagnostics'
    )

    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    pointcloud_validator = Node(
        package='spot_mapping_common',
        executable='pointcloud_validator_node',
        name='pointcloud_validator_node',
        parameters=[{
            'input_topic': LaunchConfiguration('pointcloud_input_topic'),
            'expected_rate_hz': 10.0,
            'packet_loss_threshold': 0.005,
        }, sim_time],
        output='screen',
    )

    imu_adapter = Node(
        package='spot_mapping_common',
        executable='spot_imu_adapter_node',
        name='spot_imu_adapter_node',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('imu_source'), "' == 'spot'"])),
        parameters=[{
            'mode': LaunchConfiguration('imu_mode'),
            'input_topic': LaunchConfiguration('imu_input_topic'),
            'output_topic': LaunchConfiguration('imu_output_topic'),
            'extrinsics_file': LaunchConfiguration('extrinsics_file'),
            'expected_rate_hz': LaunchConfiguration('imu_expected_rate_hz'),
            'gravity_nominal': 9.81,
            'gravity_tolerance': 0.3,
        }, sim_time],
        output='screen',
    )

    imu_normalizer = Node(
        package='spot_mapping_common',
        executable='imu_normalizer_node',
        name='imu_normalizer_node',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('imu_source'), "' == 'generic'"])),
        parameters=[{
            'mode': LaunchConfiguration('imu_mode'),
            'input_topic': LaunchConfiguration('imu_input_topic'),
            'output_topic': LaunchConfiguration('imu_output_topic'),
            'output_frame_id': LaunchConfiguration('imu_output_frame'),
            'extrinsics_file': LaunchConfiguration('extrinsics_file'),
            'expected_rate_hz': LaunchConfiguration('imu_expected_rate_hz'),
            'gravity_nominal': 9.81,
            'gravity_tolerance': 0.3,
        }, sim_time],
        output='screen',
    )

    scan_extractor = Node(
        package='spot_mapping_common',
        executable='scan_extractor_node',
        name='scan_extractor_node',
        parameters=[{
            'input_topic': LaunchConfiguration('pointcloud_input_topic'),
            'output_topic': LaunchConfiguration('scan_output_topic'),
            'frame_id': LaunchConfiguration('scan_frame'),
            'target_frame': LaunchConfiguration('scan_frame'),
            'transform_timeout_sec': LaunchConfiguration('scan_transform_timeout_sec'),
            'min_height': -0.1,
            'max_height': 0.5,
            'range_min': 0.2,
            'range_max': 30.0,
        }, sim_time],
        output='screen',
    )

    timing_monitor = Node(
        package='spot_mapping_common',
        executable='timing_monitor_node',
        name='timing_monitor_node',
        condition=IfCondition(LaunchConfiguration('enable_timing_monitor')),
        parameters=[{
            'pointcloud_topic': LaunchConfiguration('pointcloud_input_topic'),
            'imu_topic': LaunchConfiguration('imu_input_topic'),
            'extrinsics_file': LaunchConfiguration('extrinsics_file'),
        }, sim_time],
        output='screen',
    )

    return LaunchDescription([
        declare_imu_source,
        declare_imu_mode,
        declare_pointcloud_input,
        declare_imu_input,
        declare_imu_output,
        declare_imu_expected_rate,
        declare_extrinsics,
        declare_scan_frame,
        declare_scan_output,
        declare_scan_transform_timeout,
        declare_use_sim_time,
        declare_imu_output_frame,
        declare_enable_timing_monitor,
        pointcloud_validator,
        imu_adapter,
        imu_normalizer,
        scan_extractor,
        timing_monitor,
    ])
