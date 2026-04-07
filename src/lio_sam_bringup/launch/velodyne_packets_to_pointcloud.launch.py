"""Minimal packet-to-pointcloud launch for packet-only Velodyne bag replay."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from spot_mapping_common.calibration import load_mapping_calibration


def generate_launch_description():
    ws_root = os.path.expanduser('~/nav_ws')
    extrinsics_path = os.path.join(ws_root, 'config', 'mapping', 'extrinsics.yaml')
    frames = load_mapping_calibration(extrinsics_path)['frames']

    velodyne_pointcloud_share = get_package_share_directory('velodyne_pointcloud')
    params_path = os.path.join(
        velodyne_pointcloud_share,
        'config',
        'VLP16-velodyne_transform_node-params.yaml',
    )
    with open(params_path, 'r', encoding='utf-8') as handle:
        params = yaml.safe_load(handle)['velodyne_transform_node']['ros__parameters']
    params['calibration'] = os.path.join(
        velodyne_pointcloud_share,
        'params',
        'VLP16db.yaml',
    )
    params['frame_id'] = frames['lidar']

    declare_packet_topic = DeclareLaunchArgument(
        'packet_topic',
        default_value='/velodyne_packets',
        description='VelodyneScan input topic',
    )
    declare_pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/velodyne_points',
        description='PointCloud2 output topic',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
    )

    pointcloud_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform_node',
        parameters=[
            params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('velodyne_packets', LaunchConfiguration('packet_topic')),
            ('velodyne_points', LaunchConfiguration('pointcloud_topic')),
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_packet_topic,
        declare_pointcloud_topic,
        declare_use_sim_time,
        pointcloud_node,
    ])
