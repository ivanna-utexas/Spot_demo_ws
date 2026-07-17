"""Visualize a saved LIO-SAM map and trajectory in RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ws_root = os.path.expanduser('~/dance_ws_pedestrian_tracking')
    default_map_dir = os.path.join(
        ws_root,
        'maps',
        'lio_sam',
        '2026-04-04_debug_local_degradation-speedway-Apr4-4pm-speedway',
    )
    rviz_config = os.path.join(
        get_package_share_directory('lio_sam_bringup'),
        'rviz',
        'saved_map.rviz',
    )

    declare_map_pcd = DeclareLaunchArgument(
        'map_pcd',
        default_value=os.path.join(default_map_dir, 'cloudGlobal.pcd'),
        description='Saved global map PCD file',
    )
    declare_trajectory_pcd = DeclareLaunchArgument(
        'trajectory_pcd',
        default_value=os.path.join(default_map_dir, 'trajectory.pcd'),
        description='Saved trajectory PCD file',
    )
    declare_fixed_frame = DeclareLaunchArgument(
        'fixed_frame',
        default_value='map',
        description='Frame used for the published PCDs and RViz fixed frame',
    )
    declare_map_topic = DeclareLaunchArgument(
        'map_topic',
        default_value='/lio_sam/saved_map',
        description='Topic for the saved global map PointCloud2',
    )
    declare_trajectory_topic = DeclareLaunchArgument(
        'trajectory_topic',
        default_value='/lio_sam/saved_trajectory',
        description='Topic for the saved trajectory PointCloud2',
    )
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Publishing rate in Hz for the saved PCD publishers',
    )

    common_params = {
        'tf_frame': LaunchConfiguration('fixed_frame'),
        'publish_rate': ParameterValue(
            LaunchConfiguration('publish_rate'),
            value_type=float,
        ),
    }

    map_publisher = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='saved_map_publisher',
        parameters=[
            common_params,
            {'file_name': LaunchConfiguration('map_pcd')},
        ],
        remappings=[
            ('cloud_pcd', LaunchConfiguration('map_topic')),
        ],
        output='screen',
    )

    trajectory_publisher = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='saved_trajectory_publisher',
        parameters=[
            common_params,
            {'file_name': LaunchConfiguration('trajectory_pcd')},
        ],
        remappings=[
            ('cloud_pcd', LaunchConfiguration('trajectory_topic')),
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_saved_map',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        declare_map_pcd,
        declare_trajectory_pcd,
        declare_fixed_frame,
        declare_map_topic,
        declare_trajectory_topic,
        declare_publish_rate,
        map_publisher,
        trajectory_publisher,
        rviz,
    ])
