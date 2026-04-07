"""
liorf_replay.launch.py — Offline replay of a raw bag through LiORF.

This is the authoritative map-building path (not live RViz).

Launches:
  1. mapping_common (validators, IMU adapter, scan extractor)
  2. liorf SLAM node with Spot-specific config
  3. Extrinsics injected from canonical config/mapping/extrinsics.yaml

Usage:
  ros2 launch spot_liorf_bringup liorf_replay.launch.py
  # In another terminal: ros2 bag play bags/raw/<date_run>/ --clock
"""
import os
import yaml
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_extrinsics(path):
    """Load canonical extrinsics and convert to liorf format."""
    defaults = {
        'trans': [0.0, 0.0, 0.5],
        'rot': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'acc_n': 0.01, 'gyr_n': 0.005, 'acc_w': 0.0002, 'gyr_w': 0.00003,
        'gravity': 9.81,
    }
    if not os.path.isfile(path):
        return defaults

    with open(path) as f:
        cal = yaml.safe_load(f)

    b2l = cal.get('body_to_lidar', {})
    tx = b2l.get('x', 0.0)
    ty = b2l.get('y', 0.0)
    tz = b2l.get('z', 0.5)
    roll = b2l.get('roll', 0.0)
    pitch = b2l.get('pitch', 0.0)
    yaw = b2l.get('yaw', 0.0)

    # Build rotation matrix from RPY
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # R_body_lidar: rotation of lidar in body frame (matches URDF convention)
    R_b2l = [
        cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
        sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
        -sp,    cp*sr,             cp*cr,
    ]

    # liorf extrinsicRot needs R_lidar_body (transpose of R_body_lidar)
    R = [R_b2l[0], R_b2l[3], R_b2l[6],
         R_b2l[1], R_b2l[4], R_b2l[7],
         R_b2l[2], R_b2l[5], R_b2l[8]]

    # liorf extrinsicTrans = translation from lidar to body, in lidar frame
    t_neg = [-tx, -ty, -tz]
    ext_trans = [
        R[0]*t_neg[0] + R[1]*t_neg[1] + R[2]*t_neg[2],
        R[3]*t_neg[0] + R[4]*t_neg[1] + R[5]*t_neg[2],
        R[6]*t_neg[0] + R[7]*t_neg[1] + R[8]*t_neg[2],
    ]

    noise = cal.get('imu_noise', {})

    return {
        'trans': ext_trans,
        'rot': R,
        'acc_n': noise.get('acc_n', defaults['acc_n']),
        'gyr_n': noise.get('gyr_n', defaults['gyr_n']),
        'acc_w': noise.get('acc_w', defaults['acc_w']),
        'gyr_w': noise.get('gyr_w', defaults['gyr_w']),
        'gravity': cal.get('gravity', defaults['gravity']),
    }


def generate_launch_description():
    ws_root = os.path.expanduser('~/nav_ws')
    extrinsics_path = os.path.join(ws_root, 'config', 'mapping', 'extrinsics.yaml')
    liorf_config = os.path.join(ws_root, 'config', 'liorf', 'liorf_spot_vlp16.yaml')

    ext = load_extrinsics(extrinsics_path)

    # Declare arguments
    declare_imu_mode = DeclareLaunchArgument(
        'imu_mode', default_value='six_axis')
    declare_pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic', default_value='/velodyne_points',
        description='Bag PointCloud2 input topic for replay')
    declare_imu_input_topic = DeclareLaunchArgument(
        'imu_input_topic', default_value='/imu',
        description='Bag IMU input topic for replay')
    declare_imu_output_topic = DeclareLaunchArgument(
        'imu_output_topic', default_value='/mapping/imu',
        description='Normalized IMU output topic for LiORF')
    declare_scan_output_topic = DeclareLaunchArgument(
        'scan_output_topic', default_value='/mapping/scan/nav',
        description='LaserScan output topic for replay helpers')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use sim time for bag replay')

    # Include mapping common infrastructure
    mapping_common_dir = get_package_share_directory('spot_mapping_common')
    mapping_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_common_dir, 'launch', 'mapping_common.launch.py')
        ),
        launch_arguments={
            'imu_mode': LaunchConfiguration('imu_mode'),
            'pointcloud_input_topic': LaunchConfiguration('pointcloud_topic'),
            'imu_input_topic': LaunchConfiguration('imu_input_topic'),
            'imu_output_topic': LaunchConfiguration('imu_output_topic'),
            'scan_output_topic': LaunchConfiguration('scan_output_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    liorf_overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'pointCloudTopic': LaunchConfiguration('pointcloud_topic'),
        'imuTopic': LaunchConfiguration('imu_output_topic'),
        'extrinsicTrans': ext['trans'],
        'extrinsicRot': ext['rot'],
        'imuAccNoise': ext['acc_n'],
        'imuGyrNoise': ext['gyr_n'],
        'imuAccBiasN': ext['acc_w'],
        'imuGyrBiasN': ext['gyr_w'],
        'imuGravity': ext['gravity'],
    }

    # LiORF node with extrinsics injected from canonical file
    liorf_node = Node(
        package='liorf',
        executable='liorf_mapOptmization',
        name='liorf_mapOptmization',
        parameters=[
            liorf_config,
            liorf_overrides,
        ],
        output='screen',
    )

    liorf_imu_node = Node(
        package='liorf',
        executable='liorf_imuPreintegration',
        name='liorf_imuPreintegration',
        parameters=[
            liorf_config,
            liorf_overrides,
        ],
        output='screen',
    )

    liorf_image_node = Node(
        package='liorf',
        executable='liorf_imageProjection',
        name='liorf_imageProjection',
        parameters=[
            liorf_config,
            liorf_overrides,
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_imu_mode,
        declare_pointcloud_topic,
        declare_imu_input_topic,
        declare_imu_output_topic,
        declare_scan_output_topic,
        declare_use_sim_time,
        mapping_common_launch,
        liorf_image_node,
        liorf_imu_node,
        liorf_node,
    ])
