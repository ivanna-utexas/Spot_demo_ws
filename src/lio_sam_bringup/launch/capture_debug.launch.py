"""Minimal capture-path launch for Velodyne + VectorNav + raw bag recording."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from spot_mapping_common.calibration import load_mapping_calibration


def generate_launch_description():
    ws_root = os.path.expanduser('~/dance_ws_pedestrian_tracking')
    extrinsics_path = os.path.join(ws_root, 'config', 'mapping', 'extrinsics.yaml')
    vectornav_config = os.path.join(ws_root, 'config', 'lio_sam', 'vectornav_vn100.yaml')
    raw_vectornav_imu_topic = '/vectornav/imu_upstream'
    record_script = os.path.join(ws_root, 'scripts', 'record_lio_sam_raw_bag.sh')
    default_qos_overrides = os.path.join(
        ws_root, 'config', 'rosbag2', 'lio_sam_capture_debug_qos.yaml'
    )
    frames = load_mapping_calibration(extrinsics_path)['frames']

    declare_device_ip = DeclareLaunchArgument(
        'device_ip',
        default_value='192.168.50.201',
    )
    declare_velodyne_rpm = DeclareLaunchArgument(
        'velodyne_rpm',
        default_value='600.0',
    )
    declare_vectornav_port = DeclareLaunchArgument(
        'vectornav_port',
        default_value='/dev/ttyUSB0',
    )
    declare_vectornav_baud = DeclareLaunchArgument(
        'vectornav_baud',
        default_value='921600',
    )
    declare_bag_name = DeclareLaunchArgument(
        'bag_name',
        default_value='capture_debug',
        description='Suffix for bags/raw/<date>_<bag_name>',
    )
    declare_estimate_gb = DeclareLaunchArgument(
        'estimate_gb',
        default_value='10',
    )
    declare_qos_overrides = DeclareLaunchArgument(
        'qos_overrides',
        default_value=default_qos_overrides,
    )
    declare_vectornav_imu_topic = DeclareLaunchArgument(
        'vectornav_imu_topic',
        default_value='/vectornav/imu',
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('spot_velodyne'),
                'launch',
                'velodyne.launch.py',
            )
        ),
        launch_arguments={
            'device_ip': LaunchConfiguration('device_ip'),
            'rpm': LaunchConfiguration('velodyne_rpm'),
            'enable_pointcloud': 'false',
        }.items(),
    )

    vectornav_node = Node(
        package='vectornav',
        executable='vectornav',
        name='vectornav',
        parameters=[
            vectornav_config,
            {
                'port': LaunchConfiguration('vectornav_port'),
                'baud': LaunchConfiguration('vectornav_baud'),
                'frame_id': frames['imu'],
            },
        ],
        output='screen',
    )
    vectornav_sensor_msgs = Node(
        package='vectornav',
        executable='vn_sensor_msgs',
        name='vectornav_sensor_msgs',
        parameters=[vectornav_config],
        remappings=[
            ('vectornav/imu', raw_vectornav_imu_topic),
            ('/vectornav/imu', raw_vectornav_imu_topic),
        ],
        output='screen',
    )
    vectornav_imu_relay = Node(
        package='spot_mapping_common',
        executable='best_effort_topic_relay_node',
        name='vectornav_imu_best_effort_relay',
        parameters=[{
            'input_topic': raw_vectornav_imu_topic,
            'output_topic': LaunchConfiguration('vectornav_imu_topic'),
            'message_type': 'imu',
            'depth': 400,
        }],
        output='screen',
    )

    record_process = ExecuteProcess(
        cmd=[
            record_script,
            '--name',
            LaunchConfiguration('bag_name'),
            '--estimate-gb',
            LaunchConfiguration('estimate_gb'),
            '--qos-overrides',
            LaunchConfiguration('qos_overrides'),
        ],
        cwd=ws_root,
        output='screen',
        emulate_tty=True,
        on_exit=Shutdown(reason='raw bag recorder exited'),
    )

    return LaunchDescription([
        declare_device_ip,
        declare_velodyne_rpm,
        declare_vectornav_port,
        declare_vectornav_baud,
        declare_bag_name,
        declare_estimate_gb,
        declare_qos_overrides,
        declare_vectornav_imu_topic,
        velodyne_launch,
        vectornav_node,
        vectornav_sensor_msgs,
        vectornav_imu_relay,
        record_process,
    ])
