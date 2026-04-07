"""Live hardware launch for the ROS 2 LIO-SAM backend."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from spot_mapping_common.calibration import flatten_matrix, load_mapping_calibration


def _static_tf_node(name, parent_frame, child_frame, transform, condition=None):
    qx, qy, qz, qw = transform['quaternion']
    tx, ty, tz = transform['translation']
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        condition=condition,
        arguments=[
            str(tx), str(ty), str(tz),
            str(qx), str(qy), str(qz), str(qw),
            parent_frame,
            child_frame,
        ],
        output='screen',
    )


def generate_launch_description():
    ws_root = os.path.expanduser('~/nav_ws')
    extrinsics_path = os.path.join(ws_root, 'config', 'mapping', 'extrinsics.yaml')
    lio_sam_config = os.path.join(ws_root, 'config', 'lio_sam', 'lio_sam_vn100_vlp16.yaml')
    vectornav_config = os.path.join(ws_root, 'config', 'lio_sam', 'vectornav_vn100.yaml')
    raw_vectornav_imu_topic = '/vectornav/imu_upstream'
    calibration = load_mapping_calibration(extrinsics_path)
    frames = calibration['frames']

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
        default_value='115200',
    )
    declare_imu_mode = DeclareLaunchArgument(
        'imu_mode',
        default_value='use_quaternion',
    )
    declare_pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/velodyne_points',
    )
    declare_imu_input_topic = DeclareLaunchArgument(
        'imu_input_topic',
        default_value='/vectornav/imu',
    )
    declare_imu_output_topic = DeclareLaunchArgument(
        'imu_output_topic',
        default_value='/mapping/imu',
    )
    declare_scan_output_topic = DeclareLaunchArgument(
        'scan_output_topic',
        default_value='/scan',
    )
    declare_scan_frame = DeclareLaunchArgument(
        'scan_frame',
        default_value=frames['base'],
    )
    declare_imu_expected_rate = DeclareLaunchArgument(
        'imu_expected_rate_hz',
        default_value='800.0',
    )
    declare_publish_flat_body_tf = DeclareLaunchArgument(
        'publish_flat_body_tf',
        default_value='false',
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
            'output_topic': LaunchConfiguration('imu_input_topic'),
            'message_type': 'imu',
            'depth': 400,
        }],
        output='screen',
    )

    mapping_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('spot_mapping_common'),
                'launch',
                'mapping_common.launch.py',
            )
        ),
        launch_arguments={
            'imu_source': 'generic',
            'imu_mode': LaunchConfiguration('imu_mode'),
            'pointcloud_input_topic': LaunchConfiguration('pointcloud_topic'),
            'imu_input_topic': LaunchConfiguration('imu_input_topic'),
            'imu_output_topic': LaunchConfiguration('imu_output_topic'),
            'imu_output_frame': frames['imu'],
            'imu_expected_rate_hz': LaunchConfiguration('imu_expected_rate_hz'),
            'extrinsics_file': extrinsics_path,
            'scan_frame': LaunchConfiguration('scan_frame'),
            'scan_output_topic': LaunchConfiguration('scan_output_topic'),
            'scan_transform_timeout_sec': '0.1',
            'enable_timing_monitor': 'true',
        }.items(),
    )

    base_to_lidar_tf = _static_tf_node(
        'base_to_velodyne_tf',
        frames['base'],
        frames['lidar'],
        calibration['base_to_lidar'],
    )
    base_to_imu_tf = _static_tf_node(
        'base_to_imu_tf',
        frames['base'],
        frames['imu'],
        calibration['base_to_imu'],
    )
    flat_body_tf = _static_tf_node(
        'flat_body_to_base_link_tf',
        frames['nav_compat_base'],
        frames['base'],
        {
            'translation': [0.0, 0.0, 0.0],
            'quaternion': [0.0, 0.0, 0.0, 1.0],
        },
        condition=IfCondition(LaunchConfiguration('publish_flat_body_tf')),
    )

    common_parameters = [
        lio_sam_config,
        {
            'pointCloudTopic': LaunchConfiguration('pointcloud_topic'),
            'imuTopic': LaunchConfiguration('imu_output_topic'),
            'lidarFrame': frames['lidar'],
            'baselinkFrame': frames['base'],
            'odometryFrame': frames['odom'],
            'mapFrame': frames['map'],
            'extrinsicTrans': calibration['lidar_to_imu']['translation'],
            'extrinsicRot': flatten_matrix(calibration['lidar_to_imu']['rotation_matrix']),
            'extrinsicRPY': flatten_matrix(calibration['lidar_to_imu']['rotation_matrix']),
            'imuAccNoise': calibration['imu_noise']['acc_n'],
            'imuGyrNoise': calibration['imu_noise']['gyr_n'],
            'imuAccBiasN': calibration['imu_noise']['acc_w'],
            'imuGyrBiasN': calibration['imu_noise']['gyr_w'],
            'imuGravity': calibration['gravity'],
        },
    ]

    imu_preintegration = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=common_parameters,
        output='screen',
    )
    image_projection = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=common_parameters,
        output='screen',
    )
    feature_extraction = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=common_parameters,
        output='screen',
    )
    map_optimization = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=common_parameters,
        output='screen',
    )

    return LaunchDescription([
        declare_device_ip,
        declare_velodyne_rpm,
        declare_vectornav_port,
        declare_vectornav_baud,
        declare_imu_mode,
        declare_pointcloud_topic,
        declare_imu_input_topic,
        declare_imu_output_topic,
        declare_scan_output_topic,
        declare_scan_frame,
        declare_imu_expected_rate,
        declare_publish_flat_body_tf,
        velodyne_launch,
        vectornav_node,
        vectornav_sensor_msgs,
        vectornav_imu_relay,
        mapping_common_launch,
        base_to_lidar_tf,
        base_to_imu_tf,
        flat_body_tf,
        image_projection,
        feature_extraction,
        imu_preintegration,
        map_optimization,
    ])
