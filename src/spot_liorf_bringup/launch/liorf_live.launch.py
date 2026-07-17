"""
liorf_live.launch.py — Live mapping with Spot + VLP-16 + LiORF.

Launches:
  1. Velodyne driver
  2. mapping_common (validators, IMU adapter, scan extractor)
  3. liorf SLAM nodes
  4. Extrinsics injected from canonical file

Note: Replay (liorf_replay.launch.py) is the authoritative map path.
Live is for collection monitoring and quick preview only.
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
    tx, ty, tz = b2l.get('x', 0.0), b2l.get('y', 0.0), b2l.get('z', 0.5)
    roll, pitch, yaw = b2l.get('roll', 0.0), b2l.get('pitch', 0.0), b2l.get('yaw', 0.0)

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # R_body_lidar: rotation of lidar in body frame (matches URDF convention)
    R_b2l = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
             sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
             -sp, cp*sr, cp*cr]

    # liorf extrinsicRot needs R_lidar_body (inverse = transpose of R_body_lidar)
    # to rotate IMU (body-frame) readings into lidar frame
    R = [R_b2l[0], R_b2l[3], R_b2l[6],   # row 0 = col 0 of R_b2l
         R_b2l[1], R_b2l[4], R_b2l[7],   # row 1 = col 1 of R_b2l
         R_b2l[2], R_b2l[5], R_b2l[8]]   # row 2 = col 2 of R_b2l

    # liorf extrinsicTrans = translation from lidar to body, in lidar frame
    # = R_lidar_body * (-body_to_lidar_translation)
    t_neg = [-tx, -ty, -tz]
    ext_trans = [
        R[0]*t_neg[0] + R[1]*t_neg[1] + R[2]*t_neg[2],
        R[3]*t_neg[0] + R[4]*t_neg[1] + R[5]*t_neg[2],
        R[6]*t_neg[0] + R[7]*t_neg[1] + R[8]*t_neg[2],
    ]

    noise = cal.get('imu_noise', {})
    return {
        'trans': ext_trans, 'rot': R,
        'acc_n': noise.get('acc_n', defaults['acc_n']),
        'gyr_n': noise.get('gyr_n', defaults['gyr_n']),
        'acc_w': noise.get('acc_w', defaults['acc_w']),
        'gyr_w': noise.get('gyr_w', defaults['gyr_w']),
        'gravity': cal.get('gravity', defaults['gravity']),
        'body_to_lidar': {
            'x': tx,
            'y': ty,
            'z': tz,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
        },
    }


def generate_launch_description():
    ws_root = os.path.expanduser('~/dance_ws_pedestrian_tracking')
    extrinsics_path = os.path.join(ws_root, 'config', 'mapping', 'extrinsics.yaml')
    liorf_config = os.path.join(ws_root, 'config', 'liorf', 'liorf_spot_vlp16.yaml')

    ext = load_extrinsics(extrinsics_path)
    b2l = ext['body_to_lidar']

    cr, sr = math.cos(b2l['roll'] / 2.0), math.sin(b2l['roll'] / 2.0)
    cp, sp = math.cos(b2l['pitch'] / 2.0), math.sin(b2l['pitch'] / 2.0)
    cy, sy = math.cos(b2l['yaw'] / 2.0), math.sin(b2l['yaw'] / 2.0)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    declare_device_ip = DeclareLaunchArgument(
        'device_ip', default_value='192.168.50.201')
    declare_velodyne_rpm = DeclareLaunchArgument(
        'velodyne_rpm', default_value='600.0')
    declare_imu_mode = DeclareLaunchArgument(
        'imu_mode', default_value='six_axis')
    declare_pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic', default_value='/velodyne_points')
    declare_imu_input_topic = DeclareLaunchArgument(
        'imu_input_topic', default_value='/imu/data')
    declare_imu_output_topic = DeclareLaunchArgument(
        'imu_output_topic', default_value='/mapping/imu')
    declare_scan_output_topic = DeclareLaunchArgument(
        'scan_output_topic', default_value='/mapping/scan/nav')

    # Velodyne driver
    velodyne_dir = get_package_share_directory('spot_velodyne')
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(velodyne_dir, 'launch', 'velodyne.launch.py')
        ),
        launch_arguments={
            'device_ip': LaunchConfiguration('device_ip'),
            'rpm': LaunchConfiguration('velodyne_rpm'),
        }.items(),
    )

    # Mapping common
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
        }.items(),
    )

    body_to_velodyne_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_velodyne_tf',
        arguments=[
            str(b2l['x']),
            str(b2l['y']),
            str(b2l['z']),
            str(qx),
            str(qy),
            str(qz),
            str(qw),
            'body',
            'velodyne',
        ],
        output='screen',
    )

    common_params = [
        liorf_config,
        {
            'pointCloudTopic': LaunchConfiguration('pointcloud_topic'),
            'imuTopic': LaunchConfiguration('imu_output_topic'),
            'baselinkFrame': 'body',
            'extrinsicTrans': ext['trans'],
            'extrinsicRot': ext['rot'],
            'imuAccNoise': ext['acc_n'],
            'imuGyrNoise': ext['gyr_n'],
            'imuAccBiasN': ext['acc_w'],
            'imuGyrBiasN': ext['gyr_w'],
            'imuGravity': ext['gravity'],
        },
    ]

    liorf_image = Node(
        package='liorf', executable='liorf_imageProjection',
        name='liorf_imageProjection', parameters=common_params, output='screen')
    liorf_imu = Node(
        package='liorf', executable='liorf_imuPreintegration',
        name='liorf_imuPreintegration', parameters=common_params, output='screen')
    liorf_map = Node(
        package='liorf', executable='liorf_mapOptmization',
        name='liorf_mapOptmization', parameters=common_params, output='screen')

    return LaunchDescription([
        declare_device_ip,
        declare_velodyne_rpm,
        declare_imu_mode,
        declare_pointcloud_topic,
        declare_imu_input_topic,
        declare_imu_output_topic,
        declare_scan_output_topic,
        velodyne_launch,
        mapping_common_launch,
        body_to_velodyne_tf,
        liorf_image,
        liorf_imu,
        liorf_map,
    ])
