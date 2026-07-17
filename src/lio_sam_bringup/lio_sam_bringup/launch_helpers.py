import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from .calibration import (
    load_backend_calibration,
    relative_imu_to_lidar,
    static_transform_arguments,
)


def workspace_root():
    return os.path.expanduser('~/dance_ws_pedestrian_tracking')


def package_share():
    return get_package_share_directory('lio_sam_bringup')


def config_path(name):
    return os.path.join(package_share(), 'config', name)


def backend_calibration():
    return load_backend_calibration(config_path('extrinsics.yaml'))


def backend_parameters():
    return {
        'common': config_path('lio_sam_common.yaml'),
        'replay': config_path('lio_sam_replay.yaml'),
        'live': config_path('lio_sam_live.yaml'),
        'vectornav': config_path('vectornav_vn100.yaml'),
    }


def lio_sam_overrides(calibration, use_sim_time=False, pointcloud_topic='/velodyne_points', imu_topic='/mapping/imu'):
    pose = relative_imu_to_lidar(calibration)
    frames = calibration.get('frames', {})
    return {
        'use_sim_time': use_sim_time,
        'pointCloudTopic': pointcloud_topic,
        'imuTopic': imu_topic,
        'lidarFrame': frames.get('lidar', 'velodyne'),
        'baselinkFrame': frames.get('base_link', 'base_link'),
        'odometryFrame': frames.get('odom', 'odom'),
        'mapFrame': frames.get('map', 'map'),
        'extrinsicTrans': pose['extrinsicTrans'],
        'extrinsicRot': pose['extrinsicRot'],
        'extrinsicRPY': pose['extrinsicRPY'],
        'imuAccNoise': float(calibration.get('imu_noise', {}).get('acc_n', 0.01)),
        'imuGyrNoise': float(calibration.get('imu_noise', {}).get('gyr_n', 0.005)),
        'imuAccBiasN': float(calibration.get('imu_noise', {}).get('acc_w', 0.0002)),
        'imuGyrBiasN': float(calibration.get('imu_noise', {}).get('gyr_w', 0.00003)),
        'imuGravity': float(calibration.get('gravity', 9.81)),
    }


def imu_normalizer_node(input_topic, output_topic, frame_id, mode, calibration, use_sim_time=False):
    noise = calibration.get('imu_noise', {})
    return Node(
        package='lio_sam_bringup',
        executable='lio_sam_imu_normalizer',
        name='lio_sam_imu_normalizer',
        output='screen',
        parameters=[{
            'input_topic': input_topic,
            'output_topic': output_topic,
            'frame_id': frame_id,
            'mode': mode,
            'time_offset_sec': float(calibration.get('lidar_imu_time_offset', 0.0)),
            'expected_rate_hz': 200.0,
            'gravity_nominal': float(calibration.get('gravity', 9.81)),
            'gravity_tolerance': 0.3,
            'orientation_covariance': [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
            'angular_velocity_covariance': [
                float(noise.get('gyr_n', 0.005)) ** 2, 0.0, 0.0,
                0.0, float(noise.get('gyr_n', 0.005)) ** 2, 0.0,
                0.0, 0.0, float(noise.get('gyr_n', 0.005)) ** 2,
            ],
            'linear_acceleration_covariance': [
                float(noise.get('acc_n', 0.01)) ** 2, 0.0, 0.0,
                0.0, float(noise.get('acc_n', 0.01)) ** 2, 0.0,
                0.0, 0.0, float(noise.get('acc_n', 0.01)) ** 2,
            ],
            'use_sim_time': use_sim_time,
        }],
    )


def static_tf_node(parent_frame, child_frame, transform):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{parent_frame}_to_{child_frame}_static_tf',
        arguments=static_transform_arguments(parent_frame, child_frame, transform),
        output='screen',
    )
