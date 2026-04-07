import math
import os

import yaml


def _rpy_to_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    ]


def _transpose(m):
    return [
        m[0], m[3], m[6],
        m[1], m[4], m[7],
        m[2], m[5], m[8],
    ]


def _matvec(m, v):
    return [
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    ]


def _quat_from_matrix(m):
    trace = m[0] + m[4] + m[8]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return [
            (m[7] - m[5]) / s,
            (m[2] - m[6]) / s,
            (m[3] - m[1]) / s,
            0.25 * s,
        ]
    if m[0] > m[4] and m[0] > m[8]:
        s = math.sqrt(1.0 + m[0] - m[4] - m[8]) * 2.0
        return [
            0.25 * s,
            (m[1] + m[3]) / s,
            (m[2] + m[6]) / s,
            (m[7] - m[5]) / s,
        ]
    if m[4] > m[8]:
        s = math.sqrt(1.0 + m[4] - m[0] - m[8]) * 2.0
        return [
            (m[1] + m[3]) / s,
            0.25 * s,
            (m[5] + m[7]) / s,
            (m[2] - m[6]) / s,
        ]
    s = math.sqrt(1.0 + m[8] - m[0] - m[4]) * 2.0
    return [
        (m[2] + m[6]) / s,
        (m[5] + m[7]) / s,
        0.25 * s,
        (m[3] - m[1]) / s,
    ]


def _normalize_quaternion(q):
    norm = math.sqrt(sum(component * component for component in q))
    if norm == 0.0:
        return [0.0, 0.0, 0.0, 1.0]
    return [component / norm for component in q]


def load_backend_calibration(path):
    if not os.path.isfile(path):
        return {
            'body_to_imu': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'body_to_lidar': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'lidar_imu_time_offset': 0.0,
            'imu_noise': {'acc_n': 0.01, 'gyr_n': 0.005, 'acc_w': 0.0002, 'gyr_w': 0.00003},
            'gravity': 9.81,
            'frames': {'map': 'map', 'odom': 'odom', 'base_link': 'base_link', 'imu_link': 'imu_link', 'lidar': 'velodyne'},
        }

    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}

    return {
        'body_to_imu': data.get('body_to_imu', {}),
        'body_to_lidar': data.get('body_to_lidar', {}),
        'lidar_imu_time_offset': float(data.get('lidar_imu_time_offset', 0.0)),
        'imu_noise': data.get('imu_noise', {}),
        'gravity': float(data.get('gravity', 9.81)),
        'frames': data.get('frames', {}),
    }


def relative_imu_to_lidar(calibration):
    body_to_imu = calibration.get('body_to_imu', {})
    body_to_lidar = calibration.get('body_to_lidar', {})

    imu_rot = _rpy_to_matrix(
        float(body_to_imu.get('roll', 0.0)),
        float(body_to_imu.get('pitch', 0.0)),
        float(body_to_imu.get('yaw', 0.0)),
    )
    lidar_rot = _rpy_to_matrix(
        float(body_to_lidar.get('roll', 0.0)),
        float(body_to_lidar.get('pitch', 0.0)),
        float(body_to_lidar.get('yaw', 0.0)),
    )

    imu_t = [
        float(body_to_imu.get('x', 0.0)),
        float(body_to_imu.get('y', 0.0)),
        float(body_to_imu.get('z', 0.0)),
    ]
    lidar_t = [
        float(body_to_lidar.get('x', 0.0)),
        float(body_to_lidar.get('y', 0.0)),
        float(body_to_lidar.get('z', 0.0)),
    ]

    imu_rot_inv = _transpose(imu_rot)
    rot_imu_to_lidar = [
        lidar_rot[0] * imu_rot_inv[0] + lidar_rot[1] * imu_rot_inv[3] + lidar_rot[2] * imu_rot_inv[6],
        lidar_rot[0] * imu_rot_inv[1] + lidar_rot[1] * imu_rot_inv[4] + lidar_rot[2] * imu_rot_inv[7],
        lidar_rot[0] * imu_rot_inv[2] + lidar_rot[1] * imu_rot_inv[5] + lidar_rot[2] * imu_rot_inv[8],
        lidar_rot[3] * imu_rot_inv[0] + lidar_rot[4] * imu_rot_inv[3] + lidar_rot[5] * imu_rot_inv[6],
        lidar_rot[3] * imu_rot_inv[1] + lidar_rot[4] * imu_rot_inv[4] + lidar_rot[5] * imu_rot_inv[7],
        lidar_rot[3] * imu_rot_inv[2] + lidar_rot[4] * imu_rot_inv[5] + lidar_rot[5] * imu_rot_inv[8],
        lidar_rot[6] * imu_rot_inv[0] + lidar_rot[7] * imu_rot_inv[3] + lidar_rot[8] * imu_rot_inv[6],
        lidar_rot[6] * imu_rot_inv[1] + lidar_rot[7] * imu_rot_inv[4] + lidar_rot[8] * imu_rot_inv[7],
        lidar_rot[6] * imu_rot_inv[2] + lidar_rot[7] * imu_rot_inv[5] + lidar_rot[8] * imu_rot_inv[8],
    ]

    t_imu_to_lidar = [
        lidar_t[0] - imu_t[0],
        lidar_t[1] - imu_t[1],
        lidar_t[2] - imu_t[2],
    ]

    q_imu_to_lidar = _normalize_quaternion(_quat_from_matrix(rot_imu_to_lidar))

    return {
        'extrinsicTrans': t_imu_to_lidar,
        'extrinsicRot': rot_imu_to_lidar,
        'extrinsicRPY': rot_imu_to_lidar,
        'imuQuaternion': q_imu_to_lidar,
        'body_to_imu': body_to_imu,
        'body_to_lidar': body_to_lidar,
        'time_offset': float(calibration.get('lidar_imu_time_offset', 0.0)),
    }


def rpy_to_quaternion(roll, pitch, yaw):
    matrix = _rpy_to_matrix(roll, pitch, yaw)
    return _normalize_quaternion(_quat_from_matrix(matrix))


def static_transform_arguments(parent_frame, child_frame, transform):
    return [
        str(transform['x']),
        str(transform['y']),
        str(transform['z']),
        str(transform['qx']),
        str(transform['qy']),
        str(transform['qz']),
        str(transform['qw']),
        parent_frame,
        child_frame,
    ]


def transform_for_static_tf(transform):
    qx, qy, qz, qw = rpy_to_quaternion(
        float(transform.get('roll', 0.0)),
        float(transform.get('pitch', 0.0)),
        float(transform.get('yaw', 0.0)),
    )
    return {
        'x': float(transform.get('x', 0.0)),
        'y': float(transform.get('y', 0.0)),
        'z': float(transform.get('z', 0.0)),
        'qx': qx,
        'qy': qy,
        'qz': qz,
        'qw': qw,
    }
