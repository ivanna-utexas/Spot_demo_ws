#!/usr/bin/env python3
"""Helpers for loading and converting canonical mapping calibration."""

import math
import os

import yaml


DEFAULT_FRAMES = {
    'map': 'map',
    'odom': 'odom',
    'base': 'base_link',
    'imu': 'imu_link',
    'lidar': 'velodyne',
    'nav_compat_base': 'flat_body',
}

DEFAULT_IMU_NOISE = {
    'acc_n': 0.01,
    'gyr_n': 0.005,
    'acc_w': 0.0002,
    'gyr_w': 0.00003,
}


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def flatten_matrix(matrix):
    return [value for row in matrix for value in row]


def transpose_matrix(matrix):
    return [
        [matrix[0][0], matrix[1][0], matrix[2][0]],
        [matrix[0][1], matrix[1][1], matrix[2][1]],
        [matrix[0][2], matrix[1][2], matrix[2][2]],
    ]


def rotate_vector(matrix, vector):
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]


def multiply_matrices(lhs, rhs):
    return [
        [
            lhs[row][0] * rhs[0][col] + lhs[row][1] * rhs[1][col] + lhs[row][2] * rhs[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def invert_transform(transform):
    rotation_t = transpose_matrix(transform['rotation_matrix'])
    translation = rotate_vector(rotation_t, [-value for value in transform['translation']])
    return {
        'translation': translation,
        'rotation_matrix': rotation_t,
        'quaternion': quaternion_from_rpy(0.0, 0.0, 0.0),
    }


def compose_transforms(lhs, rhs):
    rotation = multiply_matrices(lhs['rotation_matrix'], rhs['rotation_matrix'])
    rhs_translation = rotate_vector(lhs['rotation_matrix'], rhs['translation'])
    translation = [
        lhs['translation'][0] + rhs_translation[0],
        lhs['translation'][1] + rhs_translation[1],
        lhs['translation'][2] + rhs_translation[2],
    ]
    return {
        'translation': translation,
        'rotation_matrix': rotation,
        'quaternion': quaternion_from_rpy(0.0, 0.0, 0.0),
    }


def _expand_covariance(raw_value, fallback_diag):
    if isinstance(raw_value, list):
        if len(raw_value) == 9:
            return [float(value) for value in raw_value]
        if len(raw_value) == 3:
            return [
                float(raw_value[0]), 0.0, 0.0,
                0.0, float(raw_value[1]), 0.0,
                0.0, 0.0, float(raw_value[2]),
            ]
    return [
        float(fallback_diag[0]), 0.0, 0.0,
        0.0, float(fallback_diag[1]), 0.0,
        0.0, 0.0, float(fallback_diag[2]),
    ]


def _transform_from_section(section):
    x = float(section.get('x', 0.0))
    y = float(section.get('y', 0.0))
    z = float(section.get('z', 0.0))
    roll = float(section.get('roll', 0.0))
    pitch = float(section.get('pitch', 0.0))
    yaw = float(section.get('yaw', 0.0))
    return {
        'translation': [x, y, z],
        'rotation_matrix': rpy_to_matrix(roll, pitch, yaw),
        'quaternion': quaternion_from_rpy(roll, pitch, yaw),
        'rpy': [roll, pitch, yaw],
    }


def load_mapping_calibration(path=None):
    path = path or os.path.expanduser('~/nav_ws/config/mapping/extrinsics.yaml')
    raw = {}
    if os.path.isfile(path):
        with open(path, encoding='utf-8') as stream:
            raw = yaml.safe_load(stream) or {}

    frames = dict(DEFAULT_FRAMES)
    frames.update(raw.get('frames', {}))

    base_to_lidar = _transform_from_section(
        raw.get('base_to_lidar') or raw.get('body_to_lidar') or {}
    )
    base_to_imu = _transform_from_section(
        raw.get('base_to_imu') or raw.get('body_to_imu') or {}
    )
    lidar_to_imu = compose_transforms(invert_transform(base_to_lidar), base_to_imu)

    imu_noise = dict(DEFAULT_IMU_NOISE)
    imu_noise.update(raw.get('imu_noise', {}))

    imu_covariance = raw.get('imu_covariance', {})
    angular_diag = [imu_noise['gyr_n'] ** 2] * 3
    linear_diag = [imu_noise['acc_n'] ** 2] * 3

    return {
        'path': path,
        'frames': frames,
        'base_to_lidar': base_to_lidar,
        'base_to_imu': base_to_imu,
        'lidar_to_imu': lidar_to_imu,
        'imu_noise': imu_noise,
        'imu_covariance': {
            'orientation': _expand_covariance(
                imu_covariance.get('orientation'),
                [0.02, 0.02, 0.04],
            ),
            'angular_velocity': _expand_covariance(
                imu_covariance.get('angular_velocity'),
                angular_diag,
            ),
            'linear_acceleration': _expand_covariance(
                imu_covariance.get('linear_acceleration'),
                linear_diag,
            ),
        },
        'time_offset_lidar_imu': float(
            raw.get('time_offset_lidar_imu', raw.get('lidar_to_imu_time_offset', 0.0))
        ),
        'gravity': float(raw.get('gravity', 9.81)),
    }
