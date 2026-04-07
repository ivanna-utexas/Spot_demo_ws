"""Helpers for loading canonical mapping extrinsics."""

from __future__ import annotations

import math
import os
from typing import Dict, List

import yaml


DEFAULT_FRAMES = {
    'map': 'map',
    'odom': 'odom',
    'base': 'base_link',
    'imu': 'imu_link',
    'lidar': 'velodyne',
    'nav_compat': 'flat_body',
    'nav_compat_base': 'flat_body',
}

DEFAULT_TRANSFORM = {
    'x': 0.0,
    'y': 0.0,
    'z': 0.0,
    'roll': 0.0,
    'pitch': 0.0,
    'yaw': 0.0,
}


def _transform_dict(data: Dict, keys: List[str]) -> Dict[str, float]:
    for key in keys:
        raw = data.get(key)
        if isinstance(raw, dict):
            merged = dict(DEFAULT_TRANSFORM)
            for name in DEFAULT_TRANSFORM:
                merged[name] = float(raw.get(name, merged[name]))
            return merged
    return dict(DEFAULT_TRANSFORM)


def load_mapping_extrinsics(path: str) -> Dict:
    """Load mapping calibration with legacy-key fallbacks."""
    payload: Dict = {}
    if path and os.path.isfile(path):
        with open(path, encoding='utf-8') as handle:
            payload = yaml.safe_load(handle) or {}

    frames = dict(DEFAULT_FRAMES)
    frames.update(payload.get('frames', {}) or {})

    imu_noise = {
        'acc_n': 0.01,
        'gyr_n': 0.005,
        'acc_w': 0.0002,
        'gyr_w': 0.00003,
    }
    imu_noise.update(payload.get('imu_noise', {}) or {})

    timing = payload.get('timing', {}) or {}

    return {
        'frames': frames,
        'base_to_lidar': _transform_dict(
            payload, ['base_link_to_velodyne', 'base_to_lidar', 'body_to_lidar']
        ),
        'base_to_imu': _transform_dict(
            payload, ['base_link_to_imu', 'base_to_imu', 'body_to_imu']
        ),
        'imu_noise': imu_noise,
        'gravity': float(payload.get('gravity', 9.81)),
        'time_offset_lidar_imu': float(
            timing.get(
                'lidar_to_imu_offset',
                payload.get('time_offset_lidar_imu', payload.get('lidar_to_imu_time_offset', 0.0)),
            )
        ),
        'raw': payload,
    }


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    """Return a parent<-child rotation matrix from ROS-style RPY."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def transpose_3x3(matrix: List[List[float]]) -> List[List[float]]:
    return [[matrix[j][i] for j in range(3)] for i in range(3)]


def matmul_3x3(lhs: List[List[float]], rhs: List[List[float]]) -> List[List[float]]:
    return [
        [sum(lhs[i][k] * rhs[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def matvec_3x3(matrix: List[List[float]], vector: List[float]) -> List[float]:
    return [sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3)]


def flatten_row_major(matrix: List[List[float]]) -> List[float]:
    return [value for row in matrix for value in row]


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> List[float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def transform_translation(transform: Dict[str, float]) -> List[float]:
    return [transform['x'], transform['y'], transform['z']]


def imu_to_lidar_extrinsics(calibration: Dict) -> Dict[str, List[float]]:
    """Return IMU->lidar rotation and translation for LIO-SAM-style configs."""
    base_to_lidar = calibration['base_to_lidar']
    base_to_imu = calibration['base_to_imu']

    rot_base_lidar = rpy_to_matrix(
        base_to_lidar['roll'], base_to_lidar['pitch'], base_to_lidar['yaw']
    )
    rot_base_imu = rpy_to_matrix(
        base_to_imu['roll'], base_to_imu['pitch'], base_to_imu['yaw']
    )

    rot_lidar_base = transpose_3x3(rot_base_lidar)
    rot_lidar_imu = matmul_3x3(rot_lidar_base, rot_base_imu)

    delta_base = [
        base_to_imu['x'] - base_to_lidar['x'],
        base_to_imu['y'] - base_to_lidar['y'],
        base_to_imu['z'] - base_to_lidar['z'],
    ]

    return {
        'rotation': flatten_row_major(rot_lidar_imu),
        'translation': matvec_3x3(rot_lidar_base, delta_base),
    }
