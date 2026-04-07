#!/usr/bin/env python3
"""Select usable point cloud and IMU topics from a mapping bag's metadata."""

import argparse
import os
import sys

import yaml


BACKEND_CANDIDATES = {
    'lio_sam': {
        'pointcloud': ['/velodyne_points', '/velodyne_points_upstream'],
        'imu': ['/vectornav/imu', '/vectornav/imu_upstream', '/imu', '/mapping/imu'],
        'packets': ['/velodyne_packets', '/velodyne_packets_upstream'],
    },
    'liorf': {
        'pointcloud': ['/velodyne_points', '/velodyne_points_upstream'],
        'imu': ['/imu', '/vectornav/imu', '/vectornav/imu_upstream', '/mapping/imu'],
        'packets': ['/velodyne_packets', '/velodyne_packets_upstream'],
    },
}


def load_topics(bag_dir):
    meta_path = os.path.join(bag_dir, 'metadata.yaml')
    if not os.path.isfile(meta_path):
        raise FileNotFoundError(f'metadata.yaml not found in bag directory: {bag_dir}')

    with open(meta_path, 'r', encoding='utf-8') as handle:
        metadata = yaml.safe_load(handle) or {}

    topics = {}
    entries = metadata.get('rosbag2_bagfile_information', {}).get('topics_with_message_count', [])
    for entry in entries:
        info = entry.get('topic_metadata', {}) or {}
        name = info.get('name')
        if not name:
            continue
        topics[name] = {
            'type': info.get('type', ''),
            'count': int(entry.get('message_count', 0)),
        }
    return topics


def pick_topic(topics, override, candidates, type_suffix):
    if override:
        info = topics.get(override, {})
        return override, int(info.get('count', 0)), 'override'

    for topic in candidates:
        info = topics.get(topic)
        if info and info['type'].endswith(type_suffix) and info['count'] > 0:
            return topic, info['count'], 'preferred_nonzero'

    for topic in candidates:
        info = topics.get(topic)
        if info and info['type'].endswith(type_suffix):
            return topic, info['count'], 'preferred_zero'

    matches = [
        (name, info['count'])
        for name, info in topics.items()
        if info['type'].endswith(type_suffix)
    ]
    if not matches:
        return '', 0, 'missing'

    matches.sort(key=lambda item: (-item[1], item[0]))
    topic, count = matches[0]
    reason = 'fallback_nonzero' if count > 0 else 'fallback_zero'
    return topic, count, reason


def main():
    parser = argparse.ArgumentParser(
        description='Select usable point cloud and IMU topics from a bag metadata.yaml'
    )
    parser.add_argument('bag_dir', help='Path to ROS bag directory')
    parser.add_argument('--backend', choices=sorted(BACKEND_CANDIDATES), default='lio_sam')
    parser.add_argument('--pointcloud-topic', default='')
    parser.add_argument('--imu-topic', default='')
    parser.add_argument('--packet-topic', default='')
    parser.add_argument('--format', choices=['text', 'tsv'], default='text')
    args = parser.parse_args()

    try:
        topics = load_topics(args.bag_dir)
    except (FileNotFoundError, yaml.YAMLError) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1

    candidates = BACKEND_CANDIDATES[args.backend]
    pointcloud_topic, pointcloud_count, pointcloud_reason = pick_topic(
        topics,
        args.pointcloud_topic.strip(),
        candidates['pointcloud'],
        'PointCloud2',
    )
    imu_topic, imu_count, imu_reason = pick_topic(
        topics,
        args.imu_topic.strip(),
        candidates['imu'],
        'Imu',
    )
    packet_topic, packet_count, packet_reason = pick_topic(
        topics,
        args.packet_topic.strip(),
        candidates['packets'],
        'VelodyneScan',
    )

    if args.format == 'tsv':
        print(
            '\t'.join([
                pointcloud_topic,
                str(pointcloud_count),
                pointcloud_reason,
                imu_topic,
                str(imu_count),
                imu_reason,
                packet_topic,
                str(packet_count),
                packet_reason,
            ])
        )
        return 0

    print(
        f'pointcloud_topic={pointcloud_topic or "<missing>"} '
        f'({pointcloud_count} msgs; {pointcloud_reason})'
    )
    print(f'imu_topic={imu_topic or "<missing>"} ({imu_count} msgs; {imu_reason})')
    print(f'packet_topic={packet_topic or "<missing>"} ({packet_count} msgs; {packet_reason})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
