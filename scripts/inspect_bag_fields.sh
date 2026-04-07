#!/usr/bin/env bash
# inspect_bag_fields.sh — Inspect a ROS2 bag for mapping-critical topic health.
# Checks point cloud fields, IMU rates, timestamps, and TF connectivity.
#
# Usage: ./scripts/inspect_bag_fields.sh <bag_directory>
# Example: ./scripts/inspect_bag_fields.sh bags/raw/2026-03-26_run1
set -euo pipefail

RED='\033[0;31m'; GRN='\033[0;32m'; YEL='\033[0;33m'; RST='\033[0m'
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <bag_directory>"
    exit 1
fi

BAG_DIR="$1"
if [[ ! -d "$BAG_DIR" ]]; then
    echo -e "${RED}Error: $BAG_DIR is not a directory${RST}"
    exit 1
fi

echo "===== Bag Inspection: $BAG_DIR ====="
echo ""

# --- 1. Bag info ---
echo "--- Bag Metadata ---"
if command -v ros2 >/dev/null 2>&1; then
    ros2 bag info "$BAG_DIR"
else
    echo "ros2 is not available in this shell; using metadata.yaml directly."
    sed -n '1,120p' "$BAG_DIR/metadata.yaml"
fi
echo ""

# --- 2. Point cloud field inspection ---
echo "--- PointCloud2 Field Inspection ---"
EXPECTED_FIELDS="x y z intensity ring time"

# Play first 2 seconds and capture one message
python3 - "$BAG_DIR" <<'PYEOF'
import sys
import sqlite3
import os
import struct
import yaml

bag_dir = sys.argv[1]

# Find the db3 file
db3_files = [f for f in os.listdir(bag_dir) if f.endswith('.db3')]
if not db3_files:
    print("  No .db3 file found in bag directory")
    sys.exit(1)

db_path = os.path.join(bag_dir, db3_files[0])
meta_path = os.path.join(bag_dir, 'metadata.yaml')

# Read metadata for topic-to-type mapping
with open(meta_path) as f:
    meta = yaml.safe_load(f)

topics_with_type = {}
for entry in meta.get('rosbag2_bagfile_information', {}).get('topics_with_message_count', []):
    info = entry.get('topic_metadata', {})
    topics_with_type[info.get('name', '')] = {
        'type': info.get('type', ''),
        'count': entry.get('message_count', 0)
    }

# Report topics
print("  Topics in bag:")
for topic, info in sorted(topics_with_type.items()):
    print(f"    {topic}: {info['type']} ({info['count']} msgs)")

preferred_pc_topics = ['/velodyne_points', '/velodyne_points_upstream']
preferred_imu_topics = ['/vectornav/imu', '/vectornav/imu_upstream', '/imu', '/mapping/imu']

def choose_topic(candidates, type_suffix):
    for topic in candidates:
        info = topics_with_type.get(topic)
        if info and info['type'].endswith(type_suffix) and info['count'] > 0:
            return topic, info['count'], 'preferred_nonzero'
    for topic in candidates:
        info = topics_with_type.get(topic)
        if info and info['type'].endswith(type_suffix):
            return topic, info['count'], 'preferred_zero'
    matches = [
        (name, info['count'])
        for name, info in topics_with_type.items()
        if info['type'].endswith(type_suffix)
    ]
    if not matches:
        return '', 0, 'missing'
    matches.sort(key=lambda item: (-item[1], item[0]))
    topic, count = matches[0]
    return topic, count, 'fallback_nonzero' if count > 0 else 'fallback_zero'

# Check for expected topics
pc_topics = [t for t, i in topics_with_type.items()
             if 'PointCloud2' in i['type']]
imu_topics = [t for t, i in topics_with_type.items()
              if 'Imu' in i['type']]
tf_topics = [t for t in topics_with_type if t in ['/tf', '/tf_static']]
selected_pc_topic, selected_pc_count, selected_pc_reason = choose_topic(preferred_pc_topics, 'PointCloud2')
selected_imu_topic, selected_imu_count, selected_imu_reason = choose_topic(preferred_imu_topics, 'Imu')

print()
if pc_topics:
    print(f"  PointCloud2 topics: {pc_topics}")
else:
    print("  WARNING: No PointCloud2 topics found!")

if imu_topics:
    print(f"  IMU topics: {imu_topics}")
else:
    print("  WARNING: No IMU topics found!")

if tf_topics:
    print(f"  TF topics: {tf_topics}")
else:
    print("  NOTE: No TF topics found. This is expected for minimal raw LiDAR+IMU bags")
    print("        that rely on replay launch files to publish static transforms later.")

if selected_pc_topic:
    print(f"  Recommended replay pointcloud topic: {selected_pc_topic} ({selected_pc_count} msgs; {selected_pc_reason})")
if selected_imu_topic:
    print(f"  Recommended replay IMU topic: {selected_imu_topic} ({selected_imu_count} msgs; {selected_imu_reason})")

final_points = topics_with_type.get('/velodyne_points', {}).get('count', 0)
upstream_points = topics_with_type.get('/velodyne_points_upstream', {}).get('count', 0)
packet_count = topics_with_type.get('/velodyne_packets', {}).get('count', 0)
if final_points == 0 and upstream_points > 0:
    print()
    print(f"  WARNING: /velodyne_points has 0 messages while /velodyne_points_upstream has {upstream_points}.")
    print("           Default LIO-SAM replay on /velodyne_points will not work for this bag.")
    print("           Use the fallback topic or run replay_mapping_bag.sh after updating it.")
elif not pc_topics and packet_count > 0:
    print()
    print(f"  NOTE: No PointCloud2 topics were recorded, but /velodyne_packets has {packet_count} messages.")
    print("        replay_mapping_bag.sh can now regenerate /velodyne_points for LIO-SAM replay.")

# Try to read first PointCloud2 message to inspect fields
if pc_topics:
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    topic_to_inspect = selected_pc_topic or pc_topics[0]

    # Get topic_id for the selected pointcloud topic
    cursor.execute("SELECT id, name FROM topics WHERE name = ?", (topic_to_inspect,))
    row = cursor.fetchone()
    if row:
        topic_id = row[0]
        cursor.execute(
            "SELECT data, timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp LIMIT 1",
            (topic_id,)
        )
        msg_row = cursor.fetchone()
        if msg_row:
            print(f"\n  First PointCloud2 message on {topic_to_inspect}:")
            print(f"    Timestamp: {msg_row[1]} ns")
            # Note: Full CDR deserialization would require rclpy;
            # for a quick script check, we report the raw size.
            print(f"    Raw message size: {len(msg_row[0])} bytes")
    conn.close()

# Report IMU message counts for rate estimation
if imu_topics:
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    for imu_topic in imu_topics:
        cursor.execute("SELECT id FROM topics WHERE name = ?", (imu_topic,))
        row = cursor.fetchone()
        if row:
            topic_id = row[0]
            cursor.execute(
                "SELECT MIN(timestamp), MAX(timestamp), COUNT(*) FROM messages WHERE topic_id = ?",
                (topic_id,)
            )
            ts_row = cursor.fetchone()
            if ts_row and ts_row[2] > 1:
                duration_s = (ts_row[1] - ts_row[0]) / 1e9
                rate = ts_row[2] / duration_s if duration_s > 0 else 0
                print(f"\n  IMU topic {imu_topic}:")
                print(f"    Messages: {ts_row[2]}")
                print(f"    Duration: {duration_s:.2f}s")
                print(f"    Avg rate: {rate:.1f} Hz")
                if rate < 200:
                    print(f"    WARNING: Rate {rate:.1f} Hz < 200 Hz threshold")
    conn.close()

print("\n  Field inspection complete.")
PYEOF

echo ""
echo "--- Recommended Replay Topics ---"
python3 "${SCRIPT_DIR}/select_mapping_bag_topics.py" "$BAG_DIR" --backend lio_sam || true
echo ""
echo "--- Inspection Done ---"
echo "Next steps:"
echo "  1. Verify PointCloud2 has fields: x, y, z, intensity, ring, time"
echo "  2. Verify IMU rate >= 200 Hz"
echo "  3. If this is a minimal raw bag, missing TF topics are OK"
echo "  4. Run: ./scripts/replay_mapping_bag.sh $BAG_DIR --rate 0.5 --backend lio_sam"
echo "     The replay helper will auto-fallback to /velodyne_points_upstream or regenerate /velodyne_points from /velodyne_packets when needed."
