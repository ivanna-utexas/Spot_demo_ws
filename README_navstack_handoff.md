# Nav Stack Handoff — ROS1 Offline Validation

## Overview

This document specifies the offline procedure for validating the final 2D map
(`.pgm + .yaml`) in a ROS1 Noetic environment. **No live ROS1/ROS2 bridge is
used.** All validation runs outside the Humble container.

## Prerequisites

- Ubuntu 20.04 workstation or container with ROS Noetic installed
- Final map files from `maps/final/`:
  - `map.pgm` — Occupancy grid image
  - `map.yaml` — Map metadata
- Prerecorded LaserScan bag (produced offline, see below)
- No network connection to Spot or the Orin required

## Producing the LaserScan Bag

On the Humble system (inside Docker on the Orin or a Humble workstation):

```bash
# 1. Source the workspace
source ~/nav_ws/install/setup.bash

# 2. Launch the scan extractor
ros2 run spot_mapping_common scan_extractor_node \
    --ros-args \
    -p input_topic:=/velodyne_points \
    -p output_topic:=/mapping/scan/nav \
    -p frame_id:=flat_body \
    -p min_height:=-0.1 \
    -p max_height:=0.5

# 3. In another terminal, replay the raw bag and record only the scan
ros2 bag play bags/raw/<date_run>/ --clock --rate 1.0 &
ros2 bag record /mapping/scan/nav /tf /tf_static \
    -o bags/replay/<date_run>_scan_only \
    -s sqlite3

# 4. Convert to ROS1 bag format (requires rosbags or rosbag_tools)
# Option A: Using the 'rosbags' Python package (pip install rosbags)
rosbags-convert bags/replay/<date_run>_scan_only \
    --dst bags/replay/<date_run>_scan_only.bag

# Option B: Manual conversion via CSV or custom script
# See: https://github.com/ros2/rosbag2/tree/humble for format details
```

## ROS1 Validation Procedure

### 1. Copy Files to Noetic Machine

```bash
# Copy from Orin/Humble machine to Noetic workstation
scp maps/final/map.pgm noetic-ws:~/catkin_ws/maps/
scp maps/final/map.yaml noetic-ws:~/catkin_ws/maps/
scp bags/replay/<date_run>_scan_only.bag noetic-ws:~/catkin_ws/bags/
```

### 2. Load Map in ROS1 map_server

```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Launch map_server
rosrun map_server map_server ~/catkin_ws/maps/map.yaml \
    _frame_id:=map

# Verify map is published
rostopic echo /map_metadata --count=1
```

### 3. Launch AMCL

```bash
# Terminal 3: Launch AMCL with seeded initial pose
rosrun amcl amcl \
    _odom_frame_id:=odom \
    _base_frame_id:=flat_body \
    _global_frame_id:=map \
    _scan_topic:=/mapping/scan/nav \
    _min_particles:=500 \
    _max_particles:=5000 \
    _kld_err:=0.05 \
    _update_min_d:=0.1 \
    _update_min_a:=0.1 \
    _laser_max_range:=30.0 \
    _laser_model_type:=likelihood_field \
    _initial_pose_x:=0.0 \
    _initial_pose_y:=0.0 \
    _initial_pose_a:=0.0
```

### 4. Play LaserScan Bag

```bash
# Terminal 4: Static TF (flat_body serves as base_link)
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 flat_body base_link

# Terminal 5: Play the scan bag
rosbag play ~/catkin_ws/bags/<date_run>_scan_only.bag --clock
```

### 5. Verify AMCL Convergence

```bash
# Terminal 6: Monitor AMCL pose output
rostopic echo /amcl_pose

# Or use RViz
rosrun rviz rviz -d ~/catkin_ws/rviz/amcl_validation.rviz
```

## Frame Convention

| ROS2 Frame | ROS1 Equivalent | Notes |
|------------|-----------------|-------|
| `flat_body` | `base_link` | Static identity TF in ROS1 validation |
| `map` | `map` | Same |
| `odom` | `odom` | From bag TF or fake |

**Key:** `base_link := flat_body` for this handoff.

## Expected Topics

| Topic | Type | Source |
|-------|------|--------|
| `/map` | `nav_msgs/OccupancyGrid` | `map_server` |
| `/mapping/scan/nav` | `sensor_msgs/LaserScan` | Prerecorded bag |
| `/tf` | `tf2_msgs/TFMessage` | Prerecorded bag + static publisher |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL output |

## Acceptance Gate

**AMCL converges from a seeded pose within 30 seconds on the prerecorded scan bag.**

Convergence is defined as:
- Particle cloud contracts to < 1m spread
- Pose covariance diagonal values drop below 0.1
- Visual alignment in RViz shows scan overlay matching map features

## move_base Quick Test (Optional)

```bash
# If move_base is available, verify path planning works
roslaunch move_base move_base.launch \
    base_global_planner:=navfn/NavfnROS \
    base_local_planner:=base_local_planner/TrajectoryPlannerROS

# Send a navigation goal via RViz or:
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
    "header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0}, orientation: {w: 1.0}}"
```

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| Map doesn't load | Wrong path in yaml | Check `image:` field is relative or absolute |
| AMCL doesn't converge | Bad initial pose | Set initial_pose near actual start in bag |
| No scan data | Bag topic mismatch | Check: `rosbag info <bag>` for topic names |
| TF errors | Missing flat_body→base_link | Run static_transform_publisher |
| Map orientation wrong | Y-axis flip | Check `negate:` field in yaml |
