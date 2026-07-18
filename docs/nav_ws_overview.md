## `nav_ws` Overview

`/home/ros/nav_ws` is a ROS 2 Humble workspace (Docker on a Jetson Orin, CycloneDDS) for a Boston Dynamics Spot carrying a Velodyne VLP-16. It covers: the Spot driver (`spot_ros2`), lidar-inertial SLAM for map building (LiORF/LIO-SAM/SuperOdom + `spot_liorf_bringup`), a Nav2 stack (`spot_nav` with AMCL, RPP controller, custom map server), and a social-navigation research layer (people tracking, a VLM-based policy in `vlm_policy`, a people-aware costmap plugin, `PrefNav`). Everything is launched via tmuxinator sessions in `tmux/` and run through `./container`.

## The human tracking pipeline

It's a three-stage lidar pipeline: **detect/track (C++) → normalize format (Python adapter) → transform to map frame (Python adapter)**, consumed by the VLM social-nav policy.

**Stage 1 — [hdl_people_tracking](/src/hdl_people_tracking/)** (ported from Koide's ROS 1 package, now ROS 2 nodes instead of nodelets):

- _Detection node_ ([hdl_people_detection_nodelet.cpp](/src/hdl_people_tracking/hdl_people_tracking/apps/hdl_people_tracking_nodelet.cpp)): subscribes to Velodyne points plus a `globalmap` point cloud (transient-local; there are `dummy_map_publisher*.py` scripts to feed it), does **background subtraction** against that map so only moving/non-map points remain, clusters the residue (Haselich clustering), and runs **Kidono's boosted person classifier** (`data/boost_kidono.model`) to mark clusters `is_human`. It publishes `ClusterArray` in the `tracking_frame` (default `odom`; a `_map.launch.py` variant exists).
- _Tracking node_ ([people_tracker.hpp](/src/hdl_people_tracking/hdl_people_tracking/include/hdl_people_tracking/people_tracker.hpp)): associates human clusters across frames (global nearest neighbor / Munkres) and runs a **constant-velocity Kalman filter** per person. Output: `TrackArray` on `/tracks` — each `Track` has `id`, `age`, `pos`, `vel`, 3×3 position/velocity covariances, and the associated cluster (which carries bounding-box `size`).

**Stage 2 — [people_format_adapter_node.py](/src/people_detector/scripts/people_format_adapter_node.py)** (`people_detector` package): unifies **three detection sources** into one `PeopleArray` on `/people_detections`:

1. `/tracks` (hdl tracks — has velocity, confidence 1.0),
2. `/clusters` (hdl clusters, optionally including non-human ones at low confidence),
3. a **PTv3 (Point Transformer v3) semantic-segmentation path**: per-point labels on `/pointcept/labels` matched back to buffered `/velodyne_points` clouds by point count, filtered to the "person" label, z-gated, and clustered with a grid-hash Euclidean clustering — this path yields detections but not stable IDs or velocities.

It also publishes RViz `MarkerArray`s color-coded by source (green = hdl tracks, orange = clusters, blue = PTv3).

**Stage 3 — [map_tracks_adapter_node.py](/src/people_detector/scripts/map_tracks_adapter_node.py)** (new, uncommitted): subscribes to `/tracks`, looks up TF from the track frame (odom) into **`map`**, and publishes `MapPersonArray` on `/people/map_tracks`. Each `MapPerson` gets position and velocity rotated into the map frame, covariances properly rotated (R·Σ·Rᵀ), plus timing metadata (`sample_time_sec`, `track_age_sec`, per-track `dt_sec`). This gives downstream planners globally-consistent person states even as the robot moves.

**Consumers**: `vlm_policy` (`social_nav_node.py` and the older `vlm_inference_node.py`) uses the people topics to reason about pedestrian behavior, and `social_nav_costmap_layer` is a Nav2 costmap plugin that inflates cost around people based on VLM-predicted actions — so tracked humans directly shape the robot's paths.

**Running it**: the [hdl_people_tracking tmux session](/tmux/hdl_people_tracking/.tmuxinator.yaml) launches the Azure Kinect driver, `hdl_people_tracking.launch.py`, the Velodyne driver, and RViz over VNC. The adapters have their own launch files in `src/people_detector/launch/`.

One practical caveat: the detector's background subtraction requires the `globalmap` frame to match `tracking_frame` (it refuses the map otherwise, [hdl_people_detection_nodelet.cpp:208](/src/hdl_people_tracking/hdl_people_tracking/apps/hdl_people_detection_nodelet.cpp:208)) — that mismatch is a common reason for zero detections. Also note the PTv3 source assigns a fresh ID to every cluster each frame (`_ptv3_id_counter`), so only the hdl track IDs are stable over time.