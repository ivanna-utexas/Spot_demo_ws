# composable_prefnav

ROS 2 wrapper for the PrefNav diffusion waypoint generator.

## Inputs

- `odom_topic` (`nav_msgs/msg/Odometry`)
- `tracks_topic` (`people_detector/msg/PeopleArray`)
- `ped_actions_topic` (`vlm_policy_msgs/msg/PedActionArray`, optional)

## Outputs

- `waypoints_topic` (`nav_msgs/msg/Path`)
- `legacy_diffusion_path_topic` (`std_msgs/msg/Float32MultiArray`)
- `markers_topic` (`visualization_msgs/msg/MarkerArray`)

## Notes

- The node uses tracked people directly and does **not** depend on `jackal_perception`.
- VLM actions are mapped to diffusion models as follows:
  - `FOLLOW` -> `follow`
  - `YIELD_TO` -> `yield`
  - `OVERTAKE` -> `overtake_right` by default
  - everything else -> `pretrain_dynamic`
- Static avoid regions and preferred regions still come from the PrefNav scenario YAML.
- Checkpoint paths must be provided in the ROS 2 params file.
