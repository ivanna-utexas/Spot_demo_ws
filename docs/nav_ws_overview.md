# Navigation Workspace Overview

The default pedestrian perception path is:

`/velodyne_points` → `people_detector/centerpoint_people_node` →
`/people_detections`, `/people/map_tracks`, `/nearby_people`, and
`/people_detections_markers`.

CUDA-CenterPoint receives ten motion-compensated sweeps with per-point time
lag. Full pedestrian boxes and predicted velocity are transformed into
`odom` before a covariance-aware constant-velocity Kalman tracker performs
Hungarian association. Stable, confirmed tracks feed all downstream
contracts directly; no format or map adapter is part of the default path.

The map output rotates state and covariance into `map`. The nearby output
rotates state and velocity into `base_link` and retains the nearest 16
people. A temporarily unavailable map transform produces an empty,
correctly stamped map array without stopping odom tracking.

Launch the pipeline once with:

```bash
ros2 launch people_detector pedestrian_tracking.launch.py
```

The navstack, human-feedback, dog-mode, standalone, VLM, and dedicated
pedestrian-tracking tmux sessions use this entry point. PTv3 and ZED bridges
are retained only for explicitly selected alternative perception setups.
