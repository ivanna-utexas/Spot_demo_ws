# Auto Dog Mode — Perception and Gaze

Auto Dog Mode consumes stable `PeopleArray` tracks from the canonical
CUDA-CenterPoint pipeline on `/people_detections`. The tracker operates in
`odom`; the gaze controller transforms the selected person into the robot
body frame.

The behavior is unchanged:

- select the nearest person with target-switch hysteresis;
- greet a new target, then smoothly gaze-track it;
- relax when the target leaves;
- suspend while disabled, sitting, moving, stale, or while teleop holds the
  manual deadman.

`spot_dog_mode dog_mode.launch.py` exposes `include_tracking`. Set it to
`true` only when another session is not already running
`people_detector pedestrian_tracking.launch.py`.

The `tmux/auto_dog_mode` session launches Velodyne, CenterPoint, dog mode,
teleop, and visualization exactly once. The standalone session also provides
an identity `odom -> velodyne` transform for bench testing.

Verification and tuning should cover standing, sitting, walking, carrying an
object, two-person crossings, robot motion, brief occlusion, stable IDs,
plausible velocity, and timely clearing of departed tracks.
