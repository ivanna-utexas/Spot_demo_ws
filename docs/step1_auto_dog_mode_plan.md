# Step 1: Auto Dog Mode — Perception & Gaze Behavior (Implementation Plan)

## Context

The research project ([project_outline.md](project_outline.md)) needs a "demo mode" for Spot that makes it socially approachable. Step 1 is **Auto Dog Mode**: Spot perceives nearby people and responds with a friendly, dog-like "look-up" gesture (body yaw/pitch, since Spot has no articulated head).

Aligned decisions:
- **Perception**: LiDAR-first hybrid — reuse the existing Velodyne pipeline (`hdl_people_tracking` → `people_format_adapter` → `/people_detections`); camera refinement is an optional later step.
- **Behavior**: Greet + track — a pronounced look-up "acknowledge" gesture when a person newly enters range, then smooth continuous gaze tracking, relaxing to neutral when they leave.
- **Motion scope**: Standing only at first; suspend gaze while walking/teleop.
- **Activation**: Joystick toggle in `spot_joy` + a dedicated tmux demo session.

## What already exists (reuse, don't rebuild)

| Functionality | Package | Overview |
|---|---|---|
| LiDAR person detection + tracking | `src/hdl_people_tracking/` (launch: `hdl_people_tracking.launch.py`) | Kalman-tracked people from `/velodyne_points`, output in `odom` frame |
| Pedestrian detections | `src/people_detector/scripts/people_format_adapter_node.py` | `/people_detections` (`people_detector/PeopleArray`): per-person 3D `position`, `velocity`, `size` (→ height), RViz markers |
| Body pose command | `spot_ros2` driver: `body_pose` topic (`geometry_msgs/Pose`), handled in `spot_driver/spot_ros2.py:body_pose_callback` | Sets body roll/pitch/yaw + height offset rel. to footprint via mobility params. Mock mode logs the pose when no robot is connected |
| Pose limits & quaternion math | `src/spot_nav/spot_joy/spot_joy/teleop_node.py` (MAX_ROLL/PITCH/YAW = 0.4 rad, MAX_BODY_HEIGHT = 0.3 m) | Safe command ranges + existing RPY→quaternion pattern |
| Standing/sitting/moving state | `/status/feedback` (`spot_msgs/Feedback`: `standing`, `sitting`, `moving`) — already consumed by teleop | Standing-only gate |
| Target selection w/ hysteresis | `spot_eye_animation/face_gaze_node.py:_select_face` | Pattern to copy for sticky target selection |
| Eye display | `spot_eye_animation` `eye_display_node` listening on `eye_animation/gaze` (`Gaze.msg`: normalized x/y + detected) | Optional: make the screen eyes look at the same person |
| TF | spot driver publishes `odom→body` (`tf_root: odom`); detections are in `odom` | Transform person position into body frame with tf2 |
| Ops patterns | `tmux/navstack_minimal/.tmuxinator.yaml` (has hdl + eye animation lines already, some commented) | Template for the demo tmux session |

## Implementation

### 1. New Package `src/spot_dog_mode` (ament_python)

Files: `package.xml`, `setup.py`, `resource/`, `config/params.yaml`, `launch/dog_mode.launch.py`, `spot_dog_mode/gaze_controller_node.py`.

**`gaze_controller_node.py`** — the core node:

- **Inputs**
  - `/people_detections` (`PeopleArray`) — person positions/sizes in `odom`.
  - `/status/feedback` (`spot_msgs/Feedback`) — gate: only act when `standing and not moving`.
  - `/joy` (`sensor_msgs/Joy`) — suspend while manual deadman (L1) is held so it never fights teleop's own `body_pose` publishing.
  - tf2 buffer for `odom→body`.
- **Enable/disable**: `std_srvs/SetBool` service `/dog_mode/enable` + latched `std_msgs/Bool` status topic `/dog_mode/enabled`.
- **Target selection**: nearest person within `attention_radius` (default 4 m), sticky with hysteresis (adapt `_select_face` logic: keep current target unless it disappears for `target_lost_timeout` (~1.5 s) or another person is significantly closer).
- **State machine** (timer at `command_rate`, default 10 Hz):
  - `IDLE` → no people in range; publish nothing after settling at neutral.
  - `GREET` → on new target: ramp to yaw-toward-person + extra pitch-up (`greet_pitch_boost`, e.g. +0.15 rad beyond geometric gaze) over ~0.5 s, hold `greet_hold` (~1.2 s). This is the legible "dog noticing you" moment.
  - `TRACK` → smooth continuous gaze: low-pass filter + rate limit (`max_rate_rad_s`) on commanded yaw/pitch.
  - `RELAX` → target lost: ease back to neutral pose over ~1 s, then go `IDLE`.
- **Gaze math** (person position transformed into `body` frame):
  - `yaw = atan2(p.y, p.x)`, clamped to ±`max_yaw` (0.4).
  - `pitch = -atan2(head_z, hypot(p.x, p.y))`, clamped to ±`max_pitch` (0.4); `head_z = p.position.z + size.z/2 - body_height_above_ground` with fallback person height param (1.7 m) when cluster size is degenerate. Verify pitch sign on robot (teleop convention: positive pitch = nose down).
  - Publish `geometry_msgs/Pose` on `body_pose` (same RPY→quaternion math as `teleop_node.py:232-242`).
- **Safety**: publish neutral pose once and stop commanding whenever disabled, not standing, deadman held, or detections go stale (`detection_timeout`).
- **Optional (param `publish_eye_gaze`)**: also publish normalized `Gaze` on `eye_animation/gaze` from the same target so the eyes track the person too.

**`config/params.yaml`**: all tunables above (radii, timeouts, filter constants, limits) so on-robot tuning (outline bullet 4) is config-only.

**`launch/dog_mode.launch.py`**: gaze controller + `people_format_adapter` (from `people_detector`), with an `include_hdl:=true` arg to optionally include `hdl_people_tracking.launch.py`.

### 2. Joystick Toggle — `src/spot_nav/spot_joy/spot_joy/teleop_node.py`

- Add a rising-edge toggle on an unused button (**BTN_CIRCLE** — Options is taken by Claim/Power-On, Share is reserved for gesture recording in the dance workspace) that calls `/dog_mode/enable` (SetBool, async) and logs state.
- No other teleop changes: the gaze node's own `/joy` deadman-suspend handles arbitration, and teleop's existing neutral-pose-on-release stays the final authority.

### 3. TMUX Session — `tmux/auto_dog_mode/.tmuxinator.yaml`

Clone of `navstack_minimal` minus the nav stack: spot_driver, velodyne, `hdl_people_tracking`, `dog_mode.launch.py`, spot_joy teleop, vnc + rviz (reuse `rviz2/navstack.rviz`, which already shows people markers), eye_animation pane commented in as optional.

### 4. Azure Kinect (next step)

Camera refinement: fuse `face_gaze_node` detections (Azure Kinect) in the forward sector to confirm person + refine head height; or port detection to Spot body cameras. Noted as follow-up in the package README.

## Verification

All in the Docker container (`./container shell`), per repo convention:

1. **Build**: `colcon build --packages-select spot_dog_mode spot_joy && source install/setup.bash`.
2. **Offline logic test (no robot)**: run `gaze_controller_node` + publish synthetic `PeopleArray` on `/people_detections` and a fake `Feedback` (standing=true); static TF `odom→body`; `ros2 topic echo /body_pose` to check greet→track→relax sequencing, clamping, smoothing. If a recorded Velodyne bag is available, replay through hdl_people_tracking instead.
3. **Mock driver test**: launch `spot_driver` without robot credentials — its mock mode logs "Mock mode, received command pose", confirming end-to-end wiring including the joystick toggle.
4. **On-robot**: `tmuxinator` the `auto_dog_mode` session; stand Spot, toggle dog mode via OPTIONS, walk into range: expect yaw-toward + look-up greet, smooth tracking, relax on leaving; verify L1 deadman instantly overrides; then tune `params.yaml` (radius, smoothing, pitch sign/magnitude).
