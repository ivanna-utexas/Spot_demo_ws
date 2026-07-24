# Bag Collection Plan for `spot_dog_mode` (Auto Dog Mode)

## Setup

1. Bring up the demo stack: `cd tmux/auto_dog_mode && tmuxinator local`.
2. **Verify the LiDAR-to-`odom` TF exists at cloud timestamps**. CenterPoint
   uses it to motion-compensate sweeps and track people globally:

   ```bash
   ros2 topic hz /odom            # should tick at ~10 Hz (cloud rate)
   ros2 topic echo /odom --once | head -4
   ros2 topic echo /velodyne_points --once --field header | head -3
   ```

   If `/odom` is missing live, find/start whatever node provides it before
   recording — and note it in the session log, because that's also a live
   demo bug.
3. Confirm people tracking works live: `ros2 topic hz /people_detections`
   while someone walks in front.
4. Confirm the Kinect reference video is flowing:
   `ros2 topic hz /rgb/video/compressed` — should tick at ~30 Hz. This comes
   from the `image_transport republish` pane (the k4a driver's own
   `/rgb/image_raw/compressed` topic advertises but never publishes).
5. Disk: `df -h /` — budget ~600–800 MB/min of recording (velodyne
   dominates; compressed 720p30 video adds roughly 200–400 MB/min).

## Record 
Record **inside the container** into the workspace (bags elsewhere, e.g.
`~/nav_ws`, are invisible to the container — hard-won lesson):

```bash
cd /home/ros/dance_ws_pedestrian_tracking
ros2 bag record -o test_bags/dogmode_$(date +%m%d)_<scenario_name> \
  /velodyne_points /tf /tf_static \
  /odom /odometry \
  /status/feedback /joy \
  /people_detections /people/map_tracks /nearby_people \
  /people_detections_markers /centerpoint_people/diagnostics \
  /body_pose /dog_mode/enabled /rosout \
  /rgb/video/compressed /rgb/camera_info
```

Topic groups and why:

| Group | Topics | Purpose |
|---|---|---|
| Replay inputs (essential) | `/velodyne_points /tf /tf_static /odom` | Inputs needed for sweep motion compensation and frame transforms. |
| Robot state | `/status/feedback /joy /odometry` | Real standing/moving/deadman gating — replaying the bag's feedback (instead of faking "standing") tests the gates against reality. |
| Live perception outputs | `/people_detections /people/map_tracks /nearby_people /people_detections_markers /centerpoint_people/diagnostics` | Canonical contracts and runtime evidence to compare with offline replay. |
| Live behavior outputs | `/body_pose /dog_mode/enabled /rosout` | What the controller actually commanded on-robot + its state-transition logs — the golden reference to compare sim/replay against, and the record of how it *felt* vs what it *did*. |
| Reference video | `/rgb/video/compressed /rgb/camera_info` | Kinect RGB (JPEG, 720p30) — human-viewable ground truth of what people actually did in each scenario, for labeling and for settling "was that a real miss?" disputes during replay analysis. |

## Scenarios

Spot **standing, dog mode enabled** unless stated. These bags are the
reference data for testing dog mode without the robot — each scenario below
covers one behavior of the controller's state machine or one of its safety
gates

1. **`baseline_empty`** (2 min) — nobody within ~8 m. False-positive check:
   replay must produce zero greets. Also captures the lab's static clutter
   for false-positive and tracker-ghost checks.
2. **`single_approach`** (3 min) — one person approaches from ~8 m head-on,
   stops at ~2 m for 10 s, backs away. Repeat 3–4 times, including one
   approach from the side and one from behind. Core greet→track→relax data.
3. **`walk_by`** (2 min) — lateral passes at ~2 m and ~3.5 m, both
   directions, normal walking pace. Tracking-lag and relax timing.
4. **`two_person`** (3 min) — person A stands at ~3 m; person B walks in
   and stops clearly closer (~1.5 m), holds, leaves. Then both mill around.
   Target stickiness + switch margin with real, noisy tracks.
5. **`crowd`** (3 min) — 3+ people milling at 1–4 m, crossing paths.
   Id-stability stress test (real version of `noisy_ids`).
6. **`odd_shapes`** (2 min) — person crouching/sitting on the floor, person
   pushing a chair or carrying a big box, person standing very close
   (<1 m). Exercises the degenerate-cluster head-height fallback
   (`min_person_height`) and pitch clamping.
7. **`gates_live`** (3 min) — with a person standing at ~2.5 m: toggle dog
   mode off/on (Circle), hold/release L1 deadman, sit and stand Spot,
   briefly teleop-walk a meter and stop. The real-data twin of
   `disable_midtrack` / `deadman_suspend` / `robot_sits`, with real
   `/status/feedback` timing.
8. **`walking_suspend`** (2 min) — teleop Spot slowly past people. Verifies
   gaze stays suspended while `moving` and resumes after stopping.

## While recording

- Note the true distances/timings roughly (photo of a tape measure or floor
  marks helps) — it's the only ground truth these bags will ever have, and
  it's what makes replayed gaze angles checkable later.
- Watch the robot: if any behavior looks wrong (bows instead of looking up,
  jitter, late relax), say/log the wall-clock time — `/rosout` in the bag
  will line up with it.
- Keep `attention_radius` etc. at stock `config/params.yaml` values for at
  least one full pass so bags match the committed config.
