#!/usr/bin/env bash
# Live test: runs the social_nav_costmap_layer plugin as a standalone costmap
# subscribed to the REAL /social_nav/ped_actions topic from the VLM pipeline.
# No robot required — just the VLM tmux session and a camera.
#
# Usage (inside container):
#   # Terminal 1: start the VLM pipeline
#   cd ~/dance_ws_pedestrian_tracking/tmux/vlm && tmuxinator start
#
#   # Terminal 2: run this script
#   ./container shell
#   /home/ros/dance_ws_pedestrian_tracking/scripts/test_social_nav_layer_live.sh
#
# View in RViz: Fixed Frame = map, add Map display on /costmap/costmap

set -e

dance_ws_pedestrian_tracking=/home/ros/dance_ws_pedestrian_tracking

source /opt/ros/humble/setup.bash
source ${dance_ws_pedestrian_tracking}/install/setup.bash

PIDS=()
cleanup() {
  echo ""
  echo "[test] Shutting down..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  # Belt-and-suspenders: ensure no stale costmap process survives
  pkill -9 -f nav2_costmap_2d 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Kill any leftover costmap nodes from previous runs before we start
if pgrep -f nav2_costmap_2d > /dev/null; then
  echo "[test] Killing stale nav2_costmap_2d process..."
  pkill -9 -f nav2_costmap_2d 2>/dev/null || true
  sleep 2
fi

echo "[test] Verifying plugin is installed..."
PLUGIN_XML=${dance_ws_pedestrian_tracking}/install/social_nav_costmap_layer/share/social_nav_costmap_layer/social_nav_costmap_layer.xml
if [ ! -f "${PLUGIN_XML}" ]; then
  echo "[test] ERROR: plugin XML not found. Build with:"
  echo "       colcon build --packages-select social_nav_costmap_layer"
  exit 1
fi
echo "[test] OK — plugin descriptor installed"

echo "[test] Checking /social_nav/ped_actions has a publisher..."
TOPIC_INFO=$(timeout 3 ros2 topic info /social_nav/ped_actions 2>&1 || true)
if echo "${TOPIC_INFO}" | grep -qE "Publisher count: [1-9]"; then
  echo "[test] OK — VLM pipeline is publishing pedestrian actions"
else
  echo "[test] WARNING: nobody publishing /social_nav/ped_actions yet."
  echo "[test] Start the vlm tmux session in another terminal:"
  echo "       cd ~/dance_ws_pedestrian_tracking/tmux/vlm && tmuxinator start"
  echo "[test] Continuing — costmap will stay blank until data arrives."
fi

# Generate params file for standalone costmap (node name: costmap)
PARAMS_FILE=/tmp/social_nav_live_params.yaml
cat > ${PARAMS_FILE} << 'EOF'
costmap:
  costmap:
    ros__parameters:
      update_frequency: 2.0
      publish_frequency: 2.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.1
      width: 20
      height: 20
      origin_x: -10.0
      origin_y: -10.0
      rolling_window: false
      track_unknown_space: false
      footprint: "[[0.55, 0.25], [0.55, -0.25], [-0.55, -0.25], [-0.55, 0.25]]"

      plugins: ["social_nav_layer"]

      social_nav_layer:
        plugin: "social_nav_costmap_layer/SocialNavLayer"
        enabled: true
        ped_actions_topic: "/social_nav/ped_actions"
        message_timeout_s: 5.0
        avoid_inflation_radius_m: 1.5
        avoid_predict_horizon_s: 3.0
        avoid_predict_steps: 6
        overtake_inflation_radius_m: 1.0
        overtake_behind_cost: 85
        overtake_side_clear_m: 2.0
        overtake_side_incentive_cost: 15
        overtake_predict_horizon_s: 2.0
        overtake_predict_steps: 4
        yield_inflation_radius_m: 1.2
        yield_predict_horizon_s: 3.0
        yield_predict_steps: 6
        follow_distance_m: 1.5
        follow_trail_radius_m: 0.8
        follow_trail_incentive_cost: 10
        follow_predict_horizon_s: 2.0
        follow_predict_steps: 4

      always_send_full_costmap: true
EOF

# Provide fake TF chain if the real one isn't available (no Spot running)
if ! timeout 2 ros2 run tf2_ros tf2_echo map base_link --timeout 1 > /dev/null 2>&1; then
  echo "[test] No map->base_link TF detected — publishing fake static TFs"
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom >/dev/null 2>&1 &
  PIDS+=($!)
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link >/dev/null 2>&1 &
  PIDS+=($!)
  sleep 2
else
  echo "[test] Using existing map->base_link TF from running stack"
fi

echo "[test] Launching standalone costmap with social_nav_layer..."
ros2 run nav2_costmap_2d nav2_costmap_2d \
  --ros-args --params-file "${PARAMS_FILE}" \
  > /tmp/social_nav_live.log 2>&1 &
PIDS+=($!)

echo "[test] Waiting for costmap node..."
NODE=""
for i in $(seq 1 20); do
  NODE=$(ros2 node list 2>/dev/null | grep -E "^/costmap(/costmap)?$" | head -1)
  if [ -n "${NODE}" ]; then break; fi
  sleep 0.5
done

if [ -z "${NODE}" ]; then
  echo "[test] ERROR: costmap node never appeared. Log:"
  cat /tmp/social_nav_live.log
  exit 1
fi
echo "[test] OK — found node ${NODE}"

STATE=$(ros2 lifecycle get ${NODE} 2>/dev/null | awk '{print $1}')
echo "[test] Lifecycle state: ${STATE}"
if [ "${STATE}" != "active" ]; then
  ros2 lifecycle set ${NODE} configure >/dev/null 2>&1 || true
  ros2 lifecycle set ${NODE} activate >/dev/null 2>&1 || true
fi

sleep 1
if grep -q "SocialNavLayer initialized" /tmp/social_nav_live.log; then
  echo "[test] OK — SocialNavLayer initialized inside nav2 costmap"
else
  echo "[test] WARNING: no init log line. Recent output:"
  tail -20 /tmp/social_nav_live.log
fi

echo ""
echo "[test] Costmap running — subscribed to real /social_nav/ped_actions"
echo ""
echo "[test] Inspect with:"
echo "       ros2 topic hz /social_nav/ped_actions    # VLM publishing rate"
echo "       ros2 topic hz /costmap/costmap           # costmap publishing rate"
echo "       rviz2  (Fixed Frame: map, Map display on /costmap/costmap)"
echo ""
echo "[test] Log: /tmp/social_nav_live.log"
echo "[test] Press Ctrl+C to stop."

wait
