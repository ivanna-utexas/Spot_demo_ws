#!/bin/bash

read -p "Enter name for the new map: " map_name

MAP_DIR="$HOME/dance_ws_pedestrian_tracking/src/spot_nav/spot_nav2/spot_nav/maps"

MAP_PATH="$MAP_DIR/$map_name"

ros2 run nav2_map_server map_saver_cli \
  -f "$MAP_PATH" \
  --ros-args \
    -p save_map_timeout:=10000.0 \
    -p free_thresh:=0.25 \
    -p occupied_thresh:=0.65
