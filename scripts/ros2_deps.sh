#!/bin/bash

ROS_DISTRO=humble
# =============================================================================
# ROS2 Package Dependencies
# =============================================================================
apt-get update && apt-get install -y \
  ros-$ROS_DISTRO-urg-node \
  ros-$ROS_DISTRO-diagnostic-updater \
  ros-$ROS_DISTRO-nav-msgs \
  ros-$ROS_DISTRO-foxglove-bridge \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-velodyne \
  ros-$ROS_DISTRO-velodyne-driver \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher-gui