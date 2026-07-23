"""Launch the CenterPoint LiDAR people-detector node.

Drop-in replacement for the ZED people-detection path: consumes
``/velodyne_points`` and publishes ego-frame ``bva_msgs/NearbyObstacles`` on
``/nearby_people``. Do NOT run this alongside another ``/nearby_people``
publisher (the ZED ``zed_posearray_to_nearby`` bridge or the HDL
``nearby_obstacles_bridge``) -- only one source may drive the topic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("people_detector"),
        "config",
        "centerpoint_people.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML parameters for centerpoint_people_node.",
            ),
            DeclareLaunchArgument(
                "points_topic",
                default_value="/velodyne_points",
                description="Input Velodyne PointCloud2 topic.",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/nearby_people",
                description="Output bva_msgs/NearbyObstacles topic (ego frame).",
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="base_link",
                description="Robot ego frame the detections are expressed in.",
            ),
            Node(
                package="people_detector",
                executable="centerpoint_people_node",
                name="centerpoint_people_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "points_topic": LaunchConfiguration("points_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "target_frame": LaunchConfiguration("target_frame"),
                    },
                ],
            ),
        ]
    )
