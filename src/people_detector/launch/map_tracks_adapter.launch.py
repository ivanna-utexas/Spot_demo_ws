from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("tracks_topic", default_value="/tracks"),
            DeclareLaunchArgument("output_topic", default_value="/people/map_tracks"),
            DeclareLaunchArgument("target_frame", default_value="map"),
            DeclareLaunchArgument("source", default_value="hdl_tracks"),
            DeclareLaunchArgument("confidence", default_value="1.0"),
            DeclareLaunchArgument("transform_timeout_sec", default_value="0.05"),
            Node(
                package="people_detector",
                executable="map_tracks_adapter_node.py",
                name="map_tracks_adapter",
                output="screen",
                parameters=[
                    {
                        "tracks_topic": LaunchConfiguration("tracks_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "target_frame": LaunchConfiguration("target_frame"),
                        "source": LaunchConfiguration("source"),
                        "confidence": LaunchConfiguration("confidence"),
                        "transform_timeout_sec": LaunchConfiguration("transform_timeout_sec"),
                    }
                ],
            ),
        ]
    )
