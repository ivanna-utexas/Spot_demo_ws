from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("composable_prefnav"))
    prefnav_share = Path(get_package_share_directory("prefnav"))

    default_params = package_share / "config" / "diffusion_waypoint_params.yaml"
    default_scenario = prefnav_share / "robot_deployment" / "task_scenarios" / "robot_exp" / "ahg_test_open.yaml"

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_params),
        description="Path to the ROS 2 parameter YAML file.",
    )
    scenario_arg = DeclareLaunchArgument(
        "scenario_file",
        default_value=str(default_scenario),
        description="Installed PrefNav scenario YAML.",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS log level.",
    )

    node = Node(
        package="composable_prefnav",
        executable="diffusion_waypoint_node",
        name="prefnav_diffusion_waypoint_node",
        output="screen",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[
            LaunchConfiguration("params_file"),
            {"scenario_file": LaunchConfiguration("scenario_file")},
        ],
    )

    return LaunchDescription([params_arg, scenario_arg, log_level_arg, node])
