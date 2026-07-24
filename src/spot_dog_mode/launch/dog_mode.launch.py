import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('spot_dog_mode')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    include_tracking_arg = DeclareLaunchArgument(
        'include_tracking',
        default_value='false',
        description='Also launch the canonical CUDA-CenterPoint tracker',
    )

    tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('people_detector'),
                'launch',
                'pedestrian_tracking.launch.py',
            )
        ),
        condition=IfCondition(LaunchConfiguration('include_tracking')),
    )

    gaze_controller = Node(
        package='spot_dog_mode',
        executable='gaze_controller_node',
        name='dog_mode_gaze_controller',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        include_tracking_arg,
        tracking_launch,
        gaze_controller,
    ])
