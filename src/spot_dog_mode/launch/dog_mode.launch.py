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

    include_hdl_arg = DeclareLaunchArgument(
        'include_hdl',
        default_value='false',
        description='Also launch hdl_people_tracking (otherwise expect it to run separately)',
    )

    hdl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hdl_people_tracking'),
                'launch',
                'hdl_people_tracking.launch.py',
            )
        ),
        condition=IfCondition(LaunchConfiguration('include_hdl')),
    )

    people_adapter = Node(
        package='people_detector',
        executable='people_format_adapter_node.py',
        name='people_format_adapter',
        output='screen',
        parameters=[{'enable_ptv3': False}],
    )

    gaze_controller = Node(
        package='spot_dog_mode',
        executable='gaze_controller_node',
        name='dog_mode_gaze_controller',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        include_hdl_arg,
        hdl_launch,
        people_adapter,
        gaze_controller,
    ])
