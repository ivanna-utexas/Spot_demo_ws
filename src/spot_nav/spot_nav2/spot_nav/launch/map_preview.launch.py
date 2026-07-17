import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_default_map():
    ws_map = os.path.expanduser('~/dance_ws_pedestrian_tracking/maps/final/current_nav.yaml')
    if os.path.exists(ws_map):
        return ws_map

    pkg_share = get_package_share_directory('spot_nav')
    return os.path.join(pkg_share, 'maps', 'home.yaml')


def generate_launch_description():
    pkg_share = get_package_share_directory('spot_nav')
    default_map = _resolve_default_map()
    default_rviz = os.path.join(pkg_share, 'rviz', 'map_preview.rviz')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Absolute path to the occupancy map YAML to preview',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='RViz config file for map preview',
    )

    map_server = Node(
        package='spot_nav',
        executable='simple_map_server',
        name='simple_map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map')}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_map_preview',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        declare_map,
        declare_rviz,
        map_server,
        rviz,
    ])
