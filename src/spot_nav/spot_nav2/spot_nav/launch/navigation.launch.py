import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_SCAN_TOPIC = '/mapping/scan/nav'
DEFAULT_EXTRINSICS_FILE = os.path.expanduser('~/nav_ws/config/mapping/extrinsics.yaml')


def resolve_map_yaml(map_value, package_share_dir):
    if not map_value:
        map_value = 'home'

    candidate = map_value if map_value.endswith('.yaml') else f'{map_value}.yaml'
    if os.path.isabs(candidate):
        return candidate

    return os.path.join(package_share_dir, 'maps', candidate)


def build_nav2_params(package_share_dir, scan_topic):
    nav2_config_file = os.path.join(package_share_dir, 'config', 'nav2.yaml')
    configured_scan_topic = scan_topic or DEFAULT_SCAN_TOPIC

    with open(nav2_config_file, 'r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream)

    params['amcl']['ros__parameters']['scan_topic'] = configured_scan_topic
    params['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['scan'][
        'topic'
    ] = configured_scan_topic
    params['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan'][
        'topic'
    ] = configured_scan_topic
    params['collision_monitor']['ros__parameters']['scan']['topic'] = configured_scan_topic

    with tempfile.NamedTemporaryFile(
        mode='w',
        suffix='.yaml',
        prefix='spot_nav2_',
        delete=False,
        encoding='utf-8',
    ) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        return stream.name


def load_base_to_lidar_args(extrinsics_file):
    resolved_file = os.path.expanduser(extrinsics_file or DEFAULT_EXTRINSICS_FILE)
    defaults = {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0,
        'roll': 0.0,
        'pitch': 0.0,
        'yaw': 0.0,
    }

    try:
        with open(resolved_file, 'r', encoding='utf-8') as stream:
            calibration = yaml.safe_load(stream) or {}
    except OSError as exc:
        print(
            f'[spot_nav] Warning: failed to load extrinsics file '
            f'{resolved_file}: {exc}'
        )
        calibration = {}

    transform = calibration.get('base_to_lidar') or calibration.get('body_to_lidar') or {}
    transform = {**defaults, **transform}

    return [
        str(transform['x']),
        str(transform['y']),
        str(transform['z']),
        str(transform['roll']),
        str(transform['pitch']),
        str(transform['yaw']),
        'base_link',
        'velodyne',
    ]


def launch_setup(context):
    pkg_spot_nav_dir = get_package_share_directory('spot_nav')
    pkg_spot_driver = get_package_share_directory('spot_driver')

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_value = LaunchConfiguration('map').perform(context)
    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    extrinsics_file = LaunchConfiguration('extrinsics_file').perform(context)
    map_yaml_file = resolve_map_yaml(map_value, pkg_spot_nav_dir)
    nav2_config_file = build_nav2_params(pkg_spot_nav_dir, scan_topic)
    
    # Localization (AMCL)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_config_file],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Map Server (Custom Python Node)
    # Map Server
    # Map Server (Custom Python Node)
    map_server_node = Node(
        package='spot_nav',
        executable='simple_map_server',
        name='simple_map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}]
    )

    # Lifecycle Manager for AMCL
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'bond_timeout': 0.0},
                    {'node_names': ['amcl']}]
    )

    # Navigation Stack (Planner, Controller, BT, etc.)
    # Uses our custom launch instead of nav2_bringup to ensure bond_timeout: 0.0
    # on lifecycle_manager_navigation (nav2_bringup ignores the YAML setting).
    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_spot_nav_dir, 'launch', 'nav2_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_config_file,
        }.items()
    )



    # Static Transform: flat_body -> base_link
    # User feedback: "don't use body as your base frame... use flat_body"
    # base_link is the robot root for Nav2. We attach it to flat_body.
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='flat_body_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'flat_body', 'base_link']
    )

    lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_velodyne',
        arguments=load_base_to_lidar_args(extrinsics_file)
    )

    scan_extractor_node = Node(
        package='spot_mapping_common',
        executable='scan_extractor_node',
        name='scan_extractor_node',
        output='screen',
        parameters=[{
            'input_topic': pointcloud_topic,
            'output_topic': scan_topic,
            'frame_id': 'flat_body',
            'target_frame': 'flat_body',
            'min_height': -0.1,
            'max_height': 0.5,
            'range_min': 0.2,
            'range_max': 30.0,
            'transform_timeout_sec': 0.1,
        }],
        condition=IfCondition(LaunchConfiguration('launch_scan_extractor'))
    )

    return [
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_spot_driver, 'launch', 'spot_driver.launch.py')
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('spot_config_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_driver'))
        ),
        static_tf_node,
        lidar_tf_node,
        scan_extractor_node,
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        nav2_launch_cmd,
    ]


def generate_launch_description():
    # Declare launch arguments
    declare_spot_config_file = DeclareLaunchArgument(
        'spot_config_file',
        default_value='',
        description='Path to Spot driver configuration file'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically start lifecycle nodes'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='home',
        description='Map basename in spot_nav/maps or an absolute .yaml path'
    )

    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value=DEFAULT_SCAN_TOPIC,
        description='LaserScan topic consumed by AMCL and Nav2 costmaps'
    )

    declare_pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/velodyne_points',
        description='PointCloud2 topic used to generate the navigation LaserScan'
    )

    declare_extrinsics_file = DeclareLaunchArgument(
        'extrinsics_file',
        default_value=DEFAULT_EXTRINSICS_FILE,
        description='Path to the mapping extrinsics YAML for base_link to velodyne TF'
    )

    declare_launch_scan_extractor = DeclareLaunchArgument(
        'launch_scan_extractor',
        default_value='True',
        description='Whether to launch the PointCloud2 to LaserScan projection node'
    )

    declare_launch_driver = DeclareLaunchArgument(
        'launch_driver',
        default_value='True',
        description='Whether to launch the Spot driver'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_spot_config_file)
    ld.add_action(declare_autostart)
    ld.add_action(declare_map)
    ld.add_action(declare_scan_topic)
    ld.add_action(declare_pointcloud_topic)
    ld.add_action(declare_extrinsics_file)
    ld.add_action(declare_launch_scan_extractor)
    ld.add_action(declare_launch_driver)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
