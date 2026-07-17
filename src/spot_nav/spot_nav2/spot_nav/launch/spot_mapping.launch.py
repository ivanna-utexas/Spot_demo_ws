import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    velodyne_launch_dir = os.path.join(
        get_package_share_directory("spot_velodyne"), "launch"
    )
    super_odom_launch_dir = os.path.join(
        get_package_share_directory("super_odometry"), "launch"
    )
    super_odom_share_dir = get_package_share_directory("super_odometry")

    declare_device_ip = DeclareLaunchArgument(
        "device_ip",
        default_value="192.168.50.201",
        description="IP address of the Velodyne LIDAR",
    )
    declare_velodyne_rpm = DeclareLaunchArgument(
        "velodyne_rpm",
        default_value="600.0",
        description="Spin rate configured on the Velodyne sensor in RPM.",
    )
    declare_use_imu = DeclareLaunchArgument(
        "use_imu",
        default_value="false",
        description="Launch SuperOdom IMU preintegration for the Spot mapping flow.",
    )
    declare_imu_topic = DeclareLaunchArgument(
        "imu_topic",
        default_value="/imu_sensor_broadcaster/imu",
        description="Spot IMU topic to use if SuperOdom IMU preintegration is enabled.",
    )
    declare_odom_topic = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odometry",
        description="Spot odometry topic used as SuperOdom motion prior for lidar de-skewing.",
    )
    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(super_odom_share_dir, "config", "vlp_16.yaml"),
        description="Path to the SuperOdom VLP-16 configuration file.",
    )
    declare_calibration_file = DeclareLaunchArgument(
        "calibration_file",
        default_value=os.path.join(super_odom_share_dir, "config", "velodyne", "vlp_16_calibration.yaml"),
        description="Path to the SuperOdom calibration file.",
    )
    declare_map_pcd = DeclareLaunchArgument(
        "map_pcd",
        default_value="/home/ros/dance_ws_pedestrian_tracking/maps/superodom/pointcloud_local.pcd",
        description="Path to the localization pointcloud PCD file.",
    )
    declare_world_frame = DeclareLaunchArgument(
        "world_frame",
        default_value="superodom_map",
        description="Frame used by SuperOdom odometry and TF output.",
    )
    declare_sensor_frame = DeclareLaunchArgument(
        "sensor_frame",
        default_value="velodyne",
        description="Sensor frame used by SuperOdom outputs.",
    )
    declare_use_odom_sensor_extrinsic = DeclareLaunchArgument(
        "use_odom_sensor_extrinsic",
        default_value="true",
        description="Compose Spot body odometry with the configured body-to-velodyne transform before de-skewing.",
    )
    declare_odom_to_sensor_x = DeclareLaunchArgument(
        "odom_to_sensor_x",
        default_value="0.0",
        description="Spot body-to-velodyne translation X.",
    )
    declare_odom_to_sensor_y = DeclareLaunchArgument(
        "odom_to_sensor_y",
        default_value="0.0",
        description="Spot body-to-velodyne translation Y.",
    )
    declare_odom_to_sensor_z = DeclareLaunchArgument(
        "odom_to_sensor_z",
        default_value="0.5",
        description="Spot body-to-velodyne translation Z.",
    )
    declare_odom_to_sensor_roll = DeclareLaunchArgument(
        "odom_to_sensor_roll",
        default_value="0.0",
        description="Spot body-to-velodyne roll in radians.",
    )
    declare_odom_to_sensor_pitch = DeclareLaunchArgument(
        "odom_to_sensor_pitch",
        default_value="0.0",
        description="Spot body-to-velodyne pitch in radians.",
    )
    declare_odom_to_sensor_yaw = DeclareLaunchArgument(
        "odom_to_sensor_yaw",
        default_value="0.0",
        description="Spot body-to-velodyne yaw in radians.",
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(velodyne_launch_dir, "velodyne.launch.py")),
        launch_arguments={
            "device_ip": LaunchConfiguration("device_ip"),
            "rpm": LaunchConfiguration("velodyne_rpm"),
        }.items(),
    )

    super_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(super_odom_launch_dir, "vlp_16.launch.py")),
        launch_arguments={
            "use_imu": LaunchConfiguration("use_imu"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "config_file": LaunchConfiguration("config_file"),
            "calibration_file": LaunchConfiguration("calibration_file"),
            "map_pcd": LaunchConfiguration("map_pcd"),
            "world_frame": LaunchConfiguration("world_frame"),
            "sensor_frame": LaunchConfiguration("sensor_frame"),
            "use_odom_sensor_extrinsic": LaunchConfiguration("use_odom_sensor_extrinsic"),
            "odom_to_sensor_x": LaunchConfiguration("odom_to_sensor_x"),
            "odom_to_sensor_y": LaunchConfiguration("odom_to_sensor_y"),
            "odom_to_sensor_z": LaunchConfiguration("odom_to_sensor_z"),
            "odom_to_sensor_roll": LaunchConfiguration("odom_to_sensor_roll"),
            "odom_to_sensor_pitch": LaunchConfiguration("odom_to_sensor_pitch"),
            "odom_to_sensor_yaw": LaunchConfiguration("odom_to_sensor_yaw"),
        }.items(),
    )

    return LaunchDescription([
        declare_device_ip,
        declare_velodyne_rpm,
        declare_use_imu,
        declare_imu_topic,
        declare_odom_topic,
        declare_config_file,
        declare_calibration_file,
        declare_map_pcd,
        declare_world_frame,
        declare_sensor_frame,
        declare_use_odom_sensor_extrinsic,
        declare_odom_to_sensor_x,
        declare_odom_to_sensor_y,
        declare_odom_to_sensor_z,
        declare_odom_to_sensor_roll,
        declare_odom_to_sensor_pitch,
        declare_odom_to_sensor_yaw,
        velodyne_launch,
        super_odom_launch,
    ])
