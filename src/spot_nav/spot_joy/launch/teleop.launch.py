from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    joy_respawn_delay = float(LaunchConfiguration('joy_respawn_delay').perform(context))

    return [
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': 0.1,
                'autorepeat_rate': 50.0,
            }],
            respawn=True,
            respawn_delay=joy_respawn_delay,
        ),
        Node(
            package='spot_joy',
            executable='teleop_node',
            name='spot_joy_teleop',
            output='screen',
            parameters=[{'verbose': LaunchConfiguration('verbose')}],
        ),
    ]


def generate_launch_description():
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose logging',
    )
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/ps4_primary',
        description='Stable joystick device path used by joy_linux_node',
    )
    joy_device_name_arg = DeclareLaunchArgument(
        'joy_device_name',
        default_value='Wireless Controller',
        description='Deprecated compatibility arg; ignored now that teleop uses joy_linux_node',
    )
    joy_respawn_delay_arg = DeclareLaunchArgument(
        'joy_respawn_delay',
        default_value='2.0',
        description='Seconds to wait before respawning joy_linux_node after a disconnect',
    )

    # Bind teleop to a udev-managed symlink so device numbering changes
    # caused by other controllers do not steal the joystick input.
    return LaunchDescription([
        verbose_arg,
        joy_dev_arg,
        joy_device_name_arg,
        joy_respawn_delay_arg,
        OpaqueFunction(function=launch_setup),
    ])
