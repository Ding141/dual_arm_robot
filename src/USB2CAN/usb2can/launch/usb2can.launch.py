from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('usb2can')
    default_cfg = PathJoinSubstitution([pkg, 'config', 'usb2can.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_cfg),
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0'),
        Node(
            package='usb2can',
            executable='usb2can_node',
            name='usb2can_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'device': LaunchConfiguration('device')},
            ],
            emulate_tty=True,
        ),
    ])
