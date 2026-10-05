"""
双板启动: 同时拉起两套 (usb2can_node + dm_motor_bridge), 各自独立 namespace.

  /boardA/imu/data           /boardB/imu/data
  /boardA/dcu/feedback       /boardB/dcu/feedback
  /boardA/motor/ch1/cmd      /boardB/motor/ch1/cmd   ...

要求先做 udev 规则把两块板子绑成 /dev/usb2can_a /dev/usb2can_b
(详见 README §双板部署).

用法:
  ros2 launch usb2can usb2can_dual.launch.py
  ros2 launch usb2can usb2can_dual.launch.py device_a:=/dev/ttyACM0 device_b:=/dev/ttyACM1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def _board(ns_arg: str, dev_arg: str, cfg, motors_cfg):
    return GroupAction([
        PushRosNamespace(LaunchConfiguration(ns_arg)),
        Node(
            package='usb2can', executable='usb2can_node', name='usb2can_node',
            output='screen', emulate_tty=True,
            parameters=[cfg, {'device': LaunchConfiguration(dev_arg)}],
        ),
        Node(
            package='usb2can', executable='dm_motor_bridge', name='dm_motor_bridge',
            output='screen', emulate_tty=True,
            parameters=[motors_cfg],
        ),
    ])


def generate_launch_description():
    pkg = FindPackageShare('usb2can')
    cfg        = PathJoinSubstitution([pkg, 'config', 'usb2can.yaml'])
    motors_cfg = PathJoinSubstitution([pkg, 'config', 'dm_motors.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('ns_a',     default_value='boardA'),
        DeclareLaunchArgument('ns_b',     default_value='boardB'),
        DeclareLaunchArgument('device_a', default_value='/dev/usb2can_a'),
        DeclareLaunchArgument('device_b', default_value='/dev/usb2can_b'),

        _board('ns_a', 'device_a', cfg, motors_cfg),
        _board('ns_b', 'device_b', cfg, motors_cfg),
    ])
