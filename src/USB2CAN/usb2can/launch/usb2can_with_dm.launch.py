from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('usb2can')
    default_cfg        = PathJoinSubstitution([pkg, 'config', 'usb2can.yaml'])
    default_motors_cfg = PathJoinSubstitution([pkg, 'config', 'dm_motors.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_cfg),
        DeclareLaunchArgument('motors_config', default_value=default_motors_cfg,
                              description='Per-motor 限幅 YAML (dm_motor_bridge)'),
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0'),

        # 串口桥接节点（裸字节 <-> /dcu/{command,feedback}, /imu/data）
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

        # DM 电机 MIT 桥（/dcu/{command,feedback} <-> /motor/ch{1,2,3}/{cmd,state}）
        # 全部参数 (全局默认 + per-motor 限幅) 来自 dm_motors.yaml.
        Node(
            package='usb2can',
            executable='dm_motor_bridge',
            name='dm_motor_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('motors_config')],
        ),
    ])
