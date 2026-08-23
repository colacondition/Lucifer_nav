import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('serial_driver'), 'config', 'serial_driver.yaml')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    node_output = LaunchConfiguration('node_output')

    serial_driver_node = Node(
        respawn=True, respawn_delay=2.0,  # 串口崩溃自愈（重连后继续收发）
        package='serial_driver',
        executable='serial_driver_node',
        namespace='',
        output=node_output,
        emulate_tty=True,
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('node_output', default_value='log'),
        serial_driver_node,
    ])
