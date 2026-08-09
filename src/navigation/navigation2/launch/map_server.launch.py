import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('navigation2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    node_output = LaunchConfiguration('node_output')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'params', 'navigation2.yaml')),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('node_output', default_value='log'),

        Node(
            package='navigation2',
            executable='rm_map_server_node',
            name='rm_map_server',
            output=node_output,
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            }],
            arguments=['--ros-args', '--log-level', log_level]),
    ])
