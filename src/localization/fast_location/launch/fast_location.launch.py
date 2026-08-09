import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('fast_location')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'fast_location.rviz')
    default_config = os.path.join(pkg_dir, 'config', 'fast_location.yaml')

    config_arg = DeclareLaunchArgument(
        'fast_location_config',
        default_value=default_config,
        description='Path to fast_location YAML config'
    )
    output_arg = DeclareLaunchArgument(
        'fast_location_output',
        default_value='log',
        description='Output for robot_localization_node (screen/log)'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start fast_location RViz2'
    )
    fast_location_config = LaunchConfiguration('fast_location_config')
    fast_location_output = LaunchConfiguration('fast_location_output')
    use_rviz = LaunchConfiguration('use_rviz')

    # 定位节点
    localization_node = Node(
        package='fast_location',
        executable='robot_localization_node',
        name='robot_localization_node',
        output=fast_location_output,
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[fast_location_config]
    )

    # RViz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'WARN'],
        output='log',  # 不在终端打印,只写入日志
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        config_arg,
        output_arg,
        use_rviz_arg,
        localization_node,
        rviz_node,
    ])
