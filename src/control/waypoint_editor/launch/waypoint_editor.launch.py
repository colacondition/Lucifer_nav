from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('waypoint_editor')
    rviz_config = os.path.join(pkg, 'rviz', 'rviz_waypoint_editor.rviz')
    log_level = LaunchConfiguration('log_level')
    node_output = LaunchConfiguration('node_output')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output=node_output,
        arguments=['-d', rviz_config, '--ros-args', '--log-level', log_level]
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('log_level', default_value='warn'))
    ld.add_action(DeclareLaunchArgument('node_output', default_value='log'))
    ld.add_action(rviz2)
    return ld
