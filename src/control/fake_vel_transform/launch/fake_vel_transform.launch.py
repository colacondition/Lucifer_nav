from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    log_level = LaunchConfiguration('log_level')
    node_output = LaunchConfiguration('node_output')

    fake_vel_transform_node = Node(
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        output=node_output,
        parameters=[
            {'use_sim_time': use_sim_time }
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('node_output', default_value='log'),
        fake_vel_transform_node,
    ])
