
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    # Getting directories and launch-files
    bringup_dir = get_package_share_directory('linefit_ground_segmentation_ros')
    params_file = os.path.join(bringup_dir, 'launch', 'segmentation_params.yaml')
    log_level = LaunchConfiguration('log_level')
    node_output = LaunchConfiguration('node_output')

    # Nodes launching commands
    node_start_cmd = Node(
            package='linefit_ground_segmentation_ros',
            executable='ground_segmentation_node',
            output=node_output,
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', log_level])


    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(DeclareLaunchArgument('log_level', default_value='warn'))
    ld.add_action(DeclareLaunchArgument('node_output', default_value='log'))
    ld.add_action(node_start_cmd)


    return ld
