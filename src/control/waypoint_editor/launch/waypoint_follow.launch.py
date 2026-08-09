from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_waypoint_file = DeclareLaunchArgument(
        'waypoint_file',
        default_value='',
        description='Full path to waypoint CSV file')

    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for waypoints')

    declare_goal_topic = DeclareLaunchArgument(
        'goal_topic',
        default_value='/goal_pose',
        description='Goal topic consumed by the custom navigation stack')

    declare_status_topic = DeclareLaunchArgument(
        'status_topic',
        default_value='/navigation2/status',
        description='Status topic published by the custom navigation stack')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='ROS log level')
    declare_node_output = DeclareLaunchArgument(
        'node_output',
        default_value='log',
        description='Node output target')

    waypoint_file = LaunchConfiguration('waypoint_file')
    frame_id = LaunchConfiguration('frame_id')
    goal_topic = LaunchConfiguration('goal_topic')
    status_topic = LaunchConfiguration('status_topic')
    log_level = LaunchConfiguration('log_level')
    node_output = LaunchConfiguration('node_output')

    waypoint_follow_node = Node(
        package='waypoint_editor',
        executable='waypoint_follow_executor',
        name='waypoint_follow_executor',
        output=node_output,
        parameters=[{
            'waypoint_file': waypoint_file,
            'frame_id': frame_id,
            'goal_topic': goal_topic,
            'status_topic': status_topic
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )

    ld = LaunchDescription()
    ld.add_action(declare_waypoint_file)
    ld.add_action(declare_frame_id)
    ld.add_action(declare_goal_topic)
    ld.add_action(declare_status_topic)
    ld.add_action(declare_log_level)
    ld.add_action(declare_node_output)
    ld.add_action(waypoint_follow_node)
    return ld
