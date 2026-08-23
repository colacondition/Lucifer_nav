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

    waypoint_patrol_node = Node(
        package='waypoint_editor',
        executable='waypoint_executor',
        name='waypoint_patrol_executor',
        output=node_output,
        parameters=[{
            'mode': 'patrol',
            'service_name': 'start_waypoint_through',
            'action_name': '/waypoint_editor/through_waypoints',
            'executor_status_topic': '/waypoint_editor/through_status',
            'status_timeout': 20.0,
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
    ld.add_action(waypoint_patrol_node)
    return ld
