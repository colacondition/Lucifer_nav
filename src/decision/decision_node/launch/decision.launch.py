from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")
    node_output = LaunchConfiguration("node_output")
    waypoint_root = [
        FindPackageShare("bringup"),
        "config",
        "waypoints",
        "RMUL",
    ]
    waypoint_files = {
        f"targets.{target}_waypoint_file": PathJoinSubstitution(
            [*waypoint_root, f"{target}.csv"]
        )
        for target in (
            "patrol",
            "center",
            "wait_center",
            "home",
            "wait_home",
            "wait_hp",
        )
    }


    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("decision"), "config", "bt_action_replacement.yaml"]
        ),
        description="Decision node parameter file",
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level", default_value="info", description="ROS log level"
    )
    declare_node_output = DeclareLaunchArgument(
        "node_output", default_value="screen", description="Node output target"
    )

    waypoint_follow_executor = Node(
        package="waypoint_editor",
        executable="waypoint_follow_executor",
        name="waypoint_follow_executor",
        output=node_output,
        arguments=["--ros-args", "--log-level", log_level],
    )
    decision_node = Node(
        package="decision",
        executable="bt_action_replacement_node",
        name="bt_action_replacement",
        output=node_output,
        parameters=[params_file, waypoint_files],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            declare_params_file,
            declare_log_level,
            declare_node_output,
            waypoint_follow_executor,
            decision_node,
        ]
    )
