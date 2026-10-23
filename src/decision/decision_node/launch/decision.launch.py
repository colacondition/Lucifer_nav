from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")
    node_output = LaunchConfiguration("node_output")
    start_waypoint_executor = LaunchConfiguration("start_waypoint_executor")
    # 航点单一真源：sentry_waypoints 数据包（和 bringup/launch/real.launch.py 一致）。
    # 以前这里指的是 bringup/config/waypoints/RMUL —— 那个目录**已经不存在了**
    # （航点搬到了 sentry_waypoints），于是单独起决策节点时所有航点都读不到，
    # 表现是 "读不到中心区域角点 / CENTER 段会原地不动"。
    waypoint_root = [
        FindPackageShare("sentry_waypoints"),
        "waypoints",
        "RMUL",
    ]
    waypoint_files = {
        f"targets.{target}_waypoint_file": PathJoinSubstitution(
            [*waypoint_root, f"{target}.csv"]
        )
        # zone = 中心占领区四个角点（range.csv）。追击和走位都靠它。
        for target in (
            "center",
            "wait_home",
            "home_tunnel",
            "home_safe",
        )
    }
    # 中心占领区四个角点。⚠️ 文件名是 **range.csv**，不是 zone.csv ——
    # 参数名叫 zone_waypoint_file 但航点文件是录的时候起的名，这里显式对上，
    # 不要靠 `f"{target}.csv"` 去推。
    waypoint_files["targets.zone_waypoint_file"] = PathJoinSubstitution(
        [*waypoint_root, "range.csv"]
    )


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
    # ⚠️ 默认 **false**：执行器属于导航栈，`sim.launch.py` / `real.launch.py` 都已经起了
    #    一个（real.launch.py 连决策节点一起起）。这里再起一个就是**两个同名 action
    #    server**，而 DDS 的 service 是"所有 server 都收到请求并各回一份"⇒ 客户端会收到
    #    两份响应，第二份被 rclcpp_action 记为 "unknown goal/result response, ignoring..."，
    #    并且两个执行器会各自驱动底盘互相打架。
    #
    #    只有在"单独跑决策、导航栈里没有执行器"时才把它设成 true。
    declare_start_waypoint_executor = DeclareLaunchArgument(
        "start_waypoint_executor",
        default_value="false",
        description=(
            "是否由本 launch 启动 waypoint_executor。"
            "sim.launch.py / real.launch.py 已经起了执行器，重复启动会造成两个同名 "
            "action server（现象：unknown goal/result response + 两个执行器抢底盘）。"
        ),
    )

    waypoint_follow_executor = Node(
        package="waypoint_editor",
        executable="waypoint_executor",
        name="waypoint_follow_executor",
        output=node_output,
        condition=IfCondition(start_waypoint_executor),
        arguments=["--ros-args", "--log-level", log_level],
    )
    decision_node = Node(
        package="decision",
        # 生产节点是 BT 版（旧 FSM 节点已删除）。节点名保留 bt_action_replacement，
        # 以便沿用 config/bt_action_replacement.yaml 的根键与 bringup 现有参数覆盖。
        executable="bt_decision_node",
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
            declare_start_waypoint_executor,
            waypoint_follow_executor,
            decision_node,
        ]
    )
