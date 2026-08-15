import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('navigation2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    start_map_server = LaunchConfiguration('start_map_server')
    start_global_planner = LaunchConfiguration('start_global_planner')
    start_global_costmap = LaunchConfiguration('start_global_costmap')
    start_path_smoother = LaunchConfiguration('start_path_smoother')
    start_local_costmap = LaunchConfiguration('start_local_costmap')
    start_mpc_controller = LaunchConfiguration('start_mpc_controller')
    start_goal_approach_controller = LaunchConfiguration('start_goal_approach_controller')
    start_velocity_smoother = LaunchConfiguration('start_velocity_smoother')
    start_nav2_compat = LaunchConfiguration('start_nav2_compat')
    start_tunnel_posture = LaunchConfiguration('start_tunnel_posture')
    start_gimbal_visualizer = LaunchConfiguration('start_gimbal_visualizer')
    container_name = LaunchConfiguration('container_name')

    common_params = [params_file, {'use_sim_time': use_sim_time}]

    def nav_component(plugin, name, condition, extra_params=None):
        params = list(common_params)
        if extra_params:
            params.append(extra_params)
        return LoadComposableNodes(
            condition=IfCondition(condition),
            target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package='navigation2',
                    plugin=plugin,
                    name=name,
                    parameters=params,
                    # 注意：不要在这里开 use_intra_process_comms。Humble 里开启
                    # intra-process 的节点只要用 transient_local（非 volatile 持久性）
                    # 就会在构造函数抛 "intraprocess communication allowed only
                    # with volatile durability"，而本栈的 /map、语义地图、云台状态、
                    # costmap 全是 transient_local —— 全组件加载失败。
                    # 订阅回调已统一为 ConstSharedPtr 签名（零拷贝的前置条件），
                    # 升级到 Iron+（支持 per-topic IntraProcessQoS）后在这里打开即可。
                )
            ],
        )

    # Single multi-threaded container: all first-party nav components run in
    # one process. 容器内大消息（/map、/plan、costmap）目前仍走 DDS（Humble 的
    # intra-process 与 transient_local 不兼容，见 nav_component 里的注释）。
    container = ComposableNodeContainer(
        name='nav_container',
        namespace='',
        # 容器崩溃自愈：所有组件从文件/参数重建（/map 由 msgpack 重新加载、
        # 路径重新规划），不存在不可恢复的内存态。任何未捕获异常导致的容器
        # 死亡都能在 2s 内自动恢复，而不是留下一个空转的导航栈。
        respawn=True, respawn_delay=2.0,
        package='navigation2',
        # 固定线程数组件容器（Humble 自带 component_container_mt 的线程数
        # 恒为 hardware_concurrency 且不可配）；6 个 executor 线程对互斥回调组
        # 串行化的导航组件已足够，核数无关。
        executable='nav_container_mt',
        arguments=['6'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'params', 'navigation2.yaml')),
        DeclareLaunchArgument('start_map_server', default_value='true'),
        DeclareLaunchArgument('start_global_planner', default_value='true'),
        DeclareLaunchArgument('start_global_costmap', default_value='true'),
        DeclareLaunchArgument('start_path_smoother', default_value='true'),
        DeclareLaunchArgument('start_local_costmap', default_value='true'),
        DeclareLaunchArgument('start_mpc_controller', default_value='true'),
        DeclareLaunchArgument('start_goal_approach_controller', default_value='true'),
        DeclareLaunchArgument('start_velocity_smoother', default_value='true'),
        DeclareLaunchArgument('start_nav2_compat', default_value='true'),
        DeclareLaunchArgument('start_tunnel_posture', default_value='true'),
        DeclareLaunchArgument('start_gimbal_visualizer', default_value='true'),
        DeclareLaunchArgument('container_name', default_value='nav_container'),

        container,

        nav_component(
            'navigation2::RmMapServer', 'rm_map_server', start_map_server,
            {'map_filename': map_file}),
        nav_component('navigation2::RmGlobalCostmap', 'rm_global_costmap', start_global_costmap),
        nav_component('navigation2::RmGlobalPlanner', 'rm_global_planner', start_global_planner),
        nav_component('navigation2::RmMincoPathSmoother', 'rm_minco_path_smoother', start_path_smoother),
        nav_component('navigation2::RmLocalCostmap', 'rm_local_costmap', start_local_costmap),
        nav_component('navigation2::RmMpcController', 'rm_mpc_controller', start_mpc_controller),
        nav_component('navigation2::RmVelocitySmoother', 'rm_velocity_smoother',
                      start_velocity_smoother),
        nav_component('navigation2::RmNav2Compat', 'rm_nav2_compat', start_nav2_compat),
        # 隧道云台请求：图里没有隧道时它只是每 0.1 s 发一个 false，成本可忽略，
        # 所以默认开着 —— 忘记开的代价是云台撞在顶板上。
        nav_component('navigation2::RmTunnelPosture', 'rm_tunnel_posture', start_tunnel_posture),
        # 云台状态可视化：只读电控回传和请求，不碰控制，纯显示。实车仿真都要看，
        # 默认开。忘开的代价只是 RViz 里看不到云台状态，不影响行驶。
        nav_component('navigation2::RmGimbalVisualizer', 'rm_gimbal_visualizer',
                      start_gimbal_visualizer),

        # goal_approach_controller lives in its own package (PCL-free, separate deps)
        # but composes into the same container.
        LoadComposableNodes(
            condition=IfCondition(start_goal_approach_controller),
            target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package='goal_approach_controller',
                    plugin='goal_approach_controller::GoalApproachControllerNode',
                    name='goal_approach_controller',
                    parameters=common_params,
                )
            ],
        ),
    ])
