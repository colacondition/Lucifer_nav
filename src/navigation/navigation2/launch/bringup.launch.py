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
    map_yaml = LaunchConfiguration('map')
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
                )
            ],
        )

    # Single multi-threaded container: all first-party nav components run in
    # one process with intra-process zero-copy message passing.
    container = ComposableNodeContainer(
        name='nav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
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
        DeclareLaunchArgument('container_name', default_value='nav_container'),

        container,

        nav_component(
            'navigation2::RmMapServer', 'rm_map_server', start_map_server,
            {'yaml_filename': map_yaml}),
        nav_component('navigation2::RmGlobalCostmap', 'rm_global_costmap', start_global_costmap),
        nav_component('navigation2::RmGlobalPlanner', 'rm_global_planner', start_global_planner),
        nav_component('navigation2::RmMincoPathSmoother', 'rm_minco_path_smoother', start_path_smoother),
        nav_component('navigation2::RmLocalCostmap', 'rm_local_costmap', start_local_costmap),
        nav_component('navigation2::RmMpcController', 'rm_mpc_controller', start_mpc_controller),
        nav_component('navigation2::RmVelocitySmoother', 'rm_velocity_smoother',
                      start_velocity_smoother),
        nav_component('navigation2::RmNav2Compat', 'rm_nav2_compat', start_nav2_compat),

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
