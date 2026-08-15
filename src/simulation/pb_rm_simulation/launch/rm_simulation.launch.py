#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
from launch.actions.append_environment_variable import AppendEnvironmentVariable

# Enum for world types
class WorldType:
    RMUC = 'RMUC'
    RMUL = 'RMUL'

def get_world_config(world_type):
    world_configs = {
        WorldType.RMUC: {
            'x': '3.35',
            'y': '9.6',
            'z': '0.26',
            'yaw': '0.0',
            'world_path': 'RMUC2024_world/rmuc_map_light.world'
        },
        WorldType.RMUL: {
            'x': '4.1',
            'y': '-2.25',
            'z': '0.170',    # base_link 在地面上方（轮子接触地面）
            'yaw': '1.57',
            'world_path': 'RMUL2024_world/rmul27.world'
        }
    }
    return world_configs.get(world_type, None)


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('pb_rm_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Specify xacro path
    lidar_update_rate = LaunchConfiguration('lidar_update_rate')
    imu_update_rate = LaunchConfiguration('imu_update_rate')
    sim_lidar_downsample = LaunchConfiguration('sim_lidar_downsample')
    gazebo_clock_rate = LaunchConfiguration('gazebo_clock_rate')
    default_robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('pb_rm_simulation'), 'urdf', 'simulation_waking_robot.xacro'),
    ' lidar_update_rate:=', lidar_update_rate,
    ' imu_update_rate:=', imu_update_rate,
    ' sim_lidar_downsample:=', sim_lidar_downsample])

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz', default='false')
    use_gazebo_gui = LaunchConfiguration('gazebo_gui')
    robot_description = LaunchConfiguration('robot_description')
    log_level = LaunchConfiguration('log_level')
    node_output = LaunchConfiguration('node_output')

    # 动态障碍物插件（obstacle_plugin/*.so）与 dynamic_obstacles world 已删除：
    # 场景从未启用（world 配置里被注释），插件还是 gitignore 的预编译产物。
    append_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(get_package_share_directory('pb_rm_simulation'), 'meshes')
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=WorldType.RMUC,
        description='Choose <RMUC> or <RMUL>'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_gazebo_gui_cmd = DeclareLaunchArgument(
        'gazebo_gui',
        default_value='True',
        description='Start Gazebo client GUI if true'
    )

    declare_lidar_update_rate_cmd = DeclareLaunchArgument(
        'lidar_update_rate',
        default_value='10',
        description='Gazebo Mid360 LiDAR update rate in Hz'
    )

    declare_imu_update_rate_cmd = DeclareLaunchArgument(
        'imu_update_rate',
        default_value='200',
        description='Gazebo IMU update rate in Hz'
    )

    declare_sim_lidar_downsample_cmd = DeclareLaunchArgument(
        'sim_lidar_downsample',
        default_value='3',
        description='Gazebo-only Livox ray downsample factor'
    )

    declare_gazebo_clock_rate_cmd = DeclareLaunchArgument(
        'gazebo_clock_rate',
        default_value='100.0',
        description='Gazebo /clock publish rate in simulation-time Hz'
    )

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=default_robot_description,
        description='Robot description'
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='ROS log level'
    )
    declare_node_output_cmd = DeclareLaunchArgument(
        'node_output',
        default_value='log',
        description='Node output target'
    )

    # Specify the actions
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(use_gazebo_gui),
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output=node_output,
        arguments=['--ros-args', '--log-level', log_level]
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output=node_output,
        arguments=['--ros-args', '--log-level', log_level]
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        namespace='',
        executable='rviz2',
        output=node_output,
        arguments=['-d' + os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'), '--ros-args', '--log-level', log_level]
    )

    gazebo_parameter_files = []

    def launch_gazebo_server(context, world_config):
        gazebo_params = ParameterFile(
            os.path.join(bringup_dir, 'config', 'gazebo_params.yaml'),
            allow_substs=True,
        )
        gazebo_params_path = gazebo_params.evaluate(context)
        gazebo_parameter_files.append(gazebo_params)

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
                ),
                launch_arguments={
                    'world': os.path.join(
                        bringup_dir, 'world', world_config['world_path']
                    ),
                    'params_file': str(gazebo_params_path),
                }.items(),
            )
        ]

    def create_gazebo_launch_group(world_type):
        world_config = get_world_config(world_type)
        if world_config is None:
            return None

        return GroupAction(
            condition=LaunchConfigurationEquals('world', world_type),
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    output=node_output,
                    arguments=[
                        '-entity', 'robot',
                        '-topic', 'robot_description',
                        '-x', world_config['x'],
                        '-y', world_config['y'],
                        '-z', world_config['z'],
                        '-Y', world_config['yaw']
                    ],
                ),
                OpaqueFunction(
                    function=launch_gazebo_server,
                    kwargs={'world_config': world_config},
                )
            ]
        )

    bringup_RMUC_cmd_group = create_gazebo_launch_group(WorldType.RMUC)
    bringup_RMUL_cmd_group = create_gazebo_launch_group(WorldType.RMUL)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(append_model_path)

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_gazebo_gui_cmd)
    ld.add_action(declare_lidar_update_rate_cmd)
    ld.add_action(declare_imu_update_rate_cmd)
    ld.add_action(declare_sim_lidar_downsample_cmd)
    ld.add_action(declare_gazebo_clock_rate_cmd)
    ld.add_action(declare_robot_description_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_node_output_cmd)
    ld.add_action(gazebo_client_launch)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(bringup_RMUL_cmd_group) # type: ignore
    ld.add_action(bringup_RMUC_cmd_group) # type: ignore

    # Uncomment this line if you want to start RViz
    ld.add_action(start_rviz_cmd)

    return ld
