import glob
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    simulation_launch_dir = os.path.join(
        get_package_share_directory('pb_rm_simulation'), 'launch')
    navigation2_launch_dir = os.path.join(
        get_package_share_directory('navigation2'), 'launch')
    fast_location_dir = get_package_share_directory('fast_location')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_nav_rviz = LaunchConfiguration('nav_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    log_level = LaunchConfiguration('log_level')
    node_output = LaunchConfiguration('node_output')
    waypoint_file = LaunchConfiguration('waypoint_file')

    # 测量参数 (URDF 中 base_link 到 livox_frame 的外参)
    measurement_params = os.path.join(
        bringup_dir, 'config', 'simulation', 'measurement_params_sim.yaml')
    with open(measurement_params, 'r') as f:
        launch_params = yaml.safe_load(f)

    robot_description = Command([
        'xacro ',
        os.path.join(bringup_dir, 'urdf', 'sentry_robot_sim.xacro'),
        ' xyz:=', launch_params['base_link2livox_frame']['xyz'],
        ' rpy:=', launch_params['base_link2livox_frame']['rpy'],
    ])

    navigation_params = os.path.join(bringup_dir, 'config', 'navigation2.yaml')
    fast_location_params = os.path.join(bringup_dir, 'config', 'fast_location_main.yaml')
    seg_params = os.path.join(bringup_dir, 'config', 'simulation', 'segmentation_sim.yaml')
    # small_glim 的参数分两层：包内 config/params_*.yaml 是全量默认值（glob 加载，
    # 与上游 launch 的行为一致），bringup 的 small_glim_sim.yaml 只放仿真差异项。
    small_glim_default_params = sorted(glob.glob(os.path.join(
        get_package_share_directory('small_glim'), 'config', 'params_*.yaml')))
    small_glim_params = os.path.join(
        bringup_dir, 'config', 'simulation', 'small_glim_sim.yaml')
    # RViz 配置按 mode 选：navigation.rviz 的 Fixed Frame 是 map，而 map 只有
    # nav 模式下的 fast_location / map_server 才发；建图模式下用 mapping.rviz
    # （Fixed Frame=world，带 /Laser_map 显示）。见 rviz/mapping.rviz 顶部说明。
    rviz_config = PythonExpression([
        "'", os.path.join(bringup_dir, 'rviz', 'mapping.rviz'), "'",
        " if '", LaunchConfiguration('mode'), "' == 'mapping' else ",
        "'", os.path.join(bringup_dir, 'rviz', 'navigation.rviz'), "'",
    ])
    nav_map_yaml = [PathJoinSubstitution([bringup_dir, 'map', world]), '.yaml']
    fast_location_pcd_path = ParameterValue(
        ['package://bringup/PCD/', world, '.pcd'], value_type=str)

    # 建图模式才存图：small_glim 的 enable_mapping 打开时 AsyncMapping 从启动就
    # 累积关键帧、Ctrl-C 退出（节点析构）时合并落盘；nav 模式下开着会无上限吃内存，
    # 所以按 mode 开关。
    lio_save_map = ParameterValue(
        PythonExpression(["'", LaunchConfiguration('mode'), "' == 'mapping'"]),
        value_type=bool)
    # 存图文件名跟着 world 走。yaml 里硬写成 RMUL.pcd，建 RMUC 的图会覆盖掉 RMUL。
    lio_map_name = ParameterValue([world, '.pcd'], value_type=str)

    system_libusb_env = {'LD_PRELOAD': '/lib/x86_64-linux-gnu/libusb-1.0.so.0'}
    common_log_arguments = ['--ros-args', '--log-level', log_level]

    # ===== 参数声明 =====
    declare_world = DeclareLaunchArgument('world', default_value='RMUL')
    declare_mode = DeclareLaunchArgument('mode', default_value='nav')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    declare_nav_rviz = DeclareLaunchArgument('nav_rviz', default_value='True')
    declare_gazebo_gui = DeclareLaunchArgument('gazebo_gui', default_value='True')
    declare_log_level = DeclareLaunchArgument('log_level', default_value='warn')
    declare_node_output = DeclareLaunchArgument('node_output', default_value='log')
    declare_waypoint_file = DeclareLaunchArgument(
        'waypoint_file', default_value='/tmp/navigation_waypoints.csv')
    declare_software_rendering = DeclareLaunchArgument(
        'software_rendering', default_value='False')
    # 建图输出目录。默认就是 mode:=nav 下 fast_location 要读的地方
    # （fast_location_pcd_path = package://bringup/PCD/<world>.pcd），建完直接能用。
    #
    # 两个坑：
    # 1) small_glim 的 mapping.output_dir 要给绝对路径：留空时它会退回 ~/mapping
    #    并拼时间戳子目录，跟 fast_location 找的位置对不上。
    # 2) --symlink-install 下 install/.../PCD/RMUL.pcd 是指向源码的符号链接，覆盖它
    #    会写穿到 src/bringup/PCD/RMUL.pcd —— 重建已有世界的图正是想要这样。但新世界
    #    的文件只会落在 install/ 里，下次 colcon build 就没了，要自己拷回源码。
    declare_map_save_dir = DeclareLaunchArgument(
        'map_save_dir', default_value=os.path.join(bringup_dir, 'PCD'))

    # 无 GPU 环境下用软件渲染，避免 RViz/Gazebo 段错误
    enable_software_gl = SetEnvironmentVariable(
        'LIBGL_ALWAYS_SOFTWARE', '1',
        condition=IfCondition(LaunchConfiguration('software_rendering')))

    # ===== 1. Gazebo 仿真环境 =====
    start_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_launch_dir, 'rm_simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'robot_description': robot_description,
            'gazebo_gui': gazebo_gui,
            'sim_lidar_downsample': '3',
            'gazebo_clock_rate': '100.0',
            'rviz': 'False',
            'log_level': log_level,
            'node_output': node_output,
        }.items())

    # ===== 2. small_glim (里程计 + 建图) =====
    # 话题名（/Odometry、/lio/robo/odom、/Laser_map）直接在包内 params_node.yaml
    # 里按本工作区契约配置，无需 remap。点云来自 Gazebo livox 插件的
    # /livox/lidar/pointcloud（无逐点时间戳，small_glim 自动生成伪时间戳）。
    # 参数顺序有意义：后面的覆盖前面的。
    lio_node = Node(
        package='small_glim',
        executable='small_glim_node',
        output='log',
        parameters=small_glim_default_params + [
            small_glim_params,
            {
                'use_sim_time': use_sim_time,
                'node.enable_mapping': lio_save_map,
                'mapping.output_dir': LaunchConfiguration('map_save_dir'),
                'mapping.map_name': lio_map_name,
            },
        ],
        arguments=common_log_arguments)

    # 连接 odom 与 Super-LIO 输出系（lidar_odom/world），否则 TF 树断裂
    tf_odom_to_lidar_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='log',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'lidar_odom'])
    tf_odom_to_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='log',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'world'])

    # ===== 3. 感知链 =====
    lidar_filter_node = Node(
        package='cpp_lidar_filter',
        executable='lidar_filter_node',
        name='lidar_filter',
        output=node_output,
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/livox/lidar/pointcloud',
            'output_topic': '/livox/lidar_filtered/pointcloud',
            'navigation_frame': 'base_link',
            'navigation_range': 10.0,
            'leaf_size': 0.06,
        }],
        arguments=common_log_arguments)

    ground_seg_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        name='ground_segmentation',
        output=node_output,
        additional_env=system_libusb_env,
        parameters=[seg_params, {'use_sim_time': use_sim_time}],
        arguments=common_log_arguments)

    # ===== 4. fast_location 主定位 =====
    fast_loc_node = Node(
        condition=LaunchConfigurationEquals('mode', 'nav'),
        package='fast_location',
        executable='robot_localization_node',
        name='robot_localization_node',
        output='screen',
        additional_env=system_libusb_env,
        # 基线参数来自 config/fast_location_main.yaml；下面只覆盖仿真特有项。
        # 顺序有意义：后面的条目覆盖前面的。
        parameters=[fast_location_params, {
            'use_sim_time': use_sim_time,
            'map_pcd_path': fast_location_pcd_path,
            'sub_scan_topic': '/Laser_map',
            'scan_voxel_size': 0.20,
            'submap_voxel_size_first': 0.20,
            'submap_voxel_size_track': 0.35,
            'fov_far': 12.0,
            'localization_rate_hz': 4.0,
            'gicp_num_threads': 2,
            'map_publish_rate_hz': 0.2,
        }],
        arguments=['--ros-args', '--log-level', 'info'])

    # ===== 5. Navigation2 导航栈 (容器内启动 map_server) =====
    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation2_launch_dir, 'bringup.launch.py')),
        condition=LaunchConfigurationEquals('mode', 'nav'),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav_map_yaml,
            'params_file': navigation_params,
            'start_map_server': 'True',
            'start_mpc_controller': 'True',
        }.items())

    # ===== 7. 速度转换 =====
    vel_transform_node = Node(
        condition=LaunchConfigurationEquals('mode', 'nav'),
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        name='fake_vel_transform',
        output=node_output,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=common_log_arguments)

    # ===== 8. Waypoint Editor Executors =====
    waypoint_follow_executor = Node(
        condition=LaunchConfigurationEquals('mode', 'nav'),
        package='waypoint_editor',
        executable='waypoint_follow_executor',
        name='waypoint_follow_executor',
        output=node_output,
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoint_file': waypoint_file,
            'goal_topic': '/goal_pose',
            'status_topic': '/navigation2/status',
            'saved_waypoint_file_topic': '/waypoint_editor/saved_waypoint_file',
        }],
        arguments=common_log_arguments)

    waypoint_patrol_executor = Node(
        condition=LaunchConfigurationEquals('mode', 'nav'),
        package='waypoint_editor',
        executable='waypoint_patrol_executor',
        name='waypoint_patrol_executor',
        output=node_output,
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoint_file': waypoint_file,
            'goal_topic': '/goal_pose',
            'status_topic': '/navigation2/status',
        }],
        arguments=common_log_arguments)

    # ===== 9. RViz =====
    nav_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', log_level],
        condition=IfCondition(use_nav_rviz))

    ld = LaunchDescription()
    for action in [
        declare_world, declare_mode, declare_use_sim_time,
        declare_nav_rviz, declare_gazebo_gui, declare_log_level, declare_node_output,
        declare_software_rendering, declare_waypoint_file, declare_map_save_dir,
        enable_software_gl,
        start_simulation,
        lio_node,
        tf_odom_to_lidar_odom,
        tf_odom_to_world,
        lidar_filter_node,
        ground_seg_node,
        fast_loc_node,
        start_navigation,
        vel_transform_node,
        waypoint_follow_executor,
        waypoint_patrol_executor,
        nav_rviz_node,
    ]:
        ld.add_action(action)

    return ld
