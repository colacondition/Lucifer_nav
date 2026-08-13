import glob
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
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
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    navigation2_launch_dir = os.path.join(
        get_package_share_directory('navigation2'), 'launch')
    serial_driver_launch_dir = os.path.join(
        get_package_share_directory('serial_driver'), 'launch')

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_nav_rviz = LaunchConfiguration('nav_rviz')
    log_level = LaunchConfiguration('log_level')
    node_output = LaunchConfiguration('node_output')
    waypoint_file = LaunchConfiguration('waypoint_file')
    use_serial_driver = LaunchConfiguration('use_serial_driver')
    use_decision = LaunchConfiguration('use_decision')

    # 实车测量参数 (URDF 外参)
    measurement_params = os.path.join(
        bringup_dir, 'config', 'reality', 'measurement_params_real.yaml')
    with open(measurement_params, 'r') as f:
        launch_params = yaml.safe_load(f)

    robot_description = Command([
        'xacro ',
        os.path.join(bringup_dir, 'urdf', 'sentry_robot_real.xacro'),
        ' xyz:=', launch_params['base_link2livox_frame']['xyz'],
        ' rpy:=', launch_params['base_link2livox_frame']['rpy'],
    ])

    navigation_params = os.path.join(bringup_dir, 'config', 'navigation2.yaml')
    fast_location_params = os.path.join(bringup_dir, 'config', 'fast_location_main.yaml')
    seg_params = os.path.join(bringup_dir, 'config', 'reality', 'segmentation_real.yaml')
    # slam_toolbox 建图参数，实车/仿真共用一份（HL 的 real/sim 两份 diff 为空）。
    mapper_params = os.path.join(bringup_dir, 'config', 'mapper_params_online_async.yaml')
    # small_glim 的参数分两层：包内 config/params_*.yaml 是全量默认值（glob 加载，
    # 与上游 launch 的行为一致），bringup 的 small_glim_real.yaml 只放实车差异项。
    small_glim_default_params = sorted(glob.glob(os.path.join(
        get_package_share_directory('small_glim'), 'config', 'params_*.yaml')))
    small_glim_params = os.path.join(
        bringup_dir, 'config', 'reality', 'small_glim_real.yaml')
    mid360_driver_params = os.path.join(
        bringup_dir, 'config', 'reality', 'mid360_driver_real.yaml')
    # 与 sim.launch.py 保持一致：mapping 模式没有 map 帧，用专门的 mapping.rviz。
    rviz_config = PythonExpression([
        "'", os.path.join(bringup_dir, 'rviz', 'mapping.rviz'), "'",
        " if '", mode, "' == 'mapping' else ",
        "'", os.path.join(bringup_dir, 'rviz', 'navigation.rviz'), "'",
    ])
    nav_map_file = [PathJoinSubstitution([bringup_dir, 'map', world]), '.msgpack']
    fast_location_pcd_path = ParameterValue(
        ['package://bringup/PCD/', world, '.pcd'], value_type=str)

    # 与 sim.launch.py 同一套逻辑：建图模式才存图。small_glim 的 enable_mapping
    # 打开时 AsyncMapping 从启动就累积关键帧、Ctrl-C 退出（节点析构）时合并落盘；
    # nav 模式下开着会无上限吃内存。文件名跟着 world 走，否则建 RMUC 会覆盖 RMUL。
    lio_save_map = ParameterValue(
        PythonExpression(["'", mode, "' == 'mapping'"]), value_type=bool)
    lio_map_name = ParameterValue([world, '.pcd'], value_type=str)

    system_libusb_env = {'LD_PRELOAD': '/lib/x86_64-linux-gnu/libusb-1.0.so.0'}
    common_log_arguments = ['--ros-args', '--log-level', log_level]

    # ===== 参数声明 =====
    declare_world = DeclareLaunchArgument('world', default_value='RMUL')
    declare_mode = DeclareLaunchArgument('mode', default_value='nav')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    declare_nav_rviz = DeclareLaunchArgument('nav_rviz', default_value='False')
    declare_log_level = DeclareLaunchArgument('log_level', default_value='warn')
    declare_node_output = DeclareLaunchArgument('node_output', default_value='log')
    declare_waypoint_file = DeclareLaunchArgument(
        'waypoint_file', default_value='/tmp/navigation_waypoints.csv')
    # 建图输出目录，默认就是 mode:=nav 下 fast_location 要读的地方，建完直接能用。
    # small_glim 的 mapping.output_dir 要给绝对路径（留空时它会退回 ~/mapping 并拼
    # 时间戳子目录）。--symlink-install 下覆盖已有世界会写穿到 src/bringup/PCD/，
    # 新世界的文件只落在 install/ 里，下次 colcon build 就没了，要自己拷回源码。
    declare_map_save_dir = DeclareLaunchArgument(
        'map_save_dir', default_value=os.path.join(bringup_dir, 'PCD'))
    # 串口是实车链路的终点：它订阅 /cmd_vel_chassis 下发底盘速度，并把裁判系统
    # 的 robot_status / game_status 喂给决策节点。没有它，导航算出的速度无人接收。
    # 默认开启；缺少串口设备时用 use_serial_driver:=False 关掉。
    declare_use_serial_driver = DeclareLaunchArgument(
        'use_serial_driver', default_value='True',
        description='Start serial_driver (chassis cmd downlink + referee uplink)')
    declare_use_decision = DeclareLaunchArgument(
        'use_decision', default_value='True',
        description='Start the decision node (needs serial_driver for referee data)')

    # ===== 1. robot_state_publisher (发布 base_link TF) =====
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output=node_output,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
        arguments=common_log_arguments)

    # ===== 2. Mid360 驱动（自研，被动收 UDP 推流）=====
    # 不向雷达发配置命令：需事先用 Livox Viewer 2 把推流目标主机
    # (host_ip, 端口 56301/56401) 持久化写入雷达。话题/frame 等在 yaml 里。
    lidar_driver = Node(
        package='mid360_driver',
        executable='mid360_driver_node',
        output=node_output,
        parameters=[
            mid360_driver_params,
            {'use_sim_time': use_sim_time},
        ],
        arguments=common_log_arguments)

    # ===== 3. small_glim (里程计 + 建图) =====
    # 话题名（/Odometry、/lio/robo/odom、/Laser_map）直接在包内 params_node.yaml
    # 里按本工作区契约配置，无需 remap。参数顺序有意义：后面的覆盖前面的。
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

    # 连接 odom 与 small_glim 输出系（lidar_odom/world），否则 TF 树断裂
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

    # ===== 4. 感知链 =====
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

    # ===== 4.5 建图链：点云转激光 + slam_toolbox（仅 mode:=mapping）=====
    # 为什么不用 pcd_to_navmap 的高度切片出图：实测 RMUL.pcd 地面起伏 ~0.4 m，
    # 全局 z 阈值必然满图误判。slam_toolbox 逐帧做射线更新，激光穿过的格子被
    # 反复标空闲，孤立噪点自然被洗掉。输入用去地面后的障碍点云
    # （/segmentation/obstacle），坡道/高地的地面点不会进图；odom->base_link 的
    # TF 由 small_glim 提供（enable_tf_publish: true），slam_toolbox 在其上发布
    # map->odom。建完图另开终端存 pgm+yaml，再转 msgpack：
    #   ros2 run nav2_map_server map_saver_cli -f src/bringup/map/<world>
    #   python3 tools/pgm_to_navmap.py map/<world>.yaml
    cloud_to_scan_node = Node(
        condition=LaunchConfigurationEquals('mode', 'mapping'),
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output=node_output,
        remappings=[
            ('cloud_in', '/segmentation/obstacle'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            # 投影到底盘系：侧倾时仍输出重力对齐的 2D 扫描（同 HL 的做法）。
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': 0.05,
            'max_height': 1.2,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        arguments=common_log_arguments)

    slam_mapping_node = Node(
        condition=LaunchConfigurationEquals('mode', 'mapping'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output=node_output,
        parameters=[mapper_params, {'use_sim_time': use_sim_time}],
        arguments=common_log_arguments)

    # ===== 5. fast_location 主定位 =====
    fast_loc_node = Node(
        condition=LaunchConfigurationEquals('mode', 'nav'),
        package='fast_location',
        executable='robot_localization_node',
        name='robot_localization_node',
        output='screen',
        additional_env=system_libusb_env,
        # 基线参数来自 config/fast_location_main.yaml；下面的字典只覆盖实车
        # 特有项（PCD 路径按 world 拼接、话题重映射、按算力收紧的体素/线程数）。
        # 顺序有意义：后面的条目覆盖前面的。
        parameters=[fast_location_params, {
            'use_sim_time': use_sim_time,
            'map_pcd_path': fast_location_pcd_path,
            'sub_scan_topic': '/Laser_map',
            # 实车算力有限：体素放粗、线程收到 2，牺牲一点精度换实时性。
            'scan_voxel_size': 0.20,
            'submap_voxel_size_first': 0.20,
            'submap_voxel_size_track': 0.35,
            'fov_far': 12.0,
            'localization_rate_hz': 4.0,
            'gicp_num_threads': 2,
            # 全局地图只给 RViz 看，实车压到 0.2Hz 省带宽。
            'map_publish_rate_hz': 0.2,
        }],
        arguments=['--ros-args', '--log-level', 'info'])

    # ===== 6. Navigation2 导航栈 =====
    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation2_launch_dir, 'bringup.launch.py')),
        condition=LaunchConfigurationEquals('mode', 'nav'),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav_map_file,
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

    # ===== 9. 串口驱动（底盘速度下行 + 裁判系统上行）=====
    serial_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(serial_driver_launch_dir, 'serial_driver.launch.py')),
        condition=IfCondition(use_serial_driver),
        launch_arguments={
            'log_level': log_level,
            'node_output': node_output,
        }.items())

    # ===== 10. 决策节点 =====
    # 这里直接起节点而不 include decision.launch.py：后者会再起一个
    # waypoint_follow_executor，和上面第 8 节的重复。
    #
    # decision 的路径一律走 FindPackageShare 而非 get_package_share_directory：
    # 后者在构建 LaunchDescription 时就会解析，未安装 decision 时即使
    # use_decision:=False 也会抛异常，把整条导航链带崩。
    decision_share = FindPackageShare('decision')
    decision_waypoint_files = {
        f'targets.{target}_waypoint_file': PathJoinSubstitution(
            [bringup_dir, 'config', 'waypoints', 'RMUL', f'{target}.csv'])
        for target in (
            'patrol', 'center', 'wait_center', 'home', 'wait_home', 'wait_hp')
    }
    decision_node = Node(
        condition=IfCondition(use_decision),
        package='decision',
        executable='bt_action_replacement_node',
        name='bt_action_replacement',
        output=node_output,
        # yaml 的根键是 bt_action_replacement，必须与上面的 name 一致才生效。
        parameters=[
            PathJoinSubstitution(
                [decision_share, 'config', 'bt_action_replacement.yaml']),
            {'use_sim_time': use_sim_time},
            decision_waypoint_files,
        ],
        arguments=common_log_arguments)

    # ===== 11. RViz =====
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
        declare_nav_rviz, declare_log_level, declare_node_output,
        declare_waypoint_file, declare_map_save_dir,
        declare_use_serial_driver, declare_use_decision,
        robot_state_pub,
        lidar_driver,
        lio_node,
        tf_odom_to_lidar_odom,
        tf_odom_to_world,
        lidar_filter_node,
        ground_seg_node,
        cloud_to_scan_node,
        slam_mapping_node,
        fast_loc_node,
        start_navigation,
        vel_transform_node,
        waypoint_follow_executor,
        waypoint_patrol_executor,
        serial_driver,
        decision_node,
        nav_rviz_node,
    ]:
        ld.add_action(action)

    return ld
