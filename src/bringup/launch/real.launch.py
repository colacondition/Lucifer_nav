import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


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
    mapping_nav = LaunchConfiguration('mapping_nav')
    perception_threads = LaunchConfiguration('perception_threads')

    # 建图模式下可选再跑导航（SLAM-navigation）：map 帧与 /map 由 slam_toolbox 边扫边
    # 发，语义地图缺席（还没有 .msgpack 可读），隧道相关逻辑全部退化失效，仅作普通
    # 2D 导航用。默认关：mode:=mapping 只建图（slam_toolbox + small_glim）；
    # 要边建边导航再显式 mapping_nav:=True。
    nav_condition = IfCondition(PythonExpression([
        "'", mode, "' == 'nav' or ('", mode, "' == 'mapping' and '",
        mapping_nav, "' == 'True')"
    ]))
    mapping_nav_condition = IfCondition(PythonExpression([
        "'", mode, "' == 'mapping' and '", mapping_nav, "' == 'True'"
    ]))

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

    # 运行期参数唯一权威源：先加载 common，再加载 reality 覆盖。
    common_config_dir = os.path.join(bringup_dir, 'config', 'common')
    reality_config_dir = os.path.join(bringup_dir, 'config', 'reality')
    navigation_params = os.path.join(common_config_dir, 'navigation2.yaml')
    fast_location_params = os.path.join(common_config_dir, 'fast_location.yaml')
    fast_location_env_params = os.path.join(reality_config_dir, 'fast_location.yaml')
    seg_params = os.path.join(common_config_dir, 'segmentation.yaml')
    seg_env_params = os.path.join(reality_config_dir, 'segmentation.yaml')
    mapper_params = os.path.join(common_config_dir, 'mapper.yaml')
    lidar_filter_params = os.path.join(common_config_dir, 'lidar_filter.yaml')
    cloud_to_scan_params = os.path.join(common_config_dir, 'pointcloud_to_laserscan.yaml')
    waypoint_executor_params = os.path.join(common_config_dir, 'waypoint_executor.yaml')
    decision_params = os.path.join(common_config_dir, 'decision.yaml')
    serial_driver_params = os.path.join(reality_config_dir, 'serial_driver.yaml')
    small_glim_common_params = os.path.join(common_config_dir, 'small_glim.yaml')
    small_glim_params = os.path.join(reality_config_dir, 'small_glim_real.yaml')
    mid360_driver_params = os.path.join(
        bringup_dir, 'config', 'reality', 'mid360_driver_real.yaml')
    # 与 sim.launch.py 保持一致：mapping 模式没有 map 帧，用专门的 mapping.rviz
    # （Fixed Frame=odom）。建图 + 导航（mapping_nav）用 mapping_nav.rviz：
    # Fixed Frame 取 map（slam_toolbox 发），带 GoalTool 和 /map，保留 /Laser_map（odom 系）。
    rviz_config = PythonExpression([
        "'", os.path.join(bringup_dir, 'rviz', 'mapping_nav.rviz'), "'",
        " if ('", mode, "' == 'mapping' and '", mapping_nav, "' == 'True') else ",
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
    declare_mapping_nav = DeclareLaunchArgument(
        'mapping_nav', default_value='False',
        description='If True with mode:=mapping, also run the nav stack (SLAM-navigation). Default False: mapping is slam_toolbox only.')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    declare_nav_rviz = DeclareLaunchArgument('nav_rviz', default_value='False')
    declare_log_level = DeclareLaunchArgument('log_level', default_value='warn')
    declare_node_output = DeclareLaunchArgument('node_output', default_value='log')
    declare_perception_threads = DeclareLaunchArgument(
        'perception_threads', default_value='1',
        description='感知容器 executor 线程数（lidar_filter + ground_segmentation）')
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
        respawn=True, respawn_delay=2.0,  # 驱动崩溃自愈（纯被动收包，无状态）
        package='mid360_driver',
        executable='mid360_driver_node',
        output=node_output,
        parameters=[
            mid360_driver_params,
            {'use_sim_time': use_sim_time},
        ],
        arguments=common_log_arguments)

    # ===== 3. small_glim (里程计 + 建图) =====
    # small_glim 完整运行参数来自 bringup common，reality 文件只覆盖实车差异。
    # 话题、frame、线程预算均不再从包内 params_*.yaml 隐式继承。
    lio_node = Node(
        respawn=True, respawn_delay=2.0,  # LIO 崩溃自愈（无状态，重启重新初始化）
        package='small_glim',
        executable='small_glim_node',
        output='log',
        parameters=[
            small_glim_common_params,
            small_glim_params,
            {
                'use_sim_time': use_sim_time,
                'node.enable_mapping': lio_save_map,
                'mapping.output_dir': LaunchConfiguration('map_save_dir'),
                'mapping.map_name': lio_map_name,
            },
        ],
        arguments=common_log_arguments)

    # small_glim 直接广播 odom→base_link（odometry_frame_id / cloud_frame_id 都是 odom），
    # 不再叠 odom→lidar_odom / odom→world 恒等静态 TF。

    # ===== 4. 感知链：lidar_filter + ground_segmentation 同容器 intra-process =====
    # 两节点原为独立进程，/livox/lidar_filtered/pointcloud 整帧点云每次都要走
    # DDS 序列化 + 传输 + 反序列化。合并进同一个 component_container 并显式开启
    # intra-process 后，该 topic 直接以 shared_ptr/unique_ptr 在进程内投递。
    #
    # Humble 的 intra-process 只接受 volatile + keep_last(depth>0) 的 QoS，
    # 传感器点云 topic 用的正是 SensorDataQoS（volatile + best_effort），所以
    # 可以安全打开；与导航容器不同，这里没有 transient_local 限制。
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        respawn=True, respawn_delay=2.0,  # 容器崩溃自愈（两节点均可从参数重建）
        package='cpp_lidar_filter',
        # 固定线程数容器：不用 Humble 自带的 component_container_mt，后者线程数
        # 恒为 hardware_concurrency()（本机 24）。默认 1 个 executor 线程；
        # linefit 内部用 OpenMP 做分片，不必再给 executor 超订。
        executable='perception_container_mt',
        arguments=[perception_threads] + common_log_arguments,
        output=node_output,
        additional_env=system_libusb_env)

    def perception_component(plugin, name, package, parameters):
        return LoadComposableNodes(
            target_container=perception_container,
            composable_node_descriptions=[
                ComposableNode(
                    package=package,
                    plugin=plugin,
                    name=name,
                    parameters=parameters,
                    # Humble 的 component_container 默认不会给加载的组件打开
                    # intra-process，必须逐个组件显式传入。
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
        )

    def make_perception_load_actions():
        return [
            perception_component(
                'cpp_lidar_filter::LidarFilterNode',
                'lidar_filter',
                'cpp_lidar_filter',
                [lidar_filter_params, {'use_sim_time': use_sim_time}]),
            perception_component(
                'linefit_ground_segmentation::SegmentationNode',
                'ground_segmentation',
                'linefit_ground_segmentation_ros',
                [seg_params, seg_env_params, {'use_sim_time': use_sim_time}]),
        ]

    # 容器每次退出（崩溃被 respawn 或正常退出）都重新调度组件加载。Humble 的
    # LoadComposableNodes 只执行一次，respawn 只会拉起空容器；与 navigation2
    # 容器同一套补救逻辑：等 4s 让 rclpy 图缓存里的旧 load_node 服务过期。
    def reload_perception_on_exit(event, context):
        cmd = getattr(event, 'cmd', None)
        if cmd and any('perception_container_mt' in str(part) for part in cmd):
            return [TimerAction(period=4.0, actions=make_perception_load_actions())]
        return None

    reload_perception_components = RegisterEventHandler(
        OnProcessExit(on_exit=reload_perception_on_exit))

    # ===== 4.5 建图链：点云转激光 + slam_toolbox（仅 mode:=mapping）=====
    # 为什么不用 PCD 直转的高度切片出图：实测 RMUL.pcd 地面起伏 ~0.4 m，
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
        parameters=[cloud_to_scan_params, {'use_sim_time': use_sim_time}],
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
        respawn=True, respawn_delay=2.0,  # 定位崩溃自愈（重载 PCD 重新初始化）
        package='fast_location',
        executable='robot_localization_node',
        name='robot_localization_node',
        output='screen',
        additional_env=system_libusb_env,
        # common 完整参数后加载 reality 覆盖；动态路径/时钟保留在 launch。
        parameters=[fast_location_params, fast_location_env_params, {
            'use_sim_time': use_sim_time,
            'map_pcd_path': fast_location_pcd_path,
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

    # 建图模式下的 SLAM-navigation：map 直接吃 slam_toolbox 的 /map，不起
    # RmMapServer（它会跟 slam_toolbox 抢 /map，而且还没有 .msgpack 可加载、没有
    # 语义地图可发）。隧道相关（收云台 / 限速 / 滤顶板）全部离线，仅普通 2D 导航。
    start_navigation_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation2_launch_dir, 'bringup.launch.py')),
        condition=mapping_nav_condition,
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': navigation_params,
            'start_map_server': 'False',
            'start_mpc_controller': 'True',
        }.items())

    # ===== 7. 速度转换已并进 navigation2 容器（fake_vel_transform 组件）=====

    # ===== 8. 航点执行器（单一 waypoint_executor，默认 follow）=====
    waypoint_follow_executor = Node(
        condition=LaunchConfigurationEquals('mode', 'nav'),
        package='waypoint_editor',
        executable='waypoint_executor',
        name='waypoint_follow_executor',
        output=node_output,
        parameters=[waypoint_executor_params, {
            'use_sim_time': use_sim_time,
            'waypoint_file': waypoint_file,
        }],
        arguments=common_log_arguments)

    # ===== 9. 串口驱动（底盘速度下行 + 裁判系统上行）=====
    serial_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(serial_driver_launch_dir, 'serial_driver.launch.py')),
        condition=IfCondition(use_serial_driver),
        launch_arguments={
            'params_file': serial_driver_params,
            'log_level': log_level,
            'node_output': node_output,
        }.items())

    # ===== 10. 决策节点 =====
    # 这里直接起节点而不 include decision.launch.py：后者会再起一个
    # waypoint_follow_executor，和上面第 8 节的重复。
    #
    # decision 的完整静态参数由 bringup common 提供；未安装 decision 时条件为 false，
    # Node 动作不会启动。
    # decision 的完整静态参数在 bringup common；路径按运行 world 动态覆盖。
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
            decision_params,
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
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config, '--ros-args', '--log-level', log_level],
        condition=IfCondition(use_nav_rviz))

    ld = LaunchDescription()
    for action in [
        declare_world, declare_mode, declare_mapping_nav, declare_use_sim_time,
        declare_nav_rviz, declare_log_level, declare_node_output,
        declare_perception_threads,
        declare_waypoint_file, declare_map_save_dir,
        declare_use_serial_driver, declare_use_decision,
        robot_state_pub,
        lidar_driver,
        lio_node,
        perception_container,
        *make_perception_load_actions(),
        reload_perception_components,
        cloud_to_scan_node,
        slam_mapping_node,
        fast_loc_node,
        start_navigation,
        start_navigation_mapping,
        waypoint_follow_executor,
        serial_driver,
        decision_node,
        nav_rviz_node,
    ]:
        ld.add_action(action)

    return ld
