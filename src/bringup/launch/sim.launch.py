import os
import yaml

import sys as _sys
_sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_common
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
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
    simulation_launch_dir = os.path.join(
        get_package_share_directory('pb_rm_simulation'), 'launch')
    navigation2_launch_dir = os.path.join(
        get_package_share_directory('navigation2'), 'launch')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_nav_rviz = LaunchConfiguration('nav_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    log_level = LaunchConfiguration('log_level')
    node_output = LaunchConfiguration('node_output')
    waypoint_file = LaunchConfiguration('waypoint_file')
    mode = LaunchConfiguration('mode')
    mapping_nav = LaunchConfiguration('mapping_nav')
    perception_threads = LaunchConfiguration('perception_threads')
    sim_lidar_downsample = LaunchConfiguration('sim_lidar_downsample')
    gazebo_clock_rate = LaunchConfiguration('gazebo_clock_rate')

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

    # 运行期参数唯一权威源：先加载 common，再加载 simulation 覆盖。
    common_config_dir = os.path.join(bringup_dir, 'config', 'common')
    simulation_config_dir = os.path.join(bringup_dir, 'config', 'simulation')
    navigation_params = os.path.join(common_config_dir, 'navigation2.yaml')
    fast_location_params = os.path.join(common_config_dir, 'fast_location.yaml')
    fast_location_env_params = os.path.join(simulation_config_dir, 'fast_location.yaml')
    seg_params = os.path.join(common_config_dir, 'segmentation.yaml')
    seg_env_params = os.path.join(simulation_config_dir, 'segmentation.yaml')
    mapper_params = os.path.join(common_config_dir, 'mapper.yaml')
    lidar_filter_params = os.path.join(common_config_dir, 'lidar_filter.yaml')
    cloud_to_scan_params = os.path.join(common_config_dir, 'pointcloud_to_laserscan.yaml')
    waypoint_executor_params = os.path.join(common_config_dir, 'waypoint_executor.yaml')
    simulated_gimbal_params = os.path.join(simulation_config_dir, 'simulated_gimbal.yaml')
    small_glim_common_params = os.path.join(common_config_dir, 'small_glim.yaml')
    small_glim_params = os.path.join(simulation_config_dir, 'small_glim_sim.yaml')
    # RViz 配置按 mode 选：navigation.rviz 的 Fixed Frame 是 map，而 map 只有
    # nav 模式下的 fast_location / map_server 才发；建图模式下用 mapping.rviz
    # （Fixed Frame=odom，带 /Laser_map 显示）。见 rviz/mapping.rviz 顶部说明。
    # 建图 + 导航（mapping_nav）用专门的 mapping_nav.rviz：Fixed Frame 取 map
    # （slam_toolbox 发），带 GoalTool 和 /map 显示，同时保留 /Laser_map（odom 系）。
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
    declare_mapping_nav = DeclareLaunchArgument(
        'mapping_nav', default_value='False',
        description='If True with mode:=mapping, also run the nav stack (SLAM-navigation). Default False: mapping is slam_toolbox only.')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    declare_nav_rviz = DeclareLaunchArgument('nav_rviz', default_value='True')
    declare_gazebo_gui = DeclareLaunchArgument('gazebo_gui', default_value='True')
    declare_log_level = DeclareLaunchArgument('log_level', default_value='warn')
    declare_node_output = DeclareLaunchArgument('node_output', default_value='log')
    declare_perception_threads = DeclareLaunchArgument(
        'perception_threads', default_value='1',
        description='感知容器 executor 线程数（lidar_filter + ground_segmentation）')
    declare_sim_lidar_downsample = DeclareLaunchArgument(
        'sim_lidar_downsample', default_value='3')
    declare_gazebo_clock_rate = DeclareLaunchArgument(
        'gazebo_clock_rate', default_value='100.0')
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
            'sim_lidar_downsample': sim_lidar_downsample,
            'gazebo_clock_rate': gazebo_clock_rate,
            'rviz': 'False',
            'log_level': log_level,
            'node_output': node_output,
        }.items())

    # ===== 2. small_glim (里程计 + 建图) =====
    # small_glim 完整运行参数来自 bringup common，simulation 文件只覆盖仿真差异。
    # Gazebo 点云无逐点时间戳时，common 中的 TimeKeeper 规则生成伪时间。
    # 参数顺序有意义：后面的环境与动态 launch 值覆盖 common。
    lio_node = Node(
        respawn=True, respawn_delay=2.0,  # 与 real.launch 对齐：LIO 崩溃自愈
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

    # ===== 3. 感知链：lidar_filter + ground_segmentation 同容器 intra-process =====
    # 与 real.launch.py 同一套：两节点合并进 component_container_mt，显式打开
    # intra-process，/livox/lidar_filtered/pointcloud 整帧点云走进程内指针投递，
    # 省掉 DDS 序列化 + 传输 + 反序列化。topic QoS 为 SensorDataQoS
    # （volatile + best_effort + keep_last(5)），满足 Humble intra-process 的
    # volatile 限制。
    # 感知栈三件套（容器/组件加载/崩溃重载）单一真源：nav_common.py。
    # 两 launch 不再各自复制 55 行，行为差异只可能来自参数。
    perception_stack = nav_common.build_perception_stack(
        threads_arg=perception_threads,
        log_args=common_log_arguments,
        output=node_output,
        additional_env=system_libusb_env,
        use_sim_time=use_sim_time,
        lidar_filter_params=lidar_filter_params,
        seg_params=seg_params,
        seg_env_params=seg_env_params)
    perception_container = perception_stack['container']
    make_perception_load_actions = perception_stack['make_load_actions']
    reload_perception_components = perception_stack['reload_handler']

    # ===== 3.5 建图链：点云转激光 + slam_toolbox（仅 mode:=mapping）=====
    # 与 real.launch.py 同一套（说明也见那边）：去地面障碍点云转 2D 扫描，
    # slam_toolbox 逐帧射线更新出干净的占据栅格。建完图另开终端：
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

    # ===== 4. fast_location 主定位 =====
    fast_loc_node = Node(
        condition=LaunchConfigurationEquals('mode', 'nav'),
        respawn=True, respawn_delay=2.0,  # 定位崩溃自愈（重载 PCD 重新初始化）
        package='fast_location',
        executable='robot_localization_node',
        name='robot_localization_node',
        output='screen',
        additional_env=system_libusb_env,
        # common 完整参数后加载 simulation 覆盖；动态路径/时钟保留在 launch。
        parameters=[fast_location_params, fast_location_env_params, {
            'use_sim_time': use_sim_time,
            'map_pcd_path': fast_location_pcd_path,
        }],
        arguments=['--ros-args', '--log-level', 'info'])

    # ===== 5. Navigation2 导航栈 (容器内启动 map_server) =====
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

    # ===== 6.5 仿真云台模拟器 =====
    # 实车的 /gimbal_posture_state 由 serial_driver 转发电控的持续回传；仿真没有
    # 电控，由这个节点顶替：收到收/放请求后等 action_delay（默认 0.5s）翻转内部
    # 姿态，并以固定频率（默认 20Hz）持续回传当前姿态 —— 跟电控的上报行为一致，
    # 让 MPC 的「等云台收下来再进洞」和 RViz 云台状态显示在仿真里都跑通。
    simulated_gimbal_node = Node(
        condition=nav_condition,
        respawn=True, respawn_delay=2.0,
        package='simulated_gimbal',
        executable='simulated_gimbal_node',
        name='simulated_gimbal',
        output=node_output,
        parameters=[simulated_gimbal_params, {'use_sim_time': use_sim_time}],
        arguments=common_log_arguments)

    # ===== 7. 速度转换已并进 navigation2 容器（fake_vel_transform 组件）=====

    # ===== 8. 航点执行器（单一 waypoint_executor，默认 follow）=====
    # respawn 与 real.launch.py 同步：executor 崩溃自愈，仿真/实车行为一致。
    waypoint_follow_executor = Node(
        condition=LaunchConfigurationEquals('mode', 'nav'),
        package='waypoint_editor',
        executable='waypoint_executor',
        name='waypoint_follow_executor',
        output=node_output,
        respawn=True,
        respawn_delay=2.0,
        parameters=[waypoint_executor_params, {
            'use_sim_time': use_sim_time,
            'waypoint_file': waypoint_file,
        }],
        arguments=common_log_arguments)

    # ===== 9. RViz =====
    nav_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        # RobotModel / TF 必须和导航栈走同一时钟。不传 use_sim_time 时 RViz 用墙钟
        # 去查仿真时间戳的 TF，整车会在 map 系里抖（gimbal_visualizer 里也写过这个坑）。
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config, '--ros-args', '--log-level', log_level],
        condition=IfCondition(use_nav_rviz))

    ld = LaunchDescription()
    for action in [
        declare_world, declare_mode, declare_mapping_nav, declare_use_sim_time,
        declare_nav_rviz, declare_gazebo_gui, declare_log_level, declare_node_output,
        declare_perception_threads, declare_sim_lidar_downsample,
        declare_gazebo_clock_rate,
        declare_software_rendering, declare_waypoint_file, declare_map_save_dir,
        enable_software_gl,
        start_simulation,
        lio_node,
        perception_container,
        *make_perception_load_actions(),
        reload_perception_components,
        cloud_to_scan_node,
        slam_mapping_node,
        fast_loc_node,
        start_navigation,
        start_navigation_mapping,
        simulated_gimbal_node,
        waypoint_follow_executor,
        nav_rviz_node,
    ]:
        ld.add_action(action)

    return ld
