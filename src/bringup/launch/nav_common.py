# 感知栈公共组装：sim.launch.py 与 real.launch.py 的单一真源。
#
# 此前「感知容器 + 组件加载 + 崩溃重载 handler」约 55 行在两个 launch 里逐行
# 级复制，注释靠人工约定同步（架构评审 B5）。两份差异只剩注释 —— 函数参数化
# 后合并到这里。真正的行为输入全部由调用方传入：
#   threads_arg          executor 线程数 LaunchConfiguration
#   log_args             统一日志参数
#   output               节点输出策略
#   additional_env       额外环境变量（如 libusb LD_PRELOAD）
#   use_sim_time         时钟源
#   *_params             三层 config 解析后的 yaml 路径
import launch.actions
import launch.event_handlers
import launch_ros
import launch_ros.descriptions


def build_perception_stack(
    *, threads_arg, log_args, output, additional_env,
    use_sim_time, lidar_filter_params, seg_params, seg_env_params):
    """返回 {container, make_load_actions, reload_handler}。

    container            ComposableNodeContainer（respawn 已开）
    make_load_actions()  -> [LoadComposableNodes]，供初次加载与重载复用
    reload_handler       RegisterEventHandler：容器退出后 4s 重新拉起组件
                         （Humble 的 LoadComposableNodes 只执行一次，respawn
                         只会拉起空容器）
    """
    perception_container = launch_ros.actions.ComposableNodeContainer(
        name='perception_container',
        namespace='',
        respawn=True, respawn_delay=2.0,  # 容器崩溃自愈（两节点均可从参数重建）
        package='cpp_lidar_filter',
        # 固定线程数容器：不用 Humble 自带的 component_container_mt，后者线程数
        # 恒为 hardware_concurrency()。默认 1 个 executor 线程；linefit 内部用
        # OpenMP 做分片，不必再给 executor 超订。
        executable='perception_container_mt',
        arguments=[threads_arg] + log_args,
        output=output,
        additional_env=additional_env)

    def perception_component(plugin, name, package, parameters):
        return launch_ros.actions.LoadComposableNodes(
            target_container=perception_container,
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package=package,
                    plugin=plugin,
                    name=name,
                    parameters=parameters,
                    # Humble 的 component_container 默认不给组件开 intra-process，
                    # 必须逐个显式传入。
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
        )

    def make_load_actions():
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

    def reload_on_exit(event, context):
        cmd = getattr(event, 'cmd', None)
        if cmd and any('perception_container_mt' in str(part) for part in cmd):
            return [launch.actions.TimerAction(period=4.0,
                                                   actions=make_load_actions())]
        return None

    reload_handler = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(on_exit=reload_on_exit))

    return {
        'container': perception_container,
        'make_load_actions': make_load_actions,
        'reload_handler': reload_handler,
    }
