# 内存管理 / 线程进程残留审计报告

审计方式：主代理横切 grep + 3 个并行子代理逐文件读码（navigation2 / 定位栈 / 驱动感知控制仿真）。每条含 文件:行号、严重度、修复状态。

## 已修复（本轮）

| 发现 | 位置 | 修复 |
| :- | :- | :- |
| nav2_compat 每个 goal 一个 detached 线程：组件卸载 UAF + 线程堆积 + 抢占不生效 | nav2_compat_node.cpp:59 | 单执行线程 + 目标槽位 + 代次抢占 + 析构 join ✅ |
| mpc_solver osqp_setup 失败路径泄漏 data_/settings_/csc 结构 | mpc_solver.hpp:268-273 | 失败分支释放全部自分配结构 ✅ |
| global_planner mt 容器并发回调无锁（map_/goal_/last_path_/清障缓存/轴线表竞争，plan_mtx_ 声明未用） | global_planner_node.cpp | 全部订阅+定时器入 MutuallyExclusive 回调组 ✅ |
| global/local costmap、tunnel_posture、velocity_smoother、gimbal_visualizer 同类「订阅写 shared_ptr / 定时器读」竞争 | 各 node.cpp | 同款回调组串行化 ✅ |
| linefit Viewer 死锁：drawThread 持锁自旋，visualize/析构可能永久抢不到锁 → 关停挂起 | viewer.cc:87-101 | atomic stop_ + 每轮只持锁 spinOnce 一次 ✅ |
| serial ~Port(){} 空析构 → 组件重启泄漏 fd | serial_port.cpp:273 | ~Port() 调 closePort() ✅ |
| serial transmit_buffer 端口关闭时无界增长 | serial_driver_node.cpp:328 | 上限 10 包，丢最旧（保留最新指令） ✅ |
| small_glim AsyncMapping queue_ 无上限（worker 慢时无界增长） | async_mapping.cpp:258 | 上限 100 帧丢最旧 ✅ |
| waypoint_executor 析构/重启竞态（MT executor 下 UAF 面） | waypoint_executor.cpp | 不可逆 shutdown_ 标志 ✅ |
| minco BandedSystemNoTime::operator= 复用对象时泄漏旧缓冲 | minco.hpp:188 | 先 destroy() 再 create ✅ |
| utils.hpp dt_once 函数级 static 非线程安全（且无调用点） | utils.hpp:15 | 删除死代码 ✅ |
| 全局规划器每拍 A* 分配 g_score/parent/closed（13MB 级 churn） | global_planner_node.cpp | 提为成员缓冲，尺寸不变零分配 ✅ |

## 未修（报告，留待决定）

### 中
- **fast_location 文件级全局变量**（robot_localization.cpp:9-20）：global_map/cur_scan/cur_odom/T_pcd_to_odom/initial_pcd_to_odom 等 9 个文件级非 const 全局，手工锁纪律（已抽查基本在锁内）。当前单实例侥幸安全；建议收进节点类成员（重构面较大）。
- **fast_location scan_downsample_cache_ 无锁共享**（:963-967/:1465/:574）：订阅回调 clear、定时回调读写，仅靠默认互斥回调组串行——脆弱依赖，建议加锁或集中到同一线程。
- **serial reopenPort 持 write_mutex_ sleep 1s**（serial_driver_node.cpp:494-519）：期间整节点（含 receive）冻结；receive() 无锁读 port_->fd 与 close/open 竞争。建议：锁外 sleep 或延迟重连 + receive 纳入 fd 访问保护。
- **odometry_estimation 每帧 iVox 整图深拷贝**（:135）：非泄漏（随 ISAM2 固定滞后 3s 边际化释放），是 3s×帧率 份快照的大瞬态，与已知 L3 优化合并处理。

### 低
- fake_vel_transform：MonotonicStampGate/current_angle_/target_frame_ 两订阅+定时器并发写（组件化进 mt 容器后是竞争面；当前单线程 spin 无碍）。
- goal_approach_controller：每次 cmd_vel 2 次 TF 查询（100Hz+），建议缓存/降频。
- mid360 多雷达 map 按源 IP try_emplace 无上限（异常/伪造源 IP 可扩张）；建议白名单。
- simulated_gimbal action_timer_ 不自取消：动作完成后仍每 0.5s 触发（仅刷日志）。
- livox 插件每帧新建 CustomMsg/PointCloud2（栈对象，高频分配）+ publish_custom/cloud2 硬编码 true 无订阅者也发。
- pointcloud_to_laserscan / decision：析构与缓存均有界/正确；decision 的 async_send_request 捕获 this 在 MT executor 下潜在 UAF（当前单线程 spin）。
- fast_location：initial_pose_cv_（只 notify 不 wait）、accumulated_scan_（只 new 不用）、extractFeatures（只定义不调用）死代码。
- fast_location global_search 每次调用重建整图 KdTreeFLANN（成员 kdtree_global_map 已存在可复用）。
- small_glim get_target_ivox_frame 无订阅者时仍全量拷贝（已知 L3 优化）。
- utils.hpp:17 dt_once 静态局部变量非线程安全（首次并发调用）。

## 已核实无问题的部分
- mid360 析构 stop()+io_context.stop()+join 顺序正确；pending 缓冲有上限。
- small_glim 里程计线程 kill_switch+1ms 轮询退出、输入/内部/输出队列均有上限（慢时丢帧不涨内存）；AsyncMapping request_finish+join 无死锁。
- 我上一轮的 cur_scan swap 语义经复核安全（锁内交接、快照独占、reset 路径不动在途快照）。
- cloud_preprocessor 的裸指针绑定经复核当前安全（下游全部返回 owning 副本）。
- pointcloud_to_laserscan alive_+join 正确。
- 全仓库无 popen/system/fork/atexit；reinterpret_cast 均作用于 packed 结构；minco new[]/delete[] 配对；无孤儿信号处理器。

## 附：执行单元池化管理审计（本轮补充）

### 现状盘点
- **线程池**：全仓库唯一真正的线程池是 linefit 的 `ThreadPool`（本次引入）：
  - 固定大小（n_threads-1 个 worker + 主线程参与）、空闲阻塞在条件变量（无忙等）
  - 代次号防重入执行、shutdown+join 干净退出（已通过 2000 轮压力测试）
  - 生命周期正确：linefit 节点只在构造 + 首次 TF 成功时建 segmenter_（最多 2 次），**没有每帧重建池** ✅
  - 已补注释声明「任务内不可重入 parallelFor」
- **其余并发单元**全部是「单实例长生命周期线程」，均已核实 join 纪律：
  mid360 io_thread ✅ / small_glim 2 线程 ✅ / pointcloud_to_laserscan ✅ /
  nav2_compat（本轮已改为单执行线程+join）✅ / waypoint_executor worker ✅ /
  Viewer 绘制线程（本轮修死锁后 join 必返回）✅。全仓库已无 `.detach()`、无 `std::async`。
- **进程布局（sim.launch）**：约 14 个进程——gazebo、2 个静态 TF、LIO、filter、
  ground_seg、cloud2scan、(mapping 时 slam_toolbox)、fast_location、导航容器（11 组件）、
  simulated_gimbal、fake_vel、2 个航点执行器、RViz。

### 发现与建议（未改动，均为决策项）

1. **容器 executor 线程数不可配（Humble 限制）**：实测 `component_container_mt`
   无视位置参数与 `--thread-count`，始终按 `std::thread::hardware_concurrency()`
   起 executor 线程（本机 24 线程 + 内务 ≈ 35 个 OS 线程；机器人 IPC 按其核数）。
   已见 ros2/rclcpp issue #2930 等讨论。影响：每个空闲 executor 线程占 ~8MB 虚存
   栈与 epoll 句柄，真实 CPU 近零；本轮加了互斥回调组后大部分组件只需 1 个并发
   回调。若机器人侧核数很大（16+）且在意，可写一个 ~20 行的自定义容器 main
   固定线程数替换 executable；否则维持现状即可。
2. **无任何重启/监督策略**：全部 Node 默认 `on_exit=Shutdown()`，无 respawn。
   mid360_driver / small_glim / fast_location / serial 任一崩溃后链路静默残废，
   比赛场景下不会自愈。建议至少给驱动与串口加 `respawn=True, respawn_delay=1.0`
   （launch 的 ExecuteProcess 原生支持），导航容器等有状态组件慎重（重启会丢
   latch 状态，需评估）。
3. **组件容器是单点**：11 个导航组件共享一个进程，任一组件段错误会连坐全栈。
   当前关键路径（驱动/LIO/定位/串口）已隔离在独立进程，属合理取舍；如需进一步
   隔离可把 rm_mpc_controller 拆到第二个容器（收益有限，不建议现在动）。
4. **fast_location 单线程 executor 上跑 TF(50Hz)+定位(5Hz)+地图发布**：重 ICP 会
   短暂延迟 TF 重发（tf2 自行插值兜底）。可接受；如 TF 抖动敏感可把 TF 发布移到
   独立线程/回调组。
5. **运维提示**：`kill -9` 掉 ros2 launch 会孤儿化所有子进程（launch 无 PID 命名
   空间）；日常用 Ctrl-C（launch 会转发 SIGINT 并等待）。团队排障时若用 kill -9，
   记得 `pkill -x component_container_mt` 之类的清理。

### 2026-08-15 补记（本轮三连修，全部已实现并回归）

**D3-1 容器 Ctrl-C 必崩（exit -6）**：`std::terminate ... "Asked to publish result for
goal that does not exist"`。根因是 Humble rclcpp_action 的模板化 `~ServerGoalHandle()`
会对「从未终结」的目标做自动 cancel 过渡（try_canceling → on_terminal_state →
ServerBase::publish_result）；当 C++ 句柄活得比 rcl action server 久（成员销毁顺序：
pending_goal_ 晚于 action_server_）时，publish_result 抛异常且发生在析构中 → terminate。
上游 issue：ros2/rclcpp#2757。修复：RmNav2Compat 析构体在成员销毁前显式 tryAbort 掉
active_handle_（执行线程全程持有）与 pending 槽位目标，让句柄析构不再走自动 cancel。

**D3-2 /segmentation/obstacle 发布 QoS 不兼容（建图地图空的元凶）**：
ground_segmentation 用 SensorDataQoS（best-effort）发布，而 pointcloud_to_laserscan
（cloud_in 重映射到 /segmentation/obstacle）与 RViz 用 RELIABLE 订阅——best-effort
发布者不会给 RELIABLE 订阅者投递任何消息，mapping 模式下 slam_toolbox 一直吃空扫描、
地图建不出来（日志里的 "requesting incompatible QoS ... RELIABILITY_QOS_POLICY" 即此）。
修复：obstacle/ground 发布改 QoS(KeepLast(5)).reliable()（对 best-effort 订阅者兼容，
一改全通）。

**D3-3 ground_segmentation 卡死自证机制**：线上出现 scanCallback 冻结 10s+（障碍点云
停发→代价图 stale→MPC 恢复链→导航 FAILED），SIGINT/SIGTERM 均无效只能 SIGKILL。
修复与诊断：① 两处 TF 查询加超时（重力系 50ms、安装高度 200ms），消灭「timeout 0 无限
等」这一类阻塞；② 新增独立看门狗线程：心跳停顿 >3s 时打印卡住阶段、给 executor 线程
发 SIGUSR2 用 backtrace_symbols_fd 打栈、并 dump 全线程 wchan。下次复现可直接定位。

**D3-4 「MPC 在发指令但车不动」诊断锚点**：fake_vel_transform 现在以 2s 节流 WARN 打印
「收到 /cmd_vel 非零 → 转发 /cmd_vel_chassis」的完整数值，配合
`ros2 topic echo /cmd_vel_chassis` 即可把断点二分到 fake_vel 之前/之后（下游只剩
gazebo_ros_planar_move 插件与物理侧）。

### 2026-08-15 二次补记（D4：分割节点第二次死锁，线程池移除）

现场证据（看门狗 backtrace + 全线程 wchan）：executor 线程卡在
`GroundSegmentation::assignCluster` 的线程池 barrier（pthread_cond_wait），
全部池 worker 睡在 cv_work_ 的 futex 上 —— 与第一次 arrived_ 下溢不同形态的
第二次死锁。该自定义持久线程池（代次号 + arrived_ barrier）连续两次出现
不同死锁，收益只是每帧省 ~100 次线程创建的几十微秒。

处理：
1. **移除线程池**（thread_pool.h 已删除），`runParallel()` 回到上游方案：
   每轮直接起 std::thread + join，无跨轮共享状态，结构上不可能死锁。
2. 看门狗升级：卡住时对进程内**所有**线程发 SIGUSR2（每线程打印自己带 tid
   的 backtrace），任务线程卡在哪一步一目了然；另加 segment() 慢帧告警
   （>100ms 节流 WARN，卡死前可见性能趋势）。
3. 离线压力复现 `log/seg_stress.cpp`：用 mid360.csv 同款射线模板 + 地面/
   高台/箱子/墙合成场景，30000 帧 4 线程跑完无卡死 —— 核心分割算法在常规
   几何下稳定，挂死与线程池强相关。

遗留观察（待下一轮验证）：
- pointcloud_to_laserscan 懒订阅激活后 /scan 应恢复（seg 发布已改 RELIABLE，
  与它的 best-effort 订阅兼容）；slam 拿到扫描后 /map 才有。
- RViz 退出时的 RCLError crash 为 Humble 已知无害 bug，忽略。
