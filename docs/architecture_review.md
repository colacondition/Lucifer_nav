# Lucifer_nav 架构扫描与轻量化建议

> 生成方式：主代理横切面核实 + 5 个子系统深度子代理并行扫描（navigation2 / 定位 / 感知驱动 / 决策控制串口 / 工程基础设施），所有结论均基于实际读码（文件:行号），非 README 转述。

## 实施状态（2026-08 实施轮）

已落地（每项均经编译验证；gtest 回归：navigation2 9/9、decision 5/5、linefit 5/5、fast_location 2/2、cpp_lidar_filter 8/8 通过）：

| 编号 | 改动 | 状态 |
| :- | :- | :- |
| N3 | MPC 常量矩阵移入 configure，30Hz 热路径不再重建 CSC | ✅ |
| N1 | MINCO 障碍代价错接线（机器人多边形 SDF 误当障碍距离场）删除 | ✅ |
| B1 | 删 livox_ros_driver2（5.8MB），CustomMsg 收编进 ros2_livox_simulation | ✅ |
| P1/P2 | cpp_lidar_filter 单遍化（5 次拷贝→1 次）+ 车身裁剪坐标系统一 + ApproximateVoxelGrid + 8 个新单测 | ✅ |
| L10 | fast_location TF 200Hz→50Hz、参数默认收敛；small_glim 点云订阅者门控 | ✅ |
| P8 | 传感器流 QoS 统一 best-effort（发布/订阅两侧同步改，避免断链） | ✅ |
| X5 | navigation2.yaml 参数双源收敛为包内单一真源 | ✅ |
| B2 | build.sh 删手写顺序表，单条 colcon + 包内并行 -l 限负载 | ✅ |
| B3 | 删除 13 个复制粘贴障碍物插件 + 弃用 dynamic world（-15MB） | ✅ |
| X1/N4 | 容器 intra-process 零拷贝：**回调签名部分完成，容器开关已回退**。全部订阅回调已统一为 ConstSharedPtr（零拷贝的前置条件，保留）；但 Humble 限制：开启 intra-process 的节点使用 transient_local 会直接抛 `intraprocess communication allowed only with volatile durability`，而 /map、语义地图、costmap、云台状态全是 transient_local → 全组件加载失败。升 Iron+（支持 per-topic IntraProcessQoS）后在 bringup.launch.py 的 nav_component 处打开即可 | ⚠️ 部分/回退 |
| N2 | 全局规划器距离场 Dijkstra→精确 EDT + 内容哈希缓存 + 惩罚表物化；删除 distance_field 死代码 | ✅ |
| L1 | small_glim 同帧双重降采样去重（分辨率相同时复用里程计结果） | ✅ |
| L4 | fast_location 两处整帧深拷贝改 shared_ptr swap | ✅ |
| D1 | follow/patrol 执行器合并为单一参数化 waypoint_executor（-500 行） | ✅ |
| D5 | goal_tolerance 三处手工对齐 → YAML 锚点共享（goal_approach/MPC） | ✅ |
| D3 | 串口去 asm/termios+ioctl hack、去 sudo chmod（改 udev 提示）、标准波特率、死 VMIN/VTIME | ✅ |
| L2(部分) | fast_location 死参数 use_fast_gicp/refine_fov_far 删除 | ✅ |
| P3 | linefit 持久线程池（修掉了自写池的 arrived_ 减穿死锁）+ 线拟合 QR→闭式 2×2 | ✅ |
| P5 | mid360_driver 发布三遍扫描→单遍直写 | ✅ |
| B7 | ros2_livox_simulation 硬编码 soname → find_package(Boost/Protobuf) | ✅ |

### 第二轮实施（内存/生命周期 + 全量清盘 + 设计重构）

| 编号 | 改动 | 状态 |
| :- | :- | :- |
| H1 | nav2_compat detached 线程 → 单执行线程 + 代次抢占 + join | ✅ |
| H2 | fast_location 9 个文件级全局 → 节点成员 | ✅ |
| H3 | linefit Viewer 持锁自旋死锁 → atomic stop + 单次 spinOnce | ✅ |
| 中 | serial fd 泄漏/发送队列无界/重连锁内 sleep、waypoint 析构竞态、mpc_solver 泄漏、minco operator= 泄漏、dt_once 死代码、A* 缓冲复用 | ✅ |
| L2 | small_glim VGICP 后端 + 非 naive LM + 死函数全清（子代理，0 warning） | ✅ |
| L3 部分 | small_glim ivox 帧按订阅者门控 | ✅ |
| L5 | fast_location 全局搜索复用节点 kd-tree | ✅ |
| P6/P7 | mid360 0x03 球坐标包弃解析 + udp_cnt 重排/去重窗 | ✅ |
| D2 | Trigger 服务 + 三代 generation 关联 → FollowWaypoints action（执行器新增 action server 与 Trigger 并存，decision 客户端重写 −250 行最脆弱代码，11 个新单测） | ✅ |
| D7 | decision 死依赖 ament_index_cpp + 5 个死参数清除 | ✅ |
| 杂项 | goal_approach TF 查询缓存、fake_vel 死代码、simulated_gimbal 一次性定时器、respawn 策略（6 个无状态节点） | ✅ |

| 容器线程数 | 新增 nav_container_mt 固定线程容器（默认 6 executor，替代 hardware_concurrency），launch 已切换 | ✅ |

仍未实施（纯结构/部署项，无运行期收益，已留方案）：N5 costmap 基类抽取、B6 launch 去重、
B5 CI、L6/B4 gcc-13→gcc-11（libfmt-dev 8.1.1 已确认可用：std::format→fmt::format 约 20 处 +
`std::unreachable`→`__builtin_unreachable` + CMake 去 gcc-13 硬编码；因无法在本环境跑 LIO
验证 GTSAM/gtsam_points 混编 ABI，留待实车环境做）。



## 一、现状总览

RoboMaster 哨兵导航工作区，ROS2 Humble + Gazebo Classic 11，第一方代码约 **36K LOC**：

| 子系统 | 规模 | 现状一句话 |
| :- | :- | :- |
| 感知 | ~2.5K LOC | cpp_lidar_filter → linefit 地面分割 → pointcloud_to_laserscan 三节点链 |
| 定位 | ~8.4K LOC | small_glim（GTSAM ISAM2 + GICP/iVox，GLIM 精简）+ fast_location（FastGICP 对先验 PCD） |
| 导航 | 18.5K LOC | 单包 11 组件一容器（A* + 距离场 + MINCO 平滑 + OSQP-MPC），内嵌 MINCO 4.4K |
| 控制 | ~2.7K LOC | goal_approach / velocity_smoother / fake_vel_transform / waypoint_editor |
| 决策 | 2.1K LOC | 手写 std::variant FSM（3 状态），带 5 个 gtest |
| 驱动 | ~0.9K + 5.8M | 自研 asio UDP mid360_driver + 官方 livox_ros_driver2（仅为 2 条消息定义保留） |
| 仿真 | — | Gazebo 场景 + 自研 Livox ODE 插件 + 12 个复制粘贴障碍物插件 |

**做得好、应保持的**：组件化单进程容器架构（免 nav2 pluginlib/lifecycle 重栈）；transient_local QoS 用在所有 latch 话题上（/map、语义地图、costmap）；MPC 的 OSQP warm-start 用法正确；串口协议解析层带 CRC 重同步；注释与参数自检质量高；7 个包有测试。

**三个"重"的根源**：① GTSAM 4.3a0 + gtsam_points + gcc-13/C++23 + `-march=native` ABI 一致性链；② 内嵌 ROSE MINCO 4.4K LOC（求解 20–100ms）；③ 为两条消息定义保留整个官方 Livox 驱动（5.8MB + libpcl-all-dev/apr 依赖）。

## 二、横切面发现（主代理核实）

### X1. 容器"零拷贝"目前不成立（性能）
`navigation2/launch/bringup.launch.py:51-52` 宣称 intra-process zero-copy，但全仓库只有
`fast_location/src/robot_localization.cpp:1665` 开了 `use_intra_process_comms(true)`；且导航组件订阅回调都是
**非 const SharedPtr**（`global_planner_node.cpp:47`、`mpc_controller_node.cpp:347`），即使开 intra-process
也会因可写指针强制拷贝。Humble 默认关闭 intra-process。
**修法**：`ComposableNodeContainer` 加 `extra_arguments=[{'use_intra_process_comms': True}]`，回调改
`UniquePtr`/`const SharedPtr`。容器内 /map、/plan、costmap 大消息即真零拷贝。
（注：message_filters 与 intra-process 不兼容，但其使用者 pointcloud_to_laserscan、fake_vel_transform
都在容器外，安全。）

### X2. 同一障碍数据被重复建场 ≥3 次（性能+优雅）
- `distance_field.cpp`：8 邻域 Dijkstra（结构体堆），全局规划器 clearance 用（`global_planner_node.cpp:573`），**每拍 5Hz 全图重算**
- `rc_esdf.cpp`：`RcEsdfMap` 由 minco_path_smoother（:206）与 mpc_controller（:1249）各持一份
- `semantic_map.cpp:131-213` 已有精确 O(n) Felzenszwalb-Huttenlocher EDT（注释自认 Dijkstra 版 45° 方向高估 ~8%）
**修法**：抽公共 EDT + 按地图缓存，全局规划器复用；MINCO 错接线问题见 N1。

### X3. MINCO 平滑器比被替换的 B 样条慢一个数量级（性能）
MINCO_README 自述：B 样条 ~5ms vs MINCO L-BFGS 20–100ms（默认 max_iterations=4000）。
全局重规划不频繁，可接受；但见 N1——其障碍项当前是死代码，先修正语义再谈调优。

### X4. 构建与仓库卫生（轻量）
- build.sh 单线程顺序编译 19 包，手写依赖顺序表已造成一次真实事故（build.sh:35-38 注释）
- decision `exec_depend` bringup 只为读它的 waypoints CSV（package.xml:35）——运行期数据依赖写成了包依赖
- .git 107MB：RMUL_2024.stl 22M、mid360.csv 16.8M、sim_demo.gif 7.3M + sim_demo.webm 6.3M（同一演示双格式）、
  上游样例 kitti.ply 4.2M、三个版本的 RMUL.pcd（0.6/0.7/1.4M）
- log/ 77 个 build_* 历史目录从不清理；obstacle_plugin 的 .so 是留在源码树的预编译产物、未接 colcon

### X5. 参数双源漂移（优雅/正确性）
`src/bringup/config/navigation2.yaml`（452 行，launch 实际加载）与
`src/navigation/navigation2/params/navigation2.yaml`（336 行，launch 默认值）已漂移 ~116 行
（inflation_radius、clearance_desired_distance、path_acceptance 等不同）。
**修法**：package 内 params 作唯一真源，bringup 只覆盖环境差异，或删一份并在 launch 显式传参。

## 三、子系统建议

### 导航 navigation2

- **N1【最高优先】MINCO 障碍代价错接线（实质死代码）**：
  `minco_path_smoother_node.cpp:61-81` 把 `RcEsdfMap`（由 robot_polygon_ 生成的 2m×2m 机器人多边形 SDF、
  中心在 map 原点）当作"到最近障碍距离"喂 `minco_optimizer.cpp:69-105` 的 obstacle_term；
  真实路径点几乎全在 2×2m 界外 → 查询恒 false → 障碍代价恒 0；`robot_radius` 默认 1.0（实际 0.25）语义也错。
  方案：删 smoother 的 ESDF 实例与 obstacle 项（保留 smoothness+data+tunnel_axis），或接到
  `SemanticMap::sampleCost`（semantic_map.cpp:360，已带梯度）。避障已由 clearance + MPC 局部代价地图兜底。风险低。
- **N2 规划器距离场 EDT 化+缓存**：每拍 planPath(5Hz) 全图 Dijkstra（O(n log n)，45° 高估 8%）+ 重分配
  g_score/parent/closed（:566-569）；复用 semantic_map 已有 O(n) EDT 并按地图缓存；
  clearancePenalty 一次性物化 float 代价数组，A* 扩展查表（替代 :582-588 每步浮点计算）。5Hz 最大 CPU 收益。
- **N3 MPC 常量矩阵只建一次**：`mpc_solver.hpp:57-60` 每拍 solve() 重建 Hessian/约束 CSC 并 sort(:87-207)，
  但 P/A 只依赖参数；移入 configure()，每拍只 update_lin_cost/bounds。30Hz 热路径纯浪费，改动极小。
- **N4 兑现零拷贝**：见 X1。
- **N5 抽公共 costmap 基类**：global/local costmap ~250-300 行逐字重复（processPointCloud 306/675、
  processScan 242/580、inTunnelRegion 301/670、publishFootprint 383/746、makeMarkedCloud 412/771）。
  抽 `CostmapNodeBase`；隧道语义只改一处（现注释反复警告"两边必须一起改"）。
- **N6 局部 costmap 逐格缓存**：local_costmap_node.cpp:856-862 每帧(10Hz)逐格 mapToWorld+查语义格
  → 缓存整数偏移增量更新。
- **N7 清历史包袱**：global_planner_node.cpp:935-940 的 plan_thread_/plan_in_flight_/plan_mtx_ 从未使用；
  utils/params_node.hpp ParamsNode 零引用（或启用它削减每节点 30+ 行 declare_parameter）。
- **N8 依赖瘦身**：nav2_msgs 仅为 nav2_compat 一个 NavigateToPose action → 内联定义去依赖；OSQP 保留
  （Q=diag 小 QP 手写 banded box-QP 风险高不值）；MPC 2D 全向双积分器模型合理不动；
  esdfPathSafe 暴力查询但 esdf.enable 默认 false，默认走代价地图查表，便宜。

### 定位 small_glim + fast_location

- **L1【top】消除每帧两次相同降采样+两次全量拷贝**：small_glim_node.cpp:185,192 对同一帧分别
  preprocess() 与 preprocess_for_localization()，各体素降采样一次并拷贝 times/points/intensities
  （cloud_preprocessor.cpp:62-64,216-218），两分辨率默认都是 0.05。复用结果 + std::move，
  每帧省一次数万点降采样 + 一次 ~3.2MB 拷贝，CPU 约降 15–25%，零风险。
- **L2【top】清死代码**：VGICP 整条后端（odometry_estimation.cpp:109-118,149-164,507-514）+
  GaussianVoxelMap；非 naive LM 初始化（initial_state_estimation.cpp:169-267）；find_imu_data/
  erase_imu_data_up_to；clone 系列；fast_location extractFeatures 整段（robot_localization.cpp:1281-1390）；
  use_fast_gicp_/use_cuda_/refine_fov_far_ 等死参数（runICP 硬编码 FastGICP :1001）。
  删后省 find_package(CUDA)，编译更快、二进制更小。
- **L3【top】iVox 深拷贝 + ivox_cloud 每帧构建**：create_factors 每帧 make_shared<iVox>(*target_ivox)
  深拷贝整张体素图（odometry_estimation.cpp:135）；ivox_cloud 每帧无条件点云化只为 debug 话题
  （async_odometry_estimation.cpp:161）。先做零风险的 debug 门控（get_subscription_count()>0 或 1Hz），
  iVox 快照仅 relinearize 时做（或调大 isam2_relinearize_skip），后者需 perf 验证。
- **L4 fast_location shared_ptr 交换**：*cur_scan = *stacked_scan(:1459) + 锁内 *scan_snapshot = *cur_scan(:410)
  每帧深拷贝两次 → shared_ptr swap，顺带缩小 data_mutex 临界区。
- **L5 assessAlignment 复用 FastGICP 内部近邻**：alignment_quality.hpp:210 每 ICP stage 重建 kd-tree
  → 用 FastGICP::getFitnessScore() 或 gropGlobalMapInFOV 已建 kd-tree；内点比阈值(0.85/0.90)需重标定。
- **L6 降 C++23→C++20 去 gcc-13 PPA**：C++23 实际仅 1 处 std::unreachable（cloud_covariance_estimation.cpp:193）；
  gcc-13 绑架真因是 std::format（Ubuntu 22.04 gcc-11 没有）。换成 __builtin_unreachable() + M_PI + {fmt}，
  即可系统 gcc-11/C++20，去掉 toolchain PPA（约 20 处机械替换）。
- **L7 不要重写图后端（负向结论）**：ISAM2(3s 滞后/30 状态)运行期不重，重量全在构建部署。FAST-LIO ESKF
  去 GTSAM 最彻底但把 GICP 降为 point-to-plane，违反"不降精度"；sophus/manif 不可行（gtsam_points 绑定
  gtsam::Pose3）；small_gicp+g2o 是唯一值得长期观察的选项，但不急。
- **L8 不强行抽公共包**：三套体素降采样/两套 kd-tree 并存是事实，但抽公共包引入跨包 ABI 耦合，
  得不偿失；做减法（删 extractFeatures）即可。
- **L9 fast_location 参数三处默认不一致**：localization_rate_hz 代码 10.0(:79)/包内 yaml 0.5/README 4.0；
  学 small_glim fail-fast，权威收敛到 fast_location_main.yaml 一份；删死参数。
- **L10 发布频率与订阅者门控**：fast_location 200Hz 重发相同 map→odom 且无插值(:1541，README"插值"表述不符)
  → 降到 30–50Hz（tf2 自行插值）；small_glim 1ms wall-timer 空转 + /Laser_map 无订阅者仍逐帧发布
  (small_glim_node.cpp:301) → 订阅者门控。

### 决策/控制/串口

- **D1【top】合并 follow/patrol 执行器**：follow(475 行)/patrol(451 行) 85% 重复，CSV 解析第三份在
  decision waypoint_store.cpp:14-68；且到点判定语义已漂移（follow 用 switch_distance+距离回退 :311-320，
  patrol 认 "GOAL_REACHED" 硬编码 0.35 :271-290）。合并为单一参数化执行器，净删 500-600 LOC。
- **D2【top】Trigger 服务 → ROS2 Action**：waypoint_executor_client.cpp(377 行)维护三代 generation 计数
  与影子状态(:251-266,:51-74,:310-356)——决策链最脆弱代码。改 FollowWaypoints.action，客户端缩到 ~120 行，
  抢占/取消/超时天然解决。
- **D3【top】串口去 hack**：#define termios asmtermios + ioctl(TCGETS2/TCSETS2)(serial_port.cpp:17-24,124-141)
  只为非标波特率，但实际用 115200（代码默认 961200）；sudo chmod 777(:152-169)；O_NONBLOCK 下 VMIN/VTIME
  死代码(:114-115)；reopenPort 锁内 sleep+close/open 竞态(:494-519)。方案：udev 规则替代 sudo，
  boost::asio::serial_port 或 libserial；解析层已健壮保留。
- **D4 fake_vel_transform**：手写角度簿记+cos/sin 旋转(:112-127)；spin_speed 丢弃真实 angular.z(:125)；
  死成员 planner_local_pose_；日志写 "PoseArray" 实际订阅 Path(:56)；100Hz wall_timer+MonotonicStampGate
  补 wall-clock 跳变。方案：TwistStamped + tf2_ros::Buffer::transform，删死代码。
- **D5 三处容差耦合**：goal_approach(166-178) 与 velocity_smoother/MPC 限速重叠；goal_tolerance 在
  MPC(0.25)/goal_approach(0.25)/recovery(0.35) 手工对齐（navigation2.yaml:239-241,278,408）——已踩过
  "到点反复蠕动"。方案：对准段并入 MPC speed_profile.stop_at_goal 或 smoother 近目标限速窗口；
  容差收敛为单一共享参数（短期低风险先做这个）。
- **D6 保持手写 FSM，不引入 BT.CPP**：3 大状态/7 叶子态 ~10 条迁移 + 优先级+记忆保持语义
  （decision_state_machine.cpp:43-169）；BT.CPP 在此规模反而增码。可选：迁移抽数据表 {from,guard,to,reason}。
- **D7 清死依赖/死参数**：ament_index_cpp 零引用；enable_patrol/waypoint_map_name/waypoint_search_root/
  patrol_interval_sec/target_change_hold_sec 只声明不读取。
- **D8 cmd_vel 四跳可压缩**：合并 goal_approach+smoother 减一跳（见 D5）；fake_vel 入容器收益低可缓。

### 工程基础设施（build/仿真/接口）

- **B1【收益最高】删 livox_ros_driver2**：仅为 ros2_livox_simulation 的 CustomMsg 保留整个官方驱动
  （5.8MB + liblivox_lidar_sdk_shared.so 4.7M + rapidjson 712K），拖 libpcl-all-dev/apr/git；
  插件里 CustomMsg 发布被强制开启但全链路无订阅者。方案：2 条消息定义拷进 ros2_livox_simulation/msg/
  （rosidl_generate_interfaces），或直接删发布者。源码 -5.8MB -40+ 文件，部署免 SDK .so。
- **B2 build.sh 去顺序表与 -j1**：单条 `colcon build --executor sequential` +
  `CMAKE_BUILD_PARALLEL_LEVEL=$(nproc)`（或 -l 限负载）；--packages-select/up-to 保留局部构建。
  删 39 行维护成本，编译时间按核数下降。
- **B3 障碍物插件 13 份复制粘贴**：obstacle1-13.cc 同模板仅关键帧不同；obstacle13.cc 未进 CMakeLists（孤儿）；
  CMakeLists 未被 colcon 引用，.so 是 gitignore 的预编译产物，干净 checkout 编不出；dynamic world 被注释。
  方案：弃用则整体删，保留则合并为一个参数化插件（SDF _sdf 读关键帧）。-1300 行、-13×1.1MB .so。
- **B4 gcc-13 收敛**：见 L6（{fmt} 方案后 PPA 可去掉）。
- **B5 加最小 CI + msgpack 一键化**：ccache + colcon build + colcon test（现有 pytest/gtest 复用）；
  pgm_to_navmap 抽无头函数，yaml→msgpack 一条命令。
- **B6 sim/real launch 双份重复**：395 行 vs 426 行几乎逐字重复 → 抽 bringup/launch/_common.py
  （注意 LaunchConfiguration 延迟求值）。
- **B7 次要**：ros2_livox_simulation/CMakeLists 硬编码 libprotobuf.so.9/libboost_chrono.so.1.71.0 soname
  → find_package；simulated_gimbal create_wall_timer 与 use_sim_time 脱钩；custom_msgs 11 条 .msg
  设计干净，GimbalPosture 请求/实测分离**不建议**合并。

### 感知链（mid360_driver / cpp_lidar_filter / linefit / pointcloud_to_laserscan）

- **P1【top】cpp_lidar_filter 单遍化+有类型+ApproximateVoxelGrid**：filter_node.cpp:165-188 对
  PCLPointCloud2 连续 toPCL→ExtractIndices→CropBox→VoxelGrid→fromPCL，每帧 4~5 次全量拷贝；:200-202
  用无类型云，VoxelGrid/CropBox 走运行时字段解析慢路径。方案：一次转 pcl::PointCloud<PointXYZI>，
  base_link 下单遍循环同时做半径+车身+体素（ApproximateVoxelGrid）。~5 次拷贝降 1~2 次，快 2~4 倍，重写 ~60 行。
- **P2 车身裁剪坐标系不一致（正确性）**：body_crop_ 在 livox_frame 下裁剪（:180 无变换），marker 发在
  base_link（:35,124），半径裁剪却变换到 base_link（:149-168）——有安装偏航/偏移时 box 错位。并入 P1 单遍循环统一到 base_link。
- **P3【top】linefit 去线程 churn + 闭式线拟合**：ground_segmentation.cc:63-76/132-144/209-228 三处各
  spawn 4 线程并 join，10Hz 下每秒 120 次线程创建；segment.cc:143-159 每次拟合动态分配 MatrixXd +
  colPivHouseholderQr 求 3~6 点回归；:38-88 用 std::list。方案：持久线程池或单线程 + 2×2 正规方程闭式 + list→vector。
- **P4 Mid360 非重复扫描→不要换 ring 分割（负向结论）**：Mid360 旋转棱镜无固定 scanline，ring index 分割
  不适用；linefit 的 atan2+range polar bin（ground_segmentation.cc:242-244）恰是 ring 无关的正确做法，保留。
- **P5【top】mid360_driver 发布路径单遍化**：mid360_driver_node.cpp:43-48 每包 copy 进 pending；:96-109
  再建 vector<const Point*>；:151-163 第三次写 msg.data——每点 3 次搬运 + 3 遍扫描。方案：解析直写预分配
  扁平缓冲，发布单遍融合过滤直接写 msg.data。3 遍→1 遍，拷贝减半，改 ~80 行。
- **P6 驱动死代码与 0x03 球坐标**：driver.cpp:24-26 保留 0x02/0x03 分支但 Mid360 默认只发 0x01；:330-335
  球坐标 θ/φ 约定与 Livox 官方不符 → 删分支或按官方公式修正。
- **P7 丢包/乱序无处理**：driver.cpp:39 解析 udp_cnt 从不使用 → 小重排窗或丢重复/乱序包（低优先级）。
- **P8 QoS 统一 best-effort**：驱动/filter 默认 reliable、ground_seg 用 SensorDataQoS、laserscan 用 reliable
  （node.cpp:83-84），全链兼容但 reliable 对传感器流是多余开销。
- **P9 依赖瘦身**：cpp_lidar_filter/CMakeLists.txt:10 的 pcl io 组件拉 pcl_io/VTK 无用库；linefit 核心
  package.xml 误声明 pcl_ros+pcl_conversions；viewer.cc:7 无条件编译 pcl_visualization→VTK；
  ground_segmentation_node.cc:7 的 ply_io.h 死 include。
- **P10 pointcloud_to_laserscan 是上游 vendor**：ros-perception 2.0.1 近乎原样 vendor（作者 Paul Bovbel/
  Michel Hidalgo），非自研 → 改 apt 包（ros-humble-pointcloud-to-laserscan）或合并进 linefit 单包。
- **P11 扫描质量参数**：real.launch.py:238-241 scan_time=0.3333 与实际 10Hz 不符；max_height 1.2 会纳入
  低矮顶板，按场地收窄。

## 四、ROI 总排序与分期路线

**第一期 · quick wins（半天级，零/低风险）**
1. B1 删 livox_ros_driver2 —— 三重收益，风险最低
2. N3 MPC 常量矩阵只建一次 —— 30Hz 热路径，改动极小
3. N1 修/删 MINCO 障碍代价错接线 —— 死代码澄清
4. P1+P2 cpp_lidar_filter 单遍化 —— 链上最大拷贝源头，快 2~4 倍
5. L10 订阅者门控 + TF 降频、P8 QoS best-effort —— 纯 CPU 节省
6. X5 参数双源收敛、L9 fast_location 参数收敛 —— 正确性

**第二期 · 结构性重构（周级）**
1. N2 距离场 EDT 化+缓存（+X2 去重）
2. L1 去重降采样、L4 shared_ptr 交换
3. X1/N4 兑现容器零拷贝
4. P3 linefit 去线程 churn、P5 驱动单遍化
5. D1 follow/patrol 合并、D3 串口去 hack
6. B2 build.sh 单条 colcon、B3 障碍物插件合并/删除
7. N5 costmap 基类抽取

**第三期 · 大手术（需联调回归）**
1. D2 Trigger→Action
2. D5 三处容差收敛 + goal_approach 合并
3. L6/B4 降 C++20 去 gcc-13 PPA
4. L2 全量死代码清理（VGICP 后端等）、P9/P10 依赖瘦身
5. L3 iVox 快照策略优化（perf 验证）
6. B5 CI + msgpack 一键化、B6 launch 去重

## 五、明确不建议做的事

- **不引入 BehaviorTree.CPP**（决策只有 3 状态，BT 反而增码）
- **不重写为 FAST-LIO ESKF**（GICP→point-to-plane 降精度）
- **不换 sophus/manif**（gtsam_points 绑定 gtsam::Pose3，不可行）
- **不把 linefit 换成 ring 分割**（Mid360 非重复扫描无固定 scanline，polar bin 才是正确做法）
- **不回退 nav2 官方栈**（现有组件化免 pluginlib/lifecycle，官方栈反而更重）
- **不手写 banded box-QP 替代 OSQP**（风险高于收益）
- **不抽定位公共包**（跨包 ABI 耦合得不偿失）
- **不合并 GimbalPosture 请求/实测消息**（语义分离有充分理由）
