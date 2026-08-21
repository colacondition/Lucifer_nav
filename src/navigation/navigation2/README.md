# navigation2

`navigation2` 是泓龙哨兵导航工程的 ROS 2 导航系统。系统采用地图服务、全局规划、代价地图、路径平滑、MPC 局部控制、速度平滑和 RViz 兼容 action 的模块化链路，提供从目标点接收到速度指令输出的完整导航能力。

各节点均注册为 ROS 2 Component，默认组合到一个固定线程数容器中运行。Humble 下 latch 话题不能开 intra-process。系统借鉴 Nav2 的链路划分和常用话题接口，但不依赖完整 `nav2_bringup`、BT Navigator 或 lifecycle manager。



导航组件默认运行在同一个组件容器中，同时提供独立可执行文件，便于单独调试。

## 数据流图

![navigation2 数据流图](../../../docs/images/navigation2_data_flow.png)

## 软件架构

![navigation2 软件架构](../../../docs/images/navigation2_software_architecture.png)

## 节点

- `rm_map_server_node`：读取 `map/<world>.msgpack` 语义地图并发布 `/map`（占据栅格由 `terrain` 通道的 OBSTACLE 格推导；隧道等「能站但要摆姿态才能进」的先验在另一张语义通道里，不参与 /map 占据判定）。
- `rm_global_planner_node`：订阅 `/goal_pose` 和 `/map`，生成 `/plan_raw`。含规划失败冷却、路径发布前验收和规划代次校验。
- `rm_global_costmap_node`：订阅 `/map` 和 `/segmentation/obstacle`，发布全局代价地图。
- `rm_minco_path_smoother_node`：用 MINCO L-BFGS 平滑 `/plan_raw` 并发布 `/plan`，详见 `MINCO_README.md`。
- `rm_local_costmap_node`：基于 `/segmentation/obstacle` 点云构建滚动局部代价地图。
- `rm_mpc_controller_node`：使用 OSQP 跟踪路径，发布 `/cmd_vel_nav` 和 `/predict_path`。内置接近减速、多假设弧长进度跟踪、无进展/卡住检测、恢复链 FSM（倒车/安全点脱困）、弧长域速度剖面和指令链路闭环反馈。
- `rm_velocity_smoother_node`：将 `/cmd_vel_nav` 限幅平滑为 `/cmd_vel`。
- `fake_vel_transform`：同容器内将 `/cmd_vel` 转为底盘执行话题 `/cmd_vel_chassis`。
- `rm_nav2_compat_node`：提供 `/navigate_to_pose` action，把 RViz Nav2 Goal 转成 `/goal_pose`。
- `rm_tunnel_posture_node`：订阅语义地图与车体位姿，判断车是否接近/正在隧道内，向电控发布 `/gimbal_posture` 收云台请求（距最近隧道本体格 ≤ `run_up` 发收、洞里全程保持、退开 `run_up + hysteresis` 才发抬）。判据只看距离，不读车体参数；收多低、到位没到位全归电控。
- `rm_gimbal_visualizer_node`：订阅电控持续回传的 `/gimbal_posture_state` 和请求 `/gimbal_posture`，发布 `/gimbal_status` MarkerArray（实车/仿真通用；RViz 显示方块+文字：绿=收下/低，红=立着/高）。

运行时节点和话题采用 `rm_*` 命名，以匹配泓龙哨兵工程中的航点执行器、控制链路和 RViz 配置。

## 通过总 Bringup 启动

仿真：

```sh
source install/setup.bash
ros2 launch bringup sim.launch.py \
  world:=RMUL \
  mode:=nav \
  nav_rviz:=True
```

实车：

```sh
source install/setup.bash
ros2 launch bringup real.launch.py \
  world:=RMUL \
  mode:=nav
```

导航后端由全局规划、全局/局部代价地图、路径平滑、MPC 控制器、速度平滑器和 RViz 兼容 action 组成。

全局规划器使用 8 邻域 A* 搜索。规划前会基于输入的 `OccupancyGrid` 构建一份 2D 距离场，用米制距离描述每个栅格到最近障碍物的距离；A* 扩展节点时会根据 `clearance_desired_distance` 和 `clearance_cost_weight` 对贴近障碍的候选格子增加代价，让 `/plan_raw` 更倾向从通道中间通过。该距离场是规划时的轻量计算结果，不作为 ROS 节点或话题发布。

重规划采用分层策略：机器人移动距离较小时复发上一条路径；需要重规划时，优先尝试从当前位姿 A* 搜索到上一条路径前方的接入点，并拼接仍然可通行的旧路径尾段；局部拼接失败或旧路径尾段已被当前全局代价地图判定不可通行时，再执行完整 A* 到目标点。

规划器还有三层鲁棒性保障：

- **规划失败冷却**：同一目标规划失败后进入冷却期，避免对不可达目标以 `planning_frequency` 空转重试、洪水刷日志。目标明显移动或收到明确重规划请求时忽略冷却。
- **路径发布前验收**：沿规划结果逐姿态采样代价地图，拒绝穿越代价 ≥ `path_acceptance_max_cost` 的格子。
- **规划代次校验**：目标变化或明确重规划时递增代次，A* 完成发布前对比代次，旧地图快照的晚到结果直接丢弃，防止覆盖新目标。

相关参数位于 `rm_global_planner`：

```yaml
use_clearance_cost: true
clearance_desired_distance: 0.6
clearance_cost_weight: 3.0
local_stitch_enabled: true
stitch_min_lookahead_distance: 0.8
stitch_max_distance: 3.0
plan_failure_cooldown: 2.0
path_acceptance_enabled: true
path_acceptance_max_cost: 85
```

MPC 使用 `/local_costmap/costmap` 对求解后的预测运动逐段做碰撞检查。地图缺失、过期、越界或预测路径命中障碍时，控制器发布零线速度，并通过 `/navigation2/replan_request` 通知全局规划器立即重规划。相关参数位于 `rm_mpc_controller.local_safety.*`，求解和路径参考代码位于 `src/mpc`。

### 执行层：进度跟踪与恢复链

`rm_mpc_controller` 内置三个相互配合的执行层模块：

- **多假设弧长进度跟踪（`src/mpc/route_tracker`）**：把进度建模为有向路径上的时序状态 `(s, ṡ)`，维护多个竞争的弧长假设并按位置残差 + 速度方向一致性加权、淘汰弱分支。对外报告进度单调不减，回绕路径上不会在两条支路间跳变。速度由内部对连续位置做有限差分 + EMA 估计，不依赖外部里程计 twist 的坐标系约定。
- **失效检测（`src/mpc/progress_monitor`）**：`noProgress` 只看世界系位移不增长（捕捉"MPC 自己一直在停车"的循环）；`stuck` 要求有指令但位移不增长（顶住障碍、打滑）。判据刻意用世界系位移而非弧长里程碑——规划器每移动 `min_replan_distance` 就重规划一次，弧长原点随之重置，弧长计时器永远累计不到阈值。
- **恢复链 FSM**：卡住或持续否决时先物理脱困再重规划，避免「停车 → 重规划 → 起点不可行 → 继续停车」死循环。状态为 `FOLLOW → STUCK_REVERSE → HAZARD_RECOVERY → FAILED`。倒车路径与安全点采样只做 lethal 检查（车已在膨胀圈内，用正常阈值会让所有方向立即被否）。`recovery.enable: false` 可一键退回仅检测不动作。
- **跟踪丢失 / 求解失败 / 安全否决** 三条路径统一走 `handleVeto()`：短时否决停车 + 重规划（快路径），连续累计超 `veto_recovery_time` 后升级进恢复（慢路径）。

### 弧长域速度剖面

`src/mpc/speed_profile` 按路径曲率限侧向加速度，再用前/后向扫描保证切向加减速可达，替代原先的常数 `expected_speed`。MPC 参考窗口按弧长采样并优先使用剖面速度，弯道自动减速、接近目标时减速至零。`speed_profile.enable: false` 可退回常数速度。

### 指令链路闭环

本节点发布的指令仍会被 `rm_velocity_smoother` 限幅或超时归零，所以"是否真的有指令"以链路末端 `/cmd_vel` 的实测值为准（`feedback.executed_cmd_topic`），并据此检测下游越权与链路静默。接近减速已并进本节点；目标附近用 `recovery.suppress_near_goal` 关闭失效检测。

## 单独启动

```sh
source install/setup.bash
ros2 launch navigation2 bringup.launch.py \
  use_sim_time:=True \
  map:=/home/cola/Lucifer_nav/src/bringup/map/RMUL.msgpack \
  params_file:=/home/cola/Lucifer_nav/src/navigation/navigation2/params/navigation2.yaml
```


```sh
ros2 launch navigation2 bringup.launch.py start_map_server:=False
```

队内主调参文件（唯一真源；bringup 的 launch 也显式加载这一份）：

```text
src/navigation/navigation2/params/navigation2.yaml
```

## 测试

- **单元测试（gtest）**：`RouteTracker` / `ProgressMonitor` / `RecoveryPlanner` / `SpeedProfile` / `LocalPathSafety` / `PathStitching` / `SemanticMap` / `TunnelPosture`，覆盖各模块的纯逻辑。
- **集成测试（launch_testing）**：起真节点、喂合成 `/plan`、`/Odometry`、`/local_costmap/costmap`、`/cmd_vel`，断言 FSM 行为。覆盖目标附近不误触发恢复、卡住时确实进恢复、覆写检测、veto 连击升级、`HAZARD_RECOVERY → FOLLOW` 完整闭环。测试文件位于 `test/`，共用夹具 `test/mpc_harness.py`。

运行全部测试：

```sh
colcon test --packages-select navigation2 --event-handlers console_direct+
```

## 参考与致谢

本包的模块划分、A* + MPC 导航链路、距离场思路参考了武汉科技大学崇实战队开源的 ROSE NAVIGATION，并结合泓龙哨兵工程的 `OccupancyGrid`、组件化节点和 MPC 控制链路实现。执行层的多假设弧长进度跟踪、无进展/卡住检测、倒车与安全点脱困恢复链、定位质量门控（条件数检查 + EMA 平滑）借鉴了浙江大学 Hello World 战队开源的 HWSentryNav26 设计思路。

感谢武汉科技大学崇实战队对 ROSE NAVIGATION 的开源贡献，也感谢相关开源项目对 RoboMaster 地面机器人导航生态的支持。
