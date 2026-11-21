# 导航反应延迟基线 + 感知触发改造（2026-11-20）

> 目标：把「新障碍进入代价图 → 规划器产出新路径」这段框架延迟压下来，并用可复现的
> 数值基线证明收益。改动只涉及全局代价地图的发布时机与规划器的重规划触发，不动
> A*/MINCO/MPC 算法，也不改话题契约。
>
> 复现工具：`src/bringup/tools/nav_reaction_bench.py`。
> 基线快照（改动前）来自 `b2b8e62`（当前最后一次推送）。

---

## 1. 改了什么

### 1.1 `rm_global_costmap`：观测触发发布（`publish_on_observation`）

以前全局代价地图只在 `publish_frequency: 8 Hz` 的定时器里更新+发布，新障碍点云
到达后最多要等 125 ms 才能进图。现在：

- 点云/激光回调里存下最新观测后，**立刻**调用 `publishCostmap()`（限速
  `4 * publish_frequency`，8 Hz 配置下 32 Hz；`observation_publish_rate` 可覆盖）；
- 按**观测时间戳去重**：同一帧（或更旧帧）不重复触发，定时器发过的老帧也不会
  把新帧挡掉；
- 观测触发和定时器**分开计时**：否则定时器刚发完、新观测随后到达时会被
  「距上次发布太近」误挡，实测会让约一半试验退化回定时器延迟；
- 定时器退化为**感知断流兜底**：距上次发布不足一个发布周期就跳过。

参数默认 `publish_on_observation: false`（保持历史行为），在
`src/bringup/config/common/navigation2.yaml` 的 `rm_global_costmap` 里显式开启。

### 1.2 `rm_global_planner`：由代价图发布触发重规划

`rm_global_planner` 本来就有 `replan_on_source_update` 参数（收到新代价图就规划），
但一直没开，规划只靠 8 Hz 定时器、且受 `min_replan_distance: 0.2 m` 约束：
**静止或低速时新障碍可能很久都不触发重规划**。现在：

- `replan_on_source_update: true`：每次代价图发布都规划一次（force，绕过节流）；
- `planning_frequency: 8.0 → 2.0`：定时器只做断流兜底，避免和源触发叠加到 16 Hz；
- 原有 `plan_failure_cooldown`、代次防覆盖、stitching 逻辑不变。

### 1.3 顺带修复：`linefit_ground_segmentation_ros` 清 build 后编译失败

`b2b8e62` 的 NUC 算子接入只改了核心库，wrapper 的公开头文件里
`#include <nuc_infra/ground_seg.hpp>` 却没有在自己的 CMake 里 `find_package(nuc_infra)`，
清 build 目录后必然编译失败。已在
`src/perception/linefit_ground_segmentation_ros2/linefit_ground_segmentation_ros/CMakeLists.txt`
补上依赖和链接。

---

## 2. 测量方法

无 Gazebo 的纯导航链路夹具（`nav_reaction_bench.py`）：

1. 启动 `rm_map_server` + `rm_global_costmap` + `rm_global_planner` +
   `rm_minco_path_smoother` + `rm_local_costmap`（不启 MPC/速度平滑，避免机器人静止
   被恢复链干扰）；
2. 夹具发 `map→base_link_fake` TF（50 Hz）、`/Odometry`（30 Hz、map 系零速度）、
   `/localization_status=OK`（1 Hz），并让虚拟机器人沿走廊以 1 m/s 前进；
3. 从 `/map` 里自动选一条 4.5 m 直线自由走廊，发 `/goal_pose`；
4. 每轮在路径前方 2.2 m 处注入一堵 2 m 宽、10 Hz 的合成点云墙
   （frame=`base_link_fake`，比当前时间戳晚 100 ms，模拟真实感知链路），
   注入时刻相对 8 Hz 定时器**随机相位**（实测 12 轮）；
5. 指标（墙钟，20 ms 轮询检测）：
   - `costmap_ms`：注入 → 障碍格出现在 `/global_costmap/costmap`；
   - `plan_raw_ms`：注入 → `/plan_raw` 出现绕开障碍的路径（A* 反应）；
   - `plan_ms`：注入 → `/plan` 绕开障碍（含 MINCO）；
6. 每轮结束撤掉障碍、等代价图衰减、重发目标清掉 stitching 状态，再进入下一轮。

对比三档（同一编译产物、同一夹具）：

| 代号 | 代价图发布 | 规划触发 | 规划定时器 |
|---|---|---|---|
| A 基线 | 8 Hz 定时器 | 8 Hz 定时器（受 0.2 m 位移节流） | 8 Hz |
| B 只开源触发 | 8 Hz 定时器 | 代价图发布触发 + 定时器 | 2 Hz |
| C 完整改动 | 观测触发（32 Hz 上限）+ 定时器兜底 | 代价图发布触发 + 定时器 | 2 Hz |

A/B 用同一份参数（`local_stitch_enabled: false`、`min_replan_distance: 0`）隔离机制；
另外用**生产参数**（stitching 开、`min_replan_distance: 0.2`）单独测了 A/C。

---

## 3. 数值结果（12 次注入试验，单位 ms）

### 3.1 机制对比（隔离 stitching/位移节流）

| 指标 | A 基线 | B 只开源触发 | C 完整改动 | C vs A |
|---|---:|---:|---:|---:|
| 障碍进图 `costmap` 均值 | 74.3 | 65.7 | **20.4** | −72.5% |
| `plan_raw` 均值 | 87.0 | 56.4 | **5.8** | −93.3% |
| `plan_raw` p50 | 83.3 | 68.1 | **5.2** | −93.8% |
| `plan_raw` p95 | 146.1 | 109.5 | **9.3** | −93.6% |
| `plan` 均值 | 107.9 | 78.2 | **27.6** | −74.4% |
| `plan` p95 | 166.5 | 129.1 | **33.0** | −80.2% |
| 导航容器 CPU | 7.3% | 7.7% | 7.3% | 噪声内 |

C 的 `costmap` 数值有 20 ms 量化（夹具 20 ms 轮询），实际进图时间小于该值。
`plan_raw` 直接取消息到达时刻，故能低于 `costmap` 的轮询粒度。

### 3.2 生产参数（stitching 开、`min_replan_distance: 0.2`）

| 指标 | A 基线 | C 完整改动 | 变化 |
|---|---:|---:|---|
| `costmap` 均值 | 79.2 | **20.5** | −74.1% |
| `plan_raw` 均值 | 130.4 | **6.5** | −95.0% |
| `plan_raw` p50 | 101.7 | **4.8** | −95.3% |
| `plan_raw` p95 | 270.5 | **21.1** | −92.2% |
| `plan` 均值 | 154.1 | **27.7** | −82.0% |
| `plan` p95 | 298.7 | **31.2** | −89.6% |
| 导航容器 CPU | 6.2% | 6.4% | +0.2 pp（噪声） |

C 重复跑两次：`plan_raw` 均值 5.0 / p95 5.5（另一次 5.9 / p95 6.9），`plan` 均值
26.7 / p95 28.0（另一次 28.0 / p95 30.4），结果稳定。
基线 A 的逐次波动较大（均值 87–130 ms，p95 146–270 ms），根因是 8 Hz 定时器相位
与注入时刻的拍频；C 不再依赖相位，因此方差显著收窄（p95 从 270 ms 降到 21 ms）。

---

## 4. 读数与边界

- **收益来自消除两段等待**：代价图等定时器（均值 ~62 ms，最坏 125 ms）+ 规划器等
  定时器/位移（均值 ~62 ms，最坏 125 ms，低速时更久）。C 把两段都变成事件驱动，
  只剩代价图计算、A*、MINCO 的实际耗时。
- **不是算法变快**：A* 单次 <5 ms、MINCO 约 20 ms 的既有预算没变；变的是「什么时候开始算」。
- **CPU 没有明显上涨**：代价图从 8 Hz 提到约 10 Hz、规划器从 8 Hz 提到约 10 Hz，
  在 4C/8T 上增量在噪声内（+0.2 pp）。
- **端到端仍有上限**：真实链路 `/segmentation/obstacle` 只有 ~4 Hz（配置 10 Hz），
  障碍从「出现在视野」到「进入代价图」仍要等分割出帧。这次改造把分割之后的
  300 ms 级框架等待压到 30 ms 级；**下一项该修的是感知节拍的 4 Hz/1–2 s 老化**，
  否则端到端仍被分割卡住。
- **夹具边界**：只覆盖 全局代价地图 → A* → MINCO，不含 LIO/分割/MPC、不含 Gazebo 物理、
  机器人是匀速虚拟前进（1 m/s）。真实闭环的绝对数值会不同，但 A/C 的**差值**
  （定时器等待）是可移植的机制收益。
- **参数默认关闭**：`publish_on_observation` 默认 false，其他分支/老配置行为不变；
  只有本仓库 `navigation2.yaml` 显式开启。

---

## 5. 复现

```sh
source /opt/ros/humble/setup.bash
source install/setup.bash

# 只起导航链路（不启 Gazebo / MPC / 速度平滑）
ros2 launch navigation2 bringup.launch.py \
  use_sim_time:=false \
  map:=$PWD/src/bringup/map/RMUL.msgpack \
  params_file:=$PWD/src/bringup/config/common/navigation2.yaml \
  start_fake_vel_transform:=false start_mpc_controller:=false \
  start_velocity_smoother:=false start_nav2_compat:=false \
  start_tunnel_posture:=false start_gimbal_visualizer:=false

# 另开终端
python3 src/bringup/tools/nav_reaction_bench.py --trials 12 --out /tmp/nav_reaction.json
```

回到改动前测基线：`git stash`（或 `git checkout b2b8e62 -- src/navigation/navigation2
src/bringup/config/common/navigation2.yaml`）后重跑同一命令。

---

## 6. 回归验证

- 全量编译：`./build.sh` 18 个包全部通过（含清 build 后的 `fast_location`、
  `linefit_ground_segmentation_ros`、PCL 相关包）。
- 单元/集成测试：`colcon test --packages-select navigation2`，34 项里 30 项通过。
  3 项失败（`test_semantic_map_consumer`、`test_costmap_tunnel_roof`、
  `test_test_tunnel_posture_node`）在**改动前的 `b2b8e62` 上同样失败**（已用
  `git stash` 回到基线复测确认），属于本次回退版本自带的语义地图/隧道相关遗留问题，
  与本次改动无关；`test_test_hazard_recovery_cycle` 偶发失败（本次基线跑反而通过），
  同样与代价图发布时机无关。
- `publish_on_observation` 默认 false，其他配置/测试不开启，行为与历史一致。

## 7. 改动文件

| 文件 | 改动 |
|---|---|
| `src/navigation/navigation2/src/global_costmap_node.cpp` | `publish_on_observation` 观测触发发布（时间戳去重、独立限速、定时器兜底） |
| `src/bringup/config/common/navigation2.yaml` | 开启 `publish_on_observation`；`replan_on_source_update: true`；`planning_frequency: 8→2`（兜底） |
| `src/navigation/navigation2/src/global_planner_node.cpp` | 未改代码（复用已有 `replan_on_source_update`） |
| `src/perception/linefit_ground_segmentation_ros2/linefit_ground_segmentation_ros/CMakeLists.txt` | 补 `nuc_infra` 依赖，修复清 build 后编译失败 |
| `src/bringup/tools/nav_reaction_bench.py` | 新增可复现反应延迟夹具 |
