# fast_location

点云对先验 PCD 的主定位。吃 small_glim 的 `/small_glim/deskewed_cloud`（scan-start 去畸变、odom 系权威云）和
`/lio/robo/odom`（10Hz 里程计），用 **FastGICP 多尺度级联**把当前扫描配准到先验
点云图，输出 **`map→odom` TF**。本工作空间里它是**唯一的 map→odom 来源**：
navigation2 代价地图、RViz 的定位都靠它把 odom 树和 map 树连起来。

```text
/small_glim/deskewed_cloud ──┐       ┌──► map→odom TF (50Hz timer, EMA only)
              ├─► FastGICP 级联 ──┤
/lio/robo/odom┘   3.0→2.0→1.5→1.0  └──► 退化/平滑后修正 map→odom
```

## 架构

| 文件 | 职责 |
| :- | :- |
| `src/robot_localization.cpp` | 主节点 `RobotLocalizationNode`：定位线程、GICP 级联、退化处理、TF 发布 |
| `src/global_search.cpp` | 全局重定位：平面包围盒网格粗搜 → 打分 → top-k → ICP 精化 |
| `include/fast_location/alignment_quality.hpp` | 配准质量：条件数退化投影、scan-to-map NIS、对外 `IntegrityState`（OK/LOST） |
| `include/fast_location/global_search.hpp` | 全局搜索数据结构 |
| `include/fast_location/frame_transforms.hpp` | 位姿/点云坐标变换工具 |
| `test/` | `frame_transforms` / `global_search` 单元测试 |

点类型是 `pcl::PointXYZI`，但**只当容器用**：强度值从未参与匹配（`search_point.intensity`
是唯一写入，恒为 0）。先验 PCD 直接 `pcl::io::loadPCDFile<PointXYZI>` 加载，支持
small_glim 输出的纯 xyz PCD。

## 工作流程

`locationThread` 以 `localization_rate_hz`（当前实车/仿真 10Hz）定时触发：

1. **初始定位**（未 `initialized`）：开机走出生点 `globalLocalization(initial_pcd_to_odom_)`
   （保证在家，不跑全场）。`LOST` 后若开了搜索，仍全场网格；接受时 `map→odom` XY
   相对上一拍不能超过 `global_search_max_map_odom_jump`（默认 4m）。RMUL 对面角约 11m，
   跳过去会被丢掉。`/initialpose` 把 generation 作废：相对上一拍 ≤4m 才当先验做局部 ICP；
   跳过大（点到对面角）立刻 `LOST` 全场召回，且不把错误种子写进 `T_pcd_to_odom_`。
   关掉搜索时 `LOST` 也用上一拍做多尺度 ICP。首次匹配须过 `first_localization_th`。
2. **跟踪**（已初始化）：仅当 `has_new_scan_`（新扫描到来）才重定位，pose_guess 用上次
   `map→odom`。跟踪阈值 `localization_th`。
3. **GICP 级联**（`runICP`）：同一帧按体素尺度 3.0→2.0→1.5→1.0 逐级配准，粗到细。
   前两级用 `gicp_max_iterations_first`，后两级用 `gicp_max_iterations_track`。
   下采样按尺度缓存；体素滤波在 `data_mutex_` 外做，哈希表只短锁 lookup/insert，
   全场搜索时 SubScan 仍能进缓冲。`scan_accumulate_frames` 可堆积多帧。
4. **退化处理**：对齐点云 2D 分布条件数超 `degenerate_condition_threshold`（软阈值）
   时，**不整拍丢弃**，把修正投影到可观测方向——垂直走廊那维照常修正，沿走廊方向
   交回 LIO 推算；超 `degenerate_hard_reject_condition`（硬阈值，连 yaw 都不可信）才
   整拍拒绝、沿用上一次 map→odom。这是为了避免"整拍拒绝 → map→odom 冻结 → 精度
   完全交给 LIO 开环漂移"。
5. **EMA 平滑**：每次接受新结果 `T_smooth = ema_alpha·T_new + (1-ema_alpha)·T_prev`，
   0.7 ≈ 3 拍收敛；1.0 则直接接受。
6. **完整性写门**（节点内部，不广播成中间态）：过几何质量后算 scan-to-map NIS；走廊投影仍写 TF。
   拒写握住上一拍。连败 / `map→odom` 跳变过大 / 远处 `/initialpose` 才 `enterLost`。
   对外只发 `OK <reason>` 或 `LOST <reason>`，见「完整性契约」。

## 全局重定位（global_search）

运行配置 `enable_global_search: true`。开机不搜（在家，局部 ICP）。`LOST` 后走
全场 `performGlobalSearch()`；精化与 `acceptLocalizationResult` 都会丢掉
`map→odom` XY 跳变超过 `global_search_max_map_odom_jump`(4.0m) 的候选。
RMUL 对面角约 11m，锁过去会被拒绝；只有远角胜出来时停在 `LOST`。
跳变门配 0 关闭。

流程：AABB × 360° yaw 按 `xy_step`(2.0m)/`yaw_step`(30°) 布候选 → KD-tree 命中率打分
→ 取 `top_k`(6) 并按 `candidate_separation` 去重 → 分差小于
`global_search_min_score_margin`(0.03) 则整次拒绝 →
`refine_radius` 邻域精细 ICP（跳变过门的丢掉） → `enable_temporal_verification` 用下一帧再打分，
过 `temporal_min_score` 才写入 `map→odom`。

候选数硬上限 `max_candidates`，超了自动加大 `xy_step`，避免一次重定位 OOM。
粗搜在定位线程串行打分，精化仍用 `gicp_num_threads=2`，不加线程。

## 话题

| Topic | 方向 | 类型 | 说明 |
| :- | :- | :- | :- |
| `/small_glim/deskewed_cloud` | 订阅 | `PointCloud2` | small_glim 唯一权威的 scan-start 去畸变 odom 云；fast_location 与实时感知共同消费 |
| `/lio/robo/odom` | 订阅 | `Odometry` | small_glim 里程计，提供位姿初猜与帧对齐 |
| `initialpose_3d` / `/initialpose` | 订阅 | `PoseStamped` / `PoseWithCovarianceStamped` | 手动/外部播种初始位姿 |
| `pc_in_map` | 发布 | `PointCloud2` | 匹配到 map 后的当前扫描 |
| `global_map` | 发布 | `PointCloud2` | 加载的先验图（`map_publish_rate_hz` 0.2Hz 实车） |
| `submap` | 发布 | `PointCloud2` | 局部子图（配准目标） |
| `map_to_odometry` | 发布 | `Odometry` | map→odom 里程计（`publish_map_to_odometry=false` 默认不发） |
| `scan_downsampled` / `map_downsampled` | 发布 | `PointCloud2` | 调试可视化 |
| `map→odom` | 发布 | TF | `tf_publish_rate_hz` 50Hz，tf2 自行插值 |
| `localization_status` | 发布 | `String` | 二态 `OK/LOST` + reason，reliable + transient_local |

## 完整性契约

`localization_status` payload 是 `"<STATE> <reason>"`。**对外只有 `OK` / `LOST`。**
NIS、走廊投影、跳变门、连败召回都在定位内部做完再贴 reason，不广播四态。
下游只解析第一段：仅 `LOST` 停车/冻目标；`OK` 不停；从没收到过不拦。

| payload | 定位内部 | 下游（MPC / 决策） |
| :- | :- | :- |
| `OK accepted` | 本拍写了 `map→odom` | 走 |
| `OK projected` | 走廊退化：投影后仍写 TF | 走 |
| `OK nis_reject` | 握住上一拍 TF；连败满 `nis_lost_streak`(8) 才进 LOST | 走 |
| `OK frontend_diverged` | Hessian 满但匹配差；连败满 `frontend_diverged_streak`(5) 才进 LOST | 走 |
| `OK weak_obs` | 弱观测，握住 TF，**不记** `TrackingRecovery` | 走 |
| `OK pose_prior` | 近处 `/initialpose` 当先验，下一拍局部 ICP | 走 |
| `LOST waiting` | 开机未锁 | 停车 + 冻下发 |
| `LOST nis_lost` | NIS 连败 | 停车 + 全场搜 |
| `LOST tracking_lost` | ICP 连败满 `tracking_failures_before_global_search`(5) | 停车 + 全场搜 |
| `LOST map_odom_jump` | 接受结果相对上一拍 XY 跳 > `global_search_max_map_odom_jump`(4m) | 停车 + 全场搜 |
| `LOST pose_prior_jump` | 远处 `/initialpose`（点到对面角），种子**不写**进 `T_pcd_to_odom_` | 停车 + 全场搜 |
| `LOST frontend_diverged` | 前端发散连败满 | 停车 + 全场搜 |

Humble 下 `transient_local` 与 intra-process 不能同时开，见下「契约与约束」。

## 参数

基线在 `config/fast_location.yaml`（包内默认）与
`config/fast_location_main.yaml`（bringup 加载这份）。launch 里再按环境覆盖。

**地图与扫描**

| 参数 | 默认 | 说明 |
| :- | :- | :- |
| `map_pcd_path` | `package://bringup/PCD/RMUL.pcd` | 先验图，launch 按 `world` 拼 `<world>.pcd` |
| `map_voxel_size` / `scan_voxel_size` | 0.1 / 0.05 | 图/扫描体素滤波（实车 0.20） |
| `scan_input_frame_mode` | `odom` | 扫描所在坐标系：`odom`/`base` |
| `scan_accumulate_frames` | 5 | 堆积帧数；仅 odom-frame 输入允许多帧，base 模式强制为 1 |
| `scan_odom_sync.enable` | true | 扫描按 header stamp 绑定最近的 `/lio/robo/odom`，定位 timer 不再读取任意 latest odom |
| `scan_odom_sync.max_delta` | 0.03 s | 最近 odom 超过该时间差则丢弃本扫描 |
| `scan_odom_sync.history_size` | 32 | 固定容量 odom 历史；不新增线程、不深拷贝点云 |
| `scan_min_range` / `scan_max_range` | 0.0 / 0.0 | 距离过滤（米，≤0 关闭）。近场点 <1m 多为
  近地/车身，远场 >100m 稀疏，都是坏对应；需要时配 `1.0`/`100.0` |
| `fov_far` | 20.0（实车 12.0） | 视场最远距离 |
| `localization_th` / `first_localization_th` | 0.90 / 0.95 | 跟踪/首配内点比例阈值 |

**退化与平滑**

| 参数 | 默认 | 说明 |
| :- | :- | :- |
| `degenerate_condition_threshold` | 80.0 | 软阈值，条件数超标 → 只修正可观测方向 |
| `degenerate_hard_reject_condition` | 5000.0 | 硬阈值，整拍拒绝 |
| `ema_alpha` | 0.7 | map→odom 平滑系数 |

**频率**

| 参数 | 默认 | 说明 |
| :- | :- | :- |
| `localization_rate_hz` | 10.0（实车/仿真同） | GICP 触发上限 |
| `tf_publish_rate_hz` | 50.0 | map→odom TF 发布率 |
| `map_publish_rate_hz` | 5.0（实车/仿真 0.2） | 全局图发布率 |

**GICP**

| 参数 | 默认 | 说明 |
| :- | :- | :- |
| `use_fast_gicp` | true | fast_gicp 实现 |
| `gicp_num_threads` | 2 | 8 核预算；0 回退到 2，不用 hardware_concurrency |
| `gicp_max_iterations_first` / `gicp_max_iterations_track` | 100 / 70 | 首配/跟踪迭代上限 |

**完整性 / NIS**

| 参数 | 默认 | 说明 |
| :- | :- | :- |
| `enable_nis` | true | 过几何质量后再做 scan-to-map 一致性门 |
| `nis_reject_threshold` | 12.0 | 3 自由度 χ² 约 99%；超则本拍不写 `map→odom` |
| `nis_lost_streak` | 8 | 连续 NIS 拒绝后进 `LOST`；全场搜，`map→odom` 跳变过大则拒绝 |
| `nis_prior_xy_std` / `nis_prior_yaw_std_deg` | 0.20 / 8.0 | 当前位姿先验，进创新协方差 |
| `frontend_diverged_streak` | 5 | Hessian 满但匹配差连续这么多拍才进 `LOST` |

**全局搜索**：`enable_global_search`、`global_search_max_map_odom_jump`、
`global_search_xy_step`、`global_search_yaw_step_deg`、
`global_search_score_distance`、`global_search_score_stride`、`global_search_top_k`、
`global_search_min_score_margin`、`global_search_refine_radius`、
`global_search_refine_accumulate_frames`、`global_search_enable_temporal_verification`
等，见 `config/fast_location_main.yaml` 内注释。

## launch 接线

`bringup/launch/{real,sim}.launch.py` 里 `mode:=nav` 才启动，参数顺序为
`fast_location_main.yaml` → 覆盖字典：

```text
sub_scan_topic: /small_glim/deskewed_cloud
map_pcd_path:   package://bringup/PCD/<world>.pcd
scan_voxel_size: 0.20          # 实车体素；仿真 launch 更细（0.10 / 0.20）
submap_voxel_size_first/track: 0.20 / 0.35
fov_far: 12.0, localization_rate_hz: 10.0, gicp_num_threads: 2, map_publish_rate_hz: 0.2
```

## 契约与约束

- **map→odom 唯一来源**：`publish_tf=true` 时由 **TF 定时器**发 `map`(父)→`odom`(子)，
  值是 EMA 后的 `T_pcd_to_odom_`。定位回调不再另发未平滑 TF。代价地图要的是
  `map→odom→base_link` 全链可达；odom→base_link 由 small_glim 广播。首次定位成功后，
  LOST/全局召回期间继续以当前时间广播最后接受的 map→odom，防止唯一 TF 边从缓存老化
  消失；这不表示定位恢复，`localization_status` 仍保持 LOST，下游完整性门必须停车。
- **扫描下采样缓存按源代次隔离**：体素滤波仍在锁外执行；若期间 SubScan 换了扫描，
  旧计算结果只供当前调用使用，不回写新扫描的共享 cache，避免迟到结果污染下一帧。
- **先验 PCD 必须与 small_glim 建图输出同坐标系**（odom 原点 z 锚在开机雷达平面）。
  换了图（`world` 参数）PCD 路径跟着换。
- **退化时不是丢结果，是投影**：这是本工作区刻意的取舍，别改成整拍拒绝。
- **Humble 不能开 intra-process**：`localization_status` 是 transient_local，Humble
  的 intra-process 只接受 volatile。`main()` 里 `use_intra_process_comms(false)`，跟
  导航容器同一条纪律。节点是独立进程，intra-process 本来也没收益。
- **Hessian 管信息、NIS 管一致性**：条件数只说明观测有没有约束；过了几何质量后再算
  scan-to-map NIS。对外二态与 reason 见上「完整性契约」。不重置 small_glim、不加线程。
  `LOST` 后走全局搜索（margin 拒歧义，跳变门拒对面角，时序校验第二帧才接受）。
  `/initialpose` 自增 `reloc_generation_`。近处先验走局部 ICP；跳 > 4m 则 `LOST pose_prior_jump`，
  不把错误种子写进 `map→odom`。不要把 `OK nis_reject` 当停车。

## 设计取舍

- **退化时不是丢结果,是投影**：软阈值命中把修正投影到可观测方向,硬阈值才整拍拒绝。
  本工作区只有 fast_location 发 map→odom,整拍拒绝会让 TF 冻结、精度退回 LIO 开环漂移。
- **每帧都跑 GICP 而非运动门控**：输入变化时 GICP 首轮即收敛,静止成本低;间断修正
  会变成低频大跳变,不如每帧高频小修正稳定。
- **对外不广播中间态**：写门留在定位内部。下游只要「当前锁还能不能信」；`OBS_DEGRADED` / `SUSPECT` / `NORMAL`
  没有消费者按名字做事，已撤。也不要改成「只盯 `map→odom` 跳变」——跳变门看不到走廊投影、NIS 拒写、已锁错角。
