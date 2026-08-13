# fast_location

点云对先验 PCD 的主定位。吃 small_glim 的 `/Laser_map_dense`（世界系定位稠密点云）和
`/lio/robo/odom`（10Hz 里程计），用 **FastGICP 多尺度级联**把当前扫描配准到先验
点云图，输出 **`map→odom` TF**。本工作空间里它是**唯一的 map→odom 来源**：
navigation2 代价地图、RViz 的定位都靠它把 odom 树和 map 树连起来。

```text
/Laser_map_dense ──┐               ┌──► map→odom TF (200Hz)
              ├─► FastGICP 级联 ──┤
/lio/robo/odom┘   3.0→2.0→1.5→1.0  └──► 退化/平滑后修正 map→odom
```

## 架构

| 文件 | 职责 |
| :- | :- |
| `src/robot_localization.cpp` | 主节点 `RobotLocalizationNode`：定位线程、GICP 级联、退化处理、TF 发布 |
| `src/global_search.cpp` | 全局重定位：平面包围盒网格粗搜 → 打分 → top-k → ICP 精化 |
| `include/fast_location/alignment_quality.hpp` | 配准质量评估：对齐点云 2D 分布条件数 → 退化检测 |
| `include/fast_location/global_search.hpp` | 全局搜索数据结构 |
| `include/fast_location/frame_transforms.hpp` | 位姿/点云坐标变换工具 |
| `test/` | `frame_transforms` / `global_search` 单元测试 |

点类型是 `pcl::PointXYZI`，但**只当容器用**：强度值从未参与匹配（`search_point.intensity`
是唯一写入，恒为 0）。先验 PCD 直接 `pcl::io::loadPCDFile<PointXYZI>` 加载，支持
small_glim 输出的纯 xyz PCD。

## 工作流程

`locationThread` 以 `localization_rate_hz`（实车 4Hz）定时触发：

1. **初始定位**（未 `initialized`）：`enable_global_search=true` 时走
   `performGlobalSearch()` 自动重定位，否则用 `globalLocalization(initial_pcd_to_odom)`
   从内置零位姿/`/initialpose` 起步。首次匹配内点比例须过 `first_localization_th`。
2. **跟踪**（已初始化）：仅当 `has_new_scan_`（新扫描到来）才重定位，pose_guess 用上次
   `map→odom`。跟踪阈值 `localization_th`。
3. **GICP 级联**（`runICP`）：同一帧按体素尺度 3.0→2.0→1.5→1.0 逐级配准，粗到细。
   前两级用 `gicp_max_iterations_first`，后两级用 `gicp_max_iterations_track`。
   下采样结果按尺度缓存（`scan_downsample_cache_`）避免重复计算；`scan_accumulate_frames`
   可堆积多帧再匹配，稀疏场景更稳。
4. **退化处理**：对齐点云 2D 分布条件数超 `degenerate_condition_threshold`（软阈值）
   时，**不整拍丢弃**，把修正投影到可观测方向——垂直走廊那维照常修正，沿走廊方向
   交回 LIO 推算；超 `degenerate_hard_reject_condition`（硬阈值，连 yaw 都不可信）才
   整拍拒绝、沿用上一次 map→odom。这是为了避免"整拍拒绝 → map→odom 冻结 → 精度
   完全交给 LIO 开环漂移"。
5. **EMA 平滑**：每次接受新结果 `T_smooth = ema_alpha·T_new + (1-ema_alpha)·T_prev`，
   0.7 ≈ 3 拍收敛；1.0 则直接接受。

## 全局重定位（global_search）

启用 `enable_global_search`。触发时机：启动首次定位、或跟踪连续失败
`tracking_failures_before_global_search`（5）次后自动重搜。

流程：平面包围盒内按 `xy_step`(2.0m)/`yaw_step`(30°) 布候选 → 逐候选 ICP 打分
（`score_distance`/`score_stride` 控严格度）→ 取 `top_k`(6) → 对候选按
`candidate_separation` 去重 → `refine_radius` 邻域内用
`refine_accumulate_frames` 帧堆积做精细 ICP（更严的 `refine_score_distance`）→
可选 `enable_temporal_verification` 用下一帧扫描做一致性校验（对称场地建议开）。

## 话题

| Topic | 方向 | 类型 | 说明 |
| :- | :- | :- | :- |
| `/Laser_map_dense` | 订阅 | `PointCloud2` | small_glim 世界系定位稠密点云（launch 把 `sub_scan_topic` 覆盖到这条） |
| `/lio/robo/odom` | 订阅 | `Odometry` | small_glim 里程计，提供位姿初猜与帧对齐 |
| `initialpose_3d` / `/initialpose` | 订阅 | `PoseStamped` / `PoseWithCovarianceStamped` | 手动/外部播种初始位姿 |
| `pc_in_map` | 发布 | `PointCloud2` | 匹配到 map 后的当前扫描 |
| `global_map` | 发布 | `PointCloud2` | 加载的先验图（`map_publish_rate_hz` 0.2Hz 实车） |
| `submap` | 发布 | `PointCloud2` | 局部子图（配准目标） |
| `map_to_odometry` | 发布 | `Odometry` | map→odom 里程计（`publish_map_to_odometry=false` 默认不发） |
| `scan_downsampled` / `map_downsampled` | 发布 | `PointCloud2` | 调试可视化 |
| `map→odom` | 发布 | TF | `tf_publish_rate_hz` 200Hz，在 odom 更新间插值 |

## 参数

基线在 `config/fast_location.yaml`（包内默认）与
`config/fast_location_main.yaml`（bringup 加载这份）。launch 里再按环境覆盖。

**地图与扫描**

| 参数 | 默认 | 说明 |
| :- | :- | :- |
| `map_pcd_path` | `package://bringup/PCD/RMUL.pcd` | 先验图，launch 按 `world` 拼 `<world>.pcd` |
| `map_voxel_size` / `scan_voxel_size` | 0.1 / 0.05 | 图/扫描体素滤波（实车 0.20） |
| `scan_input_frame_mode` | `odom` | 扫描所在坐标系：`odom`/`base` |
| `scan_accumulate_frames` | 5 | 堆积帧数 |
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
| `localization_rate_hz` | 5.0（实车/仿真 4.0） | GICP 触发上限 |
| `tf_publish_rate_hz` | 200.0 | map→odom TF 发布率 |
| `map_publish_rate_hz` | 5.0（实车 0.2） | 全局图发布率 |

**GICP**

| 参数 | 默认 | 说明 |
| :- | :- | :- |
| `use_fast_gicp` | true | fast_gicp 实现 |
| `gicp_num_threads` | 4（实车 2） | 配准并行线程，0=自动 |
| `gicp_max_iterations_first` / `gicp_max_iterations_track` | 100 / 70 | 首配/跟踪迭代上限 |

**全局搜索**：`enable_global_search`、`global_search_xy_step`、`global_search_yaw_step_deg`、
`global_search_score_distance`、`global_search_score_stride`、`global_search_top_k`、
`global_search_refine_radius`、`global_search_refine_accumulate_frames`、
`global_search_enable_temporal_verification` 等，见 `config/fast_location_main.yaml`
内注释。

## launch 接线

`bringup/launch/{real,sim}.launch.py` 里 `mode:=nav` 才启动，参数顺序为
`fast_location_main.yaml` → 覆盖字典：

```text
sub_scan_topic: /Laser_map_dense
map_pcd_path:   package://bringup/PCD/<world>.pcd
scan_voxel_size: 0.20          # 体素放粗、线程收 2，牺牲精度换实时性
submap_voxel_size_first/track: 0.20 / 0.35
fov_far: 12.0, localization_rate_hz: 4.0, gicp_num_threads: 2, map_publish_rate_hz: 0.2
```

## 契约与约束

- **map→odom 唯一来源**：`publish_tf=true` 时发布 `map`(父)→`odom`(子)。代价地图
  要的是 `map→odom→lidar_odom→base_link` 全链可达；fast_location 只管 map→odom，
  odom→lidar_odom 由静态 TF、lidar_odom→base_link 由 small_glim 提供。
- **先验 PCD 必须与 small_glim 建图输出同坐标系**（odom 原点 z 锚在开机雷达平面）。
  换了图（`world` 参数）PCD 路径跟着换。
- **退化时不是丢结果，是投影**：这是本工作区刻意的取舍，别改成整拍拒绝。

## 设计取舍

- **退化时不是丢结果,是投影**：软阈值命中把修正投影到可观测方向,硬阈值才整拍拒绝。
  本工作区只有 fast_location 发 map→odom,整拍拒绝会让 TF 冻结、精度退回 LIO 开环漂移。
- **每帧都跑 GICP 而非运动门控**：输入变化时 GICP 首轮即收敛,静止成本低;间断修正
  会变成低频大跳变,不如每帧高频小修正稳定。
