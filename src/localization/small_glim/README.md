# small_glim

LIO 里程计 + 建图包（GLIM 的因子图精简版）。吃 Mid360 的 `/livox/lidar/pointcloud`
和 `/livox/imu`，用 **GTSAM ISAM2 固定滞后平滑 + IMU 预积分 + GICP/iVox** 估计位姿，
输出 `/Odometry`、`/lio/robo/odom`（供 fast_location）、`/Laser_map`（世界系点云），
并广播动态 `lidar_odom→base_link` TF。

```text
/livox/lidar/pointcloud ─► preprocess ─► TimeKeeper ─► AsyncOdometryEstimation ─► /Odometry
/livox/imu ──────────────────────────────────────────►   (ISAM2 + GICP/iVox)      ├─► /lio/robo/odom
                                                               │                  └─► lidar_odom→base_link TF
                                                               ▼
                                                          AsyncMapping ─► /Laser_map
                                                              │
                                                              ▼  Ctrl-C 时
                                                          save_final_map() → <map_name>.pcd
```

## 架构

| 文件 | 职责 |
| :- | :- |
| `src/small_glim_node.cpp` | ROS 接线：订阅/发布、`pub_odometry`（双发 odom + TF）、`pub_cloud`（/Laser_map）、析构时落盘 |
| `src/preprocess/` | `cloud_deskewing`（逐点时间戳去畸变）、`cloud_covariance_estimation`、`cloud_preprocessor`（距离滤波+体素降采样+离群点剔除）、`time_keeper`（时间戳校验/伪时间戳） |
| `src/odometry/` | `initial_state_estimation`（重力水平+航向锚定初始化）、`imu_integration`（预积分）、`odometry_estimation`（ISAM2 因子图）、`async_odometry_estimation`（线程封装）、`estimation_frame` |
| `src/mapping/` | `async_mapping`：关键帧选取、增量合并、退出时 `save_final_map()` |
| `include/small_glim/common/` | 配置读取、日志、ROS 转换工具 |

## 核心机制

- **因子图**：每帧 LiDAR 观测做成 GICP 因子，IMU 做预积分因子连成时间链，`smoother_lag`
  秒的固定滞后窗交给 ISAM2 增量求解。200Hz IMU 被因子图**完全消费**（预积分到每个
  修正时刻），对外输出是修正帧率（约 10Hz）。
- **初始化**（`naive_initialization=true`）：取启动后 `initialization_window_size`(1.0s)
  的 IMU 加速度平均做重力水平，再用 IMU x 轴定航向（`align_initial_odom_to_imu`）。
  **上电后需静止约 1 秒**。
- **z 锚点**（本工作区补丁）：`initial_state_estimation.cpp` 把初始位置设为
  `p0 = -R0·t_imu_lidar`，odom 原点 z 锚在**开机雷达平面**——这是工作区契约
  （super_lio 同款约定，ground_segmentation/costmap 依赖），不是默认行为。
- **配准**：`registration_type=GICP`，目标帧插入 iVox（`ivox_resolution` 0.5），
  配准点取 `target_downsampling_rate`；支持 `ivox_update_delay`（延迟插帧防自碰撞
  "impact"）、`ivox_impact_pause_duration`（撞击后暂停更新）。
- **强度纯透传**：LIO 是纯几何（xyz + 协方差），`intensity_field` 只做提取，不参与
  匹配；发布 `/Laser_map` 时逐点带 intensity（无则填 0），避免下游 PCL 报警。

## 参数分层

五份 `config/params_*.yaml` 是全量默认，bringup 的
`config/{simulation,reality}/small_glim_{sim,real}.yaml` 只放环境差异项，launch 里
按 `默认 → 环境覆盖 → 节点字典` 顺序叠加（后面的覆盖前面的）。

所有参数**没有代码内默认值**（`Config::param` 即声明即取），缺任何一个键节点启动即崩
——这是刻意的 fail-fast；新增参数必须同步写进对应的 `params_*.yaml`。

| 文件 | 关键项 |
| :- | :- |
| `params_node.yaml` | 话题、frame、`acc_scale`、`enable_mapping`、`enable_tf_publish`、时间偏移 |
| `params_sensors.yaml` | IMU 噪声/偏置/饱和、`T_lidar_imu`（xyz+xyzw）、时间间隙上限 |
| `params_preprocess.yaml` | 距离滤波 `distance_near/far_thresh`、`downsample_resolution`、离群点剔除 |
| `params_odometry_estimation.yaml` | `smoother_lag`、`registration_type`、iVox、初始化、线程数 |
| `params_mapping.yaml` | `output_dir`/`map_name`、`map_voxel_leaf_size`、关键帧阈值、IMU 精化 |

### 关键参数速查

| 参数 | 默认 | 说明 |
| :- | :- | :- |
| `node.acc_scale` | 1.0 | IMU 加速度缩放。实车 9.7946（Mid360 内置 IMU 出 g 单位）；仿真 1.0（m/s²） |
| `node.enable_mapping` | true | 启动即建图，Ctrl-C 落盘。nav 模式由 launch 置 false |
| `node.enable_tf_publish` | false | 广播 `lidar_odom→base_link`（实车/仿真 launch 置 true） |
| `sensors.T_lidar_imu` | 包内默认 | 实车 `[0.011, 0.02329, -0.04412, 0,0,0,1]`（Livox 手册 IMU 位置交叉验证）；
  仿真 `[0, 0, -0.05, 0,0,0,1]`（URDF） |
| `preprocess.distance_near/far_thresh` | 0.3 / 30.0 | 雷达系距离过滤 |
| `preprocess.downsample_resolution` | 0.05 | 体素降采样（仿真 0.1，实车 0.05） |
| `odometry_estimation.smoother_lag` | 3.0 | 固定滞后窗（仿真 1.5，缩小修正滞后） |
| `odometry_estimation.target_downsampling_rate` | 0.1 | 配准目标体素率（仿真 0.05） |
| `mapping.output_dir` / `map_name` | "" / "" | 由 launch 传 `map_save_dir` / `<world>.pcd` |
| `mapping.map_voxel_leaf_size` | 0.05 | 落盘前体素合并（实车显式 0.05，避免旧 0.25 滤过粗） |
| `mapping.keyframe_trans_thresh` / `keyframe_rot_thresh` | 0.1m / 5° | 关键帧选取 |
| `mapping.cloud_range_min/max` | 0.8 / 20.0 | 建图侧距离过滤（imu_link 系） |

## 建图工作流

`mode:=mapping` 时 `enable_mapping=true`，从启动起累积关键帧；**必须 Ctrl-C 退出**
（触发节点析构 → `save_final_map()` 合并落盘），不是自动保存。

```sh
ros2 launch bringup sim.launch.py  world:=RMUL mode:=mapping nav_rviz:=True
# 跑一圈后 Ctrl-C
```

落盘位置 = `<map_save_dir>/<world>.pcd`（launch 传 `mapping.output_dir` 为绝对路径、
`mapping.map_name` 为 `<world>.pcd`；留空则退回 `~/mapping` + 时间戳子目录）。
输出是**纯 xyz 二进制 PCD**（`pcl::PointXYZ`），fast_location 直接可读，
`pcd_to_gridmap.py`（只读 xyz）转 pgm 也兼容。

> 注意：`--symlink-install` 下覆盖已存在的 world 会写穿到 `src/bringup/PCD/`；新建
> world 只落在 `install/`，下次 colcon build 会清掉，要自己拷回源码目录。

## 话题与 TF

| Topic | 方向 | 说明 |
| :- | :- | :- |
| `/livox/lidar/pointcloud` | 订阅 | 雷达点云（仿真插件无逐点时间戳 → 自动伪时间戳） |
| `/livox/imu` | 订阅 | IMU |
| `/Odometry` | 发布 | 里程计，child=base_link 实为雷达位姿（super_lio 约定），MPC 消费 |
| `/lio/robo/odom` | 发布 | 与 /Odometry 同内容，供 fast_location |
| `/Laser_map` | 发布 | 世界系点云，frame=`world`（odom→world 是静态恒等） |
| `/small_glim/ivox_cloud` | 发布 | 调试用 iVox 目标点云 |

TF：动态 `lidar_odom→base_link`（修正帧率，`enable_tf_publish` 时）；odom 原点 z 锚在
开机雷达平面。`odom→lidar_odom`、`odom→world` 由 bringup 的静态 TF 提供。

**修正帧率 = 点云率（约 10Hz）**：三个输出（TF/odom/Laser_map）同频同源。此前试过把
TF/odom 外推到 IMU 率，引发"方块抖"和"点云拉丝"，已回退；要保持 10Hz 一致。

## 依赖

- gcc-13/g++-13 + libstdc++-13-dev（C++23 硬编码，不可降级）
- gtsam_points v1.2.0（源码编译，`-DBUILD_WITH_MARCH_NATIVE=OFF`、不开 ASAN，ABI 旗标必须与编译一致）
- GTSAM 4.3a0（`/usr/local`）
- asio（`ros-humble-asio-cmake-module` + `libasio-dev`，这是 mid360_driver 的依赖）

安装步骤见根目录 `README.md` §3.1。
