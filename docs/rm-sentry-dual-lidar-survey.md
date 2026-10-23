# RM 哨兵导航开源项目「双雷达 / 多雷达接入方式」调研报告

调研方法：对 14 个仓库逐个浅克隆（`git clone --depth 1`）+ GitHub REST API 列树 + `raw.githubusercontent.com` 拉取具体文件，全部结论来自**实际文件内容**，每条都给出「仓库 / 分支 / 文件路径」。
（GitHub 网络有间歇性 TLS 中断，部分仓库改用 API + raw 方式取得，不影响的读取的完整性。Gitee 两个仓库可直接克隆。）

---

## 0. 一句话结论

14 个仓库里**确认存在双雷达（或多传感器）接入的只有 5 个**：

| 仓库 | 雷达组合 | 接入范式的归类 |
|---|---|---|
| [HERO_2026_Sentry_NAV](https://github.com/LiuJinbo1027/HERO_2026_Sentry_NAV) | Odin1 + 2× Livox MID360（三传感器） | **主雷达(Odin1)自带 LIO；两路 MID360 先互相融合，融合后再与 Odin1 点云做"双传感器占据栅格加权融合"喂代价地图** |
| [navi_minco_bit](https://github.com/Walker152/navi_minco_bit) | 2× Livox MID360（前 135 / 后 122） | **先融合点云，再做 LIO**（融合下沉进 Livox 驱动内部，输出单帧给 Point-LIO） |
| [Front-end-Perception](https://github.com/Dongyang1525/Front-end-Perception) | 2× Livox MID360（主=右 151 / 副=左 159） | **先融合点云，再做 LIO**（改 FAST-LIO，用主雷达内置 IMU） |
| [qdu26_sentry_navigation](https://github.com/QDU-VRobot/qdu26_sentry_navigation) | Odin1 + Livox MID360 | **主雷达(Odin1)自带 LIO；副雷达点云与主雷达点云逐帧融合后只喂 terrain_analysis→代价地图** |
| [hnunavigation_-ros2](https://gitee.com/hnuyuelurm/hnunavigation_-ros2) | Livox MID360（3D）+ LDLiDAR LD06/LD19（2D） | **各走各的观测源，不融合**（本地代价地图两套独立 ObstacleLayer，全局代价地图 STVL 分 marking/clearing 两路） |

另有 [odin1_mid360_cloud_deskew](https://github.com/Cyan-chy/odin1_mid360_cloud_deskew)（Odin1 + 单 MID360）虽然不是"双雷达点云融合"，但它是目前找到的**唯一带完整 PTP 时间同步文档**的仓库，做 Odin1+MID360 组合的时序对齐非常值得抄。

**没有一个仓库采用"两个雷达各跑自己的 LIO 里程计再融合（如 robot_localization 双 odom 融合）"。** 双 MID360 的两个方案（navi_minco_bit、Front-end-Perception）都是"先融合点云、再单一 LIO"；Odin1 相关的两个方案都是"Odin1 自带 LIO 主定位 + MID360 只贡献点云给地图"。

---

## 1. 逐仓库结论表

| # | 仓库 | 雷达 | 双雷达？ | 结论 |
|---|---|---|---|---|
| 1 | [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav) | 单 Livox MID360（斜侧放） | ❌ | 未发现双雷达证据。全仓 grep `双雷达/dual lidar/lidar_2` 无命中 |
| 2 | [rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator) | 无（纯 Gazebo 世界/裁判系统） | ❌ | 89 个文件全是 world/model/裁判系统 Web，无雷达配置 |
| 3 | [pb_rm_simulation](https://github.com/LihanChen2004/pb_rm_simulation) | 单 MID360 | ❌ | pointlio/fastlio + pointcloud_to_laserscan，单雷达 |
| 4 | [HWSentryNav26](https://github.com/Polyacetone/HWSentryNav26) | 单 MID360（自研 `mid360_driver`） | ❌ | 全自研 `nav_executor` 代价地图，无 `observation_sources`。grep 命中 `kDualReturnMask` 是 **Livox 双回波 tag 掩码**，与双雷达无关 |
| 5 | [rose_navigation](https://github.com/hyheiyue/rose_navigation) | 单 MID360 | ❌ | 自研 LIO(`rose_navigation_lm_node`)+自研占据/ESDF，不用 Nav2 costmap |
| 6 | [odin1_mid360_cloud_deskew](https://github.com/Cyan-chy/odin1_mid360_cloud_deskew) | Odin1 + 单 MID360 | ⚠️（双传感器，非双雷达点云融合） | 用 Odin1 高频里程计给 MID360 逐点去畸变；**含 PTP 硬件对时全套文档** |
| 7 | [qdu26_sentry_navigation](https://github.com/QDU-VRobot/qdu26_sentry_navigation) | Odin1 + MID360 | ✅ | `odin1_integration/pointcloud_fusion_node` 融合 → `fused_cloud_slam` → terrain_analysis → Nav2 VoxelLayer |
| 8 | [HERO_2026_Sentry_NAV](https://github.com/LiuJinbo1027/HERO_2026_Sentry_NAV) | Odin1 + 2×MID360 | ✅ | `lidar_merge` 融合 → `hero_lidar_scan` 去畸变 → `dog_map` 双源 log-odds 融合 → 自研 `fast_layer` |
| 9 | [Front-end-Perception](https://github.com/Dongyang1525/Front-end-Perception) | 2×MID360 | ✅ | 改 FAST-LIO，`fuse_dual_lidar_frame_once()` 5ms 配对融合；**README 明确"使用 PTP 进行双雷达时间硬件同步"** |
| 10 | [navi_minco_bit](https://github.com/Walker152/navi_minco_bit) | 2×MID360 | ✅ | 融合下沉进 `livox_ros_driver2` 内部（`InternalLidarMerger`），输出 `livox/lidar` 给 Point-LIO |
| 11 | [sentry26](https://github.com/xiix666/sentry26) | 单 MID360 | ❌ | point_lio + terrain_analysis，自研 `voxel_obstacle`；驱动里带 stock `mixed_HAP_MID360_config.json` 但 bringup 只用单 MID360 |
| 12 | [cod_-rm2026_-navigation](https://gitee.com/codnavgation/cod_-rm2026_-navigation) | 单 MID360 | ❌（副源是 RealSense） | `observation_sources: livox_source realsense_source`，第二路是**深度相机**不是第二个雷达 |
| 13 | [hnunavigation_-ros2](https://gitee.com/hnuyuelurm/hnunavigation_-ros2) | MID360 + LD06/LD19 2D 雷达 | ✅（异构） | 两路各自进不同 layer；全局 STVL 分 `livox_mark`/`livox_clear` |
| 14 | [sentry_planning](https://github.com/zzt-180/sentry_planning) | 无（OCS2 MPC + 2024 Gazebo 仿真，Velodyne） | ❌ | 1779 个文件全是 OCS2/OCS2 第三方 + sentry_gazebo_2024，无 nav2/costmap 配置，未发现双雷达证据 |

---

## 2. 关键仓库细节

### 2.1 HERO_2026_Sentry_NAV（LiuJinbo1027）— 信息量最大

仓库 README 第 27 行直接写：`- **双激光雷达融合**（lidar_merge）：融合双雷达点云，扩大感知范围。`

**数据流（实测文件推得）**

```
2×MID360 ──► lidar_merge ──► /livox/fused_cloud (CustomMsg)
                                   │
Odin1 ──► /odin1/odometry_highfreq │
Odin1 ──► /odin1/cloud_slam ───────┤
                                   ▼
                        hero_lidar_scan（去畸变+CropBox自滤波）
                                   ▼
                          /livox/cloud_scans
                                   ▼
                     dog_map_node（双源 log-odds 占据融合）
                                   ▼
                     /rog_map/inf_occ → 自研 fast_layer → costmap
```

**(a) 融合节点**：`perception/lidar_merge/src/lidars.cpp`
- 用 `message_filters::sync_policies::ApproximateTime` 对两路 `livox_ros_driver2/msg/CustomMsg` 做**软件近似时间同步**（队列 5）：

```cpp
// perception/lidar_merge/src/lidars.cpp:66-72
#ifdef TWO_LIDARS
  lidar1_sub_.subscribe(this, lidar1_topic);
  lidar2_sub_.subscribe(this, lidar2_topic);
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(5), lidar1_sub_, lidar2_sub_);
```

- 融合不是简单拼接，而是**按每点 `offset_time` 重排**：以两帧中较新的 `timebase` 为基准，把另一帧的点偏移量平移到同一条时间轴上，再用双指针归并排序输出：

```cpp
// perception/lidar_merge/src/lidars.cpp:96-99, 153-164
  const bool is_msg1_latest = (msg1->timebase > msg2->timebase);
  fused_msg->header   = is_msg1_latest ? msg1->header : msg2->header;
  fused_msg->timebase = is_msg1_latest ? msg1->timebase : msg2->timebase;
  ...
    int64_t t1 = trans_p1[i].real_offset;
    int64_t t2 = diff2 + static_cast<int64_t>(p2_ptr[j].offset_time);
    if (t1 <= t2) { /* 输出 msg1 的点 */ } else { /* 输出 msg2 的点 */ }
```

- **自车/盲区自滤波**在同一文件 `isBlind()`：丢弃 Livox tag 不是 `0x10/0x00` 的盲区点，并丢弃距给定原点（`ori_x/ori_y/ori_z`，即车体中心）小于 `blind` 的点：

```cpp
// perception/lidar_merge/src/lidars.cpp:80-86
bool LidarMergeNode::isBlind(const livox_ros_driver2::msg::CustomPoint & pt) {
  if(((pt.tag&0x30)!=0x10)&&((pt.tag&0x30)!=0x00)) return true; // tag不为0的点直接丢弃，认为是盲区点
  double distance = std::sqrt((pt.x-ori_x)*(pt.x-ori_x) + (pt.y-ori_y)*(pt.y-ori_y) + (pt.z-ori_z)*(pt.z-ori_z));
  return distance < blind;
}
```

- 参数文件 `perception/lidar_merge/config/cfg.yaml`（真机两 MID360）与 `cfg_odin.yaml`（Odin1 版本，两雷达各自标到 Odin1 系）：

```yaml
# perception/lidar_merge/config/cfg_odin.yaml
lidar_merge_node:
  ros__parameters:
    lidar1_topic: "livox/lidar_192_168_1_193"
    lidar2_topic: "livox/lidar_192_168_1_126"
    lidar1_to_odin1_x: -0.186846      # 雷达1 -> Odin1 IMU 外参
    lidar1_to_odin1_pitch: 0.593192
    lidar2_to_odin1_x: -0.116295      # 雷达2 -> Odin1 IMU 外参
    lidar2_to_odin1_pitch: 0.622282
    ori_x: -0.1713                    # 自车中心（盲区球心）
    blind: 0.15                       # 盲区半径 0.15 m
```

> ⚠️ **重要坑（实际读码发现）**：`perception/lidar_merge/src/lidars.cpp` 的双雷达分支包在 `#ifdef TWO_LIDARS` 里，但 `perception/lidar_merge/CMakeLists.txt` 只写了 `add_compile_definitions(DEBUG)`，**没有定义 `TWO_LIDARS`**；编译的是 `src/lidars.cpp`（`merge.cpp` 根本没进构建）。同样 `hero_lidar_scan/CMakeLists.txt` 里也没有任何 `add_compile_definitions`。也就是说：**开源出来的默认构建是单雷达直通路径，要真正启用双雷达必须自己加 `-DTWO_LIDARS` 编译宏**。用这套代码时务必先确认这一点。

**(b) 去畸变 + 自车裁剪**：`perception/hero_lidar_scan/`
- `config/params.yaml`：输入是已融合的 `/livox/fused_cloud` + 一路原始雷达，位姿来自 Odin1：

```yaml
# perception/hero_lidar_scan/config/params.yaml
    lidar1_topic: "/livox/fused_cloud"     # 左雷达（已融合）
    lidar2_topic: "livox/lidar_192_168_1_126"  # 右雷达
    odom_topic: "/odin1/odometry_highfreq"
    fused_cloud_topic: "/livox/cloud_scans"
    time_offset: 0.0
    filter_size: 0.05
```

- 自车遮挡用 **PCL CropBox + `setNegative(true)`（挖掉车体盒）**，尺寸硬编码在头文件：

```cpp
// perception/hero_lidar_scan/include/hero_lidar_scan/hero_lidar_scan_node.hpp:30-35
const float crop_min_x = -0.6f;
const float crop_min_z = -0.3f;
const float crop_max_x =  0.6f;
const float crop_max_z =  1.0f;

// src/hero_lidar_scan_node.cpp:109-112
    crop_filter_.setMin(min_pt);
    crop_filter_.setMax(max_pt);
    crop_filter_.setNegative(true);   // 去除 box 内的点，保留外部点
```

- 去畸变用 Odin1 的高频里程计按点时间插值（worker 线程里 `getPoseAtTimeLocal` + `processLidar`），并且 `TWO_LIDARS` 分支下会把两路点云各自去畸变后相加：

```cpp
// perception/hero_lidar_scan/src/hero_lidar_scan_node.cpp:469-491
        double t_end = std::max(t1_end, t2_end);
        ...
        processLidar(lidar1_msg, T_lidar1_to_base_, pose_end, local_odom_buffer, cloud1);
        processLidar(lidar2_msg, T_lidar2_to_base_, pose_end, local_odom_buffer, cloud2);
        pcl::PointCloud<pcl::PointXYZI> fused_cloud = cloud1 + cloud2;
```

**(c) 两个雷达点云重叠区怎么处理 → 在占据栅格 log-odds 层面加权，不做点级去重**
`planning/dog_map/src/node.cpp` 为两个来源分别配了命中/上限阈值（`node.cpp:297-324`，`occ_map_->SetLOGParams(...)` 在 `:345`）：

```yaml
# planning/hero2025_nav_bringup/config/nav2_params.yaml:1-16
dog_map_node:
    ros__parameters:
      odin_topic: "/odin1/cloud_slam"
      cloud_topic: "/livox/cloud_scans"
      half_map_size: 10.0
      heighter_than_ground_threshold: 0.091
      higher_than_car_threshold: 0.7
      LOG_OCC_HIT_mid360: 34
      LOG_OCC_HIT_odin1: 16      # 两个传感器不同的命中权重
      LOG_OCC_FREE: -20
      THR_OCC: 30
      MAX_LOG_MID360: 34
      MAX_LOG_ODIN1: 34
```
即：**重叠区的两份点云各自往同一张占据图里打 log-odds，用不同权重解决"谁说了算"**，而不是先做体素去重。另外 `node.cpp` 里还有 `pcl::PassThrough` 的 z 限幅 `setFilterLimits(0.06, 1.0)`（`:136-139`）与 `StatisticalOutlierRemoval`。

**(d) 代价地图是"Nav2 costmap_2d 框架 + 自研 layer"，不用 observation_sources**

```yaml
# planning/hero2025_nav_bringup/config/nav2_params.yaml:128-141
      plugins: ["static_layer","fast_layer", "inflation_layer"]
      fast_layer:
        plugin: fast_layer::FastLayer
        pointcloud_topic: /rog_map/inf_occ
      static_layer:
        plugin: "update_static_layer::UpdateStaticLayer"
        cloud_update_topic: /rog_map/fi
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.40
```
副雷达点云最终以 **`/rog_map/inf_occ`（PointCloud2）** 这个自定义话题进入代价地图，坐标系由 `dog_map` 决定，**完全绕开了 `observation_sources` 的 marking/clearing 机制** —— 这也是它回避"多源互相 clearing 打洞"的根本手段。

---

### 2.2 navi_minco_bit（Walker152）— 唯一把融合下沉到驱动内部的方案

- 关键文档：`src/perception/livox_ros_driver2/INTERNAL_LIDAR_MERGE.md`（标题：《Livox ROS2 Driver 内部双 MID360 融合说明》）
- 前后两台 MID360：前 `192.168.1.135`，后 `192.168.1.122`；输出仍用 **`livox/lidar`**，`frame_id = livox_frame`，**Point-LIO 完全不用改配置**。
- **融合策略：按帧 `base_time` 配对，窗口 5ms；超窗丢旧帧、绝不发半帧**（`INTERNAL_LIDAR_MERGE.md:52-61`）：

```
1. 按 IP handle 找到 front/back 两个 LidarDataQueue
2. 使用 QueuePeek() 查看两个队头包，避免融合前深拷贝
3. 比较两帧 base_time
4. 时间差小于等于 merge_max_interval_ms 时合并发布
5. 时间差超限时丢弃更旧的一帧，并节流打印 warning
6. 发布成功后用 QueuePopUpdate() 同时弹出两帧
```

- 参数（`src/perception/livox_ros_driver2/config/mixed_MID360_component.yaml`）：

```yaml
    xfer_format: 1          # 1 = CustomMsg（Point-LIO 用这条）
    multi_topic: 1
    frame_id: livox_frame
    enable_internal_lidar_merge: true
    merge_front_ip: 192.168.1.135
    merge_back_ip: 192.168.1.122
    merge_output_topic: livox/lidar
    merge_frame_id: livox_frame
    merge_max_interval_ms: 5.0
    merge_extrinsic_back_to_front: [0.0, 0.38, -0.05, -0.36713, 0.0, 0.0]  # x,y,z,roll,pitch,yaw
```

- 输出消息字段（同文档）：`header.stamp = min(front.base_time, back.base_time)`、`timebase = min(...)`、**每点 `offset_time` 重算为相对合并帧 timebase 的偏移**、后雷达点先做 back→front 变换再写入。**重叠区没有任何去重/裁剪，就是直接拼接**。
- **时间同步要求（第 21 行）**：`Keep PTP/GPS time synchronization enabled.`；标定配置里 `sync_tolerance_ms: 5.0`。
- 配套离线标定包：`src/perception/dual_lidar_calibration/`
  - README：`Offline secondary-to-main extrinsic calibration for two synchronized Livox LiDARs and their sensor_msgs/msg/Imu streams`；用 **IMU 向量对齐 + 旋转去畸变 + 多帧 GICP + 离群帧剔除 + SE(3) 聚合**，输出 `merge_extrinsic_back_to_front` 直接喂给驱动内部融合。
  - `src/perception/dual_lidar_calibration/config/alioth_20260731.yaml`：

```yaml
main_lidar_topic: /livox/lidar_192_168_1_135
main_imu_topic: /livox/imu_192_168_1_135
secondary_lidar_topic: /livox/lidar_192_168_1_122
secondary_imu_topic: /livox/imu_192_168_1_122
initial_extrinsic: [0.0, 0.4, 0.0, -0.35453, 0.0, 0.0]
sync_tolerance_ms: 5.0
gicp: { min_overlap_ratio: 0.30, max_rmse: 0.25, ... }
```
  - 配对逻辑 `src/perception/dual_lidar_calibration/src/frame_synchronizer.cpp` 的 `pairNearestFrames()`：在 ±tolerance 内取**时间最近的一帧**，且每帧只用一次（单调用双指针扫描）。
- 代价地图（`src/navigation/navi2_bringup/params/nav2_params.yaml`）用的是标准 Nav2：本地 `nav2_costmap_2d::VoxelLayer`，`observation_sources: scan`（`data_type: LaserScan`，`topic: /scan`）；**因为点云已经合并成一个源，代价地图层面根本不存在双雷达问题**。

---

### 2.3 Front-end-Perception（Dongyang1525）— 读码最清晰的"先融合再 LIO"

- README 第 1 行：《基于双雷达的同步点云融合的高效前端感知系统》；第 5 行原文：

> 该项目使用 **PTP 进行双雷达时间硬件同步**，Mid360 雷达点云数据通过**修改后的 livox_ros_driver2** 将各自点云数据和 IMU 数据发布到各自实际物理外参坐标系下，然后将数据传入**修改后的港大开源的 Fastlio** 高鲁棒性里程计功能包，将点云数据通过 **TF 变换统一到同一坐标系下**，进行融合点云同步更新然后输出拼接后的点云和高鲁棒性里程计数据。

仓库只有感知（`src/FAST_LIO` + `src/livox_ros_driver2`），**没有代价地图**。

- 配置 `src/FAST_LIO/config/mid360.yaml`（分支 `Fastlio_feedforward`）：

```yaml
        common:
            odom_frame_id: "lio_odom"
            sensor_frame_id: "imu_link"
            base_frame_id: "base_footprint"
            # 当前工程采用"右雷达为主雷达"的双 MID360 方案
            lid_topic: "/livox/lidar_192_168_1_151"        # 单雷达兼容入口，非主订阅
            lid_topic_right: "/livox/lidar_192_168_1_151"  # 主雷达
            lid_topic_left:  "/livox/lidar_192_168_1_159"  # 副雷达
            imu_topic: "/livox/imu_192_168_1_151"          # 只有主雷达的 IMU
            time_sync_en: false                # 仅当无法硬件同步时才开软件同步
            time_offset_lidar_to_imu: 0.0
        preprocess:
            blind: 0.5
            crop_body_enable: true             # 车身近处裁切
            crop_body_negative: true           # 挖掉裁切盒内点
            crop_body_min_x: -0.4
            crop_body_max_x:  0.4
            crop_body_min_y: -0.3
            crop_body_max_y:  0.3
            crop_body_min_z: -0.1
            crop_body_max_z:  0.6
        mapping:
            extrinsic_est_en: false            # 双雷达下建议关闭在线外参估计
            extrinsic_T: [ -0.011, -0.02329, 0.04412 ]   # right_lidar -> IMU
```

- **融合实现**：`src/FAST_LIO/src/laserMapping.cpp` 的 `fuse_dual_lidar_frame_once()`（`:903-981`）。这是全报告里**最值得直接抄的一段**：

```cpp
// src/FAST_LIO/src/laserMapping.cpp:903-981（节选）
  bool fuse_dual_lidar_frame_once()
  {
    if (lidar_right_buffer.empty() || lidar_left_buffer.empty()) return false;
    while (!lidar_right_buffer.empty() && !lidar_left_buffer.empty())
    {
      const double t_151 = time_151_buffer.front();
      const double t_159 = time_159_buffer.front();
      const double dt = t_159 - t_151;

      if (std::fabs(dt) < 0.005)                       // ← 5 ms 配对窗口
      {
        tf_msg = tf_buffer_->lookupTransform("right_lidar", "left_lidar", tf2::TimePointZero);
        tf2::Transform tf_left_to_right;  tf2::fromMsg(tf_msg.transform, tf_left_to_right);
        ...
        for (const auto &pt : cloud_159->points) {
          tf2::Vector3 p(pt.x, pt.y, pt.z);
          tf2::Vector3 p2 = tf_left_to_right * p;
          PointType out = pt;
          out.x = p2.x(); out.y = p2.y(); out.z = p2.z();
          out.curvature = static_cast<float>(out.curvature + dt * 1000.0);  // 每点时间偏移
          cloud_159_in_151->push_back(out);
        }
        PointCloudXYZI::Ptr fused(new PointCloudXYZI());
        *fused += *cloud_151;          // 主雷达点直接进
        *fused += *cloud_159_in_151;   // 副雷达点变换后进
        std::sort(fused->points.begin(), fused->points.end(),
                  [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; }); // 按点时间重排
        lidar_buffer.push_back(fused);
        time_buffer.push_back(t_151);  // 整帧时间戳取主雷达
        ... 弹出两帧; return true;
      }
      if (dt > 0.005) { /* 丢主雷达旧帧 */ } else { /* 丢副雷达旧帧 */ }
    }
    return false;
  }
```

要点提炼：
- **配对窗口 5 ms，配对失败丢旧帧**（不是丢整帧，而是丢超期的那一路）。
- 副雷达点云用 **TF `right_lidar <- left_lidar`、`tf2::TimePointZero`（取最新可用 TF，不做时间插值）** 变换到主雷达系；融合帧以**主雷达时间戳**为准。
- 每点用 `curvature` 字段携带相对时间偏移，融合后**按点时间排序**，交给 FAST-LIO 做去畸变。
- 重叠区**没有去重**，纯拼接。
- 自车遮挡：`crop_body_*` 裁切盒 + `blind: 0.5`。
- 时间同步：**README 说硬件 PTP**，配置里 `time_sync_en: false`（即不做软件同步，因为硬件已同步）。

---

### 2.4 qdu26_sentry_navigation（QDU-VRobot）— Odin1 主 LIO + MID360 融合进地图

该仓库是 pb2025_sentry_nav 的改版，**移除了 point_lio**，换成 Odin1。

- `src/qdu2026_sentry_nav/odin1_integration/README.md`：
  - 新架构：`Odin1 雷达: 发布 odin1/odometry 和 odom->map TF` → `tf_inverter_node: 反转为 map->odom` → `pointcloud_fusion_node: 融合 Odin1 和 Mid-360 点云` → **融合点云可用于 Nav2 的障碍物检测**。
  - "旧架构（已移除）：point_lio / small_gicp_relocalization"。
  - 注意事项第 3 条：**"时间同步：两个雷达的时间戳应尽量同步，否则融合效果会受影响"** —— 即**没有做硬件同步，只靠同主机时间戳**。

- 融合节点参数（`src/qdu2026_sentry_nav/odin1_integration/src/pointcloud_fusion_node.cpp:81-119`）：

```cpp
    this->declare_parameter("odin1_topic", "odin1/cloud_raw");
    this->declare_parameter("mid360_topic", "/livox/lidar");
    this->declare_parameter("output_topic", "fused_pointcloud");
    this->declare_parameter("output_frame", "base_footprint");
    this->declare_parameter("robot_frame", "base_footprint");
    this->declare_parameter("enable_deskew", true);
    this->declare_parameter("allow_latest_tf_fallback", true);
    this->declare_parameter("tf_future_tolerance_sec", 0.30);
    this->declare_parameter("mid360_stamp_offset_sec", 0.0);   // 手动时间偏移补偿
    this->declare_parameter("max_sync_diff_sec", 0.15);        // 最大同步差 150 ms
    this->declare_parameter("mid360_queue_size", 5);
    this->declare_parameter("publish_without_mid360", true);   // 副雷达没数据也发主雷达
    this->declare_parameter("publish_reliable", true);
    this->declare_parameter("self_filter_half_x", 0.3);        // 自车过滤盒
    this->declare_parameter("self_filter_half_y", 0.3);
    this->declare_parameter("self_filter_height", 0.6);
    this->declare_parameter("self_filter_min_z", -0.5);
```

- **自车/云台遮挡自滤波**（`pointcloud_fusion.cpp:866-878`）：把落在以 `robot_frame` 为中心的盒子里的点直接丢掉：

```cpp
      const double dx = pt_robot.x(); const double dy = pt_robot.y(); const double dz = pt_robot.z();
      if (dx > -self_filter_half_x_ && dx < self_filter_half_x_ &&
          dy > -self_filter_half_y_ && dy < self_filter_half_y_ &&
          dz > self_filter_min_z_  && dz < self_filter_height_)
      { continue; }   // 丢弃车体/云台自遮挡点
```

- 时间同步做法：**软同步（最近邻匹配）**，`findClosestMid360()` + `max_sync_diff_sec_` 判定（`:779` `return best_dt <= max_sync_diff_sec_;`），失败时按 `publish_without_mid360` 决定是否仍发主雷达点云；另有 `mid360_stamp_offset_sec` 做固定偏差补偿，`allow_latest_tf_fallback` + `tf_future_tolerance_sec` 兜 TF 查不到的帧。
- **数据流（launch 实测）**：`src/qdu2026_sentry_nav/qdu2026_nav_bringup/launch/localization_launch.py:121-140` 里 fusion 节点被覆盖为 `output_topic = fused_cloud_slam`、`output_frame = odom`；`navigation_launch.py:116-142` 把 terrain_analysis / terrain_analysis_ext / sensor_scan_generation 的输入重映射：

```python
        remappings=[
            ("lidar_odometry", "odin1/odometry"),
            ("registered_scan", "fused_cloud_slam"),
        ],
```
  → **融合点云进 terrain_analysis → terrain_map → Nav2 VoxelLayer**；LIO 由 Odin1 自己完成（`lidar_odometry` 直接来自 Odin1）。
- 代价地图是标准 Nav2：`config/nav2_params.yaml:340-357` 本地 `VoxelLayer`、`observation_sources: terrain_map`、`sensor_frame: base_footprint`、`min/max_obstacle_height: 0.2/0.35`、`obstacle_max_range: 20.0`；**只有一个观测源**（融合后的）。

---

### 2.5 hnunavigation_-ros2（Gitee，河南大学）— 异构双雷达、各走各的源

这是唯一一个**明确把两个雷达做成两个/多路 observation source、并且用"分工"避免互相 clearing** 的例子。

- 第二个雷达实体证据：`hnurm_perception/ldlidar_stl_ros2/`（LDLiDAR LD19/LD06 驱动），`mapping.sh:6` 启动 `ros2 launch ldlidar_stl_ros2 ld06.launch.py`（`test.sh:6`、`debug_navigation.sh:6` 里被注释掉）。`driver_launcher.sh` 走的是 `hnurm_bringup/nav_with_mapping.launch.py`。
- **本地代价地图：两个独立 ObstacleLayer，各挂一个雷达**（`hnurm_navigation/params/nav2_params_steering.yaml:304-342`）：

```yaml
      plugins: ["obstacle_layer", "inflation_layer", "laser_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: livox
        livox:
          topic: /scan                       # MID360 -> pointcloud_to_laserscan
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          obstacle_max_range: 6.0
      laser_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: laser
        laser:
          topic: /ld_laserscan                # LD06/LD19 2D 雷达（经 pointcloud_filter）
          clearing: True
          marking: True
          data_type: "LaserScan"
```
  → 两个雷达**互不覆盖**：各自独立一层，靠 `combination_method` 在 master grid 上叠（默认 max）。

- **全局代价地图：`spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer` + 同一话题拆成"只 marking"和"只 clearing"两路**（`nav2_params_steering.yaml:379-409`）：

```yaml
      plugins: ["static_layer", "stvl_layer", "inflation_layer", "obstacle_layer"]
      stvl_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
        voxel_decay: 0.5
        decay_model: 0
        voxel_size: 0.05
        track_unknown_space: true
        mark_threshold: 0
        combination_method: 1
        transform_tolerance: 0.2
        observation_sources: livox_mark livox_clear
        livox_mark:
          data_type: PointCloud2
          topic: /pointcloud
          marking: true
          clearing: false            # ← 只标记
          obstacle_range: 4.0
          min_obstacle_height: 0.2
          max_obstacle_height: 2.0
          filter: "voxel"
          clear_after_reading: true
        livox_clear:
          enabled: true
          data_type: PointCloud2
          topic: /pointcloud         # ← 同一话题
          marking: false
          clearing: true             # ← 只清除
          max_z: 8.0
          min_z: 1.0                 # ← 只在 1.0 m 以上做 raytrace 清除
          vertical_fov_angle: 1.029
          horizontal_fov_angle: 360.0
          decay_acceleration: 5.0
```
  **这是应对"多源 raytrace 互相清除"最干净的一种配置技巧**：标记用 0.2–2.0 m 高度带，清除只在 1.0 m 以上高度带做，从而避免低矮障碍被自己/对方的射线打洞。同时用了 STVL 的体素时间衰减（`voxel_decay`）而不是纯 raytrace。

- 自车/自滤波：`hnurm_perception/pointcloud_filter/`（节点 `pointcloud_filter_node`）
  - 默认参数 `params/default.yaml`：

```yaml
    lidar_topic: "/segmentation/obstacle"
    output_lidar_topic_: "/pointcloud"
    odom_topic: "/Odometry"
    sensor_height: 0.31
    base_frame: "base_footprint"
    lidar_frame: "odom"
    laser_topic: "/laser/scan"
    laser_frame: "laser_link"
    robot_radius: 0.5          # 车身半径内的点/射线全部丢弃
    filter_topic: "/special_areas"
```
  - README 说明：按 `robot_radius` 去除车体回波；并支持用 `hnurm_interfaces/msg/SpecialArea` **多边形区域屏蔽**（map 系多边形经 tf 变换到传感器系，点/射线在区域内则丢弃或置 `+inf`）——这就是"屏蔽某个角度扇区/临时区域"的做法。

---

### 2.6 odin1_mid360_cloud_deskew（Cyan-chy）— Odin1 + MID360 的时间同步范本

- 输入 `/livox/lidar`（MID360 CustomMsg，**含每点相对时间偏移**）+ `/odin1/odometry_highfreq`；对每个点用相邻里程计做**平移线性插值 + 旋转 slerp**，投影到 odom 系；输出 `/cloud_cleaned`（PointCloud2）。核心公式写在 README：`p_W(t) = W_T_I(t) * I_T_M * p_M`。
- 外参经 `config/livox_cloud_deskew.yaml` 的 `I_T_M` 16 元素行主序矩阵给出（Mid360 雷达系 → Odin1 IMU 系）。
- **PTP 硬件对时全套文档**（本报告最重要的通用资产）：
  - `livox_cloud_deskew/docs/Mid-360_PTP时钟同步配置与验证.md`
  - `livox_cloud_deskew/docs/时钟同步脚本编写.md`

  文档给出的最小可运行流程（原文）：

```bash
# 1) 让主机网口当 PTP master（网卡需支持硬件时间戳，先 ethtool -T <iface> 确认）
sudo ptp4l -i <iface> -m
# 2) 把 Linux 系统时间同步到网卡硬件时钟（不跑这步雷达时间不跟随系统时间）
sudo phc2sys -s CLOCK_REALTIME -c <iface> -O 0 -m
# 3) 验证：过滤 ptp || udp.port == 319 || udp.port == 320
#    期望：主机发 Announce/Sync/Follow_Up，Mid-360 发 Delay_Req，主机回 Delay_Resp
```
  文档同时强调两个常见误区：**PTP 报文多为组播（`224.0.1.129`），不要按单播 IP 过滤**；**只启动 `ptp4l` 不够，必须配 `phc2sys`**；**PTP 与 PPS+GPRMC 是两套方案，不要混用**。
- 去畸变代码的时间处理：直接用 `msg->timebase + point.offset_time` 作为点时间戳（`src/livox_cloud_deskew.cpp:79-82, 160-161`），**默认两者同处一个时钟域** —— 这正是必须做 PTP 的原因。

---

## 3. 六个问题的直接回答

### Q1：有没有两个雷达的方案？是"先融合再 LIO"、"主雷达 LIO + 副雷达喂代价地图"、还是"各跑里程计再融合"？

有，5 个仓库。**三种范式在 RM 圈都有人用，但"各跑里程计再融合"一个都没有**：

| 范式 | 仓库 | 具体 |
|---|---|---|
| **先融合点云 → 再做 LIO** | navi_minco_bit、Front-end-Perception | 两 MID360 各自点云合并成一帧（同一时间基准、按点时间排序）后喂给**单一** Point-LIO / FAST-LIO；IMU 只用其中一个雷达的（Front-end-Perception 明说用主/右雷达内置 IMU） |
| **主雷达 LIO + 副雷达喂代价地图** | qdu26_sentry_navigation、HERO_2026_Sentry_NAV | Odin1 自带 LIO（`/odin1/odometry_highfreq`）做定位；MID360 点云（可能先与 Odin1 点云融合）只进地形分析/占据栅格→代价地图，不参与状态估计 |
| **各走各的观测源（不融合）** | hnunavigation_-ros2 | 3D Livox 与 2D LD06 分别进不同 costmap layer，从不合并点云 |
| **各跑里程计再融合** | 无 | 14 个仓库均未发现 robot_localization / 双 odom 融合的双雷达方案 |

补充：HERO_2026_Sentry_NAV 是"两步走"——**副雷达之间先融合（lidar_merge），然后主(副)雷达点云与 Odin1 点云在占据栅格层做双源加权融合**，属于上面第二、三类的混合。

### Q2：两个雷达的时间同步怎么处理？

四种做法，按可靠性排序：

1. **硬件 PTP**（最好）
   - Front-end-Perception：README 明写"使用 **PTP 进行双雷达时间硬件同步**"，因此配置里 `time_sync_en: false`（不需要软件同步）。
   - navi_minco_bit：要求 `Keep PTP/GPS time synchronization enabled`，配对窗口 `merge_max_interval_ms: 5.0`（驱动内部）/ `sync_tolerance_ms: 5.0`（标定）。
   - odin1_mid360_cloud_deskew：给出完整的 `ptp4l + phc2sys` 配置与 Wireshark 验证流程。
2. **同主机软件时间戳 + 近似时间同步（message_filters ApproximateTime）**
   - HERO_2026_Sentry_NAV `lidar_merge`：`ApproximateTime`（队列 5），融合后再用 `timebase + offset_time` 把两帧重排到同一时间轴。
   - 通用库 `dual_laser_merger`：也是 `ApproximateTime`。
3. **自写最近邻配对 + 容差窗口 + 固定偏移补偿**
   - qdu26：`max_sync_diff_sec: 0.15` + `mid360_stamp_offset_sec` + `allow_latest_tf_fallback`；README 坦承"两个雷达的时间戳应尽量同步"（即**没做硬件同步**）。
   - Front-end-Perception（FAST-LIO 内）：`std::fabs(dt) < 0.005`，超窗丢旧帧。
   - navi_minco_bit：比较两帧 `base_time`，`merge_max_interval_ms` 超窗丢旧帧。
4. **不做同步**：单雷达仓库不存在该问题；hnunavigation 两个雷达压根不融合，因此也不需要。

**结论：双 3D 雷达做点云融合的项目，全部采用"同一时间基准 + 小窗口（5 ms 级）配对 + 超窗丢帧"；头部方案用 PTP 对齐时钟，弱一些的用同主机时间戳加固定偏移补偿。**

### Q3：两个雷达点云重叠区域怎么处理？

| 项目 | 重叠区处理 |
|---|---|
| navi_minco_bit | **完全不处理**，前后两点云直接拼接（前雷达点直写、后雷达点变换后写） |
| Front-end-Perception | **完全不处理**，拼接后仅按点时间排序 |
| HERO_2026_Sentry_NAV | **不做点级去重**；在占据栅格层用 `LOG_OCC_HIT_mid360: 34` / `LOG_OCC_HIT_odin1: 16` 两个不同的 log-odds 命中权重融合，重叠区由概率模型决定；另有 z 向 PassThrough `(0.06, 1.0)` 与统计离群点滤除 |
| qdu26 | 拼接到 `base_footprint` / `odom` 系，无去重 |
| hnunavigation | 不融合，用不同 layer 分隔 |
| 通用库 dual_laser_merger | 有 **shadow filter**（`enable_shadow_filter`、`allowed_radius`）：对合并点云做 KdTree 半径搜索（半径按距离缩放 `allowed_radius/range_max * dist`），无邻居的点视为"阴影/鬼点"置为 `inf`（或 `range_max + inf_epsilon`）——这是外部库里少见的**专门处理重叠/遮挡伪点**的机制 |

### Q4：副雷达点云在哪个坐标系下发、用什么话题进代价地图？代价地图是 Nav2 还是自研？

| 项目 | 话题 | 坐标系(frame) | 代价地图 |
|---|---|---|---|
| HERO 2026 | `/livox/fused_cloud`(融合) → `/livox/cloud_scans`(去畸变) → 与 `/odin1/cloud_slam` 一起进 `dog_map` → **`/rog_map/inf_occ`** | Odin1 系 / odom（`output_frame`） | **自研**：Nav2 costmap_2d 容器 + 自研 `fast_layer::FastLayer`、`update_static_layer::UpdateStaticLayer`，**不用 observation_sources** |
| qdu26 | `fused_cloud_slam`（`output_frame: odom`）→ terrain_analysis → **`/terrain_map`**、`/terrain_map_ext` | `sensor_frame: base_footprint` | **Nav2** `nav2_costmap_2d::VoxelLayer`，`observation_sources: terrain_map` |
| navi_minco_bit | 融合后仍是 **`livox/lidar`**（`frame_id: livox_frame`）→ Point-LIO → 代价地图吃 `/scan` | `livox_frame` → `base_link` | **Nav2** `nav2_costmap_2d::VoxelLayer` / `ObstacleLayer`，`observation_sources: scan`（`data_type: LaserScan`） |
| Front-end-Perception | 融合后仍在 `right_lidar` 系，`sensor_frame_id: imu_link`，对外 TF `odom → base_footprint` | `right_lidar` / `imu_link` | **无代价地图**（纯前端感知） |
| hnunavigation | `/scan`（MID360 转 LaserScan）、`/ld_laserscan`（LD06 经 `pointcloud_filter`）、`/pointcloud`（STVL） | `base_footprint` / `laser_link` | **Nav2** `nav2_costmap_2d::ObstacleLayer` ×2 + 第三方 `spatio_temporal_voxel_layer` |
| pb2025（单雷达基线） | `<ns>/terrain_map`、`<ns>/terrain_map_ext` | `sensor_frame: front_mid360` | **Nav2 + 自研子类插件** `pb_nav2_costmap_2d::IntensityVoxelLayer` |
| cod（Gitee） | `/livox/lidar_filtered` + RealSense | `base_link` | **Nav2** `nav2_costmap_2d` + 自研 `intensity_voxel_layer` |
| HWSentryNav26 | 自研 `nav_executor` 内部消费 | 自研 | **完全自研**（`nav_executor/.../nav_map.hpp` 有 `CostMap::merge`） |

> 注意：**没有任何一个 RM 项目把"两路雷达"直接作为两个 `observation_sources` 挂在同一个 Nav2 layer 上**（hnunavigation 是把它们放进**两个不同的 layer**）。这本身就是社区规避"多源互相 clearing"的经验体现。

### Q5：有没有自车/云台遮挡的自滤波？

有，五个项目都做了，手法分四类：

| 手法 | 项目与文件 |
|---|---|
| **PCL CropBox + `setNegative(true)` 挖掉车体盒** | HERO 2026 `hero_lidar_scan`（硬编码盒 ±0.6 / z −0.3~1.0）；Front-end-Perception `mid360.yaml` 的 `crop_body_*`（±0.4/±0.3, z −0.1~0.6, `crop_body_negative: true`）；cod `cpp_lidar_filter/src/filter_node.cpp`（`min_x:-0.4 … max_x:0.4, max_z:0.6, negative:true`） |
| **自定义盒子/球体距离滤波** | qdu26 `pointcloud_fusion_node.cpp` 的 `self_filter_half_x/y/height/min_z`；HERO `lidar_merge` 的 `isBlind()`（球心 `ori_*` + `blind: 0.15`）+ Livox tag 盲区点剔除 |
| **半径去点 + 多边形区域屏蔽** | hnunavigation `pointcloud_filter`：`robot_radius: 0.5` 去车体回波；`SpecialArea` 多边形（map 系转传感器系）屏蔽任意区域/扇区，命中则丢点或置 `+inf` |
| **地形分析里的车高/相对高度带过滤** | pb2025 / qdu26 `terrain_analysis`：`vehicleHeight: 1.5`、`minRelZ: -1.5`、`maxRelZ: 0.3`、`disRatioZ: 0.2`、`clearDyObs` 等（`terrain_analysis/launch/terrain_analysis.launch`） |
| **观测源高度带限制** | hnunavigation STVL `min_obstacle_height: 0.2 / max_obstacle_height: 2.0`、`min_z: 1.0 / max_z: 8.0` |

**没有发现"屏蔽固定角度扇区"的实现**（`minDyObsVFOV/maxDyObsVFOV` 是垂直 FOV 限制，不是水平扇区）；hnunavigation 的 `SpecialArea` 多边形是最接近"任意扇区屏蔽"的通用方案。

### Q6：具体参数/代码片段

见上面第 2 节各小节，全部标注了「仓库 / 文件 / 行号」。最关键的三段：
1. 双雷达融合配时间轴排序 → `HERO_2026_Sentry_NAV/perception/lidar_merge/src/lidars.cpp:96-179`
2. 5 ms 配对 + TF 变换 + 按点时间重排 → `Front-end-Perception/src/FAST_LIO/src/laserMapping.cpp:903-981`
3. 全局代价地图 marking/clearing 分路 → `hnunavigation_-ros2/hnurm_navigation/params/nav2_params_steering.yaml:379-409`

---

## 4. 通用做法：Nav2 语义与多雷达合并节点

### 4.1 `observation_sources` 里 `marking` / `clearing` 的确切语义

Nav2 的 `ObstacleLayer`/`VoxelLayer` 对**每个 source 独立**执行两件事：

- **`marking: true`** — 把该源点云/扫描中位于 `obstacle_min_range ~ obstacle_max_range`、`min_obstacle_height ~ max_obstacle_height` 内的点，**在它命中的体素上标记障碍**（写代价）。
- **`clearing: true`** — 从**该源的传感器原点**向每个点做 **raytrace（射线追踪）**，把射线经过的、且距离在 `raytrace_min_range ~ raytrace_max_range` 内的体素**清除为自由**。注意 raytrace 是"从传感器到命中点之前的格子都算空"。
- 多个 source 时，各源各自生成自己的观测缓冲，然后按 `combination_method`（0 = overwrite / 1 = maximum）合并进 master grid。
- 相关参数：`observation_persistence`（0 = 只用最新一帧，>0 = 保留该时长内的所有观测）、`clear_after_reading`（读完即清空该源缓冲）、`filter`（`passthrough` / `voxel`）、`inf_is_valid`、`expected_update_rate`。
- 官方参数文档（URL 已验证有效，页面为 JS 渲染）：[Obstacle Layer Parameters](https://docs.nav2.org/lyrical/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/) 与 [Voxel Layer Parameters](https://docs.nav2.org/lyrical/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/voxel/)；语义的经典解释见 ROS wiki [costmap_2d/obstacles](https://wiki.ros.org/costmap_2d/hydro/obstacles)。

### 4.2 "多个雷达各自一路 observation source → 重叠区互相 raytrace 清除导致障碍闪烁/打洞"有没有公认解法？

**问题本身是真实存在的**：Nav2 并没有"多源之间互不清除"的语义保证，官方文档 issue [ros-navigation/docs.nav2.org#851 "observation_sources does not specify behavior when multiple sources are provided"](https://github.com/ros-navigation/docs.nav2.org/issues/851) 就是在指这件事；voxel_layer 相关 bug/困惑见 [navigation2#5099](https://github.com/ros-navigation/navigation2/issues/5099) 与 [Nav2 local costmap voxel layer not clearing obstacles from PointCloud data](https://robotics.stackexchange.com/questions/118625/nav2-local-costmap-voxel-layer-not-clearing-obstacles-from-pointcloud-data)。

社区/工程上的通用解法（按推荐度）：

1. **合并点云成单一 source（最彻底）** —— 让代价地图永远只看到一个观测源，问题从根上消失。
   - 本报告里 navi_minco_bit / Front-end-Perception / qdu26 / HERO 全部走这条路。
2. **只让一个源 clearing，其余只 marking** —— hnunavigation 的 `livox_mark`(marking only) / `livox_clear`(clearing only) 即此模式。
3. **用不同的高度带/量程把 marking 与 clearing 错开** —— 例如标记 0.2–2.0 m、清除只在 1.0–8.0 m 以上做（hnunavigation 实测配置），避免低矮障碍被射线打洞。
4. **把副雷达限制在 ROI / 近场** —— 缩小 `obstacle_max_range`/`raytrace_max_range`，或对副雷达单独做 CropBox/PassThrough 预处理，使其射线不覆盖主雷达的标记区域。
5. **用带时间衰减的体素层代替 raytrace clearing** —— `spatio_temporal_voxel_layer`(STVL) 的 `voxel_decay` / `decay_acceleration`，或 Nav2 `VoxelLayer` 的 `voxel_decay`；障碍靠衰减老化而不是靠"被看不见就清掉"。
6. **拉长 `observation_persistence`、降低 `update_frequency`、放宽 `transform_tolerance`** —— 只减轻闪烁频率，不解决根因。
7. **开启 `footprint_clearing_enabled` 并保证自车点被滤掉** —— 否则自车/云台点会周期性标记又清除。
8. **多个雷达放进不同 layer**（每个 layer 一个源）并把 `combination_method` 设为 0（overwrite）/1（max）仔细调 —— 代价是层数变多、行为更难推理；hnunavigation 本地代价地图就是这么做的。

### 4.3 专门做多雷达点云/扫描合并的开源节点，以及它们对时间戳与 TF 的处理

| 项目 | 输入/输出 | 时间戳处理 | TF 处理 | 备注 |
|---|---|---|---|---|
| [dual_laser_merger](https://github.com/pradyum/dual_laser_merger)（ROS 2 Humble/Jazzy/Rolling） | 2× `LaserScan` → 1× `LaserScan`（+ 可选合并点云） | `message_filters` **ApproximateTime** 双路同步（`src/dual_laser_merger.cpp:46-53`）；合并消息 `merged.header = cloud_out.header`，`time_increment = 0`，`scan_time` 参数化 | 两路各自 `tf2_buffer->transform(cloud, target_frame, tf2::durationFromSec(tolerance_param))`，`tolerance` 默认 `0.01` s；要求存在 laser_1/laser_2 → `target_frame` 的 TF，支持 `*_x_offset/y_offset/yaw_offset` 额外外参 | 有 **shadow filter**（`enable_shadow_filter`, `allowed_radius`）处理重叠/伪点；按 `min_height/max_height` 过滤高度 |
| [ira_laser_tools](https://github.com/iralabdisco/ira_laser_tools) | `laserscan_multi_merger`：多路 `LaserScan` → 1 路 | 见 issue [iralabdisco/ira_laser_tools#36 "Use latest timestamp in laser merger scan publish"](https://github.com/iralabdisco/ira_laser_tools/issues/36) —— 社区要求/说明输出时间戳取**最新一帧**的时间 | 通过 `destination_frame` + `laser*_frame` 的 TF 把各扫描变换到目标坐标系 | ROS1 时代的老牌方案，主要用于多 2D 雷达 |
| Livox 驱动内置多雷达（`multi_topic=1` / 混合 HAP+MID360） | 多台 Livox → 各自话题，或驱动内合并 | 驱动在同一主机上收 UDP，`base_time` 同源 | 驱动配置里的 `extrinsic_parameter`（roll/pitch/yaw/x/y/z）为**每台雷达**的外参 | navi_minco_bit 把它改造成 `InternalLidarMerger`（见 2.2） |
| Autoware `pointcloud_preprocessor` 的 [concatenate_data](https://raw.githubusercontent.com/miursh/autoware.universe/refs/heads/main/sensing/pointcloud_preprocessor/docs/concatenate-data.md) | 多路 `PointCloud2` → 1 路 | 逐个输入按 `tf` 变换后拼接，输出时间戳策略可配 | 每路用自己的 `frame_id`，查 TF 到 `output_frame` | 工业级参考实现，参数化程度高 |
| `robot_localization` | 多路 odom/imu → 融合状态 | 各自时间戳进 EKF/UKF | — | **注：本报告 14 个 RM 仓库中没有任何一个用它对双雷达里程计做融合**；如果真要走"两雷达各跑 LIO 再融合"，用它做双 odom 融合在理论上可行，但要额外解决"两套 LIO 各自漂移/各自 map→odom"的问题 |

---

## 5. 给"双雷达哨兵"的建议（基于以上证据）

1. **架构上优先"合并点云成单一观测源"**：要么在驱动/独立节点里合并（navi_minco_bit、Front-end-Perception），要么在代价地图之前合并（qdu26、HERO）。**不要**把两路雷达直接配成同一 `observation_sources` 的两项。
2. **时间同步优先 PTP**：两台 MID360 与主机放到同一二层网络，`ptp4l`（主机做 master）+ `phc2sys -s CLOCK_REALTIME`；用 Wireshark 过滤 `ptp || udp.port == 319 || udp.port == 320` 确认看到 `Delay_Req/Delay_Resp`。做不到就在软件层做"5 ms 窗口配对 + 超窗丢旧帧 + 固定 `stamp_offset` 补偿"。
3. **融合时务必重排逐点时间**：Livox `CustomMsg` 的 `timebase + point.offset_time` 是去畸变的基础；两帧融合后要按点时间排序（Front-end-Perception 用 `curvature` 字段承载偏移并排序），否则 LIO 去畸变会把两帧运动混在一起。
4. **重叠区不要靠点级去重**：优先级是（a）占据栅格 log-odds 加权（HERO 的 `LOG_OCC_HIT_mid360/odin1`），（b）体素下采样/阴影点过滤（dual_laser_merger 的 shadow filter）。
5. **自车/云台自滤波至少做两层**：点云侧 CropBox（挖掉云台/车体盒）+ 代价地图侧高度带限制；如果云台会转到侧后方，考虑 hnunavigation 的"多边形区域屏蔽"思路做个可动态发布的屏蔽区。
6. **抄 HERO 时要先修 `TWO_LIDARS` 编译宏**，否则拿到的是单雷达直通版本。

---

## 6. 明确未验证/不确定的部分

- `LiuJinbo1027/HERO_2026_Sentry_NAV` 里 `lidar_merge` 的 `merge.cpp`（双外参版本）**没有被 CMakeLists 编译**（只编译 `src/lidars.cpp`），且两个包的 CMakeLists 都未定义 `TWO_LIDARS`。仓库只有 1 个 commit（"Initial commit: HERO 2026 哨兵导航算法开源"），无法从 git 历史判断作者实机用的是哪个版本。**结论"该项目实机启用双雷达"依据的是 README 明述 + `cfg_odin.yaml`/`params.yaml` 中的双雷达话题配置**，代码路径需要自己加宏编译。
- `hnunavigation_-ros2` 的 LD06/LD19 驱动在 `mapping.sh` 中启动，在 `test.sh`/`debug_navigation.sh` 中被注释；`navigation.sh` 未启动它。**因此"双雷达同时在线"只在建图流程中被证实，导航流程下第二路 `/ld_laserscan` 是否在线未能验证**，但代价地图配置确实为两路都留了位置。
- gitee 两个仓库使用 HTTPS 克隆成功，但未逐文件对比其 CI/构建状态。
- `Front-end-Perception` 的默认分支是 `Fastlio_feedforward`（不是 main），如果看 main 分支可能看不到双雷达代码。
- `navi_minco_bit` 的 `nav2_params.yaml` 中 `use_sim_time: True`、观测源是 `/scan`（LaserScan），推测那是仿真配置；真机配置未在该文件中找到，故其"真机代价地图输入"未完全验证。
- Nav2 官方文档页面为前端渲染，抓取时正文被截断；`marking`/`clearing` 语义的依据为 Nav2 参数文档 URL + ROS wiki 经典描述 + 各仓库实配参数的交叉印证。
