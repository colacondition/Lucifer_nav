# 审计 A：linefit 地面分割正确性 + 安全修复 + 回归测试

- 任务：task-8（本文件）。范围：`src/perception/linefit_ground_segmentation_ros2/`
  （算法库 + ROS 节点 + 测试）与三份 `segmentation.yaml`。
- 环境：ROS 2 Humble（构建命令见 §8）。
>
> **决策/后续更新（2026-10-23）**：
> 1. `r_min/r_max` 外点已改为 `label=2`（忽略），ROS 节点不再把它们当障碍发布（D2 关闭）；
> 2. 云台只有 yaw 旋转、没有俯仰：整变换到 base_link 的处理链本来就正确，
>    审计中提出的"重力对齐-原点"实验模式已撤销（前提不成立）；
> 3. 去车身语义已明确为"去云台上其他结构"，与地面分割的 `sensor_height` 链解耦（详见 audit-body.md）。
>    本报告 §0 表中 D1/D3 仍为待决策项。
> **2026-10-23 更新（用户决策）**：曾按用户要求回退过一次；随后确认导航倾斜/“飞起来”
> 的根因是 **PCD/建图时的 LIO 初始姿态**，与地面分割无关，用户要求把地面分割修复恢复。
> 当前代码已恢复审计中的全部修复（`computeBinIndex`/`bin` 防护、`label=2`、节点输出过滤、
> 鲁棒性测试），并保留本机 PCL 1.12 的构建修复。
>
> 原则：只做不改变正常场地分割结果的低风险修复（越界/NaN/输入校验/构建依赖），
  会改行为或需实车的参数进「待决策」；未改 URDF/外参/真实配置值；未 commit。
- 关联审计：`base_link` 语义与 costmap 高度带见 `docs/tdt-compare/audit-body.md`
  （下称 **body 审计**），本报告只在 §7 引用其 §6，不重复推导。

---

## 0. 结论摘要（先看这张表）

| # | 结论 | 严重度 | 证据 | 处置 |
| :- | :- | :- | :- | :- |
| 1 | `insertionThread` 的 bin 索引靠浮点不变量兜底，`n_bins=0`/极端参数或舍入可越界写 `segments_[seg][n_bins]`（UB） | 🟠 中（潜伏） | `ground_segmentation.cc:245-292`（旧版 :258-262）；本次改为 `computeBinIndex`（`ground_segmentation.h:14-42`） | **已修**（clamp + 参数校验 + gtest 复现） |
| 2 | NaN/Inf 点的去向：label 恒为 0（障碍），旧实现会原样进 `/segmentation/obstacle`；z=NaN 还会让 bin 留下 `min_z=DBL_MAX` 的假点污染线拟合 | 🟠 中 | `bin.cc:16-38`、`ground_segmentation.cc:267`、node `:521-540` | **已修**（bin 拒收 + 输出过滤）；`r_min/r_max` 外点仍输出 → 待决策 |
| 3 | `r_min/r_max` 之外的点（含 <0.2m 和 >10m）label=0，与真障碍一起发布到 `/segmentation/obstacle`；下游靠各自 range 参数二次过滤 | 🟡 低-中 | 库 `ground_segmentation.cc:273-275`、node `:521-540`；`pointcloud_to_laserscan.yaml:11-12`、`navigation2.yaml:119-120` | **待决策**（报告现状与影响，未改输出行为） |
| 4 | `line_search_angle=0.8 rad(≈46°)` 过大：邻域补线最远跨 45 个 segment（1°/segment）、每点最多 45×2 次线查询；能桥接 10° 级空扇区，但也会把远处地形/矮结构按邻域地面线判成地面 | 🟠 中（风险/性能） | `ground_segmentation.cc:185-206`、`config/common/segmentation.yaml:8,20` | **待决策**（不改真实配置；报告风险与建议区间） |
| 5 | `max_start_height=0.1` 的真实语义 = "能识别的最小台阶高度"；`0.1` 能看见 20cm 台面（有回归测试），>0.1 的台阶会被静默吞掉 | 🟢 低（约定正确） | `segment.cc:76-88`、`test_step_detection.cpp:138-191`、`segmentation.yaml:19` | **维持**（测试锁住） |
| 6 | `processing_frame=base_link`+`gravity_aligned_frame=base_link`+输入 header=odom 的链**自洽**：base_link=雷达位姿，云变换后地面在 z=-h，与 `sensor_height` 0.49/0.175 一致；但 50ms TF 超时+严格 stamp 会丢帧 | 🟢 低-中 | node `:435-455`、`:462-490`、body 审计 §1.1 | **待决策**（缺 TF 时无 latest 兜底） |
| 7 | `sensor_height_frame: ""` 的真实行为：**整段 TF 解析被跳过**，实际生效的是 yaml 兜底值（real 0.49 / sim 0.175）；注释"真值来自 TF / measurement_params 是唯一来源"与运行期不符 | 🟠 中（配置-文档） | node `:253-266`、`:332`、`:358-407`；`common/segmentation.yaml:11-12`、`measurement_params_sim.yaml:5` | **待决策**（不改配置值，见 §3.3 与 body 审计 §1.4） |
| 8 | 实车外参 `measurement_params_real.yaml:18` xyz 的 z 实际是 `0.0`，同文件注释却写"现按 xacro default 取 0.49"，且与 `sensor_height: 0.49` 语义冲突 | 🔴 高（外参） | `config/reality/measurement_params_real.yaml:11-18`、`common/segmentation.yaml:11`、body 审计 §1.3 | **只报告**（禁止改外参） |
| 9 | 20cm 高台在**消费端**仍会静默消失（costmap 高度带相对雷达平面而非底盘） | 🔴 高（功能） | 见 body 审计 §6；`navigation2.yaml:112-120,279-282` | **只报告**（跨包，非本任务 scope） |
| 10 | `common/segmentation.yaml:23` 的 `latch: false` 节点从未 `declare_parameter("latch")`，是死配置；输出无 latching | 🟡 低 | node `:234-332`（无 latch 声明） | 报告/建议 |
| 11 | ros 包 CMakeLists 未 `find_package(PCL)`，旧 build 目录能编过是继承了环境 include 路径；清理 build 后编译失败 | 🟠 中（构建） | `linefit_ground_segmentation_ros/CMakeLists.txt:10-25` | **已修**（显式 PCL common） |
| 12 | `RMUL.pcd` 的"地面主峰 -0.17（37287 点里 14558 个）"注释**无法从当前仓库 PCD 复现**（工作区 PCD 已被改成 834 点；HEAD 版 12354 点，z 直方图峰值约 -0.32） | 🟡 低（文档/数据） | `navigation2.yaml:112`、`measurement_params_sim.yaml:6`；`src/bringup/PCD/RMUL.pcd`（git 状态 M） | 报告/建议复测（只影响 `base_to_ground_z` 默认 0，当前配置下未被使用） |

一句话：**"20cm 高台不被当坡道"在 segmentation 内成立（测试锁死），真正失效点在
分割→costmap 的坐标系语义（body 审计 §6）；本轮把三处输入边界（越界/NaN/非法
参数）从"UB 或脏数据"变成"显式拒绝"，未改变正常场地分割结果。**

---

## 1. 算法正确性

### 1.1 `insertionThread` 的 bin 索引越界（已修）

旧代码（修复前 :256-262）把 `(range-r_min)/bin_step` **直接**
`static_cast<unsigned int>`，并假设 `range_square < r_max_square` 时商必 < `n_bins`。
这是**浮点不变量，不是类型保证**：`GroundSegmentationParams` 是 public API
（gtest/其它节点可绕过 yaml 校验），病态 `r_min/r_max/n_bins` 或
`sqrt(x²+y²)` 的舍入都可能让商落到 `[n_bins, n_bins+1)`，随后
`segments_[seg][n_bins]` 是 `std::vector` 越界写（UB，不保证当场崩）。
已用真实浮点病态用例复现：`r_min=0, r_max=1, n_bins=7` 时
`nextafter(r_max,0)/bin_step == 7.0`（`test_ground_segmentation_robustness.cpp:73-91`
的前置 `ASSERT_EQ` 就是钉这个前提）。现场配置 `r_min=0.2, r_max=10, n_bins=120`
下该边界**实际不可达**（float 平方和离 `r_max²` 最近 ~1e-6，远大于舍入阈值），
所以是**潜伏 bug 而非现网已触发**；此外 `n_bins=0` 会让
`fitSegmentLines()` 解引用空 `bins_` 的 `end()`（`segment.cc:31-36`），
`n_segments=0` 会让 `segment_step=inf` 并访问空 `segments_`，节点以前不校验这些。

修复（`ground_segmentation.h:14-42` + `ground_segmentation.cc:245-292`）：
新增 `computeBinIndex()`——非有限输入/区间外/非法参数返回 `-1`，合法输入返回值
**保证**在 `[0,n_bins)`，逼近上界时 clamp 到 `n_bins-1`（in-range 点结果与旧实现
逐点相同）；`insertionThread` 只在 `bin_index>=0` 时写 bin；segment 索引从
"只处理 `==n_segments`"改为显式 clamp。节点启动时校验
`n_bins/n_segments/n_threads/line_search_angle/sensor_height`（node `:285-305`），
配置错误启动即抛 `invalid_argument`。

### 1.2 `r_min/r_max` 之外与 NaN/Inf 点的最终 label（现状，部分修复）

`insertionThread` 的 else 分支给 `bin_index_[i]=(-1,-1)`（`ground_segmentation.cc:273-275`），
`assignClusterThread` 只处理 `segment_index >= 0` 的点（`ground_segmentation.cc:181`），
因此**所有 out-of-range / NaN / Inf 点 label 保持初始值 0**，与"障碍"同义。
node 旧代码无条件把 `labels[i] != 1` 的点塞进 `obstacle_cloud`（node `:521-540`），
所以：

- `r < r_min`（0.2m，含自车/近场）与 `r >= r_max`（10m）的点**都会出现在
  `/segmentation/obstacle`**；
- NaN/Inf 点（label 0）也会原样发布。

影响链（为什么目前没炸）：`pointcloud_to_laserscan.yaml:11-12` 的
`range_min: 0.45 / range_max: 10.0` 会再滤一次；costmap 的
`obstacle_min_range/obstacle_max_range`（`navigation2.yaml:99-100,266-267`）也是
3~6m 级。所以**消费端当前不会因此产生可见幽灵障碍**，但 `/segmentation/obstacle`
是给 RViz/slam_toolbox 建图/未来消费者共用的话题，"标签 0 = 障碍"的语义被
边外点污染是隐患（尤其建图模式下 slam_toolbox 直接吃该话题）。

本次已修的部分：NaN/Inf 点在**出口统一剔除**（node `:527-529`），且 bin 层拒收
非有限 z（`bin.cc:16-38`，见 1.3）；合法点 label 完全不变。
**未改** `r_min/r_max` 外点的输出行为（会改变话题内容/带宽，且需要确认建图与
costmap 无依赖），列入「待决策」并给出建议（§7 D2）。

### 1.3 per-bin 最小 z：NaN 污染（已修）

`Bin::addPoint` 旧逻辑先 `has_point_=true`，再 `if (z < min_z)` 更新——`z=NaN`
时比较恒假，于是 bin「有最小点」但 `min_z` 仍是 `DBL_MAX`，`getMinZPoint()`
返回 `(z=1.8e308, d=0)` 的假点，被 `fitSegmentLines` 当最低点参与拟合
（`segment.cc:29-96`），把整条地面线误差拉爆（通常会被 `max_error`/`max_slope`
拒掉重起，但这是"一帧坏点 → 整个方位地面丢失"的合理机制）。修复：
`addPoint` 入口 `!std::isfinite(d) || !std::isfinite(z)` 直接 return
（`bin.cc:16-38`），正常点逻辑/并发快路径/`update_mutex_` 语义未动；
单测 `BinRejectsNonFinitePoints`（`test_ground_segmentation_robustness.cpp:119-148`）。

### 1.4 线拟合 / 续线逻辑审计（未改）

`Segment::fitSegmentLines()`（`segment.cc:29-96`）逐条核对：无点保护正确
（找不到点直接 return，:31-36）；拒线回退 `pop_back → 提交前一条线(size>=3) →
erase(begin,--end()) → --line_iter` 重新处理被拒点（:56-72）不会丢点/死循环；
起线高度判据见 §1.6；`fitLocalLine` 闭式最小二乘 + 病态保护（:145-170）正确；
`verticalDistanceToLine` 只对 `[first.d-0.1, second.d+0.1]` 插值（:112-124），
margin 0.1 与 `max_dist_to_line` 同量级。两个**未修**小问题（不改行为）：
① 循环遍历所有 line 且不 break，重叠区间时后加入的（更远）线覆盖前者——取
最近距离更稳；② `dist < max_dist && dist != -1`（`ground_segmentation.cc:206`）
用魔法值 `-1` 当 sentinel，`dist >= 0` 更清晰（建议 S1）。

### 1.5 `line_search_angle: 0.8` 的实际含义与风险

- 单位：rad；`n_segments=360` → 每 segment 1°（`segment_step=2π/360`）。
  `assignClusterThread` 在"本 segment 无线命中"时向两侧搜索
  `steps*segment_step < 0.8`，即**最多 45 个 segment、约 ±45.8°**
  （`ground_segmentation.cc:185-206`），最坏每点 45×2 次
  `verticalDistanceToLine`（每次再遍历该 segment 的 line 列表）。
- 语义风险：邻域地面线是**外推**到该点的 d，只有 `|z - expected_z| < 0.1`
  才算地面。因此：
  - 20cm 台面不会因 0.8 被吞（0.2 > 0.1，测试已在 `line_search_angle=0.8`
    下覆盖，见 `test_step_detection.cpp:138-147`）；
  - 但**≤ `max_dist_to_line`（10cm）+ 邻域线斜率误差**的矮结构/地形起伏会被
    45° 外的地面线判成地面——这是参数放大的直接代价；0.2 rad（原版上游默认）
    只有 ±11.5°，被外推到 45° 后的地面高度误差也相应放大；
  - 远处（10m）45° 横向跨度约 7m，若该处地面有坡/坎，邻域线高度已不可信，
    只是被 10cm 容差兜住。
- 性能风险：障碍/欠采样帧里大量点走邻域搜索，最坏 45×2×lines 次查询；
  node 只在 `segment_ms > 100ms` 打警告（node `:501-508`），正常 <10ms。
- 判断：**高风险参数，但不在本任务改**。建议用同一帧对比 `0.2/0.4/0.8` 的
  obstacle 点云与 `segment_ms`，确认 0.8 是跨稀疏雷达空扇区的实测需要（本次补的
  `LineSearchAngleBridgesMissingAngularSector` 证明它能跨 10° 空扇区，但 0.2
  只有 ±11.5°，理论上也能跨）还是历史遗留。

### 1.6 `max_start_height: 0.1` 与"20cm 高台不被当坡道"约定

机制（`segment.cc:39,76-88`）：`cur_ground_height` 初值 `-sensor_height`；一条线
被拒后只剩最后一点，下一点要续线必须满足
`|上一点 z - 已提交地面线末端高度| < max_start_height`，否则另起新线（而新线在
台面上因连续超差永远长不到 3 点），台面因此拿不到地面线 → 被
`verticalDistanceToLine` 判为障碍。

结论：**该参数的真实语义是"能识别的最小台阶高度"**，配置 `0.1`
（`segmentation.yaml:19`）⇒ 高度 >10cm 的台阶/高台判障碍，≤10cm 的起伏允许
被当地面。这与 navigation 注释里的 20cm 约定一致（`navigation2.yaml:279-280`
明确要求 `max_start_height < 最矮台阶高度`）。已有 5 条回归测试锁死
（`test_step_detection.cpp:138-191`），其中：
- `ConfiguredParamsSeeTwentyCentimetrePlatform`：配置值必须看见 20cm 台面；
- `LargeMaxStartHeightSwallowsPlatform`：`0.5` 必须吞掉台面（证明参数因果）；
- `MaxStartHeightBoundsSmallestVisibleStep`：10/15/20/30cm 都要可见；
- `SlopedGroundDoesNotBecomeObstacle`：0–15% 坡度不虚警；
- `RadialResolutionDoesNotAffectStepDetection`：n_bins 80–300 不影响台阶。

风险提示：`max_start_height` 与 `sensor_height` 强耦合——`sensor_height` 若比实际地面
高度差 >0.1（例如配错 0.59 vs 实测 0.49），起线判据永远不满足，**整帧地面都长不出线
→ 全部判障碍**。所以 sensor_height 一致性是安全项，不是文档洁癖。

---

## 2. 节点正确性

### 2.1 变换链：`processing_frame=base_link` + `gravity_aligned_frame=base_link` + 输入 header=odom

- 输入 `/livox/lidar_filtered/pointcloud` 的 frame 是 `odom`
  （`common/segmentation.yaml:25`；lidar_filter `cloud_frame_id: odom`）。
  `processing_frame_` 非空且 ≠ 输入 frame 时，按 **msg stamp** 查
  `lookupTransform(base_link, odom, msg->header.stamp, 0.05s)` 做完整平移+旋转
  （node `:435-455`），输出 frame 改为 `base_link`；随后 `is_original_pc=false`，
  `gravity_aligned_frame` 分支被跳过（node `:462-490`）——两个都写 `base_link` 时
  第二次旋转是**死代码**，不会重复旋转，链是对的。
- `base_link` 语义由 body 审计 §1.1 锁定：**原点在雷达上**（small_glim 的
  odometry child frame = 雷达位姿）。所以变换后地面在 `z≈-h`
  （实车 -0.49 / 仿真 -0.175），与 `sensor_height` 一致（`segment.cc:39` 用它
  初始化 `cur_ground_height`）——**这条链自洽**。
- 风险（待决策 D4）：TF 只用 msg stamp + 50ms 超时，不做 latest 兜底；LIO 10Hz
  输出与点云 stamp 不同源时启动/抖动会整帧 `return`（2s 节流 WARN，
  node `:431-455`）。若实测掉帧，可"严格 stamp 失败 → 退 latest + WARN 一次"，
  会改变行为，留待决策。

### 2.2 msg stamp 与 TF 匹配、输出 header

- 输出 `header = msg->header`（保留原 stamp），只改 `frame_id = output_frame`
  （node `:533-535`、`:546-550`）——下游按同一 stamp 变回 map，正确。
- `output_frame` 在无变换时是输入 frame（odom），有变换时是 base_link；
  配置下恒为 base_link，与 body 审计 §6 的消费端假设一致。
- 未发现 stamp/frame 错配；历史 bug（`labels` 对应 `cloud_proc` 却复制原始
  `cloud`）已在代码注释中记录并修复（node `:518-521`）。

### 2.3 `sensor_height` 在"base_link=雷达位姿 + 点云 frame=odom + `sensor_height_frame: ''`"下的真实行为

- 节点逻辑（node `:253-266`、`:332`、`:358-407`）：
  `sensor_height_frame_.empty()` ⇒ 构造时 `sensor_height_resolved_=true`
  ⇒ `resolveSensorHeight()` 第一行直接 return，**TF 解析整段不执行**；
  实际 `params_.sensor_height` 就是 yaml 值：**real 0.49 / sim 0.175**
  （`common/segmentation.yaml:11` + `reality/segmentation.yaml:4` /
  `simulation/segmentation.yaml:4`）。
- 若把 `sensor_height_frame` 打开：查 `base_link ← odom` 的 z ≈ **雷达在
  odom 中的高度**。按 body 审计 §1.1/§1.4，odom 原点锚在开机雷达平面，
  `base_link` 也是雷达位姿，两者 z≈0 ⇒ `resolved≈0` ⇒ 触发
  `resolved<=0` 的 ERROR 分支并锁回 yaml 兜底（node `:392-400`）。
  也就是说：**"measurement_params 是雷达离地高度唯一来源"的注释在当前帧链上不成立**
  （`measurement_params_sim.yaml:5`、node `:253-257`），生产实际以 yaml 为准。
  这与 body 审计 §1.3/§1.4 结论一致，本报告不重复推导。
- 与 PCD 注释的自洽性：sim 0.175 与"地面主峰 -0.17"在 0.005 内吻合
  （`measurement_params_sim.yaml:6`）。但当前仓库 `RMUL.pcd` 工作区版本只有
  834 点（git 状态 M），HEAD 版 12354 点、z 直方图峰值约 -0.32；
  navigation 注释里的"37287 点里 14558 个"（`navigation2.yaml:112`）**无法复现**。
  该注释只被用来论证 `base_to_ground_z` 默认 0（当前配置下该参数未生效），
  影响低，建议用实车过滤后点云重测一次（建议 S3）。
- 实车外参矛盾：`measurement_params_real.yaml:18` 实际 z=`0.0`，同文件注释
  `:12-14` 写"现按 xacro 原有 default 取 0.49"——文件与注释互相矛盾（body 审计 §1.3）。
  **禁止在本任务修改**，交外参负责人决策（D5）。

### 2.4 QoS / 看门狗 / 多线程 / publish 前订阅数

- 输入 `SensorDataQoS`（best-effort，node `:314-316`），与上游 lidar_filter 一致，
  intra-process 可走进程内指针投递（Humble 只接受 volatile+keep_last）。
- 输出 `KeepLast(5)+RELIABLE`（node `:319-329`）：pointcloud_to_laserscan/RViz/
  slam_toolbox 都是 RELIABLE 订阅，best-effort 发布对它们不投递；RELIABLE 发布对
  best-effort 订阅兼容，方向正确。
- publish 前订阅数（node `:415-421`）：ground/obstacle 都无订阅者时直接 return，
  省掉 TF/分割 CPU；副作用是**没有任何订阅者时节点不产出**（单测/工具须先订阅，
  已有 launch test 正是这么做）。可接受，建议 S2 增加 `always_process` 参数。
- 看门狗（node `:131-193`）：独立线程 + `SIGUSR2` backtrace + wchan dump，节流
  （首次全量、10s 状态、60s 全栈）合理。风险：`SIGUSR2` handler 与 `g_seg_*`
  是**进程级**的，同容器加载多个本组件实例会互相干扰（当前只加载一个，风险低，
  建议 S2 改实例成员）；看门狗 `fprintf(stderr)` 可能与卡住线程的 stdio 锁互等
  （只影响诊断，不影响主流程）。
- 多线程：`runParallel` 用 OpenMP fork-join（`ground_segmentation.cc:68-95`），
  分片与旧实现一致；`Bin` 用原子量+`update_mutex_` 保证 (min_z,min_z_range)
  成对更新（`bin.cc:16-38`）；点云级并行只写各自下标，`insertPoints` 结束后才
  `getLines`/`assignCluster`，无读-写竞争。**未发现数据竞争**。
- `n_threads` 配置 2（`segmentation.yaml:4`），容器 executor 线程数由
  `perception_threads` 控制（`nav_common.py:29-40`），不超订，合理。

---

## 3. 配置一致性（三份 yaml）

| 参数 | common | reality | simulation | 风险判断 |
| :- | :- | :- | :- | :- |
| `sensor_height` | 0.49 (`:11`) | 0.49 (`:4`) | 0.175 (`:4`) | 与实际安装高度一致时正确；配错 >0.1 会导致整帧无地面线（§1.6 耦合） |
| `sensor_height_frame` | `""` (`:12`) | 未覆盖 | 未覆盖 | **关闭 TF 解析**，yaml 是唯一实际来源；与注释不符（D3） |
| `base_to_ground_z` | 0.0 (`:13`) | — | — | `sensor_height_frame=""` 时**完全不生效**（死参数），只在打开 TF 解析时有意义 |
| `r_min / r_max` | 0.2 / 10.0 (`:5-6`) | — | — | 与成本图 range、点云有效距离一致；边界点见 §1.2 |
| `n_bins / n_segments` | 120 / 360 (`:7-8`) | — | — | 径向 9.8/120≈8.2cm，角度 1°；`test_step_detection` 证明 n_bins 不影响台阶；越界防护已加 |
| `max_dist_to_line` | 0.1 (`:9`) | — | — | 与 `line_search_angle` 共同决定"把多高的结构当地面"；0.1 合理 |
| `line_search_angle` | 0.8 (`:20`) | — | — | 46°，**待实测**（§1.5，D1） |
| `max_start_height` | 0.1 (`:19`) | — | — | 与 20cm 约定一致，**测试锁死**（§1.6） |
| `processing_frame` / `gravity_aligned_frame` | base_link / base_link (`:21-22`) | — | — | 链自洽，gravity 分支死代码；见 §2.1 |
| `latch` | false (`:23`) | — | — | **未声明参数，静默忽略**（D6） |
| `input_topic` | `/livox/lidar_filtered/pointcloud` (`:25`) | — | — | 与 nav_common 的感知栈一致 |

结论：**三份 yaml 的数值本身没发现会直接改变正常场地分割结果的错误**；问题集中在
"注释/文档声称的机制与运行期实际机制不一致"（D3）、实车外参（D5）、消费端高度带（§7）。

---

## 4. 已修复（本轮改动，全部在任务 write scope 内）

1. **bin 索引越界防护**：新增 `computeBinIndex()`
   （`ground_segmentation.h:14-42`），`insertionThread` 改用并 clamp
   （`ground_segmentation.cc:245-292`）；in-range 点结果与旧实现逐点一致。
2. **参数输入校验**：节点在构造段器前校验 `n_bins/n_segments/n_threads/
   line_search_angle/sensor_height`（node `:285-305`），非法配置启动即失败。
3. **NaN/Inf 不污染分割**：`Bin::addPoint` 拒收非有限 d/z
   （`bin.cc:16-38`）；node 输出前剔除非有限点（node `:527-529`）。
4. **构建可复现**：ros 包 CMakeLists 显式 `find_package(PCL REQUIRED COMPONENTS
   common)` 并链接/加 include（`linefit_ground_segmentation_ros/CMakeLists.txt:10-25`）。
   旧 build 目录能编过是继承了环境 include 路径，清 build 后必然失败。
5. **回归测试**：新增 `test/test_ground_segmentation_robustness.cpp`（6 条）并注册
   到 `linefit_ground_segmentation/CMakeLists.txt:17-32`：
   覆盖：bin 索引 clamp（含真实浮点病态用例，`:73-91`）、随机 10 万点永不出界 +
   NaN/Inf 返回 -1（`:93-117`）、Bin 拒收 NaN/Inf（`:119-148`）、混入 NaN/Inf 后
   **合法点 label 逐点不变**（`:150-189`，"不改正常结果"的直接契约）、空点云/
   r_min/r_max 边界点安全（`:191-234`）、`line_search_angle=0.8` 跨 10° 空扇区而
   置 0 失败（`:236-270`）。`test_step_detection.cpp` 原有 5 条全部通过，正常场地
   分割结果不变。

---

## 5. 待决策（会改变行为或需实测/跨包，本轮不改）

| ID | 事项 | 为什么不能本轮改 | 建议 |
| :- | :- | :- | :- |
| D1 | `line_search_angle=0.8` | 改小可能让稀疏/遮挡帧的地面变障碍（改变分割结果），需实车/仿真对比 | 用同一帧回放对比 0.2/0.4/0.8 的 obstacle 点云与 `segment_ms`，确认是否为跨空扇区所必需 |
| D2 | `r_min/r_max` 外点与 NaN 点是否入 `/segmentation/obstacle` | 改变话题内容，影响建图（slam_toolbox 直接吃该话题）与 bag 体积；需确认消费端无依赖 | 建议库内引入第 3 类 label（如 `2=忽略`）或 node 按 `r_min/r_max` 过滤；先加统计日志观察实际占比 |
| D3 | `sensor_height_frame: ""` 与"TF/measurement_params 是唯一真值"注释冲突 | 打开 TF 解析会改变运行时 sensor_height（当前链路下会解析成 0 并锁兜底），需先修 frame 语义 | 二选一：①保留 yaml 兜底，删除/改写"来自 TF"的注释；②把点云 frame 概念（odom vs livox_frame）理顺后再启用 |
| D4 | 严格 stamp TF 查询失败即丢帧 | 加 latest 兜底会引入时间偏移，需评估定位精度 | 先统计 `cannot transform processing frame` WARN 频率；若 >0 再做"严格失败 → latest + 一次性 WARN" |
| D5 | 实车外参 z=0.0 与注释 0.49 / `sensor_height: 0.49` 矛盾 | 禁止改外参/URDF；且会影响导航高度带 | 交外参负责人：卷尺实测后统一 `measurement_params_real.yaml` 与 xacro default（body 审计 §1.3） |
| D6 | `latch` 是未声明死参数 | 加 latch 支持属于新功能 | 删除该行，或节点声明并实现 transient_local 发布（当前无需求，建议删除） |

---

## 6. 建议（不改代码的低优先级改进）

- S1：`verticalDistanceToLine` 重叠区间取"最近距离"而非"最后一条线"
  （`segment.cc:112-124`），并把 `dist != -1` 改成 `dist >= 0`
  （`ground_segmentation.cc:206`）。
- S2：看门狗全局状态改成节点实例成员、publish 前订阅数门控增加
  `always_process` 参数（node `:415-421`），便于离线回放/自测。
- S3：重测 `RMUL.pcd` 地面主峰（当前仓库数据与注释不一致），并把
  `base_to_ground_z` 的用途写清楚（当前配置下不生效）。

---

## 7. 与 costmap 的接口（20cm 高台消费端影响，引用 body 审计 §6）

segmentation 本身在配置参数下**能**把 20cm 台面判成障碍（§1.6，测试锁死），
但 `/segmentation/obstacle` 进入 costmap 后，高度带判据
`height_to_robo = point.z - robo_z` 的 `robo_z` 来自 `base_link_fake`
（与 `base_link` 平移重合 = 雷达平面），并没有抵消安装高度 h，于是
`height_to_robo = s - h`：

- 仿真 h=0.175：20cm 台面 → +0.025，低于 local 门 0.05 / global 门 0.1 → **静默消失**；
- 实车 h=0.49：20cm 台面 → −0.29 → **静默消失**；
- 一般化：地面以上低于 `h + obstacle_z_min_to_robo`（实车 ~0.54m、仿真 ~0.225m）
  的障碍全部不可见。

完整代码链、代数推导、为什么现有 costmap 测试没抓到、以及建议的修法见
**body 审计 §6（`docs/tdt-compare/audit-body.md` 的 §6.1–§6.4）**，本报告不重复。
对本审计的含义：**不要试图用调 segmentation 参数去补这个洞**——把
`max_start_height` 调小只会让分割更激进，`obstacle_z_min_to_robo` 的下界由
消费端坐标系语义决定；两处必须一起看（`navigation2.yaml:112-120,279-282`）。

---

## 8. 验证记录

```text
colcon build --symlink-install --packages-select \
  linefit_ground_segmentation linefit_ground_segmentation_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release            → Summary: 2 packages finished

# 本机 LD_LIBRARY_PATH 带 /opt/MVS/lib/64 会 shadow libusb，涉及 PCL IO 需预加载
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
colcon test --packages-select linefit_ground_segmentation \
  linefit_ground_segmentation_ros --event-handlers console_direct+
→ linefit_ground_segmentation: test_step_detection + 新增
  test_ground_segmentation_robustness 全 Passed（100% tests passed, 0 failed out of 2）
→ linefit_ground_segmentation_ros: test_test_sensor_height_from_tf.py Passed
  （launch test, 6.66s；100% passed, 0 failed out of 1）
```

已知环境坑：launch test 会拉起链接 PCL 的节点，不预加载系统 libusb 时可能 dlopen 失败（机器环境问题，非本包缺陷）。

---

## 9. 未做 / 边界

未改任何 URDF、外参、真实配置值或 `cpp_lidar_filter`，未 commit；未用真实
rosbag/实车数据回放（所有结论来自代码推导 + 合成点云 gtest，`line_search_angle`、
`sensor_height` 最终取值需实测）；未做 sanitizer/valgrind（越界已有单测覆盖）。
