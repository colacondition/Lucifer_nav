# 05 · 审计：去车身过滤 + 帧/z 链一致性

> 范围：`src/perception/cpp_lidar_filter`（filter_node + single_pass_filter）及其与 TF / 地面分割 /
> 代价地图的接口。
> 快照：2026-10-23 02:14 CST，HEAD `9eabcf5`（工作区 dirty；**`fake_vel_transform` 正在被另一工作流
> 修改**，涉及该文件处只引用语义、行号可能漂移）。
> 约束：不改 URDF / `measurement_params_*` 数值、不改导航代码、不 `git commit`；高风险项只报告。
> 本文引用均为仓库相对路径 `文件:行号`，可直接核对。
>
> **决策更新（2026-10-23，用户）**：去车身 = **去掉云台上其他结构**（雷达本身也装在云台上）。
> 因此 `base_link`=雷达/云台位姿是正确语义，雷达系 box 就是"云台自车结构盒"，**不按底盘盒处理**；
> 本文 §3.3 中"意图 B（剪底盘）"的备选不再适用。本轮已把 box 参数显式写入
> `config/common/lidar_filter.yaml`、口径统一为"云台自车结构"，数值保持 `x±0.4/y±0.3/z[-0.1,0.6]`。
> 另：云台只有 yaw 旋转、没有俯仰；地面分割的 base_link 处理链不需要 pitch 补偿。
> costmap 高度带（§6）仍是独立的待决策项。

## 0. 结论摘要（先看这张表）

| # | 发现 | 等级 | 结论 | 本轮动作 |
| :- | :- | :- | :- | :- |
| 1 | 车身裁剪盒在 `base_link` 下，而运行时 `base_link` 实际是**雷达位姿**；`z=[-0.1,0.6]` 是相对雷达平面，不是相对底盘/地面 | 🔴 高（语义） | 盒子实际覆盖地面高度带 `[h-0.1, h+0.6]`：抓到雷达上方 0.6m、下方仅 0.1m 的自车结构；**底盘下沿基本漏掉**。是否按 `-sensor_height` 偏移取决于盒子的真实意图（见 §3.3），两种意图的取值公式不同 | **只报告**，未改 box 默认值/外参 |
| 2 | costmap 高度带 `height_to_robo = point.z - robo_z` 的 `robo_z` 来自 `base_link_fake`，而 `base_link_fake` 相对 `base_link` 平移恒为 0 ⇒ `robo_z` 是**雷达平面**不是底盘，做差没有抵消安装高度 | 🔴 高（功能） | 20cm 高台在**实车和仿真都会静默消失**；更一般地，低于 `h + obstacle_z_min_to_robo` 的障碍全部不可见（实车 h≈0.49 ⇒ <0.54m，仿真 h=0.175 ⇒ <0.225m）。现有回归测试用的是"底盘系 base_link_fake"的 TF 假设，生产 TF 不满足 | **只报告**，未改导航行为/参数 |
| 3 | `filterSinglePass` 判定逻辑（frame/range/box/voxel/NaN）整体正确；确认的 bug：① 合法的 padded `row_step` 云被整帧丢弃；② 空云被当解析错误；③ 节点 marker 只换 `header.frame_id` 不换位姿，`marker_frame != navigation_frame` 时错位 | 🟡 低 | ① ② ③ 全部已修 + 补 gtest（8→16） | **已修** |
| 4 | 地面分割的 `sensor_height` "从 TF 查单一真值"设计在生产链路上**没有生效**：`sensor_height_frame: ""` 关闭了分辨率；即使打开，点云 frame 是 `odom` 而不是 `livox_frame`，查 `base_link←odom` 的 z≈0，会判非法并锁回 yaml 兜底 | 🟠 中（链路） | 当前实际生效的 h 是 yaml（实车 0.49 / 仿真 0.175）；"measurement_params 是唯一来源"的注释与实现不符 | **只报告**（不在本任务 write scope） |

一句话：**"去车身"这层的坐标系语义是雷达系，而下游代价地图的高度带是按底盘系写的；两处都自称 `base_link`，
实际相差一个雷达安装高度 h。** 过滤本身没有算错，错的是"base_link 到底在哪"。

---

## 1. 坐标系事实链（证据）

### 1.1 生产 TF 树里 `base_link` 在雷达上

- `src/localization/small_glim/config/params_node.yaml:38`
  `base_frame_id: "base_link" # odometry child frame; carries the lidar pose (super_lio convention)`
- `src/bringup/config/common/small_glim.yaml:8,23-27`：运行期 `enable_tf_publish: true`、
  `base_frame_id: "base_link"`、`perception_cloud_frame_id: "odom"`、`cloud_frame_id: "odom"`。
- `src/localization/small_glim/src/small_glim_node.cpp:343-347`：
  `T_odom_lidar = T_world_imu * T_lidar_imu.inverse()`，注释明确 "the `base_link` odometry actually
  carries the lidar pose, keeping the odom origin z anchored at the power-on lidar plane"。
- 同文件 `:374-388`：发布的 TF 就是 `odometry_frame_id -> base_frame_id`（`odom -> base_link`），
  变换值 = `T_odom_lidar`。

⇒ **`odom` 系 z=0 是开机时的雷达平面；`base_link` 原点在雷达上，不在底盘、更不在地面。**
（这与 `RMUL.pcd` 地面主峰在 −0.17/−0.175 的实测一致，见
`src/bringup/config/common/navigation2.yaml:108-120` 的注释。）

### 1.2 `base_link_fake` 与 `base_link` 只差一个 yaw

- `src/control/fake_vel_transform/src/fake_vel_transform.cpp`（`publishTransform()`）：
  `header.frame_id = "base_link"`、`child_frame_id = "base_link_fake"`，
  `translation.x/y/z = 0.0`，只有旋转（路径切向 yaw）。
- 因此 **`T_base_link_fake = T_base_link`（位置完全重合）**。

### 1.3 雷达安装高度 h 的"三个来源"互相矛盾

| 来源 | 实车 | 仿真 | 说明 |
| :- | :- | :- | :- |
| `config/*/measurement_params_*.yaml`（URDF `base_link->livox_frame` 外参） | `z=0.0`（`reality/measurement_params_real.yaml:11-18`，但注释文字写"现按 xacro default 取 0.49"） | `z=0.175`（`simulation/measurement_params_sim.yaml:7-9`） | 文件注释把这里说成"雷达离地高度唯一来源" |
| URDF xacro default | `0.49`（`urdf/sentry_robot_real.xacro:20`） | `0.175`（`urdf/sentry_robot_sim.xacro:27`） | 只在手工 xacro 时生效，launch 会覆盖 |
| `segmentation.yaml`（当前**实际生效**的 sensor_height） | `0.49`（`config/common/segmentation.yaml:11` + `reality/segmentation.yaml:4`） | `0.175`（`config/simulation/segmentation.yaml:4`） | 因为 `sensor_height_frame: ""`（common:12）关闭了 TF 分辨率，见 §1.4 |

### 1.4 地面分割的 sensor_height 解析在生产链路不成立

`src/perception/linefit_ground_segmentation_ros2/.../src/ground_segmentation_node.cc`：

- `:253-265` 注释与参数：`sensor_height` 只是兜底；真值 = `z(sensor_height_frame -> 点云 frame) + base_to_ground_z`。
- `:331-360` `resolveSensorHeight(cloud_frame)`：查 `sensor_height_frame_("base_link") -> cloud_frame`
  的 z。生产链路里 `cloud_frame = "odom"`（因为 lidar_filter 输出 header 保持输入帧，见 §2），
  于是它查到的是 `base_link←odom` 的 z = **−雷达在 odom 中的 z ≈ 0**，`resolved = 0 + 0 = 0`，
  判 `<=0` 非法 → 打 ERROR 并锁定 yaml 兜底。
- 另外运行期 common 配置把 `sensor_height_frame` 设成空串（`common/segmentation.yaml:12`），
  `sensor_height_resolved_ = sensor_height_frame_.empty()` 直接为真（node `:311`），**TF 分辨率整段被跳过**。
- 唯一能自洽的链路是"点云 frame = `livox_frame`"（集成测试
  `linefit_ground_segmentation_ros/test/test_sensor_height_from_tf.py:97-117` 正是这么搭的），
  但生产环境点云 frame 是 `odom`。

⇒ 结论：**"h 只有一个来源"的设计没有真正上线**；当前 h 由 segmentation yaml 决定
（实车 0.49 / 仿真 0.175）。真正的单一来源需要么让 lidar_filter 输出 `livox_frame`（会牵扯下游），
要么把 `sensor_height_frame`/点云 frame 概念理顺。属于链路级 TODO，本任务只报告。

---

## 2. 当前帧链一张图

```text
small_glim:
  /small_glim/deskewed_cloud   frame=odom, z=0 在开机雷达平面
  TF odom -> base_link          base_link 原点 = 雷达位姿            (params_node.yaml:38, small_glim_node.cpp:343-388)

cpp_lidar_filter (config/common/lidar_filter.yaml:4-7):
  input  = /small_glim/deskewed_cloud (odom)
  lookup base_link <- odom     (msg stamp, 无超时；失败整帧丢弃)
  判定：navigation_frame = base_link 下的 半径 + 车身 box
  output = /livox/lidar_filtered/pointcloud, header=odom, 点坐标仍是 odom

linefit ground_segmentation (config/common/segmentation.yaml:21-22):
  processing_frame = base_link → 把 odom 云变换到 base_link（雷达系）
  sensor_height fallback = 0.49(real)/0.175(sim)；地面被视为 z = −h
  output /segmentation/obstacle  header=base_link（雷达系）

local/global costmap (navigation2.yaml:94/253, 119-120/281-282):
  robot_base_frame = base_link_fake
  robo_z = T_map_base_link_fake.z   ← 因 fake 平移=0，等于 T_map_base_link.z（雷达平面）
  height_to_robo = point.z(map) - robo_z = 点相对雷达平面的高度
```

---

## 3. 车身裁剪盒到底裁了什么

### 3.1 参数与判定位置

`src/perception/cpp_lidar_filter/src/filter_node.cpp:40-53`（默认值，两份 yaml 均未覆盖 box）：

```text
min_x=-0.4  max_x=0.4   min_y=-0.3  max_y=0.3   min_z=-0.1  max_z=0.6   negative=true
```

判定在 `single_pass_filter.cpp:147-194`：先把点变换到 `navigation_frame`（运行期 = `base_link`），
再做半径/box 判定；输出点坐标仍保持输入系（`odom`）。box 是**闭区间**（`>=`/`<=`）。
⇒ box 的 z 语义 = **相对 `base_link` 原点的 z，而 `base_link` 原点在雷达**。

### 3.2 几何换算：地面高度带

设雷达安装高度 h（地面到雷达平面）。点在雷达系的 z = 地面高度 − h，于是盒子在地面系覆盖：

```text
z_ground ∈ [min_z + h, max_z + h] = [h - 0.1, h + 0.6]
```

| 环境 | h | 盒子覆盖的地面高度带 | 盒子横向范围 |
| :- | :- | :- | :- |
| 实车（segmentation 兜底） | 0.49 | **[0.39 m, 1.09 m]** | x±0.4、y±0.3（对角半径 ≈0.5m） |
| 仿真（segmentation） | 0.175 | **[0.075 m, 0.775 m]** | 同上 |

结合 URDF 的底盘几何（`urdf/sentry_robot_real.xacro:29-46`：圆柱 r=0.25、高 0.1，按"base_link 在地面"画成 z=0..0.1）
可以判断：

- **实车**：盒子下沿在离地 0.39m，底盘 0~0.39m 的自车点**不在盒子里**（若雷达能扫到，就会漏到下游）；
  盒子主要抓的是"雷达上方 0.6m 内的结构"（云台/自身上部）。
- **仿真**：盒子下沿只离地 7.5cm，会连底盘上半段一起抓，但最下面的 7.5cm 仍漏。
- 横向：`x±0.4, y±0.3` 能覆盖 r=0.25 的圆柱，但覆盖不到更外侧的轮组/结构（轮组在 URDF 里半径 0.06 且几乎贴地，
  雷达下视 FOV 也未必看得到——这条无法仅凭仓库代码定量）。

### 3.3 是否应按 `-sensor_height` 偏移？——先定义意图

同一组数字，两种意图对应两种不同公式，当前实现只在"意图 = 雷达系"时自洽：

| 意图 | 期望的盒子语义 | 应填参数（`base_link`=雷达系） | 与当前值的关系 |
| :- | :- | :- | :- |
| A. 剪掉**雷达周边/上方自车结构**（云台、支架、线缆） | 雷达系：下沿略低于雷达、上沿高于雷达 | 就是现在的 `[-0.1, 0.6]` | ✅ 语义自洽；但注释/命名"去车身"容易让人以为是底盘，且底盘下沿会漏 |
| B. 剪掉**底盘**（按地面/底盘系，`[-0.1, 0.6]` 相对地面） | 地面系 → 雷达系要整体下移 h | `[min_z - h, max_z - h]` = 实车 **`[-0.59, 0.11]`**、仿真 **`[-0.275, 0.425]`** | ❌ 当前少偏移了 h |

结论（按证据保守表述）：

1. **当前盒子不是底盘系盒子**。如果产品意图是"去掉车身"= 底盘/车轮/防护结构，则应把 `min_z/max_z`
   改成 `min_z - h` / `max_z - h`（h 取 segmentation 的实际值）；但这样会同时**不再裁剪雷达上方 0.11m 以上的云台结构**，
   是否可接受取决于自车几何——仓库里没有云台的 URDF/实测点云，无法离线定论。
2. 如果产品意图本来就是"雷达上方云台"，则当前 z 值可以保留，但**必须把注释改成"相对雷达平面"**，
   并明确"底盘下沿不保证被裁剪"（当前注释只写"车身 box，navigation 系"，会被误读成地面系）。
3. **高风险，不在本任务修改**：改 box 会改变下游看到哪些障碍（可能把真障碍裁掉，或把自车点放进去），
   还可能踩到 `negative` 语义反转。上线前需要实车量一次 h、采一帧含自车/云台/底盘的完整点云，
   再做一次离线重放验证。本轮只把语义写进启动日志（`filter_node.cpp:90-108`，见 §7）。

---

## 4. `filterSinglePass` 逐项审计

| 项 | 现状 | 结论 | 证据 |
| :- | :- | :- | :- |
| 判定坐标系 | 先 `navigation_from_cloud` 变换，半径/box 都在 navigation 系；输出仍写输入系坐标 | ✅ 正确（旧五连实现里 CropBox 直接作用在输入系，实际是错的；单遍版已修） | `single_pass_filter.cpp:147-194`；旧实现见 `git show b1d1e87:src/perception/cpp_lidar_filter/src/filter_node.cpp` |
| range | `abs(tx)>range || abs(ty)>range || tx²+ty²>range²` | ✅ 圆域裁剪；前两项是冗余但一致的快速剪枝，不算 bug | `:183-185` |
| box | 3D 闭区间；`inside_box == negative` 决定去留 | ✅ 正确，边界含等问题见 §7 测试 | `:188-194` |
| NaN / ±Inf | x/y/z 任一分量非有限即跳过；变换后再次检查 | ✅ 正确 | `:167-172,180` |
| intensity | 只接受 `count==1 && FLOAT32` 且 offset 合法；值非有限填 0；缺失填 0 | ✅ 正确 | `:99-110,210-215` |
| 字段布局 | 要求 x/y/z `count==1`、FLOAT32/FLOAT64、`offset+size<=point_step`；拒绝 big-endian | ✅ 正确 | `:85-98` |
| `row_step` | 旧：要求 `row_step == width*point_step`，否则整帧丢 | ❌ **已修**：接受 `row_step >= packed`（行内 padding）且允许最后一行无 padding | `:112-130` |
| 空点云 | 旧：`width==0` 当非法返回 false → 节点打 ERROR | ❌ **已修**：布局合法时返回 true + 空输出 | `:116-130` |
| 数据越界 | 数据区长度检查 + 每点越界检查，失败清空返回 false | ✅ 正确（不会 OOB） | `:112-130,156-161` |
| voxel key | 用**输入系**(odom)坐标量化、每格留首点；64 位乘法 XOR 当哈希 | ✅ 设计取舍（头文件已注明）；理论上有哈希碰撞（n≈1e5 时概率 ~1e-9，可忽略） | `:196-204`；`single_pass_filter.hpp:22,25-33` |
| `seen` 复用 | `thread_local unordered_set`，每次 clear+reserve | ✅ 单回调线程下安全；不跨帧泄漏 | `:141-146` |
| 输出 header | 节点用 `output->header = msg->header`（保持输入 frame） | ✅ 与"输出坐标在输入系"一致 | `filter_node.cpp:224-226` |
| 点数量上限 | `point_count > INT_MAX` 拒绝 | ⚠️ 无害的保守限制（现实点云不会到） | `:132-135` |

结论：**过滤算法本身没有发现会改变 box/range 语义的正确性 bug**。已修的三处都属于"合法输入被误拒/误报 + 可视化错位"。

---

## 5. lidar_filter 的 TF 使用审计

`filter_node.cpp:190-226`：

- 查询 `navigation_frame <- msg.header.frame_id`，时间戳用 **msg 的 stamp**（不是 now）✅；
- **没有 timeout**（`lookupTransform` 默认 0，立即返回）；失败时整帧丢弃并节流 WARN ✅（安全侧）；
- 开机头几帧 TF 未就绪 → 丢帧；`odom->base_link` 由 small_glim 以 100Hz 发布（`common/small_glim.yaml:9`），
  10Hz 点云按 stamp 查询通常可用；
- 若上下游时钟不一致（use_sim_time 配置错误），会持续丢帧——目前 launch 已统一设置，属可接受风险；
- 失败处理是**整帧丢弃**而不是用最近 TF 兜底：对"离车身近的点被误裁/漏裁"来说丢帧更安全，✅ 保留。

一个次级观察：输入 `/small_glim/deskewed_cloud` 头帧是 `odom`，而 `navigation_frame=base_link` 是**运动系**。
`odom->base_link` 是唯一把两者连起来的边；若 small_glim 的 TF 抖动/丢边，filter 会整帧停发（上游会表现为障碍消失，
而不是错误障碍），这一点在审计中未发现额外 bug。

---

## 6. 与 costmap 的 z 带接口：20cm 高台推导

### 6.1 代码链

| 环节 | 证据 |
| :- | :- |
| costmap 用 `robot_base_frame = base_link_fake` 查位姿 | `local_costmap_node.cpp:200`、`global_costmap_node.cpp:133`；`navigation2.yaml:251`（local）/ `:87`（global） |
| `robo_z = transform.translation.z` | local `:1219-1222`；global `:492-499` |
| 高度带判定 `height_to_robo = point.z - robo_z` | local `:915-918`；global `:364-366` |
| 高度带取值 local `0.05` / global `0.1` | `navigation2.yaml:281-282`（local）、`:119-120`（global）；代码默认值 local 0.05 / global 0.1 |
| 点云来自 `/segmentation/obstacle`（header=base_link） | `navigation2.yaml:94/253`；`segmentation.yaml:26` |
| obstacle 点坐标在 base_link（雷达系） | `ground_segmentation_node.cc:402-409,430,513-520`（`output_frame = processing_frame` 并写回 header） |

### 6.2 代数推导

设地面高度 `z_ground`（map 系）、雷达平面高度 `z_lidar = z_ground + h`：

```text
robo_z   = T_map_base_link_fake.z = T_map_base_link.z = z_lidar        （fake 平移为 0，§1.2）
point.z(map) = z_lidar + z_base_link                                  （点云在雷达系的 z + 平移）
height_to_robo = point.z - robo_z = z_base_link                      （安装高度被"保留"而不是抵消）
```

即：**`height_to_robo` 是"点相对雷达平面"的高度**，而不是注释里写的"相对底盘"。对地面上的一个高 `s` 的台面：

```text
height_to_robo = s - h
可见条件:      obstacle_z_min_to_robo <= s - h <= obstacle_z_max_to_robo
等价地面高度带: s ∈ [h + min, h + max]
```

| 环境 | h | 20cm 台面 `height_to_robo` | local 门(0.05) | global 门(0.1) | 结论 |
| :- | :- | :- | :- | :- | :- |
| 仿真 | 0.175 | **+0.025** | <0.05 → 丢 | <0.1 → 丢 | 🚫 **静默消失** |
| 实车 | 0.49（segmentation 兜底） | **−0.29** | <0.05 → 丢 | <0.1 → 丢 | 🚫 **静默消失** |

更一般的可见性：**地面高度低于 `h + obstacle_z_min_to_robo` 的障碍都不可见** —— 实车约 `0.54m`、仿真约 `0.225m`
（仿真低于 22.5cm 的台面/矮箱/蹲伏目标全丢）。再叠加地面分割 `max_start_height=0.1`
（`segmentation.yaml:19`，语义 = 能识别的最小台阶高度，见 `linefit.../test/test_step_detection.cpp:1-31`），
台阶的端到端可见条件变成 `s > max(0.1, h + 0.05)`，实车约 `0.54m`。

### 6.3 为什么现有测试没抓到

`src/navigation/navigation2/test/test_costmap_height_frame.py:126-132`（常量在 `:36-51`）搭的 TF 是：

```text
map -> base_link_fake(z = ROBOT_MAP_Z = -0.30)   # 把 base_link_fake 当成"底盘"
base_link_fake -> livox_frame(z = 0.175)          # 传感器在底盘上方
```

点云 frame 是 `livox_frame`，于是 `robo_z` 确实是底盘 z，`height = 0.2` 通过。
但**生产 TF 不是这棵树**：生产是 `odom -> base_link`（= 雷达位姿）+ `base_link -> base_link_fake`（平移 0），
没有任何节点把 `base_link_fake` 放到地面/底盘高度。也就是说：**测试锁住了"意图"，但没有锁住"生产 TF 是否满足这个意图"。**
新增/修改高度带逻辑时，请同时加一条"生产语义"的 TF（`base_link_fake` 与 `base_link` 平移 0）作为对照，
否则这个 bug 还会回来。

### 6.4 建议（只报告，本轮不改导航）

按"改动量 / 风险"排序：

1. **最小、可回退**：给 `local/global costmap` 增加 `robo_z_reference_offset`（默认 `0.0` = 保持当前行为），
   部署时按环境填 `-h`（实车 `-0.49`、仿真 `-0.175`）。⚠️ 必须默认关闭并在参数注释写明
   "把参考面从雷达平面移到地面，会让地面分割残留点（负高度）被 0.05 下界挡住——这正是保留正下界的原因"，
   同时改 `test_costmap_height_frame.py` 的 TF 假设。
2. **语义更干净**：让 `fake_vel_transform` 发布 `base_link -> base_link_fake` 时带一个可配置的 z 偏移
   （默认 0，新增参数），由该节点负责"底盘高度"概念；costmap 不用改。风险：`base_link_fake` 是导航控制链的公共坐标系，
   影响 MPC / 决策 / 航点执行，**不建议在无人验证时改**。
3. **直接改 `obstacle_z_min_to_robo` 为负数**（如 `0.05 - h`）：一行配置即可验证，但负下界会让地面分割残留、
   坡道/地形点的 z 抖动直接变成障碍（YAML 注释里"负值会把车围死"的顾虑就是它），**只适合临时实验**。
4. 无论选哪条，都需要：仿真里放 10/15/20/30cm 台并检查 `/local_costmap/costmap_raw`；
   实车用卷尺/纸箱做同样测试。**本任务没有做这些验证，也未改任何 nav 参数**。

---

## 7. 本轮低风险修复（已改，均在 `src/perception/cpp_lidar_filter/` 内）

| 文件 | 改动 | 理由 |
| :- | :- | :- |
| `src/single_pass_filter.cpp` | `row_step` 允许 `>= width*point_step` 的行 padding，允许最后一行省略 padding；空点云返回 true + 空输出 | 合法 PointCloud2 被整帧误丢/误报 ERROR；见 §4 表 |
| `src/filter_node.cpp` | marker 位姿从 `navigation_frame` 显式变换到 `marker_frame`（TF 查询失败回退 `navigation_frame` + 节流 WARN）；两者相同时零开销 | 旧实现只改 `header.frame_id`、位姿仍按 navigation 系数值解释，`marker_frame != navigation_frame` 时 marker 错位 |
| `src/filter_node.cpp` | 启动时 INFO 打印 box 的坐标系语义（雷达系，`[min_z-h, max_z-h]` 地面带）；错误日志措辞同步 `row_step` 放宽 | 让 §3 的语义坑在日志里可见；不改任何默认值 |
| `src/.../single_pass_filter.hpp` | 注释更新（padding/空云/FLOAT64 语义） | 与实现一致 |
| `src/.../test/test_single_pass_filter.cpp` | 8 → 16 个用例 | 见下 |

新增 gtest（全部通过）：

- `AcceptsPaddedRowStepOrganizedCloud`、`AcceptsLastRowWithoutPadding`、`RejectsTruncatedOrganizedData`
  —— 锁 `row_step` 边界，并确保截断数据被拒绝（不越界读）；
- `EmptyCloudIsValidAndProducesEmptyOutput` —— 空帧不再误报；
- `AcceptsFloat64XyzFields` —— FLOAT64 布局；
- `RotationAppliedBeforeBoxJudgement` —— 证明半径/box 判定发生在变换到 navigation 系之后，输出坐标仍是输入系；
- `BoxBoundaryIsInclusive` —— box 闭区间（用二进制可精确表示的边界值，避开 float→double 1ulp 问题）；
- `DropsNonFinitePointsAndSanitizesIntensity` —— ±Inf/NaN 点丢弃、NaN intensity 归 0。

节点级 smoke（本轮临时脚本 `/tmp/lidar_filter_smoke.py`，未入库）：起 `lidar_filter_node`，
发静态 `odom->base_link=(1,0,0)`，点云 frame=odom；验证 ① 输出只保留 box 外点且坐标仍是 odom；
② `marker_frame=odom` 时 marker header=odom、位置 = box 中心经 TF 变换后的 (1.0, 0, 0.25)。**PASS**。

---

## 8. 验证记录

```text
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cpp_lidar_filter --cmake-args -DCMAKE_BUILD_TYPE=Release
  → Finished <<< cpp_lidar_filter（0 warning）
colcon test --packages-select cpp_lidar_filter --event-handlers console_direct+
  → 1/1 test passed；gtest 16/16 passed
```

构建/测试均不需要 `LD_PRELOAD`（该包只链 `pcl_common`，不触发本机 MVS libusb 问题；若在本机跑
带 `pcl_io` 的可执行，仍需 `LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0`，属既有环境问题）。

---

## 9. 未做 / 风险 / 后续

- ❌ **未改**：`min_z/max_z` 等 box 默认值、URDF、`measurement_params_*`、导航参数/代码。§3、§6 的结论只报告。
- ❌ 未做实车/仿真场地验证（没有实车、没有场地 bag）；§6 的 20cm 结论是**静态代码推导**，
  但每一步都有行号可核对；上线前请按 §6.4 的验证清单实测。
- ⚠️ `fake_vel_transform` 在本审计期间正被另一工作流修改（新增 `velocity_transform.hpp` 等），
  本文引用该文件只依赖语义（`base_link_fake` 平移=0）；若该工作流给 fake 加了 z 偏移，§6 的结论需要重算。
- ⚠️ §1.4 的 sensor_height 分辨率问题不在本任务 write scope，建议单独立项：要么让
  `sensor_height_frame` 指向真正的雷达 frame，要么删掉这段"从 TF 取真值"的死代码/注释，
  避免下一个人以为它生效了。
