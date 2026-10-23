# 05 · 审计总报告：地面分割 / 去车身 / fake_vel_transform（2026-10-23）

> 范围：按用户要求，审计三块非 LIO/导航主链的算法与接口，并做**低风险修复 + 回归测试**。
> 分册：[audit-ground.md](audit-ground.md)（350 行）· [audit-body.md](audit-body.md)（325 行）· [audit-fakevel.md](audit-fakevel.md)（358 行）。
> 约束：LIO 与导航主链算法未改；未改 URDF / `measurement_params_*` 数值 / 真实场地配置值；未 git commit。
> 验证：全部为编译 + 单测 + headless/离线验证；**未上实车、未跑 Gazebo 场地**。

---

## 0. 一句话结论

三块里，**算法本身没有大错**，问题集中在"坐标系语义"和"参数/配置与实现脱节"：

1. **最严重的问题不是分割算法，而是分割消费端**：`base_link` 运行时是**雷达位姿**，costmap 的高度带 `height_to_robo = point.z - robo_z` 实际是"相对雷达平面"，不是"相对底盘"。结果 **20cm 高台在实车和仿真都会静默消失**，更一般地，地面以上低于 `h + obstacle_z_min_to_robo` 的障碍全部不可见（实车 ≈0.54m、仿真 ≈0.225m）。这是本轮唯一建议**优先决策 + 实测**的项。
2. **地面分割算法本身可用**：`max_start_height=0.1` 的"20cm 台面可见"语义有测试锁死；`line_search_angle=0.8`（≈46°）仍偏大，建议 A/B；`r_min/r_max` 外点/NaN 的 `label=2` 修复已恢复（此前曾因误判为地面分割问题回退）。云台只有 yaw 旋转、没有俯仰，原处理链（整变换到 base_link）本来就正确，审计中提出的"重力对齐-原点"实验模式已撤销。
3. **去车身 = 去掉云台上的其他结构**（用户澄清；雷达本身装在云台上）。因此运行时 `base_link`=雷达/云台位姿是正确语义，盒子 `z=[-0.1,0.6]`（相对雷达平面）就是"云台自车结构盒"，不是底盘盒。本轮把它显式写进 `lidar_filter.yaml`（含坐标语义/调参说明）、改名"云台自车结构"并保留当前数值；底盘下沿不裁是预期行为。
4. **fake_vel_transform 的旋转数学是对的**（用组合恒等式钉死）。审计时发现"小陀螺"链路三级冲突且部署下从未生效；**按用户明确决策：本工程模型就是可平移的运动云台，不需要小陀螺**，因此相关自旋代码/参数/配置已全部删除，节点只做坐标变换（线速度旋转 + 角速度透传）。

---

## 1. 各模块结论

### 1.1 地面分割（linefit）

| 结论 | 严重度 | 处置 |
| :- | :- | :- |
| bin 索引靠浮点不变量兜底，病态参数/舍入可越界写 `segments_[seg][n_bins]`（UB） | 中（潜伏） | **已修**：`computeBinIndex()` clamp + 参数校验；现场配置实际不可达 |
| NaN/Inf 点会污染 bin 的 min-z 并被当障碍发布 | 中 | **已修**：`Bin::addPoint` 拒收非有限、输出前剔除 |
| `r_min/r_max` 之外的点 label=0（障碍）一起发布 | 低-中 | **已恢复修复**：库内 label=2（忽略），ROS 节点不发布；范围外点不再污染 obstacle/建图 |
| `line_search_angle=0.8 rad` 过大：最多跨 45 个 segment、每点最坏 90 次线查询；能跨 10° 空扇区，但会放大远处地形/≤10cm 矮结构误判 | 中（风险/性能） | **待决策**：建议同帧 A/B `0.2/0.4/0.8`，对比 obstacle 与 `segment_ms` |
| `max_start_height=0.1` 语义 = "能识别的最小台阶高度"；20cm 台面在分割内可见 | 低（约定正确） | 维持（已有测试） |
| `sensor_height` 的"TF 单一真值"没有生效 | 中（链路） | **待决策**：运行期 `sensor_height_frame: ""` 关闭了解析；实际用 yaml 兜底（real 0.49 / sim 0.175）；若打开，因点云 frame=odom 会解析出 ≈0 再锁回兜底——注释与实现不符 |
| ros 包 CMake 未显式依赖 PCL，靠环境 include 才编过 | 中（构建） | **已修**：显式 `find_package(PCL)` |

新增/保留测试：`test_step_detection`（5）+ `test_ground_segmentation_robustness`（6：bin clamp/随机 10 万点/NaN 拒收/合法点结果逐点不变/空云与边界/line_search 跨空扇区）；ROS 包 launch test（sensor_height from TF）通过。

### 1.2 去车身（cpp_lidar_filter）

**算法正确性**：frame/range/box/voxel/NaN/intensity/输出 header 逐项审计，**没有发现会改变 box/range 语义的错误**。已修的是三类"合法输入被误拒/误报/可视化错位"：

- `row_step` 行尾 padding 的合法 PointCloud2 被整帧丢弃 → 已接受 `row_step >= packed`（含最后一行省略 padding）；
- 空点云被当解析错误（ERROR 日志）→ 已返回空输出；
- marker 只改 `header.frame_id` 不改位姿 → `marker_frame != navigation_frame` 时错位；已显式 TF 变换（失败回退+节流 WARN）。

测试 8→16 个用例。**未改**：box 默认值/外参/URDF。

**坐标语义（已按用户澄清明确）**：运行期 `base_link` = 雷达/云台位姿，且**雷达本身装在同一云台上**，所以这个 box 就是"云台自车结构盒"（去支架/相机/线缆/云台臂），不是底盘盒。`z=[-0.1,0.6]` 覆盖的地面高度带实车 0.39~1.09m、仿真 0.075~0.775m，符合"雷达周边/上方结构"的意图；**底盘下沿不裁是预期**。横向 `x±0.4 / y±0.3` 覆盖云台结构；换结构后按 RViz 的 `gimbal_self_structure` marker 重标。所有 box 参数已显式写入 `config/common/lidar_filter.yaml`（含语义与调参说明），此前只有代码默认值。

### 1.3 fake_vel_transform

| 结论 | 级别 | 处置 |
| :- | :- | :- |
| 旋转数学正确：`offset=φ-β`，`v_B = R(offset)·v_F`；旧写法 `R(-angle_diff)` 等价，但绕、易被改错 | 确认 | 抽成显式 `R(offset)` 纯函数 + 组合恒等式 gtest + headless 实测 |
| **小陀螺三级语义冲突，部署下从未触发**：MPC 所有路径 `angular.z = default_wz`（部署 0.0）→ 平滑器限幅 ±3 后转发 → fake 触发条件 `|angular| >= deadband` 恒假；一旦调大 default_wz，spin_speed=-6 在平滑器之后注入、绕过 ±3 | **高** | **已按用户决策删除**：模型=运动云台，无小陀螺；spin 代码/参数/配置全部移除 |
| 旧实现非小陀螺时丢弃导航角速度 | 中 | 已修为透传（模型本身也不需要角速度注入） |
| TF 短失败：TF 发布保持旧偏置（对），cmd 回调直接零速（fail-stop，安全） | 中 | 保持 fail-stop，补注释/单测 |
| 时钟回拨时 `MonotonicStampGate` 会停发 TF 直到追平；`TimePointZero` 不判 stale；`serial_driver` 无速度看门狗（NUC 静默时 MCU 可能保持最后一帧） | 中 | **仅报告**，建议电控看门狗 |
| 参数原本无任何 yaml 覆盖，`spin_speed=-6` 是隐藏默认值 | 中 | 已删除该参数；yaml 只留 `path_lookahead_distance` / `yaw_filter_alpha` |

测试：`velocity_transform_test` 10 个 + `monotonic_stamp_gate_test` 1 个；headless 端到端验证旋转/TF yaw/零速/角速度透传。

---

## 2. 跨模块最高优先级发现（建议先决策）

### D0（🔴 高）：costmap 高度带相对雷达平面，20cm 高台静默消失

推导（证据见 [audit-body.md](audit-body.md) §6）：

```text
robo_z = T_map_base_link_fake.z = T_map_base_link.z = 雷达平面 z   （fake 的 base_link→base_link_fake 平移恒 0）
height_to_robo = point.z - robo_z = 点相对雷达平面的高度 = s - h
可见条件：obstacle_z_min_to_robo <= s - h <= obstacle_z_max_to_robo
```

| 环境 | h | 20cm 台面 height | local 门 0.05 | global 门 0.1 |
| :- | --: | --: | :- | :- |
| 仿真 | 0.175 | **+0.025** | 丢 | 丢 |
| 实车 | 0.49 | **−0.29** | 丢 | 丢 |

即"地面以上低于 `h + 0.05`（实车 0.54m / 仿真 0.225m）的障碍全部不可见"。现有 `test_costmap_height_frame.py` 能过，是因为它假设的 TF 把 `base_link_fake` 放在底盘、再在其上放 `livox_frame`；**生产 TF 不是这棵树**（生产 `base_link_fake` 与 `base_link` 平移 0）。

**为什么本轮不改**：这是导航参数/代码，用户明确"nav 没问题"；且改动会引入地面残留点变障碍的风险，必须用 10/15/20/30cm 台面在仿真+实车验证。**可选最小修复**（按风险排序）：

1. 给 local/global costmap 加 `robo_z_reference_offset`（默认 0 = 现行为），部署填 `-h`（实车 -0.49 / 仿真 -0.175）；同步改 `test_costmap_height_frame.py` 的 TF 假设。
2. 让 fake_vel_transform 在发布 `base_link→base_link_fake` 时带可配置 z 偏移（默认 0）——影响 MPC/决策/航点执行，**不建议无人验证时改**。
3. 临时把 `obstacle_z_min_to_robo` 改成 `0.05 - h` 做实验；负下界会把地面残留变障碍（YAML 注释里的"负值会把车围死"），只适合短时验证。

### D1（🟠 中）：`line_search_angle=0.8` 过大

最多跨 45 段（1°/segment），每点最坏 90 次线查询。能桥接 10° 空扇区，但会把远处地形/≤10cm 矮结构按邻域地面线判成地面。建议同帧 A/B `0.2/0.4/0.8`，同时看 `obstacle` 变化与 `segment_ms`（节点已有 >100ms WARN）。

### D2（✅ 已恢复修复）：`r_min/r_max` 外点不再当"障碍"发布

库内 `assignClusterThread` 对范围外/非有限点置 `label=2`（忽略），ROS 节点输出时跳过；正常范围内的 label=0/1 语义不变。`test_ground_segmentation_robustness` 断言锁住"明确范围外必须是 2"。

> 备注：曾按用户要求回退过一次；后确认导航倾斜/“飞起来”是 PCD/LIO 初始姿态问题，与地面分割无关，已恢复全部修复。

### D3（🟠 中）：`sensor_height` "唯一来源"设计没上线

运行期 `sensor_height_frame: ""` 跳过 TF 解析，实际用 yaml 兜底；即便打开，点云 frame=odom 也会解析成 ≈0。且实车 `measurement_params_real.yaml` 注释说"取 0.49"、实际值仍是 `0.0`（仿真 0.175）。三处说法需要统一：要么删掉死代码/注释，要么把 `sensor_height_frame` 指向真正的雷达 frame 并让点云 frame 语义配套。

### D4（✅ 已明确）：去车身 = 去掉云台上的其他结构

用户澄清：雷达也装在云台上，"去车身"是去掉**同云台的其他结构**（支架/相机/线缆/云台臂）。因此 `base_link`=雷达位姿是正确语义，现有 box `z=[-0.1,0.6]`（相对雷达平面）就是正确的"云台自车结构盒"。本轮：
- 把 box 全部参数显式写进 `src/bringup/config/common/lidar_filter.yaml`（此前只有代码默认值），并写清坐标语义与调参方法；
- 代码/日志/README 口径统一为"云台自车结构"（不再叫"车身/底盘"）；
- 数值保持 `x±0.4/y±0.3/z[-0.1,0.6]`（覆盖实车 0.39~1.09m 地面带），换云台结构后按 RViz marker 重标；底盘不裁是预期。

云台只有 yaw 旋转、没有俯仰：把 odom 云完整变换到 `base_link`（雷达/云台系，z 保持重力对齐）本来就是对的。审计期间曾加过"重力对齐-传感器原点"实验模式，因前提（云台/底盘 pitch）不成立已删除；ground/obstacle 输出 header 恢复为 `base_link`。

### D6（✅ 已修，2026-10-23）：建图链 `pointcloud_to_laserscan` 的 min_height 也是雷达系

建图模式 `/scan` 来自 `/segmentation/obstacle` → `pointcloud_to_laserscan`（target_frame 为
`base_link`=雷达位姿）。原 common 的 `min_height: 0.05` 实际是"雷达平面以上 5cm"：实车
（h≈0.49）等于地面以上 0.54m，仿真（h=0.175）等于 0.225m，20cm 高台台面（实车 z≈-0.29、
仿真 z≈+0.025）会被整片滤掉，建图只剩零星噪点。

分割本身是对的：`test_step_detection` 已锁死 20cm 台面被判成障碍（不是地面）。所以这次不是
地面分割的锅，是**转激光时的高度带又用了雷达系**。修法：`reality/` 与 `simulation/` 各加
`pointcloud_to_laserscan.yaml`，把 `min_height` 改成地面以上 5cm（实车 -0.44、仿真 -0.125），
launch 在 common 之后覆盖；输入已是去地面障碍云，负值不会把地面带回来。

### D5（✅ 已按决策关闭）：小陀螺

**用户明确：不需要小陀螺，模型就是可平移的运动云台。** 审计中提出的 `spin_enable` 方案已废弃；`fake_vel_transform` 的 `spin_* / angular_deadband / min_translate_speed_for_spin` 代码与配置全部删除（旧参数只保留声明以兼容旧 yaml，设置会 WARN 并忽略），节点只做线速度旋转 + 角速度透传。`mpc_controller_node.cpp` 里"小陀螺由 default_wz 驱动"的注释也已改为虚拟云台语义；`default_wz` 保持 0.0、仅作通用角速度透传源。

### D7（🟢 低）：其他

- `segmentation.yaml` 的 `latch: false` 节点从未 `declare_parameter("latch")`，是死配置；
- `RMUL.pcd` 的"地面主峰 -0.17"注释无法从当前工作区 PCD 复现（PCD 已被改动）；
- `navigation2/README` 说"电控负责小陀螺"、MPC 注释说 `default_wz` 驱动小陀螺——这些口径已随"删除小陀螺"统一：fake 节点不再注入自旋，`default_wz` 仅作为通用角速度透传源（部署恒 0）。

---

## 3. 本轮实际改动（已修）

| 包 | 文件 | 改动 |
| :- | :- | :- |
| linefit_ground_segmentation | `ground_segmentation.h/.cc`、`bin.cc` | `computeBinIndex` clamp；非法参数拒绝；Bin 拒收 NaN/Inf；范围外/非有限点 label=2（忽略） |
| linefit_ground_segmentation_ros | `ground_segmentation_node.cc` | 启动参数校验；输出剔除非有限点与 label=2 |
| 建图链 | `config/{reality,simulation}/pointcloud_to_laserscan.yaml`、`launch/{real,sim}.launch.py` | min_height 改为地面以上 5cm（实车 -0.44 / 仿真 -0.125），修复 20cm 高台建图消失 |
| linefit_ground_segmentation | `test/test_ground_segmentation_robustness.cpp` | 6 个鲁棒性用例（含 label=2 边界断言） |
| cpp_lidar_filter | `single_pass_filter.cpp`、`filter_node.cpp`、`single_pass_filter.hpp` | row_step padding/空云；marker TF 变换；日志语义；**口径统一为"云台自车结构"** |
| cpp_lidar_filter 配置 | `src/bringup/config/common/lidar_filter.yaml` | 显式写出云台自车结构 box 参数（x/y/z/negative）+ 坐标语义与调参说明 |
| cpp_lidar_filter | `test/test_single_pass_filter.cpp` | 8→16 用例 |
| fake_vel_transform | 删除 `spin_*` 注入与配置；`velocity_transform.hpp` / `velocity_transform_test.cpp` | 只保留 R(offset) 线速度旋转 + 角速度透传；10 个单测（旋转/组合恒等式/恒等偏置/零速/透传/TF 失败） |
| fake_vel_transform | `fake_vel_transform.cpp`、`fake_vel_transform.hpp`、`CMakeLists.txt`、`README.md` | 删除自旋参数/日志/成员；旧参数仅声明兼容 |
| navigation2 params（两份） | `src/bringup/config/common/navigation2.yaml`、`src/navigation/navigation2/params/navigation2.yaml` | `fake_vel_transform:` 段只留 `path_lookahead_distance` / `yaw_filter_alpha`；上一轮 `repair.time_budget_ms: 80.0` 保留 |
| mpc_controller_node.cpp | 一处注释 | "小陀螺由 default_wz 驱动"改为虚拟云台语义（无行为变化） |

---

## 4. 验证

```text
cpp_lidar_filter            build ✅ | gtest 16/16 ✅
fake_vel_transform          build ✅ | gtest 10/10 + gate 1/1 ✅（9/9 ctest）
linefit_ground_segmentation build ✅ | test_step_detection 5/5 + robustness 6/6 ✅
linefit_ground_segmentation_ros build ✅ | launch test ✅（需 LD_PRELOAD 系统 libusb，见下）
全工作区 colcon test-result  → 632 tests, 0 errors, 0 failures（103 skipped，均为原有）
```

> 环境注：本机 `LD_LIBRARY_PATH` 含 `/opt/MVS/lib/64`，其老 libusb 会 shadow 系统库，导致所有 PCL IO/launch test 报
> `undefined symbol: libusb_set_option`。跑测试/工具时加
> `LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0` 即可（既有环境问题，与代码无关）。

---

## 5. 建议的下一步（按优先级）

1. **D0**：先在仿真放 10/15/20/30cm 台子，看 `/local_costmap/costmap_raw` 是否出现；确认后选 §2-D0 方案 1（opt-in 偏移）做 A/B，再上实车。
2. **回放验证**：用真实/仿真 bag 确认 label=2 生效后 obstacle 点数与建图质量无回退（云台只转 yaw，处理链未改）。
3. **D1**：同帧 A/B `line_search_angle`（0.2/0.4/0.8），用 `segment_ms` + obstacle 点数 + 建图质量评估。
4. **D3**：清理 `sensor_height` 死代码或修正链路；统一 `measurement_params_real.yaml` 的注释/数值。
5. **云台 box 重标**：换云台结构后按 RViz `gimbal_self_structure` marker 调整 `lidar_filter.yaml` 的 box，确保盖住自车结构且不吞场地障碍。
