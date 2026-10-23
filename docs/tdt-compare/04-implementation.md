# 04 · 落地实现报告（2026-10-23）

> 范围：按用户要求，落地「定位三项真差距 + P0 三项」中本工作区可独立完成的部分。
> 环境：ROS 2 Humble，本机无实车、无赛场 bag；本报告的验证 = 编译 + 单元测试 + 离线/headless 实测。
> 对比与依据见 [总报告](README.md) / [01 定位](01-localization.md) / [02 导航](02-navigation.md) / [03 工程横评](03-engineering.md)。

## 0. 交付总览

| # | 项 | 原优先级 | 状态 | 主要文件 | 验证 |
| :- | :- | :- | :- | :- | :- |
| 1 | 配准验收升级：3D Schur + 白化 + 退化感知 χ² NIS + 首配完整位姿门 | P0-2 | ✅ 已落地（默认开启，可回退） | `alignment_quality.hpp`、`robot_localization.cpp`、`test/alignment_quality_test.cpp` | build ✅；gtest 15/15 ✅ |
| 2 | Scan Context 描述子重定位 + 离线建库 | 真差距③ / P1-1 | ✅ 已落地（默认关闭，LOST 时启用） | `scan_context.{hpp,cpp}`、`tools/build_scan_context_map.cpp`、`test/scan_context_test.cpp` | build ✅；gtest 6/6 ✅；RMUL 建库 96 条 ✅ |
| 3 | 地图资产：离线清动态残影 + keyframe 落盘开关 | P0-1 | ✅ 已落地（离线工具） | `bringup/tools/clean_map_dynamic.py`、`test_clean_map_dynamic.py`、`launch/{sim,real}.launch.py` | pytest 5/5 ✅；ctest ✅；814k 点实测 ✅ |
| 4 | MINCO 墙钟预算 + 性能基准脚本 | P0-3 | ✅ 已落地（部署配置 80ms） | `smoothing_budget.hpp`、`minco_path_smoother_node.cpp`、`tools/bench_planning.py` | build ✅；gtest 8/8 ✅；headless 降级实测 ✅ |
| 5 | 输出协方差 / 诊断词表 | P1-6 | ⏳ 未做（下一步） | — | — |
| 6 | 非静止初始化 / reset 协调 / 位姿图 / UWB / 动态障碍预测层 | P1-3/P1-2/P2 | ⏳ 未做（依赖实车/bag 或属于换骨架） | — | — |

> 全部改动均**未 git commit**；未动 `src/bringup/PCD`、`map` 真实资产和其他包源码。

---

## 1. fast_location 配准验收升级（P0-2）

### 1.1 新增能力

`alignment_quality.hpp`（header-only，无新依赖）：

- `SixDofObservability` + `analyzeSixDofObservability(H, ratio)`：FastGICP 状态序 `[rot(0:3), trans(3:6)]`，
  平移 Schur `S_tt = H_tt − H_tr H_rr⁻¹ H_rt`、旋转 Schur `S_rr` 同理；LDLT + 对角正则求逆，
  输出升序谱、弱方向、条件数、观测秩（`λ_i > ratio·λ_max`）。
- `chiSquareUpperTail(x, dof)` / `chiSquareThreshold(dof, p)`：自实现 regularized upper incomplete
  gamma（级数 + 连分式），不引 boost；dof 1–6。
- `computeScanToMapNisProjected(...)`：沿用 3 维创新 `(dx,dy,dyaw)`，对 H3 做特征截断伪逆（只保留
  可观测方向）得到观测协方差 R，`S=R+P`，NIS=νᵀS⁻¹ν，dof=可观测秩，返回 χ² 上尾概率。
- `isFullPoseObservable(six, min_translation_rank=2)`：平移秩≥2 且 yaw 可观测。
- 原 `computeScanToMapNis` 与 2D `is_degenerate`/投影逻辑**未改语义**（向后兼容）。

`robot_localization.cpp`：

- 跟踪期 NIS：只要 `enable_nis` 就做（**退化不再跳过**），χ² 判拒；`nis_use_chi2=false` 精确回退旧行为。
- 首配/全局重定位：`require_full_pose_on_init=true` 时要求完整位姿，否则拒绝并给
  `LOST init_partial_pose`（不写 TF，继续等下一帧）。
- NIS 只在跟踪期做：首配/重定位的 `pose_guess` 可能是出生点或漂移后的错锚点，拿它当先验做 NIS
  会拒掉正确候选（`lost_escape` 的教训），这部分交给完整位姿门。

### 1.2 参数（两份 yaml 已同步）

| 参数 | 默认 | 说明 |
| :- | :- | :- |
| `nis_use_chi2` | true | false = 旧行为（首配不做 NIS、退化跳过、固定阈值 12） |
| `nis_rejection_probability` | 1e-3 | χ² 上尾概率门限（越小越严） |
| `nis_observable_eigen_ratio` | 1e-3 | 观测秩相对特征值阈值（H3 与 6DoF Schur 共用） |
| `min_observable_rank` | 2 | 首配/重定位平移秩下限（实际取 max(2, 值)） |
| `require_full_pose_on_init` | true | 首配/重定位完整位姿门 |

### 1.3 验证

- `colcon build --packages-select fast_location` ✅
- `colcon test --packages-select fast_location` → **10/10 通过**；gtest：alignment_quality 15/15、
  global_search 21/21、scan_context 6/6、frame_transforms 5/5。
- 节点用 bringup yaml 启动成功，`ros2 param get` 确认新参数类型/值正确。

### 1.4 风险与回退

1. **首配 NIS 不做** 是有意为之；若现场希望更严格，可后续加独立开关，但不要直接拿上一拍先验判首配。
2. `require_full_pose_on_init=true` 依赖 Hessian 子块可逆；真机若出现 `six_dof.valid=false`，首配会被拒。
   一键回退：`require_full_pose_on_init: false`。
3. yaw 可观测性按旋转 Schur 的 yaw 轴判断（fast_gicp `rot=[roll,pitch,yaw]` 假设），强耦合时是近似。
4. 阈值需用走廊/隧道 bag 标定：观察 `OK nis_reject` 比例、`LOST init_partial_pose` 次数。

---

## 2. Scan Context 重定位（真差距③）

### 2.1 模块

新增 `fast_location/scan_context.{hpp,cpp}`：

- 描述子：`rings×sectors` 极坐标高度图，格子取 `max(z 相对传感器)`，空格 0；`sector key` 粗排 + 全 shift
  有效列余弦距离；yaw 符号约定 `yaw_rad = -shift·2π/sectors`（= `map_from_base` 的 yaw），已由单测钉死。
- `ScanContextDatabase`：`build()`（AABB + `grid_step` 网格，与 global_search 约定一致）、
  `prepare()+addAtPosition()`（离线增量建库）、`save()/load()`（`LSCDB001` versioned 二进制）、
  `query(scan_odom, odom_from_base, top_k)` → `{position, yaw_rad, distance}`。
- 离线工具 `build_scan_context_map`：`--input --output [--step --max-radius --rings --sectors --map-voxel --map-sensor-z --min-range]`。

### 2.2 在线接线（Lead）

`performGlobalSearch()` 候选生成改为：SC 可用时先召回 top-k → 转成与网格搜索同语义的
`T_map_base(yaw, xy) * T_base_odom`；召回为空或少于 `scan_context_min_candidates` 时**自动回退**
原 AABB×360° 网格搜索。GICP 精化、margin、时序校验、跳变门全部复用，不新增第二套验收。

参数（默认关闭）：

```yaml
enable_scan_context: false
scan_context_map_path: ""        # .scdb 绝对路径
scan_context_top_k: 15
scan_context_min_candidates: 2
```

### 2.3 验证

- gtest `scan_context_test` 6/6（合成房间两位置、yaw 0/±90°/45°、save/load、空图/空扫描/NaN 边界）。
- RMUL PCD 实测：`build_scan_context_map --input src/bringup/PCD/RMUL.pcd --output /tmp/RMUL.scdb --step 1.0`
  → 96 条、1.0 MB、0.02 ms/条。
- 节点实测 `enable_scan_context:=true + scan_context_map_path:=/tmp/RMUL.scdb`：
  日志 `Scan Context database loaded: 96 entries, 30 rings x 90 sectors, max_radius=30.0m`。
- 查询成本：121 条库约 7 ms/次；只在 LOST 时用（现有接线即如此），不要每帧跑。

### 2.4 使用前提与限制

1. 点云需**重力对齐**（Lucifer odom/PCD 满足；z=0 在开机雷达平面，`map_sensor_z=0`）。
2. 位置精度受 `grid_step` 限制（最大约 step/√2）；RMUL 建议 `step=1.0, max_radius=30, rings=30, sectors=90`。
3. `.scdb` 与建库 config 强绑定；换图/精修地图后必须重建。
4. 对称场地多候选靠下游 GICP + margin + 时序校验兜底；SC 只提供候选与 yaw 初值。

---

## 3. 地图资产：离线清动态残影 + keyframe 落盘（P0-1）

### 3.1 工具

`src/bringup/tools/clean_map_dynamic.py`（numpy，无 scipy/open3d）：

- 输入：`--map <world.pcd> --keyframes <dir>`（small_glim `mapping.save_raw_mapping_frames=true` 产出的
  `frame_<idx>.pcd` + `poses.txt`，每行 `se3(tx,ty,tz,qx,qy,qz,qw)`）。
- 原理：每关键帧建 0.5° 方位×俯仰球面深度图，地图点投到每帧取同格深度：
  `|dist-d| ≤ occ_tol` → observed +1；`dist < d − free_margin` → free +1；其余不计。
  删除条件：`observed < min_observed` 且 `free ≥ min_free`；**无证据一律保留**。
- 输出：清理后 PCD（binary xyz，PCL 可读）、removed PCD、统计。
- 参数：`--voxel 0.05 --occ-tol 0.15 --free-margin 0.30 --min-observed 2 --min-free 3 --max-range 20 --chunk 200000`。

### 3.2 接线

- `src/bringup/launch/{sim,real}.launch.py` 新增 `save_keyframes`（默认 False）→
  `mapping.save_raw_mapping_frames`。
- `clean_map_dynamic.py` 已 `install(PROGRAMS)`，可直接 `ros2 run bringup clean_map_dynamic.py ...`。
- 文档：`src/bringup/tools/README_clean_map.md`。

### 3.3 使用流程

```sh
# 1) 建图时保存关键帧（nav/mapping 均可；mapping 模式最常用）
ros2 launch bringup sim.launch.py world:=RMUL mode:=mapping save_keyframes:=True
# 跑完 Ctrl-C（PCD + frame_*.pcd + poses.txt 落盘）

# 2) 离线清理
ros2 run bringup clean_map_dynamic.py \
  --map src/bringup/PCD/RMUL.pcd \
  --keyframes <map_save_dir> \
  --output /tmp/RMUL_clean.pcd --removed /tmp/RMUL_removed.pcd

# 3) CloudCompare 目检 clean/removed，确认后备份原 PCD 再替换
```

### 3.4 验证

- pytest 5/5（合成房间+动态方块：动态删除率 100%、静态保留率 100%、PCD 往返、参数边界）。
- ament/ctest 注册通过（bringup 测试）。
- 814k 点 × 30 关键帧合成数据：体素后 129k 代表点，证据阶段 3.25 s、墙钟 4.19 s，
  动态方块删除率 100%、静态/无观测保留 100%。
- 输出 PCD 已用 `pcl::io::loadPCDFile<pcl::PointXYZI>` 读回验证（intensity 缺省为 0，正常）。

### 3.5 注意

- 关键帧很密（0.1 m）时人走过一处会被连续多帧直接观测，`observed` 可能 3–5，默认 `min_observed=2`
  删不掉 → 调高 `--min-observed`（如 5），代价是稀疏静态点风险上升，**必须目检 --removed**。
- 遮挡/无观测区域没有证据不删，残影可能残留；贴地残影底部常判不成 free。
- 清完后重建 `.scdb`（若用 Scan Context），否则描述子对不上。

---

## 4. MINCO 墙钟预算 + 性能基准（P0-3）

### 4.1 预算

- 新增 `navigation2/include/smoothing_budget.hpp`（header-only，steady_clock，<=0/NaN/Inf = 关闭）。
- `minco_path_smoother_node.cpp`：`repair.time_budget_ms`（代码默认 0；部署
  `src/bringup/config/common/navigation2.yaml` 设 **80 ms**，≈8 Hz 回调 125 ms 的 64%）。
- 超时行为：
  - repair 循环：不再追加尝试 → 走已有"回退原始路径"分支，perf `failure_reason=time_budget`；
  - 两阶段：跳过第二阶段（不放大段时间），**保留第一阶段合法结果**，不判失败。
- 预算=0 时行为与改动前完全一致（attempt 次数/发布结果/日志）。

### 4.2 基准脚本

`navigation2/tools/bench_planning.py`（rclpy，无 Gazebo）：

```sh
ros2 run navigation2 bench_planning.py --goals "1.0,2.0;3.5,-1.5" --timeout 2.0 --repeat 3
ros2 run navigation2 bench_planning.py --count 20 --seed 7 --csv /tmp/bench_planning.csv
```

发 `/goal_pose`，以 `/plan` stamp 变化计时，记录 latency、路径长度、成功/超时，输出 p50/p95/p99 + CSV。
用 `/map` 矩形内固定 seed 采样时，`--count N` 可复现。

### 4.3 验证

- `colcon build --packages-select navigation2` ✅；gtest `test_smoothing_budget` 8/8 ✅。
- headless 实测：小预算触发 `repair attempt 2/3 skipped: wall-clock budget exhausted … publishing original path`
  与 `two-stage fine optimization skipped … keeping first-stage result`；perf CSV `failure_reason=time_budget`；
  预算=0 时与旧行为逐条一致。
- bench 脚本：成功/超时/无 `/map` 三条分支实测通过，`py_compile` 通过。

---

## 5. 集成改动清单（Lead）

| 文件 | 改动 |
| :- | :- |
| `src/localization/fast_location/CMakeLists.txt` | 新增 `scan_context` 库、`build_scan_context_map` 可执行、`scan_context_test`；节点链接 |
| `src/localization/fast_location/src/robot_localization.cpp` | SC 参数声明/读取/加载；`performGlobalSearch` SC 召回 + 网格回退；NIS 限定到跟踪期（见 §1） |
| `src/localization/fast_location/include/fast_location/robot_localization.hpp` | SC 成员与 include |
| `src/localization/fast_location/config/fast_location.yaml`、`src/bringup/config/common/fast_location.yaml` | 验收 5 参数 + SC 4 参数 + 注释 |
| `src/bringup/config/common/navigation2.yaml` | `repair.time_budget_ms: 80.0`（部署真源；包内 params 也已有） |
| `src/bringup/CMakeLists.txt` | `clean_map_dynamic.py` 加入 `install(PROGRAMS)` |
| `src/bringup/tools/clean_map_dynamic.py` | 加可执行位 |

---

## 6. 全量验证结果

```text
fast_location : build ✅ | colcon test 10/10 ✅
                alignment_quality 15/15, global_search 21/21, scan_context 6/6, frame_transforms 5/5
navigation2   : build ✅ | smoothing_budget gtest 8/8 ✅
bringup       : build ✅ | test_clean_map_dynamic ✅ (pytest 5/5)
config        : python3 src/bringup/tools/check_nav_contract.py . 全 [ OK ]
python        : clean_map_dynamic.py / bench_planning.py py_compile ✅
SC 端到端     : RMUL.pcd → 96 entries / 1.0MB / 0.02ms per entry；节点加载日志 ✅
地图清理实测   : 814k 点/30 关键帧 → 3.25s，动态删除 100%、静态保留 100%
```

未跑：Gazebo 仿真、实车、真实赛场 bag、navigation2 launch_test（需仿真环境）。

---

## 7. 下一步建议

1. **先做实车/回放基线**（本轮只到单测+离线）：
   - 走廊/隧道 bag 对比 `nis_use_chi2` 开/关：`OK nis_reject` 率、`LOST init_partial_pose` 次数、map→odom 抖动；
   - 对称场地 LOST 对比 SC 开/关：重定位耗时、误锁率、margin 分布；
   - 真实建图 + `clean_map_dynamic.py`：目检 `--removed`，再跑定位看 LIT/LOST 频率；
   - 观察 `repair.time_budget_ms=80` 的 `time_budget` 触发率，用 `bench_planning.py` 的 p95/p99 反推预算。
2. **未做项**：非静止初始化（P1-3）、reset 协调（P1-2）、输出协方差/诊断词表（P1-6）、
   动态障碍预测层（P1-5）、LOS 化简（P1-4）、真位姿图/UWB（P2）。
3. **调参重点**：
   - `nis_rejection_probability`（1e-3 起步，走廊场景可放宽到 1e-2 观察拒帧率）；
   - `require_full_pose_on_init`（若首配在走廊反复失败，先关掉确认是否门太严）；
   - `scan_context_top_k` / `scan_context_min_candidates` / `--step`（步长越小库越大、位置越准）；
   - 清残影的 `--min-observed`（按关键帧密度调，必须目检）。
