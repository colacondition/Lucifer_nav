# 03 · TDT 后端工程模块 × Lucifer 迁移横评

> 输入口径：TDT 定位仓库 `/tmp/z-localization`（下文路径均相对该仓库根；`design.md` 为文章同源设计说明），
> 配套导航仓库 `/home/cola/tdt-nav-kit-main`；Lucifer 为 `/home/cola/Lucifer_nav`（下文路径相对该根）。
> 修订口径：按用户要求，**不考虑许可证/版权影响**；本册聚焦"哪些模块可直接复用、技术耦合在哪、工作量怎么变"。
> 前端 ESKF、去畸变与描述子数学细节见本目录另两册，导航算法细节见导航分册。
> 全部结论均给 `文件:行号`；仓库中查不到的写"未见"。本次未编译、未启动 ROS、未改任何源码。

## 0. 摘要

1. **TDT 后端不是"算法难"，是"工程账本厚"**：backend 24519 行（src 13232 / include 4179 / apps 3428 / tools 3680，161 个 .cpp/.hpp），
   前端另有 9008 行；其价值集中在六件事——五态诊断、描述子重定位、UWB 时空对齐、配准验收、位姿图、地图工具链。
2. **Lucifer 的定位栈在"结构"上与 TDT 后端正相反**：Lucifer 是 `small_glim`(LIO, GTSAM ISAM2) + `fast_location`(GICP 对先验 PCD, 只发 `map→odom`)，
   没有全局因子图、没有描述子、没有 UWB、没有状态重置协议；对外只有 `OK/LOST` 二态（`src/localization/fast_location/README.md:108,237-238`）。
3. **最值得做的不是"把后端搬过来"，而是三件独立、可离线验收的工程补强**：
   ① 地图包化 + 自动清残影（A6）；② Scan Context 离线描述子 + 重定位（A2）；③ 配准验收升级（A4）。
   这三项都不动 `small_glim`/MPC 的既有契约，可先离线验证，风险最低、收益最直接。
4. **五态状态机不建议照搬为对外契约**：Lucifer 已用"二态 + reason 词表"替代，且下游（MPC/决策）只按 `LOST` 动作
   （`src/navigation/navigation2/test/test_integrity_gate.py:66-96`；门控配置 `src/bringup/config/common/navigation2.yaml:445-449`）。
   照搬只会同时破坏现有测试与下游语义；应只吸收其**诊断维度**（A1）。
5. **UWB 与位姿图（A3/A5）是"高收益但高耦合"项**：二者互为前提（UWB 要挂在全局图上才成立），
   在当前 Lucifer 架构下都属于"换骨架"级别，建议保持观察，不做首轮移植。
6. **去掉许可约束后，TDT 后端的大半工程件可以直接复用**：离线 cleanup/map_bundle/build 工具（独立 C++ + PCL）、Scan Context（约 300–400 行）、`pose_conditioning` + `registration_solver` 白化（约 280 行）、`graph_innovation` 的 NIS/χ²（约 60 行）、reset 协调（约 500 行）、UWB 对齐/融合（约 600 行）都能按模块 vendor；A2/A4/A6/A7 的工作量各降一档（详见第 4 章）。**真成本在技术耦合**：GTSAM 4.2.x vs 4.3a0、自定义前端 frame/session 协议、boost/PCL/TBB 依赖、8 核线程预算与配置体系差异。
7. **Lucifer 并不弱**：双雷达驱动级融合 + PTP（实测 0.42 ppm）、语义地图/隧道云台收放、MINCO+SFC+OSQP MPC 全链、
   307 个 gtest + 53 个 pytest、Gazebo 双场景、21 份分层配置、`build.sh` 固定编译顺序 + 覆盖检查——这些是 TDT 定位仓库没有或更弱的工程面（第 2 章）。
8. **Adoption matrix 判定"最值得做"的三项**：**A6 地图包化 + 自动清残影 → A2 SC 重定位 → A4 配准验收升级**（第 1 章）。

## 1. Adoption matrix

### 1.1 评级口径与总表

- 改动量：**S** ≤ ~300 行新代码 + 1 个测试文件；**M** 300–1500 行或新增一个独立节点/工具；**L** >1500 行、新增包或改 `small_glim`/`fast_location` 主干契约。
- 收益/风险按 1–5 打分（收益=对比赛鲁棒性/定位精度/工程速度的影响；风险=破坏现有可用栈、引入硬件依赖、返工概率）。

| 编号 | 模块 | 结论 | 改动量 | 收益 | 风险 | 关键证据锚点 |
| --- | --- | --- | --- | --- | --- | --- |
| A1 | 五态状态机与下游契约 | **需改造**（只做内部诊断维度，不新增对外状态） | S | 中(3) | 低(1) | `localization_state.hpp:13-19,50-57`；`publisher_legacy.cpp:17`；`fast_location/README.md:108,237-238` |
| A2 | Scan Context 重定位 + 离线描述子地图包 | **可直接移植**（先离线工具 + 独立召回模块） | M | 高(4) | 中(3) | `scan_context.hpp:16-31,40-120`；`tools/build.cpp:47-114`；`fast_location/README.md` 全局搜索段 |
| A3 | UWB 融合 / 无图定位 | **硬件条件**（代码可复用，但仍需全局图才完整） | M–L | 高(4,有硬件时) | 高(4) | `uwb_alignment.hpp:20-48`；`uwb_fusion.hpp:115-164`；`backend.yaml:203-266`；Lucifer 全仓无 UWB（grep 空） |
| A4 | 配准验收（Schur/预条件/白化/NIS/χ²） | **可复用代码改造**（2D 条件数 → 3D 平移/旋转 Schur + χ² 尾概率） | S | 中高(4) | 低(1) | `pose_conditioning.cpp:32-84,89-153`；`registration_solver.cpp:230-270`；Lucifer `alignment_quality.hpp:200-243`、`robot_localization.cpp:939-944` |
| A5 | 关键帧位姿图（滑窗/边缘化/锚点） | **分阶段**：S=重锚连续化；L=真图（模块可复用，受 GTSAM 版本约束） | S / L | 中(3) / 高 | 中(3) / 高 | `pose_graph.hpp:1-130`；`pose_graph.cpp:104-160`；`fast_location/README.md:208-222` |
| A6 | 地图自动清残影 + 地图包工具链 | **可直接复用离线工具**（keyframe 落盘 + cleanup/build 工具改造） | S–M | 高(5) | 低(1) | `tools/cleanup/visibility.cpp:1-5`；`tools/cleanup.cpp:9-49`；`map_bundle.hpp:15-35`；`test/evaluation/metrics.py:12-40` |
| A7 | reset 协调语义 | **需改造**（`small_glim` 暴露 reset/epoch + `fast_location` 接边界，先在仿真） | M | 高(4) | 中(3) | `reset_coordinator.hpp:16-31,47-64`；`backend/README.md:135-141`；`test/integration/localization_reset_test.cpp:9-116` |
| A8 | G-PROBE / 回环+PCM / body_calibration / startup_hint | **不建议/观察** | — | 低(1–2) | — | `src/backend/include/backend/gprobe.hpp:1`；`src/backend/src/loop_consistency.cpp:1`；`src/backend/tools/body_calibration.cpp:1`；`backend.yaml:353-365` |

### 1.2 A1 五态状态机与下游契约

**TDT 作用**：把"匹配差"拆成可区分的证据链——点云弱 → `OBS_DEGRADED`（继续信位姿、拒冲突校正）；漂移 → `SUSPECT`（保留前端、重找 `map→odom`）；
发散 → `LOST`（重置前端后重定位）。判定谓词收敛在 `LocalizationStateMachine`：`statusName`、`poseTrusted`、`tfFrozen`、`recoveryPath`
（`src/backend/src/localization_state.cpp:15-51`），状态集合见 `localization_state.hpp:13-19`；冲突阈值来自
`innovation.conflicts_before_suspect=7`（`config/backend.yaml:370`）、`suspect_failures_before_lost=3`（`config/backend.yaml:194`）。
下游契约是 `pose_trusted` + 冻结语义（`apps/publisher_state.cpp:44-56` 的 `key=value` 串）、以及兼容危险码
`/LAM_danger_situation`（可信=0，否则=9，`apps/publisher_legacy.cpp:17`；文档表 `src/backend/README.md:111,147-153`）。
另有 `generation` 代次身份，保证 reset 期间旧工作不污染新阶段（`localization_state.hpp:28-34`）。

**Lucifer 现状**：二态 `OK <reason>` / `LOST <reason>`，reliable + transient_local，reason 词表已覆盖
`nis_reject / projected / frontend_diverged / pose_prior_jump / tracking_lost / map_odom_jump`（`src/localization/fast_location/README.md:108-125`）；
下游只有 `LOST` 让 MPC 停车、决策冻航点（`README.md:59,288-290`；门控配置 `src/bringup/config/common/navigation2.yaml:445-449`）；
并已明确撤销中间态（`fast_location/README.md:237-238`）。注意：Lucifer 的 `OK nis_reject`（握上一拍 TF）在行为上**已经等价于**
TDT 的 `OBS_DEGRADED`，`LOST`（握手 TF + 全场搜）等价于 TDT 的 `SUSPECT/LOST`。

**结论：需改造（S）**——不新增对外状态（会破坏 `test_integrity_gate*.py` 与 MPC 门控），只做两件事：
(i) 把 TDT 的四分诊断维度映射进 reason 词表（如 `hold_conflict` / `hold_weak` / `recover_local` / `recover_global`），
(ii) 在 `fast_location` 内部区分"握 TF"与"改锚点"，供 A5/A7 的阶段化使用。收益在可观测性而非精度，改动极小。

### 1.3 A2 Scan Context 重定位 + 离线描述子地图包

**TDT 作用**：把"点云查询"做成键值对——描述子当键，关联地图位置与局部点云（`docs/design.md:326-364`）。
配置 30 环 × 90 扇区、`max_radius 80`、`downsample 0.6`、`lidar_height 0.35`、3 份平移增强（`lateral_shift 2.0`）、
候选 15、距离门 0.8（`config/backend.yaml:36-64`）；检索先按描述子距离排序再取 yaw 初值（`scan_context.hpp:80-90,103-116`；
实现在 `src/backend/src/scan_context/descriptor.cpp:38-53,181-226`），小地图下直接遍历、不用 KD-tree（`/tmp/article.txt:113`）。
**离线生成**是关键工程点：`build_pcd_map_bundle` 在精修 PCD 上按网格采样模拟扫描，重新生成关键帧 + 描述子 + `map.pcd`
（CLI 见 `tools/build.cpp:47-114`，文档 `src/backend/docs/parameters.md:413-483`；流程 `README.md` 第 5 步），
另有 `rebuild_scan_context_descriptors` 增量重建（`README.md:166`）。地图包结构 = `keyframes.txt` + `clouds/` + 描述子数据（`README.md:169-170`；
`keyframe_map.hpp:84-91`、`map_bundle.hpp:20-35`）。

**Lucifer 现状**：先验图是**单个 PCD**（`map_pcd_path`，`src/localization/fast_location/src/robot_localization.cpp:15,338,1646`），
无关键帧、无描述子、无地图包格式；重定位靠 `global_search`：AABB×360° 网格 + KD-tree 命中率打分 + top-k + ICP 精化 + 时序校验
（`fast_location/README.md`「全局重定位」段；实现 `src/localization/fast_location/src/global_search.cpp:1`，291 行）。
子图是"全局图按 FOV 裁剪"（`robot_localization.cpp:705,857` `gropGlobalMapInFOV`），不是关键帧子图。

**依赖**：新库无（Eigen 足够）；新话题无；但需要**地图资产升级**（PCD → 地图包）；若沿用 SC，需重力调平（TDT 用前端 roll/pitch，
Lucifer 可由 `small_glim` odom 提供）。
**收益**：LOST 后候选从"几何网格穷举"变成"描述子召回 + yaw 种子"，直接压缩重定位耗时与 ICP 次数；离线可算召回率。
**风险**：双轨地图资产维护（旧 PCD + 新 bundle）；对称场地误召回（TDT 也靠 margin/时序补，`fast_location` 已有 margin 机制）；SC 对高度分布敏感。
**结论：可直接移植（M）**——`src/backend/src/scan_context/` 的描述子构造/yaw 估计可直接 vendor，先做**独立离线工具**（输入 Lucifer PCD + bag，输出描述子索引 + 自测召回报告），
验证通过后再进 `fast_location`（届时把地图加载抽象成 `map_provider`，S 改动）。适配点：点云类型、重力调平（用 small_glim odom 的 roll/pitch）、参数标定。

### 1.4 A3 UWB 融合 / 无图定位

**TDT 作用**：双通道。(i) 启动对齐：配对同一时刻的里程计/地图天线位置，鲁棒拟合 SE(2)+Z，给可观性/稳定性验收
（`uwb_alignment.hpp:20-48`；配置 `bootstrap.min_baseline=3.0 m`、`stable_solutions=2`、`max_yaw_std_deg=10`，`config/backend.yaml:246-266`）；
(ii) 在线融合：把 `T_map_uwb` 与 **时间偏置** 作为共享变量进因子图（`uwb_fusion.hpp:115-164`、`pose_graph.hpp:103-114`，
配置 `estimate_zero_pose_online=true`、`time_offset_seconds=-0.15`、`estimate_time_offset_online`，`backend.yaml:203-232`）。
无图 + UWB 时跑几米即可让 yaw 可观（`/tmp/article.txt:28,130-133`）。

**Lucifer 现状**：**全仓无任何 UWB 代码/话题/配置**（`grep -rn 'uwb\|UWB' src` 为空）；架构上也没有无图模式——
`fast_location` 必须有先验 PCD（`src/localization/fast_location/README.md:215`「先验 PCD 必须与 small_glim 建图输出同坐标系」）。
**依赖**：UWB 硬件与协议驱动、新话题（`PoseStamped`）、时间同步、**全局因子图（A5）或至少一条绝对观测通道**。
**收益**：强遮挡/洞内位置兜底、无图启动；**风险**：硬件未上车的假设全部未验证；UWB 噪声/多径在赛场未必友好；耦合 A5。
**结论：硬件条件（首轮不做）**。若保留可能性，最小化路径是"UWB→`/initialpose` 种子生成器"（独立节点，M），
只把 UWB 位置当初始位姿提示，不参与在线优化——这样既不碰 `fast_location` 主环路，也不需要因子图。代码层面 `uwb_alignment`/`uwb_fusion` 可直接复用，等硬件到位再评估。

### 1.5 A4 配准验收（Schur 补 / 预条件 / 白化 / NIS / 卡方）

**TDT 作用**：配准后按"残差 + 支撑 + 覆盖 + Hessian"四级验收（`quality.hpp:15-53`）；
平移/旋转各做 3×3 Schur 谱分解、取轴对齐强度、弱轴标记与谱封顶预条件（`pose_conditioning.cpp:32-84`，入口 `:89-153`），
求解用预条件 CG 或数值秩伪逆（`quality.hpp:111-126`）；
信息矩阵按"保留秩 + 残差方差"白化构造，弱方向直接降秩（`registration_solver.cpp:230-270`），
并对共模平移/旋转加协方差地板（`registration_solver.cpp:284-310`，配置 `measurement_translation_std_floor=0.005`、`max_rmse=0.25`、
`min_correspondences=80`、`min_coverage_sectors=6`、`degeneracy_condition_threshold=10.0`，`config/backend.yaml:104-128`）；
最后用图预测协方差算 NIS 与 χ² 上尾概率（boost `chi_squared`，`src/backend/src/graph_innovation.cpp:19-54`），
阈值等价于 `rejection_probability=0.001`（`backend.yaml:368`）。

**Lucifer 现状**：**已有一半**——2D Hessian/点云分布条件数 + 软/硬阈值退化投影
（`src/localization/fast_location/include/fast_location/alignment_quality.hpp:34-52,200-243`，参数
`degenerate_condition_threshold=80`、`degenerate_hard_reject_condition=5000`，`src/localization/fast_location/README.md:150-156`），
scan-to-map NIS + χ² 阈值 12.0（3 自由度 99%）（`robot_localization.cpp:47-51,939-944`）、
NIS 连败 8 拍进 LOST、前端发散连败 5 拍进 LOST、EMA 平滑（`robot_localization.cpp:1029-1074`）。
**缺口**：条件数是 **2D**（不含 z/roll/pitch）；没有平移/旋转分块 Schur 谱与轴向弱方向；没有预条件器（GICP 内部求解，不暴露）；
"白化"退化为"投影可观测方向"（因为 `fast_location` 不产出因子，只产 TF）。

**结论：可复用代码改造（S）**——`pose_conditioning.cpp`（约 200 行）与 `registration_solver.cpp:230-310` 的白化/降秩逻辑可直接移植，把 `computeHessianConditionNumber` 升级为 3D 平移/旋转 Schur 谱 + 轴向弱方向标记，
并把 NIS 的固定阈值换成可配置的 χ² 尾概率（`graph_innovation.cpp` 约 60 行可直接复用）。
收益=隧道/走廊抖动与误拒减少；风险低（纯几何判据，不动控制链）。**不引入 GTSAM/预条件 CG**——那是 A5 的配套。

### 1.6 A5 关键帧位姿图（滑动窗口 / 边缘化 / 锚点）

**TDT 作用**：GTSAM iSAM2 + 固定滞后窗（`graph.fixed_lag_seconds=30.0`，`config/backend.yaml:330`），
窗尾 `marginalizeLeaves` 保留约束（`src/backend/src/pose_graph.cpp:110-160`）；因子含里程计、先验图配准、回环、UWB、重力
（`pose_graph.hpp:60-120`）；初始化/恢复成功后加 Prior 作为锚点（`docs/design.md:282-303`「因子图设计」）。
**GTSAM 可用性**：Lucifer 侧要求 GTSAM 4.3a0 + gtsam_points v1.2.0（`src/localization/small_glim/README.md:130-136`），但 TDT 要求 **4.2.x**
（`src/backend/CMakeLists.txt:36` `find_package(GTSAM 4.2 CONFIG REQUIRED)`、`src/backend/README.md:43`），
且 TDT 为 4.2 的 smoother 缺陷写了专门规避（`pose_graph.cpp:104-108` 注释：4.3a1 才含修复）。
→ 移植必须先在 4.3a0 重测，那段规避逻辑可能不适用。
**Lucifer 现状**：`small_glim` 本身就是 iSAM2 固定滞后平滑（`src/localization/small_glim/README.md:34`），但它在 **odom 系**；
map 系只有 `fast_location` 的单点 `map→odom`，无图、无锚点、无回环。
**收益**：LOST 重锚定后轨迹不会突变（现在靠跳变门 + `lost_escape` 超时放行，`src/localization/fast_location/README.md:71-90`「跳变门超时放行」）；
UWB/回环才有挂载点。**风险**：等于给定位栈换骨架；`fast_location` 要改成"产因子"，`small_glim` 要开放关键帧，A7 前置。
**结论：分阶段**。阶段 1（S，建议做）：在 `fast_location` 内部做"分段 `map→odom` 偏移 + 重锚点连续性"，
把 LOST 恢复后的 EMA 大修正变成可审计的段切换；阶段 2（L，暂不）：真位姿图，等 A3/A7 需求明确再评估。

### 1.7 A6 地图自动清残影 + 地图包工具链

**TDT 作用**：建图后 `clean_map_bundle` 用"多帧可见空间/自由证据"清动态残影（参考 FreeDOM，`tools/cleanup/visibility.cpp:1-5`；
CLI/语义 `tools/cleanup.cpp:9-49`；文档 `/tmp/z-localization/README.md:104-113`「3. 自动清理动态残影」），再 CloudCompare 精修，
最后 `build_pcd_map_bundle` 生成地图包；配套还有 `transform_map_bundle`、`filter_map_bundle`、`rebuild_scan_context_descriptors`（`README.md:162-166`）。
清理效果有**量化评测**：点级 precision/recall 与体素级 preservation/rejection（`test/evaluation/metrics.py:12-40`），
并用带固定 SHA256 的标注快照做验收（`test/evaluation/test_labeled_cleanup.py:15-16`）。

**Lucifer 现状**：建图两条链——`small_glim` 出 PCD、`slam_toolbox` 出栅格（`README.md:199` 建图双链表格），
语义图由 `pgm_to_navmap.py` + `semantic_map_editor.py` 人工标注（`README.md:243-259`）。
**没有**任何动态残影清理工具；且 `small_glim` 只在析构时合并落盘 PCD（`src/localization/small_glim/README.md:82-100`「建图工作流」），
**不保存逐帧/关键帧点云与位姿**——这是自动清理缺的前提。
**依赖**：`small_glim` 增加"关键帧点云 + 位姿落盘"（S：复用 `keyframe_trans_thresh`/`rot_thresh`，`src/localization/small_glim/README.md:63-80`）；
离线清理工具（独立可执行/脚本，M）；无需新库（PCL 足够）。
**收益**：地图干净 → 定位内点率、代价地图假障碍、语义图人工标注工作量全部改善；比赛适应性训练环境脏时收益最大。
**风险**：低（纯离线，失败不影响在线）；注意清理参数需按场地调（`voxel_resolution/raycast_range/free_observations`，`tools/cleanup.cpp:24-31`）。
**结论：可直接复用（S–M），最优先**。TDT 的 cleanup/map_bundle/build 工具是独立 C++/PCL，可改成"PCD 进、PCD 出"直接移植；保留其来源声明以便追溯即可。

### 1.8 A7 reset 协调语义

**TDT 作用**：`/localization_reset` 触发前后端协调重置；事务身份（requester/transaction/target_epoch）保证幂等与重试安全
（`reset_coordinator.hpp:16-31,47-64`），前端返回 `outcome/has_boundary/source_epoch/old_from_new` 边界变换
（`src/localization_interfaces/README.md:92-140` 的 ResetFrontend 段）；无边界时按"机器人静止"假设衔接（`backend/README.md:138`）。
这一协议有 5 组集成测试覆盖拒绝/重试/冷启动/不确定态（`test/integration/localization_reset_test.cpp:9-116`、
`localization_discontinuity_test.cpp:9-179`），用合成场景 fixture 离线跑，不需要机器人。

**Lucifer 现状**：**没有协调重置**。`fast_location` 明确"不重置 `small_glim`"（`README.md:222`）；`small_glim` 只在启动时初始化
（需静止约 1 秒，`src/localization/small_glim/README.md:34-36`）；LOST 的恢复手段是全场搜索 + `/initialpose` 种子
（`src/localization/fast_location/README.md:106-125` 完整性契约表）。若 `small_glim` 本身发散（LIO 爆炸），当前**唯一出路是重启整个栈**，
重启会丢 `odom` 系 → `map→odom` 失效。
**依赖**：`small_glim` 加 reset 服务与新 epoch（S–M）；`fast_location` 消费边界做位姿衔接、并接受 `odom` 系重置（M）；仿真验证。
**收益**：LOST 自愈能力（这是 TDT 全文最"工程"的部分）；**风险**：动 LIO 生命周期，容易引入时序/初始化竞态；必须先在仿真做。
**结论：可复用协议（S–M），排第 4 优先**。`reset_coordinator.cpp`（约 500 行）的编排/事务语义可裁剪复用；先只做"冷重启 + 边界位姿服务"的最小版（不含事务幂等），
事务幂等留到多节点协同（决策/电控也要参与）时再补。

## 2. Lucifer 相对 TDT 的对等与领先能力（"并不弱"的依据）

### 2.1 双雷达驱动级融合 + PTP 时间同步（TDT 定位仓无此能力）

- 融合在驱动内部完成，对下游零改动：`src/driver/mid360_driver/src/mid360_driver_node.cpp:272-443`
  （主雷达选择 `:52-55`、20 ms 窗口与超窗降级 `:323-331`、近点裁剪 `:383-433`）；参数 `src/bringup/config/reality/mid360_driver_dual.yaml:33-48`。
- 实测：合并 10.000 Hz、13,757 点、融合丢 0 帧、PTP 下相对漂移 0.42 ppm（无 PTP 10.2 ppm）（`docs/dual_lidar.md:45-56`）；
  CPU 预算实测 ≈51% 单核 / ≈400 MB（`docs/dual_lidar.md:154-165`）。
- PTP 主机做主钟的原因与踩坑（18~37 s 纪元差会让代价地图丢"未来时间戳"点云）写在 `src/bringup/tools/ptp_master.sh:3-21`；
  状态检查 `src/bringup/tools/ptp_status.sh:1`；开机自启 `ptp-master.service`。
- 对照：TDT 定位仓只有 Livox 官方驱动 + `livox_ros_driver2`（`/tmp/z-localization/README.md:26-37` 依赖表），未见双雷达与 PTP 工程。

### 2.2 语义地图 / 隧道云台收放（TDT 定位仓无此能力）

- 语义图是**唯一真源** `map/<world>.msgpack`，三态 terrain + TUNNEL 通道 + 轴线 + 净高/净宽/限速（`README.md:243-259`）；
  加载校验严格（越界/未知标签直接拒绝，`src/navigation/navigation2/src/semantic_map.cpp:129-136,474-493,726-735`），
  隧道判定 `:253`。
- 云台链路：`rm_tunnel_posture` 按"距隧道本体 ≤ run_up(1.2 m) + 迟滞 0.3 m"收放（`src/navigation/navigation2/src/tunnel_posture_node.cpp:45-48,75-76`），
  MPC 在"请求收、实态还高"时停车等（`README.md:268-278`）；仿真由 `simulated_gimbal` 顶替电控回传。
  这是"能站但要摆姿态才能进"的场地语义落地，TDT 定位仓没有同类模块。

### 2.3 MINCO + SFC 校验 + OSQP MPC 的完整控制链

- 规划：A* + 距离场 clearance（`src/navigation/navigation2/src/kinodynamic_astar.cpp:1`、`distance_transform.cpp:1`），
  平滑：MINCO（`src/navigation/navigation2/src/minco/minco_optimizer.cpp:225`、`minco_time_allocation.hpp:1`），
  走廊校验：SFC（`src/navigation/navigation2/src/sfc_corridor.cpp:194`、`include/sfc_corridor.hpp:1-40`），
  控制：OSQP 全向跟踪 MPC（`src/navigation/navigation2/src/mpc/mpc_solver.hpp:1,27,127-142`，`CMakeLists.txt:25-26,46`）。
  双保险校验：全图 SFC + 5×5 m 距离场互补（`src/navigation/navigation2/src/minco_path_smoother_node.cpp:57,370,635`）。
- 值得注意：Lucifer **已经移植了 TDT 导航仓的 SFC 生成器与 Kinodynamic A***，并按现有惯例标注来源——`include/sfc_corridor.hpp:6`、
  `include/kinodynamic_astar.hpp:6`、`minco_path_smoother_node.cpp:42,57`，还修了 TDT 原实现的 origin 符号与 O(r²) 查询问题。
  这也说明 TDT 的**导航**部分对 Lucifer 已无增量，真正缺的是**定位后端**。

### 2.4 单元测试与自动化测试覆盖

- 总量：**307 个 gtest + 53 个 pytest**，52 个测试文件（统计口径：`TEST(_F)?(` / `def test_`，同口径 TDT 全仓为 155 + 36 / 39 文件）。
- 分布：`navigation2` 145+27（29 文件，含 11 个 `add_launch_test`：`CMakeLists.txt:195-225`）、
  `decision` 90+1（9 gtest + 1 launch test，`CMakeLists.txt:94-118,90-91`）、`serial` 27、`fast_location` 31+5、
  `bringup` 0+12、`cpp_lidar_filter` 8、`linefit` 5+2、`waypoint_editor` 0+6、`fake_vel_transform` 1。
- 测试是**行为级**的：ROS 级 launch_test 起真节点喂合成话题断言 FSM（`src/navigation/navigation2/test/mpc_harness.py:1-10`
  自述"creep-at-goal bug 就是单测全过、上车才现"）；配置契约静态校验 `src/bringup/tools/check_nav_contract.py:1-11`（可当 CI 跑）。
- **诚实的短板**：`small_glim` **0 个测试**，`fast_location` 的测试集中在几何与搜索（`alignment_quality_test.cpp`、
  `global_search_test.cpp`、`frame_transforms_test.cpp`），没有定位状态机/重置的集成测试——这正是 TDT 的强项（A7 的 fixture 值得借鉴）。

### 2.5 Gazebo 仿真与自动化测试

- `sim.launch.py` 一条入口带起 Gazebo + LIO + fast_location + 导航 + 模拟云台 + 航点执行（`src/bringup/launch/sim.launch.py:159-330`）；
  两套场地 `RMUC2024_world/rmuc_map_light.world`、`RMUL27_world/RMUL27.world`（`src/simulation/pb_rm_simulation/launch/rm_simulation.launch.py:35,52`）。
- 仿真与实车共用同一配置真源、只覆盖差异（见 2.8），所以仿真验证的结论对实车有迁移性。
- 未见 CI 配置（无 `.github/`、无 CI yaml）——测试需手动 `colcon test`；这是可补的工程债，但不影响"测试资产存在"这一事实。

### 2.6 构建脚本与依赖文档

- `build.sh`（134 行）：固定 18 包编译顺序、单线程低负载、`--list`、以及"`src/` 有包没写进列表就告警"的覆盖检查（`build.sh:22-56`）。
- 依赖文档按"换机顺序"写清硬约束：gcc-13/C++23 不可降级、`gtsam_points v1.2.0` 必须 `-DBUILD_WITH_MARCH_NATIVE=OFF` 且
  `ldconfig`（ABI 陷阱）、GTSAM 4.3a0 装 `/usr/local`（`README.md:138-181` 3.1 依赖段）；并有 `tools/check_abi_flags.sh`。
- 对照：TDT README 用表格列依赖与 bag 流程（`/tmp/z-localization/README.md:26-37`），文档组织同样优秀，
  但没有"MARCH/ABI 旗标一致"这类**踩坑级**说明。

### 2.7 决策航点与巡逻

- 决策节点持有 `WaypointStore` + `WaypointExecutorClient`，用 action 接口跑跟进/穿越（`src/decision/decision_node/src/decision_node.cpp:63-93`）；
  巡检/中心区行为 `src/decision/decision_node/src/center_patrol.cpp:1`（`PatrolPath`，`include/decision/center_patrol.hpp:34`）；
  航点数据独立包 `src/interfaces/sentry_waypoints`；编辑器 `src/control/waypoint_editor`。
- 对照：TDT 定位仓不含决策/航点；导航仓只有算法组件（A*/MinimumSnap + Mask），无决策层。

### 2.8 配置分层

- 三层 `common/reality/simulation`（21 份 YAML：9/7/5），加载顺序 common → 环境 → launch 动态值，且明确"只放差异、禁止整份复制"
  （`src/bringup/config/README.md:5-28`）；跨文件契约有静态检查（2.4）。TDT 是单份 `config/backend.yaml`（441 行）+ `pcd/site.yaml` 覆写，
  分层能力弱于 Lucifer。

### 2.9 小结与自曝短板

领先/对等：双雷达+PTP、语义图/云台、MINCO+SFC+MPC、测试规模、Gazebo 双场景、构建/依赖文档、决策航点、配置分层。
明确落后：**模型层**（无描述子重定位、无全局图、无 UWB、无 reset 协议）、**地图工具链**（无自动清理、无地图包）、
**定位侧测试**（`small_glim` 0 测试、无状态机集成测试）。

## 3. 验证方案

### 3.1 可离线先验证（不需要车）

| 项 | 手段 | 落点建议 |
| --- | --- | --- |
| A1 诊断词表 | A2 之外的纯状态映射单测 | `src/localization/fast_location/test/` 新增 gtest；用现有 `alignment_quality_test.cpp` 风格 |
| A2 描述子召回 | 离线工具 + 已有 rosbag + 人工标注重访点，统计 recall@k / 距离-召回曲线 | 新增 `src/bringup/tools/` 脚本或独立包；**不改 `fast_location` 主路径** |
| A3（若做 seed） | 录一段 UWB+odom bag，离线跑对齐器 | 独立 gtest，输入合成噪声轨迹 |
| A4 Schur/χ² | 合成 Hessian 用例（走廊、平面、满秩）+ 现网 bag 回放算诊断分布 | `src/localization/fast_location/test/alignment_quality_test.cpp` 扩用例 |
| A5 阶段 1 | 重放 LOST 事件日志，检查段切换连续性 | 单测 + `fast_location` 内部函数 |
| A6 清残影 | bag 建图 → 清理 → CloudCompare 目检 + 点/体素指标 | 新工具 + 标注数据；可直接复用 `metrics.py:12-40` 的 precision/recall/preservation/rejection 指标实现，换 Lucifer 的数据结构 |
| A7 最小版 | 合成场景 fixture 跑 reset 拒绝/重试/冷启动 | 新增 `fast_location/test/test_reset_flow.cpp`，测试语义可直接参照 TDT `localization_reset_test.cpp:9-116` |

常用命令（Lucifer 现有入口，未验证即未执行）：

```bash
# 纯定位离线回放（不跑导航）
ros2 launch bringup sim.launch.py world:=RMUL mode:=mapping nav_rviz:=False
ros2 bag play /abs/path/site_loop --clock

# 导航回放 + 全量记录（供离线指标）
ros2 bag record -o /tmp/loc_eval /tf /tf_static /map /localization_status \
  /lio/robo/odom /small_glim/deskewed_cloud /plan /cmd_vel_chassis

# 配置契约静态检查（现成）
python3 src/bringup/tools/check_nav_contract.py .

# 单包测试（现成）
colcon test --packages-select fast_location navigation2 --event-handlers console_direct+
```

### 3.2 必须上车的

- 双雷达融合/PTP 的真实漂移与丢包（`docs/dual_lidar.md:45-56` 的台架数据只覆盖已测项）；
- UWB 的多径与时延（A3）；
- 隧道云台收放的时序（请求→电控执行→MPC 门控，仿真有 0.5 s 延迟模型（`README.md:277`）但真机不同）；
- 真实场地地图质量（灯光/观众/适应性训练环境脏），即 A6 的最终验收；
- 长时间运行的 CPU/内存与热漂移（`docs/dual_lidar.md:154-165` 是台架短时数据）。

### 3.3 指标定义（所有提升项共用）

| 指标 | 定义 | 采集方式 |
| --- | --- | --- |
| 定位 ATE/RMSE | 与仿真真值（或全站仪/动捕）逐点误差的 RMSE；分 XY / yaw | 仿真取 Gazebo 真值（当前未配真值输出，需加 `gazebo_ros_state` 或真值插件）；实车用标记点复核 |
| 重定位耗时 | 从判 `LOST` 到首次回 `OK` 的时间（p50/p95/p99） | `/localization_status` 时间戳差分 |
| 重定位成功率 | 成功回 `OK` 的 LOST 事件 / 总 LOST 事件；按"同位置/异地"分层 | 同上 |
| LOST 误报率 | 真值误差仍 < 阈值却被判 `LOST` 的时长占比（次/小时） | 真值 + 状态流对齐 |
| CPU/内存 | `%/核` 与 RSS，p95；关键进程分别统计 | `pidstat -p <pid> 1`、`ros2 topic hz` 辅助 |
| 规划成功率 | 下发目标后 N 秒内到达 / 总目标数 | `/plan` + `/cmd_vel_chassis` + 位姿 |
| 端到端 p99 时延 | `/plan` 生成 → `/cmd_vel_chassis` 首个响应；及 MPC 单拍求解时延 | rosbag 时间戳差分 |
| 急停/抖动 | TF 修正量 p99、加速度峰值、`/cmd_vel_chassis` 反向次数 | bag 离线统计（建议写 `eval/` 脚本） |

### 3.4 建议节奏（依赖关系）

```text
M1（离线，1–2 周）：A6 关键帧落盘 + cleanup/build 工具移植 + 量化评测   ← 无依赖，先做
M2（离线，1–2 周）：A2 描述子离线工具（复用 scan_context）+ 召回评估   ← 依赖 M1 的地图资产
M3（低风险在线，1 周）：A4 验收升级（复用 pose_conditioning/白化）+ A1 诊断词表
M4（仿真，1–2 周）：A7 最小重置版（复用 reset 协议）+ A5 阶段 1 重锚连续化
（可选）A/B：把 z-localization 后端按 legacy 接口跑成第二条 map→odom 来源
（观察）A3 UWB：硬件到位且 M4 完成后评估
```

## 4. 代码复用与移植策略（不考虑许可证）

### 4.1 可复用清单与工作量变化

| 模块 | 位置 | 规模（估） | 复用方式 | 适配成本 | 原估→新估 |
| --- | --- | --- | --- | --- | --- |
| 离线清残影 + 地图包/描述子生成 | `tools/cleanup*`、`map_bundle.cpp`、`tools/build.cpp` | 1000+ 行 | 独立 C++/PCL，可直接编译；输入从 bundle 改成 PCD | 关键帧落盘 + 格式转换 | M → S–M |
| Scan Context 描述子 + yaw | `src/backend/src/scan_context/`、`scan_context.hpp` | 300–400 行 | vendor 进新包/子模块 | 点云类型 + 重力调平 + 参数标定 | M–L → M |
| Schur 谱 + 预条件 | `pose_conditioning.cpp` | ~200 行 | 改造成 `AlignmentAssessment` 的 6×6 分析 | 输入换 fast_gicp Hessian | S–M → S |
| 白化/降秩/协方差地板 | `registration_solver.cpp:230-310` | ~80 行 | 直接并入验收逻辑 | 与现有 NIS 合并 | — |
| NIS + χ² 上尾概率 | `graph_innovation.cpp` | ~60 行 | 直接调用/移植 | 增加 boost `chi_squared` | — |
| 五态谓词/状态机 | `localization_state.cpp` | ~250 行 | 只取谓词与 reason 映射 | 不新增对外状态 | — |
| reset 协调 | `reset_coordinator.cpp` | ~500 行 | 裁剪成 `/localization_reset` + epoch | small_glim 需先有 reset 接口 | M → S–M |
| UWB 对齐/融合 | `uwb_alignment.cpp`、`uwb_fusion.cpp` | ~600 行 | vendor | 硬件 + 全局图 | 仍 L（硬件条件） |
| 位姿图/因子 | `pose_graph.*`、`graph_*.cpp` | 2000+ 行 | vendor | GTSAM 版本 + 前端协议 | 仍 L |

### 4.2 技术耦合清单（复用前必须处理）

1. **GTSAM 版本**：TDT 后端 `find_package(GTSAM 4.2 CONFIG REQUIRED)`（`src/backend/CMakeLists.txt:36`），并为 4.2 smoother 缺陷写了规避（`pose_graph.cpp:104-108`，注释称 4.3a1 才修复）；Lucifer 用 GTSAM 4.3a0（`small_glim/README.md:130-136`）。纯几何模块（SC、cleanup、pose_conditioning）不依赖 GTSAM，可先做；图相关模块需在 4.3a0 重测或给 TDT 侧独立编译 4.2 前缀。
2. **前端协议**：TDT 后端默认吃自定义 `/frontend/frame`（点云 + 位姿 + update 状态 + session/epoch，`src/frontend/README.md:73-85`）；接 small_glim 需写适配层，或走 legacy 接口 `/Odometry` + `/cloud_registered`（`src/backend/README.md:69-84`）。
3. **依赖与预算**：boost（χ²）、PCL、OpenMP/TBB、nanoflann、yaml-cpp；Lucifer 是 8 核写死预算（根 `README.md:18`），图优化线程/内存需重测。
4. **配置/参数体系**：TDT 单份 `config/backend.yaml`（441 行）与 Lucifer 三层 `common/reality/simulation` 不同构；复用模块只搬算法语义常量，不搬配置结构。
5. **接口/命名/TF**：TDT 的状态串、frame 名、服务名与 Lucifer 契约不同，需统一到 Lucifer 的 `/localization_status`、`map→odom` 契约。

### 4.3 三种集成方式与推荐

| 方式 | 做法 | 优点 | 缺点 | 结论 |
| --- | --- | --- | --- | --- |
| ① 按模块 vendor | 把纯几何/离线件拷入新包或 `third_party/`，保留来源注释与文件头，改数据类型/接口后接入 | 风险可控、可逐项验收、主链路不动 | 需要自己写适配与测试 | **推荐（主路径）** |
| ② 独立进程复用整套后端 | `z-localization` 独立编译运行（GTSAM 4.2 用独立前缀/容器），通过 legacy 接口喂 small_glim 数据 | 能拿到同场景下"因子图后端 vs 单帧配准"的 A/B 数据；无需改主链路 | 部署两套 GTSAM、TF/状态需桥接；实时性未知 | **推荐作为评测基线**，不做主链路 |
| ③ 整体替换 fast_location | 直接用 TDT 后端出 map→odom，删掉 fast_location | 一步拿到最完整后端能力 | 改 TF 契约/下游状态解析/参数体系，风险最高；与 8 核预算冲突未验证 | **暂不推荐**，等 ①/② 有数据再评估 |

### 4.4 执行建议

1. vendor 时保留来源与差异记录（沿用 Lucifer 现有 `sfc_corridor.hpp:6-23` 的写法："算法来源、改了什么、为什么改"）。
2. 先做不依赖 GTSAM 的模块：离线 cleanup/build 工具 → SC 描述子 → Schur/白化 → reset（依赖 GTSAM 前最后做）。
3. 每个模块先有独立单测/离线工具，通过后再进主链路；主链路保留参数开关与回退。
4. 线程/内存预算按 Lucifer 8 核重新标定，不照搬 TDT 的后端线程模型。
5. 参数只提取语义常量，不照搬 yaml 结构；接口统一到 Lucifer 话题/TF 契约。

## 5. 证据索引

| # | 主题 | 证据（文件:行号） |
| --- | --- | --- |
| 1 | TDT 五态枚举 / 谓词 | `/tmp/z-localization/src/backend/include/backend/localization_state.hpp:13-19,50-57` |
| 2 | TDT 状态机实现与阈值 | `.../src/localization_state.cpp:15-51,169-186`；`config/backend.yaml:194,370` |
| 3 | TDT 下游契约（危险码/状态串） | `.../apps/publisher_legacy.cpp:17`；`apps/publisher_state.cpp:44-56`；`src/backend/README.md:111,147-153` |
| 4 | TDT 状态设计说明 | `docs/design.md:304-325`；`/tmp/article.txt:101-107` |
| 5 | SC 配置与检索 | `.../include/backend/scan_context.hpp:16-31,80-116`；`src/backend/src/scan_context/descriptor.cpp:38-53,181-226`；`config/backend.yaml:36-64` |
| 6 | 离线地图包工具 | `.../tools/build.cpp:47-114`；`src/backend/README.md:162-170`；`src/backend/docs/parameters.md:413-483` |
| 7 | 地图包结构 | `.../include/backend/map_bundle.hpp:15-35`；`keyframe_map.hpp:84-91` |
| 8 | UWB 对齐/融合 | `.../include/backend/uwb_alignment.hpp:20-48`；`uwb_fusion.hpp:115-164`；`config/backend.yaml:203-266,246-266` |
| 9 | Schur/预条件 | `.../src/pose_conditioning.cpp:32-84,89-153`；`include/backend/quality.hpp:68-126` |
| 10 | 白化/信息构造 | `.../src/registration_solver.cpp:230-310`；`src/measurement.cpp:40-70` |
| 11 | NIS/χ² | `.../src/graph_innovation.cpp:19-54`；`config/backend.yaml:368` |
| 12 | 位姿图/边缘化/GTSAM 兼容 | `.../include/backend/pose_graph.hpp:1-130`；`src/pose_graph.cpp:104-160`；`src/backend/CMakeLists.txt:36` |
| 13 | 清残影工具与指标 | `.../tools/cleanup/visibility.cpp:1-5`；`tools/cleanup.cpp:9-49`；`test/evaluation/metrics.py:12-40` |
| 14 | reset 事务与集成测试 | `.../include/backend/reset_coordinator.hpp:16-31,47-64`；`test/integration/localization_reset_test.cpp:9-116` |
| 15 | TDT 规模/测试总量 | backend 24519 行（src/include/apps/tools）、161 文件；155 gtest + 36 pytest / 39 文件（本次统计） |
| 16 | Lucifer 二态契约 | `src/localization/fast_location/README.md:104-125,208-238`；`src/navigation/navigation2/test/test_integrity_gate.py:66-96` |
| 17 | Lucifer 退化/NIS | `src/localization/fast_location/include/fast_location/alignment_quality.hpp:34-52,200-243`；`src/robot_localization.cpp:47-51,939-944,1029-1074` |
| 18 | Lucifer 全局搜索 | `src/localization/fast_location/README.md:56-70`「全局重定位」；`src/global_search.cpp:1`（291 行） |
| 19 | Lucifer 地图/子图加载 | `src/localization/fast_location/src/robot_localization.cpp:15,338,705,857,1646` |
| 20 | small_glim 能力与测试缺口 | `src/localization/small_glim/README.md:14-36`（ISAM2/初始化）；该包 `test/` 目录不存在（0 测试） |
| 21 | 双雷达融合实现 | `src/driver/mid360_driver/src/mid360_driver_node.cpp:52-55,272-443`；`src/bringup/config/reality/mid360_driver_dual.yaml:33-48` |
| 22 | PTP 与实测 | `src/bringup/tools/ptp_master.sh:3-21`；`docs/dual_lidar.md:45-56,154-165` |
| 23 | 语义图/隧道云台 | `src/navigation/navigation2/src/semantic_map.cpp:129-136,253,474-493`；`src/tunnel_posture_node.cpp:45-48,75-76`；`README.md:243,260` |
| 24 | MINCO/SFC/OSQP 链 | `src/navigation/navigation2/CMakeLists.txt:25-26,46-70`；`src/minco/minco_optimizer.cpp:225`；`src/sfc_corridor.cpp:194`；`src/mpc/mpc_solver.hpp:1,27,127-142` |
| 25 | 移植先例与差异记录 | `src/navigation/navigation2/include/sfc_corridor.hpp:6-23`；`include/kinodynamic_astar.hpp:6-23`；`src/minco_path_smoother_node.cpp:42,57` |
| 26 | Lucifer 测试规模/注册 | 307 gtest + 53 pytest / 52 文件；`src/navigation/navigation2/CMakeLists.txt:143-225`；`src/decision/decision_node/CMakeLists.txt:90-118` |
| 27 | Gazebo 双场景 | `src/bringup/launch/sim.launch.py:159-330`；`src/simulation/pb_rm_simulation/launch/rm_simulation.launch.py:35,52` |
| 28 | 构建与依赖文档 | `build.sh:1-56`（134 行）；`README.md:138-181`（gcc-13/gtsam_points ABI 陷阱）；`src/bringup/tools/check_abi_flags.sh:1` |
| 29 | 配置分层 | `src/bringup/config/README.md:5-28`；`src/bringup/tools/check_nav_contract.py:1-11` |
| 30 | 可复用模块位置/规模/依赖 | `.../src/backend/src/scan_context/`；`pose_conditioning.cpp`；`registration_solver.cpp:230-310`；`graph_innovation.cpp`；`tools/cleanup*`；`reset_coordinator.cpp`；`src/backend/CMakeLists.txt:36`（GTSAM 4.2） |
