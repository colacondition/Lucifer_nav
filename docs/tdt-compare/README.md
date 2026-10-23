# TDT 哨兵系统 × Lucifer 对比与提升路线（总报告）

> **对比对象**
> - 文章：[RM2026 东北大学 TDT 哨兵定位 开源](https://bbs.robomaster.com/article/1971759?source=4)（RoboMaster 社区）
> - 文章配套定位仓库：`z-localization`（本机 clone 于 `/tmp/z-localization`）
> - 用户指定的"文章配套部分"：`tdt-nav-kit`（TDT 导航前后端，`/home/cola/tdt-nav-kit-main`）
> - 本地工程：**Lucifer**（`/home/cola/Lucifer_nav`，ROS 2 Humble）
>
> **修订口径**：按用户要求，**不考虑许可证/版权影响**。因此本报告中的"复用/移植"只按技术成本评估，第 6 章的"不建议"只从技术收益/风险判断；原报告中的 GPL 规避章节已替换为"代码复用与技术耦合"。
>
> **证据分册（本目录）**：[01 定位对比](01-localization.md) · [02 导航对比](02-navigation.md) · [03 工程横评与复用](03-engineering.md)
> 三册共约 1060 行，所有结论带 `文件:行号`；本报告只做汇总与排序。本次分析**只读代码/文档，未编译、未启动 ROS、未实跑**，性能数字均来自 TDT 自述或 Lucifer 文档/参数，收益为工程推断，需按 §7 实测。

---

## 0. 结论速览

**一句话**：Lucifer 不需要"换骨架"。定位上双方同为"LIO 前端 + 全局定位后端"分层，差别在**后端形态**；导航上 Lucifer 已经把 TDT nav-kit 最可迁移的机制吸收，并做得更工程化。真正的提升空间集中在**定位后端的地图资产与几何验收**——这两块可以离线验证、不碰在线主环、风险最低。

**五条核心结论**

1. **同一分层、不同后端。** TDT = ZLIO（15 维 ESKF、SE₂(3)⋉se(3)）→ 关键帧位姿图（ISAM2 + 里程计/先验图/回环/UWB/重力因子 + 五态状态机 + Scan Context + reset）；Lucifer = small_glim（ISAM2 固定滞后 + GICP/iVox）→ fast_location（单帧 FastGICP 对先验 PCD → 唯一 `map→odom`，含 Hessian 退化投影、NIS/χ²、EMA、跳变门、全场重定位）。
2. **Lucifer 已用别的工程手段解决了 TDT 强调的大部分问题**：退化方向投影 ≈ TDT 的"弱方向不增加约束"；`OK nis_reject` 握 TF ≈ OBS_DEGRADED；`LOST` 握 TF + 全场搜 ≈ SUSPECT/LOST；100Hz IMU 补帧 + 10Hz GICP ≈ "预测 + 观测"节拍。**这些不是差距**（详见 [01-localization.md](01-localization.md) §2 的 20 项判定表）。
3. **定位侧三项真差距**：① **地图资产**（无关键帧落盘、无动态残影清理、无描述子/地图包）；② **配准验收**（2D 条件数而非 3D 平移/旋转 Schur 谱；退化时直接跳过 NIS；无观测协方差/白化）；③ **重定位手段**（无描述子，LOST 后 AABB×360° 网格穷举 + KD-tree 打分）。另有"非静止初始化 / reset 协调 / UWB 无图模式"是差距，但前两者被 Lucifer"不重置 LIO"的架构部分规避，UWB 依赖硬件。
4. **导航侧 TDT 的剩余增量很小**：TDT nav-kit 三件招牌中，SFC 生成器（积分图实现更优）与碰撞迭代修复已被 Lucifer 移植并改造；Kinodynamic A* 已移植但**刻意未接入**（对全向底盘 + 短距低速机动无收益，`MINCO_README.md:69-74`）；SFC 在 Lucifer 里是"发布前全图校验 + 有界修复"而非硬约束——而 TDT 文档自己承认方盒走廊"并不保证不与障碍碰撞"（`MinimumSnap.md:196`）。真正值得拿的是：**LOS 视线保证的前端化简**、**分阶段性能基线方法论**、**定时掩码的"预测层"语义**。
5. **去掉许可约束后，最值得重新评估的是"直接复用 TDT 的工程件"**：离线清残影/地图包工具（PCL 实现，几乎可原样编译）、Scan Context 描述子模块、Schur 谱/白化/χ² 验收代码、状态机与 reset 协调都可以直接拿，P0-1/P0-2/P1-1 的工作量各降一档（M→S–M、S–M→S、M–L→M）。但**技术耦合仍是主要成本**：TDT 后端要求 GTSAM 4.2.x（Lucifer 是 4.3a0）、自定义前端 frame/session 协议、24k 行后端的参数与线程模型。若把整个后端接进来（其 legacy 接口支持 `/Odometry` + `/cloud_registered`），可作为 P2 的 A/B 对照方案。

**如果只做三件事**：

| # | 做什么 | 为什么是它 |
| :- | :- | :- |
| 1 | **地图资产工程化**：small_glim 关键帧落盘 → 离线动态残影清理 → 精修后重生成 PCD | 先验图质量直接决定 GICP 内点率、NIS 拒帧率、LOST 频率；纯离线、可量化，且 TDT 工具可直接移植 |
| 2 | **配准验收升级**：2D 条件数 → 3D 平移/旋转 Schur 谱 + 弱方向白化；**退化时仍做一致性检验（χ² 尾概率）** | 直指隧道/走廊误锁与"退化即失明"；TDT 的 `pose_conditioning`/`registration_solver`/`graph_innovation` 可直接参考移植 |
| 3 | **实时性兜底 + 性能基线**：MINCO 墙钟预算；A*/MINCO/SFC/MPC 分阶段 p50/p95 落盘 | 最坏 3 次修复 × (800+300) 迭代无 deadline，可能超过 8Hz 回调周期；没有基线，其余改动无法验收 |

> 注：许可放开后第 1、2 项可以直接复用现成实现，工作量各降一档；第 3 项本来就不涉及复用。
>
> **落地状态（2026-10-23）**：P0-1（地图清残影 + keyframe 落盘）、P0-2（3D Schur + χ² NIS 验收）、
> P0-3（MINCO 墙钟预算 + bench 脚本）、以及"描述子重定位"（Scan Context）**已实现并有单元测试**，
> 详见 [04-implementation.md](04-implementation.md)。未做：非静止初始化、reset 协调、UWB、位姿图、
> 输出协方差、动态障碍预测层。

---

## 1. 对比口径

| 项 | TDT | Lucifer |
| :- | :- | :- |
| 文章主题 | 定位（前后端架构、前端 ESKF、后端因子图/状态机/描述子/UWB） | — |
| 配套仓库 | `z-localization`（定位） | — |
| 用户指定配套 | `tdt-nav-kit`（导航前后端） | — |
| 平台 | ROS 2 Humble，Mid-360，GTSAM 4.2.x，8 核级工控机（文章：裸负载 CPU≈15%，内存换时间） | ROS 2 Humble + Gazebo，Mid-360（支持双雷达驱动级融合 + PTP），GTSAM 4.3a0，**8 核线程预算写死**（根 `README.md:18`） |
| 定位链 | ZLIO 前端 + 定位后端（map→odom 因子图） | small_glim（LIO）+ fast_location（先验图定位，唯一 map→odom） |
| 导航链 | 算法组件库（无 ROS、无控制层） | 10 节点组件化：A* + MINCO + SFC 校验 + OSQP MPC + 语义地图/隧道云台 |

> 注意：文章正文讲的是**定位**；`tdt-nav-kit` 是 TDT 同队的**导航**开源仓库（文章页未链接），两者是不同仓库，分析时分开对待。

---

## 2. 系统级对比

| 维度 | TDT | Lucifer | 判定 |
| :- | :- | :- | :- |
| LIO 前端 | ZLIO：ESKF + VoxelMap 平面矩 + 平面协方差/相关 + 时空自适应 + 非静止初始化 | small_glim：GTSAM ISAM2 固定滞后 + GICP/iVox + 逐点去畸变 + 100Hz IMU 补帧 | **对等（路线不同）**；TDT 在"面建模不确定性"上更严谨，Lucifer 在"批优化 + 高频补帧"上更工程化 |
| 初始化 | 多帧点云 + IMU 预积分联合窗口，不要求静止 | 1s 加速度均值定重力，**要求上电静止约 1s** | **Lucifer 落后**（后果被架构部分规避，见 P1-3） |
| 全局定位 | 关键帧位姿图（滑窗/边缘化/锚点）+ 多源因子 | 单帧 GICP → map→odom + EMA + 门控 | **TDT 结构更完备**；但单雷达 + 先验图场景下 Lucifer 的工程补偿已覆盖主要失效模式 |
| 状态机 | 五态（INITIALIZING/NORMAL/OBS_DEGRADED/SUSPECT/LOST）+ pose_trusted/冻结 | 二态 OK/LOST + reason 词表；**刻意不广播中间态** | **语义对等、形态不同**；不建议照搬对外五态 |
| 重定位 | Scan Context（30×90）召回 15 候选 + yaw 初值 + 几何验证 | AABB×360°/30° 网格（2m 步长）+ KD-tree 命中率 + top-k + margin + 时序验证 | **Lucifer 落后**（可做，P1-1） |
| 多源融合 | UWB 因子 + 在线坐标/时间标定 + 回环 + 重力因子 | **无 UWB、无回环、无全局因子图** | **Lucifer 缺失**（UWB 硬件条件；图=换骨架，观察） |
| 地图资产 | 关键帧包 + 描述子 + 自动清残影 + 变换/过滤/重建工具 + 量化评测 | 单张 PCD（Ctrl-C 落盘）+ 语义 msgpack；**无清理工具** | **Lucifer 落后**（优先项，P0-1） |
| 配准验收 | 点到面 + Huber；Schur 平移/旋转谱、预条件、白化、NIS+χ²、部分秩；初始/重定位要求完整位姿 | FastGICP 多尺度级联；2D 条件数投影；固定阈值 NIS=12；**退化时跳过 NIS**；无协方差 | **Lucifer 落后在粒度**（P0-2） |
| 导航前端 | A*（8 邻域 + 势场代价图 + 多层定时 Mask + DP/直线化简 + LOS 检查） | A*（8 邻域 + EDT clearance + 转角/隧道轴向代价 + 切角检查 + 分层重规划） | **Lucifer 更安全/更工程化**；TDT 的 LOS 化简值得拿 |
| 导航后端 | Minimum Snap（闭式/OSQP + SFC 方盒 + 碰撞迭代 + 时间归一化 + 控制点速度） | MINCO S3NU（L-BFGS + ESDF 软代价 + 隧道走廊 + 两阶段 v/a 精确检查）+ SFC 全图校验 + 有界修复 | **路线不同、Lucifer 不弱**；TDT 自己结论"梯度后端上限更高"（`MinimumSnap.md:405`） |
| 控制/执行 | 无 | OSQP MPC + 弧长速度剖面 + 多假设进度跟踪 + 恢复链 FSM | **Lucifer 独有** |
| 场景语义 | 无 | 语义 msgpack（三态 terrain + 隧道 + 轴线 + 净高/净宽/限速）+ 云台收放 + MPC 门控等待 | **Lucifer 独有** |
| 双雷达/时间同步 | 未见（单雷达 + 官方驱动） | 驱动级融合（20ms 窗口/超窗降级）+ PTP（实测 0.42 ppm） | **Lucifer 独有** |
| 测试/仿真/构建 | 155 gtest + 36 pytest；无 ROS 集成/仿真栈 | **307 gtest + 53 pytest**；Gazebo 双场景；`build.sh` 固定编译顺序 + 覆盖检查；21 份分层配置 | **Lucifer 领先**（短板：small_glim 0 测试、无 CI） |

---

## 3. 定位：真差距 vs 伪差距

### 3.1 架构对照

```text
TDT:   LiDAR+IMU ─► ZLIO 前端(ESKF/VoxelMap) ─► /frontend/frame(点云+位姿+代次)
                                                    │
                                                    ▼
        先验图/描述子/UWB/回环 ─► 关键帧位姿图(ISAM2, 滑窗+边缘化+锚点) ─► map→odom + 五态 + 协方差

Lucifer: LiDAR+IMU ─► small_glim(ISAM2 固定滞后 + GICP/iVox) ─► odom→base_link + /lio/robo/odom
                                                              └─► /small_glim/deskewed_cloud
                                                                        │
        先验 PCD ─► fast_location: FastGICP 级联 + 退化投影 + NIS + EMA + 跳变门 + 全场搜 ─► map→odom (唯一)
```

### 3.2 关键判定（完整 20 项见 [01-localization.md](01-localization.md) §2）

| 维度 | 判定 | 一句话理由 |
| :- | :- | :- |
| 前端估计器（ESKF vs 因子图） | 已有等价能力 | 两者都是"IMU 预积分 + 观测更新 + 局部地图"；Lucifer 的 ISAM2 批优化不弱 |
| 平面矩 VoxelMap / 平面协方差 | 理论差距、不建议替换 | 与 GICP 残差结构绑定，替换是架构级改动（L），收益不确定 |
| 去畸变 | 小差距（P2） | TDT 分段二次轨迹 vs Lucifer IMU 位姿插值；200Hz 下差异有限 |
| 时空自适应采样 | 小差距（P2） | TDT 按运动/信息矩阵/场景尺度自适应；Lucifer 固定分辨率 + 100Hz 补帧 |
| 初始化 | **真差距** | 需静止 1s；TDT 不要求静止 |
| 后端结构 / 因子 | **真差距但大工程** | 单帧配准 vs 关键帧图；Lucifer 无锚点/多源融合/协方差 |
| 配准求解 | **真差距** | Lucifer 无鲁棒核、无 Schur 分块/预条件/白化 |
| 观测验收 | **真差距（逻辑缺口）** | 退化时跳过 NIS；固定 3 自由度阈值 |
| 描述子重定位 | **真差距** | 无描述子；全场网格穷举 |
| 状态机导出 | 语义对等 | `OK nis_reject`/`LOST` 分别覆盖 OBS_DEGRADED/SUSPECT；刻意不广播中间态（`fast_location/README.md:237-238`） |
| reset 协调 | **真差距** | 无 reset 服务；LIO 发散只能重启整栈 |
| UWB / 无图 | **真差距（硬件条件）** | 全仓无 UWB；无图模式不存在 |
| 回环 | 差距（仅无图/建图场景需要） | 有先验图时不需要 |
| 地图工具链 | **真差距（优先）** | 只有单张 PCD + 语义标注 |
| 输出协方差 | **真差距（改动小）** | `/Odometry` 与 map→odom 都不填 covariance |

---

## 4. 导航：已吸收与剩余

### 4.1 TDT nav-kit 各机制在 Lucifer 的状态

| TDT 机制 | Lucifer 状态 | 判定 |
| :- | :- | :- |
| 8 邻域 A* + 势场代价 | 已接入；Lucifer 用 EDT clearance + 转角/隧道轴向代价，且有切角保护 | **Lucifer 更优** |
| DP + 直线（LOS）路径化简 | 未接入；Lucifer 只有"段内最大代价"prune（默认关）+ 0.18m resample | **值得拿**（P1-4） |
| SFC 方盒走廊 | 已移植生成器（积分图 O(log r)，修了 TDT 的 origin 符号/FLT_MAX 问题），但**只做发布前校验** | **形式不同、收益已覆盖**；不建议硬约束 |
| 碰撞迭代修复（插中点/缩走廊） | 已改造为"有界修复"（放大 data_weight / 插路标点，预算内回退） | **已覆盖** |
| Kinodynamic A* | 已移植、**刻意未接入**（对全向底盘无收益） | 保持现状 |
| Minimum Snap 闭式/OSQP | 未采用；Lucifer 用 MINCO L-BFGS（无约束 + 软代价） | **不建议替换**（TDT 自己也说梯度后端上限更高） |
| 时间归一化 | 未采用；MINCO 分段三次无该病态 | **不适用** |
| 控制点硬速度约束 | 未采用；Lucifer 在控制层用 speed_profile 曲率限速 + 隧道 [vmin,vmax] | **不需要** |
| 多层/命名/定时 Mask | 未采用；Lucifer 用逐格 hit/miss 时间衰减（keep 0.8s / clear 1.2s），有记忆无预测 | **只取"预测层"语义**（P1-5，条件） |
| 1000 次随机起终点性能基准 | Lucifer 有 `PerformanceMonitor` 但默认关、无落盘基线 | **值得拿**（P0-3） |

### 4.2 结论

TDT nav-kit 是"算法原型库"，Lucifer 是"完整导航栈"；Lucifer 已经吸收其最可迁移的两件半，且实现质量更高（A* 切角、EDT 缓存、积分图 SFC、软代价 + 校验 + 修复 + 回退）。**剩余增量不在算法移植，而在工程基础设施与两个小算法点**：LOS 化简、性能基线、墙钟预算、动态障碍预测层（条件触发）。

---

## 5. 可提升项清单（按优先级）

> 改动量口径：S < 1 天、M 1–3 天、L > 3 天（1 人）。收益/风险按 1–5 分（收益=对比赛鲁棒性/精度/工程速度；风险=破坏现有栈/硬件依赖/返工概率）。**工作量已按"许可放开、可直接复用 TDT 代码"重新估算，并在每项标注复用可省掉什么。**

### P0（建议立即做）

#### P0-1 地图资产工程化：关键帧落盘 + 自动清残影 + 精修重生成（S–M，收益 5 / 风险 1）

- **现状**：建图只在 Ctrl-C 时合并落盘单张 PCD（`src/localization/small_glim/src/mapping/async_mapping.cpp:322`）；**没有**动态残影清理工具；先验图里的残影（建图时的人/车）会直接变成 GICP 误配准源。语义地图编辑服务的是导航，不改善定位图。
- **TDT 做法**：建图保存关键帧包（`keyframes.txt` + `clouds/` + 描述子，`map_bundle.cpp:35-77`）→ `clean_map_bundle` 用多帧可见/自由空间证据清残影（`tools/cleanup/pipeline.cpp:117,134`）→ CloudCompare 精修 → `build_pcd_map_bundle` 按网格重生成地图包与描述子（`README.md` 建图流程第 3–5 步）。
- **可直接复用**：TDT 的 `clean_map_bundle` / `build_pcd_map_bundle` 是**独立 C++ 工具 + PCL**，不依赖其前端/后端运行栈；把输入从 "bundle" 换成 Lucifer 的 PCD（或先让 small_glim 落关键帧）即可。省掉清理算法设计与参数摸索（约 1–2 天）。
- **Lucifer 落点**：`small_glim` 增加"关键帧点云 + 位姿落盘"（复用 `keyframe_trans_thresh/rot_thresh`，S）；`src/bringup/tools/` 移植/改造离线清理工具（S–M）；清理参数按场地标定。**不引入 bundle 格式**，先做最小闭环。
- **收益**：内点率↑、NIS 拒帧↓、LOST↓、代价地图假障碍↓、语义图人工标注量↓；对适应性训练的脏场地收益最大。
- **风险/回滚**：清理过度会削掉墙/顶导致欠约束——保留原始 PCD 与"清理前后"对比图即可回滚。

#### P0-2 配准验收升级：3D Schur 谱 + 弱方向白化 + 退化时仍做一致性检验（S，收益 4 / 风险 1）

- **现状**：`runICP` 只有 2D 平移 Hessian 条件数 + 可观测方向投影（`alignment_quality.hpp:200-243`），软阈值 80、硬阈值 5000；**`enable_nis_ && !last_assessment_.is_degenerate` 才做 NIS**（`robot_localization.cpp:939`）——退化场景完全没有一致性检验；NIS 固定 3 自由度、阈值 12、先验仅对角 0.2m/8°；无观测协方差输出。
- **TDT 做法**：点到面 + Huber；对平移/旋转各做 3×3 Schur 谱分解、轴对齐强度、弱轴标记与谱封顶预条件（`pose_conditioning.cpp:32-153`）；按"保留秩 + 残差方差"构造白化信息矩阵、弱方向降秩、共模协方差地板（`registration_solver.cpp:230-310`）；NIS + χ² 上尾概率联合验收（`graph_innovation.cpp:19-54`）；**初始/重定位要求完整位姿通过验收**（正常跟踪可只用部分方向约束）。
- **可直接复用**：`pose_conditioning.cpp`（约 200 行）、`graph_innovation.cpp`（约 60 行 + boost χ²）、`registration_solver.cpp:230-310` 的白化/降秩逻辑，都可改造成 Lucifer 的数据结构后直接进 `alignment_quality`；`fast_gicp` 已能拿 `getFinalHessian()`，接口天然对得上。
- **Lucifer 落点**：`alignment_quality.hpp` 增加 6×6 观测协方差 + 白化/弱方向接口；`runICP` 的退化分支改为"按弱方向降权后仍做 χ² 检验"；`globalLocalization` 首配/重定位要求 6DoF 完整通过；参数进 `src/bringup/config/common/fast_location.yaml`。
- **收益**：隧道/长走廊不再"退化即失明"，减少沿隧道方向的错误修正与整拍冻结；为下游输出可信度（配合 P1-6）。
- **风险/回滚**：协方差估计错误会放大误配准——阈值必须用 bag 回放标定，保留旧逻辑开关。

#### P0-3 实时性兜底 + 分阶段性能基线（S，收益 4 / 风险 1）

- **现状**：MINCO 修复最多 `1 + repair.max_attempts(2) = 3` 次完整优化（`minco_path_smoother_node.cpp:320-419`；`params/navigation2.yaml:237`），每次一阶段 800 迭代 + 可选二阶段 300 迭代（`yaml:170,179`）；**MINCO 平滑链路无 deadline/time_budget**（`minco_path_smoother_node.cpp` / `minco_optimizer.cpp` grep 零命中；MPC 侧 OSQP 的 `time_limit` 不约束平滑器），最坏耗时可越过 8Hz/125ms 回调周期 → 路径陈旧 → MPC veto → 重规划的放大回路。`PerformanceMonitor` 默认关（`yaml:155,394`），无落盘基线。
- **TDT 做法**：`iterate` 同时受 `maxIter` 与 `timeLimit` 约束（`minimumSnap.cpp:125-163`）；`benchTime` 记录分阶段 avg/max，固定地图 1000 次随机起终点出表（`README.md:90-113`）。
- **Lucifer 落点**：① `smoothAndPublish` 取采样起点，参数 `repair.time_budget_ms`，在修复/二阶段每轮前检查，超预算走已有"回退原始路径"分支（`node:403-419`）；② 新增离线 benchmark（复用组件接口，固定起终点集合，输出 A*/MINCO/SFC 构建/校验/修复/MPC solve 的 p50/p95/p99 与 `global_clearance`/`safety_reject`/`optimize_failed` 次数）→ CSV + 回归对比。
- **收益**：最坏回调延迟有界；为所有后续改动提供"改前/改后"判据。
- **风险/回滚**：预算过紧会让修复退化为不做——按基线 p95 定阈值，保留开关。

### P1（下一轮）

#### P1-1 描述子重定位（Scan Context 式，先离线后在线）（M，收益 4 / 风险 3）

- **现状**：LOST 后全场 AABB×360°/30° 网格（2m 步长）+ KD-tree 命中率 + top-k=6 + margin=0.03 + ICP + 时序验证（`global_search.cpp`），候选上限 200 万；无离线描述子库，对称场地靠 `score_tie_epsilon`/mean_error 兜底。
- **TDT 做法**：SC 30 环 × 90 扇区、3 份平移增强、15 候选、距离门；小地图直接遍历；离线按网格从精修 PCD 重生成描述子（`scan_context.hpp:16-31`；`build_pcd_map_bundle`；文章 §3.3）。作者明确 G-PROBE 未充分测试、不建议照搬（`docs/design.md:354-364`）。
- **可直接复用**：`src/backend/src/scan_context/`（描述子构造 + yaw 估计）约 300–400 行，接口是"点云进、候选出"，与 Lucifer 的 PCD/map provider 可对接；离线生成逻辑可从 `tools/build.cpp` 裁剪。相比从零写，省约 2–3 天（主要是参数与重力矫正标定）。
- **Lucifer 落点**：先做**独立离线工具**（输入 Lucifer PCD + bag，输出描述子索引 + recall@k 报告，不改 fast_location 主路径）；验证通过后再把地图加载抽象为 `map_provider`，在 `global_search` **前置**候选检索层，最终仍走现有 GICP + margin + 时序校验。
- **收益**：LOST 恢复耗时与 ICP 次数下降；对称场地区分度提升。
- **风险/回滚**：SC 对高度分布/重力矫正敏感，地图精修后需重建；双轨地图资产维护成本。

#### P1-2 reset 协调：small_glim 可重置 + fast_location 编排（S–M，收益 4 / 风险 3）

- **现状**：`fast_location` 无任何 service；`small_glim` 无 reset 接口；明确"LOST 不重置 small_glim"（`robot_localization.cpp:1276-1289`）。若 LIO 自身发散，唯一出路是重启整栈（丢 odom 系）。
- **TDT 做法**：后端编排 `/localization_reset` → 前端暖重置（保留 IMU 零偏/速度、重锚 odom 原点、返回边界变换），事务身份保证幂等（`reset_coordinator.cpp:180-260`；`docs/design.md:404-411`）。
- **可直接复用**：TDT 的 reset 协议/状态机（`reset_coordinator` 约 500 行）可直接裁剪成 Lucifer 的 `/localization_reset` 服务 + small_glim 的 reset/epoch 接口；省掉协议设计与幂等边界推理（约 1–2 天）。
- **Lucifer 落点**：small_glim 增加 reset 服务（复用 `InitialStateEstimation::set_init_state`）+ 前端 epoch；fast_location 增加编排：冻结输出 → 请求 LIO 重置 → 重新全局定位 → 新锚点。先在仿真验证。
- **收益**：前端发散后真正自愈，而不是带着漂移继续配准。
- **风险/回滚**：动 LIO 生命周期，时序/初始化竞态风险高——先做最小版（冷重启 + 边界位姿），事务幂等后补。

#### P1-3 非静止初始化（M，收益 3 / 风险 2）

- **现状**：`naive_initialization` 取 1s 加速度均值定重力，README 明示"上电后需静止约 1 秒"（`small_glim/README.md:34-36`）；云台跟随自瞄转动时初始姿态会偏。
- **TDT 做法**：短窗 IMU 重力初值 → IMU 传播 + 点云粗配准 → 多帧共面约束 + 预积分联合窗口优化位姿/速度/零偏/重力，每轮重新去畸变（`initialization.cpp:246-345`；`docs/design.md:112`）。
- **复用判断**：这段**不能直接复用**——它绑定 ZLIO 的 ESKF 状态、VoxelMap 与残差定义，搬到 small_glim 的 GTSAM 因子图等于重写。工作量不因许可放开而下降。
- **Lucifer 落点**：small_glim 增加 `windowed` 初始化分支（GTSAM 已在依赖内），失败回退 naive；或先做低成本兜底：fast_location 首配用多帧 + 多候选交叉验证。
- **收益**：适应性训练/重启不再要求停车静止。
- **风险/回滚**：初始化 1s 内无输出；与 odom 原点 z 锚点契约、地面分割/costmap 的 z=0 假设耦合，必须同步验证。

#### P1-4 LOS 视线保证的前端化简（S–M，收益 3 / 风险 2）

- **现状**：`path_prune_enabled: false`（`yaml:63`），`/plan_raw` 是逐格路径 + 0.18m 重采样（`yaml:65`），10m 路径约 55 段，直接决定 MINCO 规模；现有 prune 只看段内最大代价值，不校验视线。
- **TDT 做法**：DP 化简递归检查 `lineInObsticle`，失败保留中间点；再做方向性弦优化，保证化简后线段无碰（`yastar.cpp:195-276`；可直接移植）。
- **Lucifer 落点**：`global_planner_node.cpp` 的 `buildPath` 在 resample 前插入 LOS 化简（用现有可通行性/射线检查），新参数 `path_los_simplify_enabled`。
- **收益**：段数 55 → 约 10–20，MINCO 一阶段耗时近似线性下降，省出的预算给修复/二阶段。
- **风险/回滚**：控制点减少会弱化"贴 A* 路径"的约束，可能更贴墙/切角——用基准对比最小 clearance、长度、SFC 拒绝率，保留开关。

#### P1-5 动态障碍预测层（条件触发；基础设施 S，预测源 M/L，收益 3 / 风险 3）

- **现状**：局部代价图逐格 hit/miss 时间衰减（keep 0.8s / clear 1.2s，`yaml:298-300`），全局图 12 帧整数衰减 + 2s 观测超时——**有记忆、无预测**；决策只有位置 EMA（`target_input.cpp:135-145`），无速度/轨迹预测；无 Mask 层概念。
- **TDT 做法**：多层/命名/定时 Mask：按传感器帧整层过期、按 `duration`/`maxCount` 回收，可把"预测位置"作为独立层推入而不污染基础图（`yastar.cpp:451-505`；`Usage.md:336-354`）。
- **Lucifer 落点**：分两阶段——A. 基础设施（S）：代价图增加"预测层"入口（新话题 `/predicted_obstacles`：点集 + TTL，写入现有 `hit_age` 体系，不建第二张地图）；B. 预测源（M/L）：点云聚类跟踪，或先只扩展现有 `TargetInput`（位置 EMA → α-β 速度）。
- **收益**：对高速横向接近的敌人提前让行，减少 MPC veto/恢复。
- **风险/回滚**：误预测导致绕远/抖动/规划振荡——**先用 P0-3 基线证明动态障碍相关的 veto/恢复占比可观（如 >5%）再做**，否则不做。

#### P1-6 输出协方差与诊断词表（S，收益 3 / 风险 1）

- **现状**：`/Odometry` 从不写 `pose.covariance`（`small_glim_node.cpp:374-420`）；fast_location 只发 TF；对外二态 + reason 词表。内部其实已有 `nis_reject/weak_obs/projected/frontend_diverged` 等诊断。
- **TDT 做法**：前后端都输出位姿协方差与 key=value 状态字典；五态 + pose_trusted/冻结谓词。
- **Lucifer 落点**：small_glim 用 ISAM2 边缘协方差（或固定滞后窗末位）填 `pose.covariance`；fast_location 输出 map→odom 协方差（可先由白化后的观测协方差近似）；**不改对外二态**，只在 reason 里补 `hold_conflict/hold_weak/recover_local/recover_global` 等词，供日志与后续策略用。
- **收益**：为 P0-2 铺路（没有协方差谈不上白化验收）；下游未来可做分级降速。

### P2（观察/条件项）

| 项 | 说明 | 结论 |
| :- | :- | :- |
| 关键帧位姿图 | TDT 的滑窗+边缘化+多源因子；= 给定位栈换骨架，需 fast_location 产因子、small_glim 开放关键帧 | **观察**；先做 P0-1/P0-2。若做，可直接复用 `pose_graph`/`graph_*` 模块，但受 GTSAM 版本（TDT 4.2.x vs Lucifer 4.3a0）与前端协议约束 |
| TDT 后端整体接入（A/B 基线） | TDT 后端有 legacy 接口（`/Odometry` + `/cloud_registered`），理论上可把 small_glim 的里程计 + 去畸变云喂给它，作为第二条 map→odom 来源做对照 | **可选**：不改主链路，独立进程跑，能拿到同场景下"因子图 vs 单帧配准"的真实数据；部署成本 M–L（GTSAM 4.2 需独立前缀/容器） |
| UWB / 无图定位 | TDT：UWB 因子 + 在线坐标/时间标定 + 轨迹 yaw 可观性 + 重力因子 | **硬件条件**；无 UWB 不做。代码可直接复用（`uwb_alignment`/`uwb_fusion` 约 600 行），但仍需全局因子图（上两项）才完整 |
| SFC 软走廊项 | 把 `usableBoxAt` 方盒做成 MINCO 带解析梯度的软惩罚（不做硬约束） | P2，先看 P0-3 的 SFC 拒绝率 |
| MINCO 起点状态接里程计 | 当前首尾状态写死零速度/零加速度；新路径都从静止起步 | P2，需 A/B 验证收敛 |
| 时间分配引入曲率限速 | 当前距离/转角固定分配，与 MPC speed_profile 的曲率限速意图不一致 | P2 |

---

## 6. 不建议照搬（技术判断）

> 许可不再是约束，以下均基于"技术收益 < 工程成本/风险"。

1. **SFC 硬约束 + 交替迭代**：需要把无约束 L-BFGS 换成 SQP/内点；而 TDT 自己承认方盒走廊"不保证不与障碍碰撞"，仍需碰撞迭代兜底。Lucifer 的"软 ESDF + 双校验 + 有界修复 + 回退"已覆盖同一收益且更简单。
2. **闭式解/双后端**：闭式只能处理等式约束、不能表达走廊；TDT 自己也说"闭式不建议配 JPS 前端"（`MinimumSnap.md:403`）。MINCO L-BFGS 已是梯度法，无需再开一条维护面。
3. **Kinodynamic A* / 混合搜索接入**：状态空间大两个数量级、全局搜索约 2s；对全向底盘 + 短距低速机动无收益（Lucifer `MINCO_README.md:69-74` 的判断成立）。
4. **时间归一化**：只对高次多项式 + 物理时间幂次的病态有意义，MINCO 分段三次不适用；TDT 自己也说归一化结果"相对较差"（`MinimumSnap.md:174`）。
5. **独立 Mask 地图**：会形成"规划器一套动态障碍语义、MPC 另一套"，违反 Lucifer 的几何权威约定（`navigation2/README.md:88-92`）。只取"按层 + TTL 推预测位置"的语义。
6. **G-PROBE**：作者明确未充分测试、召回率不足，慎重（`docs/design.md:354-364`）。
7. **在线动态点滤除**：TDT 试过放弃（`docs/design.md:272-278`）；Lucifer 也是纯几何，别为此加开销。
8. **重写 small_glim 为 ESKF / 换 iVox**：成本 L、风险高、收益不确定；Lucifer 的 GTSAM 固定滞后 + 100Hz 补帧已满足比赛需求。
9. **照搬五态对外契约**：Lucifer 已刻意撤销中间态，下游 `test_integrity_gate*.py` 与 MPC 门控只按 `LOST` 动作；照搬会同时破坏测试与下游语义。只吸收诊断维度。

**复用 ≠ 零成本**：TDT 代码的耦合点必须提前处理——① GTSAM 版本（TDT 后端 `find_package(GTSAM 4.2)`，Lucifer 是 4.3a0；TDT 还为 4.2 smoother 缺陷写了专门规避，移植前需在 4.3a0 重测）；② 自定义前端 frame/session 协议（直接接后端需做协议适配，或走 legacy `/Odometry` + `/cloud_registered`）；③ 第三方依赖（boost `chi_squared`、PCL、OpenMP/TBB）与 8 核线程预算；④ 代码风格/日志/参数体系差异。**推荐"按模块 vendor"而不是"整体替换"**：先把离线工具与纯几何模块（scan_context、pose_conditioning、cleanup）拿进来，主链路保持不动。

---

## 7. 落地路线与验证

### 7.1 节奏（依赖关系）

```text
M1（1–2 周，纯离线）：P0-1 关键帧落盘 + 清残影工具（可复用 TDT 工具）+ 量化评测；P0-3 性能基线   ← 无依赖，先做
M2（1–2 周，低风险在线）：P0-2 配准验收升级（复用 pose_conditioning/whitening）+ P1-6 协方差/诊断词表
M3（2 周，离线+仿真）：P1-1 描述子离线工具与 recall@k（复用 scan_context）；P1-3 非静止初始化；P1-4 LOS 化简
M4（仿真）：P1-2 reset 最小版（复用 reset 协调语义）；P1-5 预测层（仅当 P0-3 基线证明值得）
（可选）P2：TDT 后端作为第二条 map→odom 来源跑 A/B；真位姿图/UWB 等需求明确后再评估
```

### 7.2 指标（所有项共用）

| 指标 | 定义 | 采集 |
| :- | :- | :- |
| 定位 ATE / RMSE | 与真值逐点误差（分 XY / yaw） | 仿真接真值插件；实车用标记点复核 |
| 重定位耗时 | 判 LOST → 首次回 OK（p50/p95/p99） | `/localization_status` 时间戳差分 |
| 重定位成功率 / 误锁率 | 成功回 OK / LOST 事件总数；对称场地错锁次数 | 同上 + 人工标注 |
| LOST 误报率 | 真值误差仍小却被判 LOST 的时长占比 | 真值 + 状态流对齐 |
| 地图质量 | 清理前后点级 precision/recall、体素级 preservation/rejection | 离线工具 + 人工标注快照 |
| 规划/跟踪 | 成功率、到达时间、最小 clearance、SFC 拒绝率、修复触发率、veto/恢复次数 | benchmark + bag 离线统计 |
| 时延/资源 | A*/MINCO/MPC 单拍 p50/p95/p99、CPU（%/核）、RSS | `performance.*` 落盘 + pidstat |

### 7.3 验收入口（现有资产）

```bash
# 定位离线回放（mapping + nav 两条链已有）
ros2 launch bringup sim.launch.py world:=RMUL mode:=mapping nav_rviz:=False
ros2 bag play /abs/path/site_loop --clock

# 记录评测数据
ros2 bag record -o /tmp/loc_eval /tf /tf_static /map /localization_status \
  /lio/robo/odom /small_glim/deskewed_cloud /plan /cmd_vel_chassis

# 配置契约静态检查（现成）
python3 src/bringup/tools/check_nav_contract.py .

# 单包测试（现成）
colcon test --packages-select fast_location navigation2 --event-handlers console_direct+
```

现有测试资产：navigation2 29 个测试文件（含 11 个 launch_test）、decision 10 个、fast_location 4 个；**短板**：small_glim 0 测试、无 CI。新增提升项都应先补单测（`alignment_quality_test.cpp` 可扩退化/白化用例；MINCO 加"预算耗尽回退"用例；LOS 化简加"化简后仍无碰"用例）。

---

## 8. 分册索引与边界

| 分册 | 内容 | 行数 |
| :- | :- | :- |
| [01-localization.md](01-localization.md) | 定位 20 项逐项对比（含前端 ESKF/初始化/去畸变/地图/时空自适应）、P0/P1/P2 清单、直接复用清单与技术耦合、故障注入验证方案、证据索引 | 276 |
| [02-navigation.md](02-navigation.md) | 导航逐项对比、TDT 五问确定答案（Kastar 未接入/SFC 仅校验/无 Mask 层/时间分配/v-a 约束）、8 项提升、8 项不建议、证据索引 | 391 |
| [03-engineering.md](03-engineering.md) | TDT 后端 adoption matrix（A1–A8）、Lucifer 优势盘点（双雷达/PTP、语义图、测试规模等）、离线/上车验证方案与指标、代码复用与移植策略 | 394 |
| [04-implementation.md](04-implementation.md) | 落地实现报告：四项改动的 API/参数/接线/验证结果、回退开关、调参与下一步 | 见文件 |
| [05-audit.md](05-audit.md) | 审计总报告：地面分割 / 去车身 / fake_vel_transform 的结论、已修项、待决策项（含 costmap 20cm 高台推导） | 见文件 |
| [audit-ground.md](audit-ground.md) / [audit-body.md](audit-body.md) / [audit-fakevel.md](audit-fakevel.md) | 三份分册：逐项证据、推导、测试与验证清单 | 350/325/358 行 |

**边界声明**：本次分析未编译、未启动 ROS、未运行任何仿真或实车实验；TDT 的性能数字来自其 README/文章，Lucifer 的收益为代码级工程推断。所有"未见"均指本机 `grep` 范围内无命中，不代表外部依赖或未入仓代码不存在。**按用户要求，全文不考虑许可证/版权影响。** 下一阶段建议以 P0-1/P0-3 先建立数据基线，再决定 P1 项的投入顺序。
