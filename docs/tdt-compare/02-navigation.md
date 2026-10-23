# 02 · TDT nav-kit 与 Lucifer navigation2 导航对比

> 对比对象：`/home/cola/tdt-nav-kit-main` vs `/home/cola/Lucifer_nav/src/navigation/navigation2`。
> 方法：只读源码与文档取证，未改任何源码/配置；引用统一为 `文件:行号`，查不到的写"未见"。
> 修订口径：按用户要求，**不考虑许可证/版权影响**；`tdt-nav-kit` 代码按技术成本直接复用即可（本册无许可相关内容）。
> 口径说明：RoboMaster 社区文章正文 `/tmp/article.txt` 是**定位**主题（ESIKF、弱观测、Schur 补、DCReg 预条件，
> article.txt:36、125），不含导航算法；用户指定的"文章配套部分"即上述导航仓库。因此本文全部导航结论以仓库
> 代码/文档为准，不以文章叙事为准。

## 0 摘要

**一句话结论**：两套系统是同一工程哲学的两个完成度——TDT 是可移植的算法原型库，Lucifer 已把其中最可迁移的
三件（SFC 生成器、碰撞迭代修复、Kinodynamic A*）吸收了两件半；**剩余可提升空间有限，且不在算法移植，而在
工程基础设施（求解预算、性能观测、前端化简）**。TDT 的几个"招牌机制"移植到 Lucifer 后收益递减的原因逐项见 §4。

五个待核实事实的确定答案：

- **(a) kinodynamic_astar：已移植、未接入任何运行链路**。头文件自述来源 TDT-nav-kit（include/kinodynamic_astar.hpp:6-31），
  全仓库只有 `test/test_kinodynamic_astar.cpp:116-243` 调用 `kinodynamicSearch`，CMake 里只挂 gtest（CMakeLists.txt:181-184）；
  `grep -rn 'kinodynamic'` 在节点源码中零命中。MINCO_README.md:69-74 明确记录"当前未接入任何运行链路"并给出理由。
- **(b) SFC：Lucifer 是"发布前全图安全校验（global_check）+ 有界修复"，不是优化硬约束**。证据：
  minco_path_smoother_node.cpp:642-684（校验）与 336-419（修复后重优化）；MINCO_README.md:22-27。TDT 的 SFC 是
  OSQP 中控制点位置的硬箱式上下界（minimumSnap.cpp:785-905）。TDT 文档自己承认方盒"不是真正的安全走廊，
  不能保证轨迹不与障碍碰撞"（MinimumSnap.md:196），仍必须靠碰撞迭代兜底——所以把它搬进 MINCO 的 L-BFGS 是
  "用大工程换一个仍需事后校验的软保证"，**不建议**（§4.1）。
- **(c) 动态障碍：Lucifer 没有 TDT 式多层/命名/定时 Mask，代价地图有时间记忆但无预测**。`grep -rn 'mask'` 在
  navigation2 的 src/include 下零命中；局部代价图用逐格 hit/miss 衰减（`hit_inc`、`dynamic_decay.keep_time=0.8`、
  `clear_time=1.2`，params/navigation2.yaml:292-300；local_costmap_node.cpp:620-700），全局图为固定整数衰减 12 帧
  + 2 s 观测超时（params:129-131）。`src/decision` 只有位置 EMA 滤波、**没有速度/预测**（target_input.cpp:135-145、
  target_input.hpp:53-80），它下发的是追击目标点（decision_node.cpp:711-725），导航侧不消费任何敌方预测。
- **(d) 时间/速度约束**：TDT = 梯形距离预分配（minimumSnap.cpp:281-305）+ 可选时间归一化（MinimumSnap.md:210-354）；
  Lucifer = 距离/速度 + 转角附加的**固定预分配**，时间不进优化变量（include/minco_time_allocation.hpp:13-19；
  minco_path_smoother_node.cpp:302-303），v/a 靠**两阶段精确求根检测 + 放大段时间重优化**（node:422-548，TDT 没有）。
  TDT 的时间归一化对 MINCO 参数化不适用（§4.4）；TDT 的控制点硬速度约束在 Lucifer 的等价物是控制层
  speed_profile 曲率限速 + 隧道 [vmin,vmax] 窗口（speed_profile.hpp:9-12、95-115；mpc_controller_node.cpp:288-297）。
- **(e) 路径化简**：TDT = DP 化简 + 带视线检查的直线二次化简（yastar.cpp:195-276；yastar.hpp:193-196），在求解前使用
  （main.cpp:133-135）。Lucifer = prune（代价值阈值贪心，yaml:63 默认**关**；global_planner_node.cpp:366-402）+
  resample 0.18（yaml:65；global_planner_node.cpp:429-465）+ stitch（README.md:64；global_planner_node.cpp:792-831）。**实质差异**：TDT 直线化简用
  `lineInObsticle` 保证线段无碰；Lucifer prune 只看段内最大代价值、无视线保证，且当前关闭。这是少数值得借鉴的
  算法细节（P1-1）。

**最有价值的 4 项**：P0-1 分阶段性能基线、P0-2 平滑器墙钟预算与降级、P1-1 LOS 保证的前端化简前置、P1-2 动态障碍
预测层（条件触发）。**明确不建议照搬**：SFC 硬约束、闭式解后端、Kastar/混合搜索、时间归一化、独立 Mask 地图、
控制点速度约束（§4）。

## 1 双方架构

### 1.1 TDT nav-kit：可裁剪的算法组件库（无 ROS、无控制层）

- 定位：面向二维栅格的算法组件，"可以根据实际工程需要选择其中的一部分接入"（README.md:5）；前端 YAstar
  （普通 A* + Kinodynamic A*），后端 MinimumSnapOsqp（README.md:13-14）。
- 地图/代价：0.05 m 栅格；占用图 + 人工势场生成代价图（`initCostMap`，yastar.cpp:760-800；
  `setCostField`，yastar.hpp:181-182、331-348），势场函数 `f(d)=0.5/(0.1+d)`（main.cpp:108-111）。
- 动态障碍：`Mask` 位打包多层掩码（yastar.hpp:20-49），支持命名层、定时有效期、按组最大数量
  （`setMaskMapTimed`，yastar.cpp:451-505；Usage.md:336-354）。
- 前端搜索：8 邻域加权 A*，`g` 用代价图 × 对角系数（无转向惩罚），`h` 为格距离（yastar.cpp:802-882）。
- 动力学前端：状态 `[x,y,vx,vy]`、加速度控制、三次多项式 shot、终点速度 0（kinodynamicAstar.hpp:11-72；
  Astar.md:58-80）。文档描述 A*→Kastar 的"混合搜索"（Astar.md:82-86），但仓库内**未见实现**（grep 无 hybrid；
  main.cpp:398-432 只是独立 demo）。
- 后端：6 参数多项式、3 阶导数（minimum jerk；main.cpp:145-146），闭式/OSQP/OSQP+走廊三种后端
  （minimumSnap.hpp:33-42）；方形安全走廊 + 碰撞检测交替迭代（MinimumSnap.md:191-208；minimumSnap.cpp:109-171、
  907-995）；时间归一化与速度约束为可选能力（minimumSnap.hpp:123-131）。
- 数据流（main.cpp:96-175）：图 → 势场/代价图 → A* → DP/直线化简 → 时间分配 → SFC → QP 求解 → 采样 + 碰撞迭代。

### 1.2 Lucifer navigation2：完整 ROS 2 导航链路（含控制/执行层）

- 10 个组件节点：地图、全局规划、全局/局部代价图、MINCO 平滑、MPC、速度平滑、nav2 兼容、隧道姿态、云台可视化
  （README.md:25-35）。整车建模为二维"虚拟导航云台"（README.md:5-7），与 TDT 的全向平移模型一致，因此算法可对比。
- 规划链：A*（8 邻域 + EDT clearance + 转角惩罚 + 隧道轴向代价，global_planner_node.cpp:624-760）→ `/plan_raw`
  → MINCO L-BFGS（固定时间、ESDF 软障碍、隧道走廊软项、两阶段 v/a 修正，minco_path_smoother_node.cpp:422-548）
  → 发布前 SFC/ESDF 双校验 + 有界修复（node:642-743）→ `/plan`。
- 控制链：OSQP MPC（mpc/mpc_solver.hpp:1-3、38-57）跟踪路径，速度由 speed_profile 曲率限速 + 隧道窗口给出
  （speed_profile.hpp:9-12、95-115；mpc_controller_node.cpp:288-297、826-850），另有局部安全否决、恢复链 FSM、
  多假设进度跟踪（README.md:86、94-105）。
- 几何权威：语义 msgpack 为真源，在线 ESDF 只由局部代价图生成、以只读 registry 供 MINCO/MPC 共用（README.md:88-92）。

### 1.3 可比性边界

- 可比：前端 A*、路径化简、SFC 构建、后端参数化/求解器/时间分配、碰撞处理与校验。
- 不可比（标"不适用"）：TDT 无控制层（MPC/速度剖面/恢复链/进度跟踪）、无 ROS 代价图与时间衰减、无语义地图
  （隧道/云台收放）、无双雷达。这些是 Lucifer 的净增量，不构成"提升点"。

## 2 逐项对比

### 2.1 对比总表

| 维度 | TDT nav-kit | Lucifer navigation2 | 判断 |
|---|---|---|---|
| 前端搜索 | 8 邻域 A*，`g` 用势场代价图 × costWeight，`h` 为格欧氏距离（yastar.cpp:802-882） | 8 邻域 A*，`g` 含 EDT clearance 二次惩罚、转角惩罚、隧道轴向代价；`h` 为格欧氏距离（global_planner_node.cpp:624-760） | 同构；Lucifer 代价项更丰富 |
| 对角切角 | 未检查对角两侧格（yastar.cpp:854-875 只查目标格） | 显式检查两侧格防穿角（global_planner_node.cpp:692-702） | **Lucifer 更安全** |
| 势场/净空 | 稀疏 SDF + 衰减函数预算代价图（yastar.cpp:760-800） | 精确 EDT 一次、按内容哈希缓存（global_planner_node.cpp:624-646） | 等价；Lucifer 缓存更工程化 |
| 路径化简 | DP 化简 + 带 `lineInObsticle` 的直线化简（yastar.cpp:195-276） | 成本阈值 prune（默认关，yaml:63）+ resample 0.18 + stitch（README.md:64） | TDT 的 LOS 保证值得借鉴（P1-1） |
| 动态障碍 | 多层/命名/定时 Mask（yastar.hpp:20-49、117-134；yastar.cpp:451-505） | 逐格时间衰减，无掩码层、无预测（yaml:129-131、292-300；local_costmap_node.cpp:620-700） | **实质差距在"预测"，不在"记忆"** |
| SFC 构建 | 每个路标点找最大无障方盒，O(r²) 逐格扩张（sfcSquare.cpp:71-107、144+） | 积分图 O(log r) 查询（include/sfc_corridor.hpp:15-19、100-113） | Lucifer 实现更优 |
| SFC 作用 | OSQP 硬箱约束（minimumSnap.cpp:785-905） | 仅发布前校验 + 修复触发（node:642-684；MINCO_README.md:22-27） | 形式不同；见 §2.2.4 判断 |
| 碰撞处理 | 采样后 `lineInObsticle` → 插中点/缩走廊迭代（minimumSnap.cpp:109-171、955-988） | ESDF 软代价 + SFC/ESDF 校验 + 有界修复（放大 data_weight / 插路标点）+ 失败回退（node:336-419、642-743） | 机制等价，Lucifer 有回退路径 |
| 参数化 | 分段 5 次多项式、3 阶导（min jerk），控制点位置硬/软约束（MinimumSnap.md:27-51） | MINCO S3NU 分段三次、控制点自由、固定段时间（MINCO_README.md:18-20；minco_optimizer.hpp:31-58） | 不同流派；MINCO 变量更少 |
| 求解器 | 闭式求解 / OSQP（ADMM）+ SFC 后端（minimumSnap.hpp:33-42） | L-BFGS 无约束（minco_optimizer.cpp:470-510）；MPC 另有 OSQP | 各取所需 |
| 时间分配 | 梯形距离预分配（minimumSnap.cpp:281-305） | 距离/速度 + 转角附加，固定预分配（minco_time_allocation.hpp:13-65） | 大体同级 |
| 时间归一化 | 可选 `normTime`，Q/A/采样同步缩放（MinimumSnap.md:210-354） | 无此机制；以 `min_segment_time=0.1` 规避退化段（yaml:164；minco_optimizer.cpp:406-431） | 对 MINCO **不适用**（§4.4） |
| v/a 约束 | 控制点硬速度（minimumSnap.cpp:826-833）；加速度只进时间分配 | 两阶段精确求根检测后放大时间重优化（node:422-548）；控制层曲率/隧道限速 | **Lucifer 表达更完整** |
| 性能基准 | README 内嵌 1000 次 avg/max 表（README.md:90-113；minimumSnap.md:389-401） | performance_monitor 存在但默认关（yaml:155、394），无落盘基线 | **Lucifer 缺基准（P0-1）** |
| 控制/执行层 | 无 | MPC + 速度剖面 + 恢复链 + 进度跟踪（README.md:94-105） | 不适用（Lucifer 独有） |

### 2.2 关键差异拆解

**2.2.1 前端搜索：Lucifer 已经领先，不需要向 TDT 看齐。** TDT 的 A* 只有势场代价和 8 邻域，没有对角切角
保护，也没有转向惩罚（yastar.cpp:854-875；864 只按目标格代价做转移）；启发式是裸格距离、不乘代价权重（yastar.cpp:867-869），
与 `g` 的代数量纲不一致，属有意简化而非可调设计。Lucifer 的 A* 有切角检查（global_planner_node.cpp:692-702）、
`turn_penalty`（yaml:30）、隧道轴向项（yaml:54）和 EDT clearance（yaml:20-22）。TDT 唯一值得借鉴的是
"化简前置"思路（§2.2.2）。

**2.2.2 路径化简：TDT 的"视线保证"值得抄，Lucifer 的 prune 目前没启用。** TDT 的 DP 化简递归检查两点间
是否 `lineInObsticle`，失败则回退保留中间点（yastar.cpp:195-237），再用 `simplifyPathHypot` 做方向性弦优化
（yastar.cpp:246-276），注释明确"保证路径不进入障碍物"（yastar.hpp:193-196）。Lucifer 的 `prunePathIndices`
只要求"段内最大代价值 ≤ prune_segment_max_cost"（global_planner_node.cpp:366-402），不校验视线；且
`path_prune_enabled: false`（yaml:63），意味着当前 `/plan_raw` 是 A* 逐格路径经 0.18 m 重采样后的结果
（yaml:65、global_planner_node.cpp:429-465）——10 m 路径约 55 段（minco_path_smoother_node.cpp:452 注释）。
段数直接决定 MINCO 的求解规模，这是 P1-1 的依据。

**2.2.3 动态障碍：差距在"预测"，不在"记忆"。** TDT 的 Mask 是"栅格位图叠加层"：`masked(x,y)` 对 64 层做
按位或（yastar.cpp:60-67），`setMaskMapTimed` 每次 push 一层 `name@id`，超过 `duration` 的层被回收、超过
`maxCount` 覆盖最旧层（yastar.cpp:451-505；Usage.md:336-354）。它的工程价值是：①按传感器帧整层过期，过期
语义原子；②可以把"预测位置"作为独立层推入而不污染基础图。Lucifer 没有层概念，但逐格衰减同样实现了"旧观测
会消失"（局部 0.8/1.2 s、全局 12 帧），且 `observation_timeout=2.0 s` 对齐了多跳链路延迟（yaml:129、283）。
**因此直接移植 Mask 数据结构是重复建设**；真正缺的是"把障碍按未来位置推入地图"的那一步——而这需要目标
跟踪/速度估计，Lucifer 目前完全没有（decision 只有位置 EMA）。故此项列为**条件触发**的 P1-2，而不是 P0。

**2.2.4 SFC 与碰撞处理：形式不同，收益已被覆盖。** TDT 用 OSQP 把每个中间控制点的位置约束成方盒上下界
（minimumSnap.cpp:785-905 的 `makeBounds`/`postProcess`），轨迹在盒内可自由浮动，从而弱化了"必须经过控制点"
的刚性；每轮迭代交替"插中点重解"与"把碰撞段的走廊减半"（minimumSnap.cpp:955-988）。但文档同时说明：
方盒不保证轨迹与障碍无碰、也不保证走廊相接（MinimumSnap.md:196），所以它**仍然只是迭代启发式**，不是硬保证。
Lucifer 的对应设计是：ESDF 二次插值软代价（minco_optimizer.cpp:163-230）+ 发布前 SFC 全图校验
（node:642-668）+ 局部距离场校验（node:669-684）+ 有界修复（node:336-419、689-743）+ 失败回退原始路径。
**判断：Lucifer 的"软代价 + 校验 + 修复"在安全性上不弱于 TDT 的"方盒硬约束 + 迭代"，且不需要重建 QP 链；
把 SFC 变成硬约束属于 §4.1 的"不建议照搬"。** 唯一可考虑的是把 SFC 盒变成带解析梯度的软项（P2-1），
用来降低校验拒绝率，而非替代校验。

**2.2.5 参数化/求解器/时间：MINCO 与 MinimumSnap 是两条路，不要互相替换。** MinimumSnap 控制点有强
位置/连续性/速度等式约束，Q 矩阵分段构造（MinimumSnap.md:52-165），优点是 QP 凸、可硬约束、闭式解极快
（README 表 125 us，MinimumSnap.md:392-401），缺点是控制点质量差时无解或撞墙（MinimumSnap.md:181-189），
必须靠 SFC + 迭代打补丁，作者自己也承认"基于线性 minimum snap 的局限性……基本已经到头了"
（MinimumSnap.md:405）。MINCO 相反：控制点自由、固定段时间、无约束 L-BFGS + 解析梯度，靠软代价表达
障碍/数据/走廊，变量更少、可扩展性好（MINCO_README.md:18-21；minco_optimizer.hpp:31-58）。两者**不是同一
优化问题的两种解法**，互相移植只会丢掉各自的结构性优势。时间归一化只对"高次多项式系数 + 物理时间幂次"
的病态有意义（MinimumSnap.md:210-354），MINCO 分段三次不存在该病态，故 §4.4 判"不适用"。
TDT 的时间归一化本身也是实验性："归一化结果与不归一化不一致、相对较差"（MinimumSnap.md:174），README 也
标注"提高长距离数值稳定性"，属于可选项而非必选。

### 2.3 性能与可观测性

- TDT 把基准写进了工程流程：`benchTime` 模板记录 avg/max（main.cpp:19-58），README 直接给出 RMUC2024/2026
  地图 1000 次搜索的分阶段表（A* 7627 us / MinimumSnap 1255 us / 生成代价图 1604 us / 化简 34 us；README.md:90-113），
  文档还有前端质量 × 后端的矩阵对比（MinimumSnap.md:389-401）。这套数据是它敢做"势场权重/迭代次数/后端"
  取舍的依据。
- Lucifer 有 `PerformanceMonitor`（include/performance_monitor.hpp），但 MINCO/MPC 两处默认全关
  （yaml:155-158、394-397），仓库无落盘基线；tests 覆盖较全（CMakeLists.txt:98-184，17 个 gtest +
  launch test），但没有性能回归。**这是最便宜、杠杆最大的一项（P0-1）。**

## 3 提升项清单

> 改动量口径：S < 1 天、M 1-3 天、L > 3 天（1 人）。所有落点只写建议接口/参数，不改源码。

### 3.1 P0-1 分阶段性能基线（S）

- **现状证据**：performance_monitor 默认关（yaml:155、394）；无 A*/MINCO/SFC/MPC 的耗时与触发率数据；
  MINCO_README.md:41-43 自己提出"P95 是是否保留修复的判据"，但仓库没有这个数。
- **TDT 做法**：`benchTime` 记录 avg/max（main.cpp:19-58）+ 固定地图/随机起终点 1000 次 +
  README 表（README.md:90-113；MinimumSnap.md:389-401）。
- **Lucifer 落点**：新增离线工具（仿 `tools/sfc_corridor_dump.cpp` 的独立 executable）或 `test/bench_*`
  驱动，复用 `rm_global_planner_node`/`rm_minco_path_smoother_node` 的组件接口，在固定 seed/起终点集上跑
  N 次，输出 CSV：A*、MINCO 一阶段/二阶段、SFC 构建、校验、修复、MPC solve 的 avg/max/P95，以及
  `global_clearance`/`safety_reject`/`optimize_failed` 次数。参数用 `performance.enable: true` 打开采样。
- **收益**：为 §3 其余所有改动提供"改前/改后"判据；直接暴露 8 Hz 回调的 P95 与修复风暴。
- **风险**：几乎为零（只加工具）；注意基准必须用真实地图与真实起终点分布，否则会重复 TDT "随机取点"的乐观偏差。
- **优先级**：P0（先做，其他项以它为门）。

### 3.2 P0-2 平滑器墙钟预算与降级闸门（S）

- **现状证据**：修复最多 `1 + repair.max_attempts = 3` 次完整优化（node:336；yaml:237），每次含一阶段
  `lbfgs.max_iterations=800` 与可选的二阶段 300（yaml:169-170、175-179）；二阶段还要再跑一次优化
  （node:485-510）。`grep -n 'deadline|time_limit|budget|elapsed'` 在 minco 节点/优化器中**零命中**，
  即最坏耗时可越过 8 Hz 回调周期（125 ms），路径新鲜度下降。
- **TDT 做法**：`iterate` 同时受 `maxIter` 与 `timeLimit` 约束，`elapsed >= timeLimit` 直接停
  （minimumSnap.cpp:125-163；timeLimit 默认 1 s，minimumSnap.hpp:232；`setTL` 在 main.cpp:149）。
- **Lucifer 落点**：`smoothAndPublish` 入口取 `steady_clock` 起点，定义预算（例如 `0.5 × 1/8 Hz = 60 ms`
  或按 P0-1 的 P95 定）；在 `total_attempts` 循环每轮开始、二阶段开始前检查，超预算则跳过剩余修复/二阶段，
  直接走已有的"失败回退原始路径"分支（node:382-399）。预算做成参数 `repair.time_budget_ms`。
- **收益**：最坏回调延迟有界，避免"修复风暴 → 路径陈旧 → MPC 追踪旧路径 → veto → 重规划"的放大回路。
- **风险**：预算过紧会让修复退化为"不做"，需要 P0-1 的 P95 定阈值；回退到 A* 折线已被 speed_profile 限速，
  是可接受的降级。
- **优先级**：P0（时延安全，改动小）。

### 3.3 P1-1 LOS 保证的前端化简前置（S/M）

- **现状证据**：`path_prune_enabled: false`（yaml:63），当前 `/plan_raw` 为逐格路径 + 0.18 m 重采样
  （yaml:65），10 m 路径约 55 段（node:452 注释），MINCO 的段数/控制点数随之放大。
- **TDT 做法**：DP 化简（递归 + `lineInObsticle` 视线检查，yastar.cpp:195-237）+ 方向性弦优化
  （yastar.cpp:246-276），保证化简后线段仍无碰，用在求解前（main.cpp:133-135）。
- **Lucifer 落点**：`global_planner_node.cpp` 的 `buildPath` 在 `resampleWorldPoints` 之前插入
  "LOS 化简"：用现有 `isTraversable`/`raytraceLine` 做 Bresenham 视线校验，替代 `prunePathIndices` 的
  成本阈值判据；新参数 `path_los_simplify_enabled`、`los_simplify_max_deviation`。TDT 代码可直接移植，
  但需改成 Lucifer 的 `OccupancyGrid` 语义与 `obstacle_threshold`。
- **收益**：段数 55 → 约 10-20（取决于路径直度），MINCO 一阶段耗时近似线性下降；节省的预算可给 P0-2 的
  修复/二阶段；控制点更少 → 数据项与平滑项权重关系更清晰。
- **风险**：控制点减少会弱化"贴着 A* 路径"的约束（data_weight 相对变小），可能出现更贴墙/切角的轨迹；
  必须用 P0-1 对比最小 clearance、轨迹长度、SFC 拒绝率三项，并保留开关。
- **优先级**：P1。

### 3.4 P1-2 动态障碍预测层（条件触发；A=S，B=M/L）

- **现状证据**：局部代价图时间衰减 `keep_time=0.8 / clear_time=1.2 s`（yaml:298-300）、`hit_inc`（yaml:294）、
  全局 12 帧整数衰减（yaml:131）——有记忆、无预测；`Mask` 层概念不存在（grep 零命中）；decision 只有 EMA
  位置（target_input.cpp:135-145），状态无速度（target_input.hpp:73-80）。
- **TDT 做法**：多层 Mask 可把任意栅格位图作为一层压入并按 `duration` 过期、按 `maxCount` 覆盖
  （yastar.cpp:451-505；Usage.md:336-354）；定时层语义天然适配"把预测位置推入规划地图"。
- **Lucifer 落点（分两阶段，勿一步到位）**：
  - A. 基础设施（S）：在 `rm_local_costmap`/`rm_global_costmap` 增加一个"预测层"入口（新话题
    `/predicted_obstacles`，字段为点集 + TTL），把预测格以低于 `obstacle_threshold` 的代价 + TTL 写入本地
    `hit_age` 体系，复用现有衰减设施，不引入第二张地图。
  - B. 预测源（M/L）：新增轻量 track（点云聚类 + 匀速/α-β 滤波）或先只扩展现有 `TargetInput` 状态
    （位置 EMA → 加 α-β 速度），后者只覆盖"正在追的敌人"，收益有限；要覆盖多目标必须做聚类跟踪。
- **收益**：对高速横向接近的敌人可提前让行，减少 MPC veto/恢复；对追击场景（decision 已在追同一目标）收益小。
- **风险**：误检/误预测 → 绕远、抖动、规划振荡；必须 P0-1 先证明"动态障碍相关的 veto/恢复占比可观"再投入。
  若基线显示该占比 < 5%，**不建议做**，维持现有逐格衰减（§4.5）。
- **优先级**：P1（有条件）。

### 3.5 P2-1 把 SFC 盒做成 MINCO 的带梯度软走廊项（M）

- **现状证据**：SFC 只用于校验（node:642-684）；优化器只有隧道走廊软项（minco_optimizer.hpp:40-44；
  `attach_corridor_functional`，minco_optimizer.cpp:236-320），全路径没有走廊项。
- **TDT 做法**：每个路标点的方盒作为 OSQP 位置上下界（minimumSnap.cpp:785-905）。
- **Lucifer 落点**：复用 `SfcCorridor::usableBoxAt`（sfc_corridor.cpp:231-247，目前只有离线 dump 工具用，
  tools/sfc_corridor_dump.cpp:350-361），把"以路标点为中心的无障方盒"作为 `attach_corridor_functional`
  式的软惩罚（盒内 0、盒外二次、解析梯度），权重参数化。**不做硬约束**。
- **收益**：降低 `safety_reject`/`global_clearance` 与修复触发率，从而减少 P0-2 的预算压力。
- **风险**：软项无法硬保证，仍需校验；权重扫描成本；方盒对窄通道可能过紧 → 与隧道走廊项冲突。
  先看 P0-1 的拒绝率再决定。
- **优先级**：P2。

### 3.6 P2-2 MINCO 起点状态接入里程计（S）

- **现状证据**：MINCO 头/尾状态被写死为 `[waypoint, 0, 0]`（位置、零速度、零加速度，
  minco_optimizer.cpp:428-434），即每条新路径都从静止起步；而 MPC 只用快照的速度**方向**、模长走
  speed_profile（mpc_controller_node.cpp:826-850），所以实际影响被削弱。
- **TDT 做法**：`setInitVel` 可设起点速度（main.cpp:164；doc/Usage.md:894-906），终点速度默认 0（README.md:87-88）。
- **Lucifer 落点**：`MincoOptimizer::optimize` 增加 `head_state` 入参（位置/速度/加速度），节点从 odom twist
  取起点 v/a；尾状态在"接近目标"时保持 0，否则用路径末端切向 × 名义速度。
- **收益**：新路径首段不必从静止加速，贴合真实运动；对高频重规划下的首段形状更自然。
- **风险**：非零起点加速度可能让 L-BFGS 初值离最优更远、收敛变差；MPC 当前只吃方向，收益需 A/B 实测。
- **优先级**：P2。

### 3.7 P2-3 时间分配引入曲率限速（S/M）

- **现状证据**：段时间 = 距离/`default_velocity` + 转角附加（minco_time_allocation.hpp:20-65），与曲率无关；
  MPC 另用 speed_profile 按曲率限速（speed_profile.hpp:9-12、88-92），两者对同一路径给出不同的速度意图
  （MPC 注释也承认"MINCO 时间分配目前是 dist/v 常数、未做曲率限速"，mpc_controller_node.cpp:821-824）。
- **TDT 做法**：时间分配直接吃 `maxSpeed/maxAcc` 做梯形分配（minimumSnap.cpp:281-305），同样不看曲率。
- **Lucifer 落点**：`allocateMincoSegmentTimes` 的速度项改用"路径曲率限速后的平均速度"（复用
  `speed_profile` 的 `v_curve = sqrt(a_lat/κ)` 公式，或调用现有路径参考类），离散步长按弧长采样。
- **收益**：MINCO 自身的速度/时间与 MPC 参考一致，减少两阶段因超速触发的放大重优化。
- **风险**：需要路径曲率估计（A* 折线本身曲率尖锐，需先平滑或按窗口估计）；收益中等，需 P0-1 衡量。
- **优先级**：P2。

### 3.8 提升项汇总

| 项 | 优先级 | 改动量 | 核心收益 | 主要风险 |
|---|---|---|---|---|
| P0-1 分阶段性能基线 | P0 | S | 其余所有改动的判据；暴露 P95/拒绝率 | 基准代表性问题 |
| P0-2 平滑器墙钟预算 | P0 | S | 最坏回调延迟有界 | 预算过紧导致降级 |
| P1-1 LOS 化简前置 | P1 | S/M | 段数/求解耗时显著下降 | 数据项相对变弱、贴墙 |
| P1-2 动态障碍预测层 | P1（条件） | M/L | 提前让行；仅当基线显示需要 | 误预测 → 绕远/抖动 |
| P2-1 SFC 软走廊项 | P2 | M | 降低校验拒绝/修复触发 | 软项无硬保证、权重调参 |
| P2-2 起点状态接里程计 | P2 | S | 首段运动学更真实 | 收敛变差、收益待测 |
| P2-3 时间分配按曲率 | P2 | S/M | MINCO 与 MPC 速度意图一致 | 曲率估计复杂 |

## 4 不建议照搬

**4.1 SFC 硬约束 + 交替迭代（L，不建议）**。TDT 用 OSQP 硬箱界约束（minimumSnap.cpp:785-905），前提是
"后端是 QP + 控制点位置可放松"；Lucifer 后端是无约束 L-BFGS（minco_optimizer.cpp:470-510），要做硬约束
必须换 SQP/内点/增广拉格朗日，等于重建优化器。TDT 文档自己也说方盒"并不保证轨迹不与障碍碰撞"
（MinimumSnap.md:196），仍需碰撞迭代兜底；Lucifer 的软 ESDF + 双校验 + 有界修复已覆盖同一收益且更简单。
只保留 P2-1 的"软走廊"方向。

**4.2 闭式解后端（S，不建议）**。闭式解只能处理等式约束（MinimumSnap.md:167-172），最快（README 表
125 us）但不能表达走廊；TDT 自己的结论是"闭式求解不建议使用 jps 作为前端，质量很差基本不可优化"
（MinimumSnap.md:403）。Lucifer 的 MINCO L-BFGS 已经是无约束问题的闭式梯度 + 拟牛顿，再引入一条
"闭式/OSQP 双后端"分支只会增加维护面。

**4.3 Kastar/混合搜索接入（S-M，不建议）**。已移植未接入（include/kinodynamic_astar.hpp:6-31），
MINCO_README.md:69-74 的理由成立：全向底盘 + 短距低速机动用射线可达性即可，Kastar 状态空间大两个数量级
（include/kinodynamic_astar.hpp:27-28）、全局搜索约 2 s（Astar.md:90）。TDT 作者自己的结论也是"前端只负责能不能到，后端负责怎样更好地到"
（Astar.md:88-92），这与 Lucifer 现有分工一致，反向接入是倒退。

**4.4 时间归一化（S，不适用）**。该机制解决的是高次多项式系数在物理时间幂次下的病态
（MinimumSnap.md:210-354），MINCO 用分段三次 + `min_segment_time`（yaml:164；minco_optimizer.cpp:406-431）
已规避退化段，不存在同类病态。TDT 的归一化本身也是实验性（MinimumSnap.md:174；main.cpp:162 注释
"轨迹质量不算很好"）。

**4.5 独立 Mask 地图/独立动态障碍层（M，不建议作为独立地图）**。TDT 的 Mask 是纯栅格位图叠加
（yastar.hpp:20-49），Lucifer 已有局部/全局代价图与衰减，另建一张掩码地图会造成"两套动态障碍语义"
（规划器用一套、MPC 用另一套），违反 README.md:88-92 的几何权威约定。**只借鉴其"按层/TTL 推入预测位置"
的语义**（P1-2），不搬数据结构。

**4.6 控制点速度约束 Item（S，不需要）**。TDT 的 `Item.vxvy` 用于"过洞、上下台阶需要对准"的场景
（MinimumSnap.md:356-357；Usage.md:757-776）。Lucifer 的对应能力已在控制层：speed_profile 的隧道
[vmin, vmax] 窗口（speed_profile.hpp:95-115；mpc_controller_node.cpp:288-297）+ 隧道轴向/横向走廊软代价
（minco_optimizer.hpp:35-44）+ 云台收放由电控负责（README.md:34）。在 MINCO 内加硬速度约束会与现有
时间分配/两阶段冲突。

**4.7 TDT 前端/走廊原型的实现细节（不建议回抄）**。8 邻域 A* 缺对角切角检查（yastar.cpp:854-875）；
启发式不乘代价权重（yastar.cpp:867-869）；`SfcSquare` 逐格扩张 O(r²)，且 origin 符号不一致、`FLT_MAX`
转 int 是 UB（Lucifer 已修复并改成积分图 O(log r)，include/sfc_corridor.hpp:10-23）。这些 Lucifer 已经
更优，回抄是负收益。

**4.8 用"段内最大代价值"做 prune（S，不建议保留现状）**。Lucifer 现有 `prunePathIndices`
（global_planner_node.cpp:366-402）不校验视线，参数默认关（yaml:63）。若要化简，应采用
P1-1（LOS 保证型），而不是重新打开旧 prune。

## 5 验证方法

1. **基准（对应 P0-1）**：仿 TDT `benchTime`（main.cpp:19-58）建离线 benchmark：固定地图 + N 组固定
   起终点（含直道/急弯/隧道/窄通道四类），每组重复 1000 次，记录 avg/max/P95 与成功率；分阶段打点
   （A*、MINCO 一阶段、二阶段、SFC 构建、校验、修复、MPC solve）。输出 CSV，纳入 CI 的非阻塞任务。
2. **A/B 指标**：每次提升项上线前后对比——成功率、总耗时分布、`global_clearance`/`safety_reject`/
   `optimize_failed` 次数、修复触发率、二阶段触发率、轨迹长度、最小 clearance（用
   `tools/sfc_corridor_dump.cpp` + `tools/render_corridor.py` 出图，对照 docs/images/sfc_corridor*.png）、
   MPC 跟踪误差、veto 次数、恢复触发次数、到达时间。
3. **单元测试**：现有 gtest 已覆盖 SFC/kinodynamic/minco/two-stage/repair 等（CMakeLists.txt:98-184），
   P1-1 需加"LOS 化简后仍无碰"的用例；P0-2 需加"预算耗尽时回退原路径"的用例；P1-2 需加"TTL 过期后
   预测格消失"的用例。
4. **仿真回归**：`ros2 launch bringup sim.launch.py world:=RMUL`（README.md:43-49）跑固定航点序列，
   对比 `/plan` 与 `/cmd_vel` 的耗时/里程/恢复次数；实车前先用 `sfc_corridor_dump` 在真实 msgpack 上
   离线复现，避免带着参数上真车。
5. **落地判据**：P0-2 以"最坏单次 MINCO 回调 ≤ 1 个控制周期"为硬门槛；P1-1 以"段数下降 ≥ 50% 且
   最小 clearance 不回退"为门槛；P1-2 以"动态障碍相关 veto 占比下降且误绕行不增加"为门槛；达不到就
   回滚（所有项都保留参数开关）。

## 6 证据索引

### 6.1 TDT nav-kit

- README.md:5 组件定位；13-14 组件清单；31-47 依赖；70-88 参数表；90-113 性能表。
- doc/Astar.md:22-28 八邻域局限；30-40 势场代价；50 多层掩码；58-80 Kastar；82-86 混合搜索（仅文档）；
  88-92 前后端职责结论。
- doc/MinimumSnap.md:167-177 闭式解 + 碰撞插入；181-189 线性 MinimumSnap 的局限；191-208 SFC + 交替迭代；
  210-354 时间归一化；356-387 速度约束；389-401 性能矩阵；403 闭式解不建议 JPS；405 方法上限。
- src/YAstar/yastar.hpp:20-49 Mask；63-66 DP 化简；117-134 掩码层/定时；181-182 势场；193-214 化简/碰撞。
- src/YAstar/yastar.cpp:13-17 Mask 构造；60-67 `masked`；69-122 `pushMask`；123-140 `duration`；
  151-184 `lineInObsticle`；195-237 DP 化简；246-276 `simplifyPathHypot`；358-378 构造/`costWeight`；
  451-505 `setMaskMapTimed`；760-800 `initCostMap`；802-882 `search`；936-943 `isInObstacle`。
- src/YAstar/kinodynamicAstar.hpp:11-72（状态/配置/结果/接口）。
- src/MinimumSnapOsqp/minimumSnap.hpp:33-42 后端；64-72 时间分配；123-131 速度/加速度/归一化开关；
  224 梯形时间分配；232 `timeLimit`。
- src/MinimumSnapOsqp/minimumSnap.cpp:109-171 碰撞迭代；217-277 solve 入口；281-305 梯形时间分配；
  785-905 OSQP 走廊硬约束；907-995 `_solve`；955-988 交替修复实现。
- src/MinimumSnapOsqp/sfcSquare.cpp:71-107 `getBound`/`getCorridor`。
- main.cpp:19-58 基准模板；96-175 主流程示例；145-164 求解参数；233-352 控制点/速度约束基准；
  275-276 时间分配；398-432 Kastar 独立 demo。

### 6.2 Lucifer navigation2

- README.md:5-7 虚拟云台；25-35 节点清单；62 全局 A*；64 分层重规划；66-70 鲁棒性；86 MPC 安全；
  88-92 几何权威；94-105 执行层。
- MINCO_README.md:18-21 功能特性；22-27 SFC 校验 + 修复；30-40 隧道走廊；41-43 两阶段精确检查；
  64-67 SfcCorridor；69-74 KinodynamicAstar 未接入；78-86 平滑器对比表。
- include/sfc_corridor.hpp:4-32 来源与三处差异；61-74 参数；100-113 查询/`pathClear`。
- src/sfc_corridor.cpp:231-247 `usableBoxAt`；249-275 `pathClear`。
- include/minco_time_allocation.hpp:13-19 设计说明；20-65 实现。
- include/kinodynamic_astar.hpp:4-31 来源与接入判断。
- src/minco/minco_optimizer.hpp:31-58 参数；src/minco/minco_optimizer.cpp:163-230 障碍软代价；
  236-320 走廊软项；406-431 段时间校验；428-434 首尾状态；470-510 L-BFGS；512-547 结果护栏。
- src/minco_path_smoother_node.cpp:100 采样步长；286-298 几何去重；302-303 时间分配；308-316 距离场快照；
  319-419 修复循环；422-548 两阶段；551+ 采样输出；642-684 双校验；689-743 插点修复。
- src/global_planner_node.cpp:256-275 可通行/格代价；366-402 prune；429-465 resample；624-646 clearance；
  648-654 启发式；669-702 8 邻域/切角；716-760 代价合成；792-831 stitch。
- params/navigation2.yaml:63-65 prune/resample；129-131 全局图衰减；155-158 MINCO 性能开关；
  161-165 时间分配；174-182 两阶段；221-231 global_check；237-240 修复；261-262 局部图尺寸；
  283/290-300 局部图衰减；362-389 MPC 速度；394-397 MPC 性能开关；398-411 局部安全；474 恢复限速。
- src/local_costmap_node.cpp:250-262 衰减参数；598-700 复用/衰减实现；1292-1297 成员。
- src/mpc_controller_node.cpp:288-297 隧道限速窗口；826-850 快照方向 + 速度剖面；923-928 碰撞否决；
  1111+ `handleVeto`。
- src/mpc/speed_profile.hpp:9-12 剖面步骤；51-56/95-115 隧道窗口；src/mpc/mpc_solver.hpp:1-3、38-57、127-143。
- src/decision/decision_node/include/decision/target_input.hpp:28-33 观测字段；53-80 配置/状态；
  src/decision/decision_node/src/target_input.cpp:135-145 位置 EMA；decision_node.cpp:711-725 standoff 目标。
- CMakeLists.txt:35 源文件 glob；98-184 测试清单；180-184 kinodynamic 仅测试；230-232 走廊 dump 工具。
- docs/images/sfc_corridor.png、sfc_corridor_tunnel.png、sfc_corridor_demo.png；tools/render_corridor.py:11-13。
