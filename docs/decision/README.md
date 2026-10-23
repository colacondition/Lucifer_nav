# 决策重构：用行为树替代手写状态机（RMUL 2027）

> 输入：用户提供的 4 份 PDF（RMUL 2027 规则、2026 通信协议、串口协议 V1.9、ROS2 行为树教程）。
> 本项目是 **RMUL 3V3 哨兵（编号 7）**；数据路径是 **电控先从裁判系统取数，再通过串口发给 NUC**。
> 2027 通信协议尚未发布，本设计以 2026 协议为基线，并把"协议变化"隔离在适配层。
>
> 配套文档：
> - [`rules-and-data.md`](rules-and-data.md)：规则要点 + 数据可得性矩阵（全部带 PDF 页码/表号）。
> - [`current-audit.md`](current-audit.md)：现有 FSM/接口/测试审计 + BT 叶子清单 + 鸡肋点。
> - [`bt-scaffold.md`](bt-scaffold.md)：BT 实现说明（生产树、黑板 key、XML 约定、运行时）。

## 0. 结论

1. **现有决策的问题不在"算法弱"，而在"输入契约 + 动作生命周期 + 决策维度"**：
   - 串口上行只有 3 个净荷字段（`game_type/game_progress`、`stage_remain_time`、`sentry_hp`），
     热量/弹量/金币/增益/RFID/控制区全部未转发；
   - `serial_driver` 原来用 `SensorDataQoS`（best_effort）发布状态，决策用默认 QoS（reliable）订阅
     —— **静态不兼容，实机上决策可能一帧都收不到且不报错**（本轮已修，见 §3）；
   - 现有 FSM 只有 `game_active / current_hp / executor_event` 三个输入，动作完成事件是**单槽
     optional**；追击抢占巡逻时旧动作终态可能丢失，导致"巡逻永久停住"；
   - 决策维度只有"去哪儿"：没有阶段、控制区、复活/虚弱、热量、弹药、判罚、攻击态势。
2. **BT 的价值不是把 if-else 画成树，而是把"优先级/抢占/动作生命周期/可测试性"从 900 行编排代码里
   拆出来**：XML 调优先级，叶子节点自持 `onStart/onRunning/onHalted`，树可以用假节点离线单测。
3. **当前状态（P2 已完成）**：旧状态机（`DecisionStateMachine` / `decision_node.cpp`）已删除；
   `bt_decision_node` 是唯一生产节点（launch 已切），行为树 + `MissionRuntime` 接管执行器/
   TF/追击/走位/`/decision/state`；`colcon test --packages-select decision` 9 个 ctest 全过
   （含 11 个 BT 场景）。**行为与旧 FSM 等价，战术仍是哨兵站中心**。

## 1. RMUL 2027 对哨兵决策的硬约束（细节见 `rules-and-data.md`）

| 约束 | 决策含义 |
| :- | :- |
| 3V3 阵容：重装(1) / 步兵(3) / 哨兵(7)；哨兵 **0 操作手、全自动** | 决策必须自恢复，不能等人 |
| **多机通信不允许**（表 3-5） | 拿不到队友意图/坐标，不能做协同围剿；只能靠自己状态 + 视觉 + 地图先验 |
| 胜负核心 = **控制度**：3 个控制区、占领进度 0–5、双方 600、战亡 −10、差值 200/400 给对方恢复增益 | "占住控制区"比"多杀人"更接近胜负；但 **2026 协议没有占领进度/控制度字段** |
| 哨兵 HP 400、热量 260/冷却 30、弹量 **750 局内不可增**、功率 100W | 弹药不参与兑换；决策不需要"买弹"，但要避免无谓死亡（−10 控制度） |
| 战亡读条 5/10/15…s；复活后 20% HP、无敌 30s、**虚弱：不能开火/不能占区，碰己方补给区解除** | 复活后唯一正确目标 = 回补给区；**无协议字段可读读条**，只能按规则推算 |
| 补给区回血 25%/s；高地增益 25% 防御；隧道 1.5× 冷却 60s | 低血撤退到补给区是硬收益；隧道增益只在特定路线经过时顺带拿 |
| 2027 规则全文**未见哨兵姿态/强化姿态机制**（2026 协议 0x020D/0x0120 的姿态位是 RMUC 语义） | 不把"姿态选择"写进 2027 决策；等 2027 协议/规则确认 |
| 五秒倒计时后禁止遥控（R22）；准备/自检阶段不得离开初始区域（R36） | 阶段门必须严格：`game_progress != 4` 时只允许 Wait/Home |

## 2. BT 设计

### 2.1 生产树（P2，已落地）：与旧 FSM 行为等价

```
PriorityFallback（每拍从高到低重选，只保留一个 RUNNING 分支）
├── ① 定位 LOST            → Holding（不新发 map 系目标，不取消已在跑的目标）
├── ② 游戏未进行           → 不在家 GoHomeSafe → WaitHome
├── ③ 血量 0（死亡）       → Holding（等待自动复活）
├── ④ 低血（120/400 迟滞） → 不在家：Revived 走 GoHomeTunnel，否则 GoHomeSafe；到家原地 hold（无 wait_hp.csv）
├── ⑤ 比赛已开始但无血量   → WaitHome（等数据）
├── ⑥a 还没到中心          → GoCenter（center.csv）
└── ⑥b 已到中心            → 先 hold center；每 group_gap_sec 在 range.csv 内随机取点走一组；
                             有目标 ChaseStandoff
```

- **先 GoCenter 到中心，之后才追敌/走位**；追击不会在回家/去中心途中发生（旧 FSM 只在 CENTER 段追击）。
- **回家两条路线**：刚复活（`HP 0→>0`，20% 血/无敌/虚弱）走 `home_tunnel.csv`，趁无敌快速到补给区解除虚弱；
  残血但本局没死过走 `home_safe.csv`，绕远避开隧道，避免被拦截打死。到补给区成功后清复活标记。
- 低血/比赛结束抢占中心；恢复后重新 GoCenter；追击失败回退走位，不卡死。
- XML：[`config/trees/rmul.xml`](../../src/decision/decision_node/config/trees/rmul.xml)，改优先级只改 XML。
- 叶子：`include/decision/bt_nodes.hpp`；运行时接线：`src/mission_runtime.cpp` +
  `include/decision/mission_runtime.hpp`（执行器、TF、target_input、center_zone、patrol 游标）。
- 动作全部 `StatefulActionNode`，`onHalted` 清理；旧 FSM 的 Chase/Patrol 抢占竞态已修。
- `/decision/state` 仍发旧字符串（`HOME.WAIT_HOME`、`MOVE.GO_CENTER`、`CENTER.CHASE/PATROL`），外部脚本不用改。
- 测试：`test/test_bt_tree.cpp` 14 个场景（回家/回血/恢复回中心/LOST/到中心前不追敌/追敌与走位切换/
  抢占后恢复/走位组间停顿/追击失败回退/死亡 Holding/复活走隧道/残血走安全线）。

### 2.2 目标树（第二步，等接线/数据）

在 2.1 的骨架上按顺序补（**每个新条件都必须有"数据缺失时的安全默认"**）：

1. **阶段节点**：`game_progress`、`stage_remain_time`、时间窗 → `Prep/Hold`、`Match`、`Settle`。
2. **复活/虚弱**：`HP==0 → 死亡等待`；`HP 突增到 ~20% → Weak` → `ReturnToSupply`（碰补给区解除），
   不依赖读条字段；宁可多回一次家，也不要在虚弱期往控制区送。
3. **低血策略**：用哨兵自己的阈值（建议 low≈120、high≈320，而不是照抄 400），迟滞仍在黑板 latch。
4. **控制区任务**：默认中心控制区；按"时间/血量/是否被攻击/敌人是否在区内"在
   `Center / OwnHighland / EnemyHighland` 间切换。**不要依赖控制度数值**（拿不到），只做
   "在区内待够时间/被打就换位/残局守区"的启发式。
5. **交战**：目标只在"威胁到当前区/补给路线"时追；否则守区。standoff、重发死区、目标丢失超时
   全部复用 `target_input` 现有逻辑。
6. **可选增益**：隧道冷却增益/高地防御增益的路径顺带（没有协议字段，靠地图先验 + 自己位置）。
7. **安全兜底**：任何输入新鲜度超时（见 §4）→ `Hold`，绝不在"不知道比赛是否开始"时乱走。

> 控制区选择需要 2027 新场地的地图与航点。当前 `sentry_waypoints/waypoints/RMUL/*.csv` 是旧场地，
> 新场地（12m×8m、隧道/工作台/三控制区）必须先重录 `range.csv`/`center.csv`/`home_tunnel.csv`/`home_safe.csv` 等，
> 否则树只能跑旧中心逻辑。

## 3. 本轮已修的实机风险

1. **QoS 不兼容（已修）**：`serial/src/serial_driver_node.cpp` 的 `robot_status`/`game_status`
   发布从 `SensorDataQoS`（best_effort）改为 `QoS(KeepLast(10))`（reliable），与决策侧默认
   reliable 订阅匹配。**不修的表现**：话题存在、决策一帧收不到、无报错、FSM 永远 WAIT_HOME。
   新 BT 节点的 `/localization_status` 订阅也改成与 fast_location 一致的 `transient_local+reliable`。
2. **旧节点的动作生命周期竞态（已随旧节点删除）**：追击抢占巡逻后 `in_flight` 永久为 true、
   单槽事件丢事件；BT 用 `StatefulActionNode` 自持生命周期 + `onHalted` 清理，已有场景测试锁住。
3. **配置双真源**：`config/bt_action_replacement.yaml` 与 `src/bringup/config/common/decision.yaml`
   并存；BT 新参数必须两边同步（或只保留 bringup 一份），否则现场改了一份不生效。

## 4. 给电控的数据契约（建议，等 2027 协议发布后按版本适配）

当前上行帧（`src/serial/include/packet.hpp`）用 `payload_len` 变长，**追加字段不需要改帧型**：

- **建议追加（按优先级）**：`hp_max`（归一化用）→ `shooter_heat` / `heat_limit` / `cooling`
  （0x0202/0x0201）→ `buffer_energy`（0x0202）→ `ammo_17mm` / `gold`（0x0208）→
  `buffs`（0x0204）→ `rfid/event_data`（0x0209/0x0101）→ `warning`（0x0104）→
  `damage_diff`/`hurt_reason`（0x0003/0x0206）→ `self_pos`（0x0203，可不加，NUC 有 TF）。
- **下行**：2026 协议 0x0120（复活确认/立即复活/兑换/姿态）在 2027 是否保留未知；且 2027
  哨兵 750 发不可增、规则未见姿态机制，**先不要为 0x0120 做决策功能**，只在协议确认后加
  一个 `'H','D'` 帧（payload_len 同样预留）由电控转发。
- **适配层**：所有新字段进 decision 前先过 `referee_adapter`（`version + optional<T>`），
  缺失/超时 → `std::nullopt`；BT 条件节点遇到 `nullopt` 走保守分支（Hold/守区），
  不把 0 当真实值。

## 5. 路线图

| 阶段 | 内容 | 验收 |
| :- | :- | :- |
| ✅ P1 | 规则/数据/审计 + BT 骨架（XML/节点/单测/新可执行） | `colcon build/test --packages-select decision` 通过（11 ctest） |
| ✅ P2 | 删除 FSM；`bt_decision_node` 成为唯一生产节点；`MissionRuntime` 接管执行器/TF/追击/走位/状态发布；行为等价（站中心） | 9 个 ctest 全过（11 个 BT 场景 + helper/launch）；冒烟验证 游戏开始→MOVE.GO_CENTER、低血→HOME.WAIT_HP |
| P3 | 数据扩展：电控追加字段 + adapter + 新鲜度；先接 `hp_max/heat/ammo`，再加 zone/RFID | 单元测试 + 回放 bag |
| P4 | 战术节点：阶段/复活/控制区/隧道增益 + 2027 新航点 | 仿真对手/脚本化场景回归 |
| ✅ P5 | 旧节点/FSM 退役：launch 已切 `bt_decision_node`，旧可执行名仅作兼容别名 | 全量测试 + 现场演练（待实车） |

## 6. 文件索引

- `docs/decision/rules-and-data.md` — RMUL 2027 规则 + 2026 协议 + 数据矩阵（373 行）
- `docs/decision/current-audit.md` — 现有实现/接口/测试审计（398 行）
- `docs/decision/bt-scaffold.md` — BT 实现说明（生产版）
- `src/decision/decision_node/include/decision/bt_nodes.hpp` — 纯 C++ BT 节点
- `src/decision/decision_node/src/bt_decision_node.cpp` — 唯一生产可执行（订阅→黑板→tick→执行器）
- `src/decision/decision_node/src/mission_runtime.cpp` / `include/decision/mission_runtime.hpp` — ROS/执行器/追击/走位接线
- `src/decision/decision_node/config/trees/rmul.xml` — 生产行为树（PriorityFallback）
- `src/decision/decision_node/test/test_bt_tree.cpp` — BT 单测
