# 战斗感知决策 (Combat-Aware Decision) 改造计划

## 1. 目标

消除哨兵在中心区的"机械感":当前逻辑只看**比赛时间**和**绝对血量**，
到达中心后按固定 `patrol_interval_sec` 定时巡逻，对"此刻正在发生什么"完全无感知。

本计划让哨兵**对战斗本身做出反应**：同一个位置，会因为"正在挨打 / 正在对枪 / 平静"
而做出不同动作。全部只用哨兵**自身**的数据，不依赖队友坐标、不依赖敌方检测、不实时调用 LLM。

## 2. 现状与根因

- 决策输入只有三路：`game_active`、`current_hp`、`executor_event`（见 `decision_state_machine.cpp`）。
- `CENTER` 段是死循环：`WaitCenter --(死等 10s)--> Patrol --(跑完)--> WaitCenter`。
- `RobotStatus` 里已有但**完全未使用**的信号：
  - `is_attacked`：瞬时命中标志。
  - `shooter_heat`：枪口热量，非零≈正在开火/交战。
  - `current_hp`：其**变化率**是判断"是否正在被打、被打多凶"的最强信号。

## 3. 核心设计

### 3.1 战斗态势推断 (CombatAssessment)

只用两个直接信号判定，不做掉血率推断：

| 信号 | 来源 | 判定 |
|---|---|---|
| `firing` | shooter_heat 非零 | 我正在开火/最近射击 |
| `is_attacked` | RobotStatus 直接命中 | 我正在挨打 |

交战状态（优先级：开火 > 挨打 > 平静）：

```
ENGAGING   : 在开火（热量高）→ 站定输出
SUPPRESSED : 正在挨打（is_attacked）且没开火 → 换位脱离
CALM       : 无开火无挨打 → 守中心
```

切换防抖由状态机的 `engage_hold_sec` / `reposition_hold_sec` 最小保持时长承担，
`SUPPRESSED` 是关键——它区分了"我在对枪"和"我被人偷了打不还手"，
这两种情况人类哨兵手的反应完全不同。

### 3.2 CENTER 段改为交战驱动的战术子状态

保留三大状态框架（HOME / MOVE / CENTER），只重写 CENTER 内部：

```
CENTER
├─ HOLD       占区且平静 → 守中心锚点维持占领，漂移纠偏，不发起身运动
├─ ENGAGE     ENGAGING   → 站定当前有利位专注输出（乱走反而打不准）
├─ REPOSITION SUPPRESSED → 主动换到另一个战术点（脱离+重新找视野）
└─ REGAIN     丢失占领    → 重新冲占领点
```

候选战术点直接**复用现有 `patrol.csv` 的十几个点**（已分布在中心区周围），
`REPOSITION` 换位从这组点里随机挑 N 个（默认 3）不重复点连成路径依次走完，
中途判定正面交火就下发"当前位"停住；`HOLD` 只守中心锚点，不游走。
无需重新标点。

### 3.3 保命层不动

低血量撤退 (`hp_recovery`)、游戏结束回家的优先级**保持最高**，压在战斗逻辑之上。
最坏情况（信号缺失/异常）退化为当前的定时巡逻行为，不会比现在更差。

## 4. 代码改动清单

1. **`decision_config.hpp` / `decision_config.cpp` / yaml**
   - 新增 `CombatConfig`：`firing_heat_threshold`、`engage_hold_sec`、
     `reposition_hold_sec`、`reposition_grace_sec` 等阈值。

2. **`decision_context.hpp` / `.cpp`**
   - 暴露 `combatAssessment()`（直接依据 is_attacked / shooter_heat 判交战状态）。

3. **`decision_state.hpp` / `.cpp` / `types.hpp`**
   - `CenterSubstate` 扩展为 `Hold / Engage / Reposition / Regain`。
   - 更新状态字符串与 `targetForState` 映射（发布到 `/decision/state` 供 Web 显示）。

4. **`decision_state_machine.hpp` / `.cpp`**
   - `DecisionInputs` 增加 `CombatAssessment` 字段。
   - 重写 CENTER 段：按交战状态选子状态与战术路径；HOLD 守中心锚点，
     REPOSITION 随机 N 点连路径换位，ENGAGE 停当前位。

5. **`decision_node.cpp`**
   - 时间窗微分喂进 `DecisionInputs`。
   - 战术点选择 → 下发给 `WaypointExecutorClient`（复用现有下发链路）。

6. **测试**
   - 扩充 `test_decision_state_machine.cpp`：ENGAGING 站定、SUPPRESSED 换位、
     CALM 守中心、低血量抢占仍优先。

## 5. LLM 的位置

- **实车不跑**：NUC 纯 CPU，实时 LLM 会拖垮定位/导航，且延迟不可接受。
- **离线使用**：把 3.1/3.2 的"信号→状态→动作"表和全部阈值
  （开火/挨打判定、换位触发与保持时长）交给 LLM，
  用典型场景帮助想全边界、调优参数。这是当前硬件下 LLM 最实在的价值。
- **可选展示层**：如需答辩亮点，可在规则**外**并联一个云端慢层
  （2~3s 一次，手机热点兜底，断网退回规则）。加分项，非主线。

## 6. 动手前需确认的前置条件

1. **`is_attacked` 与 `shooter_heat` 实车下位机是否真的透传上来？**
   - 若已透传 → 全套可落地。
   - 若只有仿真面板发过 → 先降级：仅用 `current_hp` 变化率也能驱动
     ENGAGE/CALM/HOLD，`SUPPRESSED` 精度会下降但不阻塞主线。
2. 中心区战术点先用 `patrol.csv` 现有 5 点；后续可用 `waypoint_editor` 微调视野更好的位置。

## 7. 交付顺序

1. 阶段一：`DecisionContext` 加血量时间窗 + `CombatAssessment`（可先只用 hp 微分）。
2. 阶段二：扩展 `CenterSubstate` 与状态机 CENTER 段，接战术点选择。
3. 阶段三：补测试、实车联调阈值。
4. 阶段四（可选）：离线 LLM 调参 / 云端展示层。
