# BT 决策实现说明（生产版）

> 状态：**生产**。旧 FSM（`DecisionStateMachine` / `decision_node.cpp`）已删除；
> `bt_decision_node` 是唯一生产可执行，`MissionRuntime` 负责 ROS/执行器接线，行为树只决定
> "选哪条分支"。设计背景见 [`README.md`](README.md)，规则/数据见 [`rules-and-data.md`](rules-and-data.md)。

## 1. 文件

| 文件 | 作用 |
| :- | :- |
| `include/decision/bt_nodes.hpp` | 与 ROS 无关的 BT 资产：黑板 key、`MissionBackend` 接口、`DecisionLatch`、条件/动作节点、`PriorityFallback`、`registerDecisionNodes()` |
| `include/decision/mission_runtime.hpp` / `src/mission_runtime.cpp` | 生产后端：订阅、低血迟滞、比赛判定、执行器 action、TF、`target_input`、`center_zone`、patrol 游标、`/goal_pose`、`/decision/state`、target_debug |
| `src/bt_decision_node.cpp` | 唯一生产节点：创建 runtime + 加载 XML + 10Hz tick；节点名保留 `bt_action_replacement` 兼容 yaml |
| `config/trees/rmul.xml` | 生产行为树（安装到 `share/decision/trees/rmul.xml`） |
| `test/test_bt_tree.cpp` | 14 个场景：真实 XML + 真实叶子节点 + `FakeBackend`，不需要 ROS |

旧可执行名 `bt_action_replacement_node` 仍安装，但只是 `bt_decision_node` 的**别名**（给 bringup
的 launch 兼容）；两者不要同时启动。

## 2. 树与优先级

```
PriorityFallback（每拍从高到低重选，只保留一个 RUNNING 分支）
├── ① LocalizationLost → Holding
├── ② !GameActive      → HomeReached ? WaitHome : (GoHome → WaitHome)
├── ③ HpZero           → Holding（死亡等复活）
├── ④ HpLow            → HomeReached ? WaitHp
│                         : Revived ? (GoHomeTunnel → WaitHp)
│                                   : (GoHomeSafe → WaitHp)
├── ⑤ !HpKnown         → HomeReached → WaitHome
├── ⑥a !CenterReached  → GoCenter
└── ⑥b CenterReached   → PriorityFallback:
                          ChaseReady ? ChaseStandoff : PatrolCenter
```

- 为什么自定义 `PriorityFallback` 而不是内置 `ReactiveFallback`：内置版本会继续 tick 低优先级兄弟，
  且允许出现多个 RUNNING 的异步子节点；这里要"高优先级抢占 + 任何时刻只有一个 RUNNING 分支"。
- `Holding` 不取消执行器里正在跑的目标（与旧完整性门一致），只是不再新发 map 系目标。
- 低血/比赛结束抢占中心；恢复后重新走 `GoCenter`。
- 动作全部 `StatefulActionNode`：`onStart` 下发/等待，`onRunning` 消费执行器终态，`onHalted` 清状态；
  因此 Chase 抢占 Patrol 后 Patrol 不会永久 `in_flight`。

## 3. 黑板

节点每拍 `syncBlackboard()` 写入派生 bool，树只读这些 bool：

| key | 类型 | 说明 |
| :- | :- | :- |
| `mission_backend` | `MissionBackend*` | runtime 指针，动作通过它下发/查询 |
| `game_active` | bool | 比赛进行中（progress + 时间窗） |
| `hp_low` | bool | 低血迟滞结果（120 进 / 400 出） |
| `hp_known` | bool | 收到过 robot_status |
| `hp_zero` | bool | 血量为 0（死亡等待复活） |
| `revived` | bool | 本局刚复活（0→>0），到补给区后清除 |
| `localization_lost` | bool | `/localization_status` 第一段 == LOST |
| `home_reached` | bool | 旧 FSM 的 HOME 段锁存 |
| `center_reached` | bool | 旧 FSM 的 CENTER 段锁存 |
| `chase_ready` | bool | 可追（enabled + 中心区 + 目标有效 + TF） |

段锁存 `DecisionLatch`：`GoCenter` 下发时清 `home_reached`，`Center` 成功时置 `center_reached`；
`GoHome` 下发时清 `center_reached`，`Home` 成功时置 `home_reached`。生产 runtime 与单测共用同一实现。

## 4. XML 约定 / 如何加节点

- 根：`<root main_tree_to_execute="RMULDecision">`；标签名必须与 `registerDecisionNodes()` 注册 ID 完全一致。
- 调优先级 = 调 `PriorityFallback` 子节点顺序；不要在 XML 里写比较/迟滞，派生 bool 在 runtime 算。
- 新增条件：从 `BT::ConditionNode` 派生，读黑板 bool（`DECISION_BOOL_CONDITION` 宏可省样板）。
- 新增动作：继承 `MissionActionBase`（已处理 `onStart/onRunning/onHalted`、目标下发、结果消费），
  或自己实现三件套；被抢占时必须在 `onHalted` 清本地状态。
- 新增输入：在 runtime 里维护/计算，`syncBlackboard()` 写入；不要在 XML 里放 ROS msg。

## 5. 构建 / 测试 / 冒烟

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select decision
source install/setup.bash
colcon test --packages-select decision --event-handlers console_direct+
ros2 launch decision decision.launch.py            # 生产入口（执行器由导航栈起）
```

冒烟（无执行器也能看状态切换）：

```bash
ros2 topic pub --once /game_status decision_interfaces/msg/GameStatus \
  "{game_type: 4, game_progress: 4, stage_remain_time: 300}"
ros2 topic pub --once /robot_status decision_interfaces/msg/RobotStatus \
  "{robot_id: 7, current_hp: 400, is_attacked: false}"
# 预期 /decision/state: HOME.WAIT_HOME -> MOVE.GO_CENTER
ros2 topic pub --once /robot_status decision_interfaces/msg/RobotStatus \
  "{robot_id: 7, current_hp: 100, is_attacked: true}"
# 预期 -> HOME.WAIT_HP
```

## 6. 已知限制

- 战术只有"站中心"；控制区/复活/热量/弹药等节点等 2027 协议数据到位后再加。
- 2027 新场地航点/中心区尚未重录，`sentry_waypoints/RMUL` 仍是旧场地。
- 参数两处（包内 yaml + bringup common）并存，现场改哪份取决于 launch 的 params_file。
