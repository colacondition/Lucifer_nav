# decision（BT 唯一生产节点）

导航决策层。**旧手写状态机（`DecisionStateMachine` / `decision_node.cpp`）已删除**，
现在由 BehaviorTree.CPP v3 的行为树做出"选哪条分支"的决定，`MissionRuntime` 只负责
ROS/导航/执行器接线。

- 可执行：`bt_decision_node`（节点名仍是 `bt_action_replacement`，为了兼容现有 yaml 段；
  安装时同时提供旧名 `bt_action_replacement_node` 别名给 bringup 的 launch）。
- 行为树：`config/trees/rmul.xml`（安装到 `share/decision/trees/rmul.xml`），改优先级只改 XML。
- 运行时：`src/mission_runtime.cpp` + `include/decision/mission_runtime.hpp`。
- 叶子/黑板：`include/decision/bt_nodes.hpp`。
- 设计背景、规则/协议数据矩阵、审计见 [`docs/decision/README.md`](../../../docs/decision/README.md)。

## 1. 行为（哨兵站中心，等价旧 FSM）

每拍从高优先级到低优先级重新求值，只保留一个 RUNNING 分支：

| 优先级 | 条件 | 行为 | 旧 FSM 对应 |
| :- | :- | :- | :- |
| ① | 定位 `LOST` | `Holding`：不新发 map 系目标，不取消已在跑的目标 | 完整性门 |
| ② | 比赛未进行（`game_progress != 4` 或时间窗不对） | 不在家 → `GoHomeSafe`；到家 → `WaitHome` | `MOVE.GO_HOME_SAFE → HOME.WAIT_HOME` |
| ③ | 低血（120/400 迟滞） | 不在家 → 回家；到家后原地 hold（**不需要 `wait_hp.csv`**）。**复活走 `home_tunnel.csv`，残血走 `home_safe.csv`** | `HOME.WAIT_HP / MOVE.GO_HOME_*` |
| ④ | 比赛已开始但还没收到血量 | 在家 `WaitHome`，等数据 | `HOME.WAIT_HOME`（等血量） |
| ⑤a | 还没到中心 | `GoCenter`（`center.csv`） | `MOVE.GO_CENTER` |
| ⑤b | 已到中心 | 先 hold `center`；每 `group_gap_sec` 在 `range.csv` 内随机取点走一组，再回 center；有敌目标时 `ChaseStandoff` | `CENTER` 段追敌/巡逻 |

关键语义：

- **先到中心，才允许追敌/走位**；追击不会在回家/去中心途中发生。
- 低血/比赛结束会抢占中心行为；恢复后重新走 `GoCenter`。
- 追击失败（无 TF/无目标/在死区）交还中心走位，不会卡住。
- 动作全部是 `StatefulActionNode`，被抢占时 `onHalted` 清自己的状态；
  旧 FSM 里"Chase 抢占 Patrol 后巡逻永久停住"的竞态已修。
- `/decision/state` 仍发布旧字符串（`HOME.WAIT_HOME`、`MOVE.GO_CENTER`、`CENTER.CHASE`、
  `CENTER.PATROL` 等），外部看板/脚本不需要改。

## 2. 输入 / 输出

输入：

- `game_status`（`GameStatus`）：比赛阶段 + 剩余时间。
- `robot_status`（`RobotStatus`）：`current_hp`（`is_attacked` 已进黑板，暂无战术分支）。
- `/localization_status`（`String`，`transient_local`）：第一段 `LOST` 时 Holding。
- `/auto_aim_target_pos`（`String`，云台系）：经 TF + `target_input` 转 map，standoff 1.5m，
  夹进中心区域（`center_zone`）。
- TF `map -> base_link`（雷达/云台位姿，不是 `base_link_fake`）。

输出：

- `/goal_pose`（回家/等待类目标的立即抢占）。
- `FollowWaypoints` action（`/waypoint_editor/follow_waypoints`）。
- `/decision/state`（latched String）。
- `center_patrol/target_debug`（0.5s 节流调试）。

## 3. 配置

`config/bt_action_replacement.yaml` 与 `src/bringup/config/common/decision.yaml` 是**两份**
参数文件，现场改哪份取决于 launch 传的 `params_file`；改阈值/航点时两边同步，别只改一份。
航点真源：`sentry_waypoints/waypoints/RMUL/*.csv`（`center.csv`、`range.csv`、
`home_tunnel.csv`、`home_safe.csv`、`wait_home.csv`）。

主要参数：

- `game.progress / lower_remain_time / higher_remain_time`
- `hp_recovery.low_threshold / high_threshold`
- `target_chase.*`（standoff、有效/丢失时间、重发死区）
- `center_patrol.*`（网格间距、每组点数、组间停顿）
- `center_zone.*`（`lidar_frame=base_link`、`margin_m`）
- `waypoint_executor.*`、`maintain_goal.*`
- `loop_hz`（默认 10Hz；BT tick 周期）

## 4. 构建 / 运行 / 测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select decision
source install/setup.bash

ros2 launch decision decision.launch.py          # 起决策；执行器由导航栈起
ros2 run decision bt_decision_node               # 单独起（需自己配航点参数）
colcon test --packages-select decision           # 9 个 ctest，含 14 个 BT 场景
```

BT 场景测试覆盖：游戏未进行、低血进入/退出、恢复后回中心、LOST、到中心前不追敌、
中心内追敌/走位切换、抢占后走位恢复、走位组间停顿基准、追击失败回退、死亡 Holding、
复活走隧道、残血走安全线。

## 5. 已知限制 / 下一步

- 当前战术只有"站中心"；控制区占领、复活/虚弱、热量/弹药、判罚等节点等 2027 协议字段
  到位后再加（`docs/decision/rules-and-data.md` 有数据矩阵）。
- 新场地（2027 12m×8m）航点/中心区尚未重录，`sentry_waypoints/RMUL` 仍是旧场地。
- 参数双真源（包内 + bringup）暂时保留；后续可只留 bringup 一份。
