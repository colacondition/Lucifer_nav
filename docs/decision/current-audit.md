# 决策重构 B：现有决策 / 接口审计与 BT 叶子清单

> 范围（只读）：`src/decision/decision_node`、`src/serial`、`src/interfaces`、`src/control/waypoint_editor`。
> 本文**未改任何代码、未 git commit**；全部结论给 `文件:行号`，行号来自当前工作树 `grep -n` / `read`。
> 路径缩写：`D/`=`src/decision/decision_node/`，`S/`=`src/serial/`，`W/`=`src/control/waypoint_editor/`，
> `I/`=`src/interfaces/`，`R/`=`docs/decision/rules-and-data.md`（task-11，规则与数据可得性），
> `BT/`=`docs/decision/bt-scaffold.md`（task-13，BT 骨架），`sm.cpp`=`D/src/decision_state_machine.cpp`，
> `H/`=`/opt/ros/humble/include/rclcpp/rclcpp/`。查不到的写「未见」，不做推断。

## 0 摘要

1. **现有决策 = 5 叶子 FSM + CENTER 段三选一行为 + 一个 action 客户端**。所有行为最终都收敛到同一件事：
   给 `waypoint_executor` 下发一条航点路径（`/goal_pose` 只在回家类目标做"立即抢占"额外直发）。
   FSM 本身只有三个输入：`game_active`、`current_hp`、`executor_event`（`D/include/decision/decision_state_machine.hpp:19-23`）。
2. **三个必须先解决的设计问题**（BT 重构的输入，详见 §7.1）：
   - **问题① 输入契约不可靠也不完整**：串口上行只有 3 个字段；`is_attacked`/`game_type` 已算出却无人消费；没有新鲜度/超时；
     发布端 QoS 与订阅端 **静态不兼容**（best_effort vs reliable，见 §3.3 I-1，需实机复核）。
   - **问题② 动作生命周期与 FSM 事件强耦合**：完成事件是**单槽** `optional`（`D/src/decision_node.cpp:194`）；preempt 时旧动作的
     终态可能被客户端丢弃，而"走位一组已完成"的记账只挂在旧动作的回调上（§7.1 B-2）——BT 必须改成动作自己持有生命周期。
   - **问题③ 决策维度缺失**：只有"去哪儿"，没有**阶段/控制区/占领/虚弱/热量/弹药/姿态/aggression**。若 BT 只把现有 if-else 改画成树，
     等于换壳；重构前必须决定"本迭代要补哪些 context 条件"，否则 XML 里的条件节点无处可取。
3. **测试覆盖"单元厚、链路薄"**：11 个 gtest 文件共 117 个场景（§6.1），但 `decision_node.cpp`（927 行，全部行为编排）
   **没有任何节点级测试**，也没有 decision↔executor 的真实 action 集成、没有 QoS/串口端到端、没有 preempt 生命周期用例（§6.2）。
4. **BT 叶子复用度高**：现有模块大多已经"无 ROS 纯逻辑 + 薄节点层"，条件叶子 11 个（另 2 个待数据）、同步动作 8 个、异步动作 5 个可直接包（§5）。
   真正需要重写的是"编排层"和"动作生命周期"，不是算法。

## 1 现有 FSM 与每拍数据流

### 1.1 状态结构（3 大类、5 叶子）

- 用 `std::variant` 表示：`HomeState{WaitHome,WaitHp}`、`MoveState{GoHome,GoCenter}`、`CenterState{}`；
  **CENTER 没有子状态**（`D/include/decision/decision_state.hpp:14-32`；删除 HOLD/ENGAGE/REPOSITION 的原因见 `D/README.md:33-35`）。
- 初始状态固定 `HOME.WAIT_HOME`（`D/src/decision_state_machine.cpp:50`）。
- 状态 → 字符串（`/decision/state`）与状态 → 目标名的映射集中在两处，新增状态必须同时改
  （`D/src/decision_state.cpp:8-20`、`:22-36`；`D/README.md:541-548` 的注意事项）。

| 叶子 | 字符串 | 下发目标 | 语义 |
|---|---|---|---|
| `HOME.WAIT_HOME` | `HOME.WAIT_HOME` | `WaitHome`（守点型） | 比赛未开始，出生点等待（`D/src/decision_state.cpp:13,27`） |
| `HOME.WAIT_HP` | `HOME.WAIT_HP` | `WaitHp`（守点型） | 低血回家等待恢复 |
| `MOVE.GO_HOME` | `MOVE.GO_HOME` | `Home`（一次性） | 从场上回家 |
| `MOVE.GO_CENTER` | `MOVE.GO_CENTER` | `Center`（一次性） | 回家恢复后去中心 |
| `CENTER` | `CENTER` | `Patrol` / `Chase` / `WaitCenter` | 中心段：走位 / 追击 / 关走位时守锚点 |

### 1.2 tick 优先级（代码真实顺序）

`DecisionStateMachine::tick()`（`D/src/decision_state_machine.cpp:34-103`）每拍按固定优先级：

1. **血量迟滞状态更新**：`hp_recovery_active_` 在 `<low_threshold` 置位、`>=high_threshold` 清除
   （`sm.cpp:37-43`，阈值默认 120/400，`D/include/decision/decision_config.hpp:52-56`）。
   注意这是 FSM 的**内部记忆**，不是每拍纯函数——BT 里必须变成黑板 latch 或带状态的条件节点。
2. **优先级 1：比赛未进行**（`sm.cpp:45-61`）：`WAIT_HOME` 保持；`WAIT_HP→WAIT_HOME`；`GO_HOME` 到家才转 `WAIT_HOME`；
   其余（含 CENTER）一律 → `GO_HOME`。
3. **优先级 2：低血恢复**（`sm.cpp:66-80`）：在家→`WAIT_HP`；`WAIT_HP` 保持；`GO_HOME` 到家转 `WAIT_HP`；其余→`GO_HOME`。
4. **优先级 3：正常流程**（`sm.cpp:82-100`）：`WAIT_HP`/`GO_HOME` → `GO_CENTER`（"恢复完成"）；
   `WAIT_HOME` 需**已收到血量**才 `GO_CENTER`（`:88-91`）；`GO_CENTER` 只认 `Center+Succeeded` 才进 `CENTER`（`:93-96`）；
   `CENTER` 无内部迁移（`:97-100`）。

`DecisionInputs` 只有三个字段（`D/include/decision/decision_state_machine.hpp:19-23`）；事件匹配必须"目标+类型"同时相等（`sm.cpp:14-19`）。

### 1.3 每拍数据流（30 Hz）

`onTimer()` 入口 `D/src/decision_node.cpp:179`，实际顺序：

```text
1. 查重名/查双执行器（5 s 节流）                  D/src/decision_node.cpp:181-186, 469-525
2. 执行器超时守卫：无任何 action 事件则合成 Aborted   :188-190；实现 D/src/waypoint_executor_client.cpp:122-160
3. 组装 inputs：game_active/hp/exchange(单槽事件)    :191-194
4. FSM tick → {state,target,transitioned,reason}    :195
5. 状态变化才发 /decision/state（latched）           :197-203（初值在 :69-71）
6. 定位闸：仅 LOST 时停止 map 系下发，状态机已推进    :208-215
7. CENTER ? handleCenterTactical() : 清走位账 + 固定映射 :219-233
8. target_debug（0.5 s 节流）                        :233, 797-830
```

两个细节值得注意：① 超时守卫**先于**事件 `exchange`，所以守卫合成的 Aborted 本拍就能进 FSM（`D/src/decision_node.cpp:188-194`，
注释见 `:186-190`）；② 定位 LOST 时 `chaseStandsDown("localization_lost")` 但仍继续 tick（`:208-215`），
即"状态照走、目标不发"。

### 1.4 CENTER 段：三选一行为

`handleCenterTactical()`（`D/src/decision_node.cpp:253-309`）：

1. `center_patrol.enable==false` → 下发 `WaitCenter` 守锚点（`:255-262`，故意用守点型目标）。
2. `dispatchChaseIfAble()` 返回 true → 追击占住这一拍，整段走位跳过（`:265-268`）。
3. 否则走位：`active_pool_` 为空→原地不动（`:271-276`）；上一组在飞→返回（`:277-279`）；
   未到 `next_patrol_sec_`→返回（`:280-283`）；按游标取 `path_len` 个点（`:285-289`）；
   下发（`:296-301`）；成功才推进游标并置在飞（`:301-303`）。
   完成/失败都算"这一组结束"，在 action 结果回调里置 `patrol_path_done_` 并把下次时刻从**走完那一刻**起算
   （`:83-109`，尤其 `:88-99`）。

追击：`dispatchChaseIfAble()`（`:683-756`）。顺序是"先判目标可用、再查 TF"，避免 TF 抖动干扰主决策（`:694-706`）；
goal = 云台系目标 → map（`targetToMap`，`D/src/target_input.cpp:197-239`）→ standoff 圆 → 夹进区域
（`standoffGoal`/`clampToZone`，`D/src/target_input.cpp:174-195`、`D/src/center_zone.cpp:247-268`）；
再按 `min_goal_move_m` 死区和 `shouldReissue` 判重发（`:722-745`）。"追击占不占这一拍"是显式两出口
`chaseOwnsTick`/`chaseStandsDown`（`:768-795`），下发失败必须交还主决策（`:735-741`）。

## 2 模块职责与接口

| 模块 | 职责 | 关键接口 | 证据 |
|---|---|---|---|
| `DecisionContext` | 保存最近一帧 RobotStatus/GameStatus + localization_status；给 hp/lost 查询 | `setRobotStatus/setGameStatus/setLocalizationStatus`、`effectiveCurrentHp`、`localizationLost` | `D/include/decision/decision_context.hpp:28-62`；`:17-26` 的 `integrityStateIsLost` |
| `DecisionStateMachine` | 5 叶子迁移、优先级、血量迟滞、事件匹配 | `tick(DecisionInputs)`、`state()`、`hpRecoveryActive()` | `D/src/decision_state_machine.cpp:34-103` |
| `TargetInput` | 解析自瞄串、连续有效防抖、一阶滤波、丢失容忍、重发判据、云台系→map | `parseTargetPayload`、`update`、`state`、`shouldReissue`、`markReissued`、`standoffGoal`、`targetToMap` | `D/src/target_input.cpp:50-83,96-157,159-172,174-239` |
| `CenterZone` | 四角点→凸安全区（内缩 margin）、点在区内、最近点夹取 | `makeCenterZone`、`zoneContains`、`clampToZone`（`zoneContainsAll` 仅测试用） | `D/include/decision/center_zone.hpp:23-53`；`D/src/center_zone.cpp:145-268` |
| `CenterPatrol` | 区域网格（格子中心+蛇形）、按游标取一组点 | `generateZoneGrid`、`takePatrolPath` | `D/include/decision/center_patrol.hpp:31-42`；`D/src/center_patrol.cpp:10-79` |
| `WaypointExecutorClient` | action 下发/防抖/抢占/守点漂移重发/结果超时合成/终态回调 | `shouldRequestTarget`、`stepExecutorTarget`、`checkGoalTimeout`、`setResultCallback`、`targetNeedsMaintainGoal` | `D/include/decision/waypoint_executor_client.hpp:30,72-114`；`D/src/waypoint_executor_client.cpp:58-160,163-322,375-384` |
| `WaypointStore` | CSV 读取/缓存、锚点、起点前方选点 | `loadWaypoints`、`anchorPose`、`prepareFollowWaypoints` | `D/src/waypoint_store.cpp:41-68,98-152,182-206` |
| `DecisionConfig` | 参数声明/读取/clamp | `declareDecisionParameters`、`loadDecisionConfig`、`validateDecisionConfig` | `D/include/decision/decision_config.hpp:141-166`；`D/src/decision_config.cpp:180-195` |
| `decision_node` | ROS 装配 + 编排（订阅、TF、CENTER 战术、下发、调试） | `onTimer`/`handleCenterTactical`/`dispatchChaseIfAble`/`stepExecutorTarget` | `D/src/decision_node.cpp:55-161,179-309,363-403,683-830` |

状态机刻意"不看时间"（`D/include/decision/decision_state_machine.hpp:39-41`），时间只在节点层用于走位组间隔、
守点漂移、debug 节流。这条边界在 BT 里应保持：**FSM/树只读条件，计时由动作/黑板承担**。

## 3 输入源审计

### 3.1 现有输入与使用情况

| 输入 | 来源 / 类型 | 字段 | 当前用途 | 未用 / 问题 |
|---|---|---|---|---|
| `game_status` | serial 裁判上行帧 `'H','R'` → `I/custom_msgs/msg/GameStatus.msg` | `game_type`、`game_progress`、`stage_remain_time` | `gameActive()` 只用 `progress==game.progress(4)` 且 `0<=remain<=300`（`D/src/decision_node.cpp:170-178`） | `game_type` **未校验**（协议里 4=3v3，`I/custom_msgs/msg/GameStatus.msg:6-11`；`S/include/referee_conversion.hpp:22-30` 已拆位但只写进 msg）；无阶段语义（0 未开始/1 准备/2 自检/3 倒计时/4 比赛中/5 结算，`GameStatus.msg:10`） |
| `robot_status` | serial 裁判上行帧 → `RobotStatus.msg` | `robot_id`、`current_hp`、`is_attacked` | 只用 `current_hp`（`D/src/decision_context.cpp:26-33`）；`is_attacked` **零消费者**（全仓库 grep 无决策侧引用） | `robot_id` 未用；`is_attacked` 由串口侧跨帧差分算出（`S/src/serial_driver_node.cpp:309-311`；`S/include/referee_conversion.hpp:33-36,52-60`）但没人用；单帧无 max_hp（上限靠本地常量，`D/doc/sentry_tactics_rmul.md:364-373`） |
| `localization_status` | fast_location，`std_msgs/String`，reliable+transient_local（`src/localization/fast_location/src/robot_localization.cpp:335-339`） | `"<STATE> <reason>"` | 只取首 token 判 `LOST`（`D/include/decision/decision_context.hpp:17-26`）；`LOST` 时冻 map 系下发（`D/src/decision_node.cpp:208-215`） | `OK nis_reject`/`projected` 一律当 OK（有意，`D/README.md:8-9`）；无新鲜度（从没收到→不拦，`D/src/decision_context.cpp:62-71`） |
| `auto_aim_target_pos` | sp_vision_25，`std_msgs/String`，RELIABLE（`D/src/decision_node.cpp:128-145` 注释说明必须 reliable） | `"x,y,valid,id"`，云台系位置（`D/include/decision/target_input.hpp:11-33`） | 仅 CENTER 段使用：防抖 `min_valid_sec`、丢失容忍 `lost_timeout_sec`、EMA 滤波、重发门控（`D/src/target_input.cpp:96-172`） | 无速度/无预测（状态只有 x,y,range，`target_input.hpp:73-80`）；无 header，用到达时刻（`D/src/decision_node.cpp:136-137`；`D/include/decision/target_input.hpp:91-93`） |
| TF `map→base_link_fake` | TF | 位姿 | 守点漂移判定（`D/src/decision_node.cpp:405-412,363-403`；frame 来自 `D/include/decision/decision_config.hpp:100-106`） | 只用于守点型目标（`targetNeedsMaintainGoal`，`D/src/waypoint_executor_client.cpp:375-384`） |
| TF `map→base_link` | TF | 位姿 | 追击：把云台系目标转 map（`D/src/decision_node.cpp:455-462`；frame 配置 `decision_config.hpp:80-92`） | 拿不到 TF 时本拍不追（`decision_node.cpp:700-706`） |
| `gimbal_posture_state` | serial 云台实测（`S/src/serial_driver_node.cpp:95-96`） | 状态 | **决策不订阅**（只有导航 `rm_tunnel_posture`/MPC 用） | 决策侧姿态相关行为全缺（§3.2） |

### 3.2 战术设计需要 vs 现有可得性（与 task-11 对齐）

`R:206-284` 的可得性矩阵结论在实现侧的表现：**串口上行帧只有 3 个净荷字段**
（`S/include/packet.hpp:41-43,133-141`；接口说明 `S/doc/referee_frame.md:16-37`，其中 `:35-37` 明确列出"曾经列过又砍掉"的
中心占领状态、队友血量、伤害差、射击热量、中心 RFID）。因此：

| 决策需要的输入 | 协议来源 | 现状 | 影响 |
|---|---|---|---|
| 阶段/比赛是否进行 | 0x0001 | ✅ 仅 progress+时间窗（硬编码 4 与 0..300） | 无法区分准备/自检/倒计时/结算/复活 |
| 自身血量 | 0x0201 | ✅ 10 Hz | 低血阈值静态 120/400 |
| 占领状态（谁在占） | 0x0101 bit23-24 | ❌ 未转发 | 无法"守/抢"，BT 的 phase 条件缺依据（`R:224-258`） |
| 中心 RFID（是否压点） | 0x0209 bit23 | ❌ 未转发 | 无低延迟"进区"判据 |
| 热量/缓冲能量 | 0x0202 | ❌ 未转发 | 无法判"在开火/交战" |
| 弹量 | 0x0208 | ❌ 未转发 | 无法做弹药相关决策 |
| 队友位置/血量 | 0x020B / 0x0003 | ❌ 未转发 | 无法做撤离是否安全/让位 |
| 伤害差 | 0x0003 | ❌ 未转发 | 无法做平局保险 |
| 虚弱/复活读条 | 无直接字段 | ❌ 无 | 复活后回血/无敌期把路全缺（`R:62-69`） |
| 姿态（0x020D） | 3v3 是否生效待确认 | ❌ 未转发 | 云台/姿态决策在电控侧，非本节点（可接受） |

**结论**：BT 重构的第一约束是数据可得性（与 `R:11-29` 一致），不是树结构。短期只能用"时间/血量/定位/视觉目标/自身位置"
构造条件；占领、热量、弹药、队友、虚弱必须显式列为"未接入数据"，否则树里会出现假条件。

### 3.3 输入健康度问题（按严重度）

**I-1（高，静默）发布/订阅 QoS 静态不兼容 —— 需实机复核。**
serial 侧 `robot_status`/`game_status` 用 `rclcpp::SensorDataQoS()` 发布（best_effort, depth 5，`S/src/serial_driver_node.cpp:89-92`；
Humble 定义见 `H/qos.hpp:356-380`），decision 侧用 `create_subscription<T>(topic, 10, cb)` 订阅（`D/src/decision_node.cpp:110-118`），
该重载等价 `rclcpp::QoS(10)`，默认 **RELIABLE**（`H/qos.hpp:119-142`）。ROS 2 的兼容规则要求发布端 offered ≥ 订阅端 requested，
**best_effort 发布端无法匹配 reliable 订阅端** → 运行时表现为 "incompatible QoS, no messages will be sent"，且不报错。
日常测试掩盖它的原因：手工测试 GUI 用默认 reliable 发布（`D/scripts/test_topics_publisher_gui.py:46-47`），
仿真 launch 不启动 decision（`src/bringup/launch/sim.launch.py` 内无 decision 节点），只有实车串口链路才暴露。
**复核命令**：`ros2 topic info -v /robot_status`（看 offered/requested reliability）+ `ros2 topic hz /robot_status`；
若成立，BT 重构第 0 步先修：serial 发布改 `rclcpp::QoS(10).reliable()`（10 Hz，代价可忽略）或 decision 订阅改
`rclcpp::SensorDataQoS()`，并把 QoS 写进统一配置/launch 测试断言。

**I-2（高）无数据新鲜度/断链检测。** `DecisionContext` 存了 stamp（`D/include/decision/decision_context.hpp:48-58`），
但 `robotStatusStampSec()` **无消费者**（全仓库仅定义），`gameStatus()` 也没有超时判定（`D/src/decision_context.cpp:53-60`）。
串口断流后上一帧 `game_active=true` 会永远保持，决策继续按"比赛中"下发目标。BT 需要 `GameDataFresh`/`HpFresh` 条件
或统一 `input_timeout`。

**I-3（中）`game_type` 未校验、阶段硬编码。** 拆位已实现（`S/include/referee_conversion.hpp:22-30`），但 decision 只比较
`progress==4` 与 `0..300s`（`D/src/decision_node.cpp:170-178`、`D/include/decision/decision_config.hpp:45-50`）。
接错 RMUC/旧规则时会"看起来在跑"。

**I-4（中）`is_attacked` 是有效输入但无人用。** 串口侧已算（`S/src/serial_driver_node.cpp:309-311`），决策侧零引用；
`R` 与 `D/doc/sentry_tactics_rmul.md` 均认为"被打"是可用的态势信号之一（但后续简化决定不用，`D/doc/sentry_tactics_rmul.md:340-352` 说明不索取更多）。

**I-5（中）目标只有瞬时位置。** `TargetInputState` 只有 `x,y,range,published_id`（`D/include/decision/target_input.hpp:73-80`），
滤波是一阶 EMA（`D/src/target_input.cpp:135-145`），没有速度/航迹/预测。若后续接导航侧的"动态障碍预测层"，
这里是唯一可扩展的入口。

## 4 输出审计

| 输出 | 类型 / QoS | 谁消费 | 现状与问题 |
|---|---|---|---|
| `/decision/state` | `String`，reliable+transient_local（`D/src/decision_state.cpp:38-48`） | Web/调试、GUI（`D/scripts/test_topics_publisher_gui.py:51-56`） | 只在**状态迁移**时发布（`D/src/decision_node.cpp:197-203`）+ 启动初值（`:69-71`）；CENTER 内追击/走位变化不可见，只有 `target_debug` 能看 |
| `/goal_pose` | `PoseStamped`，depth 10（`D/src/decision_node.cpp:68`） | 导航栈 | **仅 Home/WaitHome 立即抢占时直发首点**（`D/src/waypoint_executor_client.cpp:246-249`；`shouldPreemptImmediately` 只对这两个目标为真，`:84-88`）。与 action 通路并存，BT 需明确"唯一目标生产者" |
| `/waypoint_editor/follow_waypoints`（action） | `I/custom_msgs/action/FollowWaypoints.action`：goal=`Path + stop_at_final`；result=`success+message`；feedback=`current_index+status`（`I/custom_msgs/action/FollowWaypoints.action:1-20`） | waypoint_executor（`W/src/waypoint_executor.cpp:114-166`） | 决策只创建 follow client（`D/src/decision_node.cpp:76-77`）；`feedback` **从未使用**（grep 无 feedback 引用），动作进度对决策不可见 |
| `/waypoint_editor/through_waypoints`（action） | 同 action | **无 server** | 决策创建了 through client（`D/src/decision_node.cpp:78-79`），但 executor 只注册一个 action（默认 follow，`W/src/waypoint_executor.cpp:77,114-166`；bringup 只起一个实例与 follow action：`src/bringup/config/common/waypoint_executor.yaml:27-29`）；且节点层从未以 `ExecutorThrough` 调用（`grep ExecutorThrough` 仅类型定义+客户端分支）→ **死通路** |
| `/waypoint_editor/executor_waypoints`、`/waypoint_editor/saved_waypoint_file` | `Path`/`String`，latched | RViz 面板旧通路 | 决策每次下发都顺带发布（`D/src/waypoint_executor_client.cpp:235-245`），只为面板兼容；action 内已自带航点 |
| `center_patrol/target_debug` | `String`，0.5 s 节流（`D/src/decision_node.cpp:797-830`；`next_target_debug_sec_` 节流 `:806`） | 现场调试 | 每拍更新"为什么没追"的判据；BT 应保留为叶子输出而非散在节点里 |
| executor 状态话题（`/waypoint_editor/follow_status`、`/navigation2/status`） | `String` | RViz / executor 内部 | 决策**不再订阅**（README §7 的订阅表已过时，§7.2 鸡肋点） |

**action 的真实语义（BT 必须建模）**：executor 每收到新 goal 会**抢占并 join 上一个 worker**
（`W/src/waypoint_executor.cpp:523-539`），旧任务以 CANCELED 结束（`:578-585`）；执行中每个航点到达即发 `goal_pose`
（`:335-361,563-624`）；`stop_at_final=false` 时终点也不停、全程关闭接近段（`:361-362,596-599`；接口说明见 `I/custom_msgs/action/FollowWaypoints.action:6-14`）。决策侧只消费 result，
不消费 feedback，也不主动调用 cancel（`grep async_cancel_goal` 零命中）——"停止"完全依赖"发新 goal 抢占"。

## 5 可复用为 BT 叶子的清单

> 约定：**条件节点**=`ConditionNode`（纯读黑板）；**同步动作**=`SyncActionNode`（一拍内完成）；
> **异步动作**=`StatefulActionNode`（onStart/onRunning/onHalted，持有跨拍状态）。入口均给现有实现位置。

### 5.1 条件节点（可优先落地）

| 建议叶子名 | 语义 | 现有入口 | 依赖 |
|---|---|---|---|
| `GameActive` | 比赛进行中 | `D/src/decision_node.cpp:170-178` | `game_status`；应加 `game_type==4` 与新鲜度（I-2/I-3） |
| `HpKnown` | 已收到有效血量 | `D/src/decision_state_machine.cpp:88-90` | `robot_status` |
| `HpLowHysteresis` | 低血迟滞态（带 latch） | `sm.cpp:37-43,66` | `current_hp` + 阈值；latch 需上黑板 |
| `LocalizationOk` | 定位非 LOST | `D/src/decision_node.cpp:208`；`D/include/decision/decision_context.hpp:17-26,67-71` | `/localization_status` |
| `InCenterPhase` | 处于中心段 | `D/src/decision_node.cpp:219`（variant 判断） | FSM/phase；BT 里应由 phase 条件或动作结果产出 |
| `TargetUsable` | 自瞄目标稳定可用 | `D/src/target_input.cpp:150-157` | `target_input_.state()` |
| `ZoneUsable` | 中心区四角有效 | `D/src/decision_node.cpp:689`；加载 `:605-681` | `range.csv`、`makeCenterZone` |
| `PatrolPoolAvailable` | 有走位点池 | `D/src/decision_node.cpp:271-276`；构建 `:533-571` | `range.csv`/`patrol.csv` |
| `ExecutorSucceeded(target)` | 某目标以成功终态结束 | `sm.cpp:14-19`；事件产生 `D/src/decision_node.cpp:83-109` | action result 事件（单槽，需改队列） |
| `ExecutorGoalInFlight` | 有 goal 在飞/被接受 | `D/src/waypoint_executor_client.cpp:321-322`；`state()` `:15-19` | 客户端状态 |
| `TargetMovedEnough` | goal 离上次下发超过死区 | `D/src/decision_node.cpp:724-727`；`D/src/target_input.cpp:159-172` | 目标/上次下发点 |
| `GameDataFresh`（**新增**） | 输入未超时 | **未见**（I-2） | 新增 stamp 消费 |
| `CenterOwned` / `WeakState`（**新增，需先有数据**） | 占领方/虚弱期 | **未见**（§3.2；协议 0x0101/0x0209 未转发） | 后续接串口 |

### 5.2 同步动作（一拍内完成，纯计算或发布）

| 建议叶子名 | 语义 | 现有入口 | 依赖 |
|---|---|---|---|
| `PublishDecisionState` | 发布状态字符串 | `D/src/decision_state.cpp:43-48`；`D/src/decision_node.cpp:69-71,197-203` | latched publisher |
| `PublishTargetDebug` | 发布追击调试串 | `D/src/decision_node.cpp:797-830` | debug publisher（节流 0.5 s） |
| `ComputeChaseGoal` | 云台系目标→map→standoff→夹区 | `D/src/target_input.cpp:174-239`；`D/src/center_zone.cpp:247-268` | 目标、gimbal TF、`CenterZone` |
| `PickPatrolGroup` | 按游标取一组点（蛇形网格） | `D/src/center_patrol.cpp:64-79`；网格 `:10-62` | 点池、游标、`path_len` |
| `MarkTargetReissued` | 记录本次下发点/时刻 | `D/src/target_input.cpp:167-172` | `TargetInput` |
| `DispatchGoalPoseImmediate` | Home/WaitHome 首点直发 `/goal_pose` | `D/src/waypoint_executor_client.cpp:246-249` | goal publisher、首点 |
| `ResetCenterBookkeeping` | 离开 CENTER 清走位账 | `D/src/decision_node.cpp:226-229` | 走位状态 |
| `LoadWaypoints`/`BuildPath` | CSV→`nav_msgs/Path` | `D/src/waypoint_store.cpp:98-152`；`D/src/waypoint_executor_client.cpp:326-340` | CSV 文件（启动期） |

### 5.3 异步动作（`StatefulActionNode`，生命周期是重构重点）

| 建议叶子名 | 语义 | 现有实现入口 | 生命周期/依赖/风险 |
|---|---|---|---|
| `FollowWaypointsAction` | 下发一条航点路径并等终态；`stop_at_final` 区分"到点停/穿过不停" | 客户端 `D/src/waypoint_executor_client.cpp:192-324`；executor `W/src/waypoint_executor.cpp:563-624` | onStart 发 goal；onRunning 等 result/超时；onHalted 应发 cancel（现在没有）。抢占由 executor 处理（新 goal join 旧 worker，`:523-539`），旧 goal 以 CANCELED 结束（`:578-585`）；注意客户端"只认最新 handle"（`D/src/waypoint_executor_client.cpp:289-292` 的 handle 过滤）会导致被抢占旧目标的终态**不进业务逻辑** |
| `MaintainGoalAction` | 守点型目标：偏离超容差并持续 `drift_hold_sec` 后重发 | `D/src/waypoint_executor_client.cpp:90-121`；白名单 `:375-384` | 依赖 robot/goal 位姿、`maintain_goal.*`；与 FollowWaypointsAction 共用同一 action client |
| `PatrolGroupAction` | 走位一组点 → 完成/失败 → 停 `group_gap_sec` → 下一组 | 节点层 `D/src/decision_node.cpp:253-309` + 结果回调 `:83-109` + 三个 bool/时刻（`:912-916`） | **当前状态散在节点成员里**；被 Chase 抢占时旧 Patrol 结果可能被丢弃 → `patrol_path_in_flight_` 永久为真（§7.1 B-2）；BT 里必须由动作自身 onHalted/终态清账 |
| `ChaseAction` | 追击：每拍算 goal，按 `min_goal_move_m`/`shouldReissue` 重发或保持 | `D/src/decision_node.cpp:683-756,768-795` | 不是"一条路径走完"语义，而是持续行为；BT 用 StatefulActionNode：onStart 首次下发，onRunning 反复评估/重发，onHalted 交还主决策；目标消失/下发失败必须返回 FAILURE 让 Fallback 落到走位（`:735-741`） |
| `WaitAtHomeAction` / `WaitHpAction` | 到达等待点后持续守点（MaintainGoal 的目标实例） | `sm.cpp` 状态 + `targetNeedsMaintainGoal` 白名单 | 与 MaintainGoalAction 同构，可合并为"守点动作 + 目标名端口" |

### 5.4 与 task-13 XML 的映射（建议）

`BT/` 的第一版树是 `ReactiveFallback`：①游戏未进行→等待/回家 ②LOW HP→回血 ③定位 LOST→holding ④有敌人目标→追击
⑤默认→中心巡逻。按本审计，每个分支可直接落到叶子：

| 树分支 | 条件叶子 | 动作叶子 |
|---|---|---|
| 未进行 | `GameActive`（取反，且加新鲜度） | `FollowWaypointsAction(WaitHome/Home)` |
| 低血 | `HpKnown`+`HpLowHysteresis` | `FollowWaypointsAction(Home/WaitHp)` + `MaintainGoalAction` |
| LOST | `LocalizationOk`（取反） | 无动作（holding），状态机不推进目标下发 |
| 追击 | `InCenterPhase`+`TargetUsable`+`ZoneUsable` | `ChaseAction` |
| 巡逻 | `InCenterPhase`+`PatrolPoolAvailable` | `PatrolGroupAction`；`center_patrol.enable=false` 时 `FollowWaypointsAction(WaitCenter)`+`MaintainGoalAction` |

## 6 测试覆盖与缺口

### 6.1 现有覆盖（decision 9 个 gtest 共 90 个场景；serial 2 个 gtest 27 个场景；executor 6 个 pytest；decision 1 个 launch 测试）

| 测试文件 | 场景数 | 覆盖主题 |
|---|---|---|
| `D/test/test_decision_state_machine.cpp` | 13 | 起点、开局需血量、低血迟滞、游戏结束抢占/恢复、到中心、CENTER 无迁移、陈旧事件、低血抢占 CENTER、只认 Center 事件 |
| `D/test/test_target_input.cpp` | 22 | 解析/全零/非法、编号映射、防抖、短暂丢帧、超时重获、换目标重置、EMA、太近拒绝、重发判据、standoff、云台系→map（含非归一化四元数/退化） |
| `D/test/test_waypoint_executor_client.cpp` | 17 | 重试防抖（含未被接受也记账）、Home/WaitHome 立即抢占、结果应用/忽略错目标、守点漂移重发、只守等待目标、面板话题/直发、被抢占结果不覆盖新目标、结果回调、超时守卫（开/关） |
| `D/test/test_center_patrol.cpp` | 12 | 网格在安全区内、蛇形相邻、格子中心、密度、退化、小组、游标绕回、不重复、全覆盖 |
| `D/test/test_center_zone.cpp` | 11 | 任意角点顺序、退化拒绝、margin、contains/clamp、zoneContainsAll |
| `D/test/test_waypoint_store.cpp` | 6 | CSV 读取/缓存/相对路径/缺失/锚点/前方选点 |
| `D/test/test_decision_config.cpp` | 3 | 默认值、clamp/阈值交换、名字转换 |
| `D/test/test_decision_context.cpp` | 3 | 空/非 LOST、从没收到、LOST 冻结/OK 清除 |
| `D/test/test_decision_state.cpp` | 3 | 5 叶子字符串、目标映射、latched 消息 |
| `D/test/test_decision_launch.py` | 1 | 单独 launch 只起决策链、执行器默认不启动、航点路径指向 `sentry_waypoints` |
| `S/test/test_referee_packet.cpp` | 16 | 帧布局/长度/CRC/扫描/拆位/血量差分/转换字段 |
| `S/test/test_gimbal_posture_packet.cpp` | 11 | 姿态帧布局/极性/CRC |
| `W/test/test_waypoint_executor_follow.py` | 6 | follow 语义：首发/邻近切换/终点容差/超时熔断；`stop_at_final` 两种语义 |

### 6.2 缺口（按 BT 重构优先级）

- **T-1 没有节点级测试**：`D/src/decision_node.cpp` 927 行全部编排（订阅/TF/CENTER/下发）无任何测试入口；
  `bt_action_replacement_node` 只在 launch test 里被检查结构。BT 化时要把编排拆成可注入黑板的叶子，并用 `test_bt_tree` 覆盖。
- **T-2 没有 decision→executor 的真实 action 集成测试**：客户端测试用**假 action server**（`D/test/test_waypoint_executor_client.cpp:50-70`），
  executor 侧用 pytest 模拟导航状态，两侧契约（抢占/CANCELED/`stop_at_final`/feedback）没有联合用例。
- **T-3 没有 preempt 生命周期用例**：追击抢占走位后旧 Patrol 结果的处理、`patrol_path_in_flight_` 是否清账，均未测（§7.1 B-2 的竞态）。
- **T-4 没有 QoS/串口端到端测试**：I-1 这类"静默 0 帧"没有任何自动测试能发现。
- **T-5 没有配置漂移测试**：`D/config/bt_action_replacement.yaml` 与 `src/bringup/config/common/decision.yaml` 是两份真源，
  bringup 版还残留已删除的 `combat:` 键（`src/bringup/config/common/decision.yaml:49-55`；代码里无对应声明，`grep combat D/src/decision_config.cpp` 零命中），
  且缺 `center_zone`/`target_chase`/`center_patrol` 三段（对比 `D/config/bt_action_replacement.yaml:67-114`）。
- **T-6 没有阶段/新鲜度/`game_type` 用例**：FSM 测试只覆盖 `game_active` 布尔，没有 0..5 阶段语义、断链超时。
- **T-7 没有 through 通路用例**：因为它是死的（§4）。
- **T-8 没有长跑/时序回归**：走位"组→停顿→组"、追击进出的 2 s 下界、目标时有时无的切换，只有单点单元测试，没有仿真长跑。

## 7 当前实现的「鸡肋点」清单

### 7.1 BT 重构必须解决（否则 BT 只是换壳）

**B-1 输入契约不可靠/不完整**（对应 §3.3）。QoS 不兼容（I-1）、无新鲜度（I-2）、`game_type` 未校验（I-3）、
`is_attacked` 死字段（I-4）、关键字段未转发（§3.2）。BT 的黑板必须先有"可信的输入"，否则所有条件节点都在读脏数据。
**动作**：先修 QoS + 加 `input_timeout` + `game_type` 校验；把"未接入字段"在黑板里显式标为 unavailable（而不是默认 false）。

**B-2 动作生命周期与 FSM 事件耦合（含一个记账竞态）**。现状：
① 完成事件是单槽 `optional`，每拍 `std::exchange` 取一次（`D/src/decision_node.cpp:194`），同拍两个终态会丢一个；
② 走位在飞/完成用节点成员布尔记账（`:912-916`），完成回调在**节点层**改走位账（`:83-109`）；
③ 客户端只把"仍匹配 running target"的结果喂给业务回调（`D/src/waypoint_executor_client.cpp:303-311`），
而 executor 抢占旧目标时用 CANCELED 结束、旧 goal 的 handle 已被新 goal 替换（handle 过滤 `:289-292`）；
④ 走位检查顺序是"在飞且未完成→return"在"距下次时刻"之前（`D/src/decision_node.cpp:277-283`），
所以一旦 `patrol_path_in_flight_` 卡 true，注释里承诺的"兜底时刻"（`:290-296`）永远不会生效。
**竞态**：Patrol 在飞 → Chase 抢占 → 若 Chase 的 goal_response 先于 Patrol 的 CANCELED result 到达客户端，
Patrol 终态被客户端忽略 → `patrol_path_done_` 永远不置位 → 追击结束后走位**永久停住**。
（触发条件取决于两条 DDS 流先后；不能靠"实测没遇到"来保证。）
**动作**：BT 里由异步动作自身持生命周期——`onRunning` 只认自己的 goal handle/取消信号，`onHalted` 必须清账；
执行器抢占语义改为显式 `cancel`（或在 result 里带 `superseded` 标记）；节点层不再维护"在飞/完成"布尔。
这个改动也会同时消掉 `checkGoalTimeout` 合成 Aborted（`D/src/waypoint_executor_client.cpp:122-160`）与真实失败混同的问题。

**B-3 决策维度缺失（BT 的"条件"无处可取）**。现有树只能表达"比赛是否进行 / 低血 / 定位 / 有目标 / 在中心"五件事；
规则要求的**阶段、控制区占领、控制度、虚弱/复活、热量/弹药、姿态**都没有数据或没有条件（§3.2、`R:11-29`、`R:224-258`）。
CENTER 段还被刻意压成"走位/追击二选一"，连"激进/保守"都删了（`D/README.md:120-132`；`D/doc/sentry_tactics_rmul.md:217-244`
保留了 aggression 设计但实现未落地）。**BT 重构前必须先写死"本迭代的 context 定义"**：
哪些字段进黑板、缺数据时条件默认值是什么、phase 枚举怎么定义。否则 XML 里只能把现有 if-else 原样重排。

**B-4 配置/文档双真源与死参数**。三处配置（代码默认 `D/include/decision/decision_config.hpp:141-162`、包内 yaml
`D/config/bt_action_replacement.yaml`、bringup yaml `src/bringup/config/common/decision.yaml`），差异与死项：
`waypoint.final_goal_tolerance`（只声明/校验，无行为消费：`grep` 仅 config）、`waypoint_executor.start_delay_sec`（同）、
`through_action`（无 server）、`setActiveTarget`（无调用者：`D/src/waypoint_executor_client.cpp:36-39`）、
`prepareFollowWaypoints` 的唯一节点调用点传 `std::nullopt`（`D/src/decision_node.cpp:830-838`）→ 选"前方航点"逻辑实际未启用；
bringup yaml 的 `combat:` 残段与缺失段（T-5）。BT 需要一个配置结构体 + 每参数单一来源，端口只暴露必要的可调项。

### 7.2 可后置（不影响 BT 骨架正确性）

| # | 鸡肋点 | 证据 | 处置 |
|---|---|---|---|
| C-1 | through action 死通路 | `D/src/decision_node.cpp:78-79`；`W/src/waypoint_executor.cpp:77,114-166`（只一个 action） | 删除或明确标 disabled，别在树里出现 |
| C-2 | `setActiveTarget` 无调用者 | `D/src/waypoint_executor_client.cpp:36-39`；声明 `D/include/decision/waypoint_executor_client.hpp:79` | 删 |
| C-3 | 选前方航点逻辑不会触发 | `D/src/decision_node.cpp:830-838` 传 nullopt；`D/src/waypoint_store.cpp:182-206` | 保留但注明，或接 robot pose |
| C-4 | `zoneContainsAll` 只在测试用 | `D/src/center_zone.cpp:270-290`；节点用自己的一段循环 `D/src/decision_node.cpp:654-681` | 合并去重 |
| C-5 | feedback 未消费 | `I/custom_msgs/action/FollowWaypoints.action:19-20`；executor 发 `W/src/waypoint_executor.cpp:598-601`；决策无引用 | BT 动作可用它做进度/超时，但非必须 |
| C-6 | RViz 面板 service 名与 bringup 不一致 | 面板 `W/src/waypoint_editor_panel.cpp:158-159`（`start_waypoint_following`/`start_waypoint_through`）vs bringup `src/bringup/config/common/waypoint_executor.yaml:27-28`（`/waypoint_editor/start_follow`、`/waypoint_editor/follow_waypoints`） | 面板调试路径，后置 |
| C-7 | 文档漂移 | `D/README.md:423-428,462-467` 仍写订阅 follow_status/through_status，实际已改 action result（`D/src/decision_node.cpp:83-109`）；`D/src/decision_node.cpp:206` 注释仍提已删除的 ENGAGE | 重构时同步 |
| C-8 | 查重名/查双执行器塞在 tick 路径 | `D/src/decision_node.cpp:181-186,469-525` | 移到诊断节点/启动自检 |
| C-9 | 走位网格不做障碍检查 | `D/src/center_patrol.cpp:10-62` 只做 zoneContains（`:56`），未查代价地图 | 中心区当前实测空旷；换图后需复核 |
| C-10 | 追击只在 CENTER 生效（回家/回血途中目标被忽略） | 调用点仅 `D/src/decision_node.cpp:265` | 设计取舍，记录即可 |
| C-11 | `pending_executor_event_` 单槽丢事件 | `D/src/decision_node.cpp:194,872` | 与 B-2 一起改 |
| C-12 | 决策状态只在迁移时发布，CENTER 内部行为不可见 | `D/src/decision_node.cpp:197-203` | 可加 phase/行为字段，非 BT 必需 |

## 8 建议 BT 黑板 key 列表

> 原则（与 `BT/` 的约束一致）：**黑板只放简单 POD / optional struct，不放 ROS msg 指针**；配置放一份 const 结构；
> 叶子通过 ports 拿参数，跨叶共享状态才上黑板。下表"对应现有数据"给到具体字段/函数。

| 分组 | key | 类型 | 来源/更新时机 | 对应现有数据 |
|---|---|---|---|---|
| 比赛 | `game_active` | `bool` | `game_status` 回调后每拍重算 | `D/src/decision_node.cpp:170-178` |
| 比赛 | `game_type` | `uint8` | 同上 | 未消费，`GameStatus.msg:9` |
| 比赛 | `game_progress` | `uint8` | 同上 | 同上 |
| 比赛 | `stage_remain_time` | `int` | 同上 | `D/src/decision_node.cpp:173-178` |
| 比赛 | `game_data_fresh` | `bool` | 新增（stamp + timeout） | 新增（I-2） |
| 比赛 | `match_phase` | `enum class Phase : uint8` | 由 game_type/progress/time 推导 | 新增（B-3；0..5 语义见 `GameStatus.msg:10`） |
| 自身 | `current_hp` | `optional<int>` | `robot_status` 回调 | `D/src/decision_context.cpp:26-33` |
| 自身 | `hp_recovery_active` | `bool` | 时间步更新（迟滞 latch） | `D/src/decision_state_machine.cpp:37-43` |
| 自身 | `is_attacked` | `bool` | `robot_status` 回调 | 已有未用（I-4；`S/src/serial_driver_node.cpp:309-311`） |
| 自身 | `hp_fresh` | `bool` | 新增 | 新增（I-2） |
| 自身 | `max_hp` | `int` | 常量（哨兵 400） | 本地常量（`D/doc/sentry_tactics_rmul.md:364-373`） |
| 定位 | `localization_lost` | `bool` | `/localization_status` 回调 | `D/src/decision_context.cpp:67-71` |
| 定位 | `localization_payload` | `string` | 同上（debug 用） | `D/src/decision_context.cpp:73-76` |
| 目标 | `enemy_target` | `optional<EnemyTarget{x,y,published_id,range_m,usable}>` | 自瞄回调 + 每拍 refresh | `D/src/target_input.cpp:107-157`；结构 `D/include/decision/target_input.hpp:73-80` |
| 目标 | `enemy_target_map` | `optional<Pose>` | Chase 动作内算 | `D/src/target_input.cpp:197-239` |
| 目标 | `chase_goal` | `optional<Pose>` | Chase 动作内算（standoff+夹区） | `D/src/decision_node.cpp:709-719` |
| 目标 | `chase_reason` | `string` | Chase 动作每拍照写（debug） | `D/src/decision_node.cpp:768-830` |
| 执行器 | `executor_event` | `optional<ExecutorEvent{TargetName,type}>` | action result / 超时守卫（建议改队列） | `D/src/decision_node.cpp:194`；`D/include/decision/decision_state_machine.hpp:12-23` |
| 执行器 | `executor_running_target` | `optional<TargetName>` | 客户端状态快照 | `D/include/decision/waypoint_executor_client.hpp:41-58` |
| 执行器 | `executor_ready` | `bool` | `action_server_is_ready` | `D/src/waypoint_executor_client.cpp:208-211` |
| 执行器 | `executor_goal_in_flight` | `bool` | 客户端状态 | `D/src/waypoint_executor_client.cpp:321-322` |
| 区域 | `zone` | `shared_ptr<const CenterZone>`（或值） | 启动时构建一次 | `D/src/decision_node.cpp:605-681`；结构 `D/include/decision/center_zone.hpp:23-30` |
| 区域 | `zone_usable` | `bool` | 同上 | `D/include/decision/center_zone.hpp:29` |
| 走位 | `patrol_pool` | `shared_ptr<const vector<Pose>>` | 启动时构建（网格或 CSV） | `D/src/decision_node.cpp:533-571` |
| 走位 | `patrol_cursor` | `size_t` | 每组完成后推进 | `D/src/decision_node.cpp:285-303` |
| 走位 | `patrol_group_state` | `enum{Idle,InFlight,DoneWaitingGap}` | 由 PatrolGroupAction 私有持有 | 替代 `D/src/decision_node.cpp:912-916` 三布尔 |
| 走位 | `next_patrol_sec` | `double` | 组完成后置 | `D/src/decision_node.cpp:295` |
| 位姿 | `robot_pose_map` | `optional<Pose>` | 每拍/按需 TF | `D/src/decision_node.cpp:405-412` |
| 位姿 | `gimbal_pose_map` | `optional<Pose>` | 追击需要时 TF | `D/src/decision_node.cpp:455-462` |
| 输出/调试 | `decision_state` | `string` | 状态迁移时更新 | `D/src/decision_state.cpp:8-20` |
| 输出/调试 | `active_target` | `TargetName` | FSM 输出 | `D/src/decision_state.cpp:22-36` |
| 配置 | `config` | `shared_ptr<const DecisionConfig>` | 启动加载一次 | `D/include/decision/decision_config.hpp:141-162` |
| 未来（先占位） | `occupation_state` | `enum{Unknown,None,Ours,Enemy,Both}` | 需协议 0x0101（未转发） | 新增（B-3；`R:224-258`） |
| 未来（先占位） | `center_rfid` | `bool` | 需协议 0x0209（未转发） | 新增 |
| 未来（先占位） | `weak_state`/`respawn_remaining` | `bool`/`double` | 规则推算或 0x020D（待确认） | 新增（`R:62-69`） |
| 未来（先占位） | `aggression` | `double` | 由时间缺口/血量/态势推导 | `D/doc/sentry_tactics_rmul.md:217-244`（设计已写、实现未落地） |

## 9 证据索引（入口）

- FSM：`D/include/decision/decision_state.hpp:14-32`、`D/src/decision_state_machine.cpp:34-103`、`D/src/decision_state.cpp:8-48`。
- 编排：`D/src/decision_node.cpp:55-161`（装配/订阅）、`:179-233`（tick）、`:253-309`（CENTER）、`:363-403`（下发）、`:683-830`（追击/调试）。
- 客户端：`D/include/decision/waypoint_executor_client.hpp:23-147`、`D/src/waypoint_executor_client.cpp:58-160,163-322,375-384`。
- 输入：`S/include/packet.hpp:24-51,117-186`、`S/include/referee_conversion.hpp:22-60`、`S/src/serial_driver_node.cpp:89-96,299-326`、
  `S/doc/referee_frame.md:12-69`、`I/custom_msgs/msg/GameStatus.msg:1-11`、`I/custom_msgs/msg/RobotStatus.msg:1-9`。
- 执行器：`I/custom_msgs/action/FollowWaypoints.action:1-20`、`W/src/waypoint_executor.cpp:3-10,77-78,114-166,523-539,563-624`、
  `src/bringup/config/common/waypoint_executor.yaml:21-30`。
- 配置：`D/config/bt_action_replacement.yaml:1-114`、`src/bringup/config/common/decision.yaml:1-55`、`D/include/decision/decision_config.hpp:141-166`。
- 测试：`D/test/`（9 个 gtest + 1 个 launch test）、`S/test/`（2 个 gtest）、`W/test/test_waypoint_executor_follow.py`；
  缺口清单见 §6.2。
