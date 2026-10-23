# 审计 C：fake_vel_transform 旋转 / 小陀螺 / TF 正确性（2026-10-23）

> 范围：`src/control/fake_vel_transform`（虚拟云台速度转换节点）、两份 navigation2
> params（`src/bringup/config/common/navigation2.yaml`、`src/navigation/navigation2/params/navigation2.yaml`）。
> 方法：只读代码 + 推导 + 抽纯函数补 gtest + headless（无 Gazebo）实测；
> 未改 MPC / 导航逻辑，未改 PCD/地图，未 git commit。
> 结论分级：**已修**（本轮已改代码/配置）、**已显式化**（行为不变，把隐藏语义写成参数+注释）、
> **仅报告**（风险存在但本轮不改）。

---

> **决策更新（2026-10-23，用户）**：本工程模型明确为"可平移的运动云台"，**不需要小陀螺**。
> 因此本文 §0 第 2/3/6 行提出的 `spin_enable/spin_min_speed` 方案已作废：相关自旋代码、
> 参数与 yaml 配置已全部删除，节点只保留 `R(offset)` 线速度旋转 + 角速度透传。
> 下述审计内容保留作为"为什么不应引入自旋"的推导记录。

## 0. 结论速览

| # | 发现 | 级别 | 处置 | 证据 |
| :- | :- | :- | :- | :- |
| 1 | **旋转数学是正确的**：`angle_diff = -offset` 再按 `R(-angle_diff)` 旋转，等价于 `R(φ-β)`；与 MPC 的 `R(φ)ᵀ` 复合后恰好得到底盘系所需的 `R(-β)·v_world`。写反/顺序颠倒会引入 2(φ-β) 的额外旋转 | —（确认） | 抽纯函数 + 用组合恒等式钉死 | `velocity_transform.hpp:80-92`、`test/velocity_transform_test.cpp` 的 `RotationCompositionMatchesWorldToBase` |
| 2 | **小陀螺三级语义冲突，且部署配置下从未生效**：MPC `default_wz=0.0` → 旧触发条件 `|angular.z| >= angular_deadband` 恒假；平滑器把 angular 限到 ±3；fake 节点在**平滑器之后**注入 `-6`，绕过限幅 | **高** | 新增显式 `spin_enable`（默认 false，保持现状）+ `spin_min_speed`；旧参数降级为弃用别名；两份 yaml 显式化 | 旧逻辑 `git show HEAD:src/control/fake_vel_transform/src/fake_vel_transform.cpp:205-208`；`mpc_controller_node.cpp:941,963,978`；`velocity_smoother_node.cpp:105-114`；`navigation2.yaml:391/394` |
| 3 | 旧逻辑**完全丢弃导航角速度**（非小陀螺时 `angular.z := 0`），而不是按坐标变换透传。绕 z 角速度在平面旋转下不变，正确做法是透传；部署配置 `default_wz=0.0` 时两者等价，因此可以低风险修正 | 中 | 已修为透传（`spin` 生效时才覆盖） | 旧 `…cpp:208`；新 `velocity_transform.hpp:113-126`、`fake_vel_transform.cpp:266` |
| 4 | TF 短失败的处理是**非对称**的：`publishTransform()` 保持上一拍偏置（对），`cmdVelCallback()` 直接发零速（fail-stop，安全但可能一帧急停） | 中 | 前者抽成 `offsetAfterMeasurement` 并补单测；后者保持（安全侧）并写注释 | `fake_vel_transform.cpp:288-307`、`velocity_transform.hpp:131-137` |
| 5 | 时间/TF 残余风险：`MonotonicStampGate` 在 ROS 时钟回拨（仿真 reset / NTP 步进）后会停发 TF 直到时钟追平；`TimePointZero` 查询不判 stale；100 Hz 广播在仿真暂停时重复 stamp 被丢弃（恢复后自动继续） | 中 | 仅报告（本轮不改时间语义） | `monotonic_stamp_gate.hpp:15-22`、`fake_vel_transform.cpp:282-283,296,138,232` |
| 6 | 参数原本无任何 yaml 覆盖：`spin_speed=-6` 是隐藏默认值；组件跟随 navigation2 params_file 启动，但两份 yaml 都没有 `fake_vel_transform:` 段 | 中 | 已显式化（两份 yaml 同步 + 注释 + README） | `bringup.launch.py:33,109-120`；两份 yaml `fake_vel_transform:` 段 |
| 7 | 下游 `serial_driver` 是事件驱动发送底盘帧，本身没有速度超时看门狗；若 fake 节点/上游整段静默，MCU 可能保持最后一帧（含 `-6` 自旋） | 中 | 仅报告，建议电控侧整机看门狗 | `src/serial/include/packet.hpp:75-77`、`serial_driver_node.cpp:333-347` |
| 8 | 小陀螺“谁负责”的文档语义冲突：`navigation2/README.md:6` 写“电控负责小陀螺”，本节点却在速度层注入角速度；`mpc_controller_node.cpp:977` 注释又称 `default_wz` 驱动小陀螺 | 低 | 仅报告，建议统一口径（本节点注入 = 导航侧请求，电控执行） | 见上 |

---

## 1. 数据流与坐标系

```
/map, /plan_raw ──> rm_global_planner ──> /plan_raw
/plan_raw ──> rm_minco_path_smoother ──> /plan
/plan + /Odometry + TF ──> rm_mpc_controller（robot_base_frame=base_link_fake）
        └─> /cmd_vel_nav（fake 系速度）
/cmd_vel_nav ──> rm_velocity_smoother（分量限幅/死区/超时归零）──> /cmd_vel
/cmd_vel ──> fake_vel_transform ──> /cmd_vel_chassis ──> serial_driver（实车）
                                                        └─> gazebo planar_move（仿真）
fake_vel_transform ──> TF base_link -> base_link_fake（100 Hz）
```

- MPC 的 `robot_base_frame` 是 `base_link_fake`（`mpc_controller_node.cpp:138`），
  位姿优先取 `map -> base_link_fake` TF（`mpc_controller_node.cpp:526-529`），
  所以 MPC 眼里的 yaw 是**虚拟云台 yaw φ**，不是底盘 yaw β。
- `fake_vel_transform` 用 `/plan` 的 XY 切向估计 map 系路径航向 φ（`fake_vel_transform.cpp:200-214`），
  再用 `map -> base_link` TF 得到 β，广播 `base_link -> base_link_fake` 的旋转
  `current_angle_ = φ - β`（`fake_vel_transform.cpp:306,321-323`）。
- 因此 `base_link_fake` 在 map 里的 yaw 恰为 φ；TF 树：`map -> odom -> base_link -> base_link_fake`。
- 速度话题全部是「指令」：`/cmd_vel_nav`、`/cmd_vel`、`/cmd_vel_chassis` 都只在收到上游
  消息时才发；`/cmd_vel_chassis` 的 QoS 是 KeepLast(1) reliable（`fake_vel_transform.cpp:110-111`），
  串口节点用 SensorDataQoS 订阅（best-effort 兼容，`serial_driver_node.cpp:79-81`）。

---

## 2. 旋转数学：推导、旧实现、验证

### 2.1 推导

记 `β = yaw(map→base_link)`，`φ = yaw(map→base_link_fake)`，`offset = φ - β`。

1. MPC 在 fake 系里跟踪路径，读到的是 φ；它把世界系期望速度 `v_world` 转进 fake 系：
   `v_F = R(φ)ᵀ·v_world`（`mpc_controller_node.cpp:934-942`，`yaw` 来自 `map→base_link_fake`）。
2. 速度平滑器只做逐分量限幅/死区，不换系（`velocity_smoother_node.cpp:105-114`）。
3. 底盘（base_link 系）真正要执行的速度是 `v_B = R(β)ᵀ·v_world`。
4. 于是节点应做的变换为：

```
v_B = R(β)ᵀ · R(φ) · v_F = R(φ - β) · v_F = R(offset) · v_F
```

即：**用 offset = φ - β 作为旋转角，直接以 R(+offset) 旋转 fake 系线速度**。

### 2.2 旧实现的符号/顺序

旧代码（`git show HEAD:src/control/fake_vel_transform/src/fake_vel_transform.cpp:194-210`）：

```cpp
const double angle_diff = -current_angle_;              // = -(φ-β) = β-φ
out.x =  in.x*cos(angle_diff) + in.y*sin(angle_diff);   // = R(-angle_diff)·v_F
out.y = -in.x*sin(angle_diff) + in.y*cos(angle_diff);
```

`R(-angle_diff) = R(φ-β) = R(offset)`，与推导一致 —— **符号和顺序都是对的**，
不是 bug。风险在于写法绕了一层（`-current` 再 `R(-·)`），容易被后续维护者“顺手修正”
成错的方向。本轮把它改成显式 `R(offset)`（`velocity_transform.hpp:80-92`），并用
**组合恒等式**钉死：`RotationCompositionMatchesWorldToBase` 取 β=0.7、φ=-0.4、
任意 `v_world`，先模拟 MPC 的 `R(φ)ᵀ`，再走节点变换，断言结果等于 `R(-β)·v_world`。
任何符号/顺序错误（例如少一个负号、转置写反）都会让该用例失败。

### 2.3 独立实测（headless，无 Gazebo）

静态 TF `map→base_link` yaw β=0.5、`/plan` 沿 +X（φ=0，offset=-0.5）、发布
`/cmd_vel=(1.0, 0.0, 0.5)`：

```
out = (0.8776, -0.4794, 0.5)    # R(-0.5)·(1,0)，与解析值一致
fake TF yaw = -0.5000           # base_link->base_link_fake 偏置 = φ-β
```

符号写反（用 R(+0.5) 代替 R(-0.5)）会得到 (0.8776, +0.4794)；实测与推导一致
（详见 §7 验证记录）。

### 2.4 角速度

绕 z 的角速度在平面刚体旋转下不变（`ω_B = ω_F`）。旧实现无论什么情况都把
`angular.z` 置 0 或 `spin_speed`，**丢掉了导航角速度指令**。部署配置里 MPC
`default_wz=0.0`（两份 yaml 的 `rm_mpc_controller.default_wz`，见
`src/bringup/config/common/navigation2.yaml:391`、`src/navigation/navigation2/params/navigation2.yaml:394`），
所以「丢弃」与「透传」在部署参数下输出都是 0，行为等价；本轮改成正确的透传
（`velocity_transform.hpp:124`），只在 `spin` 生效时覆盖为 `spin_speed`。

---

## 3. 小陀螺三级语义冲突（本轮核心修复）

### 3.1 三层各自在说什么

| 层 | 位置 | 实际语义 |
| :- | :- | :- |
| L1 MPC 下发 | `mpc_controller_node.cpp:941,963,978` | 正常跟踪/停车/恢复都 `cmd.angular.z = default_wz_`；部署值 `0.0`（两份 yaml）。注释说“哨兵的小陀螺由这个参数驱动”，但值就是 0 |
| L2 速度平滑器 | `velocity_smoother_node.cpp:24-32,105-114`；yaml `max_velocity: [2.0, 2.0, 3.0]` | angular 被限到 ±3 rad/s，死区默认 0；`/cmd_vel` 是“最后一道限幅” |
| L3 fake 节点（旧） | 旧 `fake_vel_transform.cpp:205-208` | `should_spin = |linear| >= min_translate_speed_for_spin && |angular.z| >= angular_deadband`；成立则输出 `spin_speed=-6`，在 L2 **之后**注入，绕过 ±3 限幅；不成立则输出 0 |

### 3.2 冲突与后果

1. **功能从未生效**：部署 `default_wz=0.0` → L1 的 angular 恒 0 → L3 第二个条件
   `|0| >= 0.05` 恒假 → 小陀螺永远不触发。参数在 yaml 里又完全缺失（无
   `fake_vel_transform:` 段），`spin_speed=-6` 只是代码默认值，没人知道它开没开。
2. **一个参数身兼两职**：`angular_deadband` 名义上是“死区”，实际是小陀螺使能阈值；
   `default_wz` 名义上是“无参考航向时的默认角速度”，实际是小陀螺的“请求信号”。
   只要有人为了转向调大 `default_wz`，L3 就会把**导航角速度整段替换成固定 -6**，
   行为从“转 1 rad/s”突变成“6 rad/s 反向自旋”，且没有任何参数能关掉。
3. **绕过限幅是隐藏契约**：`spin_speed=-6 rad/s` 是 L2 上限的 2 倍。对底盘执行层这是
   有意为之（小陀螺是底盘行为，不该受导航跟踪限幅约束），但它只写在代码里，
   `rm_velocity_smoother` 的注释还自称“最后一道限幅”，两边语义互相矛盾。
4. **使能不可独立操作**：要开小陀螺必须同时改 MPC 的 `default_wz`（导航逻辑参数），
   违反“控制链参数各管各的”的直觉，也让 A/B 测试无法只动一个变量。
5. **RECOVERY 期继续请求自旋**：`publishWorldVelocity()` 保留 `default_wz_`
   （`mpc_controller_node.cpp:975-978`），而接近段 `applyApproach` 在 0.5 m 内把
   angular 清 0（`mpc_controller_node.cpp:1009-1012`）—— 旧触发链在恢复/接近期的
   语义完全不同，没有任何文档。

### 3.3 方案（已实现，默认行为不变）

新增两个显式参数（`fake_vel_transform.cpp:47-55`）：

| 参数 | 默认 | 语义 |
| :- | :- | :- |
| `spin_enable` | `false` | 小陀螺总开关。默认 false = 与“从未触发”的现状完全一致；开启是部署侧显式动作 |
| `spin_min_speed` | `0.15` m/s | 允许注入的**最小平移速度**；停车/低速暂停、回升自动恢复（纯函数无状态，`velocity_transform.hpp:94-103`） |

生效规则（`computeChassisTwist`，`velocity_transform.hpp:113-126`）：

```
angular_out = (spin_enable && |v_linear| >= spin_min_speed) ? spin_speed
                                                            : cmd.angular_z(透传)
```

- 不再依赖 MPC 的 `default_wz`，**不需要改 MPC 就能开/关小陀螺**；
- 零速指令永远零输出（停车不自旋）；
- 开启方式：两份 yaml 的 `fake_vel_transform.spin_enable: true` + 实车核对
  `spin_speed` 正负号（默认 -6.0，方向取决于电控/轮组约定，见 `serial_driver`
  `chassis_vel_w_scale=1.0`，`src/bringup/config/reality/serial_driver.yaml:10`）。
- 旧参数保留声明但弃用（哨兵值 -1 区分“显式设置”）：`angular_deadband` 显式设置时
  WARN 并说明不再作为触发条件；`min_translate_speed_for_spin` 显式设置时作为
  `spin_min_speed` 别名生效（`fake_vel_transform.cpp:66-81`）。
- `spin_speed` 非有限/`spin_min_speed` 非法时按“不安全配置”降级为不注入/回默认
  （`fake_vel_transform.cpp:83-96`）。

**回退**：`spin_enable: false` 即恢复旧部署行为（小陀螺不注入）；无需改 MPC。

---

## 4. 时间 / TF 审计

### 4.1 MonotonicStampGate（`monotonic_stamp_gate.hpp:15-22`）

- 作用：保证 `base_link→base_link_fake` 的 TF stamp 严格递增，避免 TF2 因重复/回退
  时间戳丢弃或告警。`accept()` 对 `<= last` 直接返回 false。
- 触发条件：100 Hz wall timer（`fake_vel_transform.cpp:116-118`）取 **ROS 时钟**
  `get_clock()->now()`（`fake_vel_transform.cpp:282`）：
  - `use_sim_time=true` 且仿真暂停时，`now()` 不变 → 重复 stamp 被 gate 丢弃，
    TF 停更（暂停期间本该如此，恢复后自动继续）；
  - 仿真 reset / 系统 NTP 步进导致时钟**回拨**时，gate 会一直拒绝，直到时钟追平
    回拨前的时间戳 —— 这段时间 fake TF 冻结，MPC 的 `map→base_link_fake` 查不到新值，
    仅靠 TF 缓存里的旧值（`TimePointZero` 不判 stale）。属**仅报告**风险：可在部署
    时避免仿真 reset；若实车 NTP 会步进，建议改用“回拨超过阈值时重置 gate”。
- 100 Hz 广播本身没问题：TF2 缓存 10 s，MPC 30 Hz 用最新值；fake 偏置也在
  `publishTransform()` 里以 100 Hz 刷新（`fake_vel_transform.cpp:287-307`）。

### 4.2 TimePointZero / stale TF

所有 `lookupTransform(..., tf2::TimePointZero)`（`fake_vel_transform.cpp:138,232,296`）
取“最新可用”，**不检查数据年龄**。定位丢失（`map→odom` 停写）时 fake 节点仍会用
缓存里的旧 β 算偏置；但 MPC 有 integrity gate：`LOCALIZATION_STATUS=LOST` 时
`publishStop()` 且不进恢复链（`mpc_controller_node.cpp:665-675`），速度链路本身已停，
fake 的旧偏置不再影响执行。结论：**仅报告**，不需要 fake 节点重复实现定位健康检查。

### 4.3 target_frame 缺省

`target_frame_` 从 `/plan.header.frame_id` 取，空则取首 pose 的 frame_id，再空则
`"map"`（`fake_vel_transform.cpp:128-133`）；`/cmd_vel` 回调在还没收到 `/plan` 时
也回退 `"map"`（`fake_vel_transform.cpp:230`）。若规划器发的是 `odom` 系路径，
fake 会查 `odom→base_link` 且 `/plan` 也在 odom 下，此时 fake 偏置=路径切向-odom
航向，语义仍自洽；但与 MPC 的 `target_frame=map` 不一致（MPC 会用 map→base_link_fake）。
属配置不匹配，**仅报告**：路径 frame 必须与 MPC `target_frame` 一致。

### 4.4 TF 失败的两类处理

| 路径 | 旧行为 | 现行 | 评价 |
| :- | :- | :- | :- |
| `publishTransform()`（100 Hz TF 广播） | 查 `map→base_link` 失败时保持上一拍偏置继续广播（`git show HEAD:…cpp:245-247`） | 抽出 `offsetAfterMeasurement`：无效测量保留旧值，首拍无效则 0（`velocity_transform.hpp:131-137`，`fake_vel_transform.cpp:306`） | 正确：短失败不能把偏置归零，否则 100 Hz 广播表现为航向抖动 |
| `cmdVelCallback()`（30 Hz 速度） | 查 TF 抛异常 → 发布零速并 return | 不变：`frame_tf.valid=false` → 零速（`fake_vel_transform.cpp:258-263`） | 安全侧 fail-stop；代价是单帧 TF 抖动会让底盘收到一帧 0。**保持**，并在注释写清取舍 |

### 4.5 下游超时缺口（仅报告）

`serial_driver` 只在收到 `/cmd_vel_chassis` 时发底盘帧（`packet.hpp:75-77`、
`serial_driver_node.cpp:333-347`），没有自己的速度超时。上游正常时
`rm_velocity_smoother` 在 `/cmd_vel_nav` 超时 0.5 s 后按 30 Hz 发零速
（`velocity_smoother_node.cpp:116-127`），所以「MPC 死」→ 停车。但「fake 节点或
整个容器阻塞/崩溃」→ 没有零速帧 → MCU 保持最后一帧（可能是 -6 自旋）。
建议：电控固件加整机看门狗（例如 100 ms 无帧即停），不依赖 NUC 侧补偿。

---

## 5. 参数加载与显式化

- 部署链路：`bringup` 把 navigation2 的 params_file 传给 `navigation2/bringup.launch.py`
  （`sim.launch.py:79,283`、`real.launch.py:77`），fake 节点作为组件加载时
  `parameters=common_params`（`bringup.launch.py:33,109-120`）。**fake 节点与导航
  节点共用同一个 yaml**，因此配置必须写在 node 名 `fake_vel_transform` 段下，
  而不是 fake 包自己的 launch 里。
- 两份 yaml 现已同步新增 `fake_vel_transform:` 段（`src/bringup/config/common/navigation2.yaml:336`、
  `src/navigation/navigation2/params/navigation2.yaml:339`），显式写出
  `spin_enable/spin_speed/spin_min_speed/path_lookahead_distance/yaw_filter_alpha`，
  并注明 `spin_speed` 绕过平滑器限幅、旧参数已弃用。两份必须一起改（bringup 用前者，
  navigation2 自带 bringup 默认用后者）。
- 加载机制备注（调试时容易踩）：composable 节点的 YAML 参数由 launch_ros 在
  Python 侧解析后经 `LoadNode` 服务传入（`to_parameters_list.py:88-96`），所以
  该 yaml 顶部的 `goal_tolerance_default: &goal_tolerance_default 0.25` 锚点没问题；
  但直接 `ros2 run <node> --ros-args --params-file <该文件>` 会被 rcl 拒绝：
  `Cannot have a value before ros__parameters at line 6`。这是文件格式与加载方式的
  组合限制，不是本轮引入的缺陷，**验证参数必须走 launch 或 `-p` 覆盖**。
- 验证：用 `/tmp` 里改过值的 params 副本跑 `ros2 launch navigation2 bringup.launch.py`
  （只启 fake 组件），节点日志输出 `spin_enable=true spin_speed=-4.250
  spin_min_speed=0.770` —— 证明 yaml 段确实被加载（默认值本身与 yaml 相同，
  不能靠默认值判断）。

---

## 6. 重构与测试

### 6.1 新增纯函数 header

`include/fake_vel_transform/velocity_transform.hpp`（无 ROS 依赖）：

- `Twist2D` / `FrameTransform2D{valid, fake_from_base_yaw}` / `SpinConfig`；
- `rotateFakeToBase()`：`v_B = R(offset)·v_F`；
- `isZeroCommand()` / `isFinite()`；
- `spinActive()`：`enable && finite(spin_speed) && finite(min) && |v| >= min`；
- `computeChassisTwist()`：TF 无效/非有限 → 零速；零速 → 零速；否则旋转 + 角速度
  覆盖/透传；
- `offsetAfterMeasurement()`：TF 失败保持旧偏置。

节点 `cmdVelCallback()` 改为构造 `Twist2D` + 调纯函数（`fake_vel_transform.cpp:217-278`），
`publishTransform()` 复用 `offsetAfterMeasurement`。**没有新增线程、没有改优化/MPC 逻辑、
没有改话题/QoS**。

行为对照（默认部署配置，`default_wz=0.0`、`spin_enable=false`）：

| 输入 | 旧输出 | 新输出 |
| :- | :- | :- |
| 线速度非零、angular=0 | R(offset)·v, angular=0 | 相同 |
| 零速 | 全零 | 全零 |
| TF 失败 | 零速 | 零速 |
| 未收到 /plan（offset=0） | 恒等 | 恒等 |
| angular≠0（非部署配置） | angular 被丢弃=0 | 透传（**唯一有意行为变化**，见 §2.4） |

### 6.2 gtest

`test/velocity_transform_test.cpp`，13 个用例（`colcon test` 中 gtest 2 个 target、
共 14 用例，13 + 既有 1）：

| 用例 | 覆盖点 |
| :- | :- |
| `RotationMatchesRotationMatrix` | 7 个角度对拍 R(θ) 矩阵、角速度不参与旋转 |
| `RotationPreservesNorm` | 旋转不改变线速度模长 |
| `RotationCompositionMatchesWorldToBase` | **端到端代数恒等式**（MPC R(φ)ᵀ + 节点 R(φ-β) = R(-β)） |
| `IdentityWhenFakeFrameNotInitialized` | 初始 base_link_fake 恒等偏置 |
| `ZeroCommandStaysZero` | 零速（即使 spin 使能） |
| `DisabledSpinPassesAngularThrough` | 角速度 passthrough |
| `SpinReplacesAngularWhenMoving` | 小陀螺覆盖角速度 |
| `SpinPausesBelowMinSpeedAndResumes` | 暂停/恢复（速度跌破/回升阈值） |
| `SpinActivatesAtMinSpeedBoundary` | 阈值边界（>=） |
| `InvalidSpinConfigFallsBackToPassthrough` | 非法 spin 配置安全降级 |
| `InvalidTransformZeroesOutput` | TF 失败（无效/非有限 yaw）→ 零速 |
| `NonFiniteCommandZeroesOutput` | 输入 NaN 不泄露给底盘 |
| `OffsetHoldsLastValueOnFailedMeasurement` | TF 短失败保持旧偏置 / 首拍为 0 |

### 6.3 既有 TF 时间门单测

`test/monotonic_stamp_gate_test.cpp` 保留（严格递增/重复/回退）。

---

## 7. 验收与实测记录

```sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fake_vel_transform \
  --cmake-args -DCMAKE_BUILD_TYPE=Release     # ✅ 通过
colcon test --packages-select fake_vel_transform --event-handlers console_direct+  # ✅ 9/9 通过
```

- gtest：`velocity_transform_test` 13/13、`monotonic_stamp_gate_test` 1/1；
- 其余 linter（cppcheck/cpplint/flake8/pep257/lint_cmake/xmllint/clang_format）全通过；
- headless 端到端（rclpy 造假 TF/路径/速度，无 Gazebo）：
  - β=0.5、φ=0、`/cmd_vel=(1,0,0.5)` → `/cmd_vel_chassis=(0.8776,-0.4794,0.5)`，
    `base_link→base_link_fake` TF yaw=-0.5，零速指令输出全零 —— **PASS**；
  - `spin_enable=true` 同输入 → angular=-6.0，其余不变 —— **PASS**；
  - launch 验证 yaml 段生效（见 §5），默认 `spin_enable=false` 与旧部署一致。
- 未跑整栈仿真（无 Gazebo/GPU 环境）；`spin_speed` 的**实际转向**需实车脱轮核对，
  见风险 §8.2。

---

## 8. 风险与回退

1. **小陀螺默认关闭**：`spin_enable: false` 与历史部署等价；开启后若 `spin_speed`
   符号与实车约定相反，会反向自旋。首次开启时**架空空转**核对 `chassis_vel_w_scale`
   与转向，再落地。
2. **spin 绕过平滑器 ±3**：这是有意设计（底盘行为 ≠ 导航跟踪），但意味着
   `rm_velocity_smoother` 不再是一切角速度的最终限幅。若未来要求“任何角速度都必须
   经过限幅”，需要在 fake 节点里对 `spin_speed` 也应用一个独立的 `spin_max_speed`
   上限（本轮未加，避免过度设计）。
3. **角速度透传**：仅影响 `cmd.angular.z != 0` 的非部署配置；回退方式为把透传改回
   0（不推荐，违背坐标变换语义）。部署默认 `default_wz=0` 下输出不变。
4. **TF fail-stop（单帧零速）**：沿用旧行为，未改；若实车 TF 抖动导致顿挫，
   建议在 `cmdVelCallback` 里改为“TF 失败时保持上一拍偏置 + 限时”，但这会把
   fail-stop 降级为 fail-last，需单独评审。
5. **时钟回拨**：`MonotonicStampGate` 会停发 TF 直到时钟追平；仿真 reset / NTP 步进
   场景需观察，必要时加“回拨阈值重置”。
6. **下游看门狗缺失**：NUC 侧静默时 MCU 可能保持最后一帧，建议电控固件兜底（§4.5）。

---

## 9. 改动文件清单

| 文件 | 变更 |
| :- | :- |
| `src/control/fake_vel_transform/include/fake_vel_transform/velocity_transform.hpp` | 新增：纯函数/结构体/推导注释 |
| `src/control/fake_vel_transform/include/fake_vel_transform/fake_vel_transform.hpp` | 引入纯函数 header；用 `SpinConfig` 替换三个散装 spin 成员 |
| `src/control/fake_vel_transform/src/fake_vel_transform.cpp` | 参数显式化/弃用迁移/校验；回调改用纯函数；TF 短失败用 `offsetAfterMeasurement` |
| `src/control/fake_vel_transform/test/velocity_transform_test.cpp` | 新增 13 个 gtest |
| `src/control/fake_vel_transform/CMakeLists.txt` | 注册 `velocity_transform_test`（+include 目录） |
| `src/control/fake_vel_transform/README.md` | 坐标变换推导、小陀螺显式语义、参数表、验证命令 |
| `src/bringup/config/common/navigation2.yaml` | 新增 `fake_vel_transform:` 段（显式参数 + 注释） |
| `src/navigation/navigation2/params/navigation2.yaml` | 同上（保留上一轮 `repair.time_budget_ms: 80.0` 未动） |

未改：MPC 逻辑与参数、velocity_smoother、serial_driver、其他包、PCD/地图；未 git commit。
