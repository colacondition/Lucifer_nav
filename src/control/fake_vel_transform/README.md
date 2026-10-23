# fake_vel_transform

哨兵导航的**虚拟云台速度坐标转换**节点。

本工程把整车明确简化成一个**可在地面平移的运动云台**：导航只关心云台参考点的平面平移与
路径切向，不建模底盘小陀螺/物理轮组 yaw；云台到底盘、全向轮解算、力控由电控负责。
本节点只做一件事：把导航下发的"虚拟云台系"速度换算到"实际云台/雷达系"，
**不注入任何自旋角速度**。

## 核心思路

节点创建一个虚拟坐标系 `base_link_fake`：

- `base_link_fake` 的位置跟随 `base_link`（平移恒为 0）；
- `base_link_fake` 的 yaw 跟全局路径 `/plan` 在车体附近的 XY 切向（前瞻
  `path_lookahead_distance`），并做指数平滑。不要用 `pose.orientation`：MINCO 发布的
  `/plan` 朝向是单位四元数，拿它当航向会把 fake 锁到 map +X，重规划时整条轨迹跟着跳；
- 导航控制链路先输出并平滑中间速度 `/cmd_vel`，节点再把 `base_link_fake` 系速度
  换算到 `base_link` 系，发布 `/cmd_vel_chassis`。

坐标变换（完整推导见 `docs/tdt-compare/audit-fakevel.md`）：记 `β = yaw(map→base_link)`、
`φ = yaw(map→base_link_fake)`（`offset = φ - β` 就是本节点广播的
`base_link→base_link_fake` 旋转）。MPC 的 `robot_base_frame=base_link_fake`，输出的
`/cmd_vel` 是 fake 系速度 `v_F`；云台/底盘执行需要 base 系速度：

```
v_B = R(β)ᵀ · R(φ) · v_F = R(φ - β) · v_F = R(offset) · v_F
```

角速度绕 z 不随平面旋转变化，**原样透传** `/cmd_vel.angular.z`（不覆盖、不丢弃）。
`default_wz` 在虚拟云台模型下恒为 0，保留参数只是兼容。

## 节点接口

### 订阅

- `/cmd_vel` (`geometry_msgs/msg/Twist`)：速度平滑后的中间导航速度指令（fake 系）。
- `/plan` (`nav_msgs/msg/Path`)：全局路径。朝向由车体最近点沿弧长前瞻后的 XY 切向估计，不读 orientation。
- TF：查询 `map`（或 `/plan` 的 frame）到 `base_link` 的变换，用于计算真实云台方向。

### 发布

- `/cmd_vel_chassis` (`geometry_msgs/msg/Twist`)：转换到实际云台/雷达系的最终速度指令。
- TF：发布 `base_link` 到 `base_link_fake` 的虚拟坐标变换（100 Hz）。

安全行为：TF 查询失败或数据非有限时，本拍输出零速（fail-stop）；TF 短失败时
`base_link_fake` 偏置保持上一拍值，不会把航向归零（归零会让轨迹瞬间跳变）。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `path_lookahead_distance` | `0.8` | 从最近点沿路径前瞻的距离（m） |
| `yaw_filter_alpha` | `0.25` | 路径切向 EMA 系数 [0,1]。越小 fake 航向越稳，转向越滞后 |

> 旧版的 `spin_enable / spin_speed / spin_min_speed / angular_deadband /
> min_translate_speed_for_spin` 已全部废弃：本工程模型没有底盘小陀螺。节点仍声明这些
> 参数只为兼容旧 yaml，显式设置会打一条 WARN 并忽略。

部署时节点作为组件跑在 `nav_container` 里，用 navigation2 的 params_file，因此参数
写在两份 `navigation2.yaml` 的 `fake_vel_transform:` 段：
`src/bringup/config/common/navigation2.yaml`（bringup 的 sim/real launch 实际加载）
与 `src/navigation/navigation2/params/navigation2.yaml`（navigation2 自带 bringup 的
默认值）。两份必须保持一致。

## 启动

通常由 `bringup` 统一拉起；也可以单独启动：

```sh
source install/setup.bash
ros2 launch fake_vel_transform fake_vel_transform.launch.py use_sim_time:=True
```

实车运行时按需要设置 `use_sim_time:=False`。

## 联调提示

- 如果底盘运动方向与预期相反，优先检查电控坐标约定和 `/cmd_vel_chassis` 的符号。
- 如果 `/cmd_vel_chassis` 没有输出，检查 `/plan`、`/cmd_vel` 和 `map -> base_link` TF 是否存在。
- 如果路径跟踪抖动明显，先降低控制器速度上限，再调整 `yaw_filter_alpha`。
- 改完 fake 节点的参数后需重启导航容器（组件在启动时读参，不支持动态回填）。

## 验证

```sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fake_vel_transform \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select fake_vel_transform --event-handlers console_direct+
```

单元测试覆盖旋转符号/组合恒等式、恒等偏置、零速、角速度透传、TF 失败、
`base_link_fake` 偏置保持（`test/velocity_transform_test.cpp`）。
