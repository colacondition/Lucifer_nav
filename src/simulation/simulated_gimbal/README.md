# simulated_gimbal

仿真专用云台模拟器：顶替实车电控的云台姿态回传。

实车链路上 `/gimbal_posture_state` 由 `serial_driver` 转发电控回传的**实测姿态**，
电控持续上报当前云台姿态（不是只在变化时发一次）。仿真里没有电控、也没有
`serial_driver`，这条回传永远是空的 —— 一旦语义地图有隧道，MPC 会因
`gimbal_lowered_` 停在 false 而**永久停车**（见 `mpc_controller_node.cpp` 的
「等云台收下来再进洞」门控）。

这个节点顶替电控：

- 订阅 `/gimbal_posture`（`rm_tunnel_posture` 的收/放请求）；
- 收到目标变化后等 `action_delay`（默认 0.5 s，模拟电控执行时间）翻转内部姿态；
- 以 `report_rate`（默认 20 Hz）**持续回传** `/gimbal_posture_state` 当前姿态 ——
  动作期间保持旧值，到点翻转后继续发，和电控的上报行为一致。

这样仿真就能复现「请求 → 停顿 → 放下/升起 → 走」的完整时序，RViz 的云台状态显示
（`rm_gimbal_visualizer` + `GimbalStatus` 显示块）在仿真里也能实时看到实态与命令的
滞后，而导航侧不需要改任何代码。

## 参数

| 参数 | 默认 | 说明 |
| :- | :- | :- |
| `posture_topic` | `/gimbal_posture` | 收/放请求话题 |
| `state_topic` | `/gimbal_posture_state` | 回传话题（transient_local） |
| `action_delay` | `0.5` | 动作执行时间（s） |
| `report_rate` | `20.0` | 持续回传频率（Hz） |

## 启动

仿真 `nav` 模式下由 `sim.launch.py` 自动加载（`condition = mode == nav`），不需要单独启动。
单独调试：

```sh
ros2 run simulated_gimbal simulated_gimbal_node
```
