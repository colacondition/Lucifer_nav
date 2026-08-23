# 在导航框架中使用 Waypoint Editor

[English Version](USAGE_WITH_RM_NAVIGATION.md)

当前仓库的导航链路使用接口：

- 目标输入话题：`/goal_pose`
- 导航状态话题：`/navigation2/status`
- 当前内存航点话题：`/waypoint_editor/current_waypoints`

`waypoint_editor` 已经直接适配这套接口，不再依赖 Nav2 的 map server 或 waypoint action。

## 前提

- `/map` 已由定位模块发布，通常是 `slam_toolbox`
- 导航框架从 `/goal_pose` 接收目标
- 导航框架在 `/navigation2/status` 发布执行状态

## 编译

```bash
cd /home/cola/HL_navigation_27
colcon build --packages-select bringup waypoint_editor --symlink-install
source install/setup.bash
```

如果只单独调试 waypoint editor，也可以只编译 `waypoint_editor`。但如果要通过 `bringup` 的导航 bringup 直接使用航点执行，建议把 `bringup` 一起重新编译，避免 launch 文件仍然来自旧 install 目录。

## 直接在导航 bringup 中使用

仿真和实车导航 bringup 都会启动同一个 `waypoint_executor`（节点名 `waypoint_follow_executor`，默认 follow）。逐点巡逻用 `waypoint_patrol.launch.py`，仍是这一份可执行文件，只改 `mode:=patrol`。

常用命令：

```bash
ros2 launch bringup sim.launch.py \
  world:=RMUL \
  mode:=nav \
  nav_rviz:=True
```

实车使用 `bringup real.launch.py`，其余 waypoint 面板、服务和话题保持一致。

`/map` 由 navigation2 的 `rm_map_server` 发布。选择 `.yaml` 时面板只会提示使用现有 `/map`；选择 `.pcd` 时可加载点云地图做可视化。

## 启动独立编辑器

```bash
ros2 launch waypoint_editor waypoint_editor.launch.py
```

在 RViz2 中：

- Panels -> Add New Panel -> 选择 `WaypointEditorPanel`
- Tools -> Add New Tool -> 选择 `Add Waypoint`

## 标点、删除与保存

- 使用 `Add Waypoint` 在地图上拖拽标点和朝向
- 面板里的 `WPs` 数量会显示当前航点数量
- `Delete Last` 删除最后一个航点
- `Save WPs` 把航点保存为 CSV，供下次加载或决策脚本使用
- `Load WPs` 从 CSV 重新载入航点

保存示例：

```bash
/home/user/my_waypoints.csv
```

现在不需要保存也能跑。编辑器会把当前内存航点发布到 `/waypoint_editor/current_waypoints`，执行器会优先使用当前内存航点；只有当前内存航点为空时，才回退读取 CSV。

## 执行航点

根据任务选择下面两种模式之一。

### 模式 A：平滑穿点

机器人接近当前航点后就提前切到下一个目标，更适合平滑穿越路径。面板按钮是 `Start Follow`，服务名是：

```bash
ros2 service call /start_waypoint_following std_srvs/srv/Trigger
```

如果没有使用 `bringup`，可以在另一个终端单独启动执行器：

```bash
ros2 launch waypoint_editor waypoint_follow.launch.py waypoint_file:=/path/to/your/waypoints.csv
```

平滑穿点默认 `switch_distance=0.6`，中间点会临时发布 `/goal_approach_controller/enabled=false`，最后一个点再恢复为 `true`。如果中间点仍明显减速，优先检查局部控制器自身的目标减速、`switch_distance` 是否太小，以及航点之间是否距离过短或转角过急。

### 模式 B：逐点巡逻

机器人按顺序到达每个航点，当前航点完成后再发送下一个。面板按钮是 `Start Patrol`，服务名是：

```bash
ros2 service call /start_waypoint_through std_srvs/srv/Trigger
```

如果没有使用 `bringup`，可以在另一个终端单独启动执行器：

```bash
ros2 launch waypoint_editor waypoint_patrol.launch.py waypoint_file:=/path/to/your/waypoints.csv
```

逐点巡逻会保持 `/goal_approach_controller/enabled=true`，因此每个航点附近都会按终点减速。

## 给 C++ 决策节点使用

`ros2 run decision bt_action_replacement_node` 可以读取 waypoint editor 保存的 CSV：

- `targets.center_waypoint_file`：前往中心点路径，平滑穿点执行
- `targets.home_waypoint_file`：回家路径，平滑穿点执行
- `targets.wait_home_waypoint_file`：等待回家路径，平滑穿点执行

示例：

```bash
ros2 run decision bt_action_replacement_node \
  --ros-args \
  --params-file src/HL_decision/config/bt_action_replacement.yaml \
  -p targets.center_waypoint_file:=/path/to/center.csv \
  -p targets.home_waypoint_file:=/path/to/home.csv \
  -p targets.wait_home_waypoint_file:=/path/to/wait_home.csv
```

决策节点当前只使用自身受击状态、`current_hp`、比赛阶段和剩余时间做目标选择；参数通过 YAML 或 `-p` 覆盖。受击躲避开启时，节点会短距离发布临时目标，结束后继续执行主目标。

## 排查

如果点击 `Start Follow` 或 `Start Patrol` 后只卡一下、不走：

```bash
ros2 node list | grep waypoint
ros2 topic echo /waypoint_editor/current_waypoints --once
ros2 topic echo /goal_pose --once
ros2 topic echo /navigation2/status
```

如果看不到 `waypoint_follow_executor` 节点，重新编译并重新 source：

```bash
colcon build --packages-select bringup waypoint_editor --symlink-install
source install/setup.bash
```

## 启动入口

- 平滑穿点使用 `waypoint_follow.launch.py`
- 逐点巡逻使用 `waypoint_patrol.launch.py`
