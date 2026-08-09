# Lucifer Navigation

RoboMaster 哨兵导航工作空间，ROS 2 Humble + Gazebo Classic 11，支持仿真与实车。

## 一. 项目介绍

传感器是 Livox Mid360（雷达 + 内置 IMU），场地支持 RMUC / RMUL。

`navigation2` 是组件化的单个功能包：8 个导航节点编进同一个 shared library，
用 `rclcpp_components` 注册，全部加载进一个 `component_container_mt`，进程内零拷贝。
全局规划用 A*，在 `OccupancyGrid` 上构建 2D 距离场做 clearance cost 让路径远离障碍；
平滑后的 `/plan` 交给 MPC 控制器跟踪，控制器内含弧长进度跟踪、卡住检测、
倒车与安全点脱困恢复链和速度剖面。

数据流：

```text
Mid360 点云 ──┬─► Super-LIO ──► /Odometry, /Laser_map
              │      ▲
              │      └── /livox/imu
              └─► cpp_lidar_filter ──► linefit_ground_segmentation
                                            └─► /segmentation/obstacle
                                                       │
/Laser_map ──► fast_location ──(map→odom)──┐           │
                                           ▼           ▼
                                  navigation2 (A* + 距离场 + MPC)
                                           │
        /cmd_vel_nav_raw ──► goal_approach_controller
                             ──► /cmd_vel_nav ──► velocity_smoother
                             ──► /cmd_vel ──► fake_vel_transform
                             ──► /cmd_vel_chassis ──► 底盘 / Gazebo
```

主要话题：

| Topic | Type | 说明 |
| :- | :- | :- |
| `/livox/lidar/pointcloud` | `PointCloud2` | 雷达点云（实车另有 `/livox/lidar` 的 CustomMsg） |
| `/segmentation/obstacle` | `PointCloud2` | 地面分割后的障碍点，代价地图输入 |
| `/Odometry` | `Odometry` | Super-LIO 里程计 |
| `/Laser_map` | `PointCloud2` | Super-LIO 世界系点云，`world` 系 |
| `/map` | `OccupancyGrid` | 先验栅格地图 |
| `/goal_pose` | `PoseStamped` | 导航目标 |
| `/plan` `/predict_path` | `Path` | 全局路径 / MPC 预测轨迹 |
| `/cmd_vel_chassis` | `Twist` | 最终底盘速度 |

## 二. 代码结构

```text
src/bringup                                  总启动，仿真/实车两套 launch 与 config
src/driver/livox_ros_driver2                 Livox Mid360 驱动
src/perception/cpp_lidar_filter              去车身点云 + 降采样
src/perception/linefit_ground_segmentation_ros2
                                             地面分割（linefit_ground_segmentation
                                             + _ros 两个包）
src/localization/basic                       super_lio 的公共基础库
src/localization/super_lio                   LIO 里程计与建图
src/localization/fast_location               点云对先验 PCD 的主定位
src/navigation/navigation2                   导航栈（组件化，A* + 距离场 + MPC）
src/control/goal_approach_controller         目标接近减速
src/control/fake_vel_transform               底盘速度坐标转换
src/control/waypoint_editor                  航点编辑与 follow / patrol 执行器
src/decision/decision_node                   决策节点（包名 decision）
src/interfaces/custom_msgs                   自定义消息（包名 decision_interfaces）
src/serial                                   实车串口（包名 serial_driver）
src/simulation/pb_rm_simulation               Gazebo 场景
src/simulation/ros2_livox_simulation          Livox 仿真插件
```

`bringup` 下的关键文件：

```text
launch/sim.launch.py                         仿真入口
launch/real.launch.py                        实车入口
config/navigation2.yaml                      导航参数（launch 实际加载的是这份）
config/fast_location_main.yaml               定位参数
config/{simulation,reality}/                 分环境的外参、分割、LIO 参数
urdf/sentry_robot_{sim,real}.xacro            机器人模型
map/<world>.{pgm,yaml}                       代价地图
PCD/<world>.pcd                              fast_location 的先验点云图
rviz/navigation.rviz                         nav 模式（Fixed Frame = map）
rviz/mapping.rviz                            mapping 模式（Fixed Frame = world）
```

`navigation2` 的 8 个组件：`rm_map_server`、`rm_global_costmap`、`rm_global_planner`、
`rm_path_smoother`、`rm_local_costmap`、`rm_mpc_controller`、`rm_velocity_smoother`、
`rm_nav2_compat`；`goal_approach_controller` 从独立包组合进同一容器。

## 三. 编译

```sh
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
./build.sh
source install/setup.bash
```

`build.sh` 按依赖顺序、单线程、低优先级编译，避免全量并发把机器压死。
它会检查 `src/` 下是否有没写进列表的包并给出警告。

```sh
./build.sh --list                  # 看编译顺序
./build.sh navigation2 bringup     # 只编指定包，顺序仍按依赖表
```

实车驱动需要另装 Livox SDK2（`/usr/local/lib/liblivox_lidar_sdk_shared.so`），
仿真不需要。

## 四. 运行与调试

### 4.1 常用参数

| 参数 | 默认 | 说明 |
| :- | :- | :- |
| `world` | `RMUL` | 需与 `map/<world>.*`、`PCD/<world>.pcd` 同名 |
| `mode` | `nav` | `mapping` 建图 / `nav` 导航 |
| `nav_rviz` | 仿真 `True`，实车 `False` | 启动 RViz，配置按 `mode` 自动选 |
| `gazebo_gui` | `True` | 仿真专有 |
| `software_rendering` | `False` | 无 GPU 时置 `True`，避免 RViz/Gazebo 段错误 |
| `map_save_dir` | `<bringup>/PCD` | 建图输出目录 |
| `node_output` / `log_level` | `log` / `warn` | 排查时改 `screen` / `info` |
| `use_serial_driver` / `use_decision` | `True` | 实车专有 |


### 4.2 建图

仿真：

```sh
ros2 launch bringup sim.launch.py world:=RMUL mode:=mapping nav_rviz:=True
```

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_chassis
```

实车：

```sh
ros2 launch bringup real.launch.py world:=RMUL mode:=mapping nav_rviz:=True
```

### 4.3 导航

```sh
ros2 launch bringup sim.launch.py  world:=RMUL mode:=nav nav_rviz:=True
ros2 launch bringup real.launch.py world:=RMUL mode:=nav
```
