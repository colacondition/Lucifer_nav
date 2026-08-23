# Bringup 参数布局

比赛和仿真的**运行期参数唯一权威源**都在本目录。

## 加载顺序

- `launch/real.launch.py`：`common/*.yaml` → `reality/*.yaml` → launch 动态值
- `launch/sim.launch.py`：`common/*.yaml` → `simulation/*.yaml` → launch 动态值

后加载的值覆盖前面的值。launch 只保留不能静态写入 YAML 的内容，例如：

- `use_sim_time`
- `world` 对应的 PCD/msgpack 路径
- `mode` 决定的建图开关
- `map_save_dir`、动态地图文件名和航点文件路径

## 目录职责

- `common/`：real/sim 完全一致的完整运行参数。
- `reality/`：只保存实车确有差异的参数，例如 Mid360 驱动网络、实车外参和传感器高度。
- `simulation/`：只保存仿真确有差异的参数，例如仿真外参、IMU单位、点云降采样和模拟云台延迟。
- `waypoints/`：比赛航点数据，不是 ROS 参数。

例如 segmentation 的完整参数只在 `common/segmentation.yaml`，两个环境文件只覆盖：

```text
reality/segmentation.yaml    sensor_height: 0.49
simulation/segmentation.yaml sensor_height: 0.175
```

不要再新增 `*_real.yaml`/`*_sim.yaml` 的完整复制版。若只有一两个值不同，应采用 common + 环境覆盖。

各功能包内的 `config/`、`params/` 仅用于脱离总 bringup 单独开发时的兜底，不是实车/仿真的调参真源。
