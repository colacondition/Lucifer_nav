# 增强 PGM：用 3D 障碍点云补低矮台面

## 为什么需要

建图主链：

```text
/segmentation/obstacle -> pointcloud_to_laserscan -> /scan -> slam_toolbox -> /map
```

`LaserScan` 每个角度只能存**一个最近距离**。20 cm 高台的顶面点从多个角度/距离
进来，会被前沿/地面射线压掉，PGM 上只剩几个格子；这是 2D 表达的上限，不是
地面分割或高度阈值的问题（`min_height` 已按地面以上 5 cm 配好）。

本工具在建图时旁路订阅 `/segmentation/obstacle`，把**所有**落在
`地面+min_height .. 地面+max_height` 的点变换到 map 系，按 `/map` 的
分辨率投影计数；同一格命中次数达到 `--min-points` 才保留（过滤动态/噪点），
再膨胀 `--dilate` 格，叠加到 slam_toolbox 的 `/map` 上，输出增强 PGM。

## 用法

建图（`mode:=mapping`）正在跑时，另开终端：

```bash
# 仿真：雷达高 0.175；实车按卷尺量，通常 0.49 左右
ros2 run bringup enhance_map_with_cloud.py \
  --sensor-height 0.175 \
  --duration 20 \
  --min-points 3 \
  --min-height 0.03 \
  --max-height 0.60 \
  --dilate 1 \
  --output /tmp/RMUL_enhanced
```

会写出 `/tmp/RMUL_enhanced.pgm` + `.yaml`。之后转 msgpack：

```bash
python3 src/bringup/tools/pgm_to_navmap.py /tmp/RMUL_enhanced.yaml \
  -o src/bringup/map/<world>.msgpack
```

## 参数

| 参数 | 默认 | 说明 |
| :- | :- | :- |
| `--sensor-height` | 必填 | 雷达离地高度；用于 `ground_z = base_link.z - sensor_height` |
| `--ground-z` | 空 | 直接指定 map 系地面 z（坡道/地图 z 原点异常时用） |
| `--min-height` | 0.03 | 地面以上最小高度，滤掉地面残留 |
| `--max-height` | 0.60 | 地面以上最大高度；高墙 slam_toolbox 已有，不必重复投 |
| `--min-points` | 2 | 同一格至少命中次数；动态点只出现 1 次，调大更干净 |
| `--max-range` | 8.0 | 只投距 map 原点半径内的点；0 关闭 |
| `--dilate` | 1 | 膨胀格数，把稀疏边缘连成线 |
| `--duration` | 15 | 采样秒数；越长越稳，建图时机器人要走过台面 |

## 注意

- 使用 `--sensor-height` 时是**逐帧**用 `base_link.z - sensor_height` 估算地面；
  地图 z 原点必须与 `base_link` 同一参考（small_glim 的 odom 原点锚在开机雷达位姿，
  地面约在 `-h`）。坡道/地面起伏明显时用 `--ground-z` 或分段处理。
- 只叠加**占据**，不改原 PGM 的空闲/未知；不会凭空把整片区域变障碍。
- 如果台面在 `--max-height` 之外（例如 1 m 高台），把它调大；如果地面残留多，
  把 `--min-height` 调大或 `--min-points` 调大。
- 这个工具解决的是"PGM/msgpack 里看不到低矮台面"；导航在线避障仍应修 costmap
  高度带（`obstacle_z_min_to_robo` 相对雷达平面的问题）。
