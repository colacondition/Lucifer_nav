# 驱动内部双雷达融合（enable_internal_lidar_merge）

> 标定（`extrinsic_b`）与指标测试在配套工作空间：<https://gitee.com/colacondition/dual_lidar_test>，
> 用 `./run.sh calib` 一键产出，再填到 `merge_extrinsic_secondary_to_primary`。

> 参考实现：[`Walker152/navi_minco_bit`](https://github.com/Walker152/navi_minco_bit) 的
> `src/perception/livox_ros_driver2/INTERNAL_LIDAR_MERGE.md`（它把融合做在
> `livox_ros_driver2` 里，对外仍输出 `livox/lidar`，Point-LIO 零改动）。
> 本仓库的驱动是自研被动 UDP 版，所以按同样的**设计意图**重写，不搬代码。

## 为什么放在驱动里

1. **下游零改动**：`small_glim`、`fast_location`、`rm_local_costmap`、`rm_global_costmap`
   都只认一个 `PointCloud2` 话题，不用为双雷达加第二路观测源。
2. **顺手解决 `frame_id` 问题**：驱动现在给所有设备用同一个 `lidar_frame`（B 的
   frame_id 是错的）；在驱动里合并后，输出帧的 frame_id 就是主雷达系，自洽。
3. **不需要额外进程/带宽**：合并发生在发布前，省掉一次「订阅两路 → 合并 → 重发」
   的 480 KB×2 拷贝。

## 开关与参数（全部默认关闭，关闭时行为与原来逐位一致）

```yaml
mid360_driver:
  ros__parameters:
    is_topic_name_with_lidar_ip: true      # 必须 true，否则无法按 IP 区分设备
    enable_internal_lidar_merge: true      # ← 总开关，默认 false
    merge_primary_ip: "192.168.1.101"      # 主雷达（空 = 取 IP 数值最小者）
    merge_output_topic: "/livox/lidar"     # 生产环境用 /livox/lidar，下游零改动
    merge_frame_id: "livox_frame"
    merge_max_interval_ms: 20.0            # 两路帧头差上限
    merge_extrinsic_secondary_to_primary: [-0.002794, 0.124692, -0.010606,
                                           -0.010094, 0.004333, 0.008671]
    merge_publish_individual: false        # 是否仍发单路话题（标定时要 true）
    merge_min_point_range: 0.3             # 裁掉离主雷达这么近的点（自车/工装回波）
```

`merge_extrinsic_secondary_to_primary` = **副→主**，格式 `[x, y, z, roll, pitch, yaw]`
（米 / 弧度），与 `dual_lidar_calibration` 输出的 `translation_m` + `rotation_rpy_rad`
**完全一致，可直接粘**。

## 行为

* 每 `lidar_publish_time_interval` 一拍：两路各自 swap 出本拍累积的点；
* **两路都有数据**且帧头（各自最小点时间）差 ≤ `merge_max_interval_ms` → 合并发布；
* 否则**丢掉更旧的那一路**，这一拍不发合并帧（绝不发半帧）。注意只丢旧的：
  若把新鲜的也丢了，下一拍它会落后整整一帧，形成"丢一帧、错一拍"的死循环
  （实测能把合并帧率压到 3~5 Hz）；
* 合并时：主雷达点原样，副雷达点按外参变换到主雷达系（只转 x/y/z，
  intensity 与每点 timestamp 原样保留），再按**每点时间戳排序**，
  `header.stamp = min(两路的最小时间戳)`。

## 实测（2026-10-19，A=.101 Mid-360S / B=.102 Mid-360）

```
内部融合统计：发布 151 帧，丢弃 0 帧，最近帧头差 0.31 ms（窗口 20.0 ms）
合并帧点数：38624 ≈ 20002（A）+ 18123（B）
```

> ⚠️ **不要用 `ros2 topic hz` 判断合并帧率**：38k 点 ≈ 930 KB/帧，Python 订阅者
> 反序列化不过来，会显示 1~5 Hz（还会打 `A message was lost`），而驱动自己是 10 Hz。
> 看驱动日志里的「内部融合统计」才准。

## 为什么窗口默认是 20 ms 而不是上游的 5 ms

上游按**自然帧边界**配对（`QueuePeek` 两个队头包比 `base_time`），5 ms 合适；
我们是**共用 100 ms 定时器**把两路各自的累积缓冲拼起来，两路最小时间戳天然会差
几毫秒。5 ms 会误杀一半的帧（实测合并帧率 5 Hz 左右）。20 ms ≪ 100 ms 帧周期，
既不会配错帧，又足够容忍抖动。

## `merge_min_point_range`：裁自车近点（实测 -65% 点数）

我们的点云里 **2/3 的点在 30 cm 以内**（台面/工装/自车回波）。这些点对导航毫无用处，
却要白吃带宽、RViz 渲染、LIO/代价地图的算力。参考
[HERO_2026_Sentry_NAV](https://github.com/LiuJinbo1027/HERO_2026_Sentry_NAV) 的
`lidar_merge` 里的 `blind_distance`（默认 0.25 m，在融合时就裁掉原点附近的点）。

实测（A=.101 / B=.102，同一场景同一时刻）：

| `merge_min_point_range` | 合并帧点数 | 帧率 | 丢弃 |
| --- | --- | --- | --- |
| 0.0（默认，不裁） | **38,706** | 10 Hz | 0 |
| **0.3** | **13,422** | 10 Hz | 0 |

**默认值保持 0.0 是为了不改变既有行为**；生产环境建议设 **0.3**（机器人体型决定，
我们的云台/工装都在 30 cm 内）。注意这个裁剪是以**主雷达原点**为球心、在变换之后做的，
所以对副雷达的点也一致。
