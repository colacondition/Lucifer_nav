# 双 MID-360 集成（驱动层融合）

> 配套的**测试 / 标定**工作空间：<https://gitee.com/colacondition/dual_lidar_test>
> （指标测试、A↔B 一键标定、PTP 工具、两雷达位置可视化、BEV、无硬件仿真都在那边）

> 目标：**补盲雷达 B 接进来，但导航链路一行配置都不改。**
> 做法：融合做在 `mid360_driver` 内部，对外仍然只有 `lidar_topic` 一个话题。
>
> 前置调研见 `docs/rm-sentry-dual-lidar-survey.md`，标定工具的用法见
> `src/driver/mid360_driver/INTERNAL_LIDAR_MERGE.md`。

## 一、一图看清改动范围

```
   A 192.168.1.101 (Mid-360S) ─┐
                               ├─► mid360_driver（内部融合 + 外参 + 裁近点）
   B 192.168.1.102 (Mid-360)  ─┘            │
                                            ▼
                            /livox/lidar/pointcloud   ← 与单雷达时**同一个话题**
                            frame_id = livox_frame    ← 同一坐标系（主雷达系）
                                            │
                    ┌───────────────────────┼───────────────────────┐
                    ▼                       ▼                       ▼
              cpp_lidar_filter          small_glim            两套 costmap
              （没改）                  （没改）               （没改）
```

**唯一变的是驱动的一个参数文件**，其余包全部按原样工作。

## 二、怎么开

```bash
ros2 launch bringup real.launch.py mode:=nav dual_lidar:=true
#                                                    ^^^^^^^^^^^^^^^^^
#   不传就是原来的单雷达行为（mid360_driver_real.yaml），默认 false
```

`dual_lidar:=true` 会：

1. 改用 `config/reality/mid360_driver_dual.yaml`（多设备分流 + 驱动内融合 + 标定外参）；
2. 额外发一条 `base_link → livox_frame_b` 静态 TF（纯可视化/排障用，融合不依赖它）。

其他 launch 参数、其他包里的一切都不变。

## 三、实测数据（2026-10-19，实车台架）

| 项 | 数值 |
| --- | --- |
| 合并话题 | `/livox/lidar/pointcloud`，**10.000 Hz** |
| 合并帧点数 | **13,757**（不裁近点时为 38,706） |
| 内部融合丢弃 | **0 帧**，两路帧头差 0.00~0.23 ms |
| 外参（副→主） | 平移 (-2.8, +124.7, -10.6) mm，旋转 (-0.58°, +0.25°, +0.50°) |
| 外参可信误差 | ~1~1.5 cm / 0.4°（两条独立标定路径之差） |
| 两雷达间距 | 12.5 cm，相对姿态 <1°（**不是**"左右各 30 cm"！） |
| 时间同步 | PTP 锁定，`time_offset` −0.32 ms，相对漂移 **0.42 ppm**（无 PTP 时 10.2 ppm 且持续累积） |
| 带宽 | 单台 200,000 点/s、2083 包/s、0% 丢包；两台合计约 44 Mbps |

## 四、关键设计决定（都踩过坑）

| 决定 | 原因 |
| --- | --- |
| 融合放在驱动里，不新增节点 | 下游零改动；顺带绕开"驱动给所有设备发同一个 `frame_id`"的问题 |
| `merge_max_interval_ms: 20`（不是上游的 5） | 我们**共用一个 100 ms 定时器**发布，两路累积窗口的最小时间戳天然差几毫秒；5 ms 会误杀一半帧（实测合并率掉到 5 Hz） |
| 超窗只丢**更旧**的那一路 | 两路都丢会让落后的那路下一拍再落后一帧，形成"丢一帧、错一拍"死循环 |
| `merge_min_point_range: 0.3` | 点云里 2/3 的点在 30 cm 内（自车/工装），不裁则合并帧 38.7k、白吃渲染和算力（参考 HERO_2026 的 `blind_distance`） |
| 默认 `merge_publish_individual: false` | 单路话题没必要存在；离线标定/排障时改 true |
| `max_point_range: 30`（不是 300） | 晃车时会扫到窗外几十米的点，对导航无用且吃性能 |

## 五、PTP（时间同步）

雷达侧**不需要任何配置**：官方文档写明"接入带 PTP 主钟的网络后设备会自动同步"，优先级最高。
判据只看只读状态：`time_sync_type = 1`、`time_offset` 几百 µs。

```bash
# 手动起
sudo src/bringup/tools/ptp_master.sh start
src/bringup/tools/ptp_status.sh

# 或者开机自启（推荐上车用）
sudo cp src/bringup/tools/ptp-master.service /etc/systemd/system/
sudo sed -i 's/enp86s0/enp7s0/' /etc/systemd/system/ptp-master.service   # 改成本机网口
sudo systemctl daemon-reload && sudo systemctl enable --now ptp-master
```

时间戳模式：

| 网卡 | 参数 | 精度 |
| --- | --- | --- |
| Realtek RTL8111（开发机） | `-S`（软件） | offset ≈ −0.32 ms，抖动 ±0.9 ms |
| **Intel i225-LM（NUC11PAHi5）** | **`-H`（硬件，有 PHC）** | 预期 10~50 µs |

传输层默认 `-2`（IEEE 802.3 二层）：不依赖 IGMP，非管理交换机也不会丢 PTP 多播。

## 六、上车前检查清单

```bash
# 1) 驱动 + 融合 + 帧率（一条命令）
ros2 launch bringup real.launch.py mode:=mapping dual_lidar:=true
#   另开终端：
ros2 topic hz /livox/lidar/pointcloud        # 期望 10.000
grep "内部融合统计" ~/.ros/log/latest/*.log   # 期望 丢弃 0 帧

# 2) PTP
src/bringup/tools/ptp_status.sh

# 3) 外参没被占位值覆盖（0, -0.30, 0 是错的）
grep -A2 merge_extrinsic_secondary_to_primary \
  src/bringup/config/reality/mid360_driver_dual.yaml

# 4) 网口与 PHC（NUC 上应该出现 PTP Hardware Clock: 0）
lspci -nn | grep -i ethernet
ethtool -T enp86s0 | grep -i "PTP Hardware Clock"
```

## 七、已知限制

* 外参误差 ~1 cm / 0.4°：**够 2D 代价地图**，对 LIO 偏大（那里要 0.05° 量级）。想进 LIO
  得用标定板或更干净场景再标一次；目前设计上 B 只贡献点云给 LIO，如果发现建图变差，
  先把 `merge_publish_individual: true` 并只喂 A，验证是不是 B 拖的。
* 驱动发布帧率的"丢帧"计数在 Python 监控里会偏高（DDS best-effort 在订阅侧丢样本）。
  量真实帧率用 `ros2 topic hz` 独占进程，或看驱动的"内部融合统计"。
* 从雷达的 `frame_id` 仍是 `livox_frame`（驱动对所有设备用同一个），**融合后没问题**，
  但**不要**把 B 的原始话题直接丢给任何按 TF 变换的消费者。

## 八、联调排障实录（2026-10-19，第一次真正跑通）

按发生顺序，四个坑里有两个是我引入的、两个是环境问题：

| # | 现象 | 根因 | 修法 |
| --- | --- | --- | --- |
| 1 | `small_glim` 每帧 `drop LiDAR frame while waiting for IMU data (last_imu_time=0.000000)`，**一帧没用上** | `is_topic_name_with_lidar_ip: true` 会给**点云和 IMU 都**加 IP 后缀；融合只把点云改回无后缀，IMU 还在 `/livox/imu_192_168_1_101` 上 | 融合模式下**主雷达的 IMU 发到 `/livox/imu`**（无后缀），副雷达仍带后缀 |
| 2 | `small_glim` 每 2 s 崩一次（SIGABRT） | `libpcl_io.so.1.12: undefined symbol: libusb_set_option`（dlopen 失败 → class_loader 抛异常）。`real.launch.py` 里 `system_libusb_env` 只挂给了 perception 容器和 fast_location，**没挂给 lio_node** | 给 `lio_node` 加 `additional_env=system_libusb_env`。**单雷达模式下同样会崩**，只是以前没人注意到 |
| 3 | `mid360_driver` 每 2 s 段错误（SIGSEGV） | 融合开启 + `merge_publish_individual: false` 时我**不创建单路发布器**，但"合并失败的那一拍"（启动瞬间某台还没数据）会掉进单帧发布分支 → 空指针 | `if (enable_internal_lidar_merge && !merge_publish_individual) return;`；另加三处发布器判空 |
| 4 | 全流程没有任何数据 | **主雷达 A 的网线抖了**：设备侧 HMS 报 `0x0401`（360S 专用"通信链路曾断开、现已恢复"）+ `0x0405`（时间同步异常）。工控机侧 `carrier_changes` 没涨、`rx_errors=0`，所以抖的是 A 自己那一段。抖动后 A **推流器卡住**：控制/状态照常应答，点云/IMU 不再推送 | 重插网线/换线；`livox_set_ip.py --lidar-ip <ip> --reboot` 可让推流器恢复（实测重启后立刻回到 2083 包/s、0% 丢包） |

**HMS 速查**（`--query 0x8011`，`02 00 01 04` 这种 4 字节小端）：

| 码 | 级别 | 含义 |
| --- | --- | --- |
| `0x0104` | warning | 光窗脏污，影响点云可靠性（B 之前一直是，擦干净后点数 18.3k→19.35k） |
| `0x0401` | warning | **通信链路曾断开（360S 专用）** ← 线有问题看这个 |
| `0x0405` | warning | 时间同步异常（多半是链路抖动导致 PTP 丢失） |
| `0x0409` | warning | GPS 信号异常（没接 GPS 时正常） |

## 九、降级模式（新增，务必保留）

一颗雷达掉线时**不能一帧都不发**，否则 LIO / 两套代价地图瞬间全盲。行为：

* 掉线 < `merge_degraded_timeout_ms`（默认 1000 ms）：仍然**不发半帧**（短暂抖动不该污染 LIO）；
* 超过 1 s：只用"还活着"的那几路拼一帧发出去，`RCLCPP_WARN_THROTTLE` 每 2 s 提示一次，
  并明示"检查网线/供电，设备侧 HMS 可能报 0x0401"；
* 恢复后自动回到正常融合（日志里会看到发布帧数继续增长）。

## 十、CPU 预算（实测，24 核机器）

| 进程 | %CPU（一核=100%） | 内存 |
| --- | --- | --- |
| `mid360_driver`（双设备 + 融合） | 10.0% | 26 MB |
| `small_glim` | 33.8% | 267 MB |
| `perception_container`（filter + 地面分割） | 5.4% | 63 MB |
| `slam_toolbox` | 1.7% | 39 MB |
| **合计** | **≈ 51%（半个核）** | ≈ 400 MB |

**结论：不需要加核。** 合并帧 13.7k 点 < 单雷达 19.8k 点，下游按点数线性消耗，反而更省。
唯一贵的是 RViz（一帧 11 万点时 GPU 都扛不住），车上保持 `nav_rviz:=False`。
