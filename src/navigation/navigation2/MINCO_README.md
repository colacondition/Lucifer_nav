# MINCO Path Smoother

基于 ROSE 方案的 MINCO（Minimum Control Effort）轨迹优化器，用于平滑 A* 规划的路径。
优化默认开启（`enable_optimization: true`），几何平滑 + 数据保持 + 隧道轴向对齐 + 进程内距离场障碍 soft 代价。

在原有基础上引入了三件来自 TDT-nav-kit（MIT License，SnifferCaptain & Nathongc）的机制：
**方形安全走廊（SFC）全图校验**、**有界迭代修复**、**隧道路标点横向走廊约束**。
三者都不改变默认的优化器形式（仍是无约束 L-BFGS），只是把「软惩罚 + 事后整条拒绝」
升级成「软惩罚 + 有界修复 + 双通道校验」。

> **与 MPC 的参数耦合**：`speed_norm.enforce: true` 把 MPC 的轴向最大速度从
> `max_speed` 压到 `0.924 * max_speed`，而 `local_safety.check_steps` 是**时间**
> 窗、安全判据关心**距离**。两者必须一起改（当前 `check_steps: 11` 就是补偿
> 2.0 → 1.85 m/s 后的取值）。改任一个都要复核另一个。

## 功能特性

- **固定时间、形状优化**：L-BFGS 只优化内部路标点 XY，段时间不作为优化变量
- **转角感知时间初值**：距离/速度基础上为急弯两侧增加过渡时间
- **动力学连续性**：最小化 jerk（加加速度），保证轨迹平滑
- **L-BFGS 优化**：高效的非线性优化求解器
- **全图 SFC 校验**（`global_check.*`）：从全局代价地图建「以采样点为中心的最大
  无致命格方形」，半宽 < `robot_radius` 即拒绝。补上局部 5×5 m 距离场看不到的
  全局障碍盲区。实现见 `sfc_corridor.{hpp,cpp}`
- **有界迭代修复**（`repair.*`）：校验失败先修复再重优化，预算用尽才回退。
  第 1 次放大 `data_weight`（弱约束，压住偏离前端路径的自由度），第 2 次在最偏离
  处插入路标点（强约束，给 L-BFGS 新自由度），与 TDT 的碰撞迭代同一思路
- **几何去重**：全局规划器 5-8 Hz 重发同一条路径时不再重复跑 L-BFGS
  （指纹 = 路径几何 FNV-1a + 语义地图代次；障碍距离场不进指纹，理由见代码注释）
- **隧道横向走廊**（`tunnel_corridor_weight`，默认 20.0）：路标点横向偏移超出
  `max(0, clear_width/2 - robot_radius) + lateral_margin` 时加 `w * 超出量²`，
  出洞后按距离线性放宽（洞口喇叭形）。与轴向项互补：轴向项管「走向正不正」，
  这项管「别贴壁」。
  **RMUL 的真实情形是半宽恰好 0**：两条隧道的 `clear_width` 都是 0.5 m，而
  代价地图的 `robot_radius` 是 0.25 m —— 车体直径等于净宽，物理余量为零。
  半宽 0 的语义是「把车压在轴线上」，不是「没有约束」；`lateral_margin_m`
  是唯一的放宽手段，只有实测车体半径明显小于 0.25 m 时才需要调大。
  （最初的实现把半宽 <= 0 当成「给不了约束」直接返回，于是这项在真实地图上
  静默失效 —— 参数看着打开了，一次都没触发过。已由
  `test_semantic_map_consumer.cpp` 的真实地图用例钉死。）
- **两阶段 v/a 精确检测**（`two_stage.exact_dynamics_check`）：用 MINCO 库已有的
  多项式求根替代每段 5 点采样。5 阶多项式的速度是 4 次函数，5 个等距样本会漏掉
  段内尖峰、把峰值低估 10-20%。放大后还会复核一次，超限只告警不改行为

## 架构说明

### 核心组件

1. **MincoOptimizer** (`minco/minco_optimizer.cpp`)
   - MINCO 优化器封装
   - 使用 MINCO_S3NU（S=3 表示最小化 jerk，NU 表示非均匀时间）
   - L-BFGS 求解器配置

2. **RmMincoPathSmoother** (`minco_path_smoother_node.cpp`)
   - ROS 2 节点封装
   - 订阅原始路径 `/plan_raw`、语义地图、全局代价地图
   - 发布优化后路径 `/plan`

3. **MINCO 库** (`minco/minco.hpp`)
   - 来自 ROSE 的 MINCO 实现
   - 带状线性系统求解器
   - 梯度传播计算

4. **SfcCorridor** (`sfc_corridor.{hpp,cpp}`)
   - 方形安全走廊，移植自 TDT-nav-kit 的 `SfcSquare`
   - 积分图实现 O(log r) 查询（TDT 原版逐格扩张是 O(r²)）
   - 修复了 TDT 原版的 origin 符号不一致与 `FLT_MAX` 未定义行为

5. **KinodynamicAstar** (`kinodynamic_astar.{hpp,cpp}`)
   - 二维全向动力学可行搜索，移植自 TDT-nav-kit
   - 无状态（grid 作入参），节点数有 `max_nodes` 上限
   - 当前未接入任何运行链路：对全向底盘，恢复场景的短距低速机动用射线可达性
     已经足够，硬塞进去只增加耗时与风险。留作「需要一条动力学可行局部段」时
     的现成工具（如给 MPC 供初值、验证脱困点是否 maneuver 得过去）

### 与当前 path_smoother 的区别

| 维度 | 当前 B 样条平滑器 | MINCO 优化器 |
|---|---|---|
| **算法** | 梯度下降 + 二阶差分 | L-BFGS + 分段多项式 |
| **优化目标** | 最小化曲率（近似） | 最小化 jerk（精确） |
| **动力学** | 不考虑速度/加速度 | 考虑速度/加速度约束 |
| **时间分配** | 无（路径固定） | 距离 + 转角的固定预分配（不进入 L-BFGS） |
| **连续性** | C¹（速度连续） | C²（加速度连续） |
| **求解时间** | ~5ms | ~20-100ms |

## 使用方法

MINCO smoother 已集成到主导航流程，直接启动导航即可使用：

```bash
# 仿真环境
ros2 launch bringup sim.launch.py

# 真实环境  
ros2 launch bringup real.launch.py
```

### 参数调优

所有参数在 `params/navigation2.yaml` 中的 `rm_minco_path_smoother` 段：

```yaml
smooth_weight: 1.0        # 增大 → 轨迹更平滑，但可能偏离原路径
data_weight: 10.0         # 数据保持项，拉回原始路径
default_velocity: 1.0     # 直线段基础时间 = 距离 / 速度
min_segment_time: 0.1    # 防止极短段使 MINCO 数值退化
turn_time_weight: 0.12   # 每弧度转角增加的过渡时间（分摊到相邻两段）
```

## 调试技巧

### 1. 查看优化日志

```bash
# 设置日志级别为 DEBUG
ros2 run navigation2 rm_minco_path_smoother_node --ros-args --log-level debug
```

日志会显示：
- MINCO 优化耗时
- L-BFGS 收敛状态
- 输出路径点数量

### 2. 可视化轨迹

```bash
# 在 RViz 中添加 Path 显示
# Topic: /plan_minco
# Color: 绿色（区分原始路径）
```

### 3. 重新出走廊图（换图后）

走廊/轨迹的可视化是离线工具，换地图后重跑两条命令即可，不需要改代码 ——
隧道几何完全由 `map/<world>.msgpack` 里的 `clear_width` 驱动：

```sh
source install/setup.bash
ros2 run navigation2 sfc_corridor_dump \
  src/bringup/map/<world>.msgpack <sx> <sy> <gx> <gy> /tmp/dump.json [corridor_weight]
python3 install/navigation2/share/navigation2/tools/render_corridor.py \
  src/bringup/map/<world>.msgpack /tmp/dump.json /tmp/viz
```

产出三张图（配色对齐 TDT-nav-kit 示例图：绿=前端化简路径、蓝框=SFC 走廊、
红=MINCO 轨迹、品红=隧道本体、青线=隧道轴线）：

| 文件 | 内容 |
|---|---|
| `sfc_corridor.png` | 全图：走廊框沿路径铺开 |
| `sfc_corridor_tunnel.png` | 隧道附近放大，走廊项关/开对比 |
| `sfc_corridor_demo.png` | 受控实验：把洞内路标点人为横移 0.2 m 后再看关/开 |

RMUL 当前地图（净宽 0.5 m、车体半径 0.25 m）实测：洞内横向偏移均值从
**0.196 m 降到 0.067 m**（`tunnel_corridor_weight: 100`）。注意软惩罚压不到
「不擦壁」所需的 ±0.075 m（扫到权重 500 仍有 0.13 m），硬保证来自
`global_check` 的 SFC 校验。已出好的图在 `docs/images/sfc_corridor*.png`。

换图后的预期：走廊半宽 = `clear_width/2 - robot_radius`。净宽 0.8 m、车体半径
0.25 m 时是 **0.15 m** —— 一条真正有宽度的走廊，不再是 RMUL 那种零宽情形，
软惩罚的作用会更充分。几何全部由地图的 `clear_width` 驱动，换图不需要改代码。


### 4. 性能分析

如果优化时间过长（>100ms）：

1. **减少路径点数量**：在 global planner 中稀疏化输出
2. **降低迭代次数**：修改 `lbfgs_params_.max_iterations`（默认 4000）
3. **关闭优化**：设置 `enable_optimization: false` 作为 fallback

### 5. 常见问题

**问题：优化失败，输出原始路径**

- 原因：L-BFGS 求解器收敛失败
- 解决：
  1. 检查原始路径是否合理
  2. 增大 `data_weight` 让优化更贴近原路径

**问题：轨迹抖动**

- 原因：`smooth_weight` 太大，平滑项主导
- 解决：减小 `smooth_weight` 到 0.5 或更低

**问题：轨迹偏离原路径太多**

- 原因：`smooth_weight` 太大
- 解决：减小 `smooth_weight` 到 0.5

## 实现细节

### 时间分配策略

当前时间不作为 L-BFGS 变量，以保持优化维度、内存规模和收敛行为稳定。基础段时间为：

```cpp
double base = std::max(segment_distance / default_velocity, min_segment_time);
```

每个内部拐角再增加 `turn_time_weight * abs(turn_angle)` 秒，并按相邻段长度分摊到两侧。
直线不受影响，急弯获得更大的速度/加速度过渡时间。这吸收了 RoboWalker 2025
“路径长度 + 转角”前端时间度量的思路，但不增加节点或线程。

可选 `two_stage.enable` 提供受限二阶段：第一阶段后固定采样每段最大速度/加速度，只将
违反动力学阈值的段时间放大（带三点平滑和 `max_scale` 上限），再用较小迭代预算优化
一次原路标点。精优化的障碍 soft 梯度仅保留轨迹法向分量，避免沿切向推拉时间分配；
第二阶段失败会保留第一阶段结果。当前默认开启；objective 的固定尺寸 Eigen 工作区会按
路标数预分配并在所有 line-search evaluation 和第二阶段间复用。L-BFGS history 从旧值
256 收到 32，第一/第二阶段迭代上限分别为 800/300，避免把两个 4000 次预算相加。
`performance.*` 可记录实车耗时，必要时 `two_stage.enable: false` 一键回退。

### 轨迹采样

MINCO 输出的是分段多项式 `Piece<5, 2>`（5 阶多项式，2 维），需要采样成 ROS Path：

```cpp
for (size_t i = 0; i < pieces.size(); ++i) {
  double duration = pieces[i].getDuration();
  int num_samples = std::ceil(duration / sample_dt);
  for (int j = 0; j < num_samples; ++j) {
    double t = (j * duration) / (num_samples - 1);
    Eigen::Vector2d pos = pieces[i].getPos(t);
    // 转换为 PoseStamped
  }
}
```

## 性能基准

在测试地图上的典型性能：

| 路径长度 | 路径点数 | 优化时间 | 输出点数 |
|---------|---------|---------|---------|
| 5m      | 10      | 15ms    | 50      |
| 10m     | 20      | 35ms    | 100     |
| 20m     | 40      | 80ms    | 200     |

## 许可证

MINCO 核心算法来自 Zhepei Wang 的实现（MIT License）。
L-BFGS 库来自 ROSE 团队的移植。

## 参考文献

1. Wang, Zhepei, et al. "Geometrically Constrained Trajectory Optimization for Multicopters." (MINCO 论文)
2. ROSE 团队的 `rose_navigation` 实现
