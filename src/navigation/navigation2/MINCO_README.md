# MINCO Path Smoother

基于 ROSE 方案的 MINCO（Minimum Control Effort）轨迹优化器，用于平滑 A* 规划的路径。
优化默认开启（`enable_optimization: true`），几何平滑 + 数据保持 + 隧道轴向对齐 + 进程内距离场障碍 soft 代价。

## 功能特性

- **固定时间、形状优化**：L-BFGS 只优化内部路标点 XY，段时间不作为优化变量
- **转角感知时间初值**：距离/速度基础上为急弯两侧增加过渡时间
- **动力学连续性**：最小化 jerk（加加速度），保证轨迹平滑
- **L-BFGS 优化**：高效的非线性优化求解器

## 架构说明

### 核心组件

1. **MincoOptimizer** (`minco/minco_optimizer.cpp`)
   - MINCO 优化器封装
   - 使用 MINCO_S3NU（S=3 表示最小化 jerk，NU 表示非均匀时间）
   - L-BFGS 求解器配置

2. **RmMincoPathSmoother** (`minco_path_smoother_node.cpp`)
   - ROS 2 节点封装
   - 订阅原始路径 `/plan_raw`
   - 发布优化后路径 `/plan`

3. **MINCO 库** (`minco/minco.hpp`)
   - 来自 ROSE 的 MINCO 实现
   - 带状线性系统求解器
   - 梯度传播计算

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

### 3. 性能分析

如果优化时间过长（>100ms）：

1. **减少路径点数量**：在 global planner 中稀疏化输出
2. **降低迭代次数**：修改 `lbfgs_params_.max_iterations`（默认 4000）
3. **关闭优化**：设置 `enable_optimization: false` 作为 fallback

### 4. 常见问题

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
